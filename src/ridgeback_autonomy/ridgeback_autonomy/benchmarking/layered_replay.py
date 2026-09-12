"""Generalized measurement replay over sensor captures and mask caches."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
import multiprocessing
from pathlib import Path
import re
from types import SimpleNamespace
from typing import Any

import numpy as np

from ridgeback_autonomy.benchmarking.reduction import (
    merge_status_histograms,
    summarize_capture_events,
)
from ridgeback_autonomy.benchmarking.replay import ReplayMeasurementEvent
from ridgeback_autonomy.benchmarking.replay_artifacts import (
    MASK_STATUS_EMPTY_SEGMENTATION,
    MASK_STATUS_NO_COLOR_FRAME,
    MASK_STATUS_OVERSIZED_BOX,
    MASK_CACHE_KIND,
    ReplayArtifact,
    load_mask_trial,
    load_sensor_trial,
    trial_entry_map,
    validate_cache_alignment,
)
from ridgeback_autonomy.benchmarking.report import render_run_report
from ridgeback_autonomy.benchmarking.scoring import MISS_OUTCOMES, score_scene
from ridgeback_autonomy.benchmarking.summary import (
    build_run_document,
    build_summary_rows,
    write_run_json,
    write_trial_csv,
)
from ridgeback_autonomy.benchmarking.sweep_report import write_sweep_report
from ridgeback_autonomy.benchmarking.trial_results import build_trial_result
from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.common.models import Detection, DetectionBatch
from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    resolve_depth_gate,
)
from ridgeback_autonomy.perception.target_localization.core.intrinsics import CameraIntrinsics
from ridgeback_autonomy.perception.target_localization.core.isolation_2d import build_isolation_2d
from ridgeback_autonomy.perception.target_localization.core.isolation_3d import (
    BASE_ABOVE_FLOOR_M_DEFAULT,
    build_isolation_3d,
    camera_floor_geometry,
)
from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    DEPTH_PATH_ESTIMATORS,
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_LABELS,
    parse_estimators,
)
from ridgeback_autonomy.perception.target_localization.measurement_pipeline import (
    fill_path_measurements,
    set_mask_estimator_status,
)
from ridgeback_autonomy.benchmarking.simulation import GroundTruthInstance


@dataclass(frozen=True)
class LayeredVariant:
    name: str
    cache_name: str
    cache_artifact_id: str
    arguments: dict[str, str]


def _safe_name(value: str) -> str:
    safe = re.sub(r'[^A-Za-z0-9._-]+', '_', value).strip('._-')
    return safe or 'mask_cache'


def named_caches(caches: tuple[ReplayArtifact, ...]) -> tuple[tuple[str, ReplayArtifact], ...]:
    result = []
    seen: set[str] = set()
    for cache in caches:
        if cache.kind != MASK_CACHE_KIND:
            raise ValueError(f'Expected mask-cache artifact, got {cache.kind!r}.')
        configured = cache.manifest.get('metadata', {}).get('name')
        name = _safe_name(str(configured or cache.root.name))
        if name in seen:
            raise ValueError(f'Duplicate mask cache name "{name}".')
        seen.add(name)
        result.append((name, cache))
    return tuple(result)


def expand_variants(
    caches: tuple[ReplayArtifact, ...],
    variants: tuple[Any, ...],
) -> tuple[LayeredVariant, ...]:
    named = named_caches(caches)
    multiple = len(named) > 1
    return tuple(
        LayeredVariant(
            name=f'{cache_name}__{variant.name}' if multiple else variant.name,
            cache_name=cache_name,
            cache_artifact_id=cache.id,
            arguments=dict(variant.arguments),
        )
        for cache_name, cache in named
        for variant in variants
    )


def resolve_expanded_baseline(
    expanded: tuple[LayeredVariant, ...],
    requested: str,
) -> str:
    exact = [variant.name for variant in expanded if variant.name == requested]
    if exact:
        return exact[0]
    if ':' in requested:
        cache_name, variant_name = requested.split(':', 1)
        candidate = f'{_safe_name(cache_name)}__{variant_name}'
        if any(variant.name == candidate for variant in expanded):
            return candidate
    matches = [
        variant.name for variant in expanded
        if variant.name == requested or variant.name.endswith(f'__{requested}')
    ]
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise ValueError(
            f'Baseline "{requested}" is ambiguous across mask caches; use cache:variant.')
    raise ValueError(f'Unknown expanded replay baseline "{requested}".')


def _optional_float(arguments: dict[str, str], key: str) -> float | None:
    return float(arguments[key]) if key in arguments else None


def _measurement_event(
    event: dict,
    outcomes,
    arguments: dict[str, str],
) -> tuple[ReplayMeasurementEvent, tuple[str, ...]]:
    selected = tuple(
        estimator for estimator in parse_estimators(arguments.get('estimators'))
        if estimator in DEPTH_PATH_ESTIMATORS)
    if not selected:
        raise ValueError('Layered replay requires projective and/or euclidean estimation.')
    width, height = int(event['image_width']), int(event['image_height'])
    if len(outcomes) != len(event['detections']):
        raise ValueError('Mask-cache detection count differs from its sensor event.')
    detections = [
        Detection(
            bbox_xyxy=tuple(int(value) for value in item['bbox_xyxy']),
            label=str(item.get('label', '')),
            score=float(item.get('score', 0.0)),
        )
        for item in event['detections']
    ]
    batch = DetectionBatch(width, height, detections)
    masks = []
    for detection, outcome in zip(detections, outcomes):
        masks.append(outcome.region)
        reason = {
            MASK_STATUS_OVERSIZED_BOX: MissReason.MASK_OVERSIZED_BOX,
            MASK_STATUS_EMPTY_SEGMENTATION: MissReason.MASK_EMPTY_SEGMENTATION,
            MASK_STATUS_NO_COLOR_FRAME: MissReason.NO_COLOR_FRAME,
        }.get(outcome.status)
        if reason is not None:
            set_mask_estimator_status(detection, reason, selected)

    intrinsics_data = event.get('intrinsics')
    rotation_data = event.get('camera_rotation')
    translation_data = event.get('camera_translation')
    intrinsics = None
    if intrinsics_data is not None:
        intrinsics = CameraIntrinsics(**{
            key: intrinsics_data[key]
            for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height')
        })
    if intrinsics is None:
        for detection in detections:
            set_mask_estimator_status(detection, MissReason.NO_CAMERA_INFO, selected)
    elif (intrinsics.width, intrinsics.height) != (width, height):
        for detection in detections:
            set_mask_estimator_status(detection, MissReason.GRID_MISMATCH, selected)
    elif rotation_data is None or translation_data is None:
        for detection in detections:
            set_mask_estimator_status(detection, MissReason.TF_MISS_EXTRINSIC, selected)
    else:
        rotation = np.asarray(rotation_data, dtype=np.float64)
        translation = np.asarray(translation_data, dtype=np.float64)
        isolation_2d = build_isolation_2d(
            arguments.get('isolation_2d', 'nearest_mode_histogram'),
            bin_width_m=_optional_float(arguments, 'isolation_2d_bin_width_m'),
            band_m=_optional_float(arguments, 'isolation_2d_band_m'),
            min_bin_fraction=_optional_float(arguments, 'isolation_2d_min_bin_fraction'),
        )
        camera_height, down_optical = camera_floor_geometry(
            rotation,
            translation,
            float(event.get('base_above_floor_m', BASE_ABOVE_FLOOR_M_DEFAULT)),
        )
        isolation_3d = build_isolation_3d(
            arguments.get('isolation_3d', 'height_crop_nearest_mode_band'),
            camera_height,
            down_optical,
            floor_margin_m=_optional_float(arguments, 'isolation_3d_floor_margin_m'),
            percentile=_optional_float(arguments, 'isolation_3d_percentile'),
            ahead_m=_optional_float(arguments, 'isolation_3d_ahead_m'),
            behind_m=_optional_float(arguments, 'isolation_3d_behind_m'),
            bin_width_m=_optional_float(arguments, 'isolation_3d_bin_width_m'),
            min_bin_fraction=_optional_float(arguments, 'isolation_3d_min_bin_fraction'),
        )
        raw_depth_max = event.get('depth_usable_max_m')
        source_max = math.inf if raw_depth_max is None else float(raw_depth_max)
        depth_max = min(
            resolve_depth_gate(float(arguments.get('mask_depth_max_meters', '0.0'))),
            source_max,
        )
        fill_path_measurements(
            batch,
            masks,
            intrinsics,
            event.get('depth_m'),
            None,
            camera_rotation=rotation,
            camera_translation=translation,
            front_offset_m=float(event.get('front_offset_m', 0.25)),
            isolation_2d=isolation_2d,
            isolation_3d=isolation_3d,
            depth_max=depth_max,
            min_valid_pixels=int(arguments.get('min_valid_pixels', '10')),
            enabled=frozenset(selected),
        )

    estimates = {
        estimator: next((
            getattr(detection, ESTIMATOR_FIELD_KEYS[estimator])
            for detection in detections
            if getattr(detection, ESTIMATOR_FIELD_KEYS[estimator]) is not None
        ), None)
        for estimator in selected
    }
    return ReplayMeasurementEvent(
        stamp_ns=int(event['stamp_ns']),
        detected=bool(event['detected']),
        count=int(event['count']),
        bboxes=tuple(tuple(detection.bbox_xyxy) for detection in detections),
        image_width=width,
        image_height=height,
        detections=detections,
        estimates=estimates,
    ), selected


def evaluate_sensor_trial(
    trial: dict,
    events: list[dict],
    mask_events,
    arguments: dict[str, str],
) -> dict:
    if len(events) != len(mask_events):
        raise ValueError('Mask-cache event count differs from its sensor trial.')
    converted = [
        _measurement_event(event, outcomes, arguments)
        for event, outcomes in zip(events, mask_events)
    ]
    selected = converted[0][1] if converted else tuple(
        estimator for estimator in parse_estimators(arguments.get('estimators'))
        if estimator in DEPTH_PATH_ESTIMATORS)
    if any(item[1] != selected for item in converted):
        raise AssertionError('Estimator selection changed within one trial.')
    measurement_events = {
        (index, item[0].stamp_ns): item[0]
        for index, item in enumerate(converted)
    }
    capture = summarize_capture_events(measurement_events, selected)
    truth = [
        GroundTruthInstance(**{
            key: value for key, value in item.items() if key != 'spawn_yaw_rad'
        })
        for item in trial['ground_truth']
    ]
    scene = SimpleNamespace(
        id=trial['scene_id'],
        robots=tuple(
            SimpleNamespace(yaw=float(item['spawn_yaw_rad']))
            for item in trial['ground_truth']),
    )
    score = score_scene(
        capture['usable_by_estimator'],
        truth,
        selected,
        detector_fired=capture['any_detected'],
    )
    result = build_trial_result(
        trial,
        scene,
        truth,
        score,
        selected,
        {estimator: ESTIMATOR_LABELS[estimator] for estimator in selected},
        capture['usable_by_estimator'],
        '',
        capture['total_events'],
        captured_events=tuple(measurement_events.values()),
    )
    result['status_histogram'] = capture['status_histogram']
    result['selected_estimators'] = selected
    return result


def _evaluate_task(task) -> dict[str, dict]:
    (
        sensor_root,
        sensor_manifest,
        sensor_hash,
        sensor_entry,
        cache_payloads,
        variants,
    ) = task
    sensor = ReplayArtifact(Path(sensor_root), sensor_manifest, sensor_hash)
    trial, events = load_sensor_trial(sensor, sensor_entry)
    result = {}
    for cache_name, cache_root, cache_manifest, cache_hash, cache_entry in cache_payloads:
        cache = ReplayArtifact(Path(cache_root), cache_manifest, cache_hash)
        mask_events = load_mask_trial(cache, cache_entry)
        for variant_name, arguments in variants:
            expanded_name = (
                variant_name if len(cache_payloads) == 1
                else f'{cache_name}__{variant_name}')
            result[expanded_name] = evaluate_sensor_trial(
                trial, events, mask_events, arguments)
    return result


def evaluate_sensor_capture(
    sensor: ReplayArtifact,
    caches: tuple[ReplayArtifact, ...],
    variants: tuple[Any, ...],
    *,
    workers: int = 1,
) -> tuple[tuple[LayeredVariant, ...], dict[str, dict]]:
    if workers < 1:
        raise ValueError('Measurement workers must be at least 1.')
    if not caches:
        raise ValueError('Layered replay requires at least one mask cache.')
    validate_cache_alignment(sensor, caches)
    named = named_caches(caches)
    expanded = expand_variants(caches, variants)
    collected: dict[str, dict] = {}
    for variant in expanded:
        selected = tuple(
            estimator for estimator in parse_estimators(variant.arguments.get('estimators'))
            if estimator in DEPTH_PATH_ESTIMATORS)
        collected[variant.name] = {
            'rows': {estimator: [] for estimator in selected},
            'outcome_counts': {
                estimator: Counter({outcome: 0 for outcome in MISS_OUTCOMES})
                for estimator in selected
            },
            'status_histogram': {},
            'extra_count': 0,
            'trial_count': 0,
            'selected_estimators': selected,
        }
    sensor_entries = trial_entry_map(sensor)
    cache_maps = {name: trial_entry_map(cache) for name, cache in named}
    tasks = []
    for trial_id, sensor_entry in sensor_entries.items():
        cache_payloads = tuple(
            (
                name,
                str(cache.root),
                cache.manifest,
                cache.manifest_sha256,
                cache_maps[name][trial_id],
            )
            for name, cache in named
        )
        tasks.append((
            str(sensor.root), sensor.manifest, sensor.manifest_sha256,
            sensor_entry, cache_payloads,
            tuple((variant.name, variant.arguments) for variant in variants),
        ))
    if workers == 1:
        per_trial = [_evaluate_task(task) for task in tasks]
    else:
        with multiprocessing.get_context('spawn').Pool(processes=workers) as pool:
            per_trial = pool.map(_evaluate_task, tasks)
    for trial_result in per_trial:
        for variant in expanded:
            result = trial_result[variant.name]
            target = collected[variant.name]
            target['trial_count'] += 1
            for estimator in target['selected_estimators']:
                target['rows'][estimator].extend(result['rows'][estimator])
                for outcome, count in result['outcome_counts'][estimator].items():
                    target['outcome_counts'][estimator][outcome] += count
            merge_status_histograms(target['status_histogram'], result['status_histogram'])
            target['extra_count'] += result['extra_count']
    return expanded, collected


def write_layered_results(
    output_root: str | Path,
    sensor: ReplayArtifact,
    caches: tuple[ReplayArtifact, ...],
    variants: tuple[LayeredVariant, ...],
    results: dict[str, dict],
    *,
    baseline_name: str,
    profile: dict,
    evaluation_provenance: dict,
    worker_count: int,
    evaluation_wall_time_sec: float,
    sweep_name: str,
    sweep_description: str,
) -> None:
    from ridgeback_autonomy.benchmarking.replay import _write_json_atomic

    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=False)
    baseline_result = results[baseline_name]
    baseline_rows = {
        estimator: {
            (row['trial_id'], row['instance_index']): row
            for row in rows
        }
        for estimator, rows in baseline_result['rows'].items()
    }
    for variant in variants:
        result = results[variant.name]
        variant_root = root / variant.name
        variant_root.mkdir()
        for estimator, rows in result['rows'].items():
            write_trial_csv(str(variant_root / f'{estimator}.csv'), rows)
        summary_rows = build_summary_rows(
            result['rows'], result['extra_count'], result['outcome_counts'],
            result['status_histogram'])
        document = build_run_document(
            summary_rows,
            run_metadata={
                'label': variant.name,
                'mode': 'offline_replay',
                'profile': profile['id'],
                'trials_included': result['trial_count'],
                'trials_skipped': 0,
            },
            parameters=variant.arguments,
            display_names={
                estimator: ESTIMATOR_LABELS[estimator]
                for estimator in result['selected_estimators']
            },
            status_histograms=result['status_histogram'],
        )
        paired = {}
        for estimator, rows in result['rows'].items():
            reference = baseline_rows.get(estimator, {})
            paired[estimator] = [
                {
                    'trial_id': row['trial_id'],
                    'instance_index': row['instance_index'],
                    'baseline_outcome': reference[key]['outcome'],
                    'outcome': row['outcome'],
                    'estimate_delta_m': (
                        None
                        if row['trial_estimate_m'] is None
                        or reference[key]['trial_estimate_m'] is None
                        else row['trial_estimate_m'] - reference[key]['trial_estimate_m']
                    ),
                }
                for row in rows
                if (key := (row['trial_id'], row['instance_index'])) in reference
            ]
        document['replay'] = {
            'profile': profile,
            'sensor_capture': {
                'artifact_id': sensor.id,
                'manifest_sha256': sensor.manifest_sha256,
            },
            'mask_cache': {
                'name': variant.cache_name,
                'artifact_id': variant.cache_artifact_id,
            },
            'evaluation': evaluation_provenance,
            'evaluation_wall_time_sec': evaluation_wall_time_sec,
            'worker_count': worker_count,
            'execution_mode': 'offline',
            'paired_against': baseline_name,
            'paired_trial_differences': paired,
        }
        write_run_json(str(variant_root / 'run.json'), document)
        rows_flat = [row for rows in result['rows'].values() for row in rows]
        report = render_run_report(
            run_label=variant.name,
            scenario_path=str(sensor.manifest.get('metadata', {}).get(
                'scenario_path', 'frozen sensor capture')),
            summary_rows=summary_rows,
            estimator_rows=result['rows'],
            status_histograms=result['status_histogram'],
            display_names={
                estimator: ESTIMATOR_LABELS[estimator]
                for estimator in result['selected_estimators']
            },
            included_trials=result['trial_count'],
            skipped_trials=0,
            scenes=len({row['scene_id'] for row in rows_flat}),
            instances=len({
                (row['trial_id'], row['instance_index']) for row in rows_flat}),
            metadata={
                'Mode': f'offline replay ({profile["id"]})',
                'Sensor capture': sensor.id,
                'Mask cache': variant.cache_name,
                'Claims': ', '.join(profile['supported_claims']),
                'Limitations': ' '.join(profile['limitations']),
            },
            parameters=variant.arguments,
        )
        (variant_root / 'summary.md').write_text(report, encoding='utf-8')

    replay_document = {
        'profile': profile,
        'sensor_capture': {
            'path': str(sensor.root),
            'artifact_id': sensor.id,
            'manifest_sha256': sensor.manifest_sha256,
        },
        'mask_caches': [
            {
                'path': str(cache.root),
                'artifact_id': cache.id,
                'manifest_sha256': cache.manifest_sha256,
            }
            for cache in caches
        ],
        'variants': [variant.name for variant in variants],
        'baseline': baseline_name,
        'worker_count': worker_count,
        'evaluation_wall_time_sec': evaluation_wall_time_sec,
        'evaluation': evaluation_provenance,
    }
    _write_json_atomic(root / 'replay.json', replay_document)
    sweep_manifest = {
        'version': 1,
        'sweep': {
            'name': sweep_name,
            'description': sweep_description,
            'source': evaluation_provenance.get('sweep'),
            'source_sha256': evaluation_provenance.get('sweep_sha256'),
            'status': 'complete',
            'started': evaluation_provenance.get('started'),
            'finished': evaluation_provenance.get('finished'),
            'wall_time_sec': evaluation_wall_time_sec,
        },
        'provenance': {
            key: evaluation_provenance.get(key)
            for key in ('commit', 'branch', 'dirty_count')
        },
        'selected_configs': [variant.name for variant in variants],
        'configs': [
            {
                'name': variant.name,
                'status': 'success',
                'output_path': variant.name,
                'arguments': variant.arguments,
            }
            for variant in variants
        ],
        'replay': replay_document,
    }
    _write_json_atomic(root / 'sweep.json', sweep_manifest)
    write_sweep_report(str(root), sweep_manifest)
