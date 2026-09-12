"""Versioned, ROS-free replay data and evaluation for the depth estimators.

V1 deliberately stores only the evidence the box-gated depth paths read: raw
detection identity, its aligned-depth ROI, camera intrinsics, and the
camera-to-base transform.  It therefore cannot accidentally start Gazebo,
OWLv2, or a depth model while evaluating a parameter sweep.

Both projective ranging and euclidean reconstruction run off that evidence.
Euclidean needs one thing projective does not -- a floor reference -- and it
comes from the same optical-to-base extrinsics the projective path already
requires, so neither estimator can run on an event the other cannot.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from ridgeback_autonomy.benchmarking.report import render_run_report
from ridgeback_autonomy.benchmarking.reduction import (
    merge_status_histograms,
    summarize_capture_events,
)
from ridgeback_autonomy.benchmarking.scoring import (
    MISS_OUTCOMES,
    score_scene,
)
from ridgeback_autonomy.benchmarking.summary import (
    build_run_document,
    build_summary_rows,
    write_run_json,
    write_trial_csv,
)
from ridgeback_autonomy.benchmarking.simulation import GroundTruthInstance
from ridgeback_autonomy.benchmarking.sweep_report import write_sweep_report
from ridgeback_autonomy.benchmarking.trial_results import build_trial_result
from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.common.models import Detection
from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    PreparedDepthRegion,
    resolve_depth_gate,
    valid_depth,
)
from ridgeback_autonomy.perception.target_localization.core.box_gate import (
    box_within_frame_fraction,
)
from ridgeback_autonomy.perception.target_localization.core.euclidean_reconstruction import (
    localize_prepared_euclidean_reconstruction,
)
from ridgeback_autonomy.perception.target_localization.core.intrinsics import CameraIntrinsics
from ridgeback_autonomy.perception.target_localization.core.isolation_2d import build_isolation_2d
from ridgeback_autonomy.perception.target_localization.core.isolation_3d import (
    BASE_ABOVE_FLOOR_M_DEFAULT,
    ISOLATION_3D_DEFAULT,
    build_isolation_3d,
    camera_floor_geometry,
)
from ridgeback_autonomy.perception.target_localization.core.mask import region_from_bbox
from ridgeback_autonomy.perception.target_localization.core.projective_ranging import (
    localize_prepared_projective_ranging,
)
from ridgeback_autonomy.perception.target_localization.core.vehicle_frame import (
    ROBOT_FRONT_OFFSET_M,
    optical_to_base_planar,
)
from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    DEPTH_PATH_ESTIMATORS,
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_LABELS,
    parse_estimators,
)
from ridgeback_autonomy.perception.target_localization.measurement_pipeline import (
    set_mask_estimator_status,
)


REPLAY_SCHEMA_VERSION = 1
REPLAY_CAPTURE_BATCHES_DEFAULT = 5
MANIFEST_NAME = 'manifest.json'


@dataclass
class ReplayMeasurementEvent:
    """The ROS-free event shape consumed by the shared scoring module."""

    stamp_ns: int
    detected: bool
    count: int
    bboxes: tuple[tuple[int, int, int, int], ...]
    image_width: int
    image_height: int
    detections: list[Detection]
    estimates: dict[str, float | None]


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json_atomic(path: Path, document: dict) -> None:
    temporary = path.with_name(f'.{path.name}.partial')
    with open(temporary, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=2, sort_keys=True)
        handle.write('\n')
    os.replace(temporary, path)


def _scalar_string(array: np.ndarray) -> str:
    return str(np.asarray(array).item())


@dataclass(frozen=True)
class ReplayDataset:
    root: Path
    manifest: dict

    @property
    def trial_entries(self) -> tuple[dict, ...]:
        return tuple(self.manifest['trials'])


class ReplayDatasetWriter:
    """One compressed payload per benchmark trial plus a readable manifest."""

    def __init__(self, root: str | Path, manifest: dict) -> None:
        self.root = Path(root).expanduser().resolve()
        if self.root.exists():
            raise ValueError(f'Replay dataset directory already exists: {self.root}')
        self.root.mkdir(parents=True)
        self.trials_dir = self.root / 'trials'
        self.trials_dir.mkdir()
        self.manifest = {
            'schema_version': REPLAY_SCHEMA_VERSION,
            'state': 'capturing',
            'trials': [],
            **manifest,
        }
        _write_json_atomic(self.root / MANIFEST_NAME, self.manifest)

    def write_trial(self, trial: dict, events: list[dict]) -> None:
        trial_id = str(trial['trial_id'])
        relative_payload = f'trials/{trial_id}.npz'
        payload_path = self.root / relative_payload
        if payload_path.exists():
            raise ValueError(f'Replay dataset already contains trial "{trial_id}".')

        arrays: dict[str, np.ndarray] = {}
        payload_events: list[dict] = []
        for event_index, event in enumerate(events):
            event_metadata = {key: value for key, value in event.items() if key != 'detections'}
            detections: list[dict] = []
            for detection_index, detection in enumerate(event.get('detections', ())):
                item = {key: value for key, value in detection.items() if key != 'depth_roi'}
                depth_roi = detection.get('depth_roi')
                if depth_roi is not None:
                    key = f'depth_{event_index}_{detection_index}'
                    arrays[key] = np.asarray(depth_roi, dtype=np.float32)
                    item['depth_key'] = key
                detections.append(item)
            event_metadata['detections'] = detections
            payload_events.append(event_metadata)

        arrays['metadata_json'] = np.asarray(json.dumps({
            'schema_version': REPLAY_SCHEMA_VERSION,
            'trial': trial,
            'events': payload_events,
        }, sort_keys=True))
        temporary = payload_path.with_name(f'.{payload_path.name}.partial.npz')
        np.savez_compressed(temporary, **arrays)
        os.replace(temporary, payload_path)
        self.manifest['trials'].append({
            'trial_id': trial_id,
            'payload': relative_payload,
            'sha256': sha256_file(payload_path),
            'event_count': len(payload_events),
        })
        _write_json_atomic(self.root / MANIFEST_NAME, self.manifest)

    def finalize(self, **metadata: Any) -> None:
        skipped = int(metadata.get('trials_skipped', 0))
        included = int(metadata.get('trials_included', len(self.manifest['trials'])))
        if skipped:
            raise ValueError(
                f'Cannot finalize replay dataset with {skipped} skipped trial(s).')
        if included != len(self.manifest['trials']):
            raise ValueError(
                f'Replay dataset contains {len(self.manifest["trials"])} payload(s) '
                f'but reports {included} included trial(s).')
        self.manifest.update(metadata)
        self.manifest['state'] = 'complete'
        _write_json_atomic(self.root / MANIFEST_NAME, self.manifest)

    def mark_incomplete(self, **metadata: Any) -> None:
        """Close a partial capture without making it loadable as benchmark input."""

        self.manifest.update(metadata)
        self.manifest['state'] = 'incomplete'
        _write_json_atomic(self.root / MANIFEST_NAME, self.manifest)


def load_dataset(path: str | Path) -> ReplayDataset:
    root = Path(path).expanduser().resolve()
    manifest_path = root / MANIFEST_NAME
    try:
        with open(manifest_path, encoding='utf-8') as handle:
            manifest = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'Cannot read replay dataset manifest {manifest_path}: {exc}') from exc
    if manifest.get('schema_version') != REPLAY_SCHEMA_VERSION:
        raise ValueError(
            f'Unsupported replay dataset schema {manifest.get("schema_version")!r}; '
            f'expected {REPLAY_SCHEMA_VERSION}.')
    if manifest.get('state') != 'complete':
        raise ValueError(f'Replay dataset is not complete: {root}')
    skipped = manifest.get('trials_skipped', 0)
    if isinstance(skipped, bool) or not isinstance(skipped, int) or skipped != 0:
        raise ValueError(
            f'Replay dataset reports {skipped!r} skipped trial(s): {root}')
    trials = manifest.get('trials')
    if not isinstance(trials, list) or not trials:
        raise ValueError(f'Replay dataset {root} contains no trials.')
    included = manifest.get('trials_included', len(trials))
    if isinstance(included, bool) or not isinstance(included, int) or included != len(trials):
        raise ValueError(
            f'Replay dataset reports {included!r} included trial(s) but contains '
            f'{len(trials)} payload(s): {root}')
    for entry in trials:
        payload = root / str(entry.get('payload', ''))
        if not payload.is_file() or sha256_file(payload) != entry.get('sha256'):
            raise ValueError(f'Replay dataset payload hash mismatch: {payload}')
    return ReplayDataset(root=root, manifest=manifest)


def load_trial(dataset: ReplayDataset, entry: dict) -> tuple[dict, list[dict]]:
    path = dataset.root / entry['payload']
    try:
        with np.load(path, allow_pickle=False) as payload:
            metadata = json.loads(_scalar_string(payload['metadata_json']))
            if metadata.get('schema_version') != REPLAY_SCHEMA_VERSION:
                raise ValueError('payload schema version differs from manifest')
            events: list[dict] = []
            for event in metadata['events']:
                copied = {key: value for key, value in event.items() if key != 'detections'}
                copied_detections = []
                for detection in event['detections']:
                    copied_detection = dict(detection)
                    depth_key = copied_detection.pop('depth_key', None)
                    copied_detection['depth_roi'] = (
                        np.array(payload[depth_key], copy=True) if depth_key is not None else None)
                    copied_detections.append(copied_detection)
                copied['detections'] = copied_detections
                events.append(copied)
    except (OSError, KeyError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f'Cannot read replay payload {path}: {exc}') from exc
    return metadata['trial'], events


def _optional_float(arguments: dict[str, str], key: str) -> float | None:
    return float(arguments[key]) if key in arguments else None


def _selected_estimators(arguments: dict[str, str]) -> tuple[str, ...]:
    selected = tuple(
        estimator for estimator in parse_estimators(arguments.get('estimators'))
        if estimator in DEPTH_PATH_ESTIMATORS)
    if not selected:
        raise ValueError(
            'Replay requires projective ranging and/or euclidean reconstruction.')
    return selected


def _event_from_payload(
    event: dict,
    arguments: dict[str, str],
) -> tuple[ReplayMeasurementEvent, tuple[str, ...]]:
    selected = _selected_estimators(arguments)
    detections: list[Detection] = []
    width = int(event['image_width'])
    height = int(event['image_height'])
    intrinsics_data = event.get('intrinsics')
    rotation_data = event.get('camera_rotation')
    translation_data = event.get('camera_translation')
    raw_depth_max = event.get('depth_usable_max_m')
    depth_max = math.inf if raw_depth_max is None else float(raw_depth_max)
    gate = resolve_depth_gate(float(arguments.get('mask_depth_max_meters', '0.0')))
    effective_depth_max = min(gate, depth_max)
    intrinsics = (
        CameraIntrinsics(**{key: intrinsics_data[key] for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height')})
        if intrinsics_data is not None else None)
    rotation = np.asarray(rotation_data, dtype=np.float64) if rotation_data is not None else None
    translation = np.asarray(translation_data, dtype=np.float64) if translation_data is not None else None
    isolation = build_isolation_2d(
        arguments.get('isolation_2d', 'nearest_mode_histogram'),
        bin_width_m=_optional_float(arguments, 'isolation_2d_bin_width_m'),
        band_m=_optional_float(arguments, 'isolation_2d_band_m'),
        min_bin_fraction=_optional_float(arguments, 'isolation_2d_min_bin_fraction'),
    )
    isolation_3d = None
    if 'euclidean_reconstruction' in selected and rotation is not None and translation is not None:
        # The floor reference euclidean needs, derived from the same extrinsics
        # projective already requires. V1 carries no chassis height of its own,
        # so the offset of base_link above the floor is the shipped constant --
        # a chassis figure rather than per-event evidence, confirmed in sim at
        # 0.0259 m, which is where this dataset was captured.
        camera_height, down_optical = camera_floor_geometry(
            rotation, translation,
            float(event.get('base_above_floor_m', BASE_ABOVE_FLOOR_M_DEFAULT)))
        isolation_3d = build_isolation_3d(
            arguments.get('isolation_3d', ISOLATION_3D_DEFAULT),
            camera_height,
            down_optical,
            floor_margin_m=_optional_float(arguments, 'isolation_3d_floor_margin_m'),
            percentile=_optional_float(arguments, 'isolation_3d_percentile'),
            ahead_m=_optional_float(arguments, 'isolation_3d_ahead_m'),
            behind_m=_optional_float(arguments, 'isolation_3d_behind_m'),
            bin_width_m=_optional_float(arguments, 'isolation_3d_bin_width_m'),
            min_bin_fraction=_optional_float(arguments, 'isolation_3d_min_bin_fraction'),
        )
    min_valid_pixels = int(arguments.get('min_valid_pixels', '10'))
    for payload_detection in event['detections']:
        detection = Detection(
            bbox_xyxy=tuple(int(value) for value in payload_detection['bbox_xyxy']),
            label=str(payload_detection.get('label', '')),
            score=float(payload_detection.get('score', 0.0)),
        )
        if not box_within_frame_fraction(detection.bbox_xyxy, height, width):
            set_mask_estimator_status(detection, MissReason.MASK_OVERSIZED_BOX, selected)
        elif intrinsics is None:
            set_mask_estimator_status(detection, MissReason.NO_CAMERA_INFO, selected)
        elif (intrinsics.width, intrinsics.height) != (width, height):
            set_mask_estimator_status(detection, MissReason.GRID_MISMATCH, selected)
        elif rotation is None or translation is None:
            set_mask_estimator_status(detection, MissReason.TF_MISS_EXTRINSIC, selected)
        elif payload_detection.get('depth_roi') is None:
            set_mask_estimator_status(detection, MissReason.NO_DEPTH_FRAME, selected)
        else:
            region = region_from_bbox(detection.bbox_xyxy, height, width)
            roi_depth = np.asarray(payload_detection['depth_roi'], dtype=np.float32)
            if roi_depth.shape != region.roi_shape:
                raise ValueError(
                    f'Depth ROI shape {roi_depth.shape} does not match box region {region.roi_shape}.')
            # Euclidean reconstruction deprojects with full-grid indices against
            # the original colour intrinsics, so the stored window is placed
            # back at its own origin on an empty frame. Nothing outside the
            # window is ever addressed: every index comes from the region.
            depth_full = np.zeros((height, width), dtype=np.float32)
            depth_full[region.origin_v:region.origin_v + region.roi_shape[0],
                       region.origin_u:region.origin_u + region.roi_shape[1]] = roi_depth
            prepared = PreparedDepthRegion(
                region=region,
                depth_full=depth_full,
                roi_depth=roi_depth,
                valid_masked=region.data & valid_depth(roi_depth, effective_depth_max),
            )
            front_offset = float(event.get('front_offset_m', ROBOT_FRONT_OFFSET_M))
            if 'projective_ranging' in selected:
                result, reason = localize_prepared_projective_ranging(
                    prepared, intrinsics, isolation=isolation,
                    min_valid_pixels=min_valid_pixels)
                detection.projective_ranging_status = int(reason)
                if result is not None:
                    (
                        detection.projective_ranging_lateral_m,
                        detection.projective_ranging_forward_m,
                        detection.projective_ranging_distance_m,
                    ) = optical_to_base_planar(
                        result.xyz_optical, rotation, translation, front_offset)
            if 'euclidean_reconstruction' in selected:
                result, reason = localize_prepared_euclidean_reconstruction(
                    prepared, intrinsics, isolation=isolation_3d,
                    min_valid_points=min_valid_pixels)
                detection.euclidean_reconstruction_status = int(reason)
                if result is not None:
                    (
                        detection.euclidean_reconstruction_lateral_m,
                        detection.euclidean_reconstruction_forward_m,
                        detection.euclidean_reconstruction_distance_m,
                    ) = optical_to_base_planar(
                        result.xyz_optical, rotation, translation, front_offset)
        detections.append(detection)
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
        bboxes=tuple(tuple(item.bbox_xyxy) for item in detections),
        image_width=width,
        image_height=height,
        detections=detections,
        estimates=estimates,
    ), selected


def evaluate_trial(trial: dict, events: list[dict], arguments: dict[str, str]) -> dict:
    converted = [_event_from_payload(event, arguments) for event in events]
    selected = converted[0][1] if converted else _selected_estimators(arguments)
    measurement_events = {
        (index, item[0].stamp_ns): item[0]
        for index, item in enumerate(converted)
    }
    capture = summarize_capture_events(measurement_events, selected)
    truth = [
        GroundTruthInstance(**{key: value for key, value in item.items() if key != 'spawn_yaw_rad'})
        for item in trial['ground_truth']
    ]
    scene = SimpleNamespace(
        id=trial['scene_id'],
        robots=tuple(SimpleNamespace(yaw=float(item['spawn_yaw_rad'])) for item in trial['ground_truth']),
    )
    scene_score = score_scene(
        capture['usable_by_estimator'], truth, selected,
        detector_fired=capture['any_detected'])
    result = build_trial_result(
        trial,
        scene,
        truth,
        scene_score,
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


def _evaluate_entry(task: tuple[str, dict, dict, tuple[tuple[str, dict[str, str]], ...]]) -> dict[str, dict]:
    """Worker boundary: load one payload, then keep its ROIs hot for all variants."""

    root, manifest, entry, variants = task
    dataset = ReplayDataset(root=Path(root), manifest=manifest)
    trial, events = load_trial(dataset, entry)
    return {
        name: evaluate_trial(trial, events, arguments)
        for name, arguments in variants
    }


def evaluate_dataset(
    dataset: ReplayDataset,
    variants: tuple[Any, ...],
    *,
    workers: int = 1,
) -> dict[str, dict]:
    """Evaluate every variant over every trial, in deterministic input order."""

    collected = {}
    for variant in variants:
        selected = _selected_estimators(variant.arguments)
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
    encoded_variants = tuple((variant.name, variant.arguments) for variant in variants)
    tasks = [
        (str(dataset.root), dataset.manifest, entry, encoded_variants)
        for entry in dataset.trial_entries
    ]
    if workers == 1:
        per_trial = [_evaluate_entry(task) for task in tasks]
    else:
        # ``map`` preserves task order.  This is the determinism boundary: all
        # reporting then sees the scenario/repeat order regardless of scheduling.
        with multiprocessing.get_context('spawn').Pool(processes=workers) as pool:
            per_trial = pool.map(_evaluate_entry, tasks)
    for trial_results in per_trial:
        for variant in variants:
            result = trial_results[variant.name]
            target = collected[variant.name]
            target['trial_count'] += 1
            for estimator in target['selected_estimators']:
                target['rows'][estimator].extend(result['rows'][estimator])
                for outcome, count in result['outcome_counts'][estimator].items():
                    target['outcome_counts'][estimator][outcome] += count
            merge_status_histograms(target['status_histogram'], result['status_histogram'])
            target['extra_count'] += result['extra_count']
    return collected


def write_replay_results(
    output_root: str | Path,
    dataset: ReplayDataset,
    variants: tuple[Any, ...],
    results: dict[str, dict],
    *,
    evaluation_provenance: dict,
    worker_count: int,
    evaluation_wall_time_sec: float,
    sweep_name: str,
    sweep_description: str,
) -> None:
    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=False)
    baseline_name = variants[0].name
    baseline_rows = {
        estimator: {(row['trial_id'], row['instance_index']): row for row in rows}
        for estimator, rows in results[baseline_name]['rows'].items()
    }
    for variant in variants:
        result = results[variant.name]
        variant_root = root / variant.name
        variant_root.mkdir()
        display_names = {
            estimator: ESTIMATOR_LABELS[estimator]
            for estimator in result['selected_estimators']
        }
        for estimator, rows in result['rows'].items():
            write_trial_csv(str(variant_root / f'{estimator}.csv'), rows)
        summary_rows = build_summary_rows(
            result['rows'], result['extra_count'], result['outcome_counts'], result['status_histogram'])
        # A variant that selects an estimator the baseline did not has nothing
        # to be paired against, so those rows carry no delta rather than a
        # comparison with a row measured by a different estimator.
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
                        else row['trial_estimate_m'] - reference[key]['trial_estimate_m']),
                }
                for row in rows
                if (key := (row['trial_id'], row['instance_index'])) in reference
            ]
        document = build_run_document(
            summary_rows,
            run_metadata={
                'label': variant.name,
                'mode': 'offline_replay',
                'trials_included': result['trial_count'],
                'trials_skipped': 0,
            },
            parameters=variant.arguments,
            display_names=display_names,
            status_histograms=result['status_histogram'],
        )
        document['replay'] = {
            'dataset_id': dataset.manifest.get('dataset_id'),
            'dataset_content_hash': sha256_file(dataset.root / MANIFEST_NAME),
            'schema_version': REPLAY_SCHEMA_VERSION,
            'evaluation': evaluation_provenance,
            'evaluation_wall_time_sec': evaluation_wall_time_sec,
            'worker_count': worker_count,
            'execution_mode': 'offline',
            'paired_against': baseline_name,
            'paired_trial_differences': paired,
        }
        write_run_json(str(variant_root / 'run.json'), document)
        rows_flat = [row for rows in result['rows'].values() for row in rows]
        scenes = len({row['scene_id'] for row in rows_flat})
        instances = len({(row['trial_id'], row['instance_index']) for row in rows_flat})
        report = render_run_report(
            run_label=variant.name,
            scenario_path=str(dataset.manifest.get('scenario_path', 'frozen replay dataset')),
            summary_rows=summary_rows,
            estimator_rows=result['rows'],
            status_histograms=result['status_histogram'],
            display_names=display_names,
            included_trials=result['trial_count'],
            skipped_trials=0,
            scenes=scenes,
            instances=instances,
            metadata={
                'Mode': 'offline replay',
                'Dataset': str(dataset.manifest.get('dataset_id', 'unknown')),
                'Scenario': str(dataset.manifest.get('scenario_path', 'frozen inputs')),
            },
            parameters=variant.arguments,
        )
        (variant_root / 'summary.md').write_text(report, encoding='utf-8')
    dataset_hash = sha256_file(dataset.root / MANIFEST_NAME)
    replay_document = {
        'dataset': str(dataset.root),
        'dataset_id': dataset.manifest.get('dataset_id'),
        'dataset_content_hash': dataset_hash,
        'schema_version': REPLAY_SCHEMA_VERSION,
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
