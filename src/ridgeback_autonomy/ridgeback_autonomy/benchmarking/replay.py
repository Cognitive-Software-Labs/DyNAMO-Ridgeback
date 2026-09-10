"""Versioned, ROS-free replay data and evaluation for projective ranging.

V1 deliberately stores only the evidence the box-gated projective path reads:
raw detection identity, its aligned-depth ROI, camera intrinsics, and the
camera-to-base transform.  It therefore cannot accidentally start Gazebo,
OWLv2, or a depth model while evaluating a parameter sweep.
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
from ridgeback_autonomy.perception.target_localization.core.intrinsics import CameraIntrinsics
from ridgeback_autonomy.perception.target_localization.core.isolation_2d import build_isolation_2d
from ridgeback_autonomy.perception.target_localization.core.mask import region_from_bbox
from ridgeback_autonomy.perception.target_localization.core.projective_ranging import (
    localize_prepared_projective_ranging,
)
from ridgeback_autonomy.perception.target_localization.core.vehicle_frame import (
    ROBOT_FRONT_OFFSET_M,
    optical_to_base_planar,
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


def _event_from_payload(event: dict, arguments: dict[str, str]) -> ReplayMeasurementEvent:
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
        bin_width_m=float(arguments['isolation_2d_bin_width_m'])
        if 'isolation_2d_bin_width_m' in arguments else None,
        band_m=float(arguments['isolation_2d_band_m'])
        if 'isolation_2d_band_m' in arguments else None,
        min_bin_fraction=float(arguments['isolation_2d_min_bin_fraction'])
        if 'isolation_2d_min_bin_fraction' in arguments else None,
    )
    min_valid_pixels = int(arguments.get('min_valid_pixels', '10'))
    for payload_detection in event['detections']:
        detection = Detection(
            bbox_xyxy=tuple(int(value) for value in payload_detection['bbox_xyxy']),
            label=str(payload_detection.get('label', '')),
            score=float(payload_detection.get('score', 0.0)),
        )
        if not box_within_frame_fraction(detection.bbox_xyxy, height, width):
            detection.projective_ranging_status = int(MissReason.MASK_OVERSIZED_BOX)
        elif intrinsics is None:
            detection.projective_ranging_status = int(MissReason.NO_CAMERA_INFO)
        elif (intrinsics.width, intrinsics.height) != (width, height):
            detection.projective_ranging_status = int(MissReason.GRID_MISMATCH)
        elif rotation is None or translation is None:
            detection.projective_ranging_status = int(MissReason.TF_MISS_EXTRINSIC)
        elif payload_detection.get('depth_roi') is None:
            detection.projective_ranging_status = int(MissReason.NO_DEPTH_FRAME)
        else:
            region = region_from_bbox(detection.bbox_xyxy, height, width)
            roi_depth = np.asarray(payload_detection['depth_roi'], dtype=np.float32)
            if roi_depth.shape != region.roi_shape:
                raise ValueError(
                    f'Depth ROI shape {roi_depth.shape} does not match box region {region.roi_shape}.')
            prepared = PreparedDepthRegion(
                region=region,
                depth_full=roi_depth,
                roi_depth=roi_depth,
                valid_masked=region.data & valid_depth(roi_depth, effective_depth_max),
            )
            result, reason = localize_prepared_projective_ranging(
                prepared, intrinsics, isolation=isolation, min_valid_pixels=min_valid_pixels)
            detection.projective_ranging_status = int(reason)
            if result is not None:
                lateral, forward, distance = optical_to_base_planar(
                    result.xyz_optical, rotation, translation,
                    float(event.get('front_offset_m', ROBOT_FRONT_OFFSET_M)))
                detection.projective_ranging_lateral_m = lateral
                detection.projective_ranging_forward_m = forward
                detection.projective_ranging_distance_m = distance
        detections.append(detection)
    estimates = [item.projective_ranging_distance_m for item in detections]
    first_estimate = next((value for value in estimates if value is not None), None)
    return ReplayMeasurementEvent(
        stamp_ns=int(event['stamp_ns']),
        detected=bool(event['detected']),
        count=int(event['count']),
        bboxes=tuple(tuple(item.bbox_xyxy) for item in detections),
        image_width=width,
        image_height=height,
        detections=detections,
        estimates={'projective_ranging': first_estimate},
    )


def evaluate_trial(trial: dict, events: list[dict], arguments: dict[str, str]) -> dict:
    selected = ('projective_ranging',)
    measurement_events = {
        (index, int(event['stamp_ns'])): _event_from_payload(event, arguments)
        for index, event in enumerate(events)
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
        {'projective_ranging': 'Projective Ranging'},
        capture['usable_by_estimator'],
        '',
        capture['total_events'],
        captured_events=tuple(measurement_events.values()),
    )
    result['status_histogram'] = capture['status_histogram']
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

    collected = {
        variant.name: {
            'rows': {'projective_ranging': []},
            'outcome_counts': {'projective_ranging': Counter({outcome: 0 for outcome in MISS_OUTCOMES})},
            'status_histogram': {},
            'extra_count': 0,
            'trial_count': 0,
        }
        for variant in variants
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
            target['rows']['projective_ranging'].extend(result['rows']['projective_ranging'])
            for outcome, count in result['outcome_counts']['projective_ranging'].items():
                target['outcome_counts']['projective_ranging'][outcome] += count
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
        (row['trial_id'], row['instance_index']): row
        for row in results[baseline_name]['rows']['projective_ranging']
    }
    for variant in variants:
        result = results[variant.name]
        variant_root = root / variant.name
        variant_root.mkdir()
        rows = result['rows']['projective_ranging']
        csv_path = variant_root / 'projective_ranging.csv'
        write_trial_csv(str(csv_path), rows)
        summary_rows = build_summary_rows(
            result['rows'], result['extra_count'], result['outcome_counts'], result['status_histogram'])
        paired = []
        for row in rows:
            baseline = baseline_rows[(row['trial_id'], row['instance_index'])]
            paired.append({
                'trial_id': row['trial_id'],
                'instance_index': row['instance_index'],
                'baseline_outcome': baseline['outcome'],
                'outcome': row['outcome'],
                'estimate_delta_m': (
                    None if row['trial_estimate_m'] is None or baseline['trial_estimate_m'] is None
                    else row['trial_estimate_m'] - baseline['trial_estimate_m']),
            })
        document = build_run_document(
            summary_rows,
            run_metadata={
                'label': variant.name,
                'mode': 'offline_replay',
                'trials_included': result['trial_count'],
                'trials_skipped': 0,
            },
            parameters=variant.arguments,
            display_names={'projective_ranging': 'Projective Ranging'},
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
        scenes = len({row['scene_id'] for row in rows})
        instances = len({(row['trial_id'], row['instance_index']) for row in rows})
        report = render_run_report(
            run_label=variant.name,
            scenario_path=str(dataset.manifest.get('scenario_path', 'frozen replay dataset')),
            summary_rows=summary_rows,
            estimator_rows=result['rows'],
            status_histograms=result['status_histogram'],
            display_names={'projective_ranging': 'Projective Ranging'},
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
