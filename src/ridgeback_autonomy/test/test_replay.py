from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from ridgeback_autonomy.benchmarking.replay import (
    ReplayDatasetWriter,
    evaluate_dataset,
    load_dataset,
)


def _trial() -> dict:
    return {
        'trial_id': 'single_target',
        'repeat_index': 1,
        'scene_id': 'single_target',
        'ground_truth': [{
            'index': 0,
            'model_name': 'target',
            'world_x': 2.0,
            'world_y': 0.0,
            'forward_m': 1.75,
            'lateral_m': 0.0,
            'distance_m': 1.75,
            'spawn_yaw_rad': 0.0,
        }],
    }


def _event(*, depth_roi=None, detected=True) -> dict:
    return {
        'stamp_ns': 123,
        'frame_id': 'camera',
        'detected': detected,
        'count': 1 if detected else 0,
        'image_width': 6,
        'image_height': 6,
        'intrinsics': {
            'fx': 1.0, 'fy': 1.0, 'cx': 2.5, 'cy': 2.5, 'width': 6, 'height': 6,
        },
        'camera_rotation': [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        'camera_translation': [0.0, 0.0, 0.0],
        'front_offset_m': 0.25,
        'depth_usable_max_m': None,
        'detections': ([] if not detected else [{
            'bbox_xyxy': [1, 1, 5, 5],
            'label': 'target',
            'score': 0.9,
            'depth_roi': depth_roi,
        }]),
    }


def _variant(name='baseline', **arguments):
    return SimpleNamespace(name=name, arguments={
        'estimators': 'projective_ranging',
        'mask_gate': 'box',
        'depth_source': 'stereoscopic',
        'isolation_2d': 'nearest_mode_histogram',
        **arguments,
    })


def _dataset(tmp_path: Path, events: list[dict]):
    writer = ReplayDatasetWriter(tmp_path / 'dataset', {
        'dataset_id': 'fixture',
        'scenario_path': '/frozen/scenario.yaml',
    })
    writer.write_trial(_trial(), events)
    writer.finalize()
    return load_dataset(tmp_path / 'dataset')


def test_replay_uses_box_roi_and_same_projective_result(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_event(depth_roi=np.full((4, 4), 2.0, dtype=np.float32))])

    result = evaluate_dataset(dataset, (_variant(),))['baseline']

    row = result['rows']['projective_ranging'][0]
    assert row['outcome'] == 'scored'
    assert row['trial_estimate_m'] == pytest.approx(1.75)
    assert result['status_histogram']['projective_ranging'] == {0: 1}


def test_empty_detector_batch_is_a_detector_miss(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_event(detected=False)])

    row = evaluate_dataset(dataset, (_variant(),))['baseline']['rows']['projective_ranging'][0]

    assert row['outcome'] == 'detector_miss'


def test_missing_depth_is_explicit_no_value_reason(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_event(depth_roi=None)])

    row = evaluate_dataset(dataset, (_variant(),))['baseline']['rows']['projective_ranging'][0]

    assert row['outcome'] == 'no_value'
    assert row['miss_reason'] == 'NO_DEPTH_FRAME'


def test_intrinsics_grid_mismatch_preserves_the_live_miss_reason(tmp_path: Path) -> None:
    event = _event(depth_roi=np.full((4, 4), 2.0, dtype=np.float32))
    event['intrinsics']['width'] = 5
    dataset = _dataset(tmp_path, [event])

    row = evaluate_dataset(dataset, (_variant(),))['baseline']['rows']['projective_ranging'][0]

    assert row['outcome'] == 'no_value'
    assert row['miss_reason'] == 'GRID_MISMATCH'


def test_replay_parallel_output_keeps_order_and_values(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_event(depth_roi=np.full((4, 4), 2.0, dtype=np.float32))])
    variants = (_variant('baseline'), _variant('wide', isolation_2d_band_m='0.75'))

    sequential = evaluate_dataset(dataset, variants, workers=1)
    parallel = evaluate_dataset(dataset, variants, workers=2)

    assert parallel == sequential


def test_dataset_hash_mismatch_is_rejected(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_event(depth_roi=np.full((4, 4), 2.0, dtype=np.float32))])
    payload = dataset.root / dataset.manifest['trials'][0]['payload']
    payload.write_bytes(b'changed')

    with pytest.raises(ValueError, match='hash mismatch'):
        load_dataset(dataset.root)
