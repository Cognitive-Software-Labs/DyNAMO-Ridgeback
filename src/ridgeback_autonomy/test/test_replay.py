import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from ridgeback_autonomy.benchmarking.replay import (
    ReplayDatasetWriter,
    evaluate_dataset,
    load_dataset,
    write_replay_results,
)
from ridgeback_autonomy.benchmarking.target_offline_replay_benchmark import (
    REPLAY_V1_ARGUMENT_NAMES,
    _source_repository,
    _validated_v1_variants,
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


def _deprojectable_event() -> dict:
    """The shared event with a focal length and a camera height worth deprojecting.

    Projective ranging reads depth off the window and never deprojects, so the
    shared fixture's 1-pixel focal length and floor-level camera cost it
    nothing. Euclidean reconstruction does deproject: there the same numbers
    scatter a 4x4 patch across six metres and put it below the floor, which its
    isolation then correctly throws away.
    """

    event = _event(depth_roi=np.full((4, 4), 2.0, dtype=np.float32))
    event['intrinsics'].update({'fx': 100.0, 'fy': 100.0})
    event['camera_translation'] = [0.0, 0.0, 1.16]
    return event


def test_euclidean_reconstruction_measures_the_same_frozen_roi(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_deprojectable_event()])

    result = evaluate_dataset(
        dataset, (_variant(estimators='euclidean_reconstruction'),))['baseline']

    # The floor reference comes from the stored optical-to-base extrinsics, so
    # this needs no evidence the projective path did not already require.
    row = result['rows']['euclidean_reconstruction'][0]
    assert row['outcome'] == 'scored'
    assert row['trial_estimate_m'] == pytest.approx(1.75)
    assert 'projective_ranging' not in result['rows']


def test_both_depth_estimators_score_from_one_pass(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_deprojectable_event()])

    result = evaluate_dataset(dataset, (_variant(
        estimators='projective_ranging,euclidean_reconstruction'),))['baseline']

    assert set(result['rows']) == {'projective_ranging', 'euclidean_reconstruction'}
    assert {rows[0]['outcome'] for rows in result['rows'].values()} == {'scored'}
    # Both read one prepared region, so neither can score on a selection the
    # other never saw.
    assert [rows[0]['trial_estimate_m'] for rows in result['rows'].values()] == [
        pytest.approx(1.75), pytest.approx(1.75)]


def test_a_missing_depth_roi_is_a_miss_for_every_selected_estimator(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_event(depth_roi=None)])

    result = evaluate_dataset(dataset, (_variant(
        estimators='projective_ranging,euclidean_reconstruction'),))['baseline']

    assert {rows[0]['miss_reason'] for rows in result['rows'].values()} == {'NO_DEPTH_FRAME'}


def test_an_estimator_the_legacy_dataset_cannot_feed_is_refused() -> None:
    config = SimpleNamespace(
        name='baseline',
        arguments=_variant(estimators='polar_profiling').arguments,
        explicit_keys=frozenset({'estimators'}),
    )

    with pytest.raises(ValueError, match='needs evidence the legacy dataset does not carry'):
        _validated_v1_variants(SimpleNamespace(configs=(config,)))


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


def test_dataset_with_skipped_trials_cannot_be_finalized(tmp_path: Path) -> None:
    writer = ReplayDatasetWriter(tmp_path / 'dataset', {'dataset_id': 'partial'})
    writer.write_trial(_trial(), [_event(detected=False)])

    with pytest.raises(ValueError, match='skipped trial'):
        writer.finalize(trials_included=1, trials_skipped=1)

    manifest = json.loads((writer.root / 'manifest.json').read_text(encoding='utf-8'))
    assert manifest['state'] == 'capturing'


def test_loader_rejects_legacy_complete_dataset_with_skipped_trials(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_event(detected=False)])
    manifest_path = dataset.root / 'manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest['trials_included'] = 1
    manifest['trials_skipped'] = 4
    manifest_path.write_text(json.dumps(manifest), encoding='utf-8')

    with pytest.raises(ValueError, match='4 skipped trial'):
        load_dataset(dataset.root)


def test_replay_variants_strip_live_defaults_and_reject_irrelevant_axes() -> None:
    config = SimpleNamespace(
        name='baseline',
        arguments={
            **_variant().arguments,
            'scenario': '/unused/scenario.yaml',
            'repeats': '5',
            'record_video': 'false',
        },
        explicit_keys=frozenset({'isolation_2d'}),
    )
    variants = _validated_v1_variants(SimpleNamespace(configs=(config,)))

    assert set(variants[0].arguments) == REPLAY_V1_ARGUMENT_NAMES
    assert variants[0].arguments['isolation_2d'] == 'nearest_mode_histogram'

    config.explicit_keys = frozenset({'capture_sec'})
    with pytest.raises(ValueError, match='does not affect offline depth measurement'):
        _validated_v1_variants(SimpleNamespace(configs=(config,)))


def test_replay_output_includes_timing_and_cross_variant_report(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_event(depth_roi=np.full((4, 4), 2.0, dtype=np.float32))])
    variants = (_variant(),)
    results = evaluate_dataset(dataset, variants)
    output = tmp_path / 'output'

    write_replay_results(
        output,
        dataset,
        variants,
        results,
        evaluation_provenance={
            'commit': 'abc123',
            'branch': 'test',
            'dirty_count': 0,
            'sweep': '/tmp/sweep.yaml',
            'sweep_sha256': '1234',
            'started': '2026-09-09 12:00:00 +0200',
            'finished': '2026-09-09 12:00:01 +0200',
        },
        worker_count=1,
        evaluation_wall_time_sec=0.25,
        sweep_name='unit_replay',
        sweep_description='unit replay output',
    )

    replay = json.loads((output / 'replay.json').read_text(encoding='utf-8'))
    assert replay['evaluation_wall_time_sec'] == 0.25
    assert (output / 'sweep.json').is_file()
    assert '# Benchmark sweep unit_replay' in (output / 'summary.md').read_text(
        encoding='utf-8')


def test_offline_provenance_resolves_from_the_evaluator_source() -> None:
    assert (_source_repository() / '.git').exists()


def test_offline_entrypoint_imports_without_ros() -> None:
    """Keep the executor usable on a machine with NumPy/YAML but no ROS."""

    package_dir = Path(__file__).parents[1]
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join(filter(None, (
        str(package_dir),
        env.get('PYTHONPATH', ''),
    )))
    script = '''
import importlib.abc
import sys

blocked = {
    'cv_bridge', 'geometry_msgs', 'launch', 'launch_ros', 'rclpy',
    'ridgeback_autonomy.msg', 'sensor_msgs', 'std_msgs', 'tf2_ros',
    'visualization_msgs',
}

class BlockRos(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if any(fullname == name or fullname.startswith(name + '.') for name in blocked):
            raise ModuleNotFoundError(f'blocked ROS import: {fullname}')
        return None

sys.meta_path.insert(0, BlockRos())
import ridgeback_autonomy.benchmarking.target_offline_replay_benchmark
'''
    subprocess.run([sys.executable, '-c', script], check=True, env=env)
