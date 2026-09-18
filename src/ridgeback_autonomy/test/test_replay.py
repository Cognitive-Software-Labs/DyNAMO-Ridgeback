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
    sha256_file,
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


# Scan frame (x forward, y left, z up) into the camera optical frame
# (X right, Y down, Z forward), with the two origins coincident.
SCAN_TO_OPTICAL = [
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
]


def _scan() -> dict:
    """A 2 m target against a 6 m wall, both landing inside the detection box.

    The fixture intrinsics are fx = fy = 1 on a 6x6 grid, so a beam at bearing t
    lands on image column ``2.5 - tan(t)`` and row 2.5; every bearing out to
    roughly +-1.1 rad falls inside the [1, 5) box. The target occupies the
    innermost +-0.2 rad and the wall fills the rest of the mask, which is the
    parallax case polar actually has to segment: a mask selects both, and only
    ``range_jump_m`` splitting the profile keeps the wall out of the estimate.

    The target's angular extent is deliberately small. Beams at constant range
    lie on an arc, so a target spanning a wide bearing window would have a
    median depth visibly shorter than its range -- true of the geometry, but it
    would make the expected number an artifact of the fixture rather than the
    2.0 m the beams report.
    """

    bearings = -1.4 + np.arange(29) * 0.1
    return {
        'ranges': np.where(np.abs(bearings) <= 0.2, 2.0, 6.0).astype(np.float32),
        'angle_min': -1.4,
        'angle_max': 1.4,
        'angle_increment': 0.1,
        'range_min': 0.15,
        'range_max': 12.0,
        'frame_id': 'lidar2d_0',
        'stamp_ns': 123,
    }


def _event(*, depth_roi=None, detected=True, scan=False) -> dict:
    if scan:
        return {
            **_event(depth_roi=depth_roi, detected=detected),
            'scan': _scan(),
            'scan_rotation': SCAN_TO_OPTICAL,
            'scan_translation': [0.0, 0.0, 0.0],
        }
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


def _schema_1(dataset):
    """Rewrite a dataset as the scan-less evidence written before schema 2.

    The writer only emits schema 2 now, so the guarantee that the historical
    datasets stay loadable has to be tested against a payload that genuinely
    lacks the scan keys -- not one that merely carries them empty.
    """

    manifest = json.loads(json.dumps(dataset.manifest))
    manifest['schema_version'] = 1
    for entry in manifest['trials']:
        path = dataset.root / entry['payload']
        with np.load(path, allow_pickle=False) as payload:
            arrays = {
                key: payload[key] for key in payload.files
                if not key.startswith('scan_')
            }
        metadata = json.loads(str(arrays.pop('metadata_json').item()))
        metadata['schema_version'] = 1
        for event in metadata['events']:
            event.pop('scan', None)
            event.pop('scan_key', None)
        arrays['metadata_json'] = np.asarray(json.dumps(metadata, sort_keys=True))
        np.savez_compressed(path, **arrays)
        entry['sha256'] = sha256_file(path)
    (dataset.root / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, sort_keys=True), encoding='utf-8')
    return load_dataset(dataset.root)


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


def test_an_estimator_the_legacy_dataset_cannot_feed_is_refused(tmp_path: Path) -> None:
    """Refused against the dataset, not against a fixed list in the CLI.

    Which estimators are reachable depends on the schema version of the file
    being replayed, so a check that could not see the file would either repeat
    the dataset's own answer or contradict it.
    """

    dataset = _schema_1(_dataset(
        tmp_path, [_event(depth_roi=np.full((4, 4), 2.0, dtype=np.float32))]))

    with pytest.raises(ValueError, match='can evaluate none of the requested') as caught:
        evaluate_dataset(dataset, (_variant(estimators='polar_profiling'),))

    assert 'schema version 1' in str(caught.value)


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


def test_polar_profiling_is_measured_from_the_stored_scan(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path, [_event(
        depth_roi=np.full((4, 4), 2.0, dtype=np.float32), scan=True)])

    result = evaluate_dataset(
        dataset, (_variant(estimators='projective_ranging,polar_profiling'),))['baseline']

    assert result['dropped_estimators'] == {}
    row = result['rows']['polar_profiling'][0]
    assert row['outcome'] == 'scored'
    # The target beams sit at 2.0 m and the robot front is 0.25 m ahead of the
    # camera, so polar and the depth row converge on the same 1.75 m truth.
    assert row['trial_estimate_m'] == pytest.approx(1.75, abs=0.05)
    assert result['rows']['projective_ranging'][0]['outcome'] == 'scored'


def test_a_missing_depth_frame_does_not_take_polar_profiling_down_with_it(
    tmp_path: Path,
) -> None:
    """Polar reads the scan, so the depth gate is not its gate.

    The live kernel runs polar whether or not a depth frame arrived. Replay that
    reported NO_DEPTH_FRAME for a scan-based row would understate its coverage
    against the run it is supposed to reproduce.
    """

    dataset = _dataset(tmp_path, [_event(depth_roi=None, scan=True)])

    result = evaluate_dataset(
        dataset, (_variant(estimators='projective_ranging,polar_profiling'),))['baseline']

    assert result['rows']['projective_ranging'][0]['miss_reason'] == 'NO_DEPTH_FRAME'
    assert result['rows']['polar_profiling'][0]['outcome'] == 'scored'


def test_a_schema_1_dataset_still_loads_and_runs_the_depth_path(tmp_path: Path) -> None:
    dataset = _schema_1(_dataset(
        tmp_path, [_event(depth_roi=np.full((4, 4), 2.0, dtype=np.float32))]))

    result = evaluate_dataset(dataset, (_variant(),))['baseline']

    assert dataset.manifest['schema_version'] == 1
    assert result['rows']['projective_ranging'][0]['outcome'] == 'scored'


def test_an_estimator_this_dataset_cannot_feed_is_dropped_with_its_reason(
    tmp_path: Path,
) -> None:
    dataset = _dataset(tmp_path, [_event(depth_roi=np.full((4, 4), 2.0, dtype=np.float32))])
    variants = (_variant(estimators='pointcloud,projective_ranging'),)

    results = evaluate_dataset(dataset, variants)

    result = results['baseline']
    assert result['selected_estimators'] == ('projective_ranging',)
    assert 'point cloud' in result['dropped_estimators']['pointcloud']
    # The estimator that can run still runs: one absent channel does not void
    # the variant.
    assert result['rows']['projective_ranging'][0]['outcome'] == 'scored'

    output = tmp_path / 'output'
    write_replay_results(
        output, dataset, variants, results,
        evaluation_provenance={'commit': 'abc123', 'branch': 'test', 'dirty_count': 0},
        worker_count=1, evaluation_wall_time_sec=0.25,
        sweep_name='unit_replay', sweep_description='unit replay output')

    document = json.loads((output / 'baseline/run.json').read_text(encoding='utf-8'))
    # Carried into the results rather than left as a smaller table: a run.json
    # missing a row is indistinguishable from one whose estimator scored nothing.
    assert 'point cloud' in document['replay']['dropped_estimators']['pointcloud']
    assert 'Estimators not run' in (output / 'baseline/summary.md').read_text(
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
    'ridgeback_interfaces.msg', 'sensor_msgs', 'std_msgs', 'tf2_ros',
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
