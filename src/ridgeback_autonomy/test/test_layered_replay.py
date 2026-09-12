from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from ridgeback_autonomy.benchmarking.layered_replay import evaluate_sensor_capture
from ridgeback_autonomy.benchmarking.replay_artifacts import (
    MASK_STATUS_EMPTY_SEGMENTATION,
    MASK_STATUS_OK,
    CachedMaskOutcome,
    MaskCacheWriter,
    SensorCaptureWriter,
    load_artifact,
    load_mask_trial,
    load_replay_input,
    load_sensor_trial,
    producer_signature,
)
from ridgeback_autonomy.benchmarking.replay_jobs import parse_job
from ridgeback_autonomy.benchmarking.replay import ReplayDatasetWriter
from ridgeback_autonomy.benchmarking.replay_materialization import (
    materialize_masks,
    materializer_producer_document,
)
from ridgeback_autonomy.benchmarking.replay_profiles import (
    PROFILE_MASK_OUTPUT,
    ProfileValidationError,
    describe_capabilities,
    validate_profile_axes,
)
from ridgeback_autonomy.benchmarking.target_replay_benchmark import (
    _materialization_kwargs,
    main as replay_main,
)
from ridgeback_autonomy.perception.target_localization.core.mask import (
    MaskPrecision,
    empty_region,
    region_from_blob,
)


HEIGHT, WIDTH = 60, 80


def _trial(trial_id='trial'):
    return {
        'trial_id': trial_id,
        'repeat_index': 1,
        'scene_id': 'single',
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


def _event(*, rgb=True, depth=True, detected=True):
    depth_m = np.full((HEIGHT, WIDTH), 4.0, dtype=np.float32)
    depth_m[20:40, 30:50] = 2.0
    return {
        'stamp_ns': 123,
        'frame_id': 'camera',
        'detected': detected,
        'count': 1 if detected else 0,
        'image_width': WIDTH,
        'image_height': HEIGHT,
        'intrinsics': {
            'fx': 100.0, 'fy': 100.0, 'cx': 40.0, 'cy': 30.0,
            'width': WIDTH, 'height': HEIGHT,
        },
        'camera_rotation': [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        'camera_translation': [0.0, 0.0, 0.0],
        'front_offset_m': 0.25,
        'base_above_floor_m': 0.026,
        'depth_usable_max_m': None,
        'rgb': np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8) if rgb else None,
        'depth_m': depth_m if depth else None,
        'detections': ([] if not detected else [{
            'bbox_xyxy': [30, 20, 50, 40],
            'label': 'target',
            'score': 0.9,
        }]),
    }


def _sensor(tmp_path: Path, name='sensor', events=None):
    writer = SensorCaptureWriter(
        tmp_path / name,
        producer={
            'commit': 'abc', 'dirty_count': 0, 'model': 'capture',
            'model_revision': None, 'dependencies': {'numpy': 'test'},
            'parameters': {'capture_batches': 1},
        },
        metadata={'scenario_path': '/frozen/scenario.yaml'},
    )
    writer.write_trial(_trial(), events or [_event()])
    return writer.finalize(trials_included=1, trials_skipped=0)


class _FakeSegmenter:
    def __init__(self, _model, _logger, **_kwargs):
        self._model = SimpleNamespace(config=SimpleNamespace(_commit_hash='resolved-revision'))

    def load(self):
        pass

    def segment_boxes(self, rgb, boxes, **_kwargs):
        assert rgb.shape == (HEIGHT, WIDTH, 3)
        blob = np.zeros((HEIGHT, WIDTH), dtype=bool)
        # Deliberately outside the prompt box on each side.
        blob[19:41, 29:51] = True
        return [blob.copy() for _box in boxes]


def _variant(name='baseline', estimators='projective_ranging', **arguments):
    return SimpleNamespace(name=name, arguments={
        'estimators': estimators,
        'isolation_2d': 'nearest_mode_histogram',
        'isolation_3d': 'height_crop_nearest_mode_band',
        **arguments,
    })


def test_capability_contract_owns_four_profiles_and_structured_frozen_error():
    contract = describe_capabilities()
    assert [profile['id'] for profile in contract['profiles']] == [
        'measurement', 'mask-output', 'mask-model', 'live-system']
    axes = {axis['name']: axis for axis in contract['axes']}
    assert all(axis['type'] != 'launch-value' for axis in axes.values())
    live = next(profile for profile in contract['profiles'] if profile['id'] == 'live-system')
    assert live['axis_defaults']['estimators'] == 'all'
    assert live['axis_defaults']['repeats'] == 5
    assert set(live['axis_defaults']) <= set(live['allowed_axes'])

    with pytest.raises(ProfileValidationError) as caught:
        validate_profile_axes(PROFILE_MASK_OUTPUT, {'segmentation_model'})
    assert caught.value.as_dict() == {
        'field': 'segmentation_model',
        'code': 'upstream_stage_frozen',
        'message': (
            'segmentation_model is frozen by profile "mask-output"; '
            'choose profile "mask-model" to vary it.'),
        'profile': 'mask-output',
        'suggested_profile': 'mask-model',
    }

    with pytest.raises(ProfileValidationError, match='at most 1.0'):
        parse_job({
            'job_version': 1,
            'profile': 'mask-model',
            'inputs': {'sensor_capture': '/sensor'},
            'sweep': {
                'sweep': {'name': 'one'},
                'configs': [{'name': 'one', 'estimators': 'projective_ranging'}],
            },
            'materializations': [{
                'name': 'bad', 'mask_producer': 'slimsam',
                'segmentation_min_iou': 1.5,
            }],
        })


def test_sensor_capture_round_trips_exact_rgb_depth_and_explicit_absence(tmp_path):
    events = [_event(), _event(rgb=False, depth=False, detected=False)]
    sensor = _sensor(tmp_path, events=events)

    trial, loaded = load_sensor_trial(sensor, sensor.trial_entries[0])

    assert trial == _trial()
    assert np.array_equal(loaded[0]['rgb'], events[0]['rgb'])
    assert loaded[0]['rgb'].tobytes() == events[0]['rgb'].tobytes()
    assert np.array_equal(loaded[0]['depth_m'], events[0]['depth_m'], equal_nan=True)
    assert loaded[1]['rgb'] is None
    assert loaded[1]['depth_m'] is None
    assert loaded[1]['detections'] == []


def test_manifest_dispatcher_keeps_legacy_measurement_inputs_loadable(tmp_path):
    writer = ReplayDatasetWriter(tmp_path / 'legacy', {'dataset_id': 'legacy'})
    writer.write_trial(_trial(), [{
        'stamp_ns': 1, 'detected': False, 'count': 0,
        'image_width': 1, 'image_height': 1, 'detections': [],
    }])
    writer.finalize(trials_included=1, trials_skipped=0)

    loaded = load_replay_input(tmp_path / 'legacy')

    assert loaded.manifest['schema_version'] == 1


def test_runner_sensor_mode_keeps_full_depth_and_waits_for_exact_rgb():
    from ridgeback_autonomy.benchmarking.target_distance_benchmark_runner_node import (
        TargetDistanceBenchmarkRunner,
    )

    event = _event(rgb=False, depth=False)
    event['intrinsics'] = None
    event['intrinsics_stamp_ns'] = None
    runner = SimpleNamespace(
        sensor_writer=object(),
        capture_batches=1,
        replay_capture_events={123: event},
        capture_events={('camera', 123): SimpleNamespace(stamp_ns=123)},
        get_logger=lambda: SimpleNamespace(warning=lambda _message: None),
    )
    depth = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)
    TargetDistanceBenchmarkRunner.store_replay_depth(
        runner, event, depth[:, :-1])
    TargetDistanceBenchmarkRunner.store_replay_rgb(
        runner, event, np.zeros((HEIGHT, WIDTH - 1, 3), dtype=np.uint8))
    TargetDistanceBenchmarkRunner.store_replay_intrinsics(
        runner,
        event,
        SimpleNamespace(
            width=WIDTH - 1, height=HEIGHT,
            k=[100.0, 0.0, 40.0, 0.0, 100.0, 30.0, 0.0, 0.0, 1.0],
        ),
        stamp_ns=123,
    )
    assert event['depth_m'] is None
    assert event['rgb'] is None
    assert event['intrinsics'] is None

    TargetDistanceBenchmarkRunner.store_replay_depth(runner, event, depth)
    TargetDistanceBenchmarkRunner.store_replay_intrinsics(
        runner,
        event,
        SimpleNamespace(
            width=WIDTH, height=HEIGHT,
            k=[100.0, 0.0, 40.0, 0.0, 100.0, 30.0, 0.0, 0.0, 1.0],
        ),
        stamp_ns=123,
    )

    assert np.array_equal(event['depth_m'], depth)
    assert event['depth_m'] is not depth
    assert event['intrinsics_stamp_ns'] == 123
    assert not TargetDistanceBenchmarkRunner.replay_capture_is_complete(runner)

    rgb = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    TargetDistanceBenchmarkRunner.store_replay_rgb(runner, event, rgb)
    assert event['rgb'] is not rgb
    assert TargetDistanceBenchmarkRunner.replay_capture_is_complete(runner)


def test_mask_region_cache_round_trip_preserves_none_empty_holes_and_extent(tmp_path):
    sensor = _sensor(tmp_path)
    blob = np.zeros((HEIGHT, WIDTH), dtype=bool)
    blob[18:42, 28:52] = True
    blob[25:28, 35:39] = False
    blob[10, 10] = True
    region = region_from_blob(blob, MaskPrecision.TIGHT)
    writer = MaskCacheWriter(
        tmp_path / 'cache', parent=sensor,
        producer={
            'commit': 'abc', 'dirty_count': 0, 'model': 'fake',
            'model_revision': 'one', 'dependencies': {}, 'parameters': {},
        },
    )
    writer.write_trial('trial', [[
        CachedMaskOutcome(region, MASK_STATUS_OK, 1.5),
        CachedMaskOutcome(None, MASK_STATUS_EMPTY_SEGMENTATION, 1.5),
        CachedMaskOutcome(
            empty_region(HEIGHT, WIDTH, MaskPrecision.TIGHT), MASK_STATUS_OK, 0.0),
    ]])
    cache = writer.finalize()

    loaded = load_mask_trial(cache, cache.trial_entries[0])[0]

    assert np.array_equal(loaded[0].region.to_full_array(), blob)
    assert loaded[0].region.origin_u == 10
    assert loaded[0].region.origin_v == 10
    assert loaded[0].region.precision is MaskPrecision.TIGHT
    assert loaded[1].region is None
    assert loaded[2].region.is_empty
    assert loaded[2].region is not None


def test_lineage_payload_corruption_and_incomplete_state_fail_before_evaluation(tmp_path):
    sensor = _sensor(tmp_path, 'sensor-a')
    other_event = _event()
    other_event['stamp_ns'] = 456
    other_sensor = _sensor(tmp_path, 'sensor-b', events=[other_event])
    cache = materialize_masks(
        sensor, tmp_path / 'cache', producer='box',
        code_provenance={'commit': 'abc', 'dirty_count': 0}, model=None)

    with pytest.raises(ValueError, match='parent artifact id mismatch'):
        load_artifact(cache.root, parent=other_sensor)

    payload = sensor.root / sensor.trial_entries[0]['payload']
    payload.write_bytes(payload.read_bytes() + b'corrupt')
    with pytest.raises(ValueError, match='payload hash mismatch'):
        load_artifact(sensor.root, require_parent=False)

    writer = SensorCaptureWriter(
        tmp_path / 'incomplete',
        producer={'commit': 'abc', 'dirty_count': 0, 'parameters': {}},
    )
    writer.write_trial(_trial(), [_event()])
    writer.mark_incomplete(reason='interrupted')
    with pytest.raises(ValueError, match='not complete'):
        load_artifact(tmp_path / 'incomplete', require_parent=False)


def test_materializer_signature_and_identity_change_with_mask_parameter(tmp_path):
    sensor = _sensor(tmp_path)
    first = materialize_masks(
        sensor, tmp_path / 'first', producer='slimsam', model='fake/model',
        code_provenance={'commit': 'abc', 'dirty_count': 0},
        prompt_padding_rel=0.05, segmenter_factory=_FakeSegmenter)
    second = materialize_masks(
        sensor, tmp_path / 'second', producer='slimsam', model='fake/model',
        code_provenance={'commit': 'abc', 'dirty_count': 0},
        prompt_padding_rel=0.10, segmenter_factory=_FakeSegmenter)

    assert first.id != second.id
    assert (
        first.manifest['producer']['signature_sha256']
        != second.manifest['producer']['signature_sha256'])
    region = load_mask_trial(first, first.trial_entries[0])[0][0].region
    assert region.origin_u == 29
    assert region.origin_v == 19

    with pytest.raises(ValueError, match='finite and non-negative'):
        materializer_producer_document(
            'slimsam', model='fake/model', model_revision=None,
            prompt_padding_rel=float('nan'), min_predicted_iou=0.5,
            device='auto', dtype='auto', preprocessing='transformers-default',
            code_provenance={'commit': 'abc', 'dirty_count': 0},
        )


@pytest.mark.parametrize(
    ('path', 'replacement'),
    [
        (('model',), 'candidate/model'),
        (('model_revision',), 'revision-b'),
        (('parameters', 'segmentation_prompt_padding_rel'), 0.10),
        (('parameters', 'segmentation_min_iou'), 0.75),
        (('parameters', 'segmentation_preprocessing'), 'processor-v2'),
        (('dependencies', 'transformers'), '99.0'),
        (('commit',), 'def'),
    ],
)
def test_producer_signature_covers_every_reproducibility_input(path, replacement):
    producer = {
        'commit': 'abc',
        'dirty_count': 0,
        'model': 'current/model',
        'model_revision': 'revision-a',
        'dependencies': {'numpy': '1.0', 'torch': '2.0', 'transformers': '3.0'},
        'parameters': {
            'segmentation_prompt_padding_rel': 0.05,
            'segmentation_min_iou': 0.5,
            'segmentation_preprocessing': 'transformers-default',
        },
    }
    changed = json.loads(json.dumps(producer))
    target = changed
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = replacement

    assert producer_signature(changed) != producer_signature(producer)


def test_box_and_slimsam_caches_feed_many_variants_without_model_rerun(tmp_path):
    sensor = _sensor(tmp_path)
    box = materialize_masks(
        sensor, tmp_path / 'box', producer='box',
        code_provenance={'commit': 'abc', 'dirty_count': 0})
    slim = materialize_masks(
        sensor, tmp_path / 'slim', producer='slimsam', model='fake/model',
        code_provenance={'commit': 'abc', 'dirty_count': 0},
        segmenter_factory=_FakeSegmenter)
    variants = (
        _variant('baseline', 'projective_ranging,euclidean_reconstruction'),
        _variant(
            'wide', 'projective_ranging,euclidean_reconstruction',
            isolation_2d_band_m='0.5'),
    )

    expanded, result = evaluate_sensor_capture(sensor, (box, slim), variants)
    parallel_expanded, parallel = evaluate_sensor_capture(
        sensor, (box, slim), variants, workers=2)

    assert parallel_expanded == expanded
    assert parallel == result
    assert [variant.name for variant in expanded] == [
        'box__baseline', 'box__wide', 'slim__baseline', 'slim__wide']
    for name in result:
        assert result[name]['rows']['projective_ranging'][0]['outcome'] == 'scored'
        assert result[name]['rows']['euclidean_reconstruction'][0]['outcome'] == 'scored'


def test_canonical_job_validates_profiles_resources_and_baseline(tmp_path):
    job = parse_job({
        'job_version': 1,
        'question': 'compare-mask-outputs',
        'profile': 'mask-output',
        'inputs': {'sensor_capture': '/sensor', 'mask_caches': ['/box', '/slim']},
        'sweep': {
            'sweep': {'name': 'paired'},
            'configs': [{
                'name': 'baseline', 'estimators': 'projective_ranging',
                'isolation_2d_band_m': 0.35,
            }],
        },
        'resources': {'measurement_workers': 2},
        'output_dir': '/new-output',
        'comparison_baseline': 'box:baseline',
    })
    assert job.profile == 'mask-output'
    assert job.measurement_workers == 2
    assert job.comparison_baseline == 'box:baseline'

    model_job = parse_job({
        'job_version': 1,
        'profile': 'mask-model',
        'inputs': {'sensor_capture': '/sensor'},
        'sweep': {
            'sweep': {'name': 'model'},
            'configs': [{'name': 'baseline', 'estimators': 'projective_ranging'}],
        },
        'materializations': [{'name': 'current', 'mask_producer': 'slimsam'}],
    })
    resolved = _materialization_kwargs(model_job.materializations[0])
    assert resolved['model'] == 'Zigeng/SlimSAM-uniform-50'
    assert resolved['preprocessing'] == 'transformers-default'

    incompatible = json.loads(json.dumps(model_job.document))
    incompatible['inputs']['mask_caches'] = ['/ignored-cache']
    with pytest.raises(ProfileValidationError) as caught:
        parse_job(incompatible)
    assert caught.value.as_dict() == {
        'field': 'inputs.mask_caches',
        'code': 'incompatible_input',
        'message': 'Profile "mask-model" does not consume input "mask_caches".',
        'profile': 'mask-model',
    }

    frozen_default = json.loads(json.dumps(job.document))
    frozen_default['sweep']['defaults'] = {'detector_fps': 7.0}
    with pytest.raises(ProfileValidationError) as caught:
        parse_job(frozen_default)
    assert caught.value.suggested_profile == 'live-system'

    live = json.loads(json.dumps(model_job.document))
    live['profile'] = 'live-system'
    live['inputs'] = {}
    live['materializations'] = None
    with pytest.raises(ProfileValidationError) as caught:
        parse_job(live)
    assert caught.value.code == 'live_sweep_requires_path'


def test_mask_output_job_runs_atomically_and_labels_claim_boundary(tmp_path):
    sensor = _sensor(tmp_path)
    box = materialize_masks(
        sensor, tmp_path / 'box', producer='box',
        code_provenance={'commit': 'abc', 'dirty_count': 0})
    output = tmp_path / 'output'
    job_path = tmp_path / 'job.yaml'
    job_path.write_text(yaml.safe_dump({
        'job_version': 1,
        'question': 'compare-mask-outputs',
        'profile': 'mask-output',
        'inputs': {
            'sensor_capture': str(sensor.root),
            'mask_caches': [str(box.root)],
        },
        'sweep': {
            'sweep': {'name': 'paired'},
            'configs': [{
                'name': 'baseline', 'estimators': 'projective_ranging',
            }],
        },
        'resources': {'measurement_workers': 1},
        'output_dir': str(output),
        'comparison_baseline': 'baseline',
    }, sort_keys=False), encoding='utf-8')

    assert replay_main([str(job_path)]) == 0

    replay = json.loads((output / 'results/replay.json').read_text())
    assert replay['profile']['id'] == 'mask-output'
    assert 'latency' not in replay['profile']['supported_claims']
    assert (output / 'results/baseline/projective_ranging.csv').is_file()
    assert not any(path.name.startswith(f'.{output.name}.partial') for path in tmp_path.iterdir())


def test_layered_replay_imports_without_ros_or_model_stack():
    package_dir = Path(__file__).parents[1]
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join(filter(None, (
        str(package_dir), env.get('PYTHONPATH', ''),
    )))
    script = '''
import importlib.abc
import sys

blocked = {
    'cv_bridge', 'geometry_msgs', 'rclpy', 'sensor_msgs', 'std_msgs',
    'tf2_ros', 'torch', 'transformers', 'visualization_msgs',
}

class BlockHeavy(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if any(fullname == name or fullname.startswith(name + '.') for name in blocked):
            raise ModuleNotFoundError(f'blocked import: {fullname}')
        return None

sys.meta_path.insert(0, BlockHeavy())
import ridgeback_autonomy.benchmarking.target_replay_benchmark
'''
    subprocess.run([sys.executable, '-c', script], check=True, env=env)
