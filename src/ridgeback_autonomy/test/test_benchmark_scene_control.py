"""Runner integration with the simulator-neutral scene-control contract."""

from __future__ import annotations

import contextlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from ridgeback_autonomy.benchmarking.asset_catalog import load_asset_catalog
from ridgeback_autonomy.benchmarking.scenarios import ObjectSpec, RobotSpec, Scene
from ridgeback_autonomy.benchmarking.scene_control import SceneSnapshot
from ridgeback_autonomy.benchmarking.simulation import GroundTruthInstance, normalize_pose
from ridgeback_autonomy.benchmarking.target_distance_benchmark_runner_node import (
    TargetDistanceBenchmarkRunner,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
CATALOG_PATH = (
    REPO_ROOT / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'benchmark_assets.yaml'
)


class _Logger:
    def __init__(self):
        self.errors = []

    def info(self, _message):
        pass

    def error(self, message):
        self.errors.append(message)


class _TrackingSceneControl:
    def __init__(self, *, fail_spawn=False):
        self.entities = []
        self.fail_spawn = fail_spawn

    @contextlib.contextmanager
    def scene(self, specs):
        try:
            for index, spec in enumerate(specs):
                self.entities.append(spec.name)
                if self.fail_spawn and index == 0:
                    raise RuntimeError('spawn rejected')
            yield tuple(self.entities)
        finally:
            self.entities.clear()


def _scene() -> Scene:
    return Scene(
        id='two_targets',
        robots=(
            RobotSpec(x=2.0, y=0.0, yaw=3.14),
            RobotSpec(x=3.0, y=0.5, yaw=3.14),
        ),
        objects=(ObjectSpec(model='table', x=1.0, y=-0.5, yaw=0.25),),
        repeats_override=None,
    )


def _trial_runner(scene_control, *, truth=None, capture=None):
    logger = _Logger()
    specs = [SimpleNamespace(name='bench_run_target_0'),
             SimpleNamespace(name='bench_run_obj_0')]
    runner = SimpleNamespace(
        run_label='run',
        scene_control=scene_control,
        scene_spawn_specs=lambda _scene, _prefix: (
            specs, [(0, 'bench_run_target_0', _scene.robots[0])]),
        spin_for=lambda _duration: None,
        settle_sec=0.0,
        compute_scene_ground_truth=(
            truth if truth is not None else lambda _models: [GroundTruthInstance(
                index=0,
                model_name='bench_run_target_0',
                world_x=2.0,
                world_y=0.0,
                forward_m=1.75,
                lateral_m=0.0,
                distance_m=1.75,
            )]
        ),
        capture_measurement_window=(capture or (lambda _duration: (_ for _ in ()).throw(
            RuntimeError('capture failed')))),
        capture_sec=0.0,
        capture_artifact_enabled=lambda: False,
        selected_estimators=('pointcloud',),
        estimator_display_names={'pointcloud': 'Pointcloud'},
        status_aggregate={},
        capture_events={},
        active_truth=None,
        active_trial_id='',
        get_logger=lambda: logger,
    )
    return runner, logger


def test_runner_waits_for_an_advancing_clock_before_its_first_scene_operation() -> None:
    events = []
    scene_control = SimpleNamespace(
        wait_for_services=lambda timeout: events.append(('services', timeout)),
        require_capabilities=lambda formats: events.append(('capabilities', formats)),
        wait_for_advancing_clock=lambda timeout: events.append(('clock', timeout)) or (
            SimpleNamespace(advanced_stamp_ns=1_500_000_000)),
        readiness_timeline=lambda: {'clock_advance_after_services_sec': 0.25},
        wait_until_present=lambda names: events.append(('robot', tuple(names))),
    )
    messages = []
    runner = SimpleNamespace(
        log_code_provenance=lambda: events.append(('provenance',)),
        get_logger=lambda: SimpleNamespace(info=messages.append),
        estimator_display_names={'pointcloud': 'Pointcloud'},
        selected_estimators=('pointcloud',),
        world='target_distance_calibration',
        backend=SimpleNamespace(name='isaac'),
        robot_model_name='r100_0001/robot',
        scene_control=scene_control,
        asset_selection=SimpleNamespace(
            spawn_formats=frozenset({'usd'}),
            world=SimpleNamespace(native_name='g1_distance_calibration')),
        wait_for_required_streams=lambda: events.append(('streams',)),
        start_recording=lambda: events.append(('recording',)),
        run_trials=lambda: events.append(('trials',)),
        stop_recording=lambda: events.append(('stopped',)),
        log_readiness_timeline=lambda: events.append(('timeline',)),
    )

    TargetDistanceBenchmarkRunner.run(runner)

    assert [event[0] for event in events] == [
        'provenance', 'services', 'capabilities', 'clock', 'robot', 'streams',
        'recording', 'trials', 'stopped', 'timeline',
    ]
    assert any('native world "g1_distance_calibration"' in message for message in messages)
    assert any('Simulation clock advancing at 1.500 s' in message for message in messages)


def test_runner_records_backend_world_catalog_and_readiness_provenance(monkeypatch) -> None:
    from ridgeback_autonomy.benchmarking import target_distance_benchmark_runner_node as module
    from ridgeback_autonomy.benchmarking.backends import BENCHMARK_BACKENDS
    from ridgeback_autonomy.benchmarking.scene_control import SimulatorCapabilities

    isaac_catalog = (
        REPO_ROOT / 'src' / 'ridgeback_autonomy_isaac' / 'sim' / 'benchmark_assets.yaml')
    selection = load_asset_catalog(str(isaac_catalog)).select(
        'target_distance_calibration', 'g1', {'table'})
    monkeypatch.setattr(
        module, 'simulator_provenance',
        lambda backend: {'adapter_package': backend.adapter_package,
                         'simulator': {'name': 'isaacsim', 'version': '6.1.0.0'}})
    timeline = {'services_available_wall': 1.0, 'clock_advanced_wall': 2.0,
                'first_spawn_requested_wall': 3.0}
    runner = SimpleNamespace(
        backend=BENCHMARK_BACKENDS['isaac'],
        world='target_distance_calibration',
        backend_settings={'depth_fidelity': 'ideal', 'headless': 'true',
                          'rtf': '1.0', 'sim_mode': 'realtime'},
        get_parameter=lambda name: SimpleNamespace(value={'camera_profile': '640x480'}[name]),
        scene_control_service_namespace='/isaac_sim',
        scene_control=SimpleNamespace(
            capabilities=SimulatorCapabilities(frozenset({10, 0, 1}), ('usd',), 'static'),
            readiness_timeline=lambda: timeline),
        asset_selection=selection,
    )

    simulation = TargetDistanceBenchmarkRunner.simulation_provenance(runner)

    assert simulation['backend'] == 'isaac'
    assert simulation['public_world'] == 'target_distance_calibration'
    assert simulation['native_world'] == 'g1_distance_calibration'
    assert simulation['settings']['depth_fidelity'] == 'ideal'
    assert simulation['camera_profile'] == '640x480'
    assert simulation['simulator'] == {'name': 'isaacsim', 'version': '6.1.0.0'}
    assert simulation['features'] == {
        'features': [0, 1, 10], 'spawn_formats': ['usd'], 'custom_info': 'static'}
    assert simulation['asset_catalog']['schema'] == 'ridgeback_benchmark_assets/1'
    assert simulation['asset_catalog']['sha256'] == selection.catalog.sha256
    assert simulation['asset_catalog']['world']['native_name'] == 'g1_distance_calibration'
    assert simulation['asset_catalog']['objects']['table']['format'] == 'usd'
    assert simulation['readiness'] == timeline


def test_catalog_resources_resolve_into_target_first_spawn_order() -> None:
    catalog = load_asset_catalog(str(CATALOG_PATH))
    selection = catalog.select('target_distance_calibration', 'g1', {'table'})
    runner = SimpleNamespace(asset_selection=selection)

    specs, targets = TargetDistanceBenchmarkRunner.scene_spawn_specs(
        runner, _scene(), 'bench_run_two_targets_r0')

    assert [spec.name for spec in specs] == [
        'bench_run_two_targets_r0_target_0',
        'bench_run_two_targets_r0_target_1',
        'bench_run_two_targets_r0_obj_0',
    ]
    assert [Path(spec.uri).name for spec in specs] == ['model.sdf'] * 3
    assert specs[-1].uri.endswith('/models/table/model.sdf')
    assert [name for _index, name, _robot in targets] == [
        'bench_run_two_targets_r0_target_0',
        'bench_run_two_targets_r0_target_1',
    ]


def test_truth_reads_robot_and_every_target_from_one_snapshot() -> None:
    calls = []
    poses = {
        'r100_0001/robot': normalize_pose({
            'name': 'r100_0001/robot',
            'position': {'x': 0.0, 'y': 0.0},
            'orientation': {'w': 1.0},
        }),
        'bench_run_target_0': normalize_pose({
            'name': 'bench_run_target_0',
            'position': {'x': 2.0, 'y': 0.0},
            'orientation': {'w': 1.0},
        }),
        'bench_run_target_1': normalize_pose({
            'name': 'bench_run_target_1',
            'position': {'x': 3.0, 'y': 0.5},
            'orientation': {'w': 1.0},
        }),
    }

    def snapshot(names):
        calls.append(tuple(names))
        return SceneSnapshot('world', 123, {name: poses[name] for name in names})

    runner = SimpleNamespace(
        robot_model_name='r100_0001/robot',
        scene_control=SimpleNamespace(get_scene_snapshot=snapshot),
    )
    targets = [
        (0, 'bench_run_target_0', object()),
        (1, 'bench_run_target_1', object()),
    ]

    truth = TargetDistanceBenchmarkRunner.compute_scene_ground_truth(runner, targets)

    assert calls == [(
        'r100_0001/robot', 'bench_run_target_0', 'bench_run_target_1')]
    assert [value.world_x for value in truth] == [2.0, 3.0]


def test_spawn_rejection_leaves_no_owned_entity() -> None:
    scene_control = _TrackingSceneControl(fail_spawn=True)
    runner, logger = _trial_runner(scene_control)

    result = TargetDistanceBenchmarkRunner.run_trial(
        runner, {'trial_id': 's1_r0', 'scene': _scene()})

    assert result is None
    assert scene_control.entities == []
    assert logger.errors


def test_missing_truth_state_leaves_no_owned_entity() -> None:
    scene_control = _TrackingSceneControl()
    runner, logger = _trial_runner(
        scene_control,
        truth=lambda _models: (_ for _ in ()).throw(RuntimeError('target state missing')),
    )

    result = TargetDistanceBenchmarkRunner.run_trial(
        runner, {'trial_id': 's1_r0', 'scene': _scene()})

    assert result is None
    assert scene_control.entities == []
    assert logger.errors


def test_capture_interrupt_leaves_no_owned_entity() -> None:
    scene_control = _TrackingSceneControl()
    runner, _logger = _trial_runner(
        scene_control,
        capture=lambda _duration: (_ for _ in ()).throw(KeyboardInterrupt()),
    )

    with pytest.raises(KeyboardInterrupt):
        TargetDistanceBenchmarkRunner.run_trial(
            runner, {'trial_id': 's1_r0', 'scene': _scene()})

    assert scene_control.entities == []


def test_successful_trial_removes_the_scene_before_return(monkeypatch) -> None:
    import ridgeback_autonomy.benchmarking.target_distance_benchmark_runner_node as module

    scene_control = _TrackingSceneControl()
    capture = lambda _duration: {
        'status_histogram': {},
        'usable_by_estimator': {'pointcloud': []},
        'usable_events': [],
        'total_events': 0,
        'any_detected': False,
    }
    runner, _logger = _trial_runner(scene_control, capture=capture)
    monkeypatch.setattr(module, 'score_scene', lambda *_args, **_kwargs: object())
    monkeypatch.setattr(module, 'build_trial_result', lambda *_args, **_kwargs: 'complete')

    result = TargetDistanceBenchmarkRunner.run_trial(
        runner, {'trial_id': 's1_r0', 'scene': _scene()})

    assert result == 'complete'
    assert scene_control.entities == []


def test_common_scene_control_has_no_simulator_command_path() -> None:
    benchmark_root = (
        REPO_ROOT / 'src' / 'ridgeback_autonomy' / 'ridgeback_autonomy' / 'benchmarking'
    )
    sources = '\n'.join(
        (benchmark_root / name).read_text(encoding='utf-8')
        for name in (
            'target_distance_benchmark_runner_node.py',
            'target_benchmark_sweep.py',
            'simulation.py',
            'process_utils.py',
        )
    )

    for obsolete in (
        'ros_gz_sim/create',
        "'gz', 'service'",
        "'gz', 'topic'",
        'gz.msgs.Entity',
        '--json-output',
        'spawn_model_command',
        'despawn_model_command',
        'pose_snapshot_from_payload',
    ):
        assert obsolete not in sources
