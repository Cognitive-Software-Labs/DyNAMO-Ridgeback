from __future__ import annotations

from collections import deque
import json
import importlib.util
import sys
import math
from pathlib import Path

import numpy as np
import pytest
from sensor_msgs.msg import LaserScan

from ridgeback_autonomy.common.lidar_contract import (
    FRONT_LIDAR_XY_YAW,
    MERGED_ANGLE_MAX,
    MERGED_ANGLE_MIN,
    MERGED_RANGE_MAX,
    MERGED_SCAN_BINS,
    RAW_ANGLE_INCREMENT,
    RAW_RANGE_MAX,
    slam_max_laser_range,
)
from ridgeback_autonomy.common.scan_merger_node import ScanMergerNode


def _load_repo_module(name, relative_path):
    repo = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(name, repo / relative_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _scan(stamp_s: float, *, angle: float = 0.0, distance: float = 10.0):
    scan = LaserScan()
    scan.header.stamp.sec = int(stamp_s)
    scan.header.stamp.nanosec = round((stamp_s - int(stamp_s)) * 1e9)
    scan.angle_min = angle
    scan.angle_max = angle
    scan.angle_increment = RAW_ANGLE_INCREMENT
    scan.scan_time = 0.025
    scan.range_min = 0.06
    scan.range_max = RAW_RANGE_MAX
    scan.ranges = [distance]
    return scan


def _bare_merger():
    merger = object.__new__(ScanMergerNode)
    merger.front_pose = FRONT_LIDAR_XY_YAW
    merger.rear_pose = (-0.3922, 0.0, math.pi)
    merger.output_frame_id = 'base_link'
    merger._range_pad = 0.3922
    merger._deltas = deque(maxlen=500)
    merger._n_paired = 0
    merger._n_equal_stamp = 0
    merger._n_motion_compensated = 0
    merger._n_front_only = 0
    merger._n_rear_dropped = 0
    merger._tf_misses = 0
    merger._pub = _Publisher()
    return merger


def test_slam_range_threshold_follows_the_scan_frame():
    assert slam_max_laser_range('front_only') == pytest.approx(10.0)
    assert slam_max_laser_range('merged') == pytest.approx(10.3922)
    with pytest.raises(ValueError, match='slam_source'):
        slam_max_laser_range('unknown')


def test_merged_contract_is_1440_base_link_bins_with_padded_range():
    assert MERGED_SCAN_BINS == 1440
    assert MERGED_ANGLE_MIN == pytest.approx(-math.pi)
    assert math.degrees(MERGED_ANGLE_MAX) == pytest.approx(179.75)
    assert MERGED_RANGE_MAX == pytest.approx(10.3922)


def test_front_sensor_maximum_ray_remains_valid_after_base_transform():
    merger = _bare_merger()
    points = merger._scan_points_in_frame(_scan(10.0), merger.front_pose)
    ranges, _ = merger._project_points((points,), RAW_ANGLE_INCREMENT)

    finite = ranges[np.isfinite(ranges)]
    assert finite.tolist() == pytest.approx([MERGED_RANGE_MAX])
    assert finite.max() <= MERGED_RANGE_MAX


def test_equal_stamp_and_motion_compensated_paths_are_counted_separately():
    merger = _bare_merger()
    merger._odom_base = lambda stamp: (0.0, 0.0, 0.1 * stamp)

    merger._publish_merge(10.0, _scan(10.0), 10.0, _scan(10.0))
    assert merger._n_paired == 1
    assert merger._n_equal_stamp == 1
    assert merger._n_motion_compensated == 0

    merger._publish_merge(10.01, _scan(10.01), 10.0, _scan(10.0))
    assert merger._n_paired == 2
    assert merger._n_equal_stamp == 1
    assert merger._n_motion_compensated == 1
    assert merger._tf_misses == 0
    assert merger._n_rear_dropped == 0


def test_missing_rear_and_missing_tf_are_distinct_branches():
    merger = _bare_merger()
    merger._odom_base = lambda stamp: None
    merger._publish_merge(10.0, _scan(10.0), None, None)
    merger._publish_merge(10.01, _scan(10.01), 10.0, _scan(10.0))
    assert merger._n_front_only == 1
    assert merger._n_rear_dropped == 1
    assert merger._tf_misses == 1
    assert merger._n_paired == 0


def test_base_frame_range_filter_precedes_nearest_collision():
    points = np.array([[0.03, 0], [2, 0], [11, 0], [-11, 0]])
    ranges, _ = ScanMergerNode._project_points(
        (points,), RAW_ANGLE_INCREMENT, 0.06, MERGED_RANGE_MAX)
    assert ranges[np.isfinite(ranges)].tolist() == [2.0]


def test_queued_startup_front_waits_for_rear_callback(monkeypatch):
    import ridgeback_autonomy.common.scan_merger_node as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.0)
    merger = _bare_merger()
    merger.tolerance = 0.02
    merger._n_front = 0
    merger._front_pending = deque(maxlen=20)
    merger._rear_buffer = deque(maxlen=20)
    merger._on_front(_scan(1.0))  # old capture time, newly received callback
    assert len(merger._pub.messages) == 0
    merger._on_rear(_scan(1.0))
    assert merger._n_equal_stamp == 1
    assert merger._n_front_only == 0

    merger._on_front(_scan(2.0))
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.1)
    merger._drain_pending()
    assert merger._n_front_only == 1


def test_noise_seed_is_threaded_to_both_reproducible_noise_streams():
    repo = Path(__file__).resolve().parents[3]
    runner = (repo / 'src/ridgeback_autonomy_isaac/sim/isaac' /
              'isaac_runner.py').read_text(encoding='utf-8')
    backend = (repo / 'src/ridgeback_autonomy_isaac/launch' /
               'backend.launch.py').read_text(encoding='utf-8')
    simulation = (repo / 'src/ridgeback_autonomy/launch/includes' /
                  'simulation.launch.py').read_text(encoding='utf-8')
    exploration = (repo / 'src/ridgeback_autonomy/launch' /
                   'ridgeback_exploration.launch.py').read_text(encoding='utf-8')

    assert 'seed=args.noise_seed' in runner
    assert '_random.Random(args.noise_seed + 1)' in runner
    for text in (backend, simulation, exploration):
        assert 'noise_seed' in text


def test_slam_launch_rewrites_range_from_slam_source():
    repo = Path(__file__).resolve().parents[3]
    launch = (repo / 'src/ridgeback_autonomy/launch/includes' /
              'slam.launch.py').read_text(encoding='utf-8')
    assert 'max_laser_range = str(slam_max_laser_range(slam_source))' in launch
    assert "'max_laser_range': max_laser_range" in launch


@pytest.mark.parametrize('source,expected', [('front_only', 10.0), ('merged', 10.3922)])
def test_slam_launch_generated_yaml_contains_numeric_source_range(monkeypatch, source, expected):
    from launch import LaunchContext
    import yaml

    module = _load_repo_module('qualification_slam_launch',
                              'src/ridgeback_autonomy/launch/includes/slam.launch.py')
    actual_rewriter = module.RewrittenYaml
    rewritten = []

    def record_rewriter(**kwargs):
        result = actual_rewriter(**kwargs)
        rewritten.append(result)
        return result

    monkeypatch.setattr(module, 'RewrittenYaml', record_rewriter)
    context = LaunchContext()
    repo = Path(__file__).resolve().parents[3]
    context.launch_configurations.update(
        setup_path=str(repo / 'clearpath'), use_sim_time='true', slam_source=source)
    module.launch_setup(context)
    generated = Path(rewritten[0].perform(context))
    try:
        parameters = yaml.safe_load(generated.read_text())['r100_0001']['slam_toolbox']['ros__parameters']
        original = yaml.safe_load((repo / 'src/ridgeback_autonomy/config/slam_toolbox_params.yaml').read_text())['slam_toolbox']['ros__parameters']
        assert parameters['max_laser_range'] == pytest.approx(expected)
        assert parameters['scan_buffer_maximum_scan_distance'] == original['scan_buffer_maximum_scan_distance']
    finally:
        generated.unlink()


@pytest.mark.parametrize('backend', ['isaac', 'gz', 'hardware'])
def test_seed_is_forwarded_only_to_isaac_backend(backend):
    from launch import LaunchContext
    module = _load_repo_module('qualification_sim_launch',
                              'src/ridgeback_autonomy/launch/includes/simulation.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(backend=backend, noise_seed='17')
    include = module._include_selected_backend(context)[0]
    arguments = dict(include.launch_arguments)
    assert ('noise_seed' in arguments) == (backend == 'isaac')
    if backend == 'isaac':
        assert arguments['noise_seed'].perform(context) == '17'


def test_runner_seed_parser_default_and_override(monkeypatch):
    import sys
    module = _load_repo_module('qualification_runner',
                              'src/ridgeback_autonomy_isaac/sim/isaac/isaac_runner.py')
    monkeypatch.setattr(sys, 'argv', ['isaac_runner'])
    assert module.parse_args().noise_seed == 0
    monkeypatch.setattr(sys, 'argv', ['isaac_runner', '--noise-seed', '17'])
    assert module.parse_args().noise_seed == 17


def test_seed_streams_preserve_default_and_change_with_seed(monkeypatch):
    import ast
    import random
    from types import SimpleNamespace

    monkeypatch.setitem(sys.modules, 'isaacsim.core.experimental.prims',
                        SimpleNamespace(Articulation=lambda path: None))
    rig_module = _load_repo_module('qualification_rig',
                                  'src/ridgeback_autonomy_isaac/sim/isaac/robot_rig.py')
    repo = Path(__file__).resolve().parents[3]
    runner_ast = ast.parse((repo / 'src/ridgeback_autonomy_isaac/sim/isaac/isaac_runner.py').read_text())
    assignment = next(node for node in ast.walk(runner_ast)
                      if isinstance(node, ast.Assign)
                      and any(isinstance(target, ast.Name) and target.id == 'imu_rng'
                              for target in node.targets))
    expression = compile(ast.Expression(assignment.value), '<imu seed>', 'eval')
    streams = []
    for seed in (0, 1, 2):
        odom = rig_module.RidgebackRig('/robot', seed=seed)._rng
        imu = eval(expression, {'_random': random,
                                'args': SimpleNamespace(noise_seed=seed)})
        expected_odom, expected_imu = random.Random(seed), random.Random(seed + 1)
        odom_values = [odom.gauss(0, 1) for _ in range(10)]
        imu_values = [imu.gauss(0, 1) for _ in range(10)]
        assert odom_values == [expected_odom.gauss(0, 1) for _ in range(10)]
        assert imu_values == [expected_imu.gauss(0, 1) for _ in range(10)]
        streams.append((odom_values, imu_values))
    assert streams[0] != streams[1] != streams[2]


@pytest.mark.parametrize('defect', [None, 'nan', 'duplicate', 'missing', 'contention', 'pilot'])
def test_matrix_summary_rejects_incomplete_or_nonfinite_trials(tmp_path, monkeypatch, defect):
    from types import SimpleNamespace
    module = _load_repo_module('qualification_summary', 'tools/isaac/lidar_qualification.py')
    monkeypatch.setattr(module, 'REPO', tmp_path)
    reports = []
    monkeypatch.setattr(module, '_write_report',
                        lambda out, name, report, args: reports.append(report) or out / name)
    records = []
    for noise in (0.0, 1.0):
        for seed in (0, 1, 2):
            for source in ('front_only', 'merged'):
                record = dict(odom_noise=noise, noise_seed=seed,
                              slam_source=source, aborted=False)
                for path in module.METRIC_PATHS.values():
                    current = record
                    for key in path[:-1]:
                        current = current.setdefault(key, {})
                    current[path[-1]] = 1.0
                records.append(record)
    if defect == 'nan':
        records[0]['pose_rmse_m'] = math.nan
    elif defect == 'duplicate':
        records.append(records[0])
    elif defect == 'missing':
        records.pop()
    elif defect == 'contention':
        (tmp_path / 'host-samples.json').write_text(json.dumps([
            {'gpu_processes': {'stdout': '123, isaac\n456, other-isaac'}}]))
    elif defect == 'pilot':
        (tmp_path / 'attempt.json').write_text(json.dumps({'purpose': 'timing_pilot'}))
    for index, record in enumerate(records):
        (tmp_path / f'{index}_metrics.json').write_text(json.dumps(record))
    result = module.summarize(SimpleNamespace(
        metrics=['*_metrics.json'], out=tmp_path / 'summary'))
    assert result == (0 if defect is None else 1)


def test_namespaced_tf_skew_and_maximum_ray_integration(tmp_path, monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setenv('ROS_DOMAIN_ID', '198')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    module = _load_repo_module('qualification_tool', 'tools/isaac/lidar_qualification.py')
    assert module.skew(SimpleNamespace(out=tmp_path / 'skew', launch_arg=[])) == 0


@pytest.mark.parametrize('delta_ns,valid', [(24999998, True), (25000002, True),
                                         (25000010, True), (25000011, False),
                                         (50000000, False), (0, False)])
def test_capture_cadence_allows_only_nanosecond_rounding(delta_ns, valid):
    module = _load_repo_module('qualification_cadence', 'tools/isaac/lidar_qualification.py')
    scans = [_scan(1.0), _scan(1.0)]
    scans[1].header.stamp.nanosec = delta_ns
    assert module._cadence(scans)['regular_40hz'] is valid


def test_isaac_61_internal_rotary_rate_preserves_public_scan_contract():
    repo = Path(__file__).resolve().parents[3]
    isaac_dir = repo / 'src/ridgeback_autonomy_isaac/sim/isaac'
    attributes = json.loads(
        (isaac_dir / 'ust10lx_2d.json').read_text(encoding='utf-8')
    )['attributes']
    public_hz = attributes['omni:sensor:tickRate']
    internal_hz = attributes['omni:sensor:Core:scanRateBaseHz']
    firing_hz = attributes['omni:sensor:Core:patternFiringRateHz']

    assert public_hz == pytest.approx(40.0)
    assert internal_hz == public_hz
    assert firing_hz / internal_hz == pytest.approx(1440.0)

    robot_usd = (isaac_dir / 'usd/robots/ridgeback_r100/ridgeback_r100.usda'
                 ).read_text(encoding='utf-8')
    assert robot_usd.count(
        'omni:sensor:Core:scanRateBaseHz = 40') == 2
    assert robot_usd.count(
        'omni:sensor:Core:patternFiringRateHz = 57600') == 2
    assert attributes['omni:sensor:Core:validStartAzimuthDeg'] == 0
    assert attributes['omni:sensor:Core:validEndAzimuthDeg'] == 360
    assert 'rtx_lidar_l' not in robot_usd
