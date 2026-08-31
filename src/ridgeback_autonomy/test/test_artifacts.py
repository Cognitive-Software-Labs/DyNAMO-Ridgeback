from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

from ridgeback_autonomy.benchmarking.paths import (
    default_output_directory,
    subprocess_log_environment,
)
from ridgeback_autonomy.benchmarking.sweep import parse_sweep
from ridgeback_autonomy.benchmarking import target_benchmark_sweep as sweep
from ridgeback_autonomy.benchmarking.sweep_report import collect_comparison_rows


REPO_ROOT = Path(__file__).resolve().parents[3]


def test_benchmark_output_default_is_pure_and_workspace_relative(tmp_path):
    assert default_output_directory(str(tmp_path)) == str(tmp_path / 'artifacts/benchmarks')
    assert not (tmp_path / 'artifacts').exists()


@pytest.mark.parametrize('override', [None, '', 'custom'])
def test_child_ros_logs_stay_with_output_unless_overridden(tmp_path, monkeypatch, override):
    monkeypatch.delenv('ROS_LOG_DIR', raising=False)
    if override is not None:
        monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / override) if override else '')
    monkeypatch.setenv('ROS_DOMAIN_ID', '47')
    before = os.environ.copy()
    output = tmp_path / 'run'
    environment = subprocess_log_environment(str(output))
    expected = tmp_path / 'custom' if override else output / 'logs/ros'
    assert environment['ROS_LOG_DIR'] == str(expected)
    assert expected.is_dir()
    assert environment['ROS_DOMAIN_ID'] == '47'
    assert dict(os.environ) == before


def test_benchmark_launch_uses_shared_output_default(tmp_path, monkeypatch):
    from launch.actions import DeclareLaunchArgument
    from launch import LaunchContext

    path = REPO_ROOT / 'src/ridgeback_autonomy/launch/target_benchmark_config.launch.py'
    spec = importlib.util.spec_from_file_location('artifact_benchmark_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    share = tmp_path / 'install/ridgeback_autonomy/share/ridgeback_autonomy'
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(share))
    arguments = {action.name: action for action in module.generate_launch_description().entities
                 if isinstance(action, DeclareLaunchArgument)}
    output = ''.join(value.perform(LaunchContext()) for value in arguments['output_dir'].default_value)
    assert output == default_output_directory(str(tmp_path))


@pytest.mark.parametrize('override', [False, True])
def test_standalone_runner_default_and_override(tmp_path, monkeypatch, override):
    import rclpy
    from ridgeback_autonomy.benchmarking import target_distance_benchmark_runner_node as runner

    share = tmp_path / 'install/ridgeback_autonomy/share/ridgeback_autonomy'
    monkeypatch.setattr(runner, 'get_package_share_directory', lambda _: str(share))
    scenario = tmp_path / 'scenes.yaml'
    scenario.write_text('scenes:\n  - id: one\n    robots: [{ x: 2.0, y: 0.0 }]\n')
    expected = tmp_path / 'custom output' if override else tmp_path / 'artifacts/benchmarks'
    args = ['--ros-args', '-p', f'scenario:={scenario}', '-p', 'estimators:=pointcloud',
            '-p', 'run_dir_name:=preserved_name']
    if override:
        args += ['-p', f'output_dir:={expected}']
    rclpy.init(args=args)
    node = None
    try:
        node = runner.TargetDistanceBenchmarkRunner()
        assert node.output_dir == str(expected)
        assert node.run_output_dir == str(expected / 'preserved_name')
        assert Path(node.images_dir).is_dir()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('override', [False, True])
def test_sweep_routes_output_and_child_logs_without_changing_commands(tmp_path, monkeypatch, override):
    scenario = tmp_path / 'scenes.yaml'
    scenario.write_text('scenes:\n  - id: one\n    robots: [{ x: 2.0, y: 0.0 }]\n')
    defaults = {'scenario': str(scenario), 'repeats': 1}
    expected = tmp_path / 'custom output' if override else tmp_path / 'artifacts/benchmarks'
    monkeypatch.delenv('ROS_LOG_DIR', raising=False)
    if override:
        defaults['output_dir'] = str(expected)
        monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'custom ros'))
    spec = parse_sweep({'sweep': {'name': 'unit'}, 'defaults': defaults,
                        'configs': [{'name': 'pointcloud', 'estimators': 'pointcloud'}]},
                       source=str(tmp_path / 'sweep.yaml'))
    calls = []

    class FakeProcess:
        returncode = 0

        def __init__(self, command, **kwargs):
            calls.append((command, kwargs))
            if command[3] == 'target_benchmark_config.launch.py':
                arguments = dict(token.split(':=', 1) for token in command[4:])
                run = Path(arguments['output_dir']) / arguments['run_dir_name']
                (run / 'images').mkdir(parents=True)
                (run / 'run.json').write_text(json.dumps({
                    'run': {}, 'parameters': arguments, 'estimators': []}))

        def poll(self):
            return None

    monkeypatch.setattr(sweep.subprocess, 'Popen', FakeProcess)
    monkeypatch.setattr(sweep.time, 'sleep', lambda _: None)
    monkeypatch.setattr(sweep, 'git_provenance', lambda _: {})
    monkeypatch.setattr(sweep, 'stop_process_group', lambda *_: None)
    monkeypatch.setattr(sweep, 'reap_orphan_benchmark_entities', lambda _: [])
    monkeypatch.setattr(sweep, 'sample_real_time_factor', lambda: 1.0)
    monkeypatch.setattr(sweep, '_wait_for_config', lambda *_: ('success', None))
    share = tmp_path / 'install/ridgeback_autonomy/share/ridgeback_autonomy'
    sweep_dir, success = sweep.run_sweep(spec, spec.configs, package_share=str(share))
    assert success
    assert Path(sweep_dir).parent == expected
    assert len(calls) == 2
    assert calls[0][0] == sweep._environment_command(spec)
    assert calls[1][0] == sweep._config_command(sweep._resolved_arguments(spec, spec.configs[0], sweep_dir))
    for index, role in enumerate(('environment', 'pointcloud')):
        expected_logs = tmp_path / 'custom ros' if override else Path(sweep_dir) / role / 'logs/ros'
        assert calls[index][1]['env']['ROS_LOG_DIR'] == str(expected_logs)
        assert calls[index][1]['start_new_session'] is True
        assert expected_logs.is_dir()
    assert not (tmp_path / 'benchmark-results').exists()


def test_relocated_sweep_keeps_resume_and_report_inputs(tmp_path, monkeypatch):
    scenario = tmp_path / 'scenes.yaml'
    scenario.write_text('scenes:\n  - id: one\n    robots: [{ x: 2.0, y: 0.0 }]\n')
    spec = parse_sweep({
        'sweep': {'name': 'unit'},
        'defaults': {'scenario': str(scenario), 'repeats': 1},
        'configs': [{'name': 'pointcloud', 'estimators': 'pointcloud'}],
    }, source=str(tmp_path / 'sweep.yaml'))
    old = tmp_path / 'benchmark-results/20260831_120000_unit'
    run = old / 'pointcloud'
    run.mkdir(parents=True)
    # Recorded absolute paths are provenance; moving must not rewrite them.
    document = {'run': {'label': 'pointcloud'}, 'parameters': {'output_dir': str(old)},
                'estimators': [{'key': 'pointcloud', 'mean_abs_error_m': 0.2}]}
    (run / 'run.json').write_text(json.dumps(document))
    original_bytes = (run / 'run.json').read_bytes()
    monkeypatch.setattr(sweep, 'git_provenance', lambda _: {})
    manifest = sweep._new_manifest(spec, spec.configs, str(old), str(tmp_path))
    manifest['configs'][0]['status'] = 'success'
    sweep._write_manifest(str(old), manifest)
    before_rows = collect_comparison_rows(manifest, str(old))

    new_root = Path(default_output_directory(str(tmp_path)))
    new_root.parent.mkdir()
    (tmp_path / 'benchmark-results').rename(new_root)
    new = new_root / old.name
    assert sweep._find_resumable_sweep(str(new_root), spec, spec.configs) == str(new)
    assert collect_comparison_rows(manifest, str(new)) == before_rows
    assert sweep._valid_run_document(str(new / 'pointcloud/run.json')) == document
    assert (new / 'pointcloud/run.json').read_bytes() == original_bytes
    resumed = sweep._resume_manifest(manifest, spec, spec.configs, str(new))
    assert resumed['configs'][0]['arguments']['output_dir'] == str(new)
    assert resumed['configs'][0]['output_path'] == 'pointcloud'


@pytest.fixture
def script_workspace(tmp_path, monkeypatch):
    """Exercise real shell control flow with inert ROS setup/cleanup/commands."""
    (tmp_path / 'install').mkdir()
    (tmp_path / 'install/setup.bash').write_text('true\n')
    (tmp_path / 'ros_setup.bash').write_text('true\n')
    (tmp_path / 'cleanup.sh').write_text('exit 0\n')
    for name in ('start_exploration.sh', 'build_and_start_expl.sh'):
        script = (REPO_ROOT / name).read_text().replace(
            'source /opt/ros/jazzy/setup.bash', 'source "$SCRIPT_DIR/ros_setup.bash"')
        (tmp_path / name).write_text(script)
    commands = tmp_path / 'bin'
    commands.mkdir()
    stubs = {
        'ros2': '#!/bin/bash\nprintf "ros-log-dir=%s\\n" "$ROS_LOG_DIR"\nprintf "arg=%s\\n" "$@"\nexit 7\n',
        'date': '#!/bin/bash\nprintf "2026-08-31_12-00-00\\n"\n',
        'colcon': '#!/bin/bash\nprintf "colcon-arg=%s\\n" "$@"\n',
    }
    for name, content in stubs.items():
        path = commands / name
        path.write_text(content)
        path.chmod(0o755)
    monkeypatch.setenv('PATH', str(commands) + os.pathsep + os.environ['PATH'])
    for name in ('LOG_DIR', 'ROS_LOG_DIR', 'COLCON_LOG_PATH', 'FASTRTPS_NO_SHM', 'EXPLORER'):
        monkeypatch.delenv(name, raising=False)
    return tmp_path


@pytest.mark.parametrize('custom_output', [False, True])
def test_exploration_logs_and_overrides_are_non_clobbering(script_workspace, monkeypatch, custom_output):
    root = script_workspace
    output = root / 'artifacts/exploration'
    if custom_output:
        output = root / 'custom console logs'
        monkeypatch.setenv('LOG_DIR', str(output))
        monkeypatch.setenv('ROS_LOG_DIR', str(root / 'custom ros logs'))
    for _ in range(2):
        result = subprocess.run(
            ['bash', str(root / 'start_exploration.sh'), 'office', 'custom', 'estimators:=pointcloud'],
            cwd=root.parent, capture_output=True, text=True, timeout=10)
        assert result.returncode == 7  # tee must not hide launch failures.
        assert 'arg=world:=office' in result.stdout
        assert 'arg=explorer:=custom' in result.stdout
        assert 'arg=estimators:=pointcloud' in result.stdout
    runs = sorted(output.iterdir())
    assert len(runs) == 2
    for run in runs:
        ros_logs = root / 'custom ros logs' if custom_output else run / 'ros'
        assert ros_logs.is_dir()
        assert f'ros-log-dir={ros_logs}' in (run / 'console.log').read_text()
    assert not (root / 'logs').exists()


@pytest.mark.parametrize('override', [False, True])
def test_build_helper_sets_log_base_outside_workspace(script_workspace, monkeypatch, override):
    root = script_workspace
    expected = root / 'custom colcon logs' if override else root / 'artifacts/colcon'
    if override:
        monkeypatch.setenv('COLCON_LOG_PATH', str(expected))
    result = subprocess.run(
        ['bash', str(root / 'build_and_start_expl.sh'), 'office', 'custom'],
        cwd=root.parent, capture_output=True, text=True, timeout=10)
    assert result.returncode == 7
    assert f'colcon-arg=--log-base\ncolcon-arg={expected}\ncolcon-arg=build' in result.stdout


@pytest.mark.parametrize('override', [False, True])
def test_colcon_discovers_workspace_log_default_and_allows_cli_override(tmp_path, override):
    pytest.importorskip('colcon_defaults')
    colcon = shutil.which('colcon')
    if colcon is None:
        pytest.skip('colcon CLI is unavailable')
    shutil.copyfile(REPO_ROOT / 'colcon_defaults.yaml', tmp_path / 'colcon_defaults.yaml')
    source = tmp_path / 'src'
    source.mkdir()
    expected = tmp_path / 'custom logs' if override else tmp_path / 'artifacts/colcon'
    command = [colcon]
    if override:
        command += ['--log-base', str(expected)]
    command += ['list', '--base-paths', str(source)]
    result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    assert expected.is_dir()
    assert not (tmp_path / 'log').exists()
