"""Gazebo bringup: the scene-control server, the optional GUI, and world lookup."""

from __future__ import annotations

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import (
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
    UnsetEnvironmentVariable,
)
import pytest
from ros_gz_sim.actions import GzServer


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_DIR = PACKAGE_ROOT / 'launch'


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def gazebo_launch():
    return _load('gazebo.launch.py')


def _context(tmp_path: Path, headless: str) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update({
        'world': 'target_distance_calibration',
        # No robot.yaml here, so the GUI uses Clearpath's layout unchanged.
        'setup_path': str(tmp_path),
        'headless_rendering': headless,
    })
    return context


def test_worlds_resolve_in_search_order_and_unknown_worlds_fail(gazebo_launch, tmp_path) -> None:
    first, second = tmp_path / 'first', tmp_path / 'second'
    for directory in (first, second):
        directory.mkdir()
        (directory / 'shared.sdf').write_text('<sdf/>')
    (second / 'only_second.sdf').write_text('<sdf/>')

    assert gazebo_launch.resolve_world_sdf('shared', [str(first), str(second)]) == str(
        first / 'shared.sdf')
    assert gazebo_launch.resolve_world_sdf('only_second', [str(first), str(second)]) == str(
        second / 'only_second.sdf')
    with pytest.raises(ValueError, match='missing'):
        gazebo_launch.resolve_world_sdf('missing', [str(first), str(second)])


def test_packaged_benchmark_world_is_found(gazebo_launch) -> None:
    path = gazebo_launch.resolve_world_sdf(
        'target_distance_calibration', gazebo_launch._world_search_dirs())

    assert Path(path).name == 'target_distance_calibration.sdf'


def test_headless_rendering_starts_only_the_server_without_a_display(
        gazebo_launch, tmp_path) -> None:
    actions = gazebo_launch._launch_gazebo(_context(tmp_path, 'true'))

    assert len(actions) == 1 and isinstance(actions[0], GroupAction)
    grouped = [
        entity for entity in actions[0].get_sub_entities()
        if isinstance(entity, (UnsetEnvironmentVariable, GzServer))
    ]
    unset = [
        ''.join(part.perform(LaunchContext()) for part in entity.name)
        for entity in grouped if isinstance(entity, UnsetEnvironmentVariable)
    ]
    assert unset == ['DISPLAY', 'WAYLAND_DISPLAY']
    # The display is removed before the server starts, inside the scope.
    assert isinstance(grouped[-1], GzServer)


def test_normal_rendering_adds_a_gui_client_of_the_same_server(gazebo_launch, tmp_path) -> None:
    actions = gazebo_launch._launch_gazebo(_context(tmp_path, 'false'))

    assert [type(action) for action in actions] == [GzServer, ExecuteProcess]
    command = [
        ''.join(part.perform(LaunchContext()) for part in argument)
        for argument in actions[1].cmd
    ]
    # No shell wrapper: the stop signal must reach the GUI process itself.
    assert command[:3] == ['gz', 'sim', '-g']
    assert '--gui-config' in command
    assert not actions[1].shell


def test_generated_gui_config_is_registered_for_shutdown_removal(gazebo_launch) -> None:
    context = _context(REPO_ROOT / 'clearpath', 'false')
    actions = gazebo_launch._launch_gazebo(context)
    gui_config = [
        ''.join(part.perform(LaunchContext()) for part in argument)
        for argument in actions[1].cmd
    ][-1]

    try:
        assert Path(gui_config).name.startswith('ridgeback_gz_gui_')
        assert Path(gui_config).is_file()
        assert isinstance(actions[-1], RegisterEventHandler)
    finally:
        gazebo_launch._remove_gui_config(None, path=gui_config)

    assert not Path(gui_config).exists()


def test_backend_starts_gazebo_through_the_scene_control_server() -> None:
    backend = (LAUNCH_DIR / 'backend.launch.py').read_text(encoding='utf-8')
    gazebo = (LAUNCH_DIR / 'gazebo.launch.py').read_text(encoding='utf-8')

    assert "'launch', 'gazebo.launch.py'" in backend
    assert 'simulation.launch.py' not in backend
    assert 'GzServer(' in gazebo
    assert "'robot_spawn.launch.py'" in gazebo


def test_clock_bridge_message_dependency_is_direct() -> None:
    manifest = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')

    assert '<exec_depend>rosgraph_msgs</exec_depend>' in manifest


def test_clearpath_patch_no_longer_changes_unused_launch_files() -> None:
    patch = (REPO_ROOT / 'patches' / 'clearpath_gz_customizations.patch').read_text(
        encoding='utf-8')

    assert 'gz_sim.launch.py' not in patch
    assert 'simulation.launch.py' not in patch
