from __future__ import annotations

from pathlib import Path


def test_public_launch_surface_is_limited_to_two_entrypoints() -> None:
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    top_level_launches = sorted(
        path.name
        for path in launch_dir.glob('*.launch.py')
    )

    assert top_level_launches == [
        'g1_distance_benchmark.launch.py',
        'ridgeback_exploration.launch.py',
    ]


def test_internal_launch_includes_exist_and_are_referenced() -> None:
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    includes_dir = launch_dir / 'includes'

    expected_includes = [
        'explore.launch.py',
        'nav2.launch.py',
        'simulation.launch.py',
        'slam.launch.py',
    ]
    assert sorted(path.name for path in includes_dir.glob('*.launch.py')) == expected_includes

    exploration_text = (launch_dir / 'ridgeback_exploration.launch.py').read_text(encoding='utf-8')
    benchmark_text = (launch_dir / 'g1_distance_benchmark.launch.py').read_text(encoding='utf-8')

    assert 'includes' in exploration_text
    assert 'includes' in benchmark_text


def test_benchmark_default_world_is_allowed_by_clearpath_simulation() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    benchmark_text = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' / 'g1_distance_benchmark.launch.py'
    ).read_text(encoding='utf-8')
    clearpath_simulation_text = (
        repo_root
        / 'src'
        / 'clearpath_simulator'
        / 'clearpath_gz'
        / 'launch'
        / 'simulation.launch.py'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('world', default_value='g1_distance_calibration')" in benchmark_text
    assert "'g1_distance_calibration'" in clearpath_simulation_text


def test_benchmark_launch_uses_new_multi_estimator_interface() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    benchmark_text = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' / 'g1_distance_benchmark.launch.py'
    ).read_text(encoding='utf-8')

    assert "'estimators'" in benchmark_text
    assert "'output_dir'" in benchmark_text
    assert 'rgb,sensor_depth,depth_anything,pointcloud,lidar' in benchmark_text
    assert 'measurement_backend' not in benchmark_text
    assert 'primary_metric' not in benchmark_text
    assert "DeclareLaunchArgument('depth_anything_enabled'" not in benchmark_text
    assert "DeclareLaunchArgument('output_csv'" not in benchmark_text
