from __future__ import annotations

from pathlib import Path


def test_public_launch_surface_is_limited_to_known_entrypoints() -> None:
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    top_level_launches = sorted(
        path.name
        for path in launch_dir.glob('*.launch.py')
    )

    assert top_level_launches == [
        'g1_distance_benchmark.launch.py',
        'manual_mapping.launch.py',
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


def test_exploration_uses_unique_mock_hospital_world() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    exploration_text = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' / 'ridgeback_exploration.launch.py'
    ).read_text(encoding='utf-8')
    clearpath_simulation_text = (
        repo_root
        / 'src'
        / 'clearpath_simulator'
        / 'clearpath_gz'
        / 'launch'
        / 'simulation.launch.py'
    ).read_text(encoding='utf-8')
    mock_hospital_world = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'sim' / 'worlds' / 'mock_hospital.sdf'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('world', default_value='mock_hospital')" in exploration_text
    assert "'mock_hospital'" in clearpath_simulation_text
    assert "'hospital'" not in clearpath_simulation_text
    assert '<world name="mock_hospital">' in mock_hospital_world
    assert not (
        repo_root / 'src' / 'ridgeback_autonomy' / 'sim' / 'worlds' / 'hospital.sdf'
    ).exists()
    assert not (
        repo_root / 'src' / 'ridgeback_autonomy' / 'sim' / 'worlds' / 'detailed_hospital.sdf'
    ).exists()


def test_benchmark_launch_uses_new_multi_estimator_interface() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    benchmark_text = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' / 'g1_distance_benchmark.launch.py'
    ).read_text(encoding='utf-8')

    assert "'estimators'" in benchmark_text
    assert "'output_dir'" in benchmark_text
    assert "'benchmark-results'" in benchmark_text
    assert '/tmp/g1_distance_benchmark_runs' not in benchmark_text
    assert 'rgb,sensor_depth,depth_anything,pointcloud,lidar' in benchmark_text
    assert 'measurement_backend' not in benchmark_text
    assert 'primary_metric' not in benchmark_text
    assert "DeclareLaunchArgument('depth_anything_enabled'" not in benchmark_text
    assert "DeclareLaunchArgument('output_csv'" not in benchmark_text


def test_slam_lifecycle_configure_and_activate_are_event_driven() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    slam_text = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' / 'includes' / 'slam.launch.py'
    ).read_text(encoding='utf-8')

    assert 'Transition.TRANSITION_CONFIGURE' in slam_text
    assert 'Transition.TRANSITION_ACTIVATE' in slam_text
    # Configure is gated on the lifecycle change_state service; activate fires on
    # the configured-state transition -- no fixed timer periods.
    assert 'OnStateTransition' in slam_text
    assert 'change_state' in slam_text
    assert 'period=2.0' not in slam_text
    assert 'period=8.0' not in slam_text
