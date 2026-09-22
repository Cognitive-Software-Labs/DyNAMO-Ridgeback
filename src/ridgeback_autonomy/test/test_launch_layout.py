from __future__ import annotations

from pathlib import Path
import re

import yaml

from ridgeback_autonomy.benchmarking.replay import REPLAY_CAPTURE_BATCHES_DEFAULT
from ridgeback_localization.estimator_registry import PUBLIC_ESTIMATOR_ORDER
from ridgeback_autonomy.localization_launch import CONFIG_LAUNCH_ARGUMENT_NAMES


def _package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _exploration_text() -> str:
    return (
        _package_root() / 'launch' / 'ridgeback_exploration.launch.py'
    ).read_text(encoding='utf-8')


def _exploration_rviz_path() -> Path:
    return _package_root() / 'sim' / 'rviz' / 'exploration.rviz'


def _slam_parameters() -> dict[str, object]:
    config_path = _package_root() / 'config' / 'slam_toolbox_params.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    return config['slam_toolbox']['ros__parameters']


def test_public_launch_surface_is_limited_to_known_entrypoints() -> None:
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    top_level_launches = sorted(
        path.name
        for path in launch_dir.glob('*.launch.py')
    )

    assert top_level_launches == [
        'localization_observer.launch.py',
        'manual_mapping.launch.py',
        'ridgeback_exploration.launch.py',
        'target_benchmark_config.launch.py',
        'target_benchmark_env.launch.py',
        'target_distance_benchmark.launch.py',
    ]


def test_manual_mapping_does_not_force_a_dds_profile() -> None:
    launch_text = (
        _package_root() / 'launch' / 'manual_mapping.launch.py'
    ).read_text(encoding='utf-8')

    assert 'FASTRTPS' not in launch_text
    assert 'fastrtps' not in launch_text
    assert 'SetEnvironmentVariable' not in launch_text


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
    benchmark_env_text = (launch_dir / 'target_benchmark_env.launch.py').read_text(encoding='utf-8')

    assert 'includes' in exploration_text
    assert 'includes' in benchmark_env_text


def test_benchmark_default_world_is_allowed_by_clearpath_simulation() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    benchmark_env_text = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' / 'target_benchmark_env.launch.py'
    ).read_text(encoding='utf-8')
    clearpath_simulation_text = (
        repo_root
        / 'src'
        / 'clearpath_simulator'
        / 'clearpath_gz'
        / 'launch'
        / 'simulation.launch.py'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('world', default_value='target_distance_calibration')" in benchmark_env_text
    assert "'target_distance_calibration'" in clearpath_simulation_text


def test_benchmark_forwards_gazebo_gui_choice_to_simulation() -> None:
    benchmark_env_text = (
        _package_root() / 'launch' / 'target_benchmark_env.launch.py'
    ).read_text(encoding='utf-8')
    gazebo_adapter_text = (
        _package_root().parent / 'ridgeback_autonomy_gz' / 'launch' / 'backend.launch.py'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument(\n            'gz_gui'" in benchmark_env_text
    assert "'gz_gui': gz_gui" in benchmark_env_text
    assert "'headless_rendering': PythonExpression(" in gazebo_adapter_text
    assert "'false' if '" in gazebo_adapter_text


def test_exploration_uses_unique_initial_test_world() -> None:
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
    initial_test_world_world = (
        repo_root / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'worlds'
        / 'initial_test_world.sdf'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('world', default_value='initial_test_world')" in exploration_text
    assert "'initial_test_world'" in clearpath_simulation_text
    assert "'hospital'" not in clearpath_simulation_text
    assert '<world name="initial_test_world">' in initial_test_world_world
    assert not (
        repo_root / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'worlds' / 'hospital.sdf'
    ).exists()


def test_gazebo_world_names_translate_at_the_adapter_boundary() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    adapter_text = (
        repo_root / 'src' / 'ridgeback_autonomy_gz' / 'launch' / 'backend.launch.py'
    ).read_text(encoding='utf-8')

    assert "'depot': 'warehouse'" in adapter_text
    assert "'coworking_space': 'office'" in adapter_text
    assert "'world': clearpath_world" in adapter_text
    assert 'raise ValueError' in adapter_text

    clearpath_worlds = (
        repo_root / 'src' / 'clearpath_simulator' / 'clearpath_gz' / 'worlds'
    )
    assert (clearpath_worlds / 'warehouse.sdf').exists()
    assert (clearpath_worlds / 'office.sdf').exists()


def test_public_gazebo_maps_use_the_noncolliding_world_names() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    maps = repo_root / 'src' / 'ridgeback_autonomy' / 'sim' / 'ground_truth_maps'

    for world in ('initial_test_world', 'depot', 'coworking_space'):
        assert (maps / f'{world}.pgm').exists()
        assert (maps / f'{world}.png').exists()
        assert (maps / f'{world}.yaml').read_text(encoding='utf-8').startswith(
            f'image: {world}.pgm')

    historical = maps / 'historical'
    assert not (historical / 'mock_hospital.pgm').exists()
    assert not (historical / 'warehouse.pgm').exists()
    assert not (historical / 'office.pgm').exists()
    assert not (
        repo_root / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'worlds'
        / 'detailed_hospital.sdf'
    ).exists()


def test_benchmark_launch_uses_new_multi_estimator_interface() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    benchmark_text = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' / 'target_benchmark_config.launch.py'
    ).read_text(encoding='utf-8')

    assert "'estimators'" in benchmark_text
    assert "'output_dir'" in benchmark_text
    assert 'benchmark_output_dir = default_output_directory(workspace_root)' in benchmark_text
    assert '/tmp/target_distance_benchmark_runs' not in benchmark_text
    assert "DeclareLaunchArgument('estimators', default_value='all')" in benchmark_text
    assert 'measurement_backend' not in benchmark_text
    assert 'primary_metric' not in benchmark_text
    assert "DeclareLaunchArgument('depth_anything_enabled'" not in benchmark_text
    assert "DeclareLaunchArgument('output_csv'" not in benchmark_text
    assert 'overlay_window' not in benchmark_text
    assert 'show_window' not in benchmark_text
    assert REPLAY_CAPTURE_BATCHES_DEFAULT == 5
    assert "default_value=str(REPLAY_CAPTURE_BATCHES_DEFAULT)" in benchmark_text
    # The mask-gate axis: declared once, forwarded to the mask node, the runner
    # (output names must match the node's gate), and the overlay (its mask panel
    # follows the gate).
    assert "'mask_gate'," in benchmark_text
    assert benchmark_text.count("LaunchConfiguration('mask_gate')") == 3
    # One estimators list, split per stack: each measurement node is handed only
    # the rows it owns, so a mask row can be selected as freely as the
    # pointcloud one.
    assert "enabled_estimators=','.join(selected_pointcloud)" in benchmark_text
    assert "enabled_estimators=','.join(selected_mask)" in benchmark_text


def test_exploration_runs_the_same_four_estimator_rows_as_the_benchmark() -> None:
    exploration_text = _exploration_text()

    # Nothing pinned to one row any more: exploration selects the same way the
    # benchmark does, and the shared factories are what stop the two stacks
    # measuring off different topics.
    assert "'estimators': 'pointcloud'" not in exploration_text
    assert "DeclareLaunchArgument('estimators', default_value=_backend_default(" in (
        exploration_text)
    assert "'all'," in exploration_text
    assert "'projective_ranging,euclidean_reconstruction'" in exploration_text
    assert 'pointcloud_measurement_node(' in exploration_text
    assert 'mask_measurement_node(' in exploration_text
    assert 'parse_estimators' in exploration_text

    # Rings and the distance HUD are on by default, or the four-way comparison
    # this stack now produces has nothing drawing it.
    assert "DeclareLaunchArgument('estimate_viz', default_value='true'" in exploration_text

    # The mask node's own base_frame default is the bare string 'base_link',
    # which resolves to nothing under a namespace and fails polar profiling's
    # scan->base lookup silently. Every caller has to pass it.
    assert "base_frame = LaunchConfiguration('base_frame').perform(context).strip()" in exploration_text


def test_exploration_gives_the_distance_hud_its_own_aggregator() -> None:
    # hud_node renders through QStaticText: one rich-text panel switches the
    # whole overlay to rich text, and the velocity/coverage/localization panels
    # align their columns with runs of spaces, which rich text collapses.
    exploration_text = _exploration_text()

    assert "name='hud_target_node'" in exploration_text
    assert 'marker_topic=HUD_TARGET_MARKER_TOPIC' in exploration_text
    # The rich-text panel and the alignment come from the shared factory, so the
    # benchmark's aggregator and this one cannot drift apart on that contract.
    assert 'distance_hud_node(' in exploration_text
    # The original aggregator keeps its plain panels and its own topic, and is
    # still written out here because nothing else runs one like it.
    assert "'panels': ['hud/velocity', 'hud/coverage', 'hud/localization']" in exploration_text
    assert exploration_text.count("executable='hud_node'") == 1


def test_exploration_rviz_configures_one_checkbox_per_registered_estimator() -> None:
    # A MarkerArray display renders one checkbox per marker namespace, and the
    # ring and its centre dot share one, so unticking a row drops both. Writing
    # the names into the config pins them: rename an estimator and this block
    # would silently keep a checkbox for a namespace nothing publishes while the
    # new row arrived unconfigured.
    config = yaml.safe_load(_exploration_rviz_path().read_text(encoding='utf-8'))
    displays = config['Visualization Manager']['Displays']
    estimates = next(d for d in displays if d['Name'] == 'Target Estimates')

    assert set(estimates['Namespaces']) == {
        f'target_estimates/{estimator}' for estimator in PUBLIC_ESTIMATOR_ORDER
    }
    assert all(estimates['Namespaces'].values())


def test_exploration_rviz_shows_the_overlay_and_the_distance_hud() -> None:
    config = yaml.safe_load(_exploration_rviz_path().read_text(encoding='utf-8'))
    displays = config['Visualization Manager']['Displays']
    by_name = {display['Name']: display for display in displays}

    # The overlay display has existed here all along with Enabled: false, so the
    # camera panels have never been visible in exploration.
    assert by_name['Perception overlay']['Enabled'] is True
    # A pane, so it needs a Window Geometry entry; the TextOverlays are drawn on
    # the 3D view and do not.
    assert config['Window Geometry']['Perception overlay'] == {'collapsed': False}

    assert by_name['Target HUD']['Topic']['Value'] == '/r100_0001/hud_target_overlay'
    assert by_name['HUD']['Topic']['Value'] == '/r100_0001/hud_overlay'


def test_camera_overlay_is_rviz_only() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    overlay_text = (
        repo_root
        / 'src'
        / 'ridgeback_localization'
        / 'ridgeback_localization'
        / 'overlay_node.py'
    ).read_text(encoding='utf-8')

    assert 'build_bgr8_image_message' in overlay_text
    assert 'self.overlay_pub.publish' in overlay_text
    for removed_symbol in (
        'show_window',
        'window_name',
        'render_fps',
        'render_callback',
        'cv2.namedWindow',
        'cv2.imshow',
        'cv2.waitKey',
        'cv2.destroyAllWindows',
    ):
        assert removed_symbol not in overlay_text


def test_benchmark_layers_declare_identical_shared_arguments() -> None:
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    env_text = (launch_dir / 'target_benchmark_env.launch.py').read_text(encoding='utf-8')
    config_text = (launch_dir / 'target_benchmark_config.launch.py').read_text(encoding='utf-8')

    shared_declarations = [
        "DeclareLaunchArgument('namespace', default_value='r100_0001')",
        "DeclareLaunchArgument('use_sim_time', default_value='true')",
        "DeclareLaunchArgument('world', default_value='target_distance_calibration')",
    ]
    for declaration in shared_declarations:
        assert env_text.count(declaration) == 1
        assert config_text.count(declaration) == 1

    # Both process layers source the persistent detector and restartable
    # measurement stack from the same pure mapping rather than duplicating a
    # simulation topic literal -- and that mapping is now resolved once in
    # launch_common, so no launch file calls resolve_camera_inputs itself.
    for text in (env_text, config_text, _exploration_text()):
        assert 'SIMULATION_CAMERA_INPUTS' in text
        assert 'resolve_camera_inputs(SIMULATION_BACKEND)' not in text
    assert 'default_value=SIMULATION_CAMERA_INPUTS.color_image_topic' in env_text
    assert 'default_value=SIMULATION_CAMERA_INPUTS.color_image_topic' in config_text

    declared_config_names = frozenset(re.findall(
        r"DeclareLaunchArgument\(\s*'([^']+)'",
        config_text,
    ))
    assert declared_config_names == CONFIG_LAUNCH_ARGUMENT_NAMES


def test_benchmark_wrapper_only_composes_the_two_layers() -> None:
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    wrapper_text = (launch_dir / 'target_distance_benchmark.launch.py').read_text(encoding='utf-8')

    assert 'target_benchmark_env.launch.py' in wrapper_text
    assert 'target_benchmark_config.launch.py' in wrapper_text
    assert wrapper_text.count('DeclareLaunchArgument(') == 1
    assert "'shutdown_on_complete'" in wrapper_text


def test_headless_rendering_is_optional_and_owned_by_the_environment() -> None:
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    env_text = (launch_dir / 'target_benchmark_env.launch.py').read_text(encoding='utf-8')
    config_text = (launch_dir / 'target_benchmark_config.launch.py').read_text(encoding='utf-8')
    gazebo_adapter_text = (
        _package_root().parent / 'ridgeback_autonomy_gz' / 'launch' / 'backend.launch.py'
    ).read_text(encoding='utf-8')

    assert "'headless_rendering',\n            default_value='false'" in env_text
    assert "'headless_rendering': headless_rendering" in env_text
    assert "'headless_rendering'" not in config_text
    assert "DeclareLaunchArgument('headless_rendering', default_value='false')" in (
        gazebo_adapter_text)


def test_camera_profile_reaches_both_simulators_but_not_hardware() -> None:
    source_root = _package_root().parent
    dispatcher = (_package_root() / 'launch/includes/simulation.launch.py').read_text(
        encoding='utf-8')
    exploration = _exploration_text()
    gazebo = (source_root / 'ridgeback_autonomy_gz/launch/backend.launch.py').read_text(
        encoding='utf-8')
    isaac = (source_root / 'ridgeback_autonomy_isaac/launch/backend.launch.py').read_text(
        encoding='utf-8')

    assert "arguments.pop('camera_profile')" in dispatcher
    assert "'camera_profile': LaunchConfiguration('camera_profile')" in exploration
    assert "LaunchConfiguration('camera_profile')" in gazebo
    assert "'sim_camera_horizontal_fov'" in gazebo
    assert "'--camera-profile', camera_profile" in isaac


def test_depth_fidelity_reaches_only_isaac() -> None:
    source_root = _package_root().parent
    dispatcher = (_package_root() / 'launch/includes/simulation.launch.py').read_text(
        encoding='utf-8')
    exploration = _exploration_text()
    gazebo = (source_root / 'ridgeback_autonomy_gz/launch/backend.launch.py').read_text(
        encoding='utf-8')
    hardware = (
        source_root / 'ridgeback_autonomy_hardware/launch/backend.launch.py'
    ).read_text(encoding='utf-8')
    isaac = (source_root / 'ridgeback_autonomy_isaac/launch/backend.launch.py').read_text(
        encoding='utf-8')

    assert "if backend != 'isaac':" in dispatcher
    assert "arguments.pop('depth_fidelity')" in dispatcher
    assert "'depth_fidelity': LaunchConfiguration('depth_fidelity')" in exploration
    assert "'--depth-fidelity', depth_fidelity" in isaac
    assert 'depth_fidelity' not in gazebo
    assert 'depth_fidelity' not in hardware


def test_backend_packages_keep_simulator_dependencies_out_of_core() -> None:
    source_root = _package_root().parent
    core_cmake = (_package_root() / 'CMakeLists.txt').read_text(encoding='utf-8')
    core_manifest = (_package_root() / 'package.xml').read_text(encoding='utf-8')
    gz_cmake = (source_root / 'ridgeback_autonomy_gz/CMakeLists.txt').read_text(
        encoding='utf-8')
    gz_manifest = (source_root / 'ridgeback_autonomy_gz/package.xml').read_text(
        encoding='utf-8')
    dispatcher = (_package_root() / 'launch/includes/simulation.launch.py').read_text(
        encoding='utf-8')
    benchmark_runner = (
        _package_root()
        / 'ridgeback_autonomy/benchmarking/target_distance_benchmark_runner_node.py'
    ).read_text(encoding='utf-8')

    for simulator_dependency in ('clearpath_gz', 'gz-gui', 'gz-plugin', 'Qt5', 'SpawnG1'):
        assert simulator_dependency not in core_cmake + core_manifest
    for simulator_dependency in ('clearpath_gz', 'gz-gui', 'gz-plugin', 'Qt5', 'SpawnG1'):
        assert simulator_dependency in gz_cmake + gz_manifest
    assert "'hardware': 'ridgeback_autonomy_hardware'" in dispatcher
    assert "'isaac': 'ridgeback_autonomy_isaac'" in dispatcher
    assert "'gz': 'ridgeback_autonomy_gz'" in dispatcher
    assert "get_package_share_directory('clearpath_gz')" not in dispatcher
    assert 'ridgeback_autonomy_gz' not in benchmark_runner


def test_dependency_profiles_keep_gazebo_out_of_the_common_closure() -> None:
    repo_root = _package_root().parents[1]
    core = yaml.safe_load(
        (repo_root / 'dependencies/core.repos').read_text(encoding='utf-8'))[
            'repositories']
    gazebo = yaml.safe_load(
        (repo_root / 'dependencies/gz.repos').read_text(encoding='utf-8'))[
            'repositories']
    checker = (repo_root / 'tools/check_dependencies').read_text(encoding='utf-8')

    assert set(core).isdisjoint(gazebo)
    assert set(gazebo) == {'src/clearpath_simulator'}
    assert 'src/clearpath_simulator' not in core
    assert 'core|gz|all' in checker


def test_hardware_backend_is_attach_only_and_uses_real_sensor_defaults() -> None:
    source_root = _package_root().parent
    hardware = (source_root / 'ridgeback_autonomy_hardware/launch/backend.launch.py').read_text(
        encoding='utf-8')
    exploration = _exploration_text()
    start_script = (source_root.parent / 'start_exploration.sh').read_text(encoding='utf-8')

    assert "'start_platform', default_value='false'" in hardware
    assert "'use_sim_time': 'false'" in hardware
    assert "'backend', default_value=legacy_sim" in exploration
    assert 'REALSENSE_CAMERA_INPUTS' in exploration
    assert "'projective_ranging,euclidean_reconstruction'" in exploration
    assert "'autonomous_motion_enabled'" in exploration
    assert "default_value=_backend_default('true', 'false')" in exploration
    assert 'condition=launch.conditions.IfCondition(\n                        autonomous_motion_enabled)' in (
        exploration)
    assert 'if [ "$BACKEND" = hardware ]' in start_script
    assert 'preserving existing ROS and Clearpath processes' in start_script


def test_every_entrypoint_supplies_the_cyclonedds_configuration() -> None:
    """Exploration aborts on Cyclone's default participant-index ceiling.

    It starts 51 processes; on the default setting slam_toolbox and the whole
    Nav2 stack died with "failed to find a free participant index". Each
    entrypoint is launched as its own process -- the sweep supervisor starts
    the environment and per-config layers separately -- so each has to supply
    the configuration itself rather than inherit it from a sibling.
    """

    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    for name in (
        'ridgeback_exploration.launch.py',
        'target_benchmark_env.launch.py',
        'target_benchmark_config.launch.py',
    ):
        text = (launch_dir / name).read_text(encoding='utf-8')
        assert 'cyclonedds_actions(pkg_this)' in text, name


def test_cyclonedds_configuration_yields_to_an_operator_setting(tmp_path, monkeypatch) -> None:
    from ridgeback_autonomy.localization_launch import cyclonedds_actions

    share = tmp_path / 'share'
    (share / 'config').mkdir(parents=True)
    config = share / 'config' / 'cyclonedds.xml'
    config.write_text('<CycloneDDS/>', encoding='utf-8')

    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)
    actions = cyclonedds_actions(str(share))
    assert len(actions) == 1

    # A configuration someone chose deliberately must not be discarded.
    monkeypatch.setenv('CYCLONEDDS_URI', '/etc/mine.xml')
    assert cyclonedds_actions(str(share)) == []

    # Nor may a missing file leave CYCLONEDDS_URI pointing at nothing.
    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)
    assert cyclonedds_actions(str(tmp_path / 'absent')) == []


def test_benchmark_environment_owns_detector_and_config_is_readiness_gated() -> None:
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    env_text = (launch_dir / 'target_benchmark_env.launch.py').read_text(encoding='utf-8')
    config_text = (launch_dir / 'target_benchmark_config.launch.py').read_text(encoding='utf-8')

    assert "executable='target_detector_node'" in env_text
    assert "executable='target_detector_node'" not in config_text
    assert "'launch_wait'" in config_text
    assert 'RAW_DETECTIONS_TOPIC' in config_text
    assert 'gate_benchmark_environment_ready' in config_text
    assert 'OnProcessExit' in config_text
    assert "default_value='false'" in config_text
    assert "Shutdown(reason='benchmark runner exited')" in config_text


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


def test_scan_merger_node_remaps_tf_into_the_namespace() -> None:
    """tf2_ros.TransformListener subscribes to the ABSOLUTE '/tf'; the node's
    namespace does not move it. Without the remap the merger's TF buffer stays
    permanently empty (the stack publishes into '<ns>/tf'), every odom lookup
    fails, and each rear scan is silently dropped instead of being
    motion-compensated -- a front-only scan wearing a merged scan's name."""
    repo_root = Path(__file__).resolve().parents[3]
    slam_text = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' / 'includes' / 'slam.launch.py'
    ).read_text(encoding='utf-8')

    merger_block = slam_text.split("executable='scan_merger_node'", 1)
    assert len(merger_block) == 2, 'scan_merger_node is no longer launched from slam.launch.py'
    assert "('/tf', 'tf')" in merger_block[1].split('))', 1)[0]
    assert "('/tf_static', 'tf_static')" in merger_block[1].split('))', 1)[0]


def test_async_slam_does_not_publish_scan_lag_into_nav2_tf() -> None:
    params = _slam_parameters()

    # MPPI transforms the map-frame goal at the current robot-pose timestamp.
    # Keep map->odom current and do not retain stale scans in async mapping.
    assert params['restamp_tf'] is True
    assert params['scan_queue_size'] == 1
