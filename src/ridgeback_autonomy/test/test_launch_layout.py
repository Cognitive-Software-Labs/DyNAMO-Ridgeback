from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import re

import pytest
import yaml

from ridgeback_autonomy.benchmarking import backends
from ridgeback_autonomy.benchmarking.backends import BACKEND_SETTING_NAMES
from ridgeback_autonomy.benchmarking.replay import REPLAY_CAPTURE_BATCHES_DEFAULT
from ridgeback_localization.estimator_registry import PUBLIC_ESTIMATOR_ORDER
from ridgeback_autonomy.localization_launch import (
    CONFIG_LAUNCH_ARGUMENT_NAMES,
    ENV_LAYER_CONFIG_KEYS,
)


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


def test_benchmark_default_world_is_resolved_by_the_gazebo_adapter() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    benchmark_env_text = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' / 'target_benchmark_env.launch.py'
    ).read_text(encoding='utf-8')
    gazebo_launch_text = (
        repo_root / 'src' / 'ridgeback_autonomy_gz' / 'launch' / 'gazebo.launch.py'
    ).read_text(encoding='utf-8')
    benchmark_world = (
        repo_root / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'worlds'
        / 'target_distance_calibration.sdf'
    )

    assert "DeclareLaunchArgument('world', default_value='target_distance_calibration')" in benchmark_env_text
    assert "get_package_share_directory('ridgeback_autonomy_gz'), 'sim', 'worlds'" in gazebo_launch_text
    assert benchmark_world.is_file()


def _load_launch(relative_path: str):
    path = _package_root() / 'launch' / relative_path
    spec = importlib.util.spec_from_file_location(
        'layout_' + re.sub(r'\W', '_', relative_path), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _text(context, value) -> str:
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _declared_context(*modules, **configurations):
    """A launch context holding the launch files' declared defaults, then overrides.

    As in one launch process, the first module to declare an argument sets it.
    """

    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    context = LaunchContext()
    context.launch_configurations.update(configurations)
    for module in modules:
        for action in module.generate_launch_description().entities:
            if (isinstance(action, DeclareLaunchArgument)
                    and action.name not in context.launch_configurations):
                context.launch_configurations[action.name] = _text(
                    context, action.default_value)
    return context


@pytest.fixture
def adapter_lookups(monkeypatch):
    """Serve adapter catalogs from the source tree and record every lookup."""

    looked_up = []

    def share(package):
        looked_up.append(package)
        return str(_package_root().parent / package)

    monkeypatch.setattr(backends, '_default_package_share', share)
    return looked_up


def _include_arguments(context, include) -> dict[str, str]:
    return {name: _text(context, value) for name, value in include.launch_arguments}


_DISPATCHER_ARGUMENT_NAMES = {
    'gz': {
        'setup_path', 'world', 'namespace', 'clearpath_rviz', 'gz_gui',
        'headless_rendering', 'camera_profile',
    },
    'isaac': {
        'setup_path', 'world', 'namespace', 'rtf', 'headless', 'livestream',
        'odom_noise', 'noise_seed', 'camera', 'sim_mode', 'sensor_hz',
        'camera_profile', 'depth_fidelity',
    },
    'hardware': {'setup_path', 'namespace', 'start_platform'},
}


@pytest.mark.parametrize(
    ('backend', 'adapter_package'),
    [
        ('gz', 'ridgeback_autonomy_gz'),
        ('isaac', 'ridgeback_autonomy_isaac'),
        ('hardware', 'ridgeback_autonomy_hardware'),
    ],
)
def test_simulation_dispatcher_forwards_exact_adapter_arguments(
    monkeypatch, backend, adapter_package,
) -> None:
    dispatcher = _load_launch('includes/simulation.launch.py')
    looked_up = []

    def share(package):
        looked_up.append(package)
        return str(_package_root().parent / package)

    monkeypatch.setattr(dispatcher, 'get_package_share_directory', share)
    context = _declared_context(
        dispatcher, backend=backend, start_hardware_platform='true')

    [include] = dispatcher._include_selected_backend(context)
    arguments = _include_arguments(context, include)

    assert set(arguments) == _DISPATCHER_ARGUMENT_NAMES[backend]
    assert looked_up == [adapter_package]
    if backend == 'hardware':
        assert arguments['start_platform'] == 'true'


@pytest.mark.parametrize(
    ('backend', 'native_world', 'adapter', 'settings'),
    [
        ('gz', 'target_distance_calibration', 'ridgeback_autonomy_gz',
         {'gz_gui': 'true', 'headless_rendering': 'false'}),
        ('isaac', 'g1_distance_calibration', 'ridgeback_autonomy_isaac',
         {'headless': 'true', 'rtf': '1.0', 'sim_mode': 'realtime',
          'depth_fidelity': 'ideal'}),
    ],
)
def test_benchmark_environment_starts_only_the_selected_simulator(
    adapter_lookups, backend, native_world, adapter, settings,
) -> None:
    module = _load_launch('target_benchmark_env.launch.py')
    context = _declared_context(module, backend=backend)

    include = module.build_simulation(context)[-1]
    arguments = _include_arguments(context, include)

    assert arguments['backend'] == backend
    # The adapter receives the catalog's native world; the operator never
    # spells it.
    assert arguments['world'] == native_world
    assert {name: arguments[name] for name in arguments if name in BACKEND_SETTING_NAMES} == (
        settings)
    assert arguments['camera_profile'] == '640x480'
    assert adapter_lookups == [adapter]


def test_benchmark_environment_defaults_to_gazebo(adapter_lookups) -> None:
    module = _load_launch('target_benchmark_env.launch.py')
    context = _declared_context(module)

    arguments = _include_arguments(context, module.build_simulation(context)[-1])

    assert arguments['backend'] == 'gz'
    assert adapter_lookups == ['ridgeback_autonomy_gz']


def test_benchmark_environment_forwards_the_gazebo_gui_and_rendering_choice(
    adapter_lookups,
) -> None:
    module = _load_launch('target_benchmark_env.launch.py')
    context = _declared_context(module, gz_gui='false', headless_rendering='True')

    arguments = _include_arguments(context, module.build_simulation(context)[-1])

    assert arguments['gz_gui'] == 'false'
    assert arguments['headless_rendering'] == 'true'


@pytest.mark.parametrize(
    ('backend', 'message'),
    [('hardware', 'cannot run the target-distance benchmark'),
     ('webots', 'Unknown benchmark backend')],
)
def test_benchmark_layers_reject_non_simulation_backends_before_starting_anything(
    adapter_lookups, backend, message,
) -> None:
    env = _load_launch('target_benchmark_env.launch.py')
    config = _load_launch('target_benchmark_config.launch.py')

    with pytest.raises(ValueError, match=message):
        env.build_simulation(_declared_context(env, backend=backend))
    with pytest.raises(ValueError, match=message):
        config.build_readiness_gate(_declared_context(config, backend=backend))
    assert adapter_lookups == []

    # Selection happens before the environment's first process is declared.
    entities = env.generate_launch_description().entities
    functions = [
        getattr(entity, '_OpaqueFunction__function', None) for entity in entities]
    selection = functions.index(env.build_simulation)
    rviz = next(index for index, entity in enumerate(entities)
                if getattr(entity, 'node_executable', None) == 'rviz2')
    detector = functions.index(env.build_detector)
    assert selection < rviz < detector


def test_benchmark_environment_rejects_a_world_the_selected_catalog_lacks(
    adapter_lookups,
) -> None:
    module = _load_launch('target_benchmark_env.launch.py')

    with pytest.raises(ValueError, match='no benchmark world "g1_distance_calibration"'):
        module.build_simulation(_declared_context(
            module, backend='isaac', world='g1_distance_calibration'))


def _runner_parameters(context, nodes) -> dict:
    from launch_ros.utilities import evaluate_parameters

    [runner] = [
        node for node in nodes
        if getattr(node, 'node_executable', None) == 'target_distance_benchmark_runner']
    merged = {}
    for parameters in evaluate_parameters(context, runner._Node__parameters):
        merged.update(parameters)
    return merged


@pytest.mark.parametrize(
    ('backend', 'catalog_package', 'namespace', 'settings'),
    [
        ('gz', 'ridgeback_autonomy_gz', '/gzserver',
         {'gz_gui': 'true', 'headless_rendering': 'false'}),
        ('isaac', 'ridgeback_autonomy_isaac', '/isaac_sim',
         {'depth_fidelity': 'ideal', 'headless': 'true', 'rtf': '1.0',
          'sim_mode': 'realtime'}),
    ],
)
def test_benchmark_runner_uses_the_selected_catalog_services_and_public_world(
    adapter_lookups, backend, catalog_package, namespace, settings,
) -> None:
    module = _load_launch('target_benchmark_config.launch.py')
    context = _declared_context(module, backend=backend, estimators='pointcloud')

    environment = module.resolve_environment(context)
    parameters = _runner_parameters(
        context, module.build_benchmark_nodes(context, environment=environment))

    assert adapter_lookups == [catalog_package]
    assert parameters['world'] == 'target_distance_calibration'
    assert parameters['backend'] == backend
    assert parameters['scene_control_service_namespace'] == namespace
    assert parameters['asset_catalog'] == str(
        _package_root().parent / catalog_package / 'sim' / 'benchmark_assets.yaml')
    assert json.loads(parameters['backend_settings']) == settings


@pytest.mark.parametrize('backend', ['gz', 'isaac'])
def test_single_launch_runner_keeps_public_world_in_both_child_commands(
    adapter_lookups, monkeypatch, backend,
) -> None:
    """Child launch contexts receive the same public world independently."""
    from types import SimpleNamespace

    wrapper = _load_launch('target_distance_benchmark.launch.py')
    monkeypatch.setattr(wrapper, 'get_package_share_directory', lambda _package: str(_package_root()))
    monkeypatch.setattr(wrapper, 'ExecuteProcess', lambda **kwargs: SimpleNamespace(**kwargs))
    monkeypatch.setattr(wrapper, 'OnProcessExit', lambda **kwargs: SimpleNamespace(**kwargs))
    monkeypatch.setattr(wrapper, 'RegisterEventHandler', lambda event: event)
    context = _declared_context(wrapper, backend=backend, estimators='pointcloud',
                                world='target_distance_calibration')
    supervisor = wrapper.build_supervisor(context, layers=wrapper.layer_arguments())[-1]
    environment = json.loads(supervisor.cmd[supervisor.cmd.index('--environment-command') + 1])
    configuration = json.loads(supervisor.cmd[supervisor.cmd.index('--configuration-command') + 1])

    for command in (environment, configuration):
        assert 'world:=target_distance_calibration' in command
        assert f'backend:={backend}' in command
        assert 'g1_distance_calibration' not in command
    assert 'estimators:=pointcloud' in configuration
    assert not any(token.startswith('estimators:=') for token in environment)
    assert 'shutdown_on_complete:=false' in configuration
    assert supervisor.sigterm_timeout == str(
        2 * wrapper.PROCESS_GROUP_STOP_BUDGET_SEC + wrapper.PROCESS_POLL_SEC)


def test_isaac_benchmark_bringup_never_resolves_the_gazebo_adapter(monkeypatch) -> None:
    """Every package lookup an Isaac benchmark's launch descriptions make."""

    source_root = _package_root().parent
    looked_up = []

    def share(package):
        looked_up.append(package)
        if package == 'ridgeback_autonomy_gz':
            raise AssertionError('the Gazebo adapter was resolved for an Isaac benchmark')
        return str(source_root / package)

    monkeypatch.setattr(backends, '_default_package_share', share)
    env = _load_launch('target_benchmark_env.launch.py')
    config = _load_launch('target_benchmark_config.launch.py')
    dispatcher = _load_launch('includes/simulation.launch.py')
    for module in (env, config, dispatcher):
        monkeypatch.setattr(module, 'get_package_share_directory', share)
    isaac_path = source_root / 'ridgeback_autonomy_isaac/launch/backend.launch.py'
    isaac_spec = importlib.util.spec_from_file_location('layout_isaac_backend', isaac_path)
    isaac = importlib.util.module_from_spec(isaac_spec)
    isaac_spec.loader.exec_module(isaac)
    monkeypatch.setattr(isaac, 'get_package_share_directory', share)

    context = _declared_context(config, env, backend='isaac', estimators='pointcloud')
    config.build_benchmark_nodes(context, environment=config.resolve_environment(context))
    include = env.build_simulation(context)[-1]
    for name, value in include.launch_arguments:
        context.launch_configurations[name] = _text(context, value)
    [adapter_include] = dispatcher._include_selected_backend(context)
    isaac.generate_launch_description()

    assert set(dict(adapter_include.launch_arguments)) == _DISPATCHER_ARGUMENT_NAMES['isaac']
    assert 'ridgeback_autonomy_isaac' in looked_up
    assert 'ridgeback_autonomy_gz' not in looked_up


def test_backend_settings_are_environment_owned_and_recorded_by_the_config_layer() -> None:
    assert 'backend' in CONFIG_LAUNCH_ARGUMENT_NAMES & ENV_LAYER_CONFIG_KEYS
    assert BACKEND_SETTING_NAMES <= CONFIG_LAUNCH_ARGUMENT_NAMES & ENV_LAYER_CONFIG_KEYS


def test_exploration_uses_unique_initial_test_world() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    exploration_text = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' / 'ridgeback_exploration.launch.py'
    ).read_text(encoding='utf-8')
    gazebo_launch_text = (
        repo_root / 'src' / 'ridgeback_autonomy_gz' / 'launch' / 'gazebo.launch.py'
    ).read_text(encoding='utf-8')
    initial_test_world_world = (
        repo_root / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'worlds'
        / 'initial_test_world.sdf'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('world', default_value='initial_test_world')" in exploration_text
    assert 'resolve_world_sdf(world, _world_search_dirs())' in gazebo_launch_text
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
        provenance = json.loads(
            (maps / f'{world}.provenance.json').read_text(encoding='utf-8'))
        assert provenance['public_world'] == world
        assert 'gz' in provenance['backends']
        for suffix, expected in provenance['map_sha256'].items():
            assert hashlib.sha256(
                (maps / f'{world}.{suffix}').read_bytes()).hexdigest() == expected

    exploration = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'launch' /
        'ridgeback_exploration.launch.py').read_text(encoding='utf-8')
    assert "'backend': backend" in exploration

    historical = maps / 'historical'
    assert not (historical / 'mock_hospital.pgm').exists()
    assert not (historical / 'warehouse.pgm').exists()
    assert not (historical / 'office.pgm').exists()
    assert not (
        repo_root / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'worlds'
        / 'detailed_hospital.sdf'
    ).exists()


def test_world_preview_catalog_uses_public_backend_qualified_names() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    preview_root = (
        repo_root / 'src' / 'ridgeback_autonomy' / 'sim' / 'world_previews')
    catalog = json.loads(
        (preview_root / 'catalog.json').read_text(encoding='utf-8'))

    entries = {
        (entry['backend'], entry['public_world']): entry
        for entry in catalog['worlds']
    }
    assert set(entries) == {('gz', 'depot'), ('gz', 'coworking_space')}
    for entry in entries.values():
        image = preview_root / entry['image']
        assert image.exists()
        assert hashlib.sha256(image.read_bytes()).hexdigest() == entry['image_sha256']


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

    assert by_name['Target HUD']['Topic']['Value'] == 'hud_target_overlay'
    assert by_name['HUD']['Topic']['Value'] == 'hud_overlay'

    # RViz itself is launched in the selected robot namespace. Relative display
    # topics therefore follow either simulator or hardware identity instead of
    # pinning the config to the simulator namespace.
    assert 'r100_0001' not in _exploration_rviz_path().read_text(encoding='utf-8')


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


def test_benchmark_wrapper_exposes_merged_layer_arguments(monkeypatch) -> None:
    wrapper = _load_launch('target_distance_benchmark.launch.py')
    monkeypatch.setattr(wrapper, 'get_package_share_directory', lambda _package: str(_package_root()))
    layers = wrapper.layer_arguments()
    arguments = wrapper.generate_launch_description().get_launch_arguments()
    config = layers[wrapper.CONFIG_LAUNCH]
    environment = layers[wrapper.ENVIRONMENT_LAUNCH]
    expected = list(dict.fromkeys(argument.name for argument in [*config, *environment]))

    assert [argument.name for argument in arguments] == expected
    assert len(arguments) == len({argument.name for argument in arguments})
    context = _declared_context(wrapper, namespace='test_robot', shutdown_on_complete='true')
    assert context.launch_configurations['base_frame'] == 'test_robot/robot/base_link'
    assert context.launch_configurations['shutdown_on_complete'] == 'true'
    assert CONFIG_LAUNCH_ARGUMENT_NAMES <= set(expected)


def test_headless_rendering_is_optional_and_owned_by_the_environment() -> None:
    gazebo_adapter_text = (
        _package_root().parent / 'ridgeback_autonomy_gz' / 'launch' / 'backend.launch.py'
    ).read_text(encoding='utf-8')
    gazebo = backends.BENCHMARK_BACKENDS['gz'].settings

    assert gazebo['headless_rendering'].default == 'false'
    assert gazebo['gz_gui'].default == 'true'
    # The environment forwards it (see the selected-simulator test); the config
    # layer declares it only to record it, and the sweep refuses it per config.
    assert 'headless_rendering' in ENV_LAYER_CONFIG_KEYS
    assert "DeclareLaunchArgument('headless_rendering', default_value='false')" in (
        gazebo_adapter_text)
    assert "'headless_rendering': PythonExpression(" in gazebo_adapter_text
    assert "'false' if '" in gazebo_adapter_text


def test_camera_profile_reaches_both_simulators_but_not_hardware() -> None:
    source_root = _package_root().parent
    exploration = _exploration_text()
    gazebo = (source_root / 'ridgeback_autonomy_gz/launch/backend.launch.py').read_text(
        encoding='utf-8')
    isaac = (source_root / 'ridgeback_autonomy_isaac/launch/backend.launch.py').read_text(
        encoding='utf-8')

    assert 'camera_profile' in _DISPATCHER_ARGUMENT_NAMES['gz']
    assert 'camera_profile' in _DISPATCHER_ARGUMENT_NAMES['isaac']
    assert 'camera_profile' not in _DISPATCHER_ARGUMENT_NAMES['hardware']
    assert "'camera_profile': LaunchConfiguration('camera_profile')" in exploration
    assert "LaunchConfiguration('camera_profile')" in gazebo
    assert "'sim_camera_horizontal_fov'" in gazebo
    assert "'--camera-profile', camera_profile" in isaac


def test_depth_fidelity_reaches_only_isaac() -> None:
    source_root = _package_root().parent
    exploration = _exploration_text()
    gazebo = (source_root / 'ridgeback_autonomy_gz/launch/backend.launch.py').read_text(
        encoding='utf-8')
    hardware = (
        source_root / 'ridgeback_autonomy_hardware/launch/backend.launch.py'
    ).read_text(encoding='utf-8')
    isaac = (source_root / 'ridgeback_autonomy_isaac/launch/backend.launch.py').read_text(
        encoding='utf-8')

    assert 'depth_fidelity' not in _DISPATCHER_ARGUMENT_NAMES['gz']
    assert 'depth_fidelity' in _DISPATCHER_ARGUMENT_NAMES['isaac']
    assert 'depth_fidelity' not in _DISPATCHER_ARGUMENT_NAMES['hardware']
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
    assert set(gazebo) == {'src/clearpath_simulator', 'src/gz-sim', 'src/ros_gz'}
    assert 'src/clearpath_simulator' not in core and 'src/gz-sim' not in core
    assert gazebo['src/gz-sim']['version'] == '446a44335a45b704b4d36dabcc5508ee34eeb3d8'
    assert '[src/gz-sim]="patches/gz_sim_sensors_shutdown.patch"' in checker
    # ros_gz 1.0.24, the tag the installed binaries were built from, overlaid
    # by its bridge alone.
    assert gazebo['src/ros_gz']['version'] == '5912bfe7976a79d2cc9f373b2d40a8a3a3e33da2'
    assert '[src/ros_gz]="patches/ros_gz_bridge_shutdown.patch"' in checker
    assert '[src/ros_gz]="ros_gz_bridge ros_gz_image"' in checker
    assert 'core|gz|viz|all' in checker


def test_viz_profile_overlays_only_the_patched_rviz_packages() -> None:
    repo_root = _package_root().parents[1]
    manifests = {
        name: yaml.safe_load(
            (repo_root / f'dependencies/{name}.repos').read_text(encoding='utf-8'))[
                'repositories']
        for name in ('core', 'gz', 'viz')
    }
    checker = (repo_root / 'tools/check_dependencies').read_text(encoding='utf-8')

    assert set(manifests['viz']) == {'src/rviz'}
    assert set(manifests['viz']).isdisjoint(manifests['core'])
    assert set(manifests['viz']).isdisjoint(manifests['gz'])
    # rviz 14.1.23, the tag the installed RViz binaries were built from.
    assert manifests['viz']['src/rviz']['version'] == (
        'feb01669f1297df2af755ce9cd2ed18083e7a8b2')
    assert '[src/rviz]="patches/rviz_shutdown.patch"' in checker
    assert '[src/rviz]="rviz_common rviz_default_plugins rviz_visual_testing_framework"' in checker
    assert 'viz) MANIFESTS=("$REPO_ROOT/dependencies/viz.repos")' in checker
    assert '"$REPO_ROOT/dependencies/viz.repos") ;;' in checker


def test_hardware_backend_is_attach_only_and_uses_real_sensor_defaults() -> None:
    source_root = _package_root().parent
    hardware = (source_root / 'ridgeback_autonomy_hardware/launch/backend.launch.py').read_text(
        encoding='utf-8')
    exploration = _exploration_text()
    start_script = (source_root.parent / 'start_exploration.sh').read_text(encoding='utf-8')

    assert "'start_platform', default_value='false'" in hardware
    assert "'use_sim_time': 'false'" in hardware
    assert "'namespace', default_value=''" in hardware
    assert 'resolve_namespace(' in hardware
    assert "'backend', default_value=legacy_sim" in exploration
    assert "'namespace', default_value=''" in exploration
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
