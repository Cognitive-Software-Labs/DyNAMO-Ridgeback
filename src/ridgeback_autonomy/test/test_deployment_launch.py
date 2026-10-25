import importlib.util
from pathlib import Path
import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.utilities import perform_substitutions

ROOT = Path(__file__).resolve().parents[3]


def load(relative):
    spec=importlib.util.spec_from_file_location('deployment_launch', ROOT/relative)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def context_for(module, overrides):
    context=LaunchContext()
    context.launch_configurations.update(overrides)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument): entity.execute(context)
    return context


def executables(nodes, context):
    return [(n.node_executable if isinstance(n.node_executable,str) else perform_substitutions(context,n.node_executable)) for n in nodes if isinstance(n,Node)]


def test_external_mode_has_no_compute_or_health_process_and_needs_no_venv(monkeypatch):
    module=load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')
    monkeypatch.setenv('RIDGEBACK_PERCEPTION_VENV','/nonexistent')
    context=context_for(module,dict(backend='hardware',localization_mode='external',base_frame='observed_base'))
    names=executables(module.build_target_localization_nodes(context),context)
    assert set(names)=={'target_visualization_node','hud_node','target_overlay_node','localization_status'}
    assert context.launch_configurations['localization_required']=='true'
    assert context.launch_configurations['autonomous_motion_enabled']=='false'


def test_namespace_resolution_keeps_sim_identity_and_reads_hardware_config(tmp_path):
    from ridgeback_autonomy.namespace_resolution import resolve_namespace

    assert resolve_namespace(
        backend='gz', requested='', setup_path='/not/read') == 'r100_0001'
    (tmp_path / 'robot.yaml').write_text(
        'system:\n  ros2:\n    namespace: r100_0160\n', encoding='utf-8')
    assert resolve_namespace(
        backend='hardware', requested='', setup_path=str(tmp_path)) == 'r100_0160'


def test_explicit_namespace_override_does_not_require_robot_config():
    from ridgeback_autonomy.namespace_resolution import resolve_namespace

    assert resolve_namespace(
        backend='hardware', requested='/test_robot/', setup_path='/missing') == (
            'test_robot')


def test_hardware_namespace_resolution_fails_closed(tmp_path):
    from ridgeback_autonomy.namespace_resolution import resolve_namespace

    with pytest.raises(RuntimeError, match='namespace:=<observed_namespace>'):
        resolve_namespace(
            backend='hardware', requested='', setup_path=str(tmp_path))


def test_exploration_resolves_namespace_before_derived_defaults():
    module = load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(
        backend='gz', namespace='', setup_path='/not/read')

    actions = module.resolve_launch_namespace(context)
    actions[0].execute(context)
    base_frame_argument = next(
        entity for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument) and entity.name == 'base_frame')
    base_frame_argument.execute(context)

    assert context.launch_configurations['namespace'] == 'r100_0001'
    assert context.launch_configurations['base_frame'] == 'r100_0001/robot/base_link'


@pytest.mark.parametrize('backend', ['gz','isaac'])
@pytest.mark.parametrize('profile', ['640x480','1280x720'])
def test_simulator_local_mode_and_profiles_need_no_remote_host(backend,profile):
    module=load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')
    context=context_for(module,dict(backend=backend,camera_profile=profile))
    names=executables(module.build_target_localization_nodes(context),context)
    assert names.count('target_detector_node')==1
    assert names.count('target_mask_measurement_node')==1
    assert names.count('target_pointcloud_measurement_node')==1
    assert names.count('localization_health')==1
    assert context.launch_configurations['use_sim_time']=='true'


def test_standalone_default_is_headless_and_requires_explicit_frame():
    module=load('src/ridgeback_localization/launch/localization.launch.py')
    context=context_for(module,dict(base_frame='observed_base'))
    names=executables(module.build_nodes(context),context)
    assert set(names)=={'target_detector_node','target_mask_measurement_node','localization_health'}
    assert context.launch_configurations['use_sim_time']=='false'
    context.launch_configurations['base_frame']=''
    with pytest.raises(ValueError, match='observed'): module.build_nodes(context)


def test_standalone_detection_threshold_defaults_to_the_detector_and_is_bounded():
    from ridgeback_localization.core.detection import DETECTION_THRESHOLD
    module=load('src/ridgeback_localization/launch/localization.launch.py')
    context=context_for(module,dict(base_frame='observed_base'))
    assert context.launch_configurations['detection_threshold']==str(DETECTION_THRESHOLD)
    context.launch_configurations['detection_threshold']='0.4'
    module.build_nodes(context)
    for invalid in ('0','1.5'):
        context.launch_configurations['detection_threshold']=invalid
        with pytest.raises(ValueError, match='detection_threshold'): module.build_nodes(context)


def test_stationary_intel_observer_starts_only_consumers(monkeypatch):
    monkeypatch.setenv('RIDGEBACK_PERCEPTION_VENV','/nonexistent')
    module=load('src/ridgeback_autonomy/launch/localization_observer.launch.py')
    context=context_for(module,dict(base_frame='observed_base'))
    assert set(executables(module.build_nodes(context),context)) == {
        'localization_status','target_visualization_node','target_overlay_node'}


@pytest.mark.parametrize('backend', ['gz','isaac'])
@pytest.mark.parametrize('profile', ['640x480','1280x720'])
def test_localization_disabled_gates_the_whole_stack(backend,profile):
    from launch.actions import OpaqueFunction
    module=load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')
    context=context_for(module,dict(backend=backend,camera_profile=profile,target_localization_enabled='false'))
    blocks=[x for x in module.generate_launch_description().entities if isinstance(x,OpaqueFunction)]
    conditioned_blocks = [block for block in blocks if block.condition is not None]
    assert conditioned_blocks and all(
        not block.condition.evaluate(context) for block in conditioned_blocks)


def test_sweep_labels_are_shared_with_detector_and_recorded_config():
    from ridgeback_autonomy.benchmarking.sweep import parse_sweep
    from ridgeback_autonomy.benchmarking.target_benchmark_sweep import _environment_command
    spec=parse_sweep({'sweep':{'name':'labels'},'defaults':{'target_labels':'person'},
        'configs':[{'name':'projective','estimators':'projective_ranging'}]},source='/tmp/labels.yaml')
    assert 'target_labels:=person' in _environment_command(spec)
    assert spec.configs[0].arguments['target_labels']=='person'
    with pytest.raises(ValueError):
        parse_sweep({'sweep':{'name':'labels'},'configs':[{'name':'projective',
            'estimators':'projective_ranging','target_labels':'person'}]},source='/tmp/labels.yaml')


def gate_cmd(gate, context):
    return [perform_substitutions(context, part) for part in gate.cmd]


@pytest.mark.parametrize('backend, namespace', [('gz', 'r100_0001'), ('hardware', 'r100_0160')])
def test_local_localization_waits_for_the_shared_camera_contract(monkeypatch, backend, namespace):
    from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
    module = load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')
    stack = [LogInfo(msg='localization stack')]
    monkeypatch.setattr(module, 'build_target_localization_nodes', lambda context: stack)
    context = context_for(module, dict(backend=backend, namespace=namespace, base_frame='base_link'))

    gate, handler = module.build_gated_target_localization(context)

    assert isinstance(gate, ExecuteProcess) and isinstance(handler, RegisterEventHandler)
    cmd = gate_cmd(gate, context)
    topics = [cmd[i + 1] for i, part in enumerate(cmd) if part == '--topic']
    # Both backends gate on the same names: the hardware service publishes them.
    assert topics == [f'/{namespace}/sensors/camera_0/color/image',
                      f'/{namespace}/sensors/camera_0/color/camera_info',
                      f'/{namespace}/sensors/camera_0/depth/image']
    assert '--tf' not in cmd
    # Private in launch: the actions an OnProcessExit fires.
    assert handler.event_handler._OnActionEventBase__actions_on_event == stack


def test_external_localization_is_not_gated_on_a_local_camera(monkeypatch):
    monkeypatch.setenv('RIDGEBACK_PERCEPTION_VENV', '/nonexistent')
    module = load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')
    context = context_for(module, dict(backend='hardware', namespace='r100_0160',
                                       localization_mode='external', base_frame='base_link'))

    names = executables(module.build_gated_target_localization(context), context)

    assert 'localization_status' in names


def overlay_parameters(nodes, context):
    overlay=[n for n in nodes if isinstance(n,Node) and executables([n],context)==['target_overlay_node']]
    assert len(overlay)==1
    # launch_ros normalizes values to YAML-encoded substitutions.
    resolve=lambda value: yaml.safe_load(perform_substitutions(context,value)) if isinstance(value,tuple) else value
    return {resolve(k):resolve(v) for d in overlay[0]._Node__parameters for k,v in d.items()}


def test_external_overlay_reads_local_depth_and_local_overlay_keeps_the_republish(monkeypatch):
    from ridgeback_localization.contracts import ALIGNED_DEPTH_DEBUG_TOPIC
    monkeypatch.setenv('RIDGEBACK_PERCEPTION_VENV','/nonexistent')
    module=load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')
    context=context_for(module,dict(backend='hardware',localization_mode='external',base_frame='observed_base'))
    assert overlay_parameters(module.build_target_localization_nodes(context),context)['aligned_depth_topic']=='sensors/camera_0/depth/image'
    monkeypatch.delenv('RIDGEBACK_PERCEPTION_VENV')
    context=context_for(module,dict(backend='gz',localization_mode='local'))
    assert overlay_parameters(module.build_target_localization_nodes(context),context)['aligned_depth_topic']==ALIGNED_DEPTH_DEBUG_TOPIC


def test_stationary_observer_overlay_reads_local_depth(monkeypatch):
    monkeypatch.setenv('RIDGEBACK_PERCEPTION_VENV','/nonexistent')
    module=load('src/ridgeback_autonomy/launch/localization_observer.launch.py')
    context=context_for(module,dict(base_frame='observed_base'))
    assert overlay_parameters(module.build_nodes(context),context)['aligned_depth_topic']=='sensors/camera_0/depth/image'


def rviz_layout_argument(module, overrides, tmp_path, monkeypatch):
    from launch.actions import OpaqueFunction
    monkeypatch.setattr('tempfile.tempdir', str(tmp_path))
    context = context_for(module, overrides)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, OpaqueFunction):
            nodes = entity.execute(context) or []
            rviz = [n for n in nodes if isinstance(n, Node) and executables([n], context) == ['rviz2']]
            if rviz:
                args = [perform_substitutions(context, a) if isinstance(a, list) else a
                        for a in rviz[0]._Node__arguments]
                return args[args.index('-d') + 1]
    raise AssertionError('no rviz2 node')


def test_hardware_rviz_has_no_goal_tool_and_simulation_keeps_it(tmp_path, monkeypatch):
    module = load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')
    hardware = rviz_layout_argument(module, dict(backend='hardware', base_frame='base_link'), tmp_path, monkeypatch)
    simulation = rviz_layout_argument(module, dict(backend='gz'), tmp_path, monkeypatch)
    assert 'rviz_default_plugins/SetGoal' not in Path(hardware).read_text()
    assert simulation.endswith('exploration.rviz')
    assert 'rviz_default_plugins/SetGoal' in Path(simulation).read_text()


def test_hardware_rviz_process_is_niced_and_thread_limited(tmp_path, monkeypatch):
    from launch.actions import OpaqueFunction
    monkeypatch.setattr('tempfile.tempdir', str(tmp_path))
    module = load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')

    def rviz_process(overrides):
        context = context_for(module, overrides)
        for entity in module.generate_launch_description().entities:
            if isinstance(entity, OpaqueFunction):
                for n in entity.execute(context) or []:
                    if isinstance(n, Node) and executables([n], context) == ['rviz2']:
                        d = n.process_description
                        prefix = d._Executable__prefix
                        prefix = perform_substitutions(context, prefix) if prefix else ''
                        env = {perform_substitutions(context, k): perform_substitutions(context, v)
                               for k, v in (d._Executable__additional_env or [])}
                        return prefix, env
        raise AssertionError('no rviz2 node')

    prefix, env = rviz_process(dict(backend='hardware', base_frame='base_link'))
    assert prefix.startswith('nice -n 19') and env == {'LP_NUM_THREADS': '2'}
    assert rviz_process(dict(backend='gz')) == ('', {})


def test_deployed_workers_respawn_but_benchmark_workers_do_not():
    from ridgeback_localization.launch_helpers import RESPAWN_DELAY_S, mask_measurement_node
    module=load('src/ridgeback_localization/launch/localization.launch.py')
    context=context_for(module,dict(base_frame='observed_base'))
    nodes=[n for n in module.build_nodes(context) if isinstance(n,Node)]
    respawn={executables([n],context)[0]: (n._ExecuteLocal__respawn, n._ExecuteLocal__respawn_delay) for n in nodes}
    assert respawn['target_detector_node']==(True, RESPAWN_DELAY_S)
    assert respawn['target_mask_measurement_node']==(True, RESPAWN_DELAY_S)
    # The shared helper defaults to no respawn, which the benchmarks rely on.
    helper=mask_measurement_node(namespace='ns', use_sim_time=False, enabled_estimators='projective_ranging',
                                 base_frame='base_link')
    assert helper._ExecuteLocal__respawn is False


def _node_parameters(nodes, executable, context):
    from launch.utilities import perform_substitutions

    selected = [node for node in nodes if isinstance(node, Node)
                and node.node_executable == executable]
    assert len(selected) == 1

    def resolve(value):
        if isinstance(value, (tuple, list)):
            return yaml.safe_load(perform_substitutions(context, value))
        return value

    return {resolve(key): resolve(value)
            for parameters in selected[0]._Node__parameters
            for key, value in parameters.items()}


@pytest.mark.parametrize('backend', ['gz', 'isaac', 'hardware'])
def test_detector_settings_reach_final_local_exploration_node(backend):
    module = load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')
    context = context_for(module, dict(
        backend=backend, namespace='r100_0160', base_frame='base_link',
        detector='owlv2', detection_model='sentinel-checkpoint',
        target_labels='robot, person', detection_threshold='0.7',
        detector_fps='4', detector_debug='true'))
    params = _node_parameters(
        module.build_target_localization_nodes(context), 'target_detector_node', context)
    assert {key: params[key] for key in (
        'detector', 'detection_model', 'target_labels',
        'detection_threshold', 'detector_fps', 'detector_debug')} == {
            'detector': 'owlv2', 'detection_model': 'sentinel-checkpoint',
            'target_labels': 'robot,person', 'detection_threshold': 0.7,
            'detector_fps': 4.0, 'detector_debug': True,
        }


def test_detector_settings_reach_standalone_and_benchmark_layers():
    overrides = dict(base_frame='base_link', detector='owlv2',
                     detection_model='sentinel-checkpoint',
                     target_labels='robot, person', detection_threshold='0.7',
                     detector_fps='4', detector_debug='true')
    for path, builder, executable in [
        ('src/ridgeback_localization/launch/localization.launch.py',
         'build_nodes', 'target_detector_node'),
        ('src/ridgeback_autonomy/launch/target_benchmark_env.launch.py',
         'build_detector', 'target_detector_node'),
        ('src/ridgeback_autonomy/launch/target_benchmark_config.launch.py',
         'build_benchmark_nodes', 'target_distance_benchmark_runner'),
    ]:
        module = load(path)
        context = context_for(module, overrides)
        params = _node_parameters(getattr(module, builder)(context), executable, context)
        assert params['detector'] == 'owlv2', path
        assert params['detection_model'] == 'sentinel-checkpoint', path
        assert params['target_labels'] == 'robot,person', path
        assert params['detection_threshold'] == 0.7, path
        assert params['detector_fps'] == 4.0, path
        assert params['detector_debug'] is True, path



def test_compatibility_benchmark_wrapper_inherits_detector_settings_in_both_includes():
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription

    wrapper = load('src/ridgeback_autonomy/launch/target_distance_benchmark.launch.py')
    includes = [action for action in wrapper.generate_launch_description().entities
                if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 2
    context = LaunchContext()
    context.launch_configurations.update({
        'detector': 'owlv2', 'detection_model': 'sentinel-checkpoint',
        'target_labels': 'robot, person', 'detection_threshold': '0.7',
        'detector_fps': '4', 'detector_debug': 'true',
    })
    for include, name, builder, executable in zip(
            includes,
            ('target_benchmark_env.launch.py', 'target_benchmark_config.launch.py'),
            ('build_detector', 'build_benchmark_nodes'),
            ('target_detector_node', 'target_distance_benchmark_runner')):
        description = next(action for action in include.execute(context)
                           if isinstance(action, LaunchDescription))
        assert Path(include.launch_description_source.location).name == name
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        module = load(f'src/ridgeback_autonomy/launch/{name}')
        params = _node_parameters(getattr(module, builder)(context), executable, context)
        assert params['detector'] == 'owlv2'
        assert params['detection_model'] == 'sentinel-checkpoint'
        assert params['target_labels'] == 'robot,person'
        assert params['detection_threshold'] == 0.7
        assert params['detector_fps'] == 4.0
        assert params['detector_debug'] is True


def test_external_exploration_rejects_local_detector_override():
    module = load('src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py')
    context = context_for(module, dict(
        backend='hardware', namespace='r100_0160', base_frame='base_link',
        localization_mode='external', detection_model='sentinel-checkpoint'))
    with pytest.raises(ValueError, match='External localization cannot apply local detector'):
        module.build_target_localization_nodes(context)
