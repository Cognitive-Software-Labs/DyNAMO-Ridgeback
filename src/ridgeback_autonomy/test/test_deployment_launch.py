import importlib.util
from pathlib import Path
import pytest
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
    assert blocks and all(not block.condition.evaluate(context) for block in blocks)


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
