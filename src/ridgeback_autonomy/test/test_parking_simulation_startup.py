"""Parking launch contracts and scene failures, without running physics or navigation."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml

from launch.actions import DeclareLaunchArgument, EmitEvent
from launch import LaunchContext
from simulation_interfaces.msg import Result
from simulation_interfaces.srv import SpawnEntity

from ridgeback_autonomy.mission.simulation_config import simulation_setup_path
from ridgeback_autonomy.mission.simulation_startup import (
    NAV2_NODES, DEMO_G1, DEMO_TABLE, SimulationStartup, demo_specs, missing_readiness, spawn_demo_scene,
    check_demo_entities,
)
from test_scene_control import FakeClock, FakeSimulator, FakeServiceClient, _failed
from test_mission_manager import manager_factory  # noqa: F401
from ridgeback_autonomy.benchmarking.scene_control import SceneControlClient, SceneControlError


ROOT = Path(__file__).resolve().parents[3]


def launch_module():
    spec = importlib.util.spec_from_file_location('parking_launch',
        ROOT / 'src/ridgeback_autonomy/launch/ridgeback_mission.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def defaults(**overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in launch_module().generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


@pytest.mark.parametrize('backend,world,odom', [
    ('gz', 'depot', 'platform/odom/filtered'),
    ('isaac', 'initial_test_world', 'platform/odom'),
])
def test_backend_defaults_and_explicit_odometry_override(backend, world, odom):
    context = defaults(backend=backend)
    assert context.launch_configurations['world'] == world
    assert context.launch_configurations['odometry_topic'] == odom
    assert defaults(backend=backend, odometry_topic='test/odom').launch_configurations['odometry_topic'] == 'test/odom'
    assert context.launch_configurations['parking_navigation_enabled'] == 'false'


def test_isaac_demo_has_an_unambiguous_world():
    context = defaults(backend='isaac', demo_scene_enabled='true')
    assert context.launch_configurations['world'] == 'parking_demo'


@pytest.mark.parametrize('backend', ['gz', 'isaac'])
def test_default_robot_config_is_the_checkout_independent_of_cwd(backend, monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    selected = simulation_setup_path(str(ROOT/'install/ridgeback_autonomy/share/ridgeback_autonomy'),
                                    '', backend=backend, namespace='r100_0001')
    assert selected == str(ROOT / 'clearpath') + '/'


def test_isaac_refuses_sensor_rig_different_from_committed_usd(tmp_path):
    config = yaml.safe_load((ROOT/'clearpath/robot.yaml').read_text())
    config['sensors']['camera'][0]['xyz'][2] += 0.125
    (tmp_path/'robot.yaml').write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='simulator sensor rig'):
        simulation_setup_path(str(ROOT), str(tmp_path), backend='isaac', namespace='r100_0001')


def test_namespace_mismatch_is_refused():
    with pytest.raises(ValueError, match='namespace differs'):
        simulation_setup_path(str(ROOT), '', backend='gz', namespace='other_robot')


def test_demo_only_accepts_known_world_and_stationary_plan_mode():
    module = launch_module()
    with pytest.raises(ValueError, match='other worlds may already contain'):
        module._configure_simulation(defaults(backend='isaac', demo_scene_enabled='true',
            mission_management_enabled='true', world='initial_test_world'))
    with pytest.raises(ValueError, match='demo_plan_only requires'):
        module._configure_simulation(defaults(demo_scene_enabled='true',
            mission_management_enabled='true', demo_plan_only='true', exploration_enabled='true'))


def ready_inputs():
    samples = dict.fromkeys(('clock','front_scan','rear_scan','odometry','detections','measurements'), 10.)
    lifecycle = {name: (3, 10.) for name in NAV2_NODES}
    return samples, lifecycle, ({'paused': True}, 10.)


def test_delayed_readiness_needs_received_data_clock_and_active_collision_monitor():
    samples, lifecycle, guard = ready_inputs()
    assert missing_readiness(10., samples, True, lifecycle, guard, hold_explorer=True) == []
    assert 'advancing /clock' in missing_readiness(10., samples, False, lifecycle, guard, hold_explorer=True)
    lifecycle['collision_monitor'] = (2, 10.)
    assert 'collision_monitor active' in missing_readiness(10., samples, True, lifecycle, guard, hold_explorer=True)
    assert 'fresh rear_scan' in missing_readiness(13., samples, True, lifecycle, guard, hold_explorer=True)
    with pytest.raises(RuntimeError, match='not initially paused'):
        missing_readiness(10., samples, True, lifecycle, ({'paused':False},10.), hold_explorer=True)


def test_stationary_stub_is_latched_and_real_guard_requires_fresh_status():
    samples, lifecycle, _ = ready_inputs()
    guard = ({'ownership_mode': 'stationary_mock', 'paused': False}, 0.)
    assert missing_readiness(10., samples, True, lifecycle, guard, hold_explorer=False) == []
    assert 'fresh mission/status' in missing_readiness(
        10., samples, True, lifecycle, ({'paused': True},0.), hold_explorer=True)


def test_repeated_odometry_stamp_does_not_refresh_startup_evidence(monkeypatch):
    from nav_msgs.msg import Odometry
    monkeypatch.setattr('ridgeback_autonomy.mission.simulation_startup.time.monotonic', lambda: 5.)
    node = SimpleNamespace(sample_stamps={}, samples={}, clock_ns=1_100_000_000)
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.header.stamp.sec = 1
    SimulationStartup.on_sample(node, 'odometry', msg)
    assert not node.samples
    msg.header.stamp.nanosec = 10_000_000
    SimulationStartup.on_sample(node, 'odometry', msg)
    assert node.samples['odometry'] == 5.
    monkeypatch.setattr('ridgeback_autonomy.mission.simulation_startup.time.monotonic', lambda: 10.)
    SimulationStartup.on_sample(node, 'odometry', msg)
    assert node.samples['odometry'] == 5.


def test_exploration_release_precedes_detection_admission():
    events = []
    class Service:
        def __init__(self, name): self.name = name
        def service_is_ready(self): return True
    def wait(missing): assert missing() == []
    def call(client, request):
        events.append(client.name)
        return SimpleNamespace(success=True, message='ready')
    node = SimpleNamespace(
        managers=lambda: ['mission_manager_node'],
        mission={'startup_paused': True, 'start_pose_saved': True, 'target_committed': False},
        guard=({'healthy': True}, 0.), args=SimpleNamespace(demo=True, hold_explorer=True),
        resume=Service('resume'), start_mission=Service('admit'), wait=wait, call=call,
        get_logger=lambda: SimpleNamespace(info=lambda message: None))
    SimulationStartup.release(node)
    assert events == ['resume', 'admit']


def test_existing_or_duplicate_manager_is_refused_before_any_scene_mutation():
    node = SimpleNamespace(managers=lambda: ['mission_manager_node'])
    with pytest.raises(RuntimeError, match='refusing a second manager'):
        SimulationStartup.prepare(node)
    node = SimpleNamespace(managers=lambda: ['mission_manager_node']*2,
        wait=lambda missing: missing(), mission=None)
    with pytest.raises(RuntimeError, match='exactly one required'):
        SimulationStartup.release(node)


def scene():
    clock = FakeClock()
    simulator = FakeSimulator(clock)
    clients = {name: FakeServiceClient(simulator, name) for name in simulator.available}
    return simulator, SceneControlClient(clients, lambda dt: clock.advance(dt), clock=clock,
                                         ready_timeout_sec=.1, retry_sleep_sec=.01)


def specs(backend='gz'):
    return demo_specs(backend, share=lambda package: str(ROOT/'src'/package))


def test_real_asset_catalogs_supply_the_two_models_and_world_floor_heights():
    for backend, floor in [('gz',0.),('isaac',.05)]:
        g1, table = specs(backend)
        assert (g1.name, table.name) == (DEMO_G1, DEMO_TABLE)
        assert g1.z == table.z == floor
        assert Path(g1.uri).is_file() and Path(table.uri).is_file()
        assert table.y - g1.y == pytest.approx(1.05)


def test_successful_spawns_are_verified_in_order_and_never_renamed():
    simulator, client = scene()
    spawned = []
    spawn_demo_scene(client, specs(), simulator.entities, spawned)
    assert spawned == [DEMO_G1, DEMO_TABLE]
    calls = [name for name, _ in simulator.calls]
    first, second = [i for i,name in enumerate(calls) if name == 'spawn_entity']
    assert 'get_entities_states' in calls[first+1:second]
    assert all(not r.allow_renaming for r in simulator.calls_to('spawn_entity'))


@pytest.mark.parametrize('failed_entity', [DEMO_G1, DEMO_TABLE])
def test_spawn_failure_is_reported_and_only_owned_entities_are_rolled_back(failed_entity):
    simulator, client = scene()
    def fail_table(request):
        if request.name == failed_entity:
            return _failed(SpawnEntity.Response(), Result.RESULT_OPERATION_FAILED, 'spawn failed')
        return simulator.spawn_entity(request)
    simulator.overrides['spawn_entity'] = fail_table
    spawned = []
    with pytest.raises(SceneControlError, match='spawn failed'):
        spawn_demo_scene(client, specs(), simulator.entities, spawned)
    assert spawned == ([DEMO_G1] if failed_entity == DEMO_TABLE else [])
    assert not client.rollback(spawned)
    assert DEMO_G1 not in simulator.entities
    assert 'r100_0001/robot' in simulator.entities


@pytest.mark.parametrize('name', [DEMO_G1, 'g1_stationary_eval', 'default_g1_far_right_room', DEMO_TABLE])
def test_duplicate_target_prevents_any_spawn_or_deletion(name):
    simulator, client = scene()
    simulator.entities[name] = (0.,0.,0.,0.)
    with pytest.raises(RuntimeError, match='existing G1/table'):
        spawn_demo_scene(client, specs(), simulator.entities, [])
    assert simulator.calls == []


def test_gate_failure_shuts_down_without_starting_a_manager():
    module = launch_module()
    context = LaunchContext()
    sentinel = object()
    failed = module._after_gate(SimpleNamespace(returncode=1), context, [sentinel])
    assert sentinel not in failed and any(isinstance(action, EmitEvent) for action in failed)
    assert module._after_gate(SimpleNamespace(returncode=0), context, [sentinel]) == [sentinel]


def test_readiness_deadline_reports_unmet_condition_without_starting_anything(monkeypatch):
    monkeypatch.setattr('ridgeback_autonomy.mission.simulation_startup.rclpy.ok', lambda: True)
    node = SimpleNamespace(remaining=lambda: 0.,
        get_logger=lambda: SimpleNamespace(info=lambda text: None))
    with pytest.raises(RuntimeError, match='startup deadline expired; missing: test prerequisite'):
        SimulationStartup.wait(node, lambda: ['test prerequisite'])


def test_one_manager_and_two_success_gates_and_odometry_is_passed():
    module = launch_module()
    context = defaults(mission_management_enabled='true')
    nodes = module._startup_nodes(context)
    assert len(nodes) == 3
    text = (ROOT/'src/ridgeback_autonomy/launch/ridgeback_mission.launch.py').read_text()
    assert text.count("executable='mission_manager_node.py'") == 1
    assert "'parking_navigation_enabled', 'odometry_topic'" in text
    assert defaults().launch_configurations['mission_management_enabled'] == 'false'


def test_demo_dds_profile_has_only_loopback_discovery():
    tree = ET.parse(ROOT/'src/ridgeback_autonomy/config/parking_demo_cyclonedds.xml')
    assert tree.find('.//{*}NetworkInterface').get('name') == 'lo'
    assert tree.find('.//{*}AllowMulticast').text == 'false'
    assert [p.get('Address') for p in tree.findall('.//{*}Peer')] == ['127.0.0.1']


def test_demo_manager_captures_start_but_cannot_commit_before_release(manager_factory):
    from rclpy.parameter import Parameter
    from std_srvs.srv import Trigger
    from test_mission_manager import start_search, confirm_target
    manager = manager_factory(overrides=[Parameter('startup_paused', value=True)])
    start_search(manager)
    assert manager.start_pose is not None
    confirm_target(manager)
    assert not manager.target_committed
    response = manager._start_mission(Trigger.Request(), Trigger.Response())
    assert response.success
    confirm_target(manager)
    assert manager.target_committed


def test_initial_planning_uses_clock_time_tf_without_accepting_future_data(manager_factory):
    from test_mission_manager import planning_manager, occupancy_map, PLAN_NOW_NS
    manager, _ = planning_manager(manager_factory)
    manager._on_map(occupancy_map())
    lookup = manager.tf_buffer.lookup_transform
    def clock_aligned(target, source, stamp):
        result = lookup(target, source, stamp)
        ns = stamp.nanoseconds or PLAN_NOW_NS + 100_000_000
        result.header.stamp.sec, result.header.stamp.nanosec = divmod(ns, 10**9)
        return result
    manager.tf_buffer.lookup_transform = clock_aligned
    manager.try_plan_parking(0., now_ns=PLAN_NOW_NS)
    assert manager.parking_plan is not None
    assert ('map', 'base_link', PLAN_NOW_NS) in manager.tf_buffer.calls
