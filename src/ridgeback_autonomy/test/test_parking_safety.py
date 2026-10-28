"""Conditional collision bounds and real-interface vetoes; no motion server."""

from copy import deepcopy
from dataclasses import replace
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from clearpath_platform_msgs.msg import StopStatus
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_check_compatible, QoSCompatibility

from ridgeback_autonomy.mission.parking_execution import ExecutionState
from ridgeback_autonomy.mission.parking_nav2_profile import (
    build_profile, explicit_general_tree, goal_checker_reason, parking_tree_identity,
)
from ridgeback_autonomy.mission.parking_safety import (
    ParkingSafetyMonitor, ProtectionLimits, LOCK_TOPICS, COMMAND_TOPICS, MANUAL_TOPICS,
    keepout_mask, target_execution_reason,
)
from test_parking_execution import World
from test_parking_planner import CONFIG, grid, target
from test_mission_manager import manager_factory


def inputs(monitor, w, *, velocity=0., angular=0.):
    for t in LOCK_TOPICS:
        monitor.observe_lock(t,False,w.now,w.wall)
    stop = StopStatus()
    stop.header.stamp.sec, stop.header.stamp.nanosec = divmod(w.now,1_000_000_000)
    stop.stop_power_status = True
    monitor.observe_stop_status(stop,w.wall)
    monitor.observe_controller(True,w.now,w.wall)
    msg = TwistStamped()
    msg.header.stamp = stop.header.stamp
    msg.header.frame_id = 'base_link'
    msg.twist.linear.x, msg.twist.angular.z = float(velocity),float(angular)
    for t in COMMAND_TOPICS:
        monitor.observe_command(t,msg,w.now,w.wall)


def context(w):
    return replace(w.context(),target_robot=(*w.robot[:3],w.plan.target.stamp_ns))


def test_stationary_clear_sweep_is_conditional_not_a_controller_or_manual_authorization():
    w,m = World(),ParkingSafetyMonitor()
    inputs(m,w)
    assert m.check(context(w),CONFIG) is None
    assert m.summary()['manual_exclusion_authorized'] is False
    assert m.summary()['controller_stop_bound_qualified'] is False


@pytest.mark.parametrize('topic', LOCK_TOPICS)
def test_each_existing_mux_lock_vetoes(topic):
    w,m=World(),ParkingSafetyMonitor(); inputs(m,w)
    m.observe_lock(topic,True,w.now,w.wall)
    assert topic in m.check(context(w),CONFIG)


@pytest.mark.parametrize('topic', MANUAL_TOPICS)
def test_even_a_zero_manual_input_has_mux_ownership(topic):
    w,m=World(),ParkingSafetyMonitor(); inputs(m,w)
    m.observe_command(topic,TwistStamped(),w.now,w.wall)
    assert 'manual_control_active' in m.check(context(w),CONFIG)


@pytest.mark.parametrize('problem,reason', [('missing','stop_lock_unknown_or_stale'),
    ('stop_stale','stop_loop_unknown_or_stale'),('stop_fault','stop_loop_fault'),
    ('controller_stale','platform_controller_unknown_or_stale'),('controller_fault','platform_controller_fault'),
    ('source_tf','target_source_pose_unavailable'),('displacement','target_unobserved_displacement_exceeded')])
def test_unknown_faulted_or_stale_evidence_fails_closed(problem,reason):
    w,m=World(),ParkingSafetyMonitor(); inputs(m,w)
    ctx=context(w)
    if problem=='missing': del m.locks[LOCK_TOPICS[0]]
    if problem=='stop_stale': m.stop_status=(True,1,0.)
    if problem=='stop_fault': m.stop_status=(False,w.now,w.wall)
    if problem=='controller_stale': m.controller=(True,1,0.)
    if problem=='controller_fault': m.controller=(False,w.now,w.wall)
    if problem=='source_tf': ctx=replace(ctx,target_robot=None)
    if problem=='displacement': ctx=replace(ctx,target_robot=(1.,4.,math.pi,w.plan.target.stamp_ns))
    assert reason in m.check(ctx,CONFIG)


@pytest.mark.parametrize('topic',COMMAND_TOPICS)
def test_missing_any_execution_command_evidence_is_not_assumed_zero(topic):
    w,m=World(),ParkingSafetyMonitor(); inputs(m,w)
    del m.commands[topic]
    assert topic in m.check(context(w),CONFIG,moving=True)


@pytest.mark.parametrize('linear,angular',[(.5,0.),(0.,.8),(.3,.5)])
def test_full_footprint_latency_and_braking_veto_before_static_disk_collision(linear,angular):
    w,m=World(),ParkingSafetyMonitor()
    w.robot=(5.12,4.,math.pi,w.now)
    inputs(m,w,velocity=linear,angular=angular)
    assert m.check(context(w),CONFIG,moving=True)=='predicted_g1_swept_stop_violation'
    assert m.last_clearance > w.plan.keepout_radius_m
    assert m.last_clearance < m.last_required_clearance


def test_small_map_rotation_expands_uncertainty_at_the_target_not_only_at_origin():
    w,m=World(),ParkingSafetyMonitor(); inputs(m,w)
    ctx=context(w)
    assert m.check(ctx,CONFIG) is None
    previous=m.last_required_clearance
    ctx=replace(ctx,map_odom=(0.,0.,.02,w.now))
    assert m.check(ctx,CONFIG) is None
    assert m.last_required_clearance > previous+.1


def test_continuous_prediction_cancels_the_mock_action_and_retains_ownership():
    w,m=World(),ParkingSafetyMonitor(); inputs(m,w)
    w.exec.safety_check=lambda ctx,moving: m.check(replace(ctx,target_robot=ctx.robot),CONFIG,
        moving=moving,own_goal=w.exec.goal_uuid)
    w.start()
    w.advance(.1,pose=(5.12,4.,math.pi)); inputs(m,w,velocity=.5)
    w.exec.poll()
    assert w.exec.state==ExecutionState.CANCELING and w.exec.busy
    assert w.exec.reason=='predicted_g1_swept_stop_violation'
    assert w.handle.cancels==1


def test_action_uuid_is_known_before_acceptance_so_own_goal_is_not_a_competitor():
    w,m=World(),ParkingSafetyMonitor(); inputs(m,w); w.start(accept=False)
    msg=GoalStatusArray()
    status=GoalStatus(); status.status=GoalStatus.STATUS_ACCEPTED
    status.goal_info.goal_id.uuid=list(w.exec.goal_uuid)
    msg.status_list=[status]
    m.observe_action_status('navigate_to_pose',msg,w.now,w.wall)
    assert m.intervention(w.now,w.wall,w.exec.goal_uuid) is None
    status.goal_info.goal_id.uuid=[7]*16
    m.observe_action_status('navigate_to_pose',msg,w.now,w.wall)
    assert 'competing_navigation_goal' in m.intervention(w.now,w.wall,w.exec.goal_uuid)


def test_keepout_mask_preserves_source_time_reference_and_rotated_grid_geometry():
    g=grid(size_m=8.,origin=(8.,0.),yaw=math.pi/2)
    t=target(4.,4.)
    msg=keepout_mask(g,t,.29)
    values=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
    rows,cols=np.mgrid[:msg.info.height,:msg.info.width]
    x=msg.info.origin.position.x+(cols+.5)*msg.info.resolution
    y=msg.info.origin.position.y+(rows+.5)*msg.info.resolution
    assert np.all(values[np.hypot(x-t.x,y-t.y)<=.29]==100)
    assert np.all(values[np.hypot(x-t.x,y-t.y)>.4]==-1)
    assert msg.header.stamp.sec==10 and msg.header.frame_id=='map'
    assert msg.info.origin.orientation.z==0. and msg.info.origin.orientation.w==1.


def test_keepout_mask_rejects_outside_target_and_invalid_radius():
    with pytest.raises(ValueError): keepout_mask(grid(),target(20.,20.),.29)
    with pytest.raises(ValueError): keepout_mask(grid(),target(),float('nan'))


def test_profile_preserves_all_shared_safety_values_and_adds_both_filter_consumers():
    config=Path(__file__).resolve().parents[1]/'config'
    base=yaml.safe_load((config/'nav2_params.yaml').read_text()); original=deepcopy(base)
    profile=build_profile(base,'test_robot','/tmp/general.xml','/tmp/through.xml',
        yaml.safe_load((config/'parking_goal_checker.yaml').read_text()))['test_robot']
    assert base==original
    assert profile['velocity_smoother']==base['velocity_smoother']
    assert profile['collision_monitor']==base['collision_monitor']
    controller=profile['controller_server']['ros__parameters']
    assert controller['general_goal_checker']==base['controller_server']['ros__parameters']['general_goal_checker']
    assert controller['parking_goal_checker']['xy_goal_tolerance']==.08
    values={'goal_checker_plugins':controller['goal_checker_plugins']}
    for name in controller['goal_checker_plugins']:
        values.update({f'{name}.{k}':v for k,v in controller[name].items()})
    assert goal_checker_reason(values) is None
    del values['parking_goal_checker.plugin']
    assert 'unavailable' in goal_checker_reason(values)
    for name in ('local_costmap','global_costmap'):
        p=profile[name][name]['ros__parameters']
        assert p['plugins']==base[name][name]['ros__parameters']['plugins']
        assert p['parking_g1_keepout']['filter_info_topic']=='/test_robot/parking/g1_keepout_info'


def test_default_bt_conversion_changes_only_follow_path_checker_selection():
    source='<root><BehaviorTree><RecoveryNode number_of_retries="6"><FollowPath path="{path}"/></RecoveryNode></BehaviorTree></root>'
    changed=ET.fromstring(explicit_general_tree(source))
    follow=changed.find('.//FollowPath')
    assert follow.attrib.pop('goal_checker_id')=='general_goal_checker'
    assert ET.tostring(changed)==ET.tostring(ET.fromstring(source))


def test_invalid_dynamic_bounds_are_rejected():
    with pytest.raises(ValueError): ProtectionLimits(latency_s=0.)


def test_camera_loss_does_not_renew_target_or_authorize_unbounded_blind_travel():
    w,m=World(),ParkingSafetyMonitor(); inputs(m,w)
    old= context(w)
    # A brief missed frame is not a failure while the unchanged source is fresh.
    w.advance(.2); inputs(m,w)
    ctx=replace(context(w),plan=deepcopy(w.plan),target_robot=old.target_robot)
    ctx.plan.target=old.plan.target
    # World.advance replaces its target; restore the actual old source stamp.
    ctx.plan.target=replace(ctx.plan.target,stamp_ns=old.target_robot[3])
    assert m.check(ctx,CONFIG) is None
    assert m.check(replace(ctx,robot=(3.1,4.,math.pi,w.now)),CONFIG)=='target_unobserved_displacement_exceeded'


def test_missed_protection_poll_deadline_vetoes_even_with_new_inputs():
    w,m=World(),ParkingSafetyMonitor(); inputs(m,w)
    assert m.check(context(w),CONFIG,moving=True) is None
    w.advance(.4); inputs(m,w)
    assert m.check(context(w),CONFIG,moving=True)=='protection_poll_deadline'


def test_source_and_receipt_age_are_included_in_stopping_bound():
    w,m=World(),ParkingSafetyMonitor(); inputs(m,w,velocity=.1)
    assert m.check(context(w),CONFIG,moving=True) is None
    before=m.last_required_clearance
    for topic,(values,stamp,receipt,frame) in list(m.commands.items()):
        m.commands[topic]=(values,stamp-200_000_000,receipt-.2,frame)
    assert m.check(context(w),CONFIG,moving=True) is None
    assert m.last_required_clearance==pytest.approx(before+.02)


@pytest.mark.parametrize('kind',['lock','manual','stop'])
def test_real_manager_callbacks_cancel_only_mock_navigation(manager_factory,kind):
    node=manager_factory(); w=World(); w.start()
    node.execution=w.exec
    if kind=='lock': node._on_stop_lock(LOCK_TOPICS[0],Bool(data=True))
    if kind=='manual': node._on_command_evidence(MANUAL_TOPICS[0],TwistStamped())
    if kind=='stop': node._on_platform_stop(StopStatus())
    assert w.handle.cancels==1 and w.exec.busy


@pytest.mark.parametrize('kind',['missing','stale','inactive','checker_missing','valid'])
def test_real_manager_requires_fresh_loaded_active_goal_checker(manager_factory,kind):
    node=manager_factory()
    if kind=='missing':
        assert 'unknown_or_stale' in node._goal_checker_diagnostic()
        return
    values={'goal_checker_plugins':['general_goal_checker','parking_goal_checker']}
    for name,xy,yaw,stateful in [('general_goal_checker',.4,.4,True),('parking_goal_checker',.08,.12,False)]:
        values.update({f'{name}.plugin':'nav2_controller::SimpleGoalChecker',
            f'{name}.xy_goal_tolerance':xy,f'{name}.yaw_goal_tolerance':yaw,
            f'{name}.stateful':stateful})
    now=node.get_clock().now().nanoseconds
    node._probe_values={'checker':(values,now,0.),'checker_lifecycle':(True,now,0.)}
    if kind=='stale': node._probe_values['checker']=(values,1,-3.)
    if kind=='inactive': node._probe_values['checker_lifecycle']=(False,now,0.)
    if kind=='checker_missing': del values['parking_goal_checker.plugin']
    reason=node._goal_checker_diagnostic()
    if kind=='valid': assert reason is None
    elif kind=='stale': assert 'unknown_or_stale' in reason
    elif kind=='inactive': assert reason=='parking_controller_not_active'
    else: assert 'unavailable_or_mismatched' in reason


@pytest.mark.parametrize('change', ['checker','recovery','planner','include','missing','malformed','relative','valid'])
def test_parking_tree_must_be_exact_local_staged_contract(tmp_path,change):
    source=Path(__file__).resolve().parents[1]/'config/parking_navigate_to_pose.xml'
    path=tmp_path/'parking.xml'
    text=source.read_text()
    if change=='checker': text=text.replace('parking_goal_checker','general_goal_checker')
    if change=='recovery': text=text.replace('<Sequence>','<Sequence><Spin/>')
    if change=='planner': text=text.replace('GridBased','OtherPlanner')
    if change=='include': text=text.replace('<BehaviorTree ID="MainTree">','<include path="other.xml"/><BehaviorTree ID="MainTree">')
    if change=='malformed': text='<'
    path.write_text(text)
    if change=='missing': path.unlink()
    if change=='relative': path=Path('parking.xml')
    digest,reason=parking_tree_identity(path)
    if change=='valid': assert len(digest)==64 and reason is None
    else: assert digest is None and reason.startswith('parking_bt_')


def test_changed_parking_tree_cannot_reuse_startup_qualification(manager_factory,tmp_path):
    node=manager_factory()
    path=tmp_path/'parking.xml'
    path.write_text(Path(node.execution.behavior_tree).read_text())
    node.execution.behavior_tree=str(path)
    assert node._parking_bt_diagnostic() is None
    path.write_text(path.read_text()+'\n')
    assert node._parking_bt_diagnostic()=='parking_bt_changed_restart_and_requalify'


def test_mock_action_selects_admitted_tree_and_change_cancels_owned_goal(manager_factory,tmp_path):
    node=manager_factory(); w=World()
    path=tmp_path/'parking.xml'
    path.write_text(Path(node.execution.behavior_tree).read_text())
    w.exec.behavior_tree=str(path)
    node.execution=w.exec
    w.exec.safety_check=lambda ctx,moving: node._parking_bt_diagnostic()
    w.start()
    assert w.client.calls[0][0].behavior_tree==str(path)
    assert w.exec.goal[0]==pytest.approx(4.62)  # Still the outward pre-parking pose.
    path.write_text(path.read_text().replace('parking_goal_checker','general_goal_checker'))
    w.advance(.1); w.exec.poll()
    assert w.exec.reason=='parking_bt_contract_mismatch'
    assert w.handle.cancels==1 and w.exec.busy


@pytest.mark.parametrize('ref,foot,source,reason',[
    ('generic_target_position',False,'euclidean_reconstruction','generic_target_uncertainty_unqualified'),
    ('generic_target_position',True,'euclidean_reconstruction','target_reference_inconsistent'),
    ('foot_midpoint',False,'feet','target_reference_inconsistent_or_unsupported'),
    ('unknown',False,'feet','target_reference_inconsistent_or_unsupported'),
    ('foot_midpoint',True,'euclidean_reconstruction','foot_reference_source_unqualified'),
    ('foot_midpoint',True,'projective_ranging','foot_reference_source_unqualified'),
    ('foot_midpoint',True,'pointcloud','foot_reference_source_unqualified'),
    ('foot_midpoint',True,'polar_profiling','foot_reference_source_unqualified'),
    ('foot_midpoint',True,'','foot_reference_source_unqualified'),
    ('foot_midpoint',True,'unfamiliar_sensor_feet','foot_reference_uncertainty_unqualified'),
])
def test_explicit_reference_policy_never_relabels_generic_estimators_as_feet(ref,foot,source,reason):
    t=replace(target(),reference=ref,is_foot_midpoint=foot,source=source)
    assert target_execution_reason(t)==reason


def test_manager_generic_reference_remains_non_driving_even_with_other_proofs(manager_factory):
    node=manager_factory(); w=World()
    assert node._execution_safety_check(w.context(),False)=='generic_target_uncertainty_unqualified'
    assert node.execution.enabled is False and node.execution_safety is None


def test_manager_keepout_publication_retains_source_stamp_and_does_not_clear_on_loss(manager_factory):
    from test_mission_manager import occupancy_map
    node=manager_factory(); w=World()
    node.test_ros_ns=w.now
    node.latest_map=occupancy_map(stamp_ns=w.now,size=200)
    node.parking_plan=w.plan
    masks=[]; infos=[]
    node.mask_publisher.publish=masks.append
    node.mask_info_publisher.publish=infos.append
    node._publish_keepout_mask()
    assert len(masks)==len(infos)==1
    assert masks[0].header.stamp.sec==10
    assert infos[0].filter_mask_topic.endswith('/parking/g1_keepout_mask')
    node.test_ros_ns+=6_000_000_000
    node._publish_keepout_mask()
    assert len(masks)==1  # No zero/free mask, no renewal of old target time.


def test_stop_subscription_qos_accepts_best_effort_and_reliable_sources(manager_factory):
    node=manager_factory()
    stop_topics={node.resolve_topic_name(t) for t in LOCK_TOPICS+('platform/mcu/status/stop',)}
    subs=[s for s in node._safety_subscriptions if s.topic_name in stop_topics]
    assert len(subs)==4
    for sub in subs:
        for reliability in (ReliabilityPolicy.BEST_EFFORT,ReliabilityPolicy.RELIABLE):
            offered=QoSProfile(depth=5,reliability=reliability)
            assert sub.qos_profile.reliability==ReliabilityPolicy.BEST_EFFORT
            # SYSTEM_DEFAULT liveliness can yield WARNING, not incompatibility.
            assert qos_check_compatible(offered,sub.qos_profile)[0]!=QoSCompatibility.ERROR


def test_best_effort_stop_messages_reach_actual_manager_in_private_ros_namespace():
    from test_mission_manager import FakeTfBuffer, FakePauseClient, FakePathClient
    from ridgeback_autonomy.mission.mission_manager_node import MissionManagerNode
    if os.environ.get('ROS_DOMAIN_ID')!='42':
        pytest.skip('private ROS transport qualification requires domain 42')
    rclpy.init(args=['--ros-args','-r',f'__ns:=/parking_stop_probe_{os.getpid()}'])
    node=None
    try:
        node=MissionManagerNode(tf_buffer=FakeTfBuffer(),pause_client=FakePauseClient(),
            compute_path_client=FakePathClient())
        node.control_timer.cancel()
        assert node.get_namespace().startswith('/parking_stop_probe_')
        offer=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
        lock=node.create_publisher(Bool,LOCK_TOPICS[0],offer)
        stop=node.create_publisher(StopStatus,'platform/mcu/status/stop',offer)
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:
            msg=StopStatus(); msg.header.stamp=node.get_clock().now().to_msg()
            msg.stop_power_status=True
            lock.publish(Bool(data=True)); stop.publish(msg)
            rclpy.spin_once(node,timeout_sec=.02)
            if node.safety_monitor.stop_status and node.safety_monitor.locks.get(LOCK_TOPICS[0],(False,))[0]:
                break
        assert node.safety_monitor.stop_status[0] is True
        assert node.safety_monitor.locks[LOCK_TOPICS[0]][0] is True
        assert 'mux_lock_active' in node.safety_monitor.stop_reason(
            node.get_clock().now().nanoseconds,time.monotonic())
        assert node.execution.enabled is False and node.execution_safety is None
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('fault,reason',[
    ('target_age','target_stale'),('translation','map_odom_changed'),
    ('rotation','map_odom_changed'),('target_update','selected_plan_changed'),
])
def test_visibility_faults_cancel_mock_navigation_without_releasing_ownership(fault,reason):
    w=World(); w.start(); source=w.plan.target
    w.advance(.1)
    w.plan.target=source  # No observation: never renew its measurement timestamp.
    if fault=='target_age': w.plan.target=replace(source,stamp_ns=1)
    if fault=='translation': w.map_odom=(.051,0.,0.,w.now)
    if fault=='rotation': w.map_odom=(0.,0.,.031,w.now)
    if fault=='target_update': w.plan.target=replace(source,x=source.x+.1)
    w.exec.poll()
    assert w.exec.state==ExecutionState.CANCELING and w.exec.reason==reason
    assert w.exec.busy and w.handle.cancels==1 and not w.exec.reset()
    assert len(w.client.calls)==1


def test_short_camera_miss_cannot_renew_source_or_bypass_production_admission(manager_factory):
    w=World(); source=w.plan.target
    w.start(); w.advance(.2); w.plan.target=source; w.exec.poll()
    assert w.exec.state==ExecutionState.NAVIGATING  # Mock qualified, fresh reference only.
    assert w.plan.target.stamp_ns==source.stamp_ns
    node=manager_factory()
    assert node._execution_safety_check(w.context(),True)=='generic_target_uncertainty_unqualified'
    # An unobserved physical target move has no input representation here.
    # Timestamp freshness cannot prove the humanoid remained stationary.


def test_expired_plan_remains_unexecutable_without_an_automatic_retry():
    w=World(); assert w.prepare()
    w.advance(1.1); w.exec.poll()
    assert w.exec.state==ExecutionState.FAILED
    assert not w.exec.start() and not w.client.calls
