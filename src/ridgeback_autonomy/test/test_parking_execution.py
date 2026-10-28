"""No real action server: execution, freshness and ownership safety contracts."""

from dataclasses import replace
import math
from pathlib import Path as FilePath
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.task import Future

from ridgeback_autonomy.mission.motion_settlement import MotionSettlement
from ridgeback_autonomy.mission.parking_execution import (
    ExecutionContext, ExecutionLimits, ExecutionSafety, ExecutionState, ParkingExecutor,
)
from ridgeback_autonomy.mission.parking_feasibility import PlanningSnapshot
from ridgeback_autonomy.mission.parking_planner import CandidateDecision, ParkingPlan
from test_parking_planner import CONFIG, grid, target
from test_mission_manager import manager_factory, occupancy_map, planning_manager


def checked(path, status=GoalStatus.STATUS_SUCCEEDED, code=0):
    result = ComputePathToPose.Result()
    result.path, result.error_code = path, code
    return SimpleNamespace(status=status, result=result)


class Client:
    def __init__(self):
        self.calls = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback, goal_uuid=None):
        f = Future()
        self.calls.append((goal, f, feedback_callback))
        return f


class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result, self.cancel_result = Future(), Future()
        self.cancels = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        return self.cancel_result


class World:
    def __init__(self, *, enabled=True):
        self.now, self.wall = 10_000_000_000, 2.0
        self.robot = (2.5, 4., math.pi, self.now)
        self.map_odom = (0., 0., 0., self.now)
        self.grid = grid(size_m=8.)
        self.motion = MotionSettlement()
        for i in range(21):
            ns = 8_000_000_000 + i * 100_000_000
            self.motion.observe((0.,) * 6, ns, ns, i * .1)
        self.odom = Odometry()
        self.odom.header.frame_id, self.odom.child_frame_id = 'odom', 'base_link'
        self.odom.header.stamp.sec = 10
        self.safety = ExecutionSafety(self.now, self.wall, True, True, True, True, True, True)
        c = CandidateDecision(5.4, 4., 5.12, 4., math.pi, 'selected', 'footprint_free', .6,
                              path_status='valid')
        self.plan = ParkingPlan(True, None, target(6., 4.), .29,
            candidates=[c], selected=c, selected_path=self.path(self.robot, (5.12, 4., math.pi)))
        self.snapshot = PlanningSnapshot(self.robot, self.map_odom)
        self.client = Client()
        self.exec = ParkingExecutor(self.client, self.context, CONFIG, 'parking.xml', enabled=enabled)
        self.exec.collision_state(0, self.now, self.wall)

    def pose(self, values):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp.sec, p.header.stamp.nanosec = divmod(self.now, 1_000_000_000)
        p.pose.position.x, p.pose.position.y = map(float, values[:2])
        p.pose.orientation.z, p.pose.orientation.w = math.sin(values[2]/2), math.cos(values[2]/2)
        return p

    def path(self, start, end):
        p = Path()
        p.header = self.pose(start).header
        p.poses = [self.pose(start), self.pose(end)]
        return p

    def context(self):
        return ExecutionContext(self.plan, self.grid, self.robot, self.map_odom, self.snapshot,
            self.motion, self.now, self.wall, self.safety, odometry=self.odom,
            odometry_received_s=self.wall)

    def prepare(self):
        return self.exec.prepare(checked(self.path(self.robot, (4.62, 4., math.pi))))

    def start(self, *, accept=True):
        assert self.prepare(), self.exec.reason
        assert self.exec.start(), self.exec.reason
        self.handle = Handle()
        if accept:
            self.client.calls[-1][1].set_result(self.handle)

    def advance(self, dt=.1, *, pose=None):
        self.now += round(dt * 1e9)
        self.wall += dt
        self.robot = (*((pose or self.robot)[:3]), self.now)
        self.map_odom = (*self.map_odom[:3], self.now)
        self.grid = replace(self.grid, stamp_ns=self.now)
        self.plan.target = replace(self.plan.target, stamp_ns=self.now)
        self.odom.header.stamp.sec, self.odom.header.stamp.nanosec = divmod(self.now, 1_000_000_000)
        self.motion.observe((0.,) * 6, self.now, self.now, self.wall)
        self.safety = replace(self.safety, stamp_ns=self.now, received_s=self.wall)
        self.exec.collision_state(0, self.now, self.wall)

    def result(self, status=GoalStatus.STATUS_SUCCEEDED, code=0):
        r = NavigateToPose.Result()
        r.error_code = code
        self.handle.result.set_result(SimpleNamespace(status=status, result=r))

    def settle(self, pose=None):
        for _ in range(23):
            self.advance(pose=pose or self.exec.goal)
            self.exec.poll()


def test_valid_preparation_preserves_preparking_geometry_but_disabled_sends_nothing():
    w = World(enabled=False)
    assert w.prepare()
    assert w.exec.goal == pytest.approx((4.62, 4., math.pi))
    assert w.exec.final_goal == pytest.approx((5.12, 4., math.pi))
    assert not w.exec.start()
    assert w.exec.reason == 'execution_disabled' and not w.client.calls


def test_phase_2f_direct_path_cannot_replace_preparking_path():
    w = World()
    assert not w.exec.prepare()
    assert w.exec.reason == 'preparking_path_required'
    assert not w.exec.prepare(checked(w.plan.selected_path))
    assert w.exec.reason == 'path_goal_mismatch'


@pytest.mark.parametrize('problem,reason', [
    ('target_missing', 'target_unavailable'), ('target_stale', 'target_stale'),
    ('map_stale', 'map_stale'), ('pose_missing', 'robot_pose_unavailable'),
    ('pose_stale', 'robot_pose_stale'), ('odom_missing', 'odometry_missing'),
    ('odom_stale', 'odometry_stale'), ('moving', 'motion_not_settled'),
    ('tf_missing', 'execution_tf_unavailable'), ('tf_stale', 'execution_tf_stale'),
    ('drift', 'map_odom_changed'), ('robot_changed', 'robot_changed'),
    ('geometry', 'candidate_geometry_invalid'), ('unknown', 'path_footprint_unknown'),
])
def test_invalid_inputs_never_prepare(problem, reason):
    w = World()
    if problem == 'target_missing': w.plan.target = None
    if problem == 'target_stale': w.plan.target = replace(w.plan.target, stamp_ns=1)
    if problem == 'map_stale': w.grid = replace(w.grid, stamp_ns=1)
    if problem == 'pose_missing': w.robot = None
    if problem == 'pose_stale': w.robot = (*w.robot[:3], 1)
    if problem == 'odom_missing': w.odom = None
    if problem == 'odom_stale': w.odom.header.stamp.sec = 1
    if problem == 'moving': w.motion.observe((.1, 0., 0., 0., 0., 0.), w.now+1, w.now+1, w.wall)
    if problem == 'tf_missing': w.map_odom = None
    if problem == 'tf_stale': w.map_odom = (0., 0., 0., 1)
    if problem == 'drift': w.map_odom = (.1, 0., 0., w.now)
    if problem == 'robot_changed': w.robot = (2.6, 4., math.pi, w.now)
    if problem == 'geometry': w.plan.selected.task_y = 4.2
    if problem == 'unknown': w.grid.data[80, 65] = -1
    p = w.path((2.5,4.,math.pi), (4.62,4.,math.pi))
    assert not w.exec.prepare(checked(p))
    assert reason in w.exec.reason
    assert not w.client.calls


@pytest.mark.parametrize('field', ['ownership_released','navigation_clear','manual_control_clear',
    'emergency_stop_clear','controller_keepout_verified','goal_checker_verified'])
def test_interlocks_fail_closed(field):
    w = World()
    assert w.prepare()
    w.safety = replace(w.safety, **{field: False})
    assert not w.exec.start()
    assert w.exec.reason == field and not w.client.calls


def test_missing_and_stale_safety_and_collision_state():
    for problem, reason in [('missing', 'execution_interlocks_unavailable'),
                            ('old', 'execution_interlocks_stale'),
                            ('cm_missing', 'collision_monitor_unavailable'),
                            ('cm_old', 'collision_monitor_stale')]:
        w = World(); assert w.prepare()
        if problem == 'missing': w.safety = None
        if problem == 'old': w.safety = replace(w.safety, received_s=0.)
        if problem == 'cm_missing': w.exec.collision = None
        if problem == 'cm_old': w.exec.collision = (0, 1, 0.)
        assert not w.exec.start()
        assert w.exec.reason == reason and not w.client.calls


def test_expired_preparation_is_invalidated_before_any_dispatch():
    w = World(); assert w.prepare()
    w.advance(1.1)
    w.exec.poll()
    assert w.exec.state == ExecutionState.FAILED
    assert not w.exec.start() and not w.client.calls


def test_goal_rejection_and_abort_require_new_stop_evidence_and_never_retry():
    for rejection in [True, False]:
        w = World(); w.start(accept=not rejection)
        if rejection:
            w.client.calls[-1][1].set_result(Handle(accepted=False))
        else:
            w.result(GoalStatus.STATUS_ABORTED, 1)
        assert w.exec.state == ExecutionState.SETTLING
        assert not w.exec.busy and not w.motion.ready(w.now, w.wall)
        w.settle(pose=(2.5, 4., math.pi))
        assert w.exec.state == ExecutionState.FAILED
        assert len(w.client.calls) == 1


def test_cancel_acknowledgment_does_not_release_ownership_and_reset_is_blocked():
    w = World(); w.start()
    w.exec.cancel()
    w.handle.cancel_result.set_result(SimpleNamespace(goals_canceling=[1]))
    assert w.exec.busy and not w.exec.reset()
    assert w.handle.cancels == 1
    w.result(GoalStatus.STATUS_CANCELED)
    assert not w.exec.busy and w.exec.state == ExecutionState.SETTLING
    assert not w.exec.reset()
    w.settle(pose=(2.5,4.,math.pi))
    assert w.exec.state == ExecutionState.FAILED
    assert w.exec.reset()


def test_pending_acceptance_timeout_keeps_ownership_and_cancels_late_handle():
    w = World(); w.start(accept=False)
    w.exec.cancel('navigation_timeout')
    w.handle.cancel_result.set_result(SimpleNamespace(goals_canceling=[]))
    w.advance(6.)
    w.exec.poll()
    assert w.exec.busy and w.exec.state == ExecutionState.FAILED
    assert not w.exec.reset()
    w.client.calls[-1][1].set_result(w.handle)
    assert w.handle.cancels == 1 and w.exec.busy
    w.result(GoalStatus.STATUS_CANCELED)
    assert not w.exec.busy


@pytest.mark.parametrize('problem,reason', [
    ('target_stale', 'target_stale'), ('odom_missing','odometry_missing'),
    ('occupied', 'path_footprint_occupied'), ('g1','current_g1_keepout'),
    ('competing', 'navigation_clear'), ('drift','map_odom_changed'),
    ('timeout','navigation_timeout'), ('feedback','navigation_feedback_timeout'),
    ('collision','collision_monitor_stop'),
])
def test_execution_monitor_cancels_unsafe_or_failed_navigation(problem, reason):
    w = World(); w.start()
    w.advance(.1)
    if problem == 'target_stale': w.plan.target = replace(w.plan.target, stamp_ns=1)
    if problem == 'odom_missing': w.odom = None
    if problem == 'occupied': w.grid.data[80,65] = 100
    if problem == 'g1': w.robot = (5.9,4.,math.pi,w.now)
    if problem == 'competing': w.safety = replace(w.safety, navigation_clear=False)
    if problem == 'drift': w.map_odom = (.1,0.,0.,w.now)
    if problem == 'timeout': w.exec.deadline_s = w.wall
    if problem == 'feedback': w.advance(2.1)
    if problem == 'collision': w.exec.collision_state(1,w.now,w.wall)
    w.exec.poll()
    assert w.exec.state == ExecutionState.CANCELING
    assert reason in w.exec.reason and w.exec.busy and w.handle.cancels == 1


def test_feedback_progress_and_simulation_clock_watchdogs():
    w = World(); w.start()
    for _ in range(102):
        w.advance(.2)
        feedback = NavigateToPose.Feedback()
        feedback.current_pose = w.pose(w.robot)
        feedback.distance_remaining = 2.
        w.client.calls[-1][2](SimpleNamespace(feedback=feedback))
        w.exec.poll()
        if w.exec.state == ExecutionState.CANCELING: break
    assert w.exec.reason == 'navigation_progress_timeout'
    w = World(); w.start()
    # Fresh receipts with a frozen source clock must not authorize motion.
    w.wall += .6
    w.safety = replace(w.safety, received_s=w.wall)
    w.exec.collision_state(0,w.now,w.wall)
    w.exec.poll()
    assert w.exec.reason == 'execution_clock_not_advancing'


def test_success_requires_fresh_actual_pose_and_new_physical_settlement_for_both_stages():
    w = World(); w.start(); w.result()
    assert w.exec.state == ExecutionState.SETTLING
    w.exec.poll()
    assert w.exec.state == ExecutionState.SETTLING
    w.settle()
    assert w.exec.state == ExecutionState.READY_FOR_FINAL
    assert len(w.client.calls) == 1
    assert w.exec.prepare(checked(w.path(w.robot,w.exec.final_goal)), final=True)
    assert w.exec.start()
    w.handle = Handle(); w.client.calls[-1][1].set_result(w.handle)
    w.result(); w.settle()
    assert w.exec.state == ExecutionState.PARKED
    assert w.exec.arrival_error['xy_error_m'] < .08
    assert len(w.client.calls) == 2


@pytest.mark.parametrize('problem,reason', [('missing','robot_pose_unavailable'),
    ('stale','robot_pose_stale'), ('wrong','arrival_pose_outside_parking_tolerance')])
def test_nav2_success_is_not_enough_without_fresh_accurate_final_tf(problem, reason):
    w = World(); w.start(); w.result()
    # Supply actual, new stop samples; withhold or corrupt only the pose evidence.
    for _ in range(23): w.advance(pose=w.exec.goal)
    if problem == 'missing': w.robot = None
    if problem == 'stale': w.robot = (*w.exec.goal,1)
    if problem == 'wrong': w.robot = (4.8,4.,math.pi,w.now)
    w.exec.poll()
    assert w.exec.state == ExecutionState.FAILED and reason in w.exec.reason


def test_post_navigation_settlement_times_out_without_new_odometry():
    w = World(); w.start(); w.result()
    w.advance(11.)
    w.exec.poll()
    assert w.exec.state == ExecutionState.FAILED
    assert 'post_navigation_motion_timeout' in w.exec.reason
    assert not w.exec.reset()


def test_late_feedback_and_cancel_callbacks_cannot_corrupt_new_attempt():
    w = World(); w.start()
    old_feedback = w.client.calls[-1][2]
    w.exec.cancel(); old_cancel = w.handle.cancel_result
    w.result(GoalStatus.STATUS_CANCELED); w.settle(pose=(2.5,4.,math.pi))
    assert w.exec.reset()
    w.start()
    state, reason = w.exec.state, w.exec.reason
    old_cancel.set_result(SimpleNamespace(goals_canceling=[]))
    old_feedback(SimpleNamespace())
    assert (w.exec.state,w.exec.reason) == (state,reason)


def test_manager_stages_preparation_only_and_preserves_disabled_default(manager_factory):
    manager, _ = planning_manager(manager_factory, map_msg=occupancy_map())
    assert not manager.execution.enabled
    assert manager.execution_safety is None
    assert not manager.request_parking_navigation()


@pytest.mark.parametrize('enabled', [False, True])
def test_manager_explicit_request_cannot_bypass_disabled_or_missing_interlocks(manager_factory, enabled):
    from rclpy.parameter import Parameter
    from ridgeback_autonomy.mission.mission_manager_node import MissionState
    m = manager_factory(overrides=[Parameter('parking_execution_enabled', value=enabled)])
    w = World()
    m.state = MissionState.PARKING_PATH_FEASIBLE
    m.execution.client = w.client
    m.execution.context = w.context
    assert m.prepare_parking_navigation(checked(w.path(w.robot, (4.62,4.,math.pi))))
    if enabled:
        w.safety = None
    assert not m.request_parking_navigation()
    assert not w.client.calls
    if enabled:
        assert m.state == MissionState.PARKING_EXECUTION_FAILED
        assert m.execution.reason == 'execution_interlocks_unavailable'


def test_measured_robot_motion_is_expected_but_unsafe_motion_is_not():
    w = World(); w.start()
    w.advance(.1, pose=(2.7,4.,math.pi))
    w.odom.twist.twist.linear.x = .1
    w.motion.observe((.1,0.,0.,0.,0.,0.),w.now+1,w.now+1,w.wall)
    w.exec.poll()
    assert w.exec.state == ExecutionState.NAVIGATING
    assert w.exec.busy


def test_invalid_or_nonterminal_result_does_not_release_navigation_ownership():
    w = World(); w.start()
    w.handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_EXECUTING, result=None))
    assert w.exec.busy and w.exec.state == ExecutionState.CANCELING
    assert w.handle.cancels == 1


def test_collision_or_emergency_after_success_cannot_become_parking_success():
    w = World(); w.start(); w.result()
    for _ in range(23): w.advance(pose=w.exec.goal)
    w.safety = replace(w.safety, emergency_stop_clear=False)
    w.exec.poll()
    assert w.exec.state == ExecutionState.FAILED
    assert w.exec.reason == 'emergency_stop_clear'


def test_final_stage_rejects_stale_path_and_movement_after_preparation():
    w = World(); w.start(); w.result(); w.settle()
    p = w.path(w.robot,w.exec.final_goal)
    p.header.stamp.sec = 1
    assert not w.exec.prepare(checked(p), final=True)
    assert w.exec.reason == 'execution_path_stale'
    assert w.exec.prepare(checked(w.path(w.robot,w.exec.final_goal)), final=True)
    w.robot = (4.72,4.,math.pi,w.now)
    assert not w.exec.start()
    assert w.exec.reason == 'stage_start_pose_changed'


def test_navigation_timeout_before_acceptance_is_explicitly_polled():
    w = World(); w.start(accept=False)
    w.exec.deadline_s = w.wall
    w.exec.poll()
    assert w.exec.state == ExecutionState.CANCELING and w.exec.busy
    assert w.exec.reason == 'navigation_timeout'


def test_rejected_cancel_keeps_ownership_until_terminal_result():
    w = World(); w.start(); w.exec.cancel('operator_canceled')
    w.handle.cancel_result.set_result(SimpleNamespace(goals_canceling=[]))
    assert w.exec.busy and 'rejected' in w.exec.reason
    w.result(GoalStatus.STATUS_ABORTED)
    assert not w.exec.busy and w.exec.state == ExecutionState.SETTLING


def test_g1_margin_cannot_be_reduced_and_current_velocity_cannot_be_ignored():
    w = World()
    w.plan.keepout_radius_m = .01
    assert not w.prepare()
    assert w.exec.reason == 'g1_keepout_margin_invalid'
    w = World()
    w.odom.twist.twist.angular.z = .2
    assert not w.prepare()
    assert w.exec.reason == 'robot_still_moving'


def test_repeated_acceptance_callback_after_result_cannot_restart_settlement():
    w = World(); w.start(); w.result(); w.settle()
    state, reason = w.exec.state, w.exec.reason
    w.exec._accepted(w.client.calls[-1][1], w.exec.generation)
    assert (w.exec.state,w.exec.reason) == (state,reason)
    assert not w.exec.busy


@pytest.mark.parametrize('status,code', [(GoalStatus.STATUS_ABORTED,0),
    (GoalStatus.STATUS_CANCELED,0), (GoalStatus.STATUS_SUCCEEDED,1)])
def test_stage_path_requires_successful_nav2_result_not_just_a_geometric_line(status,code):
    w = World()
    p = w.path(w.robot,(4.62,4.,math.pi))
    assert not w.exec.prepare(p)
    assert w.exec.reason == 'stage_path_result_invalid'
    assert not w.exec.prepare(checked(p,status,code))
    assert w.exec.reason == 'stage_path_nav2_failed'


def test_actual_task_point_must_remain_in_reach_even_with_acceptable_goal_error():
    w = World(); w.start(); w.result(); w.settle()
    assert w.exec.prepare(checked(w.path(w.robot,w.exec.final_goal)), final=True)
    assert w.exec.start()
    w.handle = Handle(); w.client.calls[-1][1].set_result(w.handle)
    w.result(); w.settle(pose=(5.05,4.,math.pi))
    assert w.exec.arrival_error['xy_error_m'] < .08
    assert w.exec.state == ExecutionState.FAILED
    assert w.exec.reason == 'arrival_task_point_outside_reach'


def test_failed_repreparation_cannot_leave_an_old_route_executable():
    w = World(); assert w.prepare()
    result = checked(w.path(w.robot,(4.62,4.,math.pi)), code=1)
    assert not w.exec.prepare(result)
    assert w.exec.state == ExecutionState.IDLE
    assert not w.exec.start() and not w.client.calls


def test_manager_discards_selected_pose_when_prepared_snapshot_expires(manager_factory):
    from ridgeback_autonomy.mission.mission_manager_node import MissionState
    m, w = manager_factory(), World()
    m.state = MissionState.PARKING_PATH_FEASIBLE
    m.parking_plan = w.plan
    w.plan.selected.clearance_m = .5
    m.execution.client, m.execution.context = w.client, w.context
    assert m.prepare_parking_navigation(checked(w.path(w.robot,(4.62,4.,math.pi))))
    w.plan.target = replace(w.plan.target,stamp_ns=1)
    m.execution.poll()
    assert m.state == MissionState.PARKING_EXECUTION_FAILED
    assert not m.parking_plan.ok
    assert m.parking_plan.selected is None and m.parking_plan.selected_path is None
    assert not m.request_parking_navigation() and not w.client.calls


def test_staged_goal_checker_does_not_change_shared_frontier_configuration():
    config = FilePath(__file__).resolve().parents[1] / 'config'
    shared = yaml.safe_load((config/'nav2_params.yaml').read_text())['controller_server']['ros__parameters']
    staged = yaml.safe_load((config/'parking_goal_checker.yaml').read_text())['controller_server']['ros__parameters']
    assert shared['goal_checker_plugins'] == ['general_goal_checker']
    assert shared['general_goal_checker']['xy_goal_tolerance'] == .4
    assert shared['general_goal_checker']['yaw_goal_tolerance'] == .4
    assert staged['parking_goal_checker']['xy_goal_tolerance'] == .08
    assert staged['parking_goal_checker']['yaw_goal_tolerance'] == .12
    assert staged['parking_goal_checker']['stateful'] is False
    root = ET.parse(config/'parking_navigate_to_pose.xml').getroot()
    assert root.find('.//FollowPath').get('goal_checker_id') == 'parking_goal_checker'
    assert root.find('.//Spin') is None and root.find('.//BackUp') is None
    assert root.find('.//RecoveryNode') is None
