"""Non-driving motion evidence, swept paths, action lifecycle and drift tests."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import ComputePathToPose
from rclpy.task import Future

from ridgeback_autonomy.mission.motion_settlement import MotionSettlement
from ridgeback_autonomy.mission.parking_feasibility import (
    PlanningSnapshot, path_rejection, path_length, candidate_groups, select_path_candidate,
)
from ridgeback_autonomy.mission.parking_planner import CandidateDecision, GridSnapshot, PlannerConfig
from ridgeback_autonomy.mission.mission_manager_node import MissionState
from test_parking_planner import target, grid, CONFIG, APPROACH_FROM_WEST
from test_mission_manager import (
    manager_factory, planning_manager, PLAN_NOW_NS, occupancy_map, FakePauseClient,
    trigger_response, start_search, confirm_target, WallClock,
)


def stopped(m, first=1.0, wall=0.0):
    for i in range(21):
        ns = int((first + i * 0.1) * 1e9)
        m.observe((0.,) * 6, ns, ns, wall + i * .1)
    return ns, wall + 2.


def test_stationary_requires_two_clocks_and_fresh_continuous_samples():
    m = MotionSettlement()
    ns, wall = stopped(m)
    assert m.ready(ns, wall)
    assert not m.ready(ns, wall + .6)
    assert m.reason == 'odometry_stale'


@pytest.mark.parametrize('problem,reason', [
    ('moving', 'robot_moving'), ('rotating', 'robot_moving'),
    ('nan', 'odometry_nonfinite'), ('inf', 'odometry_nonfinite'),
    ('old', 'odometry_stale'), ('future', 'odometry_timestamp'),
    ('frame', 'odometry_frame'), ('repeat', 'odometry_not_advancing'),
])
def test_bad_odometry_resets_evidence(problem, reason):
    m = MotionSettlement()
    ns, wall = stopped(m)
    v = [0.] * 6
    if problem == 'moving': v[0] = .006
    if problem == 'rotating': v[5] = .006
    if problem == 'nan': v[2] = float('nan')
    if problem == 'inf': v[3] = float('inf')
    stamp = ns + 100_000_000
    now = stamp
    if problem == 'old': now += 600_000_000
    if problem == 'future': now -= 100_000_001   # beyond the sim-clock skew allowance
    if problem == 'repeat': stamp = ns
    m.observe(v, stamp, now, wall + .1, frame_valid=problem != 'frame')
    assert not m.ready(now, wall + .1)
    assert m.reason == reason


def test_spike_or_message_gap_restarts_full_stability_interval():
    m = MotionSettlement()
    ns, wall = stopped(m)
    m.observe((.1, 0, 0, 0, 0, 0), ns + 1, ns + 1, wall)
    ns, wall = stopped(m, first=4., wall=3.)
    assert m.ready(ns, wall)
    m.observe((0.,) * 6, ns + 600_000_000, ns + 600_000_000, wall + .6)
    assert not m.ready(ns + 600_000_000, wall + .6)


def pose(x, y, yaw=0., stamp_ns=PLAN_NOW_NS):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp.sec, p.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def path(poses):
    p = Path()
    p.header = pose(0, 0).header
    p.poses = [pose(*v) for v in poses]
    return p


def path_fixture():
    g = grid(size_m=8.)
    c = CandidateDecision(3., 4., 3., 4.2, 0., 'valid', 'footprint_free', .6)
    start = (1., 4.2, 0.)
    return g, c, start, path([start, (3., 4.2, 0.)])


def test_valid_path_and_original_path_length():
    g, c, start, p = path_fixture()
    assert path_rejection(p, c, start, g, target(), CONFIG, .29) is None
    assert path_length(p) == pytest.approx(2.)


@pytest.mark.parametrize('problem,reason', [
    ('empty', 'empty_path'), ('frame', 'path_frame'), ('pose_frame', 'path_pose_frame'),
    ('nan', 'path_nonfinite_or_invalid_rotation'), ('quat', 'path_nonfinite_or_invalid_rotation'),
    ('start', 'path_start_mismatch'), ('end', 'path_goal_mismatch'),
    ('yaw', 'path_goal_yaw_mismatch'), ('unknown', 'path_footprint_unknown'),
    ('occupied', 'path_footprint_occupied'), ('bounds', 'path_footprint_out_of_bounds'),
    ('g1', 'path_g1_keepout'),
])
def test_unsafe_path_cannot_override_parking_safety(problem, reason):
    g, c, start, p = path_fixture()
    if problem == 'empty': p.poses = []
    if problem == 'frame': p.header.frame_id = 'odom'
    if problem == 'pose_frame': p.poses[0].header.frame_id = 'odom'
    if problem == 'nan': p.poses[0].pose.position.x = float('nan')
    if problem == 'quat': p.poses[0].pose.orientation.w = 0.
    if problem == 'start': p.poses[0].pose.position.x += .3
    if problem == 'end': p.poses[-1].pose.position.x += .3
    if problem == 'yaw': p.poses[-1] = pose(3, 4.2, .3)
    if problem in ('unknown', 'occupied'):
        # Obstacle between two sparse path vertices, not at either endpoint.
        g.data[84, 40] = -1 if problem == 'unknown' else 100
    if problem == 'bounds':
        start = (.1, 4.2, 0.); p.poses[0] = pose(*start)
    t = target(2., 4.2) if problem == 'g1' else target()
    assert path_rejection(p, c, start, g, t, CONFIG, .29) == reason


def test_drift_guard_detects_translation_and_rotation_not_just_age():
    s = PlanningSnapshot((1., 2., 0., 10), (.1, 0., 0., 10))
    assert s.changed((1., 2., 0., 11), (.1, 0., 0., 11)) is None
    assert s.changed((1.06, 2., 0., 11), (.1, 0., 0., 11)) == 'robot_changed'
    assert s.changed((1., 2., 0., 11), (.16, 0., 0., 11)) == 'map_odom_changed'
    assert s.changed((1., 2., 0., 11), (.1, 0., .04, 11)) == 'map_odom_changed'
    assert s.changed(None, (.1, 0., 0., 11)) == 'planning_tf_unavailable'


@pytest.mark.parametrize('case,robot,map_odom,expected', [
    ('stationary', (1.,2.,0.,11), (.1,0.,0.,11), None),
    ('slam_correction', (1.06,2.,0.,11), (.16,0.,0.,11), 'robot_changed'),
    ('physical_translation', (1.06,2.,0.,11), (.1,0.,0.,11), 'robot_changed'),
    ('physical_rotation', (1.,2.,.04,11), (.1,0.,0.,11), 'robot_changed'),
])
def test_snapshot_guard_keeps_map_correction_and_physical_motion_unsafe(case,robot,map_odom,expected):
    snapshot=PlanningSnapshot((1.,2.,0.,10),(.1,0.,0.,10))
    assert snapshot.changed(robot,map_odom)==expected
    if case=='slam_correction':
        # Same odom-frame robot position; map-frame parking coordinates changed.
        assert robot[0]-map_odom[0]==pytest.approx(snapshot.robot[0]-snapshot.map_odom[0])
        assert snapshot.changed(snapshot.robot,map_odom)=='map_odom_changed'


def test_stable_robot_cannot_make_an_expired_target_snapshot_valid(manager_factory):
    manager,_=planning_manager(manager_factory,map_msg=occupancy_map())
    manager.test_ros_ns=PLAN_NOW_NS+6_000_000_000
    manager.tf_buffer.stamp_ns=manager.test_ros_ns
    manager.latest_map=occupancy_map(stamp_ns=manager.test_ros_ns)
    for i in range(21):
        stamp=manager.test_ros_ns-2_000_000_000+i*100_000_000
        manager.motion.observe((0.,)*6,stamp,stamp,i*.1)
    assert manager.motion.ready(manager.test_ros_ns,manager._monotonic())
    assert manager._plan_invalid_reason()=='target_stale'


def test_bounded_groups_preserve_priority_and_path_length_is_final_tier():
    from ridgeback_autonomy.mission.parking_planner import plan_parking
    plan = plan_parking(grid(), target(), CONFIG, APPROACH_FROM_WEST)
    groups = candidate_groups(plan, APPROACH_FROM_WEST, 3)
    assert len(groups) == 3 and all(len(g) <= 3 for g in groups)
    assert [g[0].preference_label for g in groups] == ['approach_side', 'approach_front', 'approach_back']
    a, b = groups[0][:2]
    a.reach_error_m = b.reach_error_m = .05
    a.clearance_m = b.clearance_m = .5
    a.path_length_m, b.path_length_m = 5., 2.
    assert select_path_candidate([a, b], .05) is b


class Handle:
    accepted = True

    def __init__(self):
        self.future = Future()
        self.cancels = 0

    def get_result_async(self): return self.future

    def cancel_goal_async(self):
        self.cancels += 1
        return Future()


class Client:
    def __init__(self): self.calls = []
    def server_is_ready(self): return True

    def send_goal_async(self, request):
        f, h = Future(), Handle()
        self.calls.append((request, f, h))
        return f


def running(manager_factory, monkeypatch):
    client = Client()
    manager, published = planning_manager(manager_factory, map_msg=occupancy_map())
    manager.compute_path_client = client
    # Pure swept-path checks are tested above; isolate asynchronous orchestration here.
    monkeypatch.setattr('ridgeback_autonomy.mission.mission_manager_node.path_rejection', lambda *a: None)
    manager._next_path(now_ns=PLAN_NOW_NS)
    return manager, client, published


def finish(client, *, error=0, empty=False, status=GoalStatus.STATUS_SUCCEEDED):
    req, f, h = client.calls[-1]
    if not f.done(): f.set_result(h)
    result = ComputePathToPose.Result()
    result.error_code = error
    result.path.header = req.goal.header
    if not empty: result.path.poses = [req.start, req.goal]
    h.future.set_result(SimpleNamespace(status=status, result=result))


def test_pause_acknowledgment_does_not_bypass_physical_stop(manager_factory):
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True)))
    start_search(m); confirm_target(m)
    assert m.state == MissionState.SETTLING_MOTION
    assert m.target_position is None and m.parking_plan is None
    m._monotonic.value = 11.
    m.tick()
    assert m.state == MissionState.MOTION_SETTLEMENT_FAILED
    assert m.last_issue.code == 'motion_settlement_timeout'


def test_path_requests_are_serial_and_first_failure_tries_next(manager_factory, monkeypatch):
    m, c, _ = running(manager_factory, monkeypatch)
    m._next_path(now_ns=PLAN_NOW_NS)
    assert len(c.calls) == 1
    req = c.calls[0][0]
    assert req.use_start and req.planner_id == 'GridBased'
    assert req.start.header.frame_id == req.goal.header.frame_id == 'map'
    finish(c, error=208, status=GoalStatus.STATUS_ABORTED)
    m._next_path(now_ns=PLAN_NOW_NS)
    assert len(c.calls) == 2
    finish(c)
    while m.state == MissionState.EVALUATING_PARKING_PATHS:
        m._next_path(now_ns=PLAN_NOW_NS)
        if m._path_busy: finish(c)
    assert m.state == MissionState.PARKING_PATH_FEASIBLE
    assert m.parking_plan.selected.path_status == 'valid'
    assert m.parking_plan.selected_path is not None
    assert not hasattr(m, 'navigate_to_pose_client')


def test_empty_path_rejected_by_actual_validator(manager_factory):
    m, _ = planning_manager(manager_factory, map_msg=occupancy_map())
    c = Client(); m.compute_path_client = c
    m._next_path(now_ns=PLAN_NOW_NS); finish(c, empty=True)
    assert any(a.path_reason == 'empty_path' for a in m.parking_plan.candidates)


def test_planner_timeout_cancels_without_releasing_unresolved_goal(manager_factory, monkeypatch):
    m, c, _ = running(manager_factory, monkeypatch)
    _, f, h = c.calls[0]; f.set_result(h)
    # Keep the physical-stationarity evidence current; test the action deadline itself.
    monkeypatch.setattr(m, '_plan_invalid_reason', lambda *a: None)
    m._monotonic.value += 3.1
    m.tick()
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert m.last_issue.code == 'planner_timeout'
    assert h.cancels == 1 and m._path_busy
    finish(c, status=GoalStatus.STATUS_CANCELED)
    assert not m._path_busy
    assert m.state == MissionState.PARKING_PLANNING_FAILED


def test_cancel_pending_acceptance_cancels_late_handle_and_ignores_result(manager_factory, monkeypatch):
    m, c, published = running(manager_factory, monkeypatch)
    m.cancel_mission()
    _, f, h = c.calls[0]; f.set_result(h)
    assert h.cancels == 1 and m._path_busy
    finish(c, status=GoalStatus.STATUS_CANCELED)
    assert m.state == MissionState.MISSION_CANCELED
    assert published['markers'][-1].markers[0].action == 3


def test_changed_map_during_request_cannot_produce_valid_selection(manager_factory, monkeypatch):
    m, c, _ = running(manager_factory, monkeypatch)
    m.latest_map = occupancy_map(frame='odom')
    finish(c)
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert 'map_frame_odom' in m.last_issue.reason


def test_source_time_map_odom_is_required(manager_factory):
    m, _ = planning_manager(manager_factory)
    m._target_map_odom = None
    m._on_map(occupancy_map())
    m.try_plan_parking(m._monotonic(), now_ns=PLAN_NOW_NS)
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert m.last_issue.code == 'parking_tf_unavailable'


def test_bounded_search_exhaustion_is_not_claimed_as_fully_blocked(manager_factory, monkeypatch):
    m, c, _ = running(manager_factory, monkeypatch)
    while m.state == MissionState.EVALUATING_PARKING_PATHS:
        finish(c, error=208, status=GoalStatus.STATUS_ABORTED)
        m._next_path(now_ns=PLAN_NOW_NS)
    assert m.last_issue.code == 'bounded_search_exhausted'
    assert len(c.calls) <= 18
    assert m.parking_plan.selected is None


@pytest.mark.parametrize('change,expected', [('robot', 'robot_changed'), ('frame', 'map_odom_changed'),
                                           ('target_age', 'target_stale'), ('map_age', 'map_stale')])
def test_selected_snapshot_is_invalidated_and_visualization_cleared(manager_factory, monkeypatch, change, expected):
    m, c, published = running(manager_factory, monkeypatch)
    while m.state == MissionState.EVALUATING_PARKING_PATHS:
        finish(c)
        m._next_path(now_ns=PLAN_NOW_NS)
    assert m.state == MissionState.PARKING_PATH_FEASIBLE
    if change == 'robot': m.tf_buffer.x += .06
    if change == 'frame':
        old = m._target_map_odom
        monkeypatch.setattr(m, '_map_odom', lambda *a: (old[0] + .06, old[1], old[2], old[3]))
    if change == 'map_age': m.latest_map.header.stamp.sec -= 6
    if change == 'target_age':
        m.test_ros_ns += 6_000_000_000
        monkeypatch.setattr(m.motion, 'ready', lambda *a: True)
    m.tick()
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert expected in m.last_issue.reason
    assert m.parking_plan.selected_path is None
    assert published['markers'][-1].markers[0].action == 3


def test_reset_does_not_overlap_unsettled_path_goal(manager_factory, monkeypatch):
    m, c, _ = running(manager_factory, monkeypatch)
    _, f, h = c.calls[0]; f.set_result(h)
    m.reset_mission()
    assert m._path_busy and h.cancels == 1
    finish(c, status=GoalStatus.STATUS_CANCELED)
    assert not m._path_busy
    assert m.state == MissionState.WAITING_FOR_START_POSE


def test_rejected_goal_acceptance_can_try_next_without_cancel(manager_factory, monkeypatch):
    m, c, _ = running(manager_factory, monkeypatch)
    _, f, h = c.calls[0]; h.accepted = False; f.set_result(h)
    m._next_path(now_ns=PLAN_NOW_NS)
    assert len(c.calls) == 2 and h.cancels == 0


def test_odometry_callback_validates_frames_and_twist(manager_factory):
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True)))
    start_search(m); confirm_target(m)
    msg = Odometry()
    msg.header.stamp = pose(0, 0).header.stamp
    msg.header.frame_id, msg.child_frame_id = 'odom', 'wrong_base'
    m._on_odometry(msg)
    assert m.motion.reason == 'odometry_frame'
    msg.child_frame_id = 'base_link'
    msg.twist.twist.linear.x = float('nan')
    m._on_odometry(msg)
    assert m.motion.reason == 'odometry_nonfinite'


def slow_planning_manager(manager_factory, monkeypatch):
    from ridgeback_autonomy.mission import mission_manager_node as module
    clock = WallClock()
    original = module.plan_parking
    def delayed(*args, **kwargs):
        plan = original(*args, **kwargs)
        clock.value += .6  # Single-threaded callback cannot dispatch odometry here.
        return plan
    monkeypatch.setattr(module, 'plan_parking', delayed)
    manager, _ = planning_manager(manager_factory, map_msg=occupancy_map(), clock=clock)
    return manager, clock


def test_slow_geometry_waits_for_fresh_real_odometry_before_path_checks(manager_factory, monkeypatch):
    m, clock = slow_planning_manager(manager_factory, monkeypatch)
    assert m.state == MissionState.PARKING_CANDIDATES_READY
    assert not m._path_busy and m.motion.max_age_s == .5
    first_ns, first_wall = m.test_ros_ns, clock.value
    for i in range(22):
        m.test_ros_ns = first_ns + (i+1)*100_000_000
        m.tf_buffer.stamp_ns = m.test_ros_ns
        clock.value = first_wall + (i+1)*.1
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(m.test_ros_ns, 1_000_000_000)
        m._on_odometry(msg)
        m.tick()
        if i < 20:
            assert m.state == MissionState.PARKING_CANDIDATES_READY
    assert m.state == MissionState.EVALUATING_PARKING_PATHS
    assert m.motion.ready(m.test_ros_ns, clock.value)
    assert m.last_issue is None


def test_post_planning_odometry_wait_is_bounded_and_never_invents_a_stop(manager_factory, monkeypatch):
    m, clock = slow_planning_manager(manager_factory, monkeypatch)
    assert m.state == MissionState.PARKING_CANDIDATES_READY
    clock.value += m.motion_settlement_timeout_s + .1
    m.tick()
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert m.last_issue.code == 'motion_settlement_timeout'
    assert not m._path_busy and not m.motion.ready(m.test_ros_ns, clock.value)


def test_oversized_path_is_rejected_with_finite_work():
    g, c, start, p = path_fixture()
    p.poses = [pose(1, 4.2)] * 2001
    assert path_rejection(p, c, start, g, target(), CONFIG, .29) == 'path_sampling_budget_exceeded'


def test_safety_polling_uses_steady_clock_and_missing_server_times_out(manager_factory):
    from rclpy.clock import ClockType
    m, _ = planning_manager(manager_factory, map_msg=occupancy_map())
    assert m.control_timer.clock.clock_type == ClockType.STEADY_TIME
    # Keep ROS time fixed; a wall-time deadline must still fail instead of hanging.
    m._monotonic.value = m._path_batch_deadline + .1
    # Freshness may fail first: neither outcome may select a path or keep evaluating.
    m.tick()
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert m.parking_plan.selected is None


def test_groups_take_each_sectors_best_reach_not_the_outer_edge():
    from ridgeback_autonomy.mission.parking_planner import plan_parking, preference
    plan = plan_parking(grid(), target(), CONFIG, APPROACH_FROM_WEST)
    t = plan.target
    sector = lambda c: math.floor(math.atan2(c.task_y - t.y, c.task_x - t.x) / (math.pi / 4))
    rank = lambda c: preference(c.task_x, c.task_y, t, APPROACH_FROM_WEST)[0]
    valid = [c for c in plan.candidates if c.status in ('valid', 'selected')]
    for group in candidate_groups(plan, APPROACH_FROM_WEST):
        for key in {sector(c) for c in group}:
            picked = min(c.reach_error_m for c in group if sector(c) == key)
            best = min(c.reach_error_m for c in valid
                       if sector(c) == key and rank(c) == rank(group[0]))
            assert picked <= best + plan.resolution_m


def test_sim_clock_lagging_odometry_by_a_few_ms_keeps_evidence():
    m = MotionSettlement()
    ns, wall = stopped(m)
    m.observe((0.,) * 6, ns + 100_000_000, ns + 95_000_000, wall + .1)
    assert m.ready(ns + 95_000_000, wall + .1)


def test_unknown_cells_under_the_robot_itself_do_not_block_its_route():
    g, c, start, p = path_fixture()
    g.data[80:89, 15:26] = -1           # unseen floor beneath the start footprint
    assert path_rejection(p, c, start, g, target(), CONFIG, .29) is None
    assert (g.data[80:89, 15:26] == -1).all()   # the caller's grid is not modified
    g.data[84, 20] = 100                # a real obstacle there still rejects
    assert path_rejection(p, c, start, g, target(), CONFIG, .29) == 'path_footprint_occupied'
    g.data[84, 20] = -1
    g.data[84, 40] = -1                 # unknown beyond the robot still rejects
    assert path_rejection(p, c, start, g, target(), CONFIG, .29) == 'path_footprint_unknown'
