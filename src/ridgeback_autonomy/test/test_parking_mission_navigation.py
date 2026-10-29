"""Original mission tail: pre-parking, final pose, stop and wait, return to start."""

import json
import math
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.task import Future

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from ridgeback_autonomy.mission.parking_geometry import preparking_pose
from test_mission_manager import manager_factory  # noqa: F401 (fixture)
from test_parking_feasibility import Client, finish, running


class ParamClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.calls.append((request, future))
        return future


def answer(params, ok=True):
    request, future = params.calls[-1]
    future.set_result(SetParameters.Response(
        results=[SetParametersResult(successful=ok) for _ in request.parameters]))
    return {p.name: p.value.double_value for p in request.parameters}


def navigation_result(nav, status=GoalStatus.STATUS_SUCCEEDED, error=0):
    request, accepted, handle = nav.calls[-1]
    if not accepted.done():
        accepted.set_result(handle)
    result = NavigateToPose.Result()
    result.error_code = error
    handle.future.set_result(SimpleNamespace(status=status, result=result))
    p = request.pose.pose
    return p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w)


def feasible(manager_factory, monkeypatch, *, enabled=True):
    m, paths, published = running(manager_factory, monkeypatch)
    m.parking_navigation_enabled = enabled
    m.navigation_client = Client()
    m._goal_checker_set_client = ParamClient()
    while m.state == MissionState.EVALUATING_PARKING_PATHS:
        m._next_path(now_ns=m.test_ros_ns)
        if m._path_busy:
            finish(paths)
    return m, published


def hold_still(m, seconds=2.1):
    ns0, wall0 = m.test_ros_ns, m._monotonic.value
    for i in range(int(seconds / 0.1) + 1):
        m.test_ros_ns, m._monotonic.value = ns0 + i * 100_000_000, wall0 + i * 0.1
        m.motion.observe((0.,) * 6, m.test_ros_ns, m.test_ros_ns, m._monotonic())


def test_navigation_stays_off_by_default(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    assert m.state == MissionState.PARKING_PATH_FEASIBLE
    assert m.navigation_client.calls == [] and m._goal_checker_set_client.calls == []


def test_full_mission_parks_waits_and_returns_to_start(manager_factory, monkeypatch):
    m, published = feasible(manager_factory, monkeypatch)
    sel, params, nav = m.parking_plan.selected, m._goal_checker_set_client, m.navigation_client
    assert m.state == MissionState.NAVIGATING_TO_PREPARKING
    assert nav.calls == []   # the goal waits for the tighter goal checker
    assert answer(params) == {'general_goal_checker.xy_goal_tolerance': 0.08,
                              'general_goal_checker.yaw_goal_tolerance': 0.12}

    goal = navigation_result(nav)
    assert goal == pytest.approx(preparking_pose(sel.base_x, sel.base_y, sel.yaw, 0.5))
    assert m.state == MissionState.NAVIGATING_TO_PARKING
    assert len(nav.calls) == 1   # the final leg waits for its tighter tolerance
    assert answer(params) == {'general_goal_checker.xy_goal_tolerance': 0.03,
                              'general_goal_checker.yaw_goal_tolerance': 0.12}

    assert navigation_result(nav) == pytest.approx((sel.base_x, sel.base_y, sel.yaw))
    assert m.state == MissionState.PARKED
    m.tick()
    assert m.state == MissionState.PARKED   # no stop evidence yet

    hold_still(m)
    m.tick()
    m.test_ros_ns += 4_900_000_000
    m.tick()
    assert m.state == MissionState.PARKED and len(nav.calls) == 2
    m.test_ros_ns += 200_000_000
    m.tick()
    assert m.state == MissionState.RETURNING_TO_START
    assert answer(params) == {'general_goal_checker.xy_goal_tolerance': 0.40,
                              'general_goal_checker.yaw_goal_tolerance': 0.40}
    assert navigation_result(nav) == pytest.approx((1.0, 2.0, 0.25))   # saved start pose
    assert m.state == MissionState.MISSION_COMPLETE
    assert m.last_issue is None
    payload = json.loads(published['state'][-1].data)
    assert payload['parking_navigation_enabled'] is True
    assert set(payload['parking_mission_navigation']['arrival_errors']) == {
        'preparking', 'parking', 'start'}


def test_failed_leg_returns_to_start_and_reports_failure(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch)
    answer(m._goal_checker_set_client)
    navigation_result(m.navigation_client, status=GoalStatus.STATUS_ABORTED, error=104)
    assert m.state == MissionState.RETURNING_TO_START
    answer(m._goal_checker_set_client)
    navigation_result(m.navigation_client)
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'parking_failed'
    assert m.last_issue.reason.startswith('preparking_navigation_failed')


def test_final_pose_blocked_after_preparking_is_not_driven(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch)
    answer(m._goal_checker_set_client)
    monkeypatch.setattr('ridgeback_autonomy.mission.mission_manager_node.footprint_rejection_reason',
                        lambda *a: 'occupied')
    navigation_result(m.navigation_client)
    assert m.state == MissionState.RETURNING_TO_START
    assert m._parking_failure == 'final_pose_invalid: footprint_occupied'
    assert len(m.navigation_client.calls) == 1   # no final goal


def test_missing_goal_checker_service_never_parks_loosely(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    m.parking_navigation_enabled = True
    m._goal_checker_set_client.ready = False
    m._begin_parking_navigation()
    assert m.state == MissionState.RETURNING_TO_START
    assert m._parking_failure == 'goal_checker_tolerance_not_applied'
    navigation_result(m.navigation_client)   # return proceeds with the tighter checker
    assert m.state == MissionState.MISSION_FAILED


def test_cancel_cancels_owned_goal_and_restores_goal_checker(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch)
    params, nav = m._goal_checker_set_client, m.navigation_client
    answer(params)
    _, accepted, handle = nav.calls[-1]
    accepted.set_result(handle)
    m.cancel_mission('operator stop')
    assert m.state == MissionState.MISSION_CANCELED
    assert handle.cancels == 1
    assert answer(params)['general_goal_checker.xy_goal_tolerance'] == 0.40
    handle.future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED,
                                             result=NavigateToPose.Result()))
    assert m.state == MissionState.MISSION_CANCELED   # late result ignored


def test_leg_deadline_cancels_and_returns(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch)
    answer(m._goal_checker_set_client)
    _, accepted, handle = m.navigation_client.calls[-1]
    accepted.set_result(handle)
    m._monotonic.value += 91.0
    m.tick()
    assert handle.cancels == 1
    assert m.state == MissionState.RETURNING_TO_START
    assert 'deadline expired' in m._parking_failure


def test_revalidation_tolerates_one_missing_exact_time_transform(manager_factory, monkeypatch):
    from tf2_ros import TransformException
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    exact = m.tf_buffer.lookup_transform

    def lookup(target, source, when):
        if source == 'odom' and when.nanoseconds:
            raise TransformException('extrapolation into the future')
        return exact(target, source, when)

    m.tf_buffer.lookup_transform = lookup
    assert m._recent_map_odom(m.test_ros_ns) is not None
    assert m._plan_invalid_reason() is None
    assert m._recent_map_odom(m.test_ros_ns + 2_000_000_000) is None   # newest is too old


def test_feasibility_checks_the_driven_route_through_preparking(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    sel, route = m.parking_plan.selected, m.parking_plan.selected_path
    pre = preparking_pose(sel.base_x, sel.base_y, sel.yaw, 0.5)
    *_, nav2_end, final = route.poses
    assert (nav2_end.pose.position.x, nav2_end.pose.position.y) == pytest.approx(pre[:2])
    assert (final.pose.position.x, final.pose.position.y) == pytest.approx((sel.base_x, sel.base_y))
    assert sel.path_length_m == pytest.approx(
        math.hypot(1.0 - pre[0], 2.0 - pre[1]) + 0.5)   # fake Nav2 path is start->goal


def test_manager_budgets_final_tolerance_and_reference_out_of_the_reach_limit(manager_factory):
    m = manager_factory()
    assert m.planner_config.reach_allowance_m == pytest.approx(0.06)


def _delayed_sample(m, *, velocity=0.0, wall_gap=0.7):
    """One odometry sample arriving after a callback delay longer than the evidence window."""
    ns, wall = m.motion.last_ns + 100_000_000, m.motion.last_wall + wall_gap
    m.test_ros_ns, m._monotonic.value = ns, wall
    m.motion.observe((velocity,) + (0.,) * 5, ns, ns, wall)


def test_delayed_odometry_restart_keeps_a_feasible_plan_and_waits_to_drive(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    m.parking_navigation_enabled = True
    _delayed_sample(m)
    assert m.motion.reason == 'collecting_stationary_interval'
    m.tick()
    assert m.state == MissionState.PARKING_PATH_FEASIBLE   # plan kept, not driven yet
    assert m._goal_checker_set_client.calls == []
    hold_still(m)
    m.tf_buffer.stamp_ns = m.test_ros_ns   # TF keeps publishing while the robot stands
    m.tick()
    assert m.state == MissionState.NAVIGATING_TO_PREPARKING


def test_motion_after_planning_still_invalidates_the_plan(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    _delayed_sample(m, velocity=0.05, wall_gap=0.1)
    m.tick()
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert m.last_issue.reason == 'motion_evidence_lost: robot_moving'


def test_restarted_evidence_must_complete_within_the_settlement_timeout(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    _delayed_sample(m)
    m.tick()
    assert m.state == MissionState.PARKING_PATH_FEASIBLE
    m._monotonic.value += 10.5   # odometry stops entirely
    m.tick()
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert m.last_issue.reason.startswith('motion_evidence_lost')


def test_revalidation_tolerates_a_lagging_node_clock_but_not_stale_poses(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    m.tf_buffer.stamp_ns = m.test_ros_ns + 600_000_000   # TF ahead of a delayed clock callback
    assert m._plan_invalid_reason() is None
    m.tf_buffer.stamp_ns = m.test_ros_ns + 1_500_000_000  # beyond any plausible lag
    assert 'robot_pose_from_future' in m._plan_invalid_reason()
    m.tf_buffer.stamp_ns = m.test_ros_ns - 1_500_000_000  # stale still rejects
    assert 'robot_pose_stale' in m._plan_invalid_reason()


def test_path_checks_include_the_saved_mission_start_footprint(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    start = m.start_pose.pose
    (stood,) = m._start_footprint_poses()
    assert stood == pytest.approx(
        (start.position.x, start.position.y, 2 * math.atan2(start.orientation.z, start.orientation.w)))
    seen = []
    monkeypatch.setattr('ridgeback_autonomy.mission.mission_manager_node.path_rejection',
                        lambda *a, **k: seen.append(k) or None)
    assert m._plan_invalid_reason() is None
    assert seen == [{'occupied': m._start_footprint_poses()}]
