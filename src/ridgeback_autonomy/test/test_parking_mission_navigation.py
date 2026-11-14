"""Original mission tail: pre-parking, final pose, stop and wait, return to start."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalInfo, GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.task import Future

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from ridgeback_autonomy.mission.parking_geometry import preparking_pose
from ridgeback_autonomy.mission.parking_nav2_profile import shared_goal_checker
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


def release(handle, status=GoalStatus.STATUS_CANCELED):
    """Nav2's terminal result: the only proof that the goal no longer executes."""
    handle.future.set_result(SimpleNamespace(status=status, result=NavigateToPose.Result()))


def reply_to_cancel(handle, accepted=True):
    response = CancelGoal.Response()
    response.return_code = (CancelGoal.Response.ERROR_NONE if accepted
                            else CancelGoal.Response.ERROR_GOAL_TERMINATED)
    response.goals_canceling = [GoalInfo()] if accepted else []
    handle.cancel_futures[-1].set_result(response)


def feasible(manager_factory, monkeypatch, *, enabled=True, attempts=1):
    """A path-feasible plan; one attempt by default, so a failed leg is the last one."""
    m, paths, published = running(manager_factory, monkeypatch)
    m.parking_navigation_enabled = enabled
    m.parking_max_attempts = attempts
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
    xy, yaw = shared_goal_checker(Path(__file__).resolve().parents[1] / 'config' / 'nav2_params.yaml')
    assert answer(params) == {'general_goal_checker.xy_goal_tolerance': xy,
                              'general_goal_checker.yaw_goal_tolerance': yaw}
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
    assert m.state == MissionState.MISSION_CANCELED   # the result releases the goal only
    assert not m._nav_canceling and m._nav_goal_generation is None


def test_leg_deadline_cancels_and_returns(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch)
    answer(m._goal_checker_set_client)
    _, accepted, handle = m.navigation_client.calls[-1]
    accepted.set_result(handle)
    m._parking_clearance = lambda: None   # LiDAR clear; the deadline is under test
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


def test_odometry_ahead_of_a_lagging_node_clock_restarts_instead_of_invalidating(
        manager_factory, monkeypatch):
    """Test A run 5 on 2026-11-09: after the path checks of an 8.1 m route, an odometry
    stamp ahead of the manager's clock discarded the verified plan
    (motion_evidence_lost: odometry_timestamp) with the robot standing still."""
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    m.parking_navigation_enabled = True
    ns, wall = m.motion.last_ns + 100_000_000, m.motion.last_wall + 0.1
    m.motion.observe((0.,) * 6, ns + 300_000_000, ns, wall)    # stamped ahead of the clock
    assert m.motion.reason == 'odometry_timestamp'
    m.test_ros_ns, m._monotonic.value = ns, wall
    m.tick()
    assert m.state == MissionState.PARKING_PATH_FEASIBLE   # plan kept, not driven yet
    hold_still(m)
    m.tf_buffer.stamp_ns = m.test_ros_ns
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
    m._route_check_cache = None            # planning already checked this route once
    assert m._plan_invalid_reason() is None
    assert seen == [{'occupied': m._start_footprint_poses()}]


# Goal ownership: a cancel request never releases a goal; only its terminal result does.

def owned_leg(manager_factory, monkeypatch):
    m, published = feasible(manager_factory, monkeypatch)
    answer(m._goal_checker_set_client)
    _, accepted, handle = m.navigation_client.calls[-1]
    accepted.set_result(handle)
    m._parking_clearance = lambda: None   # LiDAR clear; ownership is under test
    return m, handle, published


def expire_leg(m):
    m._monotonic.value += 91.0
    m.tick()
    assert m.state == MissionState.RETURNING_TO_START and m._return_after_stop


def navigation_payload(published):
    return json.loads(published['state'][-1].data)['parking_mission_navigation']


@pytest.mark.parametrize('reply', ['accepted', 'rejected', 'delayed'])
@pytest.mark.parametrize('status', [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED,
                                    GoalStatus.STATUS_ABORTED])
def test_cancelled_leg_returns_only_after_terminal_result_and_a_fresh_stop(
        manager_factory, monkeypatch, reply, status):
    m, handle, published = owned_leg(manager_factory, monkeypatch)
    nav, params = m.navigation_client, m._goal_checker_set_client
    expire_leg(m)
    assert handle.cancels == 1
    if reply != 'delayed':
        reply_to_cancel(handle, accepted=reply == 'accepted')
    hold_still(m)   # zero velocity while the result is delayed proves nothing about ownership
    m.tick()
    assert len(nav.calls) == 1 and len(params.calls) == 1
    assert navigation_payload(published)['goal_owned'] is True
    release(handle, status)   # including a goal that finished while the cancel was in flight
    assert m.state == MissionState.RETURNING_TO_START
    assert 'preparking' not in m._arrival_errors   # an abandoned leg never counts as reached
    assert navigation_payload(published)['goal_owned'] is False
    m.tick()
    assert len(params.calls) == 1   # the stop must be observed after the release
    hold_still(m)
    m.tick()
    answer(params)
    assert navigation_result(nav) == pytest.approx((1.0, 2.0, 0.25))   # then a new goal
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'parking_failed'


def test_cancel_without_terminal_result_fails_explicitly_and_keeps_ownership(
        manager_factory, monkeypatch):
    m, handle, published = owned_leg(manager_factory, monkeypatch)
    expire_leg(m)
    reply_to_cancel(handle)
    hold_still(m)
    m._monotonic.value += m.execution.limits.cancellation_timeout_s
    m.tick()
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'navigation_cancel_unresolved'
    assert handle.cancels == 2   # one final request; the goal is not discarded
    assert len(m.navigation_client.calls) == 1 and len(m._goal_checker_set_client.calls) == 1
    assert navigation_payload(published)['goal_owned'] is True
    m.reset_mission()
    assert m.state == MissionState.MISSION_FAILED   # reset refused while ownership is unknown
    release(handle)   # a late terminal result still releases the goal
    assert m.state == MissionState.MISSION_FAILED
    m.reset_mission()
    assert m.state == MissionState.WAITING_FOR_START_POSE


def test_no_second_goal_is_sent_before_the_previous_one_terminates(manager_factory, monkeypatch):
    m, handle, _ = owned_leg(manager_factory, monkeypatch)
    expire_leg(m)
    m._send_return_goal(True)   # a premature return attempt
    assert len(m.navigation_client.calls) == 1
    assert m.state == MissionState.MISSION_FAILED
    assert 'previous navigation goal not released' in m.last_issue.reason


def test_reset_during_an_active_goal_waits_for_its_terminal_result(manager_factory, monkeypatch):
    m, handle, _ = owned_leg(manager_factory, monkeypatch)
    m.reset_mission()
    assert m.state == MissionState.MISSION_CANCELED and handle.cancels == 1
    m.reset_mission()
    assert m.state == MissionState.MISSION_CANCELED and handle.cancels == 1
    release(handle)
    m.reset_mission()
    assert m.state == MissionState.WAITING_FOR_START_POSE


def test_cancel_before_acceptance_cancels_the_accepted_goal_and_tracks_it(
        manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch)
    answer(m._goal_checker_set_client)
    _, accepted, handle = m.navigation_client.calls[-1]
    m.cancel_mission('operator stop')
    accepted.set_result(handle)
    assert handle.cancels == 1
    m._monotonic.value += m.execution.limits.cancellation_timeout_s + 0.1
    m.tick()
    assert m.state == MissionState.MISSION_CANCELED
    assert m.last_issue.code == 'navigation_cancel_unresolved'


def test_unknown_acceptance_never_starts_the_return(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch)
    answer(m._goal_checker_set_client)
    m.navigation_client.calls[-1][1].set_exception(RuntimeError('transport lost'))
    assert m.state == MissionState.RETURNING_TO_START
    hold_still(m)
    m._monotonic.value += m.execution.limits.cancellation_timeout_s
    m.tick()
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'navigation_cancel_unresolved'
    assert len(m.navigation_client.calls) == 1



# Up to three parking attempts: a failed direction is excluded, and only failures that
# are specific to the attempt are retried.
from test_mission_manager import localize


def aim_camera(m):
    from test_g1_exploration import aim_camera as aim   # local: that module imports this one
    aim(m)


def back_at_observation(m):
    from test_g1_exploration import back_at_observation as back
    return back(m)


def vetoed(manager_factory, monkeypatch, reason='lidar_clearance: lidar_obstacle_on_route'):
    m, published = feasible(manager_factory, monkeypatch, attempts=3)
    aim_camera(m)
    answer(m._goal_checker_set_client)
    _, accepted, handle = m.navigation_client.calls[-1]
    accepted.set_result(handle)
    failed = m.parking_plan.selected
    m._parking_navigation_failed(reason, stop_first=True)
    return m, handle, failed, published


def stopped(m, handle):
    release(handle)
    hold_still(m)
    m.tick()


def bearing(m, c):
    return math.atan2(c.task_y - m.parking_plan.target.y, c.task_x - m.parking_plan.target.x)


def test_a_vetoed_attempt_stops_relocalizes_and_tries_another_direction(manager_factory, monkeypatch):
    m, handle, failed, _ = vetoed(manager_factory, monkeypatch)
    assert m.state == MissionState.RETURNING_TO_OBSERVATION and handle.cancels == 1
    m.tick()
    assert len(m.navigation_client.calls) == 1                 # nothing before Nav2 releases the leg
    release(handle)
    m.tick()
    assert len(m.navigation_client.calls) == 1                 # nor before a confirmed stop
    hold_still(m)
    m.tick()
    assert back_at_observation(m) == pytest.approx((1.0, 2.0, 0.25))   # the robot never moved here
    hold_still(m)
    m.tick()
    assert m.state == MissionState.LOCALIZE_G1
    m.test_ros_ns = m.tf_buffer.stamp_ns = m.test_ros_ns + 10
    localize(m, first_stamp_ns=m.test_ros_ns - 4)
    assert m.state in (MissionState.PARKING_CANDIDATES_READY, MissionState.EVALUATING_PARKING_PATHS)
    near = [c for c in m.parking_plan.candidates
            if abs(math.remainder(bearing(m, c) - bearing(m, failed), 2 * math.pi)) <= math.radians(20)]
    assert near and all(c.reason == 'failed_attempt_direction' for c in near
                        if c.status != 'collision' and c.reason in ('footprint_free', 'failed_attempt_direction'))
    assert any(c.status == 'valid' for c in m.parking_plan.candidates)
    assert m._parking_attempts == 1 and len(m._failed_bearings) == 1


@pytest.mark.parametrize('reason', [
    'lidar_clearance: lidar_stale: sensors/lidar2d_0/scan', 'lidar_clearance: odometry_stale',
    'NAVIGATING_TO_PREPARKING deadline expired', 'goal transport failed: x',
    'preparking_navigation_failed: status=5, error_code=0, '])
def test_failures_not_specific_to_the_attempt_are_never_retried(manager_factory, monkeypatch, reason):
    m, handle, _, _ = vetoed(manager_factory, monkeypatch, reason)
    assert m.state == MissionState.RETURNING_TO_START
    assert m._failed_bearings == []


def test_the_third_failed_attempt_returns_home_and_reports_all_attempts(manager_factory, monkeypatch):
    m, handle, _, _ = vetoed(manager_factory, monkeypatch)
    m._parking_attempts = 3                                    # this was the last allowed attempt
    m.state = MissionState.NAVIGATING_TO_PARKING
    m._parking_navigation_failed('lidar_clearance: lidar_obstacle_in_stopping_path', stop_first=True)
    assert m.state == MissionState.RETURNING_TO_START
    stopped(m, handle)
    answer(m._goal_checker_set_client)
    navigation_result(m.navigation_client)
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'parking_failed' and 'parking attempt 3/3' in m.last_issue.reason


def test_an_unconfirmed_stop_after_a_failed_attempt_never_drives(manager_factory, monkeypatch):
    m, handle, _, _ = vetoed(manager_factory, monkeypatch)
    release(handle)
    m._monotonic.value += m.motion_settlement_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'stop_not_confirmed'
    assert len(m.navigation_client.calls) == 1


def test_an_unresolved_cancellation_fails_without_a_competing_goal(manager_factory, monkeypatch):
    m, handle, _, _ = vetoed(manager_factory, monkeypatch)
    m._monotonic.value += m.execution.limits.cancellation_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'navigation_cancel_unresolved'
    assert len(m.navigation_client.calls) == 1


def test_no_alternative_direction_returns_home_and_reports_it(manager_factory, monkeypatch):
    m, handle, failed, _ = vetoed(manager_factory, monkeypatch)
    m.parking_retry_exclusion_rad = math.pi                    # every direction counts as failed
    stopped(m, handle)
    back_at_observation(m)
    hold_still(m)
    m.tick()
    m.test_ros_ns = m.tf_buffer.stamp_ns = m.test_ros_ns + 10
    localize(m, first_stamp_ns=m.test_ros_ns - 4)
    assert m.state == MissionState.RETURNING_TO_START
    answer(m._goal_checker_set_client)
    navigation_result(m.navigation_client)
    assert m.state == MissionState.MISSION_FAILED
    assert 'no alternative after 1 parking attempts' in m.last_issue.reason


def test_relocalization_failing_during_a_retry_returns_home(manager_factory, monkeypatch):
    m, handle, _, _ = vetoed(manager_factory, monkeypatch)
    stopped(m, handle)
    back_at_observation(m)
    hold_still(m)
    m.tick()
    assert m.state == MissionState.LOCALIZE_G1
    m._monotonic.value += m.localization_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.RETURNING_TO_START
    assert 'target_localization_timeout' in m._parking_failure



def facing_left_side(m):
    """The plan's G1 heading puts the failed pose on G1's left (relative bearing +90 degrees)."""
    from dataclasses import replace
    sel = m.parking_plan.selected
    bearing = math.atan2(sel.task_y - m.parking_plan.target.y, sel.task_x - m.parking_plan.target.x)
    m.parking_plan.target = replace(m.parking_plan.target, facing_yaw_rad=bearing - math.pi / 2)
    sel.preference_label = 'g1_left'
    m.target_heading = SimpleNamespace(yaw_rad=bearing - math.pi / 2, support_points=1, inlier_fraction=1)
    m._observe_target_body = lambda: None          # keep this heading through relocalization
    return m.parking_plan.target


def relocalized(m, handle):
    stopped(m, handle)
    back_at_observation(m)
    hold_still(m)
    m.tick()
    m.test_ros_ns = m.tf_buffer.stamp_ns = m.test_ros_ns + 10
    localize(m, first_stamp_ns=m.test_ros_ns - 4)
    from ridgeback_autonomy.mission.parking_planner import preference
    side = lambda c: preference(c.task_x, c.task_y, m.parking_plan.target, None)[1]
    return {side(c) for c in m.parking_plan.candidates if c.status == 'valid'}


@pytest.mark.parametrize('veto_on_g1,side_excluded', [(True, True), (False, False)],
                         ids=['veto_on_g1_body', 'veto_elsewhere'])
def test_a_failure_on_g1s_body_moves_the_retry_to_another_side(manager_factory, monkeypatch,
                                                                 veto_on_g1, side_excluded):
    m, published = feasible(manager_factory, monkeypatch, attempts=3)
    aim_camera(m)
    t = facing_left_side(m)
    answer(m._goal_checker_set_client)
    _, accepted, handle = m.navigation_client.calls[-1]
    accepted.set_result(handle)
    m._last_veto_xy = (t.x + 0.2, t.y) if veto_on_g1 else (t.x + 1.5, t.y + 1.5)
    m._parking_navigation_failed('lidar_clearance: lidar_obstacle_in_stopping_path', stop_first=True)
    assert (m._failed_sides == {'g1_left'}) == side_excluded
    sides = relocalized(m, handle)
    assert sides                                               # other safe poses remain
    assert ('g1_left' in sides) != side_excluded


def test_a_second_failure_on_the_same_side_excludes_it(manager_factory, monkeypatch):
    m, handle, _, _ = vetoed(manager_factory, monkeypatch)
    m._failed_labels = ['g1_left']
    m._retry_after_stop = False
    m.state = MissionState.NAVIGATING_TO_PARKING
    m.parking_plan.selected = next(c for c in m.parking_plan.candidates if c.clearance_m is not None)
    m.parking_plan.selected.preference_label = 'g1_left'
    m._parking_navigation_failed('lidar_clearance: lidar_obstacle_on_route', stop_first=True)
    assert m._failed_sides == {'g1_left'}


def test_an_excluded_provisional_pick_is_not_revived(manager_factory, monkeypatch):
    # Live run on fd6d966: the planner's own pick lay on the failed side, was rejected,
    # then reset to 'valid' as a provisional selection, and attempt 3 parked there again.
    m, _ = feasible(manager_factory, monkeypatch, attempts=3)
    facing_left_side(m)
    plan = m.parking_plan
    plan.selected.status = 'selected'
    m._failed_sides = {'g1_left'}
    m._exclude_failed_directions(plan)
    assert plan.selected is None
    assert all(c.status == 'rejected' for c in plan.candidates if c.preference_label == 'g1_left')


def test_an_unchanged_route_check_is_reused_and_a_changed_map_rechecks(manager_factory, monkeypatch):
    """Gazebo standard after the dev merge (2026-11-14): sweeping a 9 m route every tick
    delayed odometry past the stationary window and the plan expired."""
    import ridgeback_autonomy.mission.mission_manager_node as node
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    calls = []
    real = node.path_rejection
    monkeypatch.setattr(node, 'path_rejection', lambda *a, **k: calls.append(1) or real(*a, **k))
    m._route_check_cache = None
    assert m._plan_invalid_reason() is None and m._plan_invalid_reason() is None
    assert len(calls) == 1
    blocked = m.latest_map
    data = list(blocked.data)
    data[0] = 100 if data[0] != 100 else 0           # same stamp, different contents
    blocked.data = data
    m._plan_invalid_reason()
    assert len(calls) == 2
