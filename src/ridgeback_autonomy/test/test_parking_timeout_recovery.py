"""After a route search cut short by its deadline (``planning_timeout``) or its route-check
budget (``route_budget_exhausted``): stop checking routes, confirm the stop, then return home
once only if the sensors are fresh and Nav2 plans a route home; otherwise stay stopped. The
mission fails under that code either way, never as ``no_valid_candidate``."""

import pytest
from action_msgs.msg import GoalStatus

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from test_mission_manager import manager_factory  # noqa: F401 (fixture)
from test_parking_feasibility import PLAN_NOW_NS, Client, finish, running
from test_parking_mission_navigation import ParamClient, answer, hold_still, navigation_result
from test_parking_clearance import feed

START = (1.0, 2.0, 0.25)


def timed_out(manager_factory, monkeypatch):
    """Driving on; the first route check outlives its 3 s request deadline."""
    m, paths, published = running(manager_factory, monkeypatch)
    m.parking_navigation_enabled = True
    m.navigation_client = Client()
    m._goal_checker_set_client = ParamClient()
    _, accepted, handle = paths.calls[0]
    accepted.set_result(handle)
    monkeypatch.setattr(m, '_plan_invalid_reason', lambda *a: None)
    m._monotonic.value += 3.1
    m.tick()
    return m, paths, handle, published


def stopped(m, scans=True):
    """Fresh stop evidence, then (by default) fresh scans and odometry, then a tick."""
    hold_still(m)
    if scans:
        m.test_ros_ns += 50_000_000
        feed(m)
    m.tick()


def test_a_timeout_stops_the_search_and_waits_for_the_stop(manager_factory, monkeypatch):
    m, paths, handle, _ = timed_out(manager_factory, monkeypatch)
    assert m.state == MissionState.RETURNING_TO_START
    assert handle.cancels == 1                                  # the timed-out check is cancelled
    assert m.parking_plan.failure_reason == 'planner_timeout' and not m.parking_plan.ok
    assert m.search_recovery['code'] == 'planning_timeout' and m.search_recovery['outcome'] is None
    assert len(paths.calls) == 1 and m.navigation_client.calls == []   # nothing new before the stop
    m.tick()
    assert len(paths.calls) == 1 and m.navigation_client.calls == []


def test_a_safe_return_goes_home_and_still_fails_as_planning_timeout(manager_factory, monkeypatch):
    m, paths, _, published = timed_out(manager_factory, monkeypatch)
    stopped(m)
    home_request = paths.calls[-1][0]
    assert len(paths.calls) == 2 and home_request.use_start
    assert (home_request.goal.pose.position.x, home_request.goal.pose.position.y) == pytest.approx(START[:2])
    assert m.navigation_client.calls == []                      # no motion before the route exists
    finish(paths)
    assert m.search_recovery['outcome'] == 'returning'
    answer(m._goal_checker_set_client)                          # the shared checker for the return
    assert navigation_result(m.navigation_client) == pytest.approx(START)
    assert m.state == MissionState.MISSION_FAILED               # never MISSION_COMPLETE
    assert m.last_issue.code == 'planning_timeout'
    assert m.last_issue.reason.startswith('planning_timeout: ComputePathToPose deadline expired')
    assert m.search_recovery['outcome'] == 'returned home'


def test_a_failed_return_reports_the_timeout_it_followed(manager_factory, monkeypatch):
    m, paths, _, _ = timed_out(manager_factory, monkeypatch)
    stopped(m)
    finish(paths)
    answer(m._goal_checker_set_client)
    navigation_result(m.navigation_client, status=GoalStatus.STATUS_ABORTED, error=104)
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'return_navigation_failed'
    assert 'after planning_timeout' in m.last_issue.reason
    assert m.search_recovery['outcome'].startswith('return failed')


def refused(m, problem):
    refused_with(m, 'planning_timeout', problem)


def refused_with(m, code, problem):
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == code
    assert problem in m.last_issue.reason and 'remaining stopped' in m.last_issue.reason
    assert m.search_recovery['outcome'].startswith('not started') and problem in m.search_recovery['outcome']
    assert m.navigation_client.calls == []                      # nothing was commanded
    calls = len(m.compute_path_client.calls)
    m._monotonic.value += 30.0
    m.tick()
    assert len(m.compute_path_client.calls) == calls and m.navigation_client.calls == []   # not retried


def test_no_route_home_keeps_the_robot_stopped(manager_factory, monkeypatch):
    m, paths, _, _ = timed_out(manager_factory, monkeypatch)
    stopped(m)
    finish(paths, status=GoalStatus.STATUS_ABORTED, error=304)
    refused(m, 'home_route_unavailable')


def test_an_unanswered_route_home_keeps_the_robot_stopped(manager_factory, monkeypatch):
    m, paths, _, _ = timed_out(manager_factory, monkeypatch)
    stopped(m)
    m._monotonic.value += m.parking_path_timeout_s + 0.1
    m.tick()
    refused(m, 'home_route_unavailable: ComputePathToPose deadline expired')
    finish(paths)                                               # a late route starts nothing
    assert m.navigation_client.calls == [] and m._goal_checker_set_client.calls == []


@pytest.mark.parametrize('problem,reason', [('missing', 'lidar_unavailable'),
                                            ('stale', 'lidar_stale'),
                                            ('odometry', 'odometry_stale')])
def test_missing_or_stale_sensors_keep_the_robot_stopped(manager_factory, monkeypatch, problem, reason):
    m, paths, _, _ = timed_out(manager_factory, monkeypatch)
    hold_still(m)
    m.test_ros_ns += 50_000_000
    if problem == 'stale':
        feed(m, stamp_ns=m.test_ros_ns - 2_000_000_000)
    else:
        feed(m)
        if problem == 'missing':
            m._scans.clear()
        else:
            m._execution_odom_received_s -= 5.0
    m.tick()
    refused(m, reason)
    assert len(paths.calls) == 1                                # no route home was even requested


def test_an_unconfirmed_stop_keeps_the_robot_stopped(manager_factory, monkeypatch):
    m, paths, _, _ = timed_out(manager_factory, monkeypatch)
    m._monotonic.value += m.motion_settlement_timeout_s + 1.0
    m.tick()
    refused(m, 'stop not confirmed')
    assert len(paths.calls) == 1


def test_unavailable_nav2_keeps_the_robot_stopped(manager_factory, monkeypatch):
    m, paths, _, _ = timed_out(manager_factory, monkeypatch)
    m.navigation_client.server_is_ready = lambda: False
    stopped(m)
    refused(m, 'nav2_unavailable')


def test_a_timeout_never_explores_or_reads_as_no_valid_candidate(manager_factory, monkeypatch):
    m, _, _, published = timed_out(manager_factory, monkeypatch)
    assert m.state != MissionState.EXPLORING_AROUND_G1
    assert m.parking_plan.failure_reason == 'planner_timeout'
    assert 'no_valid_candidate' not in m._parking_failure


def test_with_driving_off_the_plain_timeout_failure_stands(manager_factory, monkeypatch):
    m, paths, _ = running(manager_factory, monkeypatch)
    _, accepted, handle = paths.calls[0]
    accepted.set_result(handle)
    monkeypatch.setattr(m, '_plan_invalid_reason', lambda *a: None)
    m._monotonic.value += 3.1
    m.tick()
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert m.last_issue.code == 'planner_timeout' and m.search_recovery is None


def budget_spent(manager_factory, monkeypatch):
    """Driving on; the only route check the budget allows finds no route."""
    m, paths, published = running(manager_factory, monkeypatch)
    m.parking_navigation_enabled = True
    m.navigation_client = Client()
    m._goal_checker_set_client = ParamClient()
    m.g1_exploration_enabled = False
    m.path_check_budget = 1
    monkeypatch.setattr(m, '_plan_invalid_reason', lambda *a: None)
    finish(paths, error=208, status=GoalStatus.STATUS_ABORTED)
    m._next_path(now_ns=PLAN_NOW_NS)
    return m, paths, None, published


@pytest.mark.parametrize('trigger,code,plan_code', [(timed_out, 'planning_timeout', 'planner_timeout'),
                                                    (budget_spent, 'route_budget_exhausted',
                                                     'route_budget_exhausted')])
def test_timeout_and_budget_share_the_safe_return_under_their_own_codes(
        manager_factory, monkeypatch, trigger, code, plan_code):
    m, paths, _, _ = trigger(manager_factory, monkeypatch)
    assert m.state == MissionState.RETURNING_TO_START and m.navigation_client.calls == []
    assert m.search_recovery['code'] == code and m.parking_plan.failure_reason == plan_code
    stopped(m)
    finish(paths)                                               # the route home
    answer(m._goal_checker_set_client)
    assert navigation_result(m.navigation_client) == pytest.approx(START)
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == code


@pytest.mark.parametrize('trigger,code', [(timed_out, 'planning_timeout'),
                                          (budget_spent, 'route_budget_exhausted')])
def test_timeout_and_budget_both_stay_stopped_without_a_route_home(manager_factory, monkeypatch,
                                                                   trigger, code):
    m, paths, _, _ = trigger(manager_factory, monkeypatch)
    stopped(m)
    finish(paths, status=GoalStatus.STATUS_ABORTED, error=304)
    assert m.last_issue.code == code
    refused_with(m, code, 'home_route_unavailable')
