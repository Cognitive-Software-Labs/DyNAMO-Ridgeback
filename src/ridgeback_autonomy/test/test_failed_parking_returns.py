"""The older failed-parking returns (``no_valid_candidate``, ``bounded_search_exhausted``,
``no_free_donut_cells``) take the guarded return the incomplete route search introduced:
stopped, fresh scans and odometry, Nav2 ready, outside G1's protected region and a route
home planned now; otherwise the robot stays stopped. A return still ends MISSION_FAILED
under the original code."""
from ridgeback_autonomy.mission.mission_manager_node import MissionState
from test_mission_manager import manager_factory  # noqa: F401 (fixture)
from test_parking_feasibility import ORDER, Client, evaluate, facing_manager, finish
from test_parking_mission_navigation import ParamClient, answer, navigation_result
from test_parking_timeout_recovery import refused_with, stopped


def no_route_anywhere(manager_factory, monkeypatch):
    m, client, _, _ = facing_manager(manager_factory, monkeypatch, tuple(ORDER))
    m.parking_navigation_enabled = True
    m.g1_exploration_enabled = False
    m.path_check_budget = 100
    m.navigation_client = Client()
    m._goal_checker_set_client = ParamClient()
    evaluate(m, client)
    return m, client


def test_bounded_search_exhaustion_returns_only_through_the_guarded_return(manager_factory, monkeypatch):
    m, client = no_route_anywhere(manager_factory, monkeypatch)
    assert m.state == MissionState.RETURNING_TO_START and m.navigation_client.calls == []
    assert m.parking_plan.failure_reason == 'bounded_search_exhausted'
    assert m.search_recovery['code'] == 'bounded_search_exhausted'
    routes = len(client.calls)
    stopped(m)
    assert len(client.calls) == routes + 1                       # the route home, nothing driven
    finish(client)
    answer(m._goal_checker_set_client)
    navigation_result(m.navigation_client)
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'bounded_search_exhausted'


def test_bounded_search_exhaustion_without_nav2_stays_stopped(manager_factory, monkeypatch):
    m, _ = no_route_anywhere(manager_factory, monkeypatch)
    m.navigation_client.server_is_ready = lambda: False
    stopped(m)
    refused_with(m, 'bounded_search_exhausted', 'nav2_unavailable')


def test_no_return_route_is_taken_from_inside_g1s_protected_region(manager_factory, monkeypatch):
    m, _ = no_route_anywhere(manager_factory, monkeypatch)
    t = m.target_position
    current = m._current_robot_pose
    m._current_robot_pose = lambda **kw: (t.map_x_m + 0.45, t.map_y_m, 0.0) if not kw else current(**kw)
    stopped(m)
    refused_with(m, 'bounded_search_exhausted', 'g1_protected_region')


def test_an_unconfirmed_stop_keeps_the_failed_parking_return_from_starting(manager_factory, monkeypatch):
    m, _ = no_route_anywhere(manager_factory, monkeypatch)
    m._monotonic.value += m.motion_settlement_timeout_s + 1.0
    m.tick()
    refused_with(m, 'bounded_search_exhausted', 'stop not confirmed')


# The remaining failed-parking returns: a retry that cannot proceed, and the last attempt.

def test_a_retry_that_cannot_relocalize_returns_only_through_the_guarded_return(manager_factory, monkeypatch):
    from test_parking_mission_navigation import back_at_observation, guarded_return, hold_still, vetoed
    from test_parking_mission_navigation import stopped as released_and_stopped
    m, handle, _, _ = vetoed(manager_factory, monkeypatch)
    released_and_stopped(m, handle)
    back_at_observation(m)
    hold_still(m)
    m.tick()
    m._monotonic.value += m.localization_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.RETURNING_TO_START
    assert m.search_recovery['code'] == 'parking_failed'
    assert 'target_localization_timeout' in m._parking_failure
    sent = len(m.navigation_client.calls)
    guarded_return(m)
    answer(m._goal_checker_set_client)
    navigation_result(m.navigation_client)
    assert len(m.navigation_client.calls) == sent + 1
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'parking_failed'


def test_a_retry_that_cannot_relocalize_stays_stopped_without_a_route_home(manager_factory, monkeypatch):
    from test_parking_mission_navigation import back_at_observation, hold_still, vetoed
    from test_parking_mission_navigation import stopped as released_and_stopped
    from test_parking_clearance import feed
    from action_msgs.msg import GoalStatus
    m, handle, _, _ = vetoed(manager_factory, monkeypatch)
    released_and_stopped(m, handle)
    back_at_observation(m)
    hold_still(m)
    m.tick()
    m._monotonic.value += m.localization_timeout_s + 1.0
    m.tick()
    hold_still(m)
    m.test_ros_ns += 50_000_000
    feed(m)
    m.tick()
    sent = len(m.navigation_client.calls)
    finish(m.compute_path_client, status=GoalStatus.STATUS_ABORTED, error=304)
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'parking_failed'
    assert 'home_route_unavailable' in m.last_issue.reason and 'remaining stopped' in m.last_issue.reason
    m._monotonic.value += 30.0
    m.tick()
    assert len(m.navigation_client.calls) == sent                # nothing commanded, not retried


def test_the_last_final_leg_departs_stops_again_and_stays_stopped_without_a_route_home(
        manager_factory, monkeypatch):
    from action_msgs.msg import GoalStatus
    from test_parking_departure import depart, final_leg_failed, stopped_near_g1
    from test_parking_mission_navigation import hold_still
    from test_parking_clearance import feed
    m, handle, pre, _ = final_leg_failed(manager_factory, monkeypatch, attempts=1)
    assert m.state == MissionState.RETURNING_TO_START and m.search_recovery['code'] == 'parking_failed'
    stopped_near_g1(m, handle)
    depart(m, pre)                                               # the checked straight departure
    routes = len(m.compute_path_client.calls)
    m.tick()
    assert len(m.compute_path_client.calls) == routes           # a fresh stop comes first
    hold_still(m)
    m.test_ros_ns += 50_000_000
    feed(m)
    m.tick()
    assert len(m.compute_path_client.calls) == routes + 1
    sent = len(m.navigation_client.calls)
    finish(m.compute_path_client, status=GoalStatus.STATUS_ABORTED, error=304)
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'parking_failed'
    assert 'home_route_unavailable' in m.last_issue.reason
    assert len(m.navigation_client.calls) == sent                # nothing after the departure
