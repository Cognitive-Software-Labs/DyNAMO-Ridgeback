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
