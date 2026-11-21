"""The search ending without G1: once the explorer has no frontier left (its observation turns
done) and no detection vote is pending, exploration is paused and the mission fails as
``g1_not_found``, returning home only through the guarded return; otherwise it stays stopped."""
import pytest
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from test_mission_manager import (  # noqa: F401 (fixture)
    FakePauseClient, detection, manager_factory, start_search, trigger_response)
from test_parking_feasibility import Client, finish
from test_parking_mission_navigation import ParamClient, answer, navigation_result
from test_parking_timeout_recovery import refused_with, stopped

HOME = (1.0, 2.0, 0.25)


def searching(manager_factory, *, driving=True, pause=True):
    m = manager_factory(pause_client=FakePauseClient(trigger_response(pause, 'safe' if pause else 'latched')))
    m.parking_navigation_enabled = driving
    m.navigation_client, m.compute_path_client = Client(), Client()
    m._goal_checker_set_client = ParamClient()
    start_search(m)
    return m


def explorer_done(m, seconds, *, step=0.5, status='exploration_complete'):
    """The explorer repeats its status every tick while it has no frontier."""
    for _ in range(int(round(seconds / step))):
        if m.state != MissionState.SEARCH_G1:
            return
        m._on_exploration_status(String(data=status))
        m.tick()
        m._monotonic.value += step


def test_exhausted_frontiers_end_in_g1_not_found_and_a_guarded_return(manager_factory):
    m = searching(manager_factory)
    explorer_done(m, 9.0)
    assert m.state == MissionState.SEARCH_G1                     # not yet confirmed as complete
    explorer_done(m, 2.0)
    assert m.state == MissionState.RETURNING_TO_START
    assert len(m.pause_client.calls) == 1                        # exploration paused first
    assert m.search_recovery['code'] == 'g1_not_found'
    assert m.navigation_client.calls == [] and m.compute_path_client.calls == []
    stopped(m)                                                   # stop, then fresh sensors
    request = m.compute_path_client.calls[-1][0]
    assert request.use_start
    assert (request.goal.pose.position.x, request.goal.pose.position.y) == pytest.approx(HOME[:2])
    finish(m.compute_path_client)
    answer(m._goal_checker_set_client)
    assert navigation_result(m.navigation_client) == pytest.approx(HOME)
    assert m.state == MissionState.MISSION_FAILED                # never MISSION_COMPLETE
    assert m.last_issue.code == 'g1_not_found'
    assert m.search_recovery['outcome'] == 'returned home'


def test_without_a_route_home_the_robot_stays_stopped_as_g1_not_found(manager_factory):
    m = searching(manager_factory)
    explorer_done(m, 11.0)
    stopped(m)
    finish(m.compute_path_client, status=GoalStatus.STATUS_ABORTED, error=304)
    refused_with(m, 'g1_not_found', 'home_route_unavailable')


def test_stale_sensors_keep_the_robot_stopped_after_g1_not_found(manager_factory):
    m = searching(manager_factory)
    explorer_done(m, 11.0)
    stopped(m, scans=False)
    refused_with(m, 'g1_not_found', 'lidar_unavailable')
    assert m.compute_path_client.calls == []


def test_with_driving_off_g1_not_found_never_moves(manager_factory):
    m = searching(manager_factory, driving=False)
    explorer_done(m, 11.0)
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'g1_not_found'
    assert 'remaining stopped' in m.last_issue.reason
    assert m.navigation_client.calls == [] and m.compute_path_client.calls == []


def test_a_gap_in_the_reports_means_the_explorer_moved_on(manager_factory):
    m = searching(manager_factory)
    explorer_done(m, 8.0)
    m._monotonic.value += 3.0                                    # a new goal (or a look-around)
    m.tick()
    explorer_done(m, 8.0)
    assert m.state == MissionState.SEARCH_G1
    explorer_done(m, 3.0)
    assert m.state == MissionState.RETURNING_TO_START


def test_a_pending_detection_vote_holds_the_search_open(manager_factory):
    m = searching(manager_factory)
    m.process_detection(detection(1_000_000_000))                # one positive of the 3 needed
    explorer_done(m, 15.0)
    assert m.state == MissionState.SEARCH_G1 and m.pause_client.calls == []
    for k in range(5):                                           # the vote window empties
        m.process_detection(detection(2_000_000_000 + k, detected=False))
    explorer_done(m, 11.0)
    assert m.search_recovery['code'] == 'g1_not_found'


def test_g1_confirmed_while_the_explorer_reports_completion_is_still_localized(manager_factory):
    m = searching(manager_factory)
    explorer_done(m, 6.0)
    for k in range(3):
        m.process_detection(detection(1_000_000_000 + k))
    assert m.state not in (MissionState.SEARCH_G1, MissionState.RETURNING_TO_START,
                           MissionState.MISSION_FAILED)
    assert m._search_exhausted is None and m.search_recovery is None


def test_a_focused_round_or_another_state_never_ends_the_search(manager_factory):
    m = searching(manager_factory)
    explorer_done(m, 15.0, status='focus_exploration_complete')
    assert m.state == MissionState.SEARCH_G1


def test_a_latched_pause_waits_for_the_explorer_to_release_ownership(manager_factory):
    m = searching(manager_factory, pause=False)
    explorer_done(m, 11.0)
    assert m.state == MissionState.STOPPING_EXPLORATION        # a turn or goal is still owned
    assert m.navigation_client.calls == [] and m.search_recovery is None
