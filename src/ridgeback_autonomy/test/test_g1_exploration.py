"""G1-centred mapping with the shared explorer before parking planning."""

import pytest

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from test_mission_manager import (  # noqa: F401 (fixture)
    PLAN_NOW_NS, FakePauseClient, FakeTfBuffer, enter_localization, localize, manager_factory,
    occupancy_map, settle_motion, trigger_response,
)
from test_parking_feasibility import Client
from test_parking_mission_navigation import navigation_result


def exploring_manager(manager_factory, *, resume_ready=True):
    m = manager_factory(tf_buffer=FakeTfBuffer(stamp_ns=PLAN_NOW_NS),
                        pause_client=FakePauseClient(trigger_response(True, 'safe')))
    m.resume_client = FakePauseClient(trigger_response(True, 'resumed'), ready=resume_ready)
    m.navigation_client = Client()
    m.focus = []
    m.focus_publisher.publish = m.focus.append
    m._on_map(occupancy_map(value=-1))   # nothing around G1 mapped yet
    enter_localization(m)
    localize(m)
    return m


def test_unmapped_parking_area_resumes_the_explorer_focused_on_g1(manager_factory):
    m = exploring_manager(manager_factory)
    assert m.state == MissionState.EXPLORING_AROUND_G1
    assert len(m.resume_client.calls) == 1
    focus = m.focus[-1]
    assert focus.header.frame_id == 'map'
    assert (focus.point.x, focus.point.y) == pytest.approx(
        (m.target_position.map_x_m, m.target_position.map_y_m))
    m.tick()
    assert m.state == MissionState.EXPLORING_AROUND_G1   # still unknown


def test_mapped_area_pauses_returns_to_observation_and_plans_from_fresh_localization(manager_factory):
    m = exploring_manager(manager_factory)
    m._on_map(occupancy_map())
    m.tick()
    assert m.focus[-1].header.frame_id == ''   # focus cleared
    assert m.state == MissionState.RETURNING_TO_OBSERVATION
    assert navigation_result(m.navigation_client) == pytest.approx((1.0, 2.0, 0.25))
    assert m.state == MissionState.SETTLING_MOTION
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    second = PLAN_NOW_NS + 10
    m.test_ros_ns = m.tf_buffer.stamp_ns = second + 4
    m._on_map(occupancy_map(stamp_ns=second + 4))
    localize(m, first_stamp_ns=second)
    assert m.state in (MissionState.PARKING_CANDIDATES_READY, MissionState.EVALUATING_PARKING_PATHS)
    assert len(m.resume_client.calls) == 1   # one exploration round only
    assert m.parking_plan.counts()['donut_unknown'] == 0


def test_footprint_entering_the_protected_region_stops_exploration(manager_factory):
    m = exploring_manager(manager_factory)
    m.tf_buffer.x, m.tf_buffer.y = m.target_position.map_x_m - 0.6, m.target_position.map_y_m
    m.tick()
    assert m.state in (MissionState.STOPPING_EXPLORATION, MissionState.RETURNING_TO_OBSERVATION)
    assert any('protected' in t.reason for t in m.transition_history)


def test_exploration_is_bounded_in_time(manager_factory):
    m = exploring_manager(manager_factory)
    m._monotonic.value += 241.0
    m.tick()
    assert any('timeout' in t.reason for t in m.transition_history
               if t.current == MissionState.STOPPING_EXPLORATION)


def test_without_an_explorer_the_mission_plans_with_the_current_map(manager_factory):
    m = exploring_manager(manager_factory, resume_ready=False)
    assert m.state == MissionState.PARKING_PLANNING_FAILED   # nothing mapped, nothing to explore with
    assert m.resume_client.calls == []


def test_explorer_focus_keeps_goals_in_the_annulus_around_g1():
    from types import SimpleNamespace
    from ridgeback_autonomy.frontier_explorer.frontier_explorer_node import FrontierExplorerNode
    owner = SimpleNamespace(_focus=None, _focus_keepout=1.0, _focus_radius=2.5,
                            grid_to_world=lambda p: p)
    assert FrontierExplorerNode._in_focus(owner, (40.0, 0.0))   # unfocused: global exploration
    owner._focus = (3.0, 0.0)
    in_focus = lambda x, y: FrontierExplorerNode._in_focus(owner, (x, y))
    assert in_focus(4.5, 0.0) and in_focus(3.0, -2.4)
    assert not in_focus(3.5, 0.0)    # inside G1's protected radius
    assert not in_focus(6.0, 0.0)    # outside the focus region
    # A goal advanced past the region is fine, one inside the protected radius is not.
    assert FrontierExplorerNode._in_focus(owner, (6.0, 0.0), outer=False)
    assert not FrontierExplorerNode._in_focus(owner, (3.5, 0.0), outer=False)
