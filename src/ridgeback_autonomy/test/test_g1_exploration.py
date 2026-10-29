"""G1-centred mapping with the shared explorer before parking planning."""

import pytest

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from test_mission_manager import (  # noqa: F401 (fixture)
    PLAN_NOW_NS, FakePauseClient, FakeTfBuffer, enter_localization, guard_status, localize,
    manager_factory, occupancy_map, settle_motion, trigger_response,
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


# Explicit, donut-based termination: complete only when every donut cell is known.
import json
import math

import numpy as np
from std_msgs.msg import String

from ridgeback_autonomy.mission.parking_planner import (
    GENERIC_TARGET_REFERENCE, GridSnapshot, TargetReference, plan_parking)


def ring_map(m, unknown, *, stamp_ns=PLAN_NOW_NS, centre=None):
    """A free 6 m map with unknown cells where ``unknown(distance_from_G1, angle)`` holds."""
    msg = occupancy_map(stamp_ns=stamp_ns)
    cx, cy = centre or (m.target_position.map_x_m, m.target_position.map_y_m)
    g = GridSnapshot.from_msg(msg)
    rows, cols = np.mgrid[0:g.height, 0:g.width]
    x, y = g.cell_centers(cols, rows)
    mask = unknown(np.hypot(x - cx, y - cy), np.arctan2(y - cy, x - cx))
    msg.data = np.where(mask, -1, 0).astype(int).ravel().tolist()
    return msg


def finished(m):
    return next(t.reason for t in m.transition_history
                if t.current == MissionState.STOPPING_EXPLORATION and t.previous ==
                MissionState.EXPLORING_AROUND_G1)


def test_already_mapped_donut_skips_exploration_even_with_unknown_footprint_area(manager_factory):
    def manager():
        m = manager_factory(tf_buffer=FakeTfBuffer(stamp_ns=PLAN_NOW_NS),
                            pause_client=FakePauseClient(trigger_response(True, 'safe')))
        m.resume_client = FakePauseClient(trigger_response(True, 'resumed'))
        return m
    probe = exploring_manager(manager_factory)   # where the fixtures localize G1
    centre = (probe.target_position.map_x_m, probe.target_position.map_y_m)
    m = manager()
    m._on_map(ring_map(m, lambda d, a: (d > 0.65) & (d < 1.4), centre=centre))
    enter_localization(m)
    localize(m)
    assert (m.target_position.map_x_m, m.target_position.map_y_m) == pytest.approx(centre)
    assert m.resume_client.calls == []
    result = m.g1_exploration_result
    assert (result['outcome'], result['reason']) == ('not_needed', 'donut_already_mapped')
    assert result['before']['footprint_area_unknown_fraction'] > 0.9
    assert m.state != MissionState.EXPLORING_AROUND_G1


def test_skip_decision_counts_only_the_donut(manager_factory):
    m = exploring_manager(manager_factory)
    # Unknown only beyond the donut (0.6-1.4 m): the donut itself is fully mapped.
    m._on_map(ring_map(m, lambda d, a: (d > 0.65) & (d < 1.4)))
    cov = m._g1_coverage()
    assert cov['donut_unknown_cells'] == 0 and cov['footprint_area_unknown_fraction'] > 0.9
    # Unknown only inside the 0.30 m inner radius (under G1) does not count either.
    m._on_map(ring_map(m, lambda d, a: d < 0.25))
    assert m._g1_coverage()['donut_unknown_cells'] == 0


def test_coverage_uses_the_planners_donut_cells(manager_factory):
    m = exploring_manager(manager_factory)
    msg = ring_map(m, lambda d, a: (d >= 0.3) & (d <= 0.6) & (a > 0))   # northern half unknown
    m._on_map(msg)
    t = m.target_position
    plan = plan_parking(GridSnapshot.from_msg(msg), TargetReference(
        t.map_x_m, t.map_y_m, 'map', 0, GENERIC_TARGET_REFERENCE, False, ''), m.planner_config, None)
    cov = m._g1_coverage()
    assert cov['donut_unknown_cells'] == plan.counts()['donut_unknown'] > 0
    assert cov['donut_cells'] == sum(plan.counts()[k] for k in ('donut_free', 'donut_occupied',
                                                                'donut_unknown'))


def test_exploration_continues_while_coverage_improves_and_completes_at_zero(manager_factory):
    m = exploring_manager(manager_factory)
    before = m.g1_exploration_result['before']['donut_unknown_cells']
    for sector in (math.pi / 2, 0.0):   # unknown donut shrinks: 3/4, then 1/2
        m._on_map(ring_map(m, lambda d, a, s=sector: (d >= 0.3) & (d <= 0.6) & (a > s)))
        m.tick()
        assert m.state == MissionState.EXPLORING_AROUND_G1
        assert 0 < m._g1_coverage()['donut_unknown_cells'] < before
    # Only the footprint area beyond the donut is still unknown: the donut is complete.
    m._on_map(ring_map(m, lambda d, a: (d > 0.7) & (d < 1.2)))
    m.tick()
    assert 'complete (donut_mapped)' in finished(m)
    result = m.g1_exploration_result
    assert result['outcome'] == 'complete' and result['after']['donut_unknown_cells'] == 0
    assert result['after']['footprint_area_unknown_fraction'] > 0


def test_one_unknown_donut_cell_keeps_exploring(manager_factory):
    m = exploring_manager(manager_factory)
    m._on_map(ring_map(m, lambda d, a: (np.abs(d - 0.45) < 0.03) & (np.abs(a - 1.0) < 0.06)))
    assert m._g1_coverage()['donut_unknown_cells'] >= 1
    m.tick()
    assert m.state == MissionState.EXPLORING_AROUND_G1   # no percentage tolerance


def test_unreachable_remaining_cells_end_with_an_explicit_partial_result(manager_factory):
    m = exploring_manager(manager_factory)
    m._on_map(ring_map(m, lambda d, a: (d >= 0.3) & (d <= 0.6) & (np.abs(a) < 0.3)))
    m._on_exploration_status(String(data='exploration_complete'))   # global, not the focus
    m.tick()
    assert m.state == MissionState.EXPLORING_AROUND_G1
    m._on_exploration_status(String(data='focus_exploration_complete'))
    m.tick()
    assert 'partial (no_reachable_frontiers)' in finished(m)
    assert m.g1_exploration_result['after']['donut_unknown_cells'] > 0


def test_focus_complete_before_the_explorer_resumed_is_ignored(manager_factory):
    m = manager_factory(tf_buffer=FakeTfBuffer(stamp_ns=PLAN_NOW_NS),
                        pause_client=FakePauseClient(trigger_response(True, 'safe')))
    m.resume_client = FakePauseClient(None)   # resume response still outstanding
    m.navigation_client = Client()
    m.focus_publisher.publish = lambda msg: None
    m._on_map(occupancy_map(value=-1))
    enter_localization(m)
    localize(m)
    assert m.state == MissionState.EXPLORING_AROUND_G1
    m._on_exploration_status(String(data='focus_exploration_complete'))   # stale / pre-resume
    m.tick()
    assert m.state == MissionState.EXPLORING_AROUND_G1


def test_timeout_is_reported_as_partial(manager_factory):
    m = exploring_manager(manager_factory)
    m._monotonic.value += 241.0
    m.tick()
    assert 'partial (timeout)' in finished(m)
    assert m.g1_exploration_result['outcome'] == 'partial'
    assert m.g1_exploration_result['duration_s'] == pytest.approx(241.0)


def test_finishing_cancels_through_missionguard_before_any_navigation(manager_factory):
    m = exploring_manager(manager_factory)
    m.pause_client = FakePauseClient()
    m._on_map(occupancy_map())
    m.tick()
    assert m.focus[-1].header.frame_id == ''           # focus cleared
    assert m.state == MissionState.STOPPING_EXPLORATION
    m.pause_client.calls[-1][1].set_result(trigger_response(False, 'explorer goal cancelling'))
    m.process_guard_status(guard_status())             # goal still active: ownership held
    assert m.state == MissionState.STOPPING_EXPLORATION
    assert m.navigation_client.calls == []             # no observation goal before the handoff
    m.process_guard_status(guard_status(handoff_safe=True, goal_active=False, cancellation='none'))
    m.pause_client.calls[-1][1].set_result(trigger_response(True, 'safe'))   # fresh verification
    assert m.state == MissionState.RETURNING_TO_OBSERVATION
    assert len(m.navigation_client.calls) == 1


def test_handoff_state_reports_the_exploration_result(manager_factory):
    m = exploring_manager(manager_factory)
    states = []
    m.state_publisher.publish = states.append
    m._on_map(occupancy_map())
    m.tick()
    payload = json.loads(states[-1].data)
    assert payload['g1_exploration']['outcome'] == 'complete'
    assert payload['g1_exploration']['reason'] == 'donut_mapped'
    assert payload['g1_exploration']['before']['donut_unknown_cells'] > 0


def test_failed_fresh_localization_after_exploration_never_plans(manager_factory):
    m = exploring_manager(manager_factory)
    m._on_map(occupancy_map())
    m.tick()
    navigation_result(m.navigation_client)
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    m._monotonic.value += m.localization_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'target_localization_timeout'
    assert m.parking_plan is None
