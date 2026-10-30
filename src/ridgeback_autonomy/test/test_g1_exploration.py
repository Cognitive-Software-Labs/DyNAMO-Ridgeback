"""G1-centred mapping with the shared explorer before parking planning."""

import math

import pytest
from action_msgs.msg import GoalStatus

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from test_mission_manager import (  # noqa: F401 (fixture)
    PLAN_NOW_NS, FakePauseClient, FakeTfBuffer, enter_localization, guard_status, localize,
    manager_factory, occupancy_map, settle_motion, trigger_response,
)
from test_parking_feasibility import Client
from test_parking_mission_navigation import ParamClient, answer, navigation_result, release


def exploring_manager(manager_factory, *, resume_ready=True):
    m = manager_factory(tf_buffer=FakeTfBuffer(stamp_ns=PLAN_NOW_NS),
                        pause_client=FakePauseClient(trigger_response(True, 'safe')))
    m.resume_client = FakePauseClient(trigger_response(True, 'resumed'), ready=resume_ready)
    m.navigation_client = Client()
    m._goal_checker_set_client = ParamClient()
    aim_camera(m)
    m.focus = []
    m.focus_publisher.publish = m.focus.append
    m._on_map(occupancy_map(value=-1))   # nothing around G1 mapped yet
    enter_localization(m)
    localize(m)
    return m


def aim_camera(m, offset=0.0):
    """The optical axis at the robot, ``offset`` rad from the bearing to the G1 estimate."""
    def view():
        t, x, y = m.target_position, m.tf_buffer.x, m.tf_buffer.y
        return x, y, math.atan2(t.map_y_m - y, t.map_x_m - x) + offset
    m._camera_view = view


def back_at_observation(m, status=GoalStatus.STATUS_SUCCEEDED):
    """Apply the observation goal checker, then finish the one observation goal."""
    assert answer(m._goal_checker_set_client) == {
        'general_goal_checker.xy_goal_tolerance': 0.03,
        'general_goal_checker.yaw_goal_tolerance': 0.10}
    return navigation_result(m.navigation_client, status=status)


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
    assert back_at_observation(m) == pytest.approx((1.0, 2.0, 0.25))
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


# Explicit termination: complete only when unknown space leaves no parking position
# undecided that keep-out and reach allow (its task cell and whole parked footprint).
import json
import math

import numpy as np
from std_msgs.msg import String

from ridgeback_autonomy.mission.parking_planner import (
    GENERIC_TARGET_REFERENCE, GridSnapshot, TargetReference, plan_parking, pose_geometry_passes)


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


def localized_on(manager_factory, unknown):
    """A manager that localizes G1 on a map with ``unknown(distance, angle)`` cells around it."""
    def manager():
        m = manager_factory(tf_buffer=FakeTfBuffer(stamp_ns=PLAN_NOW_NS),
                            pause_client=FakePauseClient(trigger_response(True, 'safe')))
        m.resume_client = FakePauseClient(trigger_response(True, 'resumed'))
        return m
    probe = exploring_manager(manager_factory)   # where the fixtures localize G1
    centre = (probe.target_position.map_x_m, probe.target_position.map_y_m)
    m = manager()
    m._on_map(ring_map(m, unknown, centre=centre))
    enter_localization(m)
    localize(m)
    assert (m.target_position.map_x_m, m.target_position.map_y_m) == pytest.approx(centre)
    return m


def test_mapped_donut_with_unknown_parked_footprints_still_explores(manager_factory):
    # The donut is mapped, but every parked footprint reaches the unknown ring beyond it.
    m = localized_on(manager_factory, lambda d, a: (d > 0.65) & (d < 1.4))
    assert m.state == MissionState.EXPLORING_AROUND_G1
    assert len(m.resume_client.calls) == 1
    before = m.g1_exploration_result['before']
    assert before['donut_unknown_cells'] == 0 and before['undecided_positions'] > 0


def test_unknown_space_no_parked_footprint_reaches_plans_without_exploring(manager_factory):
    # Unknown floor beyond every parked footprint, and under G1 inside the donut.
    m = localized_on(manager_factory, lambda d, a: (d > 1.6) | (d < 0.25))
    assert m.resume_client.calls == []
    assert m.g1_exploration_result is None
    assert m.state in (MissionState.PARKING_CANDIDATES_READY, MissionState.EVALUATING_PARKING_PATHS)


def test_undecided_positions_follow_the_planners_own_decisions(manager_factory):
    m = exploring_manager(manager_factory)
    t = m.target_position
    target = TargetReference(t.map_x_m, t.map_y_m, 'map', 0, GENERIC_TARGET_REFERENCE, False, '')
    for unknown in (lambda d, a: (d > 0.65) & (d < 1.4),                 # beyond the donut
                    lambda d, a: (d >= 0.3) & (d <= 0.6) & (a > 0),      # northern donut half
                    lambda d, a: (d > 0.9) & (np.abs(a - 2.0) < 0.4),    # one patch
                    lambda d, a: d < 0.25):                              # under G1 only
        msg = ring_map(m, unknown)
        m._on_map(msg)
        plan = plan_parking(GridSnapshot.from_msg(msg), target, m.planner_config, None)
        allowed = lambda x, y: bool(pose_geometry_passes(
            math.hypot(x - target.x, y - target.y), m.planner_config, plan.keepout_radius_m))
        expected = (sum(c.reason in ('footprint_unknown', 'footprint_out_of_bounds')
                        and allowed(c.task_x, c.task_y) for c in plan.candidates)
                    + sum(allowed(x, y) for x, y in plan.unknown_cells))
        assert m._g1_coverage()['undecided_positions'] == expected
    assert expected == 0     # nothing under G1 can block a parked footprint


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
    # The donut is mapped, but parked footprints still reach unknown floor beyond it.
    m._on_map(ring_map(m, lambda d, a: (d > 0.7) & (d < 1.2)))
    m.tick()
    assert m.state == MissionState.EXPLORING_AROUND_G1
    assert m._g1_coverage()['donut_unknown_cells'] == 0
    # Only floor no parked footprint reaches is still unknown: every position is decided.
    m._on_map(ring_map(m, lambda d, a: d > 1.6))
    m.tick()
    assert 'complete (positions_decided)' in finished(m)
    result = m.g1_exploration_result
    assert result['outcome'] == 'complete' and result['after']['undecided_positions'] == 0
    assert result['after']['footprint_area_unknown_fraction'] == 0   # the far floor is not required


def test_one_unknown_cell_under_parked_footprints_keeps_exploring(manager_factory):
    m = exploring_manager(manager_factory)
    m._on_map(ring_map(m, lambda d, a: (np.abs(d - 0.45) < 0.03) & (np.abs(a - 1.0) < 0.06)))
    assert m._g1_coverage()['undecided_positions'] >= 1
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
    assert m.navigation_client.calls == []             # the goal waits for its goal checker
    answer(m._goal_checker_set_client)
    assert len(m.navigation_client.calls) == 1


def test_handoff_state_reports_the_exploration_result(manager_factory):
    m = exploring_manager(manager_factory)
    states = []
    m.state_publisher.publish = states.append
    m._on_map(occupancy_map())
    m.tick()
    payload = json.loads(states[-1].data)
    assert payload['g1_exploration']['outcome'] == 'complete'
    assert payload['g1_exploration']['reason'] == 'positions_decided'
    assert payload['g1_exploration']['before']['donut_unknown_cells'] > 0


def test_failed_fresh_localization_after_exploration_never_plans(manager_factory):
    m = exploring_manager(manager_factory)
    first = m.parking_plan          # the failed plan-first attempt that started exploring
    assert first.failure_reason == 'no_free_donut_cells'
    m._on_map(occupancy_map())
    m.tick()
    back_at_observation(m)
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    m._monotonic.value += m.localization_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'target_localization_timeout'
    assert m.parking_plan is first
    assert len(m.navigation_client.calls) == 1   # bounded: no second observation goal


# Observation return: G1 was detected from exactly the observation pose, and the
# detector's score varies with the view (recorded 2026-10-30), so the return must
# restore that view and verify it before relocalizing.

def returning(manager_factory):
    m = exploring_manager(manager_factory)
    m._on_map(occupancy_map())
    m.tick()
    assert m.state == MissionState.RETURNING_TO_OBSERVATION
    return m


def restored(m):
    request, _ = m._goal_checker_set_client.calls[-1]
    return {p.name: p.value.double_value for p in request.parameters} == {
        'general_goal_checker.xy_goal_tolerance': 0.40,
        'general_goal_checker.yaw_goal_tolerance': 0.40}


def test_precise_return_restores_the_shared_checker_and_relocalizes_after_a_verified_stop(
        manager_factory):
    m = returning(manager_factory)
    assert m.navigation_client.calls == []             # no goal before the tighter checker
    back_at_observation(m)
    assert len(m.navigation_client.calls) == 1
    assert restored(m)                                 # restored as soon as Nav2 released it
    assert m.state == MissionState.SETTLING_MOTION
    m.tick()
    assert m.state == MissionState.SETTLING_MOTION     # no stop evidence yet: no check, no localization
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    assert len(m.navigation_client.calls) == 1


@pytest.mark.parametrize('dx,dyaw', [(0.0, 0.30), (0.10, 0.0)])
def test_nav2_success_outside_the_observation_view_is_not_trusted(manager_factory, dx, dyaw):
    m = returning(manager_factory)
    back_at_observation(m)
    m.tf_buffer.x += dx                                # where the robot actually stopped
    m.tf_buffer.yaw += dyaw
    settle_motion(m)
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'observation_pose_not_reached'
    assert restored(m) and len(m.navigation_client.calls) == 1


def test_stop_within_the_checker_plus_drift_allowance_is_accepted(manager_factory):
    m = returning(manager_factory)
    back_at_observation(m)
    m.tf_buffer.x += 0.035                             # 0.03 m checker + 0.01 m stop drift
    m.tf_buffer.yaw += 0.11                            # 0.10 rad checker + 0.02 rad stop drift
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1


def test_g1_outside_the_usable_camera_view_fails_before_relocalizing(manager_factory):
    m = returning(manager_factory)
    aim_camera(m, offset=0.8)                          # G1 0.8 rad off the optical axis
    back_at_observation(m)
    settle_motion(m)
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'g1_outside_camera_view'


def test_missing_camera_transform_is_not_treated_as_a_view_of_g1(manager_factory):
    m = returning(manager_factory)
    m._camera_view = lambda: None
    back_at_observation(m)
    settle_motion(m)
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'observation_view_unverified'


def test_obstructed_observation_pose_fails_once_and_restores_the_checker(manager_factory):
    m = returning(manager_factory)
    back_at_observation(m, status=GoalStatus.STATUS_ABORTED)
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'observation_return_failed'
    assert restored(m) and len(m.navigation_client.calls) == 1
    m.tick()
    assert len(m.navigation_client.calls) == 1        # no retry


def test_unreached_heading_times_out_cancels_and_restores_the_checker(manager_factory):
    m = returning(manager_factory)
    answer(m._goal_checker_set_client)
    _, accepted, handle = m.navigation_client.calls[-1]
    accepted.set_result(handle)
    m._monotonic.value += m.parking_navigation_timeout_s + 1.0
    m.tick()
    assert handle.cancels == 1
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'observation_return_failed'
    assert restored(m)
    release(handle)                                    # the cancelled goal is still owned until here
    m.tick()
    assert len(m.navigation_client.calls) == 1


def test_observation_goal_is_never_sent_with_the_loose_checker(manager_factory):
    m = exploring_manager(manager_factory)
    m._goal_checker_set_client.ready = False
    m._on_map(occupancy_map())
    m.tick()
    assert m.state == MissionState.MISSION_FAILED
    assert 'observation goal tolerance not applied' in m.last_issue.reason
    assert m.navigation_client.calls == []


# Progressive planning: plan first, explore only while poses are undecided, check newly
# valid poses at bounded checkpoints, and relocalize G1 by turning toward it in place.

def stand(m, x, y, yaw):
    """The robot stands at (x, y, yaw), with the camera looking along the base heading."""
    m.tf_buffer.x, m.tf_buffer.y, m.tf_buffer.yaw = x, y, yaw
    m._camera_view = lambda: (m.tf_buffer.x, m.tf_buffer.y, m.tf_buffer.yaw)


def markers(m):
    published = []
    m.parking_marker_publisher.publish = published.append
    return published


def test_valid_poses_skip_exploration_even_with_undecided_sectors(manager_factory):
    # North of G1 is unmapped, but the mapped south already has valid parking poses.
    m = localized_on(manager_factory, lambda d, a: (d < 1.4) & (a > 0))
    assert m._g1_coverage()['undecided_positions'] > 0
    assert m.resume_client.calls == [] and m.g1_exploration_result is None
    assert m.state in (MissionState.PARKING_CANDIDATES_READY, MissionState.EVALUATING_PARKING_PATHS)


def test_newly_valid_poses_end_a_round_after_its_minimum_time_without_selecting_one(
        manager_factory):
    m = exploring_manager(manager_factory)
    published = markers(m)
    m._on_map(ring_map(m, lambda d, a: (d < 1.4) & (a > 0)))   # the south is mapped now
    m.tick()
    assert m.state == MissionState.EXPLORING_AROUND_G1          # round younger than its minimum
    donut = [mk for array in published for mk in array.markers if mk.ns == 'parking/radial_1deg']
    assert any(mk.id == 0 and mk.points for mk in donut)        # green rays drawn while exploring
    assert not any(mk.ns.startswith('parking/selected')
                   for array in published for mk in array.markers)
    m._monotonic.value += m.g1_exploration_min_round_s
    m.tick()                                                     # same map: the count is kept
    assert 'checkpoint (parking_candidates_available)' in finished(m)
    assert m.g1_exploration_result['rounds'] == 1


def test_poses_an_earlier_attempt_considered_trigger_no_checkpoint(manager_factory):
    m = exploring_manager(manager_factory)
    msg = ring_map(m, lambda d, a: (d < 1.4) & (a > 0))
    m._on_map(msg)
    assert m._refresh_g1_preview(m._monotonic()) > 0
    t = m.target_position
    plan = plan_parking(GridSnapshot.from_msg(msg), TargetReference(
        t.map_x_m, t.map_y_m, 'map', 0, GENERIC_TARGET_REFERENCE, False, ''), m.planner_config, None)
    m._g1_planned_cells = {(round(c.task_x / 0.05), round(c.task_y / 0.05))
                           for c in plan.candidates if c.status in ('valid', 'selected')}
    m._on_map(ring_map(m, lambda d, a: (d < 1.4) & (a > 0)))
    m._g1_preview_next_s = 0.0
    assert m._refresh_g1_preview(m._monotonic()) == 0


def finish_round_away_from_the_observation_pose(manager_factory):
    m = exploring_manager(manager_factory)
    t = m.target_position
    stand(m, t.map_x_m - 1.9, t.map_y_m - 0.6, -1.0)           # G1 2 m away, off to the side
    m._on_map(occupancy_map())                                  # every pose decided
    m.tick()
    assert m.state == MissionState.RETURNING_TO_OBSERVATION
    return m


def test_after_exploring_g1_is_relocalized_by_turning_toward_it_in_place(manager_factory):
    m = finish_round_away_from_the_observation_pose(manager_factory)
    t = m.target_position
    assert 'turning toward G1 in place' in m.transition_history[-1].reason
    goal = back_at_observation(m)
    assert goal[:2] == pytest.approx((t.map_x_m - 1.9, t.map_y_m - 0.6))
    assert goal[2] == pytest.approx(math.atan2(0.6, 1.9))
    m.tf_buffer.yaw = goal[2]                                    # stopped facing G1
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    assert len(m.navigation_client.calls) == 1                   # no drive back
    second = PLAN_NOW_NS + 10
    m.test_ros_ns = m.tf_buffer.stamp_ns = second + 4
    m._on_map(occupancy_map(stamp_ns=second + 4))
    localize(m, first_stamp_ns=second)
    assert m.state in (MissionState.PARKING_CANDIDATES_READY, MissionState.EVALUATING_PARKING_PATHS)


def test_failed_in_place_relocalization_falls_back_once_to_the_observation_pose(manager_factory):
    m = finish_round_away_from_the_observation_pose(manager_factory)
    m.tf_buffer.yaw = back_at_observation(m)[2]
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    m._monotonic.value += m.localization_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.RETURNING_TO_OBSERVATION
    assert back_at_observation(m) == pytest.approx((1.0, 2.0, 0.25))
    stand(m, 1.0, 2.0, 0.25)
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    m._monotonic.value += m.localization_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.MISSION_FAILED                # bounded: one fallback
    assert m.last_issue.code == 'target_localization_timeout'
    assert len(m.navigation_client.calls) == 2


def test_failed_turn_toward_g1_falls_back_to_the_observation_pose(manager_factory):
    m = finish_round_away_from_the_observation_pose(manager_factory)
    back_at_observation(m, status=GoalStatus.STATUS_ABORTED)
    assert m.state == MissionState.RETURNING_TO_OBSERVATION
    assert back_at_observation(m) == pytest.approx((1.0, 2.0, 0.25))
    assert m.state == MissionState.SETTLING_MOTION


def test_g1_too_close_for_the_camera_returns_to_the_observation_pose(manager_factory):
    m = exploring_manager(manager_factory)
    t = m.target_position
    stand(m, t.map_x_m - 1.5, t.map_y_m - 0.5, -1.0)           # 1.58 m: G1's feet out of view
    m._on_map(occupancy_map())
    m.tick()
    assert 'outside 1.7-3.5 m' in m.transition_history[-1].reason
    assert back_at_observation(m) == pytest.approx((1.0, 2.0, 0.25))


def checkpoint_and_relocalize(m, unknown):
    m._on_map(ring_map(m, lambda d, a: (d < 1.4) & (a > 0)))
    m._monotonic.value += m.g1_exploration_min_round_s
    m.tick()
    back_at_observation(m)                                       # the robot did not move
    settle_motion(m)
    second = PLAN_NOW_NS + 10 * (m._g1_rounds + 1)
    m.test_ros_ns = m.tf_buffer.stamp_ns = second + 4
    m._on_map(ring_map(m, unknown, stamp_ns=second + 4))
    localize(m, first_stamp_ns=second)


def test_checkpoint_without_a_valid_pose_resumes_exploring_in_bounded_rounds(manager_factory):
    m = exploring_manager(manager_factory)
    m.g1_exploration_max_rounds = 2
    checkpoint_and_relocalize(m, lambda d, a: d < 1.4)          # the new map shows nothing yet
    assert m.state == MissionState.EXPLORING_AROUND_G1
    assert len(m.resume_client.calls) == 2 and m.g1_exploration_result['rounds'] == 2
    checkpoint_and_relocalize(m, lambda d, a: d < 1.4)
    assert m.state == MissionState.PARKING_PLANNING_FAILED       # no third round
    assert len(m.resume_client.calls) == 2


def test_a_closed_exploration_is_not_resumed(manager_factory):
    m = exploring_manager(manager_factory)
    m._monotonic.value += 241.0
    m.tick()                                                     # timeout ends exploring for good
    back_at_observation(m)
    settle_motion(m)
    second = PLAN_NOW_NS + 10
    m.test_ros_ns = m.tf_buffer.stamp_ns = second + 4
    m._on_map(occupancy_map(value=-1, stamp_ns=second + 4))
    localize(m, first_stamp_ns=second)
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert len(m.resume_client.calls) == 1
