"""G1's localized position is checked against its head and torso in depth.

The fixture holds Gazebo depth crops of four views recorded on 2026-11-09 by
tools/parking_scenarios/capture_g1_views.py, at the robot poses of the 2026-11-08
workstation runs. ``table_in_front`` is the gz_no_feasible view whose detector surface
estimate (-5.673, -1.031) was a worktable's edge 1.25 m in front of G1, after which the
mission parked beside that estimate and reported MISSION_COMPLETE; ``panel_beside`` is
the gz_left_blocked view whose estimate was a panel's end. G1's ground truth is used
only to measure the error.
"""

import math
import os
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from ridgeback_autonomy.mission import target_orientation as to
from ridgeback_autonomy.mission.mission_manager_node import MissionState
from ridgeback_autonomy.mission.target_localization import StabilizedTargetPosition
from test_mission_manager import (  # noqa: F401 (fixture)
    FakePauseClient, enter_localization, localize, manager_factory, trigger_response,
)

DATA = np.load(os.path.join(os.path.dirname(__file__), 'data', 'g1_views_2026-11-09.npz'))
VIEWS = ('table_in_front', 'panel_beside', 'clear_view', 'side_view_close')
# Error to G1's ground truth that a usable position must stay within: the planner's
# reference margin for the generic estimate.
MAX_ERROR_M = 0.10


def view(name):
    return {key[len(name) + 1:]: DATA[key] for key in DATA.files if key.startswith(name + '_')}


def depth_m(v):
    return v['depth_mm'].astype(float) / 1000.0


def points(v):
    return to.depth_to_map_points(depth_m(v), v['intrinsics'], v['camera_rotation'],
                                  v['camera_translation'])


def head(v):
    box = to.box_pixels(v['box'], (0, 0), v['depth_mm'].shape)
    one = to.head_observation(depth_m(v), box, v['intrinsics'], v['camera_rotation'],
                              v['camera_translation'])
    return to.combined_head([one] * to.MIN_AGREEING_FRAMES)


def corrected(v, surface):
    """The mission's sequence: the head check, the legs' axis, then the torso check."""
    camera = v['camera_translation'][:2]
    h = head(v)
    center = surface if to.surface_is_g1(to.surface_offset_from_head(surface, h, camera)) \
        else to.head_axis(h, camera)
    axis = to.combined_axis([to.estimate_body_axis(points(v), center, camera)] * 3)
    return axis if axis is not None and math.dist(axis, center) <= 0.25 else center


def error(v, xy):
    return math.dist(xy, v['g1_truth'])


def test_the_head_region_ranges_g1_whatever_stands_in_front_of_it():
    for name in VIEWS:
        v = view(name)
        h = head(v)
        camera = v['camera_translation'][:2]
        truth_range = math.dist(v['g1_truth'], camera)
        assert h is not None, name
        assert -0.20 <= truth_range - h.range_m <= 0.15, (name, h.range_m, truth_range)


def test_the_recorded_mislocalizations_are_other_objects_in_the_box():
    for name, ahead in (('table_in_front', 1.0), ('panel_beside', 0.8)):
        v = view(name)
        offset = to.surface_offset_from_head(v['surface'], head(v), v['camera_translation'][:2])
        assert offset > ahead and not to.surface_is_g1(offset), (name, offset)
    v = view('clear_view')
    assert to.surface_is_g1(to.surface_offset_from_head(v['surface'], head(v),
                                                        v['camera_translation'][:2]))


def test_corrected_positions_are_on_g1():
    for name in ('table_in_front', 'panel_beside', 'clear_view'):
        v = view(name)
        assert error(v, v['surface']) > 0.95 or name == 'clear_view'
        xy = corrected(v, tuple(v['surface']))
        assert error(v, xy) <= MAX_ERROR_M, (name, xy, error(v, xy))
        assert to.torso_confirmed([to.torso_points(points(v), xy)] * 3), name


def test_no_torso_stands_on_the_worktable_edge_the_run_parked_beside():
    v = view('table_in_front')
    assert to.torso_points(points(v), tuple(v['surface'])) == 0
    assert not to.torso_confirmed([to.torso_points(points(v), tuple(v['surface']))] * 5)


def test_an_empty_head_region_gives_no_head():
    v = view('clear_view')
    far = np.zeros_like(v['depth_mm'], dtype=float)
    box = to.box_pixels(v['box'], (0, 0), far.shape)
    assert to.head_observation(far, box, v['intrinsics'], v['camera_rotation'],
                               v['camera_translation']) is None
    assert to.combined_head([None, None, head(v)]) is None


def test_box_pixels_scale_a_box_from_a_larger_colour_image():
    assert to.box_pixels((100, 50, 200, 150), (1280, 960), (480, 640)) == (50, 25, 101, 76)


def depth_image(v):
    data = depth_m(v).astype(np.float32)
    return Image(height=data.shape[0], width=data.shape[1], encoding='32FC1', data=data.tobytes())


def mission_at(manager_factory, v, *, check=True):
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')),
                        overrides=[Parameter('target_body_check', value=check)])
    enter_localization(m)
    m.target_position = StabilizedTargetPosition(
        map_x_m=float(v['surface'][0]), map_y_m=float(v['surface'][1]), sample_count=5,
        source='euclidean_reconstruction', reference='generic_target_position',
        is_foot_midpoint=False, first_stamp_ns=0, last_stamp_ns=0)
    m._camera_to_map = lambda: (v['camera_rotation'], v['camera_translation'])
    m._camera_intrinsics = tuple(float(k) for k in v['intrinsics'])
    m._localization_box = (tuple(float(b) for b in v['box']), (0, 0))
    for _ in range(5):
        m._on_depth(depth_image(v))
    return m


def test_the_mission_moves_the_recorded_table_edge_estimate_onto_g1(manager_factory):
    v = view('table_in_front')
    m = mission_at(manager_factory, v)
    assert m._observe_target_body() is None
    xy = (m.target_position.map_x_m, m.target_position.map_y_m)
    assert error(v, xy) <= MAX_ERROR_M, xy


def test_the_mission_refuses_a_position_without_g1_in_depth(manager_factory):
    v = view('table_in_front')
    m = mission_at(manager_factory, v)
    m._localization_box = None               # no head region: the table edge stays the seed
    reason = m._observe_target_body()
    assert reason is not None and 'head region' in reason
    assert (m.target_position.map_x_m, m.target_position.map_y_m) == tuple(v['surface'])


def test_an_unverified_position_is_never_localized_or_planned(manager_factory):
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')),
                        overrides=[Parameter('target_body_check', value=True)])
    enter_localization(m)
    m._observe_target_body = lambda: 'depth shows no G1 torso above (-5.673, -1.031)'
    localize(m)
    assert m.state == MissionState.LOCALIZE_G1
    assert m.target_position is None
    assert m.last_issue.code == 'target_position_unverified'
    assert m.parking_plan is None


def test_an_unverified_relocalization_keeps_the_previous_position(manager_factory):
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')),
                        overrides=[Parameter('target_body_check', value=True)])
    enter_localization(m)
    m._observe_target_body = lambda: None
    localize(m)
    previous = m.target_position
    m.state = MissionState.LOCALIZE_G1
    m._observe_target_body = lambda: 'no consistent depth in the head region'
    m.target_position = replace(previous)
    localize(m, first_stamp_ns=9_000_000_000)
    assert m.state == MissionState.LOCALIZE_G1
    assert m.target_position == previous


# The first localization turns the camera toward the confirmed detection when the
# stop left G1 off the camera axis.

def test_the_detection_ray_comes_from_the_box_column_at_its_stamp(manager_factory):
    m = manager_factory()
    m._camera_intrinsics = (384.0, 384.0, 320.0, 240.0)
    stamps = []
    m._camera_view_at = lambda stamp: stamps.append(stamp.nanoseconds) or (1.0, 2.0, 0.0)
    candidate = SimpleNamespace(bbox_xyxy=(80.0, 200.0, 120.0, 400.0))   # centre column 100
    origin, (ux, uy) = m._detection_ray(candidate, 7_000_000_000, 640)
    assert stamps == [7_000_000_000] and origin == (1.0, 2.0)
    assert math.isclose(math.atan2(uy, ux), math.atan2(220.0, 384.0), abs_tol=1e-9)   # left


def test_the_turn_is_only_for_g1_off_the_camera_axis(manager_factory):
    m = manager_factory()
    m._current_robot_pose = lambda: (1.0, 2.0, 0.3)
    m._camera_view = lambda: (1.2, 2.0, 0.3)
    m._confirmed_ray = ((1.0, 1.9), (math.cos(0.8), math.sin(0.8)))
    goal, how = m._first_view_turn()
    assert goal[:2] == (1.0, 2.0) and math.isclose(goal[2], 0.8)
    assert 'confirmed G1 detection' in how
    m._confirmed_ray = ((1.0, 1.9), (math.cos(0.35), math.sin(0.35)))
    assert m._first_view_turn() is None
    m._confirmed_ray = None
    assert m._first_view_turn() is None


def test_a_weak_view_is_admitted_only_on_the_confirmed_ray():
    from ridgeback_autonomy.mission.target_localization import _away_from_known
    ray = ((0.0, 0.0), (1.0, 0.0))
    assert not _away_from_known(8.7, 0.5, None, ray, 0.75)
    assert _away_from_known(8.7, 0.9, None, ray, 0.75)
    assert _away_from_known(-1.0, 0.0, None, ray, 0.75)          # behind the camera
    assert not _away_from_known(1.0, 1.0, (1.2, 1.2), ray, 0.75)  # a known G1 decides first


def test_the_first_localization_starts_after_the_turn_even_when_it_fails(manager_factory):
    from test_mission_manager import confirm_target, settle_motion, start_search
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')))
    m._first_view_turn = lambda: ((1.0, 2.0, 1.0), 'turning toward the confirmed G1 detection')
    start_search(m)
    confirm_target(m)
    # No goal checker service in this test: the turn fails before it is sent, and the
    # mission localizes from where it stopped instead of failing.
    states = [t.current for t in m.transition_history]
    assert MissionState.RETURNING_TO_OBSERVATION in states
    assert m.state == MissionState.SETTLING_MOTION
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1


def test_a_timeout_after_an_unverified_position_says_why(manager_factory):
    from test_mission_manager import WallClock
    clock = WallClock()
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')), clock=clock,
                        overrides=[Parameter('target_body_check', value=True)])
    enter_localization(m)
    m._observe_target_body = lambda: 'depth shows no G1 torso above (-5.673, -1.031)'
    localize(m)
    clock.value += m.localization_timeout_s + 0.1
    m.tick()
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'target_localization_timeout'
    assert 'not verified in depth' in m.last_issue.reason


def localize_weak(m, score, first_stamp_ns=4_000_000_000):
    from test_mission_manager import detection, measurement
    for offset in range(5):
        stamp_ns = first_stamp_ns + offset
        m.process_detection(detection(stamp_ns, score=score))
        msg = measurement(stamp_ns)
        msg.scores = [score]
        m.process_measurement(msg, now_ns=stamp_ns)


def test_a_weak_static_view_on_the_confirmed_ray_localizes_g1(manager_factory):
    """Test A on 2026-11-09: G1 confirmed at 0.556 while moving, about 9 m away; the
    static view after the stop scored 0.44 on every frame and the first localization
    timed out under the 0.50 floor."""
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')))
    enter_localization(m)
    # The fixture's measurement puts G1 about 2 m ahead of the robot at (1, 2), yaw 0.25.
    m._confirmed_ray = ((1.0, 2.0), (math.cos(0.25), math.sin(0.25)))
    localize_weak(m, 0.44)
    assert m.state == MissionState.G1_LOCALIZED


def test_a_weak_view_off_the_confirmed_ray_is_not_g1(manager_factory):
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')))
    enter_localization(m)
    m._confirmed_ray = ((1.0, 2.0), (math.cos(1.25), math.sin(1.25)))   # 57 deg away
    localize_weak(m, 0.44)
    assert m.state == MissionState.LOCALIZE_G1
    assert m.last_issue.reason.startswith('weak_detection_away_from_known_target')


def test_search_still_confirms_g1_only_at_the_full_floor(manager_factory):
    from test_mission_manager import detection, start_search
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')))
    m._confirmed_ray = ((1.0, 2.0), (1.0, 0.0))
    start_search(m)
    for offset in range(5):
        m.process_detection(detection(1_000_000_000 + offset, score=0.44))
    assert m.state == MissionState.SEARCH_G1


def observe_again(m, stamp_ns, *, forward=2.0, score=0.8):
    from test_mission_manager import detection, measurement
    m.process_detection(detection(stamp_ns, score=score))
    msg = measurement(stamp_ns)
    msg.scores = [score]
    msg.euclidean_reconstruction_forward_m = [forward]
    msg.euclidean_reconstruction_distance_m = [math.hypot(forward, 0.2)]
    m.process_measurement(msg, now_ns=stamp_ns)


def test_a_still_robot_keeps_g1_fresh_only_by_seeing_it_where_it_was(manager_factory):
    """Test A rerun on 2026-11-09: the settle wait, the path checks of a 6.2 m route and the
    stationary wait before driving took longer than the 5 s target age, and a verified
    plan ended PARKING_PLANNING_FAILED (target_stale) with G1 in view the whole time."""
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')))
    enter_localization(m)
    localize(m)
    assert m.state == MissionState.G1_LOCALIZED
    before = m.target_position
    observe_again(m, 9_000_000_000, score=0.44)
    assert m.target_position.last_stamp_ns == 9_000_000_000
    assert (m.target_position.map_x_m, m.target_position.map_y_m) == (before.map_x_m, before.map_y_m)
    observe_again(m, 9_100_000_000, forward=3.5)          # 1.5 m away: not this G1
    assert m.target_position.last_stamp_ns == 9_000_000_000


def test_the_observation_pose_follows_the_latest_verified_localization(manager_factory):
    """Run 3 of Test A on 2026-11-09 first localized G1 from 10 m, relocalized it from
    1.7 m, and later failed returning to the 10 m pose before its deadline."""
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')))
    enter_localization(m)
    localize(m)
    first = m._observation_pose
    m.tf_buffer.x, m.tf_buffer.y = 1.3, 2.1
    m.state = MissionState.LOCALIZE_G1
    m.localization_adapter.reset()
    localize(m, first_stamp_ns=9_000_000_000)
    assert m.state == MissionState.G1_LOCALIZED
    assert m._observation_pose[:2] == pytest.approx((1.3, 2.1)) and m._observation_pose != first


def test_after_a_turn_a_failed_first_localization_turns_back_once(manager_factory):
    """Run 4 of Test A on 2026-11-09: after the turn centred G1 about 7 m away, the static
    view scored below 0.40 on every frame; the views before the turn had scored 0.42-0.52."""
    from test_mission_manager import WallClock, confirm_target, settle_motion, start_search
    clock = WallClock()
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')), clock=clock)
    m._first_view_turn = lambda: ((1.0, 2.0, 1.0), 'turning toward the confirmed G1 detection')
    start_search(m)
    confirm_target(m)
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    assert m._observation_fallback and m._observation_pose[:2] == pytest.approx((1.0, 2.0))
    clock.value += m.localization_timeout_s + 0.1
    m.tick()
    # The turn back to the pre-turn view (no goal checker service here: it is not sent,
    # and the mission localizes from where it stands), then one more localization.
    assert m.state == MissionState.SETTLING_MOTION and not m._observation_fallback
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    clock.value += 2 * m.localization_timeout_s + 0.2
    m.tick()
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'target_localization_timeout'


# Once a G1 is chosen, a second humanoid must not become the target.

def test_with_two_humanoids_in_view_the_chosen_one_is_kept():
    from ridgeback_autonomy.mission.target_detection import (
        TargetDetectionAdapter, TargetDetectionCandidate)
    adapter = TargetDetectionAdapter()
    chosen = TargetDetectionCandidate(0, 'humanoid robot', 0.52, (100.0, 200.0, 140.0, 400.0))
    other = TargetDetectionCandidate(1, 'humanoid robot', 0.71, (500.0, 200.0, 540.0, 400.0))
    assert adapter._select_relevant_candidate([chosen, other], 0.40) is other     # by score
    near_column_120 = lambda c: abs(0.5 * (c.bbox_xyxy[0] + c.bbox_xyxy[2]) - 120.0)
    assert adapter._select_relevant_candidate([chosen, other], 0.40, near_column_120) is chosen


def test_a_confident_view_of_another_humanoid_is_not_sampled_as_the_known_g1(manager_factory):
    from test_mission_manager import detection, measurement
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')))
    enter_localization(m)
    localize(m)
    known = m.target_position
    m.state = MissionState.LOCALIZE_G1
    m.localization_adapter.reset()
    for offset in range(5):          # a confident humanoid 1.5 m beyond the known G1
        stamp_ns = 9_000_000_000 + offset
        m.process_detection(detection(stamp_ns, score=0.9))
        msg = measurement(stamp_ns)
        msg.euclidean_reconstruction_forward_m = [3.5]
        msg.euclidean_reconstruction_distance_m = [math.hypot(3.5, 0.2)]
        m.process_measurement(msg, now_ns=stamp_ns)
    assert m.state == MissionState.LOCALIZE_G1 and m.target_position == known
    assert m.last_issue.reason.startswith('detection_away_from_known_target')


def test_the_expected_column_follows_the_chosen_g1(manager_factory):
    from test_mission_manager import detection
    m = manager_factory()
    m._camera_intrinsics = (384.0, 384.0, 320.0, 240.0)
    m._camera_view_at = lambda stamp: (0.0, 0.0, 0.0)
    m._confirmed_ray = ((0.0, 0.0), (math.cos(0.3), math.sin(0.3)))   # 0.3 rad to the left
    msg = detection(5_000_000_000)
    msg.count, msg.labels, msg.scores = 2, ['humanoid robot'] * 2, [0.5, 0.7]
    msg.bbox_xyxy = [180.0, 200.0, 220.0, 400.0, 500.0, 200.0, 540.0, 400.0]
    distance = m._expected_column_distance(msg)
    left = SimpleNamespace(bbox_xyxy=(180.0, 200.0, 220.0, 400.0))
    right = SimpleNamespace(bbox_xyxy=(500.0, 200.0, 540.0, 400.0))
    assert distance(left) < 10.0 < distance(right)      # expected column 320 - 384 tan 0.3 = 202


def test_g1_localized_from_inside_its_keep_out_steps_back_before_planning(manager_factory):
    """Demo attempt 2 on 2026-11-11: the search ended with the camera 0.4 m from G1, the
    robot inside G1's keep-out, and every route was rejected (path_g1_keepout)."""
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')))
    enter_localization(m)
    m._step_back_goal = lambda robot, view, target, rng, max_step_m=1.0: (0.0, 2.0, 0.25)
    m._camera_view = lambda: (1.27, 2.07, 0.25)
    localize(m)                                # G1 about 2 m ahead: outside the keep-out
    assert m.state == MissionState.G1_LOCALIZED
    m2 = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')))
    enter_localization(m2)
    m2._step_back_goal = lambda robot, view, target, rng, max_step_m=1.0: (0.0, 2.0, 0.25)
    m2._camera_view = lambda: (1.27, 2.07, 0.25)
    from test_mission_manager import detection, measurement
    for offset in range(5):                      # G1 0.2 m ahead of the front: inside the keep-out
        stamp_ns = 4_000_000_000 + offset
        m2.process_detection(detection(stamp_ns))
        msg = measurement(stamp_ns)
        msg.euclidean_reconstruction_forward_m = [0.2]
        msg.euclidean_reconstruction_lateral_m = [0.0]
        msg.euclidean_reconstruction_distance_m = [0.2]
        m2.process_measurement(msg, now_ns=stamp_ns)
    reasons = [t.reason for t in m2.transition_history]
    assert any('inside its keep-out; stepping' in r for r in reasons)
    assert m2._planning_step_back_done


def test_a_re_seeded_g1_is_kept_fresh_by_views_of_the_same_surface(manager_factory):
    """Gazebo standard on 2026-11-14: the head check moved G1 1.0 m off the worktable
    edge the detector measured; new samples of that same edge were compared with the
    moved position, nothing was renewed, and the plan ended target_stale."""
    m = manager_factory(pause_client=FakePauseClient(trigger_response(True, 'safe')))
    enter_localization(m)
    localize(m)
    surface = (m.target_position.map_x_m, m.target_position.map_y_m)
    m.target_position = replace(m.target_position, map_x_m=surface[0] + 1.0)   # re-seeded
    m._target_surface_xy = surface
    observe_again(m, 9_000_000_000)
    assert m.target_position.last_stamp_ns == 9_000_000_000
    assert m.target_position.map_x_m == surface[0] + 1.0                       # not moved
    m._target_surface_xy = None                                                 # the old rule
    observe_again(m, 9_100_000_000)
    assert m.target_position.last_stamp_ns == 9_000_000_000
