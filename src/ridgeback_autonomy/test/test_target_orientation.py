"""G1 heading from depth points: a heading only when the forearms clearly show one."""

import math

import numpy as np
import pytest
from sensor_msgs.msg import Image

from ridgeback_autonomy.mission.target_orientation import (
    LEG_SURFACE_OFFSET_M, OrientationEstimate, combined_axis, combined_estimate, depth_metres,
    depth_to_map_points, estimate_body_axis, estimate_target_yaw,
)


RNG = np.random.default_rng(0)
G1 = (3.0, 0.0)


def body_frame_to_map(forward, left, height, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack((G1[0] + forward * c - left * s, G1[1] + forward * s + left * c, height), axis=1)


def cloud(yaw, *, bars=(-1, 1), tips_only=False, legs=True):
    """Measured G1 geometry: legs and torso near the axis, forearms forward at chest height."""
    parts = []
    n = 400
    if legs:
        for side in (-1, 1):
            parts.append((RNG.uniform(-0.04, 0.06, n), side * RNG.uniform(0.10, 0.16, n),
                          RNG.uniform(0.05, 0.60, n)))
        parts.append((RNG.uniform(-0.06, 0.08, n), RNG.uniform(-0.17, 0.17, n), RNG.uniform(0.6, 1.3, n)))
    for side in bars:
        start = 0.31 if tips_only else 0.05
        count = 25 if tips_only else 300
        parts.append((RNG.uniform(start, 0.37, count), side * RNG.uniform(0.12, 0.17, count),
                      RNG.uniform(0.84, 0.93, count)))
    forward, left, height = (np.concatenate(values) for values in zip(*parts))
    return body_frame_to_map(forward, left, height, yaw)


def error_deg(estimate, yaw):
    return abs(math.degrees(math.remainder(estimate.yaw_rad - yaw, 2 * math.pi)))


@pytest.mark.parametrize('yaw_deg', [0, 33, 90, 135, 180, 225, 270, 315])
def test_both_forearms_give_the_heading_at_every_angle(yaw_deg):
    yaw = math.radians(yaw_deg)
    estimate = estimate_target_yaw(cloud(yaw), G1)
    assert estimate is not None and error_deg(estimate, yaw) < 8.0


@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('yaw_deg', [90, 270])
def test_a_single_visible_forearm_is_not_taken_for_the_other_arm(yaw_deg, side):
    yaw = math.radians(yaw_deg)
    estimate = estimate_target_yaw(cloud(yaw, bars=(side,)), G1)
    assert estimate is not None and error_deg(estimate, yaw) < 8.0


def test_only_the_fingertips_past_the_torso_still_show_a_g1_facing_away():
    yaw = 0.0
    estimate = estimate_target_yaw(cloud(yaw, tips_only=True), G1)
    assert estimate is not None and error_deg(estimate, yaw) < 12.0


def test_the_estimate_ignores_a_surface_biased_centre():
    estimate = estimate_target_yaw(cloud(1.0), (G1[0] - 0.05, G1[1] + 0.04))
    assert estimate is not None and error_deg(estimate, 1.0) < 8.0


def test_arms_down_is_unknown_not_a_guess():
    assert estimate_target_yaw(cloud(0.7, bars=()), G1) is None


def test_a_table_edge_at_forearm_height_is_not_a_forearm():
    xs = RNG.uniform(-0.6, 0.6, 500)
    edge = np.stack((G1[0] + xs, np.full(500, 0.38), RNG.uniform(0.84, 0.93, 500)), axis=1)
    assert estimate_target_yaw(np.vstack((cloud(0.0, bars=()), edge)), G1) is None


def test_no_points_or_no_g1_is_unknown():
    assert estimate_target_yaw(np.empty((0, 3)), G1) is None
    far = np.stack((RNG.uniform(5, 6, 200), RNG.uniform(5, 6, 200), RNG.uniform(0, 2, 200)), axis=1)
    assert estimate_target_yaw(far, G1) is None


def estimate(yaw):
    return OrientationEstimate(yaw, 100, 0.95)


def test_frames_must_agree_before_a_heading_is_used():
    assert combined_estimate([estimate(0.1), estimate(0.12), None, estimate(0.08)]).yaw_rad == (
        pytest.approx(0.1, abs=0.01))
    # Across the +-pi seam.
    assert abs(combined_estimate([estimate(3.1), estimate(-3.1), estimate(3.13)]).yaw_rad) > 3.0
    assert combined_estimate([estimate(0.1), None, estimate(0.1)]) is None      # too few
    assert combined_estimate([estimate(0.1), estimate(0.1), estimate(1.2)]) is None   # disagree
    assert combined_estimate([]) is None


@pytest.mark.parametrize('encoding,array,scale', [
    ('32FC1', np.array([[1.5, 2.0], [np.nan, 0.0]], dtype=np.float32), 1.0),
    ('16UC1', np.array([[1500, 2000], [0, 0]], dtype=np.uint16), 0.001),
])
def test_depth_is_read_in_metres(encoding, array, scale):
    msg = Image(height=2, width=2, encoding=encoding, data=array.tobytes())
    assert depth_metres(msg)[0, 1] == pytest.approx(array[0, 1] * scale)
    assert depth_metres(Image(height=2, width=2, encoding='rgb8')) is None


def test_depth_pixels_become_map_points_through_the_camera_pose():
    depth = np.full((3, 3), np.nan)
    depth[1, 1] = 2.0                      # the principal point, 2 m ahead
    depth[1, 2] = 2.0                      # one pixel right
    # A level camera at (1, 0, 1) looking along map +x: optical z -> +x, x -> -y, y -> -z.
    rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    points = depth_to_map_points(depth, (100.0, 100.0, 1.0, 1.0), rotation, [1.0, 0.0, 1.0])
    assert points == pytest.approx(np.array([[3.0, 0.0, 1.0], [3.0, -0.02, 1.0]]))


def test_the_body_axis_is_the_middle_of_the_legs_plus_the_surface_correction():
    points = cloud(0.0)
    x, y = estimate_body_axis(points, (2.9, 0.0), (0.3, 0.0))
    # The legs span the axis; the near-surface correction points away from the camera.
    assert x == pytest.approx(G1[0] + 0.01 + LEG_SURFACE_OFFSET_M, abs=0.02)
    assert y == pytest.approx(G1[1], abs=0.02)


def test_the_body_axis_needs_no_forearms_and_ignores_them():
    with_arms = estimate_body_axis(cloud(1.2), (3.0, 0.0), (0.3, 0.0))
    without = estimate_body_axis(cloud(1.2, bars=()), (3.0, 0.0), (0.3, 0.0))
    assert with_arms == pytest.approx(without, abs=0.02)


def test_the_body_axis_ignores_which_leg_shows_more_pixels():
    points = cloud(0.0)
    near_leg = (points[:, 2] < 0.6) & (points[:, 1] > G1[1])
    uneven = np.vstack((points, points[near_leg], points[near_leg], points[near_leg]))
    assert estimate_body_axis(uneven, G1, (0.3, 0.0)) == pytest.approx(
        estimate_body_axis(points, G1, (0.3, 0.0)), abs=0.015)


def test_a_body_axis_needs_legs_near_the_estimate():
    assert estimate_body_axis(np.empty((0, 3)), G1, (0.3, 0.0)) is None
    assert estimate_body_axis(cloud(0.0, legs=False), G1, (0.3, 0.0)) is None
    assert estimate_body_axis(cloud(0.0), (5.0, 0.0), (0.3, 0.0)) is None


def test_frames_must_agree_on_the_body_axis():
    assert combined_axis([(3.0, 0.0), (3.01, 0.0), None, (3.0, 0.01)]) == pytest.approx((3.0, 0.0))
    assert combined_axis([(3.0, 0.0), None, (3.0, 0.0)]) is None
    assert combined_axis([(3.0, 0.0), (3.0, 0.0), (3.2, 0.0)]) is None


# What the 2D LiDARs pass under: a tabletop over free floor, seen by the depth camera.
from ridgeback_autonomy.mission.parking_planner import GridSnapshot
from ridgeback_autonomy.mission.target_orientation import occupied_cells, overhead_obstacle_points


def tabletop(x0=2.4, x1=3.6, y0=0.7, y1=1.4, z=0.75, n=4000):
    return np.stack((RNG.uniform(x0, x1, n), RNG.uniform(y0, y1, n), np.full(n, z)), axis=1)


def test_a_tabletop_at_robot_height_is_an_obstacle_but_g1_floor_and_ceiling_are_not():
    floor = np.stack((RNG.uniform(1, 5, 2000), RNG.uniform(-2, 2, 2000), np.zeros(2000)), axis=1)
    ceiling = tabletop(z=2.5)
    points = np.vstack((cloud(math.pi), tabletop(), floor, ceiling))
    kept = overhead_obstacle_points(points, G1)
    assert len(kept) >= 3000                                       # the tabletop
    assert np.all(np.hypot(kept[:, 0] - G1[0], kept[:, 1] - G1[1]) > 0.45)   # never G1's own body
    assert np.all((kept[:, 1] >= 0.69) & (kept[:, 1] <= 1.41))


def test_only_cells_with_enough_points_are_marked():
    g = GridSnapshot('map', 0, 0.05, 120, 120, 0.0, -3.0, 0.0, np.zeros((120, 120), np.int16))
    rows, cols = occupied_cells(np.array([[3.01, 1.01]] * 2 + [[3.21, 1.21]] * 3), g)
    assert list(zip(rows.tolist(), cols.tolist())) == [(84, 64)]
    assert occupied_cells(np.empty((0, 2)), g)[0].size == 0
    assert occupied_cells(np.array([[50.0, 50.0]] * 5), g)[0].size == 0   # off the map


# The heading quality gate: a heading only from a view that shows G1's lower legs.
import os

from ridgeback_autonomy.mission.target_orientation import (
    HEADING_ESTIMATED, HEADING_INCONSISTENT, HEADING_INSUFFICIENT_EVIDENCE,
    HEADING_INSUFFICIENT_LOWER_BODY, HEADING_UNAVAILABLE, HEADING_VIEW_TOO_CLOSE, MIN_LEG_POINTS,
    HeadingAssessment, assess_heading, combined_heading, lowest_visible_height,
)

DATA = os.path.join(os.path.dirname(__file__), 'data')
# The robot camera: level, 1.0345 m up, 640x480 at a 384 px focal length.
LEVEL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])   # optical axis along +x
INTRINSICS = (384.0, 384.0, 320.0, 240.0)


def camera_at(distance, height=1.0345):
    return np.array([G1[0] - distance, G1[1], height])


def test_a_level_camera_loses_g1s_lower_legs_closer_than_about_1_3_m():
    low = lambda d: lowest_visible_height(LEVEL, camera_at(d), INTRINSICS, 480, G1)
    assert low(0.86) > 0.30          # the 2026-11-19 Isaac view: legs out of the image
    assert low(1.25) > 0.30 > low(1.40)
    assert low(1.62) < 0.12          # the closest ring of the recorded sweep


def test_the_limit_is_what_the_camera_sees_not_a_fixed_range():
    tilted = LEVEL @ np.array([[1.0, 0.0, 0.0], [0.0, math.cos(-0.35), -math.sin(-0.35)],
                               [0.0, math.sin(-0.35), math.cos(-0.35)]])   # pitched 20 deg down
    assert lowest_visible_height(LEVEL, camera_at(1.0), INTRINSICS, 480, G1) > 0.30
    assert lowest_visible_height(tilted, camera_at(1.0), INTRINSICS, 480, G1) < 0.30
    assert lowest_visible_height(LEVEL, camera_at(1.0, height=0.6), INTRINSICS, 480, G1) < 0.30


def test_a_heading_needs_as_many_leg_points_as_the_body_axis():
    """Until 2026-11-19 one leg point centred the forearm fit."""
    arms = cloud(0.4, legs=False)
    sliver = np.stack((np.full(MIN_LEG_POINTS - 1, G1[0] + 0.1), np.full(MIN_LEG_POINTS - 1, G1[1] + 0.1),
                       np.linspace(0.45, 0.55, MIN_LEG_POINTS - 1)), axis=1)
    assert estimate_target_yaw(np.vstack((arms, sliver)), G1) is None
    assert estimate_target_yaw(cloud(0.4), G1) is not None


def test_a_close_view_is_too_close_however_many_points_support_a_fit():
    """2026-11-18 gz_standard_run2: -155 deg for a G1 facing -90 deg from 0.68 m, on 14589
    points and 0.968 inliers. The view, not the support, decides."""
    a = assess_heading(cloud(0.4), G1, lowest_visible_height(LEVEL, camera_at(0.68), INTRINSICS, 480, G1))
    assert a.estimate is None and a.status == HEADING_VIEW_TOO_CLOSE


def test_hidden_lower_legs_are_insufficient_lower_body():
    upper = cloud(0.4)
    upper = upper[upper[:, 2] > 0.40]                      # a bed or a panel hides the shins
    low = lowest_visible_height(LEVEL, camera_at(2.5), INTRINSICS, 480, G1)
    a = assess_heading(upper, G1, low)
    assert a.estimate is None and a.status == HEADING_INSUFFICIENT_LOWER_BODY
    assert assess_heading(cloud(0.4), G1, low).status == HEADING_ESTIMATED


def test_legs_without_forearms_are_insufficient_evidence_and_no_geometry_is_unavailable():
    low = lowest_visible_height(LEVEL, camera_at(2.5), INTRINSICS, 480, G1)
    assert assess_heading(cloud(0.4, bars=()), G1, low).status == HEADING_INSUFFICIENT_EVIDENCE
    assert assess_heading(cloud(0.4), G1, None).status == HEADING_UNAVAILABLE


def test_frames_that_disagree_are_inconsistent_and_too_few_are_insufficient():
    ok = lambda yaw: HeadingAssessment(OrientationEstimate(yaw, 100, 0.95), HEADING_ESTIMATED)
    close = HeadingAssessment(None, HEADING_VIEW_TOO_CLOSE, 'x')
    assert combined_heading([ok(0.1), ok(0.12), ok(0.08)]).status == HEADING_ESTIMATED
    assert combined_heading([ok(0.1), ok(0.1), ok(1.2)]).status == HEADING_INCONSISTENT
    assert combined_heading([ok(0.1), ok(0.1)]).status == HEADING_INSUFFICIENT_EVIDENCE
    assert combined_heading([ok(0.1), close, close]).status == HEADING_VIEW_TOO_CLOSE
    assert combined_heading([]).status == HEADING_UNAVAILABLE


def sweep_view(sweep, name):
    v = {key[len(name) + 1:]: sweep[key] for key in sweep.files if key.startswith(name + '_')}
    depth = v['depth_mm'].astype(float) / 1000.0
    points = depth_to_map_points(depth, v['intrinsics'], v['camera_rotation'], v['camera_translation'])
    rows = depth.shape[0] + int(v['image_rows_below'])
    return v, points, rows


def test_the_recorded_72_view_sweep_keeps_every_heading():
    """Gazebo, 2026-11-01: G1 turned in 15 deg steps, seen from 1.6, 1.9 and 2.7 m (far, near,
    oblique). Before the gate every view gave a heading within 12.9 deg; it still must, also
    from a seed 0.1 m off G1's axis."""
    sweep = np.load(os.path.join(DATA, 'g1_heading_sweep_2026-11-01.npz'))
    names = sorted({key[:-len('_depth_mm')] for key in sweep.files if key.endswith('_depth_mm')})
    assert len(names) == 72
    errors = []
    for name in names:
        v, points, rows = sweep_view(sweep, name)
        for seed in (G1, (G1[0] + 0.1, G1[1]), (G1[0], G1[1] - 0.1)):
            low = lowest_visible_height(v['camera_rotation'], v['camera_translation'], v['intrinsics'], rows, seed)
            heading = combined_heading([assess_heading(points, seed, low)] * 3)   # a stationary view
            assert heading.status == HEADING_ESTIMATED, (name, heading)
            errors.append(error_deg(heading.estimate, math.radians(float(v['g1_yaw_deg']))))
    assert max(errors) < 13.0 and np.mean(errors) < 8.0


def test_the_recorded_close_ward_view_is_too_close():
    """2026-11-17, hospital standard: the camera 0.70 m from G1, behind its right shoulder, the
    view of the 2026-11-19 Isaac demonstration (0.76-0.86 m there). Depth shows the forearms
    but no leg below 0.56 m."""
    ward = np.load(os.path.join(DATA, 'g1_views_2026-11-17.npz'))
    v = {key[len('ward_close_'):]: ward[key] for key in ward.files if key.startswith('ward_close_')}
    depth = v['depth_mm'].astype(float) / 1000.0
    points = depth_to_map_points(depth, v['intrinsics'], v['camera_rotation'], v['camera_translation'])
    rows = 480 - int(240 - v['intrinsics'][3])     # the crop keeps the image's bottom rows
    floor = float(v['floor_z'])
    low = lowest_visible_height(v['camera_rotation'], v['camera_translation'], v['intrinsics'], rows,
                                tuple(v['g1_truth']), floor)
    heading = combined_heading([assess_heading(points, tuple(v['g1_truth']), low, floor)] * 5)
    assert heading.estimate is None and heading.status == HEADING_VIEW_TOO_CLOSE
    for name in ('ward_front', 'ward_behind'):     # the same pen from 2.1-2.5 m: headings
        v = {key[len(name) + 1:]: ward[key] for key in ward.files if key.startswith(name + '_')}
        depth = v['depth_mm'].astype(float) / 1000.0
        points = depth_to_map_points(depth, v['intrinsics'], v['camera_rotation'], v['camera_translation'])
        rows = 480 - int(240 - v['intrinsics'][3])
        low = lowest_visible_height(v['camera_rotation'], v['camera_translation'], v['intrinsics'], rows,
                                    tuple(v['g1_truth']), float(v['floor_z']))
        heading = combined_heading([assess_heading(points, tuple(v['g1_truth']), low, float(v['floor_z']))] * 3)
        assert heading.status == HEADING_ESTIMATED and error_deg(heading.estimate, -math.pi / 2) < 2.0
