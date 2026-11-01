"""G1 heading from depth points: a heading only when the forearms clearly show one."""

import math

import numpy as np
import pytest
from sensor_msgs.msg import Image

from ridgeback_autonomy.mission.target_orientation import (
    OrientationEstimate, combined_estimate, depth_metres, depth_to_map_points,
    estimate_target_yaw,
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
