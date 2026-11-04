"""G1 heading and body axis in the map frame from one depth image, with no simulator input.

The G1 holds both forearms forward: from every side, the only part of it more
than 0.18 m from its vertical axis is two parallel bars at chest height, on
either side of the facing direction (measured from Gazebo depth, 2026-11-01).
So the depth points around the localized G1 are matched against those two bars
at every heading, and the best heading is reported only when enough points
support it and few contradict it. A heading is never inferred from their absence:
without visible forearms (hidden behind G1, or a posture with the arms down) the
estimate is None and parking keeps its undirected ranking.

The body axis is a separate estimate from the same points: the middle of the legs,
which the detector's surface centroid is not (the forearms pull it toward a camera
G1 faces). It needs no forearms and does not feed the heading.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


# Depth points kept around G1: above the floor, within reach of its forearm tips.
SEARCH_RADIUS_M = 0.60
BODY_HEIGHT_M = (0.05, 1.50)
LEG_HEIGHT_M = (0.10, 0.55)
LEG_SEARCH_RADIUS_M = 0.30   # G1's own legs: a nearby table's legs must not move the axis
MIN_LEG_POINTS = 50
LEG_SURFACE_OFFSET_M = 0.03  # only the near surfaces show: measured short by this along the view
MAX_AXIS_SPREAD_M = 0.03
# The forearm bars, in G1's frame (x forward, y left), with a margin for depth
# noise and for the centre error of a surface-based G1 estimate.
FOREARM_HEIGHT_M = (0.80, 0.97)
FOREARM_MIN_RADIUS_M = 0.20
FOREARM_MAX_RADIUS_M = 0.50
FOREARM_FORWARD_M = (0.14, 0.44)
FOREARM_LATERAL_M = (0.07, 0.22)
HEADING_STEP_RAD = math.radians(2.0)
# A forearm seen along enough of its length gives the heading directly as its own
# direction; one bar alone otherwise fits as the left or the right arm.
MIN_BAR_LENGTH_M = 0.10
# Reliability: points on the bars, and how many of the protruding points the two
# bars explain (a table edge at this height is not bar-shaped).
MIN_FOREARM_POINTS = 12
MIN_INLIER_FRACTION = 0.80
MIN_FOREARM_REACH_M = 0.28   # the fingertips, seen from any side; a stray patch stops short
# Several frames of one stationary view must agree before a heading is used.
MIN_AGREEING_FRAMES = 3
MAX_FRAME_SPREAD_RAD = math.radians(20.0)


@dataclass(frozen=True)
class OrientationEstimate:
    yaw_rad: float            # the direction G1 faces, map frame
    support_points: int       # depth points on the matched forearm bars
    inlier_fraction: float    # share of the protruding points the bars explain


def depth_metres(msg):
    """A sensor_msgs/Image depth as metres, or None for an encoding depth is not sent in."""
    if msg.encoding == '32FC1':
        data = np.frombuffer(msg.data, dtype=np.float32)
    elif msg.encoding == '16UC1':
        data = np.frombuffer(msg.data, dtype=np.uint16) * 0.001
    else:
        return None
    return data.reshape(msg.height, msg.width)


def depth_to_map_points(depth_m, intrinsics, rotation, translation, *, stride=1):
    """(N, 3) map points of a metric depth image; ``rotation`` (3x3) and ``translation``
    (3) take the optical frame to the map, ``intrinsics`` is (fx, fy, cx, cy)."""
    fx, fy, cx, cy = intrinsics
    z = np.asarray(depth_m, dtype=float)[::stride, ::stride]
    v, u = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    valid = np.isfinite(z) & (z > 0.0)
    optical = np.stack(((u[valid] - cx) * z[valid] / fx, (v[valid] - cy) * z[valid] / fy,
                        z[valid]), axis=1)
    return optical @ np.asarray(rotation).T + np.asarray(translation)


def estimate_target_yaw(points_map, center_xy, floor_z=0.0):
    """The heading G1 faces from its depth points, or None when it is not reliably visible."""
    points = np.asarray(points_map, dtype=float)
    if points.size == 0:
        return None
    dx, dy, height = points[:, 0] - center_xy[0], points[:, 1] - center_xy[1], points[:, 2] - floor_z
    around = ((np.hypot(dx, dy) <= SEARCH_RADIUS_M)
              & (height >= BODY_HEIGHT_M[0]) & (height <= BODY_HEIGHT_M[1]))
    legs = _leg_points(dx, dy, height)
    if not legs.any():
        return None
    dx, dy = dx - _middle(dx[legs]), dy - _middle(dy[legs])
    radius = np.hypot(dx, dy)
    band = around & (height >= FOREARM_HEIGHT_M[0]) & (height <= FOREARM_HEIGHT_M[1])
    bars = band & (radius >= FOREARM_MIN_RADIUS_M) & (radius <= FOREARM_MAX_RADIUS_M)
    if np.count_nonzero(bars) < MIN_FOREARM_POINTS:
        return None
    px, py = dx[bars], dy[bars]
    headings = np.arange(-math.pi, math.pi, HEADING_STEP_RAD)
    cos_h, sin_h = np.cos(headings)[:, None], np.sin(headings)[:, None]
    forward = px * cos_h + py * sin_h
    lateral = np.abs(py * cos_h - px * sin_h)
    on_bars = ((forward >= FOREARM_FORWARD_M[0]) & (forward <= FOREARM_FORWARD_M[1])
               & (lateral >= FOREARM_LATERAL_M[0]) & (lateral <= FOREARM_LATERAL_M[1]))
    counts = on_bars.sum(axis=1)
    best = counts.max()
    # The centre of the plateau of equally good headings, on the circle.
    ties = np.flatnonzero(counts == best)
    yaw = math.atan2(float(np.sin(headings[ties]).mean()), float(np.cos(headings[ties]).mean()))
    fraction = float(best) / px.size
    if best < MIN_FOREARM_POINTS or fraction < MIN_INLIER_FRACTION:
        return None
    inliers = on_bars[ties[len(ties) // 2]]
    if forward[ties[len(ties) // 2]][inliers].max() < MIN_FOREARM_REACH_M:
        return None
    return OrientationEstimate(_along_forearms(px[inliers], py[inliers], yaw), int(best),
                               round(fraction, 3))


def _leg_points(dx, dy, height):
    return ((np.hypot(dx, dy) <= LEG_SEARCH_RADIUS_M)
            & (height >= LEG_HEIGHT_M[0]) & (height <= LEG_HEIGHT_M[1]))


def _middle(values):
    """The middle of an extent: unlike a mean or median, it ignores which leg shows more pixels."""
    return float(np.mean(np.percentile(values, (2, 98))))


def estimate_body_axis(points_map, seed_xy, camera_xy, floor_z=0.0):
    """G1's vertical axis (x, y) from its legs near ``seed_xy``, or None without enough of them."""
    points = np.asarray(points_map, dtype=float)
    if points.size == 0:
        return None
    dx, dy = points[:, 0] - seed_xy[0], points[:, 1] - seed_xy[1]
    legs = _leg_points(dx, dy, points[:, 2] - floor_z)
    if np.count_nonzero(legs) < MIN_LEG_POINTS:
        return None
    x, y = seed_xy[0] + _middle(dx[legs]), seed_xy[1] + _middle(dy[legs])
    view = math.atan2(y - camera_xy[1], x - camera_xy[0])
    return (x + LEG_SURFACE_OFFSET_M * math.cos(view), y + LEG_SURFACE_OFFSET_M * math.sin(view))


def combined_axis(axes):
    """One axis from a stationary view's per-frame axes, or None unless enough frames agree."""
    seen = [a for a in axes if a is not None]
    if len(seen) < MIN_AGREEING_FRAMES:
        return None
    x, y = float(np.median([a[0] for a in seen])), float(np.median([a[1] for a in seen]))
    if max(math.hypot(a[0] - x, a[1] - y) for a in seen) > MAX_AXIS_SPREAD_M:
        return None
    return x, y


def _along_forearms(px, py, yaw):
    """Refine ``yaw`` to the mean direction of the forearm bars long enough to show one."""
    side = py * math.cos(yaw) - px * math.sin(yaw)
    sum_x = sum_y = 0.0
    for bar in (side > 0.0, side < 0.0):
        if np.count_nonzero(bar) < MIN_FOREARM_POINTS:
            continue
        points = np.stack((px[bar], py[bar]), axis=1)
        offsets = points - points.mean(axis=0)
        direction = np.linalg.svd(offsets, full_matrices=False)[2][0]
        along = offsets @ direction
        if along.max() - along.min() < MIN_BAR_LENGTH_M:
            continue
        if direction[0] * math.cos(yaw) + direction[1] * math.sin(yaw) < 0.0:
            direction = -direction
        sum_x += direction[0] * px[bar].size
        sum_y += direction[1] * px[bar].size
    return math.atan2(sum_y, sum_x) if sum_x or sum_y else yaw


def combined_estimate(estimates):
    """One heading from a stationary view's per-frame estimates (None for a frame without one),
    or None unless enough frames agree."""
    seen = [e for e in estimates if e is not None]
    if len(seen) < MIN_AGREEING_FRAMES:
        return None
    yaw = math.atan2(sum(math.sin(e.yaw_rad) for e in seen), sum(math.cos(e.yaw_rad) for e in seen))
    if max(abs(math.remainder(e.yaw_rad - yaw, 2.0 * math.pi)) for e in seen) > MAX_FRAME_SPREAD_RAD:
        return None
    return OrientationEstimate(yaw, min(e.support_points for e in seen),
                               min(e.inlier_fraction for e in seen))


# Obstacles the 2D LiDARs pass under (a tabletop): depth points between just above
# the scan plane and the robot's top, near G1 but outside its own body. The robot is
# 1.03 m tall at its camera; G1's forearms reach 0.40 m from its axis, over the bin.
OVERHEAD_HEIGHT_M = (0.25, 1.15)
OVERHEAD_RADIUS_M = 1.6
G1_BODY_RADIUS_M = 0.45
MIN_OVERHEAD_POINTS_PER_CELL = 3


def overhead_obstacle_points(points_map, center_xy, floor_z=0.0):
    """(N, 2) map points of obstacles at robot height near G1, outside its own body."""
    points = np.asarray(points_map, dtype=float)
    if points.size == 0:
        return np.empty((0, 2))
    radius = np.hypot(points[:, 0] - center_xy[0], points[:, 1] - center_xy[1])
    height = points[:, 2] - floor_z
    keep = ((height >= OVERHEAD_HEIGHT_M[0]) & (height <= OVERHEAD_HEIGHT_M[1])
            & (radius > G1_BODY_RADIUS_M) & (radius <= OVERHEAD_RADIUS_M))
    return points[keep, :2]


def occupied_cells(points_xy, grid):
    """(rows, cols) of grid cells holding at least MIN_OVERHEAD_POINTS_PER_CELL of the points."""
    points = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    lx, ly = grid.map_to_local(points[:, 0], points[:, 1])
    cols = np.floor(np.asarray(lx) / grid.resolution).astype(int)
    rows = np.floor(np.asarray(ly) / grid.resolution).astype(int)
    inside = (cols >= 0) & (cols < grid.width) & (rows >= 0) & (rows < grid.height)
    if not inside.any():
        return np.empty(0, int), np.empty(0, int)
    cells, counts = np.unique(np.stack((rows[inside], cols[inside])), axis=1, return_counts=True)
    cells = cells[:, counts >= MIN_OVERHEAD_POINTS_PER_CELL]
    return cells[0], cells[1]
