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

The position is checked against G1's head before either is used: the detector's
surface range comes from the nearest surface in the whole detection box, which is a
worktable's edge or a panel's end when one stands in front of G1's legs or beside it.
The head region of the box is G1 whatever stands there, so a surface well in front of
or behind it re-seeds the position on the head, and a position is used only when depth
shows G1's torso above it.
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


# The top quarter of G1's detection box, across its middle 40 %, is its head and upper
# torso: above a worktable (0.76 m) or bed in front of its legs, and away from the box's
# sides, where a panel beside G1 shows. The nearest fifth of those depths is G1's own
# range. On 2026-11-09 Gazebo views from 2.3-6.9 m it was 0.01-0.12 m short of G1's
# axis, where the detector's surface range was 1.25 m short behind a worktable and
# 1.1 m short beside a panel.
HEAD_ROWS_FRACTION = 0.25
HEAD_COLUMNS_FRACTION = 0.40
HEAD_RANGE_PERCENTILE = 20.0
HEAD_SURFACE_BAND_M = (0.05, 0.15)   # around that range: the head and torso surface
MIN_HEAD_POINTS = 12
HEAD_DEPTH_M = 0.07                  # head surface to G1's vertical axis
MAX_HEAD_SPREAD_M = 0.10             # between the frames of one stationary view
# G1's nearest surface can be its forearm tips, 0.44 m from its axis toward the camera,
# while its head is 0.07 m from it; a surface farther ahead or behind is another object.
MAX_SURFACE_AHEAD_OF_HEAD_M = 0.50
MAX_SURFACE_BEHIND_HEAD_M = 0.30
# G1 itself above its axis: torso and head, higher than a worktable, within its shoulders.
TORSO_HEIGHT_M = (0.85, 1.40)
TORSO_RADIUS_M = 0.25
MIN_TORSO_POINTS = 20


@dataclass(frozen=True)
class HeadObservation:
    x: float
    y: float
    range_m: float           # horizontal, from the camera
    points: int


def box_pixels(box_xyxy, image_size, depth_shape):
    """The detection box in depth pixels (x0, y0, x1, y1), scaled when the sizes differ."""
    sx = depth_shape[1] / float(image_size[0]) if image_size[0] else 1.0
    sy = depth_shape[0] / float(image_size[1]) if image_size[1] else 1.0
    x0, y0, x1, y1 = (float(v) for v in box_xyxy[:4])
    return (max(0, int(math.floor(x0 * sx))), max(0, int(math.floor(y0 * sy))),
            min(depth_shape[1], int(math.ceil(x1 * sx)) + 1),
            min(depth_shape[0], int(math.ceil(y1 * sy)) + 1))


def head_observation(depth_m, box, intrinsics, rotation, translation):
    """G1's head-region surface in the map from one depth frame, or None without enough of it.

    ``box`` is (x0, y0, x1, y1) in depth pixels."""
    x0, y0, x1, y1 = box
    width, height = x1 - x0, y1 - y0
    if width < 3 or height < 3:
        return None
    rows = (y0, y0 + max(2, int(round(HEAD_ROWS_FRACTION * height))))
    margin = (1.0 - HEAD_COLUMNS_FRACTION) / 2.0
    cols = (x0 + int(round(margin * width)), x1 - int(round(margin * width)))
    if cols[1] - cols[0] < 1:
        return None
    patch = np.asarray(depth_m, dtype=float)[rows[0]:rows[1], cols[0]:cols[1]]
    fx, fy, cx, cy = intrinsics
    v, u = np.mgrid[rows[0]:rows[0] + patch.shape[0], cols[0]:cols[0] + patch.shape[1]]
    valid = np.isfinite(patch) & (patch > 0.0)
    if np.count_nonzero(valid) < MIN_HEAD_POINTS:
        return None
    z = patch[valid]
    optical = np.stack(((u[valid] - cx) * z / fx, (v[valid] - cy) * z / fy, z), axis=1)
    points = optical @ np.asarray(rotation).T + np.asarray(translation)
    ranges = np.hypot(points[:, 0] - translation[0], points[:, 1] - translation[1])
    near = float(np.percentile(ranges, HEAD_RANGE_PERCENTILE))
    surface = (ranges >= near - HEAD_SURFACE_BAND_M[0]) & (ranges <= near + HEAD_SURFACE_BAND_M[1])
    if np.count_nonzero(surface) < MIN_HEAD_POINTS:
        return None
    return HeadObservation(float(points[surface, 0].mean()), float(points[surface, 1].mean()),
                           near, int(np.count_nonzero(surface)))


def combined_head(heads):
    """One head observation from a stationary view's frames, or None unless enough agree."""
    seen = [h for h in heads if h is not None]
    if len(seen) < MIN_AGREEING_FRAMES:
        return None
    x, y = float(np.median([h.x for h in seen])), float(np.median([h.y for h in seen]))
    if max(math.hypot(h.x - x, h.y - y) for h in seen) > MAX_HEAD_SPREAD_M:
        return None
    return HeadObservation(x, y, float(np.median([h.range_m for h in seen])),
                           min(h.points for h in seen))


def head_axis(head, camera_xy):
    """G1's vertical axis behind its head-region surface, seen from ``camera_xy``."""
    view = math.atan2(head.y - camera_xy[1], head.x - camera_xy[0])
    return head.x + HEAD_DEPTH_M * math.cos(view), head.y + HEAD_DEPTH_M * math.sin(view)


def surface_offset_from_head(surface_xy, head, camera_xy):
    """How far the measured surface lies in front of (+) or behind (-) G1's head, along the view."""
    return head.range_m - math.hypot(surface_xy[0] - camera_xy[0], surface_xy[1] - camera_xy[1])


def surface_is_g1(offset_m):
    return -MAX_SURFACE_BEHIND_HEAD_M <= offset_m <= MAX_SURFACE_AHEAD_OF_HEAD_M


def torso_points(points_map, axis_xy, floor_z=0.0):
    """How many depth points stand above ``axis_xy`` at G1's torso and head height."""
    points = np.asarray(points_map, dtype=float)
    if points.size == 0:
        return 0
    radius = np.hypot(points[:, 0] - axis_xy[0], points[:, 1] - axis_xy[1])
    height = points[:, 2] - floor_z
    return int(np.count_nonzero((radius <= TORSO_RADIUS_M) & (height >= TORSO_HEIGHT_M[0])
                                & (height <= TORSO_HEIGHT_M[1])))


def torso_confirmed(counts):
    """Whether enough frames of a stationary view show G1's torso above the axis."""
    return sum(c >= MIN_TORSO_POINTS for c in counts) >= MIN_AGREEING_FRAMES


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
