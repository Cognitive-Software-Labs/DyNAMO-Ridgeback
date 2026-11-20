"""Live LiDAR clearance for the parking legs: a mission-level veto, never a velocity source.

The shared Nav2 collision monitor gates every velocity command on both 2D
LiDARs, but only at contact (``approach`` polygon) and without knowing the
maneuver. This check reads the same two scans and adds what parking needs:

* a physical clearance margin to every return over the robot's stopping path
  (current velocity, command latency and deceleration from ``ProtectionLimits``);
* the remaining validated route, so an obstacle that appears on it invalidates
  the maneuver before the robot gets there;
* G1's protected region from the localized target.

G1's own returns are not filtered out: they are held to the same margin and are
clear only because the planned poses keep G1 outside the keep-out.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from ridgeback_autonomy.mission.parking_geometry import normalize_angle, rotated_footprint_vertices
from ridgeback_autonomy.mission.parking_planner import points_to_polygon_distance
from ridgeback_autonomy.mission.parking_safety import ProtectionLimits


@dataclass(frozen=True)
class ClearanceLimits:
    margin_m: float = 0.05         # clearance kept to any return over the stopping path
    scan_max_age_s: float = 0.5    # by stamp (ROS clock) and by receipt (steady clock, x2)
    max_range_m: float = 4.0       # returns beyond this cannot reach the remaining legs
    route_step_m: float = 0.1
    route_step_rad: float = 0.1


def scan_points(scan, laser_in_base):
    """Valid returns of a LaserScan as (x, y) arrays in the base frame."""
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + scan.angle_increment * np.arange(ranges.size)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < scan.range_max)
    r, a = ranges[valid], angles[valid]
    lx, ly, lyaw = laser_in_base
    c, s = math.cos(lyaw), math.sin(lyaw)
    px, py = r * np.cos(a), r * np.sin(a)
    return lx + c * px - s * py, ly + s * px + c * py


def stopping_poses(velocity, age_s, protection: ProtectionLimits, step_m=0.05, step_rad=0.05):
    """Base-frame poses from here to where the robot stops, at its current velocity.

    Holonomic: the translation follows the velocity vector, not the heading.
    """
    vx, vy, wz = velocity
    v, w = math.hypot(vx, vy), abs(wz)
    latency = protection.latency_s + max(0.0, age_s)
    travel = v * latency + v * v / (2.0 * protection.linear_deceleration)
    turn = w * latency + w * w / (2.0 * protection.angular_deceleration)
    n = max(1, math.ceil(travel / step_m), math.ceil(turn / step_rad))
    ux, uy = (vx / v, vy / v) if v > 1e-6 else (0.0, 0.0)
    sign = math.copysign(1.0, wz)
    return [(ux * travel * k / n, uy * travel * k / n, sign * turn * k / n) for k in range(n + 1)]


def remaining_route(route, robot, limits=ClearanceLimits()):
    """Map-frame poses of ``route`` after the one nearest the robot, densified."""
    if not route:
        return []
    nearest = min(range(len(route)), key=lambda i: math.hypot(route[i][0] - robot[0],
                                                              route[i][1] - robot[1]))
    poses = [tuple(robot[:3])] + list(route[nearest + 1:])
    out = []
    for a, b in zip(poses, poses[1:]):
        dyaw = normalize_angle(b[2] - a[2])
        n = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / limits.route_step_m),
                math.ceil(abs(dyaw) / limits.route_step_rad))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, a[2] + dyaw * k / n)
                for k in range(1, n + 1)]
    return out


def route_deviation(route, robot, xy_tolerance_m, yaw_tolerance_rad):
    """Why ``robot`` has left the corridor around the checked ``route``, or ``None``: more than
    ``xy_tolerance_m`` from the polyline (before its start or past its end included), or turned
    more than ``yaw_tolerance_rad`` from the route's heading at the nearest point."""
    best = None
    for a, b in zip(route, route[1:]) if len(route) > 1 else [(route[0], route[0])]:
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, (
            (robot[0] - a[0]) * dx + (robot[1] - a[1]) * dy) / length2))
        distance = math.hypot(a[0] + t * dx - robot[0], a[1] + t * dy - robot[1])
        if best is None or distance < best[0]:
            best = (distance, a[2] + normalize_angle(b[2] - a[2]) * t)
    distance, yaw = best
    if distance > xy_tolerance_m:
        return f'route_deviation: {distance:.3f} m off the checked route'
    turned = abs(normalize_angle(robot[2] - yaw))
    if turned > yaw_tolerance_rad:
        return f'route_deviation: turned {turned:.3f} rad off the checked route'
    return None


def to_base(pose, robot):
    dx, dy = pose[0] - robot[0], pose[1] - robot[1]
    c, s = math.cos(robot[2]), math.sin(robot[2])
    return c * dx + s * dy, -s * dx + c * dy, normalize_angle(pose[2] - robot[2])


@dataclass
class ClearanceResult:
    reason: str | None
    nearest_m: float | None = None     # closest return to the current footprint
    g1_gap_m: float | None = None      # current footprint to the G1 reference
    points: int = 0
    vetoed_by: tuple | None = None     # base-frame return and map pose it fell inside (diagnostic)


def clearance_reason(points, footprint, velocity, age_s, robot, route, g1, g1_min_gap_m,
                     protection=ProtectionLimits(), limits=ClearanceLimits()) -> ClearanceResult:
    """Why the next movement is unsafe, or ``None``.

    ``points`` are base-frame returns; ``robot``, ``route`` and ``g1`` are in the
    map frame. Order: G1 region, stopping path, then the remaining route.
    """
    px, py = points
    near = np.hypot(px, py) <= limits.max_range_m
    px, py = px[near], py[near]
    here = rotated_footprint_vertices(footprint, 0.0, 0.0, 0.0)
    result = ClearanceResult(None, points=int(px.size))
    if px.size:
        result.nearest_m = float(points_to_polygon_distance(px, py, here).min())
    gx, gy, _ = to_base((g1[0], g1[1], 0.0), robot)
    result.g1_gap_m = float(points_to_polygon_distance([gx], [gy], here)[0])
    if result.g1_gap_m < g1_min_gap_m:
        result.reason = 'g1_protected_region'
        return result
    if not px.size:
        return result
    # Only returns within the footprint's circumscribed radius (plus the margin) of a
    # pose's centre can lie inside it or within the margin, so the exact distance is
    # computed for those alone: the same result, a fraction of the work. Checked on
    # every scan of both LiDARs, the full computation took 14 ms after the dev merge
    # and the mission fell behind its scans (lidar_stale, 2026-11-14).
    radius = max(math.hypot(x, y) for x, y in footprint)
    for pose in stopping_poses(velocity, age_s, protection):
        near = np.hypot(px - pose[0], py - pose[1]) <= radius + limits.margin_m
        if not near.any():
            continue
        polygon = rotated_footprint_vertices(footprint, *pose)
        distance = points_to_polygon_distance(px[near], py[near], polygon)
        if distance.min() < limits.margin_m:
            result.reason = 'lidar_obstacle_in_stopping_path'
            result.vetoed_by = (float(px[near][distance.argmin()]), float(py[near][distance.argmin()]), None)
            return result
    for pose in remaining_route(route, robot, limits):
        base = to_base(pose, robot)
        near = np.hypot(px - base[0], py - base[1]) <= radius
        if not near.any():
            continue
        polygon = rotated_footprint_vertices(footprint, *base)
        distance = points_to_polygon_distance(px[near], py[near], polygon)
        if distance.min() <= 0.0:
            result.reason = 'lidar_obstacle_on_route'
            result.vetoed_by = (float(px[near][distance.argmin()]), float(py[near][distance.argmin()]), pose)
            return result
    return result
