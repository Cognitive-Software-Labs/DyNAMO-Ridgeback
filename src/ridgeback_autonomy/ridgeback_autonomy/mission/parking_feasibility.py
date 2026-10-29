"""Backend-neutral path sampling, bounded search and snapshot drift guards.

These checks veto a global path; they never execute it or relax geometric
parking checks. NavFn's exploration tolerance/unknown policy is not trusted.
"""

from dataclasses import dataclass, replace
import math

import numpy as np

from ridgeback_autonomy.mission.parking_geometry import (
    normalize_angle, planar_path_length, rotated_footprint_vertices,
)
from ridgeback_autonomy.mission.parking_planner import (
    footprint_rejection_reason, points_to_polygon_distance, preference,
)


def pose_xyyaw(pose):
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or norm < 1e-9:
        raise ValueError('invalid_pose')
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    return p.x, p.y, math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def candidate_groups(plan, approach, per_class=6):
    """At most six spatially diverse geometrically valid goals per priority.

    Use the existing reach/clearance tiers to order each class. Cover angular
    sectors first with each sector's best-ranked candidate (the route is checked
    via the pre-parking pose and a straight approach, so outer-reach poses gain
    no turning room). Then fill spare slots in geometric order.
    Exhaustion of this subset is not proof that the entire annulus is blocked.
    """
    groups = []
    for rank in range(3):
        pool = [c for c in plan.candidates if c.status in ('valid', 'selected')
                and preference(c.task_x, c.task_y, plan.target, approach)[0] == rank]
        ordered = []
        while pool:
            best = min(c.reach_error_m for c in pool)
            tier = [c for c in pool if c.reach_error_m <= best + plan.resolution_m]
            clearance = max(c.clearance_m for c in tier)
            tier = [c for c in tier if c.clearance_m >= clearance - plan.resolution_m]
            chosen = tier[0]
            ordered.append(chosen)
            pool = [c for c in pool if c is not chosen]
        sectors = {}
        for c in ordered:
            angle = math.atan2(c.task_y - plan.target.y, c.task_x - plan.target.x)
            sectors.setdefault(math.floor(angle / (math.pi / 4)), []).append(c)
        chosen = [bucket[0] for bucket in sectors.values()][:per_class]
        chosen += [c for c in ordered if all(c is not a for a in chosen)][:per_class - len(chosen)]
        if chosen:
            groups.append(chosen)
    return groups


def select_path_candidate(candidates, resolution):
    best_error = min(c.reach_error_m for c in candidates)
    tier = [c for c in candidates if c.reach_error_m <= best_error + resolution]
    best_clearance = max(c.clearance_m for c in tier)
    tier = [c for c in tier if c.clearance_m >= best_clearance - resolution]
    best_length = min(c.path_length_m for c in tier)
    return next(c for c in tier if c.path_length_m <= best_length + resolution)


def path_rejection(path, candidate, start, grid, target, config, keepout, *, occupied=()):
    """Check endpoints and swept footprints at <=half-cell / 0.05 rad steps.

    The swept route starts at the robot's actual pose and yaw in place of the
    planner's start cell (which must lie within 0.10 m). Identity orientations on
    intermediate poses are planner placeholders and inherit the previous heading;
    every other yaw change is swept with the rotated footprint. ``occupied`` are
    further poses the robot physically stood at (the mission start).
    """
    if path is None or not path.poses:
        return 'empty_path'
    if len(path.poses) > 2000:
        return 'path_sampling_budget_exceeded'
    if path.header.frame_id != grid.frame_id:
        return 'path_frame'
    try:
        if any(p.header.frame_id not in ('', grid.frame_id) for p in path.poses):
            return 'path_pose_frame'
        poses = [pose_xyyaw(p.pose) for p in path.poses]
    except ValueError:
        return 'path_nonfinite_or_invalid_rotation'
    if math.hypot(poses[0][0] - start[0], poses[0][1] - start[1]) > 0.10:
        return 'path_start_mismatch'
    end = poses[-1]
    if math.hypot(end[0] - candidate.base_x, end[1] - candidate.base_y) > 0.10:
        return 'path_goal_mismatch'
    if abs(normalize_angle(end[2] - candidate.yaw)) > 0.12:
        return 'path_goal_yaw_mismatch'
    for stood in (start,) + tuple(occupied):
        grid = _start_footprint_known(grid, stood, config)
    # The planner's first pose is the robot's start cell (NavFn: the grid-snapped
    # cell point, a few cm off), not a place the robot drives to, so the swept route
    # starts at the robot's actual pose. Keep the goal connector rather than
    # trusting the endpoint tolerance.
    poses = ([tuple(start[:3])] + poses[1 if len(poses) > 1 else 0:]
             + [(candidate.base_x, candidate.base_y, candidate.yaw)])
    # NavFn leaves intermediate orientations as the identity quaternion and the
    # Omni MPPI controller does not follow path orientations, so those are not
    # headings: the robot translates holonomically keeping its current yaw. Any
    # real yaw (the goal pose, the final approach) is still swept below.
    for i in range(1, len(path.poses) - 1):
        q = path.poses[i].pose.orientation
        if (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0):
            poses[i] = poses[i][:2] + (poses[i - 1][2],)
    budget = 2000
    for a, b in zip(poses, poses[1:]):
        dyaw = normalize_angle(b[2] - a[2])
        n = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / (grid.resolution * 0.5)),
                math.ceil(abs(dyaw) / 0.05))
        budget -= n + 1
        if budget < 0:
            return 'path_sampling_budget_exceeded'
        for k in range(n + 1):
            f = k / n
            polygon = rotated_footprint_vertices(config.footprint, a[0] + f * (b[0] - a[0]),
                                                 a[1] + f * (b[1] - a[1]), a[2] + f * dyaw)
            reason = footprint_rejection_reason(polygon, grid, config.occupied_threshold)
            if reason:
                return f'path_footprint_{reason}'
            if points_to_polygon_distance([target.x], [target.y], polygon)[0] < keepout:
                return 'path_g1_keepout'
    return None


def _start_footprint_known(grid, start, config):
    """The grid with unknown cells under the robot's own start footprint treated as free.

    The LiDARs cannot see beneath the chassis, so a pose the robot never left
    (e.g. its spawn pose) stays unknown although the robot stands on it.
    Occupied cells and unknown cells elsewhere on the route still reject.
    """
    polygon = rotated_footprint_vertices(config.footprint, *start[:3])
    lx, ly = grid.map_to_local([p[0] for p in polygon], [p[1] for p in polygon])
    res = grid.resolution
    c0, c1 = max(0, int(lx.min() // res) - 1), min(grid.width - 1, int(lx.max() // res) + 1)
    r0, r1 = max(0, int(ly.min() // res) - 1), min(grid.height - 1, int(ly.max() // res) + 1)
    if c0 > c1 or r0 > r1:
        return grid
    cols, rows = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
    cx, cy = grid.cell_centers(cols, rows)
    under = points_to_polygon_distance(cx, cy, polygon) <= math.sqrt(2.0) * res * 0.5
    window = grid.data[r0:r1 + 1, c0:c1 + 1]
    unknown = under & (window == -1)
    if not unknown.any():
        return grid
    data = grid.data.copy()
    data[r0:r1 + 1, c0:c1 + 1][unknown] = 0
    return replace(grid, data=data)


def path_length(path):
    return planar_path_length((p.pose.position.x, p.pose.position.y) for p in path.poses)


@dataclass(frozen=True)
class PlanningSnapshot:
    robot: tuple
    map_odom: tuple

    def changed(self, robot, map_odom, translation_limit=0.05, yaw_limit=0.03):
        if robot is None or map_odom is None:
            return 'planning_tf_unavailable'
        for name, old, new in (('robot', self.robot, robot), ('map_odom', self.map_odom, map_odom)):
            if (math.hypot(new[0] - old[0], new[1] - old[1]) > translation_limit
                    or abs(normalize_angle(new[2] - old[2])) > yaw_limit):
                return f'{name}_changed'
        return None
