"""Parking candidate generation around a localized G1, without navigation.

Ported from the original mission manager's parking pipeline (donut occupancy
classification, base pose per free task cell, rotated-footprint rejection,
footprint clearance, approach-relative preference and the tiered ranking).
The geometry itself stays in ``parking_geometry``; this module only adds what
the integration needs around it:

* the target reference is explicit. Only a validated foot midpoint earns the
  small margin; the current generic estimator gets a larger one;
* a conservative G1 keep-out disk, because the occupancy grid alone cannot be
  trusted to contain G1 (toes lie below the LiDAR plane and stationary SLAM
  drift smears the leg imprint);
* freshness checks on target, map and robot pose, since the map frame drifts;
* a 1-degree radial profile of the donut, split where its class changes, for
  visualization only.

The original ranked by Nav2 path length as its fourth tier. Path planning is
deliberately absent here, so that tier is skipped and the deterministic
input-order tie-break follows the clearance tier directly.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Sequence

import numpy as np

from ridgeback_autonomy.mission.parking_geometry import (
    base_pose_from_task_point,
    normalize_angle,
    rotated_footprint_vertices,
    task_point_from_base_pose,
)


ORIGINAL_REACH_MIN_M = 0.30
ORIGINAL_REACH_MAX_M = 0.60
ORIGINAL_TASK_POINT_X_BASE_M = -0.280
ORIGINAL_TASK_POINT_Y_BASE_M = 0.000
ORIGINAL_OCCUPIED_THRESHOLD = 50

FOOT_MIDPOINT_REFERENCE = 'foot_midpoint'
GENERIC_TARGET_REFERENCE = 'generic_target_position'
# Planar envelope of G1's feet and lower legs around the foot midpoint, from the
# static SDF meshes: feet span +-0.104 m along G1's x and reach +-0.156 m
# laterally, so hypot(0.104, 0.156) = 0.19 m covers any G1 heading.
G1_LOWER_BODY_RADIUS_M = 0.19
# Reference uncertainty added on top of the body envelope. A validated foot
# midpoint keeps a small allowance; the generic estimate was measured 2 cm from
# the foot midpoint head-on only, so other viewpoints get a body-depth margin.
REFERENCE_MARGIN_M = {
    FOOT_MIDPOINT_REFERENCE: 0.05,
    GENERIC_TARGET_REFERENCE: 0.10,
}
# Distant unsafe cells cannot change the ranking among candidates near G1.
CLEARANCE_CAP_M = 1.0


@dataclass(frozen=True)
class GridSnapshot:
    """An occupancy grid in plain values: row-major ``data`` of shape (height, width)."""

    frame_id: str
    stamp_ns: int
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: np.ndarray

    def __post_init__(self) -> None:
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError('map resolution must be finite and positive')
        if self.width <= 0 or self.height <= 0:
            raise ValueError('map dimensions must be positive')
        if not all(math.isfinite(v) for v in (self.origin_x, self.origin_y, self.origin_yaw)):
            raise ValueError('map origin must be finite')
        if self.data.shape != (self.height, self.width):
            raise ValueError('map data shape does not match dimensions')
        if not np.all(np.isfinite(self.data)) or np.any((self.data < -1) | (self.data > 100)):
            raise ValueError('map occupancy must be -1 or within [0, 100]')

    @classmethod
    def from_msg(cls, msg) -> 'GridSnapshot':
        q = msg.info.origin.orientation
        rotation = (q.x, q.y, q.z, q.w)
        norm = math.hypot(*rotation)
        if not all(math.isfinite(v) for v in rotation) or norm < 1e-9:
            raise ValueError('map origin quaternion must be finite and nonzero')
        x, y, z, w = (v / norm for v in rotation)
        # A 2D grid may be yaw-rotated, but projecting a tilted grid is unsafe.
        if abs(x) > 1e-6 or abs(y) > 1e-6:
            raise ValueError('map origin must describe a planar rotation')
        yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
        data = np.asarray(msg.data, dtype=np.int16).reshape(int(msg.info.height), int(msg.info.width))
        return cls(
            frame_id=msg.header.frame_id,
            stamp_ns=int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
            resolution=float(msg.info.resolution),
            width=int(msg.info.width),
            height=int(msg.info.height),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            origin_yaw=yaw,
            data=data,
        )

    def map_to_local(self, x_m, y_m):
        """Map coordinates -> grid-local metres (original ``_map_to_grid_local``)."""

        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        dx = np.asarray(x_m, dtype=float) - self.origin_x
        dy = np.asarray(y_m, dtype=float) - self.origin_y
        return c * dx + s * dy, -s * dx + c * dy

    def cell_centers(self, cols, rows):
        """Cell centres in map coordinates (original ``_grid_cell_center_to_map``)."""

        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        local_x = (np.asarray(cols, dtype=float) + 0.5) * self.resolution
        local_y = (np.asarray(rows, dtype=float) + 0.5) * self.resolution
        return self.origin_x + c * local_x - s * local_y, self.origin_y + s * local_x + c * local_y


@dataclass(frozen=True)
class TargetReference:
    """A localized target and what kind of point it is."""

    x: float
    y: float
    frame_id: str
    stamp_ns: int
    reference: str
    is_foot_midpoint: bool
    source: str


@dataclass(frozen=True)
class PlannerConfig:
    footprint: tuple[tuple[float, float], ...]
    reach_min_m: float = ORIGINAL_REACH_MIN_M
    reach_max_m: float = ORIGINAL_REACH_MAX_M
    task_point_x_base_m: float = ORIGINAL_TASK_POINT_X_BASE_M
    task_point_y_base_m: float = ORIGINAL_TASK_POINT_Y_BASE_M
    occupied_threshold: int = ORIGINAL_OCCUPIED_THRESHOLD
    g1_lower_body_radius_m: float = G1_LOWER_BODY_RADIUS_M
    clearance_cap_m: float = CLEARANCE_CAP_M
    # Planned reach is capped at reach_max_m - reach_allowance_m so the actual
    # reach stays inside the band after final-pose and target-reference error.
    reach_allowance_m: float = 0.0


@dataclass
class CandidateDecision:
    task_x: float
    task_y: float
    base_x: float
    base_y: float
    yaw: float
    status: str           # valid | collision | rejected | selected (original semantics)
    reason: str
    reach_m: float
    reach_error_m: float | None = None
    clearance_m: float | None = None
    preference_label: str | None = None
    path_status: str = 'unchecked'
    path_reason: str | None = None
    path_length_m: float | None = None


@dataclass
class ParkingPlan:
    ok: bool
    failure_reason: str | None
    target: TargetReference
    keepout_radius_m: float
    free_cells: list[tuple[float, float]] = field(default_factory=list)
    occupied_cells: list[tuple[float, float]] = field(default_factory=list)
    unknown_cells: list[tuple[float, float]] = field(default_factory=list)
    candidates: list[CandidateDecision] = field(default_factory=list)
    selected: CandidateDecision | None = None
    radial_rays: list[tuple[float, str]] = field(default_factory=list)
    resolution_m: float = 0.0
    selected_path: object | None = None
    paths_checked: int = 0

    def counts(self) -> dict:
        by_reason: dict[str, int] = {}
        for c in self.candidates:
            by_reason[c.reason] = by_reason.get(c.reason, 0) + 1
        return {
            'donut_free': len(self.free_cells), 'donut_occupied': len(self.occupied_cells),
            'donut_unknown': len(self.unknown_cells), 'candidates': len(self.candidates),
            'valid': sum(c.status in ('valid', 'selected') for c in self.candidates),
            'by_reason': by_reason,
            'paths_checked': self.paths_checked,
            'path_valid': sum(c.path_status == 'valid' for c in self.candidates),
            'path_rejected': sum(c.path_status == 'rejected' for c in self.candidates),
        }


def keepout_radius_m(target: TargetReference, config: PlannerConfig) -> float:
    """Body envelope plus a reference-dependent margin; unknown references get the largest."""

    reference = (FOOT_MIDPOINT_REFERENCE
                 if target.is_foot_midpoint and target.reference == FOOT_MIDPOINT_REFERENCE
                 else GENERIC_TARGET_REFERENCE)
    margin = REFERENCE_MARGIN_M.get(reference, max(REFERENCE_MARGIN_M.values()))
    return config.g1_lower_body_radius_m + margin


def validate_inputs(
    target: TargetReference | None,
    grid: GridSnapshot | None,
    robot_pose: tuple[float, float, float, int] | None,
    *,
    now_ns: int,
    map_frame: str,
    max_target_age_s: float,
    max_map_age_s: float,
    max_pose_age_s: float,
    clock_skew_s: float = 0.0,
) -> list[str]:
    """Reasons the inputs are unusable; empty when planning may proceed.

    ``clock_skew_s`` tolerates sources stamped slightly ahead of this node's clock.
    """

    future_ns = now_ns + int(clock_skew_s * 1e9)
    reasons = []
    if target is None:
        reasons.append('target_unavailable')
    else:
        if target.frame_id != map_frame:
            reasons.append(f'target_frame_{target.frame_id}')
        if not all(math.isfinite(v) for v in (target.x, target.y)):
            reasons.append('target_nonfinite')
        if (now_ns - target.stamp_ns) * 1e-9 > max_target_age_s:
            reasons.append('target_stale')
        if target.stamp_ns > future_ns:
            reasons.append('target_from_future')
    if grid is None:
        reasons.append('map_unavailable')
    else:
        if grid.frame_id != map_frame:
            reasons.append(f'map_frame_{grid.frame_id}')
        if grid.resolution <= 0.0 or grid.width <= 0 or grid.height <= 0:
            reasons.append('map_empty')
        if (now_ns - grid.stamp_ns) * 1e-9 > max_map_age_s:
            reasons.append('map_stale')
        if grid.stamp_ns > future_ns:
            reasons.append('map_from_future')
    if robot_pose is None:
        reasons.append('robot_pose_unavailable')
    else:
        if not all(math.isfinite(v) for v in robot_pose[:3]):
            reasons.append('robot_pose_nonfinite')
        if (now_ns - robot_pose[3]) * 1e-9 > max_pose_age_s:
            reasons.append('robot_pose_stale')
        if robot_pose[3] > future_ns:
            reasons.append('robot_pose_from_future')
    return reasons


def points_in_polygon(px, py, polygon: Sequence[tuple[float, float]]):
    """Vectorized ``_point_in_polygon`` (ray casting; points on an edge count as inside)."""

    px, py = np.asarray(px, dtype=float), np.asarray(py, dtype=float)
    inside = np.zeros(px.shape, dtype=bool)
    on_edge = np.zeros(px.shape, dtype=bool)
    prev = polygon[-1]
    for cur in polygon:
        ex, ey = cur[0] - prev[0], cur[1] - prev[1]
        cross = (py - prev[1]) * ex - (px - prev[0]) * ey
        dot = (px - prev[0]) * ex + (py - prev[1]) * ey
        on_edge |= (np.abs(cross) <= 1e-9) & (dot >= 0.0) & (dot <= ex * ex + ey * ey)
        straddle = (cur[1] > py) != (prev[1] > py)
        with np.errstate(divide='ignore', invalid='ignore'):
            ix = (prev[0] - cur[0]) * (py - cur[1]) / (prev[1] - cur[1]) + cur[0]
        inside ^= straddle & (px < ix)
        prev = cur
    return inside | on_edge


def points_to_polygon_distance(px, py, polygon: Sequence[tuple[float, float]]):
    """Vectorized ``_point_to_polygon_distance_m``: zero inside, edge distance outside."""

    px, py = np.asarray(px, dtype=float), np.asarray(py, dtype=float)
    best = np.full(px.shape, np.inf)
    prev = polygon[-1]
    for cur in polygon:
        sx, sy = cur[0] - prev[0], cur[1] - prev[1]
        length2 = sx * sx + sy * sy
        if length2 == 0.0:
            d = np.hypot(px - prev[0], py - prev[1])
        else:
            t = np.clip(((px - prev[0]) * sx + (py - prev[1]) * sy) / length2, 0.0, 1.0)
            d = np.hypot(px - (prev[0] + t * sx), py - (prev[1] + t * sy))
        best = np.minimum(best, d)
        prev = cur
    return np.where(points_in_polygon(px, py, polygon), 0.0, best)


def footprint_rejection_reason(
    polygon: Sequence[tuple[float, float]],
    grid: GridSnapshot,
    occupied_threshold: int,
) -> str | None:
    """Original ``_rotated_footprint_rejection_reason``: occupied > unknown > other > ok."""

    if len(polygon) < 3:
        return 'other'
    lx, ly = grid.map_to_local([p[0] for p in polygon], [p[1] for p in polygon])
    if np.any(lx < 0.0) or np.any(ly < 0.0) or np.any(lx >= grid.width * grid.resolution) \
            or np.any(ly >= grid.height * grid.resolution):
        return 'out_of_bounds'
    res = grid.resolution
    c0 = max(0, int(math.floor(lx.min() / res)) - 1)
    c1 = min(grid.width - 1, int(math.ceil(lx.max() / res)) + 1)
    r0 = max(0, int(math.floor(ly.min() / res)) - 1)
    r1 = min(grid.height - 1, int(math.ceil(ly.max() / res)) + 1)
    cols, rows = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
    cx, cy = grid.cell_centers(cols, rows)
    touching = points_to_polygon_distance(cx, cy, polygon) <= math.sqrt(2.0) * res * 0.5
    values = grid.data[r0:r1 + 1, c0:c1 + 1][touching]
    if np.any(values >= occupied_threshold):
        return 'occupied'
    if np.any(values == -1):
        return 'unknown'
    if np.any(values < 0):
        return 'other'
    return None


def footprint_clearance_m(
    polygon: Sequence[tuple[float, float]],
    grid: GridSnapshot,
    unsafe_xy: tuple[np.ndarray, np.ndarray],
    cap_m: float,
) -> float:
    """Original ``_rotated_footprint_clearance_m``, capped at ``cap_m``."""

    lx, ly = grid.map_to_local([p[0] for p in polygon], [p[1] for p in polygon])
    boundary = max(0.0, min(lx.min(), ly.min(), grid.width * grid.resolution - lx.max(),
                            grid.height * grid.resolution - ly.max()))
    clearance = min(boundary, cap_m)
    if unsafe_xy[0].size:
        d = points_to_polygon_distance(unsafe_xy[0], unsafe_xy[1], polygon)
        clearance = min(clearance, max(0.0, float(d.min()) - math.sqrt(2.0) * grid.resolution * 0.5))
    return clearance


def preference(
    task_x: float,
    task_y: float,
    target: TargetReference,
    approach_pose: tuple[float, float, float] | None,
) -> tuple[int, str]:
    """Original ``_parking_candidate_preference`` without a G1 heading: side, front, back."""

    if approach_pose is None or math.hypot(approach_pose[0] - target.x,
                                           approach_pose[1] - target.y) < 0.10:
        return 1, 'front'
    task_angle = math.atan2(task_y - target.y, task_x - target.x)
    reference_yaw = math.atan2(approach_pose[1] - target.y, approach_pose[0] - target.x)
    relative = normalize_angle(task_angle - reference_yaw)
    side_error = abs(abs(relative) - math.pi / 2.0)
    front_error = abs(relative)
    back_error = abs(abs(relative) - math.pi)
    if side_error <= front_error and side_error <= back_error:
        return 0, 'approach_side'
    if front_error <= back_error:
        return 1, 'approach_front'
    return 2, 'approach_back'


# A ray takes the first class any of its positions has. Uncertainty wins over a
# definite "blocked" or "invalid": an unmapped position might still be valid.
RAY_PRIORITY = ('valid', 'unknown', 'occupied', 'pose_invalid')


def pose_geometry_passes(task_x: float, task_y: float, target: TargetReference,
                         config: PlannerConfig, keepout_m: float) -> bool:
    """Whether a pose with its task point here passes plan_parking's G1 keep-out and reach budget."""
    base_x, base_y, yaw = base_pose_from_task_point(
        task_x, task_y, target.x, target.y, config.task_point_x_base_m, config.task_point_y_base_m)
    polygon = rotated_footprint_vertices(config.footprint, base_x, base_y, yaw)
    return (float(points_to_polygon_distance([target.x], [target.y], polygon)[0]) >= keepout_m
            and math.hypot(task_x - target.x, task_y - target.y)
            <= config.reach_max_m - config.reach_allowance_m)


def radial_profile(grid: GridSnapshot, target: TargetReference, config: PlannerConfig,
                   candidates: Sequence[CandidateDecision] = (),
                   step_deg: float = 1.0) -> list[tuple[float, str]]:
    """One ``(angle, class)`` per ray, from reach_min to reach_max (display only).

    A ray's parking positions are the donut cells it crosses (both cells where it runs
    along a cell edge) whose centre lies in the donut, as in plan_parking. Each position
    is judged as the planner judged it. A free cell takes its candidate's decision:
    ``valid``; ``occupied`` or ``unknown`` where the parked footprint is blocked or
    reaches unmapped space; ``pose_invalid`` for keep-out, reach budget or geometry. An
    occupied or unknown cell, or one outside the map, is never a candidate; it counts as
    ``occupied`` or ``unknown`` only where a pose there would pass keep-out and reach
    budget, else ``pose_invalid``. The ray takes the first class in ``RAY_PRIORITY`` that
    any of its positions has, and ``pose_invalid`` without positions or candidates. This
    is not Nav2 path feasibility, and candidates are never derived from the rays.
    """

    res = grid.resolution
    keepout = keepout_radius_m(target, config)
    decision_of = {}
    for c in candidates:
        lx, ly = grid.map_to_local(c.task_x, c.task_y)
        decision_of[(math.floor(float(ly) / res), math.floor(float(lx) / res))] = c
    px, py = (float(v) for v in grid.map_to_local(target.x, target.y))
    r0, r1 = config.reach_min_m, config.reach_max_m
    position_class = {}

    def judge(row, col):
        x, y = (float(v) for v in grid.cell_centers(col, row))
        if not r0 <= math.hypot(x - target.x, y - target.y) <= r1:
            return None                       # not a parking position
        inside = 0 <= col < grid.width and 0 <= row < grid.height
        if inside and 0 <= grid.data[row, col] < config.occupied_threshold:
            c = decision_of.get((row, col))
            if c is None:
                return 'pose_invalid'         # no decision: never green
            if c.status in ('valid', 'selected'):
                return 'valid'
            blocked = {'footprint_occupied': 'occupied', 'footprint_unknown': 'unknown',
                       'footprint_out_of_bounds': 'unknown'}.get(c.reason)
        else:
            blocked = 'occupied' if inside and grid.data[row, col] >= config.occupied_threshold else 'unknown'
        # The footprint check runs first, so a blocked pose may fail keep-out or reach too.
        return blocked if blocked and pose_geometry_passes(x, y, target, config, keepout) else 'pose_invalid'

    def indices(v):
        # A ray running exactly along a cell edge touches both cells beside it.
        k = round(v)
        return (k - 1, k) if abs(v - k) < 1e-9 else (math.floor(v),)

    rays = []
    for k in range(int(round(360.0 / step_deg))):
        angle = math.radians(k * step_deg)
        # The ray's direction in grid-local axes; cell edges lie on multiples of res there.
        dx, dy = math.cos(angle - grid.origin_yaw), math.sin(angle - grid.origin_yaw)
        cuts = {r0, r1}
        for p, d in ((px, dx), (py, dy)):
            if abs(d) > 1e-12:
                lo, hi = sorted(((p + r0 * d) / res, (p + r1 * d) / res))
                cuts.update((i * res - p) / d for i in range(math.ceil(lo), math.floor(hi) + 1))
        cuts = sorted(r for r in cuts if r0 <= r <= r1)
        found = set()
        for a, b in zip(cuts, cuts[1:]):
            if b - a < 1e-9:
                continue
            r = (a + b) / 2.0
            for row in indices((py + r * dy) / res):
                for col in indices((px + r * dx) / res):
                    if (row, col) not in position_class:
                        position_class[(row, col)] = judge(row, col)
                    found.add(position_class[(row, col)])
        rays.append((angle, next((s for s in RAY_PRIORITY if s in found), 'pose_invalid')))
    return rays


def plan_parking(
    grid: GridSnapshot,
    target: TargetReference,
    config: PlannerConfig,
    approach_pose: tuple[float, float, float] | None,
) -> ParkingPlan:
    """Generate, check and rank parking candidates exactly in the original order."""

    keepout = keepout_radius_m(target, config)
    plan = ParkingPlan(ok=False, failure_reason=None, target=target, keepout_radius_m=keepout,
                       resolution_m=grid.resolution)

    # 1. Donut task cells, row-major like the original loop (this order is the final tie-break).
    rows, cols = np.mgrid[0:grid.height, 0:grid.width]
    cx, cy = grid.cell_centers(cols, rows)
    distance = np.hypot(cx - target.x, cy - target.y)
    in_donut = (distance >= config.reach_min_m) & (distance <= config.reach_max_m)
    values = grid.data
    for mask, bucket in (
        (in_donut & (values == -1), plan.unknown_cells),
        (in_donut & (values >= config.occupied_threshold), plan.occupied_cells),
        (in_donut & (values >= 0) & (values < config.occupied_threshold), plan.free_cells),
    ):
        bucket.extend(zip(cx[mask].tolist(), cy[mask].tolist()))
    if not plan.free_cells:
        plan.radial_rays = radial_profile(grid, target, config)
        plan.failure_reason = 'no_free_donut_cells'
        return plan

    # Unsafe cells near G1 for clearance (the original scanned the whole grid).
    near = np.hypot(cx - target.x, cy - target.y) <= (
        config.reach_max_m + abs(config.task_point_x_base_m) + _footprint_radius(config.footprint)
        + config.clearance_cap_m)
    unsafe = near & ((values == -1) | (values >= config.occupied_threshold))
    unsafe_xy = (cx[unsafe], cy[unsafe])

    # 2-4. Base pose per free task cell, footprint occupancy, then G1 keep-out.
    for task_x, task_y in plan.free_cells:
        base_x, base_y, yaw = base_pose_from_task_point(
            task_x, task_y, target.x, target.y,
            config.task_point_x_base_m, config.task_point_y_base_m)
        reach = math.hypot(task_x - target.x, task_y - target.y)
        decision = CandidateDecision(task_x, task_y, base_x, base_y, yaw, 'valid', 'footprint_free', reach)
        check_x, check_y = task_point_from_base_pose(
            base_x, base_y, yaw, config.task_point_x_base_m, config.task_point_y_base_m)
        polygon = rotated_footprint_vertices(config.footprint, base_x, base_y, yaw)
        if abs(check_x - task_x) > 1e-6 or abs(check_y - task_y) > 1e-6:
            decision.status, decision.reason = 'rejected', 'footprint_other'
        else:
            rejection = footprint_rejection_reason(polygon, grid, config.occupied_threshold)
            if rejection is not None:
                decision.status = 'collision' if rejection == 'occupied' else 'rejected'
                decision.reason = f'footprint_{rejection}'
            elif float(points_to_polygon_distance([target.x], [target.y], polygon)[0]) < keepout:
                decision.status, decision.reason = 'collision', 'g1_keepout'
            elif reach > config.reach_max_m - config.reach_allowance_m:
                decision.status, decision.reason = 'rejected', 'reach_budget'
        if decision.status == 'valid':
            decision.clearance_m = footprint_clearance_m(polygon, grid, unsafe_xy, config.clearance_cap_m)
        plan.candidates.append(decision)
    plan.radial_rays = radial_profile(grid, target, config, plan.candidates)

    valid = [c for c in plan.candidates if c.status == 'valid']
    if not valid:
        plan.failure_reason = 'no_valid_candidate'
        return plan

    # 5. Original tiers: preference class, preferred-reach error within one cell,
    # clearance within one cell, (Nav2 path length: not available), input order.
    preferred_reach = (config.reach_min_m + config.reach_max_m) / 2.0
    ranked = []
    for c in valid:
        rank, c.preference_label = preference(c.task_x, c.task_y, target, approach_pose)
        c.reach_error_m = abs(c.reach_m - preferred_reach)
        ranked.append((rank, c))
    best_rank = min(rank for rank, _ in ranked)
    tier = [c for rank, c in ranked if rank == best_rank]
    best_error = min(c.reach_error_m for c in tier)
    tier = [c for c in tier if c.reach_error_m <= best_error + grid.resolution]
    best_clearance = max(c.clearance_m for c in tier)
    tier = [c for c in tier if c.clearance_m >= best_clearance - grid.resolution]
    plan.selected = tier[0]
    plan.selected.status = 'selected'
    plan.ok = True
    return plan


def _footprint_radius(footprint: Sequence[tuple[float, float]]) -> float:
    return max(math.hypot(x, y) for x, y in footprint)
