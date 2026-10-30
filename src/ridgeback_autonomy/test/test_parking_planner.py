"""Parking candidate generation: occupancy, footprint, keep-out, ranking, freshness, markers."""

from __future__ import annotations

import math
import random
from dataclasses import replace

import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

from ridgeback_autonomy.mission.parking_geometry import rotated_footprint_vertices
from ridgeback_autonomy.mission.parking_planner import (
    FOOT_MIDPOINT_REFERENCE,
    GENERIC_TARGET_REFERENCE,
    GridSnapshot,
    PlannerConfig,
    TargetReference,
    footprint_rejection_reason,
    keepout_radius_m,
    plan_parking,
    points_in_polygon,
    points_to_polygon_distance,
    validate_inputs,
)
from ridgeback_autonomy.mission.parking_visualization import clear_markers, plan_markers


# The current 16-point Nav2 footprint (config/nav2_params.yaml, unpadded).
FOOTPRINT = (
    (0.497107, 0.263718), (0.477026, 0.338663), (0.422163, 0.393526), (0.347218, 0.413607),
    (-0.347218, 0.413607), (-0.422163, 0.393526), (-0.477026, 0.338663), (-0.497107, 0.263718),
    (-0.497107, -0.263718), (-0.477026, -0.338663), (-0.422163, -0.393526), (-0.347218, -0.413607),
    (0.347218, -0.413607), (0.422163, -0.393526), (0.477026, -0.338663), (0.497107, -0.263718),
)
CONFIG = PlannerConfig(footprint=FOOTPRINT)
RES = 0.05


def grid(size_m=6.0, value=0, origin=(0.0, 0.0), yaw=0.0, stamp_ns=10_000_000_000):
    n = int(round(size_m / RES))
    return GridSnapshot('map', stamp_ns, RES, n, n, origin[0], origin[1], yaw,
                        np.full((n, n), value, dtype=np.int16))


def fill(g, x0, y0, x1, y1, value):
    """Set every cell whose centre lies in the map-frame box (axis-aligned grids only)."""
    rows, cols = np.mgrid[0:g.height, 0:g.width]
    cx, cy = g.cell_centers(cols, rows)
    g.data[(cx >= x0) & (cx <= x1) & (cy >= y0) & (cy <= y1)] = value


def target(x=3.0, y=3.0, *, foot=False, stamp_ns=10_000_000_000):
    return TargetReference(x, y, 'map', stamp_ns,
                           FOOT_MIDPOINT_REFERENCE if foot else GENERIC_TARGET_REFERENCE, foot,
                           'euclidean_reconstruction')


APPROACH_FROM_WEST = (1.0, 3.0, 0.0)


def test_free_map_selects_a_side_candidate_with_rear_toward_g1():
    plan = plan_parking(grid(), target(), CONFIG, APPROACH_FROM_WEST)

    assert plan.ok and plan.failure_reason is None
    sel = plan.selected
    assert CONFIG.reach_min_m <= sel.reach_m <= CONFIG.reach_max_m
    # Base +x points away from G1, so G1 lies behind the base (original convention).
    to_g1 = math.atan2(3.0 - sel.base_y, 3.0 - sel.base_x)
    assert abs(math.atan2(math.sin(to_g1 - sel.yaw - math.pi), math.cos(to_g1 - sel.yaw - math.pi))) < 1e-6
    # Approach from -x: the original prefers lateral sides (about +-90 deg from the approach ray).
    assert sel.preference_label == 'approach_side'
    assert abs(abs(math.atan2(sel.task_y - 3.0, sel.task_x - 3.0)) - math.pi / 2) < math.radians(45)
    assert plan.occupied_cells == [] and plan.unknown_cells == []


def test_obstacle_inside_donut_is_classified_and_never_a_candidate():
    g = grid()
    fill(g, 3.3, 2.7, 3.7, 3.3, 100)   # east part of the donut
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)

    assert plan.occupied_cells
    candidate_tasks = {(round(c.task_x, 3), round(c.task_y, 3)) for c in plan.candidates}
    assert not any((round(x, 3), round(y, 3)) in candidate_tasks for x, y in plan.occupied_cells)
    assert plan.ok


def test_obstacle_touching_only_the_footprint_rejects_as_collision():
    g = grid()
    # A wall north of the donut: clear of every task cell but inside northern footprints.
    fill(g, 2.0, 4.0, 4.0, 4.2, 100)
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)

    assert not plan.occupied_cells
    collided = [c for c in plan.candidates if c.reason == 'footprint_occupied']
    assert collided and all(c.status == 'collision' and c.task_y > 3.0 for c in collided)
    assert plan.selected.task_y < 3.0 or plan.selected.reason != 'footprint_occupied'


def test_unknown_cells_are_excluded_and_reject_footprints():
    g = grid()
    fill(g, 0.0, 0.0, 6.0, 2.9, -1)   # everything south of G1 is unexplored
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)

    assert plan.unknown_cells
    assert any(c.reason == 'footprint_unknown' and c.status == 'rejected' for c in plan.candidates)
    assert plan.selected.task_y > 3.0


def test_keepout_depends_on_reference_and_removes_close_candidates():
    generic, foot = target(), target(foot=True)
    assert keepout_radius_m(generic, CONFIG) == pytest.approx(0.29)
    assert keepout_radius_m(foot, CONFIG) == pytest.approx(0.24)
    unknown = TargetReference(3, 3, 'map', 0, 'something_else', False, 'x')
    assert keepout_radius_m(unknown, CONFIG) == pytest.approx(0.29)
    # Neither an isolated flag nor an isolated reference label certifies feet.
    assert keepout_radius_m(replace(generic, is_foot_midpoint=True), CONFIG) == pytest.approx(0.29)
    assert keepout_radius_m(replace(foot, is_foot_midpoint=False), CONFIG) == pytest.approx(0.29)

    plan_generic = plan_parking(grid(), generic, CONFIG, APPROACH_FROM_WEST)
    plan_foot = plan_parking(grid(), foot, CONFIG, APPROACH_FROM_WEST)
    keep_generic = [c for c in plan_generic.candidates if c.reason == 'g1_keepout']
    keep_foot = [c for c in plan_foot.candidates if c.reason == 'g1_keepout']
    assert keep_generic and len(keep_generic) > len(keep_foot)
    # Rear edge is 0.217 m behind the task point: reach < 0.29 + 0.217 cannot survive.
    assert all(c.reach_m < 0.507 + RES for c in keep_generic)
    for plan in (plan_generic, plan_foot):
        polygon = rotated_footprint_vertices(FOOTPRINT, plan.selected.base_x, plan.selected.base_y,
                                             plan.selected.yaw)
        assert points_to_polygon_distance([3.0], [3.0], polygon)[0] >= plan.keepout_radius_m


def test_no_feasible_candidate_reports_reason():
    occupied = plan_parking(grid(value=100), target(), CONFIG, APPROACH_FROM_WEST)
    assert not occupied.ok and occupied.failure_reason == 'no_free_donut_cells'
    assert occupied.selected is None

    g = grid()
    fill(g, 0.0, 0.0, 6.0, 6.0, 100)
    fill(g, 2.35, 2.35, 3.65, 3.65, 0)   # free donut, but every footprint hits the wall
    boxed = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)
    assert not boxed.ok and boxed.failure_reason == 'no_valid_candidate'
    assert boxed.counts()['valid'] == 0


def test_ranking_follows_original_tiers_and_is_deterministic():
    plan_a = plan_parking(grid(), target(), CONFIG, APPROACH_FROM_WEST)
    plan_b = plan_parking(grid(), target(), CONFIG, APPROACH_FROM_WEST)
    assert (plan_a.selected.task_x, plan_a.selected.task_y) == (plan_b.selected.task_x, plan_b.selected.task_y)

    valid = [c for c in plan_a.candidates if c.status in ('valid', 'selected')]
    assert len(valid) > 1
    side = [c for c in valid if c.preference_label == 'approach_side']
    best_error = min(c.reach_error_m for c in side)
    assert plan_a.selected.reach_error_m <= best_error + RES
    # Approaching from the north instead flips the lateral sides to east/west.
    north = plan_parking(grid(), target(), CONFIG, (3.0, 5.0, 0.0))
    assert abs(math.atan2(north.selected.task_y - 3.0, north.selected.task_x - 3.0)) in (
        pytest.approx(0.0, abs=math.radians(45)), pytest.approx(math.pi, abs=math.radians(45)))


def test_rotated_footprint_detects_obstacle_only_inside_rotated_corner():
    g = grid()
    base = (3.0, 3.0)
    yaw = math.radians(45)
    polygon = rotated_footprint_vertices(FOOTPRINT, base[0], base[1], yaw)
    axis_aligned = rotated_footprint_vertices(FOOTPRINT, base[0], base[1], 0.0)
    # Robot-frame (0.45, 0.30) is inside the footprint; rotated 45 deg it lands beyond
    # the axis-aligned footprint's +-0.414 m lateral extent.
    local = (0.45, 0.30)
    probe = (base[0] + math.cos(yaw) * local[0] - math.sin(yaw) * local[1],
             base[1] + math.sin(yaw) * local[0] + math.cos(yaw) * local[1])
    assert points_in_polygon([probe[0]], [probe[1]], polygon)[0]
    assert not points_in_polygon([probe[0]], [probe[1]], axis_aligned)[0]
    g.data[int(probe[1] // RES), int(probe[0] // RES)] = 100   # the cell containing the probe
    assert footprint_rejection_reason(polygon, g, 50) == 'occupied'
    assert footprint_rejection_reason(axis_aligned, g, 50) is None
    assert footprint_rejection_reason(rotated_footprint_vertices(FOOTPRINT, 1.0, 1.0, yaw), g, 50) is None


def test_rotated_and_offset_map_origin_gives_the_same_geometry():
    plain = plan_parking(grid(), target(), CONFIG, APPROACH_FROM_WEST)
    # Same free world expressed with a rotated, offset grid that still covers the target.
    rotated = grid(size_m=8.0, origin=(7.0, -1.0), yaw=math.pi / 2)
    lx, ly = rotated.map_to_local(3.0, 3.0)
    assert 0 <= lx <= 8.0 and 0 <= ly <= 8.0
    cx, cy = rotated.cell_centers(np.array([0]), np.array([0]))
    assert (cx[0], cy[0]) == pytest.approx((7.0 - 0.025, -1.0 + 0.025))
    back_x, back_y = rotated.map_to_local(cx, cy)
    assert (back_x[0], back_y[0]) == pytest.approx((0.025, 0.025))
    plan_rot = plan_parking(rotated, target(), CONFIG, APPROACH_FROM_WEST)
    assert plan_rot.ok
    assert plan_rot.selected.reach_m == pytest.approx(plain.selected.reach_m, abs=RES)
    assert plan_rot.selected.preference_label == plain.selected.preference_label


def test_stale_or_inconsistent_inputs_are_rejected():
    now = 20_000_000_000
    kwargs = dict(now_ns=now, map_frame='map', max_target_age_s=5.0, max_map_age_s=5.0,
                  max_pose_age_s=1.0)
    fresh_pose = (1.0, 2.0, 0.0, now)
    assert validate_inputs(target(stamp_ns=now), grid(stamp_ns=now), fresh_pose, **kwargs) == []
    assert 'target_stale' in validate_inputs(target(stamp_ns=now - 6_000_000_000),
                                             grid(stamp_ns=now), fresh_pose, **kwargs)
    assert 'map_stale' in validate_inputs(target(stamp_ns=now), grid(stamp_ns=now - 6_000_000_000),
                                          fresh_pose, **kwargs)
    wrong = GridSnapshot('odom', now, RES, 4, 4, 0, 0, 0, np.zeros((4, 4), dtype=np.int16))
    assert 'map_frame_odom' in validate_inputs(target(stamp_ns=now), wrong, fresh_pose, **kwargs)
    reasons = validate_inputs(None, None, None, **kwargs)
    assert {'target_unavailable', 'map_unavailable', 'robot_pose_unavailable'} <= set(reasons)
    assert 'robot_pose_stale' in validate_inputs(target(stamp_ns=now), grid(stamp_ns=now),
                                                 (1.0, 2.0, 0.0, now - 2_000_000_000), **kwargs)


@pytest.mark.parametrize('field', ['target', 'map', 'robot_pose'])
def test_future_inputs_are_rejected(field):
    now = 10_000_000_000
    future = now + 1
    reasons = validate_inputs(
        target(stamp_ns=future if field == 'target' else now),
        grid(stamp_ns=future if field == 'map' else now),
        (1.0, 2.0, 0.0, future if field == 'robot_pose' else now),
        now_ns=now, map_frame='map', max_target_age_s=5.0,
        max_map_age_s=5.0, max_pose_age_s=1.0)
    assert reasons == [f'{field}_from_future']


@pytest.mark.parametrize('changes', [
    {'resolution': float('nan')}, {'resolution': 0.0},
    {'origin_x': float('nan')}, {'origin_yaw': float('inf')},
    {'width': 0}, {'data': np.zeros((2, 2), dtype=np.int16)},
    {'data': np.full((120, 120), -2, dtype=np.int16)},
    {'data': np.full((120, 120), 101, dtype=np.int16)},
])
def test_invalid_grid_snapshot_is_rejected(changes):
    with pytest.raises(ValueError):
        replace(grid(), **changes)


@pytest.mark.parametrize('rotation', [
    (0.0, 0.0, 0.0, 0.0), (0.0, 0.0, float('nan'), 1.0),
    (math.sin(0.1), 0.0, 0.0, math.cos(0.1)),
])
def test_invalid_or_tilted_map_origin_is_rejected(rotation):
    msg = OccupancyGrid()
    msg.info.width = msg.info.height = 2
    msg.info.resolution = RES
    msg.data = [0] * 4
    q = msg.info.origin.orientation
    q.x, q.y, q.z, q.w = rotation
    with pytest.raises(ValueError, match='map origin'):
        GridSnapshot.from_msg(msg)


def test_map_message_normalizes_yaw_and_preserves_row_major_cells():
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width, msg.info.height = 3, 2
    msg.info.resolution = RES
    msg.info.origin.position.x, msg.info.origin.position.y = 2.0, -1.0
    msg.info.origin.orientation.z = 2.0 * math.sin(math.pi / 4)
    msg.info.origin.orientation.w = 2.0 * math.cos(math.pi / 4)
    msg.data = [0, 49, 50, -1, 0, 100]
    g = GridSnapshot.from_msg(msg)
    assert g.origin_yaw == pytest.approx(math.pi / 2)
    assert g.data.tolist() == [[0, 49, 50], [-1, 0, 100]]
    assert tuple(g.cell_centers(0, 0)) == pytest.approx((1.975, -0.975))


def test_footprint_edge_contact_and_map_boundary_are_conservative():
    g = grid()
    polygon = [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)]
    # This cell is outside the polygon, but its square touches the edge.
    g.data[30, 40] = 50
    assert footprint_rejection_reason(polygon, g, 50) == 'occupied'
    g.data[30, 40] = -1
    assert footprint_rejection_reason(polygon, g, 50) == 'unknown'
    assert footprint_rejection_reason(
        rotated_footprint_vertices(FOOTPRINT, 0.1, 0.1, 0.3), g, 50) == 'out_of_bounds'


def test_only_approach_back_available_is_selected_without_fabricating_g1_yaw():
    g = grid()
    fill(g, 0.0, 0.0, 3.2, 6.0, 100)
    # Restore all task points, but not the space needed for west/north/south footprints.
    fill(g, 2.35, 2.35, 3.65, 3.65, 0)
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)
    assert plan.ok
    assert plan.selected.preference_label == 'approach_back'
    assert plan.selected.base_x > target().x
    assert all(c.preference_label == 'approach_back' for c in plan.candidates
               if c.status in ('valid', 'selected'))


def _scalar_point_in_polygon(x, y, polygon):
    """The original ``_point_in_polygon`` / ``_point_on_segment``, verbatim in logic."""
    inside = False
    previous = polygon[-1]
    for current in polygon:
        cross = (y - previous[1]) * (current[0] - previous[0]) - (x - previous[0]) * (current[1] - previous[1])
        dot = (x - previous[0]) * (current[0] - previous[0]) + (y - previous[1]) * (current[1] - previous[1])
        if abs(cross) <= 1e-9 and 0.0 <= dot <= (current[0] - previous[0]) ** 2 + (current[1] - previous[1]) ** 2:
            return True
        if (current[1] > y) != (previous[1] > y):
            ix = (previous[0] - current[0]) * (y - current[1]) / (previous[1] - current[1]) + current[0]
            if x < ix:
                inside = not inside
        previous = current
    return inside


def _scalar_distance(x, y, polygon):
    if _scalar_point_in_polygon(x, y, polygon):
        return 0.0
    best = math.inf
    previous = polygon[-1]
    for current in polygon:
        sx, sy = current[0] - previous[0], current[1] - previous[1]
        t = min(1.0, max(0.0, ((x - previous[0]) * sx + (y - previous[1]) * sy) / (sx * sx + sy * sy)))
        best = min(best, math.hypot(x - previous[0] - t * sx, y - previous[1] - t * sy))
        previous = current
    return best


def test_vectorized_polygon_math_matches_the_original_scalar_routines():
    rng = random.Random(7)
    polygon = rotated_footprint_vertices(FOOTPRINT, 1.3, -0.4, 0.7)
    xs = [rng.uniform(0.0, 2.6) for _ in range(500)]
    ys = [rng.uniform(-1.7, 0.9) for _ in range(500)]
    inside = points_in_polygon(xs, ys, polygon)
    dist = points_to_polygon_distance(xs, ys, polygon)
    for k in range(500):
        assert bool(inside[k]) == _scalar_point_in_polygon(xs[k], ys[k], polygon)
        assert dist[k] == pytest.approx(_scalar_distance(xs[k], ys[k], polygon), abs=1e-9)


def test_markers_clear_first_and_show_plan_semantics():
    import rclpy.time
    stamp = rclpy.time.Time(seconds=10).to_msg()
    plan = plan_parking(grid(), target(), CONFIG, APPROACH_FROM_WEST)
    markers = plan_markers(plan, CONFIG, stamp).markers

    assert markers[0].action == Marker.DELETEALL
    namespaces = {m.ns for m in markers[1:]}
    for ns in ('parking/g1_position', 'parking/g1_keepout', 'parking/donut_boundaries',
               'parking/radial_1deg',
               'parking/selected_pose', 'parking/selected_footprint', 'parking/status'):
        assert ns in namespaces
    # The donut is drawn as lines only: no cell, candidate or path-check dots.
    assert not namespaces & {'parking/donut_cells', 'parking/candidate_task_points',
                             'parking/rejected_task_points', 'parking/path_candidates'}
    assert not any(m.type in (Marker.SPHERE_LIST, Marker.POINTS) for m in markers)
    assert all(m.header.frame_id == 'map' for m in markers[1:])
    green = next(m for m in markers if m.ns == 'parking/radial_1deg' and m.id == 0)
    assert green.type == Marker.LINE_LIST and green.points
    assert green.color.g > 0.8 and green.color.r < 0.2   # green = valid parking position

    failed = plan_parking(grid(value=100), target(), CONFIG, APPROACH_FROM_WEST)
    failed_markers = plan_markers(failed, CONFIG, stamp).markers
    assert failed_markers[0].action == Marker.DELETEALL
    assert not any(m.ns.startswith('parking/selected') for m in failed_markers)
    occupied = next(m for m in failed_markers if m.ns == 'parking/radial_1deg' and m.id == 1)
    assert len(occupied.points) == 720   # every ray is one obstacle run
    assert occupied.color.r > 0.9 and occupied.color.g < 0.1   # red = obstacle
    cleared = clear_markers().markers
    assert len(cleared) == 1 and cleared[0].action == Marker.DELETEALL


def test_reach_budget_rejects_candidates_beyond_the_allowed_planned_reach():
    from dataclasses import replace
    budget = replace(CONFIG, reach_allowance_m=0.06)
    plan = plan_parking(grid(), target(), budget, APPROACH_FROM_WEST)
    assert plan.ok
    assert all(c.reach_m <= 0.54 for c in plan.candidates if c.status in ('valid', 'selected'))
    assert any(c.reason == 'reach_budget' for c in plan.candidates)
    assert plan.selected.reach_m <= 0.54


# Candidate-geometry audit: every candidate, at off-grid G1 positions and on a rotated map.
from ridgeback_autonomy.mission.parking_geometry import normalize_angle, task_point_from_base_pose
from ridgeback_autonomy.mission.parking_planner import G1_LOWER_BODY_RADIUS_M

REAR_OVERHANG = 0.497107 - 0.280     # footprint rear edge behind the rear-bin task point
BUDGET = replace(CONFIG, reach_allowance_m=0.06)   # the node's 0.03 final + 0.03 reference
OFF_GRID = [
    # (G1 x, y), map yaw, approach pose
    ((2.93, 3.17), 0.0, (4.6, 1.4, 2.3)),
    ((3.11, 2.87), math.radians(70), (1.2, 4.9, -0.9)),
]


def centred_grid(tx, ty, yaw, size_m=8.0):
    """A free grid of the given yaw whose local centre is the target."""
    c, s = math.cos(yaw), math.sin(yaw)
    h = size_m / 2.0
    return grid(size_m=size_m, origin=(tx - c * h + s * h, ty - s * h - c * h), yaw=yaw)


def touching(polygon, g, mask):
    """Brute force over the whole grid: does any masked cell touch the polygon?"""
    rows, cols = np.nonzero(mask)
    cx, cy = g.cell_centers(cols, rows)
    return bool(np.any(points_to_polygon_distance(cx, cy, polygon) <= math.sqrt(2.0) * RES * 0.5))


def mark(g, points_xy, value):
    for x, y in points_xy:
        lx, ly = g.map_to_local(x, y)
        g.data[int(ly // RES), int(lx // RES)] = value


@pytest.mark.parametrize('g1,map_yaw,approach', OFF_GRID)
def test_every_candidate_puts_the_rear_task_point_in_the_donut_with_g1_behind(g1, map_yaw, approach):
    g = centred_grid(*g1, map_yaw)
    plan = plan_parking(g, target(*g1), CONFIG, approach)

    rows, cols = np.mgrid[0:g.height, 0:g.width]
    cx, cy = g.cell_centers(cols, rows)
    d = np.hypot(cx - g1[0], cy - g1[1])
    assert len(plan.candidates) == len(plan.free_cells) == int(np.sum((d >= 0.30) & (d <= 0.60)))
    for c in plan.candidates:
        # The (-0.28, 0) base_link task point is the candidate's donut cell...
        assert task_point_from_base_pose(c.base_x, c.base_y, c.yaw, -0.28, 0.0) == pytest.approx(
            (c.task_x, c.task_y), abs=1e-9)
        # ...and reach is measured from it to the G1 reference, never from base_link.
        assert c.reach_m == pytest.approx(math.hypot(c.task_x - g1[0], c.task_y - g1[1]))
        assert 0.30 <= c.reach_m <= 0.60
        assert math.hypot(c.base_x - g1[0], c.base_y - g1[1]) == pytest.approx(c.reach_m + 0.28)
        # G1 lies exactly behind the base (yaw + pi), with yaw normalized to (-pi, pi].
        assert -math.pi < c.yaw <= math.pi
        to_g1 = math.atan2(g1[1] - c.base_y, g1[0] - c.base_x)
        assert normalize_angle(to_g1 - c.yaw - math.pi) == pytest.approx(0.0, abs=1e-9)
    assert any(abs(c.yaw) > 3.0 for c in plan.candidates)   # the +-pi wrap is exercised


def test_g1_reference_is_used_as_given_without_hidden_offsets():
    shift = (0.73, -1.18)
    plain = plan_parking(grid(), target(), CONFIG, APPROACH_FROM_WEST)
    moved = plan_parking(grid(origin=shift), target(3.0 + shift[0], 3.0 + shift[1]), CONFIG,
                         (APPROACH_FROM_WEST[0] + shift[0], APPROACH_FROM_WEST[1] + shift[1], 0.0))

    assert len(plain.candidates) == len(moved.candidates)
    for a, b in zip(plain.candidates, moved.candidates):
        assert (b.base_x - a.base_x, b.base_y - a.base_y) == pytest.approx(shift)
        assert (b.yaw, b.status, b.reason) == (pytest.approx(a.yaw), a.status, a.reason)
    assert moved.selected.task_x - plain.selected.task_x == pytest.approx(shift[0])


@pytest.mark.parametrize('foot', [False, True])
def test_keepout_covers_the_feet_at_any_g1_heading(foot):
    # The planner takes no G1 heading: the 0.19 m envelope must cover the static SDF
    # feet (+-0.104 m along G1 x, +-0.156 m lateral) about the foot midpoint at any yaw.
    corners = [(sx * 0.104, sy * 0.156) for sx in (-1, 1) for sy in (-1, 1)]
    plan = plan_parking(grid(), target(foot=foot), CONFIG, APPROACH_FROM_WEST)
    for c in [c for c in plan.candidates if c.status in ('valid', 'selected')]:
        polygon = rotated_footprint_vertices(FOOTPRINT, c.base_x, c.base_y, c.yaw)
        for heading in np.radians(np.arange(0, 360, 15)):
            feet = rotated_footprint_vertices(corners, 3.0, 3.0, heading)
            assert max(math.hypot(x - 3.0, y - 3.0) for x, y in feet) <= G1_LOWER_BODY_RADIUS_M
            gap = points_to_polygon_distance([p[0] for p in feet], [p[1] for p in feet], polygon)
            assert gap.min() >= plan.keepout_radius_m - G1_LOWER_BODY_RADIUS_M - 1e-9


@pytest.mark.parametrize('g1,map_yaw,approach', OFF_GRID)
def test_rotated_table_legs_reject_exactly_the_footprints_they_touch(g1, map_yaw, approach):
    g = centred_grid(*g1, map_yaw)
    # A 0.9 x 0.6 m table rotated 30 deg beside G1: only its legs are in the LiDAR plane.
    legs = rotated_footprint_vertices([(0.45, 0.3), (0.45, -0.3), (-0.45, -0.3), (-0.45, 0.3)],
                                      g1[0] + 0.85, g1[1] + 0.55, math.radians(30))
    mark(g, legs, 100)
    plan = plan_parking(g, target(*g1), CONFIG, approach)

    occupied = g.data >= CONFIG.occupied_threshold
    hit = 0
    for c in plan.candidates:
        polygon = rotated_footprint_vertices(FOOTPRINT, c.base_x, c.base_y, c.yaw)
        touches = touching(polygon, g, occupied)
        hit += touches
        assert touches == (c.reason == 'footprint_occupied')
        assert not touches or c.status == 'collision'
    assert hit and plan.ok
    sel = plan.selected
    assert not touching(rotated_footprint_vertices(FOOTPRINT, sel.base_x, sel.base_y, sel.yaw),
                        g, occupied)


@pytest.mark.parametrize('g1,map_yaw,approach', OFF_GRID)
def test_unknown_cells_touching_a_rotated_footprint_reject_it(g1, map_yaw, approach):
    g = centred_grid(*g1, map_yaw)
    # A diagonal unexplored streak outside the donut, as left behind by a LiDAR shadow.
    streak = [(g1[0] - 0.9 + 0.03 * k, g1[1] + 0.4 + 0.03 * k) for k in range(30)]
    mark(g, streak, -1)
    plan = plan_parking(g, target(*g1), CONFIG, approach)

    assert not plan.unknown_cells   # nothing unknown inside the donut itself
    unknown = g.data == -1
    rejected = [c for c in plan.candidates if c.reason == 'footprint_unknown']
    assert rejected and all(c.status == 'rejected' for c in rejected)
    for c in plan.candidates:
        polygon = rotated_footprint_vertices(FOOTPRINT, c.base_x, c.base_y, c.yaw)
        assert touching(polygon, g, unknown) == (c.reason == 'footprint_unknown')


@pytest.mark.parametrize('foot', [False, True])
@pytest.mark.parametrize('g1,map_yaw,approach', OFF_GRID)
def test_keepout_and_reach_budget_bound_the_valid_band(g1, map_yaw, approach, foot):
    plan = plan_parking(centred_grid(*g1, map_yaw), target(*g1, foot=foot), BUDGET, approach)

    keepout = plan.keepout_radius_m
    for c in plan.candidates:
        polygon = rotated_footprint_vertices(FOOTPRINT, c.base_x, c.base_y, c.yaw)
        gap = points_to_polygon_distance([g1[0]], [g1[1]], polygon)[0]
        # G1 sits on the base axis, so its distance to the footprint is reach - rear overhang.
        assert gap == pytest.approx(c.reach_m - REAR_OVERHANG)
        assert (c.reason == 'g1_keepout') == (gap < keepout)
        assert (c.reason == 'reach_budget') == (gap >= keepout and c.reach_m > 0.54)
    valid = [c.reach_m for c in plan.candidates if c.status in ('valid', 'selected')]
    assert valid and min(valid) >= keepout + REAR_OVERHANG and max(valid) <= 0.54


def test_ranking_prefers_mid_reach_and_stays_inside_the_budgeted_band():
    for foot in (True, False):
        plan = plan_parking(grid(), target(foot=foot), BUDGET, APPROACH_FROM_WEST)
        valid = [c.reach_m for c in plan.candidates if c.status in ('valid', 'selected')
                 and c.preference_label == plan.selected.preference_label]
        # The 0.45 m preferred reach lies below the keep-out edge, so the tier keeps the
        # innermost reachable cells (within one cell) rather than the 0.54 m outer cap.
        assert plan.selected.reach_m <= min(valid) + RES
        assert keepout_radius_m(plan.target, BUDGET) + REAR_OVERHANG <= plan.selected.reach_m <= 0.54
    foot_plan = plan_parking(grid(), target(foot=True), BUDGET, APPROACH_FROM_WEST)
    assert foot_plan.selected.reach_m < 0.54 - RES / 2


# 1-degree radial donut lines: one class per complete ray, from the planner's own decisions.
from ridgeback_autonomy.mission.parking_planner import radial_profile

RAY_IDS = {'valid': 0, 'occupied': 1, 'unknown': 2, 'pose_invalid': 3}
PRIORITY = ('valid', 'unknown', 'occupied', 'pose_invalid')
BLOCKED = {'footprint_occupied': 'occupied', 'footprint_unknown': 'unknown',
           'footprint_out_of_bounds': 'unknown'}
GEOMETRY_ONLY = {'g1_keepout', 'reach_budget', 'footprint_other'}


def ray(plan, degrees):
    return plan.radial_rays[degrees][1]


def cell_of(g, x, y):
    lx, ly = g.map_to_local(x, y)
    return math.floor(float(ly) / RES), math.floor(float(lx) / RES)


def ray_positions(plan, g, degrees):
    """Independent sampling, every 1 mm: the donut-centred cells the ray touches."""
    t, a = plan.target, math.radians(degrees)
    near = lambda v: (round(v) - 1, round(v)) if abs(v - round(v)) < 1e-6 else (math.floor(v),)
    cells = set()
    for r in np.linspace(CONFIG.reach_min_m, CONFIG.reach_max_m, 301):
        lx, ly = (float(v) / RES for v in g.map_to_local(t.x + r * math.cos(a), t.y + r * math.sin(a)))
        cells.update((row, col) for row in near(ly) for col in near(lx))
    centre = lambda rc: tuple(float(v) for v in g.cell_centers(rc[1], rc[0]))
    return {rc for rc in cells
            if CONFIG.reach_min_m <= math.hypot(centre(rc)[0] - t.x, centre(rc)[1] - t.y) <= CONFIG.reach_max_m}


def expected_rays(plan, g):
    """Oracle: each position judged by the planner's decision for its cell, with keep-out and
    reach taken from the planner itself on a free copy of the map, then the documented priority."""
    free = replace(g, data=np.zeros_like(g.data))
    geometry_fails = {cell_of(free, c.task_x, c.task_y): c.reason in GEOMETRY_ONLY
                      for c in plan_parking(free, plan.target, CONFIG, None).candidates}
    decided = {cell_of(g, c.task_x, c.task_y): c for c in plan.candidates}
    expected = []
    for k in range(360):
        found = set()
        for row, col in ray_positions(plan, g, k):
            value = g.data[row, col]
            c = decided.get((row, col))
            if c is not None and c.status in ('valid', 'selected'):
                found.add('valid')
                continue
            blocked = (BLOCKED.get(c.reason) if c is not None else
                       'occupied' if value >= CONFIG.occupied_threshold else 'unknown')
            found.add(blocked if blocked and not geometry_fails[(row, col)] else 'pose_invalid')
        expected.append(next(s for s in PRIORITY if s in found))
    return expected


def assert_rays_match_the_planner(plan, g):
    assert [s for _, s in plan.radial_rays] == expected_rays(plan, g)


def test_radial_profile_has_360_one_degree_rays_with_wraparound():
    plan = plan_parking(grid(), target(), CONFIG, APPROACH_FROM_WEST)
    angles = [a for a, _ in plan.radial_rays]
    assert len(angles) == 360
    assert angles == [math.radians(k) for k in range(360)]
    assert angles[0] == 0.0 and math.degrees(2 * math.pi - angles[-1]) == pytest.approx(1.0)
    # An obstacle straddling 0 deg shows on both sides of the wrap, symmetrically.
    g = grid()
    fill(g, 3.40, 2.95, 3.50, 3.05, 100)
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)
    assert_rays_match_the_planner(plan, g)
    occupied = {k for k, (_, s) in enumerate(plan.radial_rays) if s == 'occupied'}
    assert {359, 0, 1} <= occupied
    assert occupied == {(360 - k) % 360 for k in occupied}
    # Rays whose poses the obstacle blocks; never the far side.
    assert all(k <= 90 or k >= 270 for k in occupied)


def test_free_map_rays_are_green_wherever_a_valid_pose_exists():
    g = grid()
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)
    assert_rays_match_the_planner(plan, g)
    # Inside the keep-out edge (0.29 + 0.217 m) no pose is valid, but every direction has
    # valid poses beyond it, so every complete ray is green.
    assert {s for _, s in plan.radial_rays} == {'valid'}


def test_table_across_part_of_the_donut_turns_only_its_rays_red():
    free = plan_parking(grid(), target(), CONFIG, APPROACH_FROM_WEST)
    g = grid()
    fill(g, 3.40, 2.80, 3.50, 3.20, 100)    # table east of G1, 0.40-0.50 m out
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)
    assert_rays_match_the_planner(plan, g)
    # Every pose east of G1 that keep-out and reach allow would hit the table.
    assert ray(plan, 0) == 'occupied'
    hit = [k for k, (_, s) in enumerate(plan.radial_rays) if s == 'occupied']
    assert hit and all(k <= 90 or k >= 270 for k in hit)
    # Unrelated rays on the far side are unchanged and still pass.
    for k in range(120, 241):
        assert ray(plan, k) == ray(free, k) == 'valid'


def test_unknown_cells_make_their_rays_orange_and_never_green():
    g = grid()
    fill(g, 2.40, 2.95, 2.50, 3.05, -1)     # unexplored patch at the west edge of the donut
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)
    assert_rays_match_the_planner(plan, g)
    assert ray(plan, 180) == 'unknown'
    assert not any(ray(plan, k) == 'unknown' for k in range(90))


def test_mixed_rejections_follow_the_documented_priority():
    """valid > unknown > occupied > pose_invalid over the positions of one ray."""
    g = grid()
    plan = plan_parking(g, target(), CONFIG, None)
    on_ray = ray_positions(plan, g, 90)
    mine = [c for c in plan.candidates if cell_of(g, c.task_x, c.task_y) in on_ray]
    viable = [c for c in mine if c.status == 'valid']
    assert len(viable) >= 2 and len(mine) > len(viable)

    def ray_90(reasons):
        for c, (status, reason) in zip(viable, reasons):
            c.status, c.reason = status, reason
        return radial_profile(g, plan.target, CONFIG, plan.candidates)[90][1]

    occupied, unknown = ('collision', 'footprint_occupied'), ('rejected', 'footprint_unknown')
    assert ray_90([occupied] * len(viable)) == 'occupied'
    assert ray_90([unknown] + [occupied] * (len(viable) - 1)) == 'unknown'
    assert ray_90([('valid', 'footprint_free'), unknown] + [occupied] * (len(viable) - 2)) == 'valid'
    # A blocked footprint where keep-out would fail anyway stays grey.
    assert ray_90([('collision', 'g1_keepout')] * len(viable)) == 'pose_invalid'
    inside = [c for c in mine if c.reason == 'g1_keepout']
    assert inside
    for c in inside:
        c.status, c.reason = 'collision', 'footprint_occupied'
    assert ray_90([('collision', 'g1_keepout')] * len(viable)) == 'pose_invalid'


@pytest.mark.parametrize('g1,map_yaw,approach', OFF_GRID)
def test_rotated_map_origin_rays_follow_the_rotated_cells(g1, map_yaw, approach):
    g = centred_grid(*g1, map_yaw)
    mark(g, [(g1[0] + 0.45 * math.cos(a), g1[1] + 0.45 * math.sin(a)) for a in (0.4, 0.42, 0.44)], 100)
    mark(g, [(g1[0] - 0.55, g1[1] + 0.05), (g1[0] - 0.52, g1[1] + 0.08)], -1)
    plan = plan_parking(g, target(*g1), CONFIG, approach)
    assert_rays_match_the_planner(plan, g)
    classes = {s for _, s in plan.radial_rays}
    assert {'occupied', 'unknown', 'valid'} <= classes


def test_each_ray_is_drawn_once_in_one_colour_and_a_new_plan_replaces_them():
    import rclpy.time
    stamp = rclpy.time.Time(seconds=10).to_msg()
    g = grid()
    fill(g, 3.40, 2.80, 3.50, 3.20, 100)
    fill(g, 2.40, 2.95, 2.50, 3.05, -1)
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)
    markers = plan_markers(plan, CONFIG, stamp).markers
    rays = {m.id: m for m in markers if m.ns == 'parking/radial_1deg'}
    assert sorted(rays) == [0, 1, 2, 3] and all(m.type == Marker.LINE_LIST for m in rays.values())
    drawn = []
    for status, marker_id in RAY_IDS.items():
        points = rays[marker_id].points
        assert len(points) == 2 * sum(s == status for _, s in plan.radial_rays)
        assert points or status == 'pose_invalid'    # every direction here has a pose past keep-out
        for p, q in zip(points[0::2], points[1::2]):
            # One straight segment from the inner to the outer reach limit.
            assert math.hypot(p.x - 3.0, p.y - 3.0) == pytest.approx(0.30)
            assert math.hypot(q.x - 3.0, q.y - 3.0) == pytest.approx(0.60)
            a = round(math.degrees(math.atan2(p.y - 3.0, p.x - 3.0))) % 360
            assert round(math.degrees(math.atan2(q.y - 3.0, q.x - 3.0))) % 360 == a
            assert ray(plan, a) == status
            drawn.append(a)
    assert sorted(drawn) == list(range(360))       # every ray exactly once
    assert rays[1].color.r > 0.9 and rays[1].color.g < 0.1          # red obstacle
    assert rays[2].color.r > 0.9 and 0.4 < rays[2].color.g < 0.7    # orange unknown
    label = next(m for m in markers if m.ns == 'parking/status')
    passing = sum(s == 'valid' for _, s in plan.radial_rays)
    assert f'rays passing {passing}/360' in label.text
    # The selection keeps its markers; the candidates themselves are not drawn as dots.
    assert {'parking/selected_pose', 'parking/selected_footprint'} <= {m.ns for m in markers}
    assert not any(m.ns.endswith('task_points') or m.ns == 'parking/path_candidates' for m in markers)

    # Replanning around a moved G1 clears everything first; no ray stays at the old centre.
    moved = plan_markers(plan_parking(grid(), target(2.0, 4.0), CONFIG, APPROACH_FROM_WEST),
                         CONFIG, stamp).markers
    assert moved[0].action == Marker.DELETEALL
    for m in moved:
        if m.ns == 'parking/radial_1deg':
            assert all(0.30 - 1e-9 <= math.hypot(p.x - 2.0, p.y - 4.0) <= 0.60 + 1e-9
                       for p in m.points)


def test_radial_profile_without_candidates_never_shows_green():
    g = grid()
    rays = radial_profile(g, target(), CONFIG)
    assert len(rays) == 360
    assert {s for _, s in rays} == {'pose_invalid'}


def test_a_ray_along_a_cell_edge_counts_the_cells_on_both_sides():
    # G1 on a cell corner: the 0 deg ray runs exactly on the edge between two cell rows.
    g = grid()
    fill(g, 3.40, 3.00, 3.45, 3.05, 100)    # one cell just north of the edge
    plan = plan_parking(g, target(3.0, 3.0), CONFIG, APPROACH_FROM_WEST)
    assert {(60, 68), (59, 68)} <= ray_positions(plan, g, 0)
    assert ray(plan, 0) == 'occupied'
    assert_rays_match_the_planner(plan, g)


def test_ray_colour_follows_the_parked_footprint_not_only_the_cells():
    """Obstacles and unmapped floor outside the donut still colour the rays whose parked
    footprints they block, and leave green rays elsewhere."""
    g = grid()
    fill(g, 3.00, 2.00, 3.05, 2.05, 100)    # a post south of G1, outside the donut
    fill(g, 3.90, 3.00, 4.40, 3.40, -1)      # unmapped floor north-east, outside the donut
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)
    donut = [c for c in plan.free_cells]
    assert len(plan.occupied_cells) == len(plan.unknown_cells) == 0 and donut
    assert_rays_match_the_planner(plan, g)
    assert {'occupied', 'unknown', 'valid'} <= {s for _, s in plan.radial_rays}


@pytest.mark.parametrize('foot', (False, True))
@pytest.mark.parametrize('g1,map_yaw,approach', OFF_GRID)
def test_reach_only_geometry_check_agrees_with_every_planner_decision(g1, map_yaw, approach, foot):
    from ridgeback_autonomy.mission.parking_planner import pose_geometry_passes
    g = centred_grid(*g1, map_yaw)
    for config in (CONFIG, BUDGET):
        plan = plan_parking(g, target(*g1, foot=foot), config, approach)
        passes = pose_geometry_passes([c.reach_m for c in plan.candidates], config, plan.keepout_radius_m)
        for c, ok in zip(plan.candidates, passes):
            if c.status in ('valid', 'selected'):
                assert ok
            elif c.reason in ('g1_keepout', 'reach_budget'):
                assert not ok
        assert passes.any() and not passes.all()
