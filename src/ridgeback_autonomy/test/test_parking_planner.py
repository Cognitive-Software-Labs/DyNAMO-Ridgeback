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
               'parking/radial_1deg', 'parking/donut_cells', 'parking/candidate_task_points',
               'parking/selected_pose', 'parking/selected_footprint', 'parking/status'):
        assert ns in namespaces
    assert all(m.header.frame_id == 'map' for m in markers[1:])
    rays = next(m for m in markers if m.ns == 'parking/radial_1deg')
    assert len(rays.points) == 720 and len(rays.colors) == 720
    free_cells = next(m for m in markers if m.ns == 'parking/donut_cells' and m.id == 0)
    assert free_cells.color.g > 0.8 and free_cells.color.r < 0.2   # green = safe

    failed = plan_parking(grid(value=100), target(), CONFIG, APPROACH_FROM_WEST)
    failed_markers = plan_markers(failed, CONFIG, stamp).markers
    assert failed_markers[0].action == Marker.DELETEALL
    assert not any(m.ns.startswith('parking/selected') for m in failed_markers)
    occupied = next(m for m in failed_markers if m.ns == 'parking/donut_cells' and m.id == 1)
    assert occupied.points and occupied.color.r > 0.9 and occupied.color.g < 0.1   # red = unsafe
    cleared = clear_markers().markers
    assert len(cleared) == 1 and cleared[0].action == Marker.DELETEALL
