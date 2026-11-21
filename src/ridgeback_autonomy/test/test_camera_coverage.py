"""Camera coverage for the G1 search: what a usable frame sees (view, range band, line of
sight), which unseen directions a stop turns to, in what order, and whether the swept
footprint (basket included) clears the costmap."""
import math

import numpy as np
import pytest

from ridgeback_autonomy.frontier_explorer.camera_coverage import (
    CameraCoverage, MapView, ViewModel, rotation_problem, turning_order, visible_cells)
from ridgeback_autonomy.mission.parking_geometry import (
    load_nav2_footprint, normalize_angle, parking_footprint)

RES = 0.05
MODEL = ViewModel()


def free_map(size_m=10.0, origin=(0.0, 0.0), value=0):
    n = int(round(size_m / RES))
    return MapView(np.full((n, n), value, dtype=np.int16), RES, *origin)


def cell(grid, x, y):
    r, c = grid.cell_of(x, y)
    return int(r) * grid.shape[1] + int(c)


def seen_from(grid, x, y, yaw):
    return set(visible_cells(grid, MODEL, x, y, yaw).tolist())


def test_only_cells_inside_the_view_and_the_range_band_are_seen():
    g = free_map()
    seen = seen_from(g, 5.0, 5.0, 0.0)
    assert cell(g, 7.0, 5.0) in seen                                  # ahead, 2 m
    assert cell(g, 5.0 + 2 * math.cos(0.4), 5.0 + 2 * math.sin(0.4)) in seen   # 23 deg off-axis
    assert cell(g, 5.0 + 2 * math.cos(0.7), 5.0 + 2 * math.sin(0.7)) not in seen  # 40 deg: image edge
    assert cell(g, 6.0, 5.0) not in seen                              # 1.0 m: G1's feet leave the image
    assert cell(g, 9.0, 5.0) not in seen                              # 4.0 m: beyond the useful range
    assert cell(g, 3.0, 5.0) not in seen                              # behind


def test_walls_and_unknown_space_block_the_view_but_an_obstacle_face_is_seen():
    g = free_map()
    r, c = g.cell_of(6.5, 5.0)
    g.data[int(r) - 3:int(r) + 4, int(c)] = 100                         # a short wall 1.5 m ahead
    seen = seen_from(g, 5.0, 5.0, 0.0)
    assert cell(g, 6.5, 5.0) in seen and cell(g, 7.5, 5.0) not in seen
    g = free_map()
    r, c = g.cell_of(6.5, 5.0)
    g.data[int(r) - 3:int(r) + 4, int(c)] = -1
    seen = seen_from(g, 5.0, 5.0, 0.0)
    assert cell(g, 6.5, 5.0) not in seen and cell(g, 7.5, 5.0) not in seen


def test_select_views_turns_only_toward_unseen_directions_and_never_more_than_three():
    g = free_map()
    cov = CameraCoverage()
    assert len(cov.select_views(g, (5.0, 5.0, 0.0), (0.27, 0.0), max_views=3)) == 3
    for k in range(36):                                              # frames all around
        yaw = k * math.radians(10)
        cov.mark(g, 5.0 + 0.27 * math.cos(yaw), 5.0 + 0.27 * math.sin(yaw), yaw)
    assert cov.select_views(g, (5.0, 5.0, 0.0), (0.27, 0.0), min_new_cells=100) == []


def test_select_views_finds_the_one_direction_left_unseen():
    g = free_map()
    cov = CameraCoverage()
    for k in range(36):
        yaw = k * math.radians(10)
        if abs(normalize_angle(yaw - math.pi / 2)) > math.radians(50):
            cov.mark(g, 5.0 + 0.27 * math.cos(yaw), 5.0 + 0.27 * math.sin(yaw), yaw)
    views = cov.select_views(g, (5.0, 5.0, 0.0), (0.27, 0.0), min_new_cells=400)
    assert len(views) == 1 and abs(normalize_angle(views[0][0] - math.pi / 2)) <= math.radians(30)
    assert views[0][1] == pytest.approx(views[0][0])                  # one signed turn from 0
    excluded = cov.select_views(g, (5.0, 5.0, 0.0), (0.27, 0.0), min_new_cells=400,
                                excluded=[views[0][0]])
    assert all(abs(normalize_angle(h - views[0][0])) >= MODEL.half_fov_rad for h, _, _ in excluded)


def test_coverage_follows_a_growing_map_by_world_position():
    g = free_map(size_m=6.0)
    cov = CameraCoverage()
    cov.mark(g, 3.0, 3.0, 0.0)
    before = {(round(g.origin_x + (c + .5) * RES, 3), round(g.origin_y + (r + .5) * RES, 3))
              for r, c in zip(*np.nonzero(cov.seen))}
    grown = MapView(np.zeros((160, 140), dtype=np.int16), RES, -1.0, -0.5)   # map grew left and down
    # Still seen after realigning (a ray sample on a cell edge may round into its neighbour).
    assert cov.unseen(grown, 3.0, 3.0, 0.0).size <= 2
    after = {(round(grown.origin_x + (c + .5) * RES, 3), round(grown.origin_y + (r + .5) * RES, 3))
             for r, c in zip(*np.nonzero(cov.seen))}
    assert after == before


@pytest.mark.parametrize('yaw,headings', [
    (math.radians(168), [48, -132, -12]), (0.0, [30, -30]), (0.0, [170, -170]), (0.0, [-60, -120]),
    (1.0, [100, 10, -80])])
def test_turns_are_signed_one_sweep_or_one_crossing_with_the_least_rotation(yaw, headings):
    views = [(math.radians(h), 1) for h in headings]
    plan = turning_order(yaw, views)
    assert sorted(round(math.degrees(h)) for h, _, _ in plan) == sorted(headings)
    # Each signed turn really ends at its heading.
    heading = yaw
    for h, turn, _ in plan:
        heading += turn
        assert normalize_angle(heading - h) == pytest.approx(0.0, abs=1e-9)
    total = sum(abs(t) for _, t, _ in plan)
    offsets = [normalize_angle(h - yaw) for h, _ in views]
    one_side = max(abs(o) for o in offsets) if all(o >= 0 for o in offsets) or all(o <= 0 for o in offsets) else None
    assert total <= 2 * math.pi
    if one_side is not None:
        assert total == pytest.approx(one_side)                       # never past a heading twice


@pytest.fixture(scope='module')
def footprint():
    from ament_index_python.packages import get_package_share_directory
    import os
    return parking_footprint(load_nav2_footprint(os.path.join(
        get_package_share_directory('ridgeback_autonomy'), 'config', 'nav2_params.yaml')))


def test_a_turn_is_refused_when_the_swept_footprint_meets_an_obstacle(footprint):
    radius = max(math.hypot(x, y) for x, y in footprint)
    half_width = max(abs(y) for _, y in footprint)
    costmap = free_map()
    assert rotation_problem(costmap, footprint, (5.0, 5.0, 0.0), math.pi / 2) is None
    # Beside the robot, clear of the footprint now, but swept by its corners during the turn.
    gap_now = radius - 0.04
    assert gap_now > half_width + 0.05                                # outside the margin now
    r, c = costmap.cell_of(5.0, 5.0 + gap_now)
    costmap.data[int(r), int(c)] = 100
    assert 'obstacle' in rotation_problem(costmap, footprint, (5.0, 5.0, 0.0), math.pi / 2)
    assert 'obstacle' in rotation_problem(costmap, footprint, (5.0, 5.0, 0.0), -math.pi / 2)
    # A short turn that keeps the margin is allowed.
    assert rotation_problem(costmap, footprint, (5.0, 5.0, 0.0), math.radians(-5)) is None


def test_unknown_space_in_the_sweep_refuses_but_unknown_under_the_robot_does_not(footprint):
    costmap = free_map()
    r, c = costmap.cell_of(5.0, 5.0)
    costmap.data[int(r), int(c)] = -1                                  # under the robot itself
    assert rotation_problem(costmap, footprint, (5.0, 5.0, 0.0), math.pi) is None
    radius = max(math.hypot(x, y) for x, y in footprint)
    r, c = costmap.cell_of(5.0, 5.0 + radius - 0.05)
    costmap.data[int(r), int(c)] = -1
    assert 'unknown' in rotation_problem(costmap, footprint, (5.0, 5.0, 0.0), math.pi)
    edge = free_map(size_m=1.0)
    assert 'leaves the costmap' in rotation_problem(edge, footprint, (0.5, 0.5, 0.0), 1.0)


def test_the_camera_frontier_map_keeps_unknown_and_occupied_space_out():
    from ridgeback_autonomy.frontier_explorer.camera_coverage import camera_frontier_map
    from ridgeback_autonomy.frontier_explorer.params import FREE, OBSTACLE, UNKNOWN
    grid = free_map(size_m=4.0)
    grid.data[:, 60:] = -1                                        # SLAM map unknown at x >= 3 m
    grid.data[11, 11] = 100                                       # under costmap cell (5, 5)
    seen = np.zeros(grid.shape, bool)
    seen[:, :20] = True                                           # camera saw x < 1 m
    costmap = MapView(np.zeros((40, 40), np.int16), 0.1, 0.0, 0.0)   # coarser, same origin
    aware = np.full((40, 40), FREE, np.int8)
    aware[30, 5] = OBSTACLE                                       # lethal in the costmap
    out = camera_frontier_map(aware, costmap, grid, seen, FREE, UNKNOWN, OBSTACLE)
    assert out[5, 5] == OBSTACLE                                  # occupied in the SLAM map
    assert out[0, 0] == FREE and out[20, 8] == FREE               # seen free
    assert (out[:, 12:28] == UNKNOWN).all()                       # free but never seen
    assert (out[:, 30:] == OBSTACLE).all()                        # unknown to the SLAM map
    assert out[30, 5] == OBSTACLE                                 # lethal in the costmap
