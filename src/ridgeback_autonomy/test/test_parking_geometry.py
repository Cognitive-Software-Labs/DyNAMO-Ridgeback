from __future__ import annotations

import math

import pytest

from ridgeback_autonomy.mission.parking_geometry import (
    base_pose_from_task_point,
    is_reach_distance_valid,
    normalize_angle,
    planar_path_length,
    preparking_pose,
    rotated_footprint_vertices,
    task_point_from_base_pose,
)


FOOTPRINT = [
    (0.480, 0.3965),
    (0.480, -0.3965),
    (-0.480, -0.3965),
    (-0.480, 0.3965),
]


@pytest.mark.parametrize(
    ('task_x', 'task_y'),
    [(0.45, 0.0), (-0.45, 0.0), (0.0, 0.45), (0.0, -0.45)],
)
def test_task_point_base_conversion_and_rear_facing_yaw(task_x, task_y) -> None:
    base_x, base_y, yaw = base_pose_from_task_point(
        task_x,
        task_y,
        0.0,
        0.0,
        -0.280,
        0.0,
    )
    recovered_x, recovered_y = task_point_from_base_pose(
        base_x,
        base_y,
        yaw,
        -0.280,
        0.0,
    )

    assert recovered_x == pytest.approx(task_x)
    assert recovered_y == pytest.approx(task_y)
    front_x = math.cos(yaw)
    front_y = math.sin(yaw)
    toward_g1_x = -task_x
    toward_g1_y = -task_y
    assert front_x * toward_g1_x + front_y * toward_g1_y < 0.0


def test_rotated_footprint_and_preparking_geometry() -> None:
    vertices = rotated_footprint_vertices(FOOTPRINT, 1.0, 2.0, math.pi / 2.0)
    assert vertices[0] == pytest.approx((1.0 - 0.3965, 2.0 + 0.480))
    assert preparking_pose(1.0, 2.0, math.pi / 2.0, 0.50) == pytest.approx(
        (1.0, 2.5, math.pi / 2.0)
    )


def test_reach_boundaries_and_path_length() -> None:
    assert is_reach_distance_valid(0.30, 0.30, 0.60)
    assert is_reach_distance_valid(0.60, 0.30, 0.60)
    assert not is_reach_distance_valid(0.299, 0.30, 0.60)
    assert not is_reach_distance_valid(0.601, 0.30, 0.60)
    assert planar_path_length([(0.0, 0.0), (3.0, 4.0), (3.0, 5.0)]) == 6.0


@pytest.mark.parametrize(
    ('angle_rad', 'expected_rad'),
    [
        (0.0, 0.0),
        (2.0 * math.pi, 0.0),
        (-2.0 * math.pi, 0.0),
        (3.0 * math.pi / 2.0, -math.pi / 2.0),
        (-3.0 * math.pi / 2.0, math.pi / 2.0),
    ],
)
def test_normalize_angle(angle_rad, expected_rad) -> None:
    assert normalize_angle(angle_rad) == pytest.approx(expected_rad, abs=1e-12)


def test_the_parking_footprint_covers_the_basket_the_robot_description_builds():
    """Nav2's shared outline is the robot body; the provisional basket's rear corners
    reach beyond its angled rear ends, so planning uses the hull of both."""
    import ast
    import re
    from pathlib import Path

    import yaml

    from ridgeback_autonomy.mission.parking_geometry import BASKET_CORNERS_BASE, parking_footprint
    from ridgeback_autonomy.mission.parking_planner import points_to_polygon_distance

    root = Path(__file__).resolve().parents[2]
    xacro = (root / 'ridgeback_common' / 'urdf' / 'camera_support.urdf.xacro').read_text()
    basket_x = float(re.search(r'name="basket_x" value="([-\d.]+)"', xacro).group(1))
    corners = set()
    for box in re.finditer(r'<xacro:ridgeback_support_box name="basket_\w+"\s+x="([^"]+)" y="([^"]+)"'
                           r' z="[^"]+" sx="([\d.]+)" sy="([\d.]+)"', xacro):
        x = eval(box.group(1).replace('${', '').replace('}', ''), {'basket_x': basket_x})
        y, sx, sy = float(box.group(2)), float(box.group(3)), float(box.group(4))
        corners |= {(round(x + dx * sx / 2, 3), round(y + dy * sy / 2, 3))
                    for dx in (-1, 1) for dy in (-1, 1)}
    assert len(corners) == 20
    hull = [(round(x, 3), round(y, 3)) for x, y in __import__(
        'ridgeback_autonomy.mission.parking_geometry', fromlist=['convex_hull']).convex_hull(corners)]
    assert set(hull) == {(round(x, 3), round(y, 3)) for x, y in BASKET_CORNERS_BASE}
    nav2 = yaml.safe_load((root / 'ridgeback_autonomy' / 'config' / 'nav2_params.yaml').read_text())
    outline = [tuple(p) for p in ast.literal_eval(str(
        nav2['global_costmap']['global_costmap']['ros__parameters']['footprint']))]
    footprint = parking_footprint(outline)
    xs, ys = zip(*corners)
    # Every basket corner and every Nav2 vertex lies inside or on the planning outline.
    inside = points_to_polygon_distance(list(xs) + [p[0] for p in outline],
                                        list(ys) + [p[1] for p in outline], footprint)
    assert max(inside) <= 1e-9
