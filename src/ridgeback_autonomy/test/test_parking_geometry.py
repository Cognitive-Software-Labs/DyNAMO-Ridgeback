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
