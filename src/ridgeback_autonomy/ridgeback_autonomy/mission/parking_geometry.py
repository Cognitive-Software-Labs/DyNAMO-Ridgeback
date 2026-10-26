from __future__ import annotations

import math
from collections.abc import Iterable, Sequence


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def task_point_from_base_pose(
    base_x_m: float,
    base_y_m: float,
    yaw_rad: float,
    task_point_x_base_m: float,
    task_point_y_base_m: float,
) -> tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        base_x_m
        + cos_yaw * task_point_x_base_m
        - sin_yaw * task_point_y_base_m,
        base_y_m
        + sin_yaw * task_point_x_base_m
        + cos_yaw * task_point_y_base_m,
    )


def base_pose_from_task_point(
    task_x_m: float,
    task_y_m: float,
    g1_x_m: float,
    g1_y_m: float,
    task_point_x_base_m: float,
    task_point_y_base_m: float,
) -> tuple[float, float, float]:
    # The base +x axis points from G1 through the task point, so a negative
    # task-point x offset places the rear task point toward the humanoid.
    yaw_rad = math.atan2(task_y_m - g1_y_m, task_x_m - g1_x_m)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    offset_map_x = (
        cos_yaw * task_point_x_base_m
        - sin_yaw * task_point_y_base_m
    )
    offset_map_y = (
        sin_yaw * task_point_x_base_m
        + cos_yaw * task_point_y_base_m
    )
    return task_x_m - offset_map_x, task_y_m - offset_map_y, yaw_rad


def rotated_footprint_vertices(
    footprint_points: Sequence[tuple[float, float]],
    center_x_m: float,
    center_y_m: float,
    yaw_rad: float,
) -> list[tuple[float, float]]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return [
        (
            center_x_m + cos_yaw * local_x_m - sin_yaw * local_y_m,
            center_y_m + sin_yaw * local_x_m + cos_yaw * local_y_m,
        )
        for local_x_m, local_y_m in footprint_points
    ]


def preparking_pose(
    final_x_m: float,
    final_y_m: float,
    final_yaw_rad: float,
    outward_offset_m: float,
) -> tuple[float, float, float]:
    return (
        final_x_m + math.cos(final_yaw_rad) * outward_offset_m,
        final_y_m + math.sin(final_yaw_rad) * outward_offset_m,
        final_yaw_rad,
    )


def planar_path_length(points_xy: Iterable[tuple[float, float]]) -> float:
    points = list(points_xy)
    return sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(points, points[1:])
    )


def is_reach_distance_valid(
    distance_m: float,
    reach_min_m: float,
    reach_max_m: float,
) -> bool:
    return reach_min_m <= distance_m <= reach_max_m
