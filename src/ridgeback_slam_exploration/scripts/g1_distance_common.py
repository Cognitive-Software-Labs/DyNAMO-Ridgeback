import math


ROBOT_FRONT_OFFSET_M = 0.25


def apply_vehicle_front_offset(x_m, z_m):
    return x_m, z_m - ROBOT_FRONT_OFFSET_M


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_vehicle_planar(dx_world: float, dy_world: float, yaw_rad: float):
    forward_m = math.cos(yaw_rad) * dx_world + math.sin(yaw_rad) * dy_world
    lateral_m = -math.sin(yaw_rad) * dx_world + math.cos(yaw_rad) * dy_world
    return forward_m, lateral_m


def planar_distance_from_vehicle_origin(
    dx_world: float,
    dy_world: float,
    yaw_rad: float,
) -> tuple[float, float, float]:
    forward_m, lateral_m = world_to_vehicle_planar(dx_world, dy_world, yaw_rad)
    _, forward_with_offset_m = apply_vehicle_front_offset(0.0, forward_m)
    distance_m = math.hypot(lateral_m, forward_with_offset_m)
    return forward_m, lateral_m, distance_m
