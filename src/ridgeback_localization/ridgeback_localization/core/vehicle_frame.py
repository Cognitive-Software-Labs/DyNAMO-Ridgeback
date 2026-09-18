"""Planar frame math shared by every path that reports against the robot.

No estimator lives here: this is the conversion between a world-frame offset and
the ``(forward, lateral, distance)`` measurement each estimator publishes, plus
the front-offset round trip that ties the two together.
"""

from __future__ import annotations

import math

import numpy as np


ROBOT_FRONT_OFFSET_M = 0.25


def optical_to_base_planar(
    xyz_optical,
    rotation: np.ndarray,
    translation: np.ndarray,
    front_offset_m: float,
) -> tuple[float, float, float]:
    """Camera-optical point -> planar position and distance in the base frame."""

    point_base = (
        np.asarray(rotation, dtype=np.float64)
        @ np.asarray(xyz_optical, dtype=np.float64)
        + np.asarray(translation, dtype=np.float64)
    )
    lateral_m = float(point_base[1])
    forward_m = float(point_base[0]) - front_offset_m
    return lateral_m, forward_m, math.hypot(lateral_m, forward_m)


def apply_vehicle_front_offset(lateral_m, forward_m):
    return lateral_m, forward_m - ROBOT_FRONT_OFFSET_M


def remove_vehicle_front_offset(lateral_m, forward_m):
    """Inverse of ``apply_vehicle_front_offset``: front-referenced -> base origin.

    Needed by anything that has to put a published measurement back on a map,
    since every estimator reports against the robot front rather than the base
    origin TF gives. It lives beside the forward conversion so the two cannot
    drift: a one-sided change to the offset would otherwise leave the round trip
    silently lossy, and the error is a constant shift that looks like a
    calibration problem.
    """

    return lateral_m, forward_m + ROBOT_FRONT_OFFSET_M


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_vehicle_planar(dx_world: float, dy_world: float, yaw_rad: float):
    forward_m = math.cos(yaw_rad) * dx_world + math.sin(yaw_rad) * dy_world
    lateral_m = -math.sin(yaw_rad) * dx_world + math.cos(yaw_rad) * dy_world
    return forward_m, lateral_m


def planar_measurement_from_vehicle_front(
    dx_world: float,
    dy_world: float,
    yaw_rad: float,
) -> tuple[float, float, float]:
    """World-frame offset -> ``(forward_m, lateral_m, distance_m)`` off the robot front.

    All three components share one reference point -- the base origin plus
    ``ROBOT_FRONT_OFFSET_M`` -- which is what every estimator publishes against,
    so ground truth and estimates are directly comparable component by component
    (the benchmark association matches them in the planar plane, not just on
    distance). Lateral is offset-invariant: the front offset is purely forward.
    """

    forward_m, lateral_m = world_to_vehicle_planar(dx_world, dy_world, yaw_rad)
    lateral_m, forward_m = apply_vehicle_front_offset(lateral_m, forward_m)
    return forward_m, lateral_m, math.hypot(lateral_m, forward_m)
