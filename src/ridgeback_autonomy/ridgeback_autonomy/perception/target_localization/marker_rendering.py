"""Construct target-localization RViz markers without owning ROS or TF state."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from ridgeback_autonomy.perception.target_localization.core.vehicle_frame import (
    remove_vehicle_front_offset,
)
from ridgeback_autonomy.perception.target_localization.visualization_style import (
    DOT_RADIUS_M,
    ESTIMATOR_COLOURS,
    ESTIMATOR_MARKER_ID_BASES,
    MARKER_Z_M,
    RING_LINE_WIDTH_M,
    RING_POINTS,
    RING_RADIUS_M,
)


def ring_points(cx: float, cy: float, z: float, radius: float, count: int) -> list:
    points = []
    for index in range(count + 1):
        angle = 2.0 * math.pi * index / count
        point = Point()
        point.x = cx + radius * math.cos(angle)
        point.y = cy + radius * math.sin(angle)
        point.z = z
        points.append(point)
    return points


def color_message(red: float, green: float, blue: float, alpha: float) -> ColorRGBA:
    colour = ColorRGBA()
    colour.r = red
    colour.g = green
    colour.b = blue
    colour.a = alpha
    return colour


def world_marker_point(
    forward_m: float,
    lateral_m: float,
    origin_x: float,
    origin_y: float,
    yaw_rad: float,
) -> tuple[float, float]:
    """Place a base-frame ``(forward, lateral)`` measurement in the world."""

    world_x = origin_x + math.cos(yaw_rad) * forward_m - math.sin(yaw_rad) * lateral_m
    world_y = origin_y + math.sin(yaw_rad) * forward_m + math.cos(yaw_rad) * lateral_m
    return world_x, world_y


def append_delete_markers(markers: list, estimator: str) -> None:
    """Remove an estimator's ring and dot without waiting for their lifetime."""

    id_base = ESTIMATOR_MARKER_ID_BASES[estimator]
    for offset in (0, 1):
        marker = Marker()
        marker.ns = f'target_estimates/{estimator}'
        marker.id = id_base + offset
        marker.action = Marker.DELETE
        markers.append(marker)


def append_estimator_markers(
    markers: list,
    estimator: str,
    forward_m: float | None,
    lateral_m: float | None,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    stamp,
    frame_id: str,
    marker_lifetime_s: float,
) -> bool:
    """Append one estimator's ring and dot; return whether it placed anything."""

    if forward_m is None or lateral_m is None:
        return False

    lateral_m, forward_m = remove_vehicle_front_offset(lateral_m, forward_m)
    world_x, world_y = world_marker_point(
        forward_m, lateral_m, robot_x, robot_y, robot_yaw)

    colour = ESTIMATOR_COLOURS[estimator]
    id_base = ESTIMATOR_MARKER_ID_BASES[estimator]
    lifetime = rclpy.duration.Duration(seconds=marker_lifetime_s).to_msg()

    ring = Marker()
    ring.header.frame_id = frame_id
    ring.header.stamp = stamp
    ring.ns = f'target_estimates/{estimator}'
    ring.id = id_base
    ring.type = Marker.LINE_STRIP
    ring.action = Marker.ADD
    ring.scale.x = RING_LINE_WIDTH_M
    ring.color = color_message(*colour)
    ring.lifetime = lifetime
    ring.points = ring_points(
        world_x, world_y, MARKER_Z_M, RING_RADIUS_M, RING_POINTS)
    markers.append(ring)

    dot = Marker()
    dot.header.frame_id = frame_id
    dot.header.stamp = stamp
    dot.ns = f'target_estimates/{estimator}'
    dot.id = id_base + 1
    dot.type = Marker.SPHERE
    dot.action = Marker.ADD
    dot.pose.position.x = world_x
    dot.pose.position.y = world_y
    dot.pose.position.z = MARKER_Z_M
    dot.pose.orientation.w = 1.0
    dot.scale.x = DOT_RADIUS_M * 2
    dot.scale.y = DOT_RADIUS_M * 2
    dot.scale.z = DOT_RADIUS_M * 2
    dot.color = color_message(*colour)
    dot.lifetime = lifetime
    markers.append(dot)
    return True
