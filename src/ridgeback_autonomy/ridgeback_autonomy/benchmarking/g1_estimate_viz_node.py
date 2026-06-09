#!/usr/bin/env python3

from __future__ import annotations

import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from ridgeback_autonomy.msg import G1Measurements


# Seconds before a marker auto-expires if no new detection arrives.
# Set to 1.5x the detector's max publish interval (detector runs at ≤5 Hz → 0.2s per frame).
MARKER_LIFETIME_SEC = 1.5

# Ring radius and center dot size in metres.
RING_RADIUS_M = 0.15
RING_POINTS = 32
DOT_RADIUS_M = 0.04

# Z height above ground plane so markers sit on top of the costmap.
MARKER_Z_M = 0.05

# (r, g, b, a) colours per estimator.
ESTIMATOR_COLOURS = {
    'rgb':          (1.0, 0.0, 0.0, 1.0),
    'sensor_depth': (1.0, 0.5, 0.0, 1.0),
    'depth_anything': (0.6, 0.0, 1.0, 1.0),
    'pointcloud':   (0.0, 0.4, 1.0, 1.0),
    'lidar':        (0.0, 0.9, 0.0, 1.0),
}

# Stable marker-id base per estimator so DELETEALL is not needed — we just overwrite.
_ESTIMATOR_ID_BASE = {name: i * 100 for i, name in enumerate(ESTIMATOR_COLOURS)}


def _ring_points(cx: float, cy: float, z: float, r: float, n: int) -> list:
    pts = []
    for i in range(n + 1):
        angle = 2.0 * math.pi * i / n
        from geometry_msgs.msg import Point
        p = Point()
        p.x = cx + r * math.cos(angle)
        p.y = cy + r * math.sin(angle)
        p.z = z
        pts.append(p)
    return pts


def _color_msg(r: float, g: float, b: float, a: float):
    from std_msgs.msg import ColorRGBA
    c = ColorRGBA()
    c.r = r
    c.g = g
    c.b = b
    c.a = a
    return c


class G1EstimateVizNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_estimate_viz_node')

        namespace_name = self.get_namespace().strip('/')
        default_base_frame = (
            f'{namespace_name}/robot/base_link' if namespace_name else 'robot/base_link'
        )

        self.declare_parameter('base_frame', default_base_frame)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('marker_lifetime_sec', MARKER_LIFETIME_SEC)

        self.base_frame = self.get_parameter('base_frame').value or default_base_frame
        self.world_frame = self.get_parameter('world_frame').value
        self.marker_lifetime = self.get_parameter('marker_lifetime_sec').value

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self._lock = threading.Lock()
        self._latest_camera: G1Measurements | None = None
        self._latest_lidar: G1Measurements | None = None

        self.create_subscription(G1Measurements, 'measurements/g1/camera',
                                 self._camera_cb, 10)
        self.create_subscription(G1Measurements, 'measurements/g1/lidar',
                                 self._lidar_cb, 10)

        self._pub = self.create_publisher(MarkerArray, 'visualization/g1/estimates', 10)

    def _camera_cb(self, msg: G1Measurements) -> None:
        with self._lock:
            self._latest_camera = msg
        self._publish()

    def _lidar_cb(self, msg: G1Measurements) -> None:
        with self._lock:
            self._latest_lidar = msg
        self._publish()

    def _publish(self) -> None:
        with self._lock:
            camera_msg = self._latest_camera
            lidar_msg = self._latest_lidar

        if camera_msg is None and lidar_msg is None:
            return

        # Drop stale cached messages so rings disappear when the G1 is gone.
        now = self.get_clock().now()
        stale_threshold = rclpy.duration.Duration(seconds=self.marker_lifetime)
        if camera_msg is not None:
            age = now - rclpy.time.Time.from_msg(camera_msg.header.stamp)
            if age > stale_threshold:
                camera_msg = None
        if lidar_msg is not None:
            age = now - rclpy.time.Time.from_msg(lidar_msg.header.stamp)
            if age > stale_threshold:
                lidar_msg = None
        if camera_msg is None and lidar_msg is None:
            return

        # Use Time(0) to get the latest available TF — avoids sim-time buffer
        # mismatches since the robot is stationary during benchmarks.
        robot_x, robot_y, robot_yaw, actual_frame = self._robot_pose_in_world(
            rclpy.time.Time()
        )
        if robot_x is None:
            return

        markers = []
        marker_time = now.to_msg()

        if camera_msg is not None and camera_msg.detected:
            for i in range(camera_msg.count):
                self._add_estimator_markers(
                    markers, 'rgb', i,
                    _get(camera_msg.rgb_forward_m, i),
                    _get(camera_msg.rgb_lateral_m, i),
                    robot_x, robot_y, robot_yaw, marker_time, actual_frame,
                )
                self._add_estimator_markers(
                    markers, 'sensor_depth', i,
                    None, None,
                    robot_x, robot_y, robot_yaw, marker_time, actual_frame,
                    distance_m=_get(camera_msg.sensor_depth_distance_m, i),
                )
                self._add_estimator_markers(
                    markers, 'depth_anything', i,
                    None, None,
                    robot_x, robot_y, robot_yaw, marker_time, actual_frame,
                    distance_m=_get(camera_msg.mono_depth_distance_m, i),
                )
                self._add_estimator_markers(
                    markers, 'pointcloud', i,
                    _get(camera_msg.pointcloud_forward_m, i),
                    _get(camera_msg.pointcloud_lateral_m, i),
                    robot_x, robot_y, robot_yaw, marker_time, actual_frame,
                )

        if lidar_msg is not None and lidar_msg.detected:
            for i in range(lidar_msg.count):
                self._add_estimator_markers(
                    markers, 'lidar', i,
                    _get(lidar_msg.lidar_forward_m, i),
                    _get(lidar_msg.lidar_lateral_m, i),
                    robot_x, robot_y, robot_yaw, marker_time, actual_frame,
                )

        if not markers:
            return

        ma = MarkerArray()
        ma.markers = markers
        self._pub.publish(ma)

    def _add_estimator_markers(
        self,
        markers: list,
        estimator: str,
        detection_index: int,
        forward_m: float | None,
        lateral_m: float | None,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        stamp,
        frame_id: str,
        distance_m: float | None = None,
    ) -> None:
        # For depth-only estimators (sensor_depth, depth_anything) we only have a scalar
        # distance along the camera boresight — treat lateral as 0.
        if forward_m is None and distance_m is not None:
            forward_m = distance_m
            lateral_m = 0.0
        if forward_m is None or lateral_m is None:
            return

        # Rotate base-frame (forward, lateral) into world frame.
        wx = robot_x + math.cos(robot_yaw) * forward_m - math.sin(robot_yaw) * lateral_m
        wy = robot_y + math.sin(robot_yaw) * forward_m + math.cos(robot_yaw) * lateral_m

        colour = ESTIMATOR_COLOURS[estimator]
        id_base = _ESTIMATOR_ID_BASE[estimator] + detection_index * 2
        lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()

        # Ring marker (LINE_STRIP circle).
        ring = Marker()
        ring.header.frame_id = frame_id
        ring.header.stamp = stamp
        ring.ns = f'g1_estimates/{estimator}'
        ring.id = id_base
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.scale.x = 0.06   # line width
        ring.color = _color_msg(*colour)
        ring.lifetime = lifetime
        ring.points = _ring_points(wx, wy, MARKER_Z_M, RING_RADIUS_M, RING_POINTS)
        markers.append(ring)

        # Centre dot (SPHERE).
        dot = Marker()
        dot.header.frame_id = frame_id
        dot.header.stamp = stamp
        dot.ns = f'g1_estimates/{estimator}'
        dot.id = id_base + 1
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD
        dot.pose.position.x = wx
        dot.pose.position.y = wy
        dot.pose.position.z = MARKER_Z_M
        dot.pose.orientation.w = 1.0
        dot.scale.x = DOT_RADIUS_M * 2
        dot.scale.y = DOT_RADIUS_M * 2
        dot.scale.z = DOT_RADIUS_M * 2
        dot.color = _color_msg(*colour)
        dot.lifetime = lifetime
        markers.append(dot)

    def _robot_pose_in_world(self, stamp: rclpy.time.Time) -> tuple[float | None, float | None, float | None, str | None]:
        base_frames = [self.base_frame]
        if self.base_frame != 'base_link':
            base_frames.append('base_link')
        for world_frame in (self.world_frame, 'odom'):
            for base_frame in base_frames:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        world_frame, base_frame, stamp,
                        timeout=rclpy.duration.Duration(seconds=0.2),
                    )
                    tx = tf.transform.translation.x
                    ty = tf.transform.translation.y
                    q = tf.transform.rotation
                    yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                    )
                    if world_frame != self.world_frame:
                        self.get_logger().warn(
                            f'"{self.world_frame}" frame unavailable; '
                            f'publishing estimate markers in "{world_frame}" frame.',
                            throttle_duration_sec=10.0,
                        )
                    return tx, ty, yaw, world_frame
                except TransformException:
                    continue
        return None, None, None, None


def _get(arr, i: int) -> float | None:
    if arr and i < len(arr) and arr[i] == arr[i]:  # NaN check
        return float(arr[i])
    return None


def main() -> None:
    rclpy.init()
    node = G1EstimateVizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
