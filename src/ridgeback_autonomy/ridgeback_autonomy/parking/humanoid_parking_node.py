#!/usr/bin/env python3

from __future__ import annotations

import math
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from explore_lite_msgs.msg import ExploreStatus
from ridgeback_autonomy.common.tf_utils import candidate_base_frames
from ridgeback_autonomy.msg import G1Measurements


class ParkingState(Enum):
    EXPLORING = 'exploring'
    STOPPING_EXPLORATION = 'stopping_exploration'
    NAVIGATING_TO_HUMANOID = 'navigating_to_humanoid'
    PARKED_WAITING = 'parked_waiting'
    RETURNING_HOME = 'returning_home'
    DONE = 'done'


def _yaw_to_quaternion(yaw_rad: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _finite(value: float | None) -> float | None:
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _get(values, index: int) -> float | None:
    if values and index < len(values):
        return _finite(values[index])
    return None


class HumanoidParkingNode(Node):
    """Stop exploration, park near a detected humanoid, then return home."""

    def __init__(self) -> None:
        super().__init__('humanoid_parking_node')

        self.declare_parameter('enabled', True)
        self.declare_parameter('world', '')
        self.declare_parameter('enabled_world', 'park_robot')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', 'platform/odom/filtered')
        self.declare_parameter('explore_status_topic', 'explore/status')
        self.declare_parameter('explore_resume_topic', 'explore/resume')
        self.declare_parameter('camera_measurement_topic', 'measurements/g1/camera')
        self.declare_parameter('lidar_measurement_topic', 'measurements/g1/lidar')
        self.declare_parameter('humanoid_pose_topic', 'humanoid/pose')
        self.declare_parameter('marker_topic', 'parking/markers')
        self.declare_parameter('parking_distance_m', 0.95)
        self.declare_parameter('humanoid_yaw_rad', 1.5708)
        self.declare_parameter('hand_side', 'right')
        self.declare_parameter('hand_forward_offset_m', 0.15)
        self.declare_parameter('use_static_humanoid_pose', True)
        self.declare_parameter('static_humanoid_x', 7.2)
        self.declare_parameter('static_humanoid_y', -4.2)
        self.declare_parameter('park_wait_sec', 5.0)
        self.declare_parameter('exploration_stop_settle_sec', 1.0)
        self.declare_parameter('return_home_distance_tolerance_m', 0.75)

        enabled = bool(self.get_parameter('enabled').value)
        self.world = str(self.get_parameter('world').value)
        enabled_world = str(self.get_parameter('enabled_world').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.parking_distance_m = float(self.get_parameter('parking_distance_m').value)
        self.humanoid_yaw_rad = float(self.get_parameter('humanoid_yaw_rad').value)
        self.hand_side = str(self.get_parameter('hand_side').value).lower()
        if self.hand_side not in ('right', 'left'):
            self.get_logger().warn(
                f'Unknown hand_side "{self.hand_side}"; using "right".'
            )
            self.hand_side = 'right'
        self.hand_forward_offset_m = float(self.get_parameter('hand_forward_offset_m').value)
        self.use_static_humanoid_pose = bool(
            self.get_parameter('use_static_humanoid_pose').value
        )
        self.static_humanoid_x = float(self.get_parameter('static_humanoid_x').value)
        self.static_humanoid_y = float(self.get_parameter('static_humanoid_y').value)
        self.park_wait_sec = float(self.get_parameter('park_wait_sec').value)
        self.exploration_stop_settle_sec = float(
            self.get_parameter('exploration_stop_settle_sec').value
        )
        self.return_home_tolerance_m = float(
            self.get_parameter('return_home_distance_tolerance_m').value
        )

        if not enabled or (self.world and self.world != enabled_world):
            self.get_logger().info(
                f'Humanoid parking disabled for world "{self.world}".'
            )
            return

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.state = ParkingState.EXPLORING
        self.start_pose_map: PoseStamped | None = None
        self.latest_odom_start: PoseStamped | None = None
        self.latest_humanoid_pose: PoseStamped | None = None
        self.parking_pose: PoseStamped | None = None
        self.exploration_returned_home = False
        self.exploration_complete_seen = False
        self.return_to_origin_seen = False
        self.exploration_stop_sent = False
        self.exploration_stopped_at = None
        self.last_base_frame_fallback = None
        self.wait_until = None
        self.retry_timer = None

        self.explore_resume_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('explore_resume_topic').value),
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            10,
        )

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            qos,
        )
        self.create_subscription(
            ExploreStatus,
            str(self.get_parameter('explore_status_topic').value),
            self._on_explore_status,
            qos,
        )
        self.create_subscription(
            G1Measurements,
            str(self.get_parameter('camera_measurement_topic').value),
            self._on_measurement,
            qos,
        )
        self.create_subscription(
            G1Measurements,
            str(self.get_parameter('lidar_measurement_topic').value),
            self._on_measurement,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('humanoid_pose_topic').value),
            self._on_humanoid_pose,
            qos,
        )

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            'Humanoid parking armed: waiting for start pose, humanoid detection, '
            'and Nav2.'
        )

    def _on_odom(self, msg: Odometry) -> None:
        if self.latest_odom_start is None:
            self.latest_odom_start = PoseStamped()
            self.latest_odom_start.header = msg.header
            self.latest_odom_start.pose = msg.pose.pose

    def _on_explore_status(self, msg: ExploreStatus) -> None:
        self.get_logger().info(
            f'Explore status: {msg.status}',
            throttle_duration_sec=2.0,
        )
        if msg.status == ExploreStatus.RETURNED_TO_ORIGIN:
            self.exploration_returned_home = True
            self._maybe_start_parking()
        elif msg.status == ExploreStatus.RETURNING_TO_ORIGIN:
            self.return_to_origin_seen = True
        elif msg.status == ExploreStatus.EXPLORATION_COMPLETE:
            self.exploration_complete_seen = True

    def _on_measurement(self, msg: G1Measurements) -> None:
        if not msg.detected or msg.count <= 0:
            return

        if self.use_static_humanoid_pose:
            self._save_static_humanoid_pose(source='G1 detection')
            return

        relative = self._best_relative_measurement(msg)
        if relative is None:
            self.get_logger().warn(
                'Humanoid was detected, but no usable distance/lateral measurement '
                'was available yet.',
                throttle_duration_sec=5.0,
            )
            return

        robot_pose = self._robot_pose_in_world()
        if robot_pose is None:
            self.get_logger().warn(
                f'Cannot save humanoid detection yet: "{self.world_frame}" to '
                f'"{self.base_frame}" TF is unavailable.',
                throttle_duration_sec=5.0,
            )
            return

        robot_x, robot_y, robot_yaw = robot_pose
        forward_m, lateral_m = relative
        pose = PoseStamped()
        pose.header.frame_id = self.world_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = (
            robot_x + math.cos(robot_yaw) * forward_m - math.sin(robot_yaw) * lateral_m
        )
        pose.pose.position.y = (
            robot_y + math.sin(robot_yaw) * forward_m + math.cos(robot_yaw) * lateral_m
        )
        pose.pose.position.z = 0.0
        pose.pose.orientation = _yaw_to_quaternion(self.humanoid_yaw_rad)
        self._save_humanoid_pose(pose, source='G1 detection')

    def _on_humanoid_pose(self, msg: PoseStamped) -> None:
        self._save_humanoid_pose(msg, source='humanoid pose topic')

    def _save_static_humanoid_pose(self, source: str) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self.world_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.static_humanoid_x
        pose.pose.position.y = self.static_humanoid_y
        pose.pose.position.z = 0.0
        pose.pose.orientation = _yaw_to_quaternion(self.humanoid_yaw_rad)
        self._save_humanoid_pose(pose, source=f'{source} static world pose')

    def _save_humanoid_pose(self, pose: PoseStamped, source: str) -> None:
        if pose.header.frame_id and pose.header.frame_id != self.world_frame:
            self.get_logger().warn(
                f'Ignoring humanoid pose in frame "{pose.header.frame_id}"; '
                f'expected "{self.world_frame}".',
                throttle_duration_sec=5.0,
            )
            return

        pose.header.frame_id = self.world_frame
        if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
            pose.header.stamp = self.get_clock().now().to_msg()

        self.latest_humanoid_pose = pose
        p = pose.pose.position
        self.get_logger().info(
            f'Saved humanoid coordinates from {source}: x={p.x:.2f}, y={p.y:.2f}.',
            throttle_duration_sec=2.0,
        )
        self._publish_markers()
        self._stop_exploration_for_parking()
        self._maybe_start_parking()

    def _best_relative_measurement(self, msg: G1Measurements) -> tuple[float, float] | None:
        for index in range(int(msg.count)):
            candidates = (
                (_get(msg.lidar_forward_m, index), _get(msg.lidar_lateral_m, index)),
                (_get(msg.pointcloud_forward_m, index), _get(msg.pointcloud_lateral_m, index)),
                (_get(msg.rgb_forward_m, index), _get(msg.rgb_lateral_m, index)),
            )
            for forward_m, lateral_m in candidates:
                if forward_m is not None and lateral_m is not None and forward_m > 0.0:
                    return forward_m, lateral_m

            for distance_m in (
                _get(msg.sensor_depth_distance_m, index),
                _get(msg.mono_depth_distance_m, index),
            ):
                if distance_m is not None and distance_m > 0.0:
                    return distance_m, 0.0

        return None

    def _tick(self) -> None:
        robot_pose = None
        if self.start_pose_map is None:
            self.start_pose_map = self._lookup_pose_stamped()
            if self.start_pose_map is not None:
                p = self.start_pose_map.pose.position
                self.get_logger().info(
                    f'Saved Ridgeback start pose in "{self.world_frame}": '
                    f'x={p.x:.2f}, y={p.y:.2f}.'
                )

        if not self.exploration_returned_home and self.start_pose_map is not None:
            robot_pose = self._robot_pose_in_world()
            if (
                robot_pose is not None
                and (self.return_to_origin_seen or self.exploration_complete_seen)
            ):
                start = self.start_pose_map.pose.position
                dist_to_start = math.hypot(robot_pose[0] - start.x, robot_pose[1] - start.y)
                if dist_to_start <= self.return_home_tolerance_m:
                    self.exploration_returned_home = True
                    self.get_logger().info(
                        'Treating exploration as returned home: robot is '
                        f'{dist_to_start:.2f}m from saved start pose.'
                    )

        if self.latest_humanoid_pose is not None and self.state == ParkingState.EXPLORING:
            self._stop_exploration_for_parking()

        if self.state in (ParkingState.EXPLORING, ParkingState.STOPPING_EXPLORATION):
            self._maybe_start_parking()

        if self.state == ParkingState.PARKED_WAITING and self.wait_until is not None:
            if self.get_clock().now() >= self.wait_until:
                self._send_home_goal()

    def _maybe_start_parking(self) -> None:
        if self.state not in (ParkingState.EXPLORING, ParkingState.STOPPING_EXPLORATION):
            return
        if self.start_pose_map is None:
            self.get_logger().warn(
                'Waiting for saved start pose before parking.',
                throttle_duration_sec=5.0,
            )
            return
        if self.latest_humanoid_pose is None:
            self.get_logger().warn(
                'Waiting for humanoid detection before parking.',
                throttle_duration_sec=5.0,
            )
            return
        if not self.exploration_stop_sent:
            self._stop_exploration_for_parking()
            return
        if self.state == ParkingState.STOPPING_EXPLORATION:
            if self.exploration_stopped_at is None:
                return
            elapsed = self.get_clock().now() - self.exploration_stopped_at
            if elapsed < Duration(seconds=self.exploration_stop_settle_sec):
                return

        self.parking_pose = self._compute_parking_pose()
        self._publish_markers()
        self._send_parking_goal()

    def _stop_exploration_for_parking(self) -> None:
        if self.exploration_stop_sent:
            return

        msg = Bool()
        msg.data = False
        self.explore_resume_pub.publish(msg)
        self.exploration_stop_sent = True
        self.exploration_stopped_at = self.get_clock().now()
        self.state = ParkingState.STOPPING_EXPLORATION
        self.get_logger().info(
            'Humanoid detected: stopping exploration and taking over Nav2 for parking.'
        )

    def _compute_parking_pose(self) -> PoseStamped:
        assert self.start_pose_map is not None
        assert self.latest_humanoid_pose is not None

        humanoid = self.latest_humanoid_pose.pose.position
        humanoid_yaw = self._humanoid_yaw()
        forward_x = math.cos(humanoid_yaw)
        forward_y = math.sin(humanoid_yaw)
        side_sign = -1.0 if self.hand_side == 'right' else 1.0
        side_yaw = humanoid_yaw + side_sign * math.pi * 0.5
        side_x = math.cos(side_yaw)
        side_y = math.sin(side_yaw)

        px = (
            humanoid.x
            + side_x * self.parking_distance_m
            + forward_x * self.hand_forward_offset_m
        )
        py = (
            humanoid.y
            + side_y * self.parking_distance_m
            + forward_y * self.hand_forward_offset_m
        )
        yaw = math.atan2(humanoid.y - py, humanoid.x - px)

        pose = PoseStamped()
        pose.header.frame_id = self.world_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = 0.0
        pose.pose.orientation = _yaw_to_quaternion(yaw)
        return pose

    def _humanoid_yaw(self) -> float:
        assert self.latest_humanoid_pose is not None
        orientation = self.latest_humanoid_pose.pose.orientation
        norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if norm < 0.01:
            return self.humanoid_yaw_rad
        return _yaw_from_quaternion(orientation)

    def _send_parking_goal(self) -> None:
        assert self.parking_pose is not None
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose server not ready; retrying parking soon.')
            if self.retry_timer is None:
                self.retry_timer = self.create_timer(1.0, self._retry_parking_once)
            return

        self.state = ParkingState.NAVIGATING_TO_HUMANOID
        self._send_goal(self.parking_pose, self._on_parking_result, ParkingState.EXPLORING)
        p = self.parking_pose.pose.position
        self.get_logger().info(
            f'Sent parking goal beside humanoid {self.hand_side} hand: '
            f'x={p.x:.2f}, y={p.y:.2f}.'
        )

    def _retry_parking_once(self) -> None:
        if (
            self.state in (ParkingState.EXPLORING, ParkingState.STOPPING_EXPLORATION)
            and self.parking_pose is not None
        ):
            if self.nav_client.server_is_ready() and self.retry_timer is not None:
                self.retry_timer.cancel()
                self.retry_timer = None
            self._send_parking_goal()

    def _send_home_goal(self) -> None:
        assert self.start_pose_map is not None
        self.state = ParkingState.RETURNING_HOME
        self.start_pose_map.header.stamp = self.get_clock().now().to_msg()
        self._send_goal(self.start_pose_map, self._on_home_result, ParkingState.DONE)
        p = self.start_pose_map.pose.position
        self.get_logger().info(f'Sent return-home goal: x={p.x:.2f}, y={p.y:.2f}.')

    def _send_goal(self, pose: PoseStamped, result_callback, rejected_state: ParkingState) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.nav_client.send_goal_async(goal)

        def _goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Nav2 rejected humanoid parking goal.')
                self.state = rejected_state
                return
            goal_handle.get_result_async().add_done_callback(result_callback)

        send_future.add_done_callback(_goal_response)

    def _on_parking_result(self, future) -> None:
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Parking goal failed with status={result.status}.')
            self.state = ParkingState.EXPLORING
            return

        self.state = ParkingState.PARKED_WAITING
        self.wait_until = self.get_clock().now() + Duration(seconds=self.park_wait_sec)
        self.get_logger().info(f'Parked near humanoid; waiting {self.park_wait_sec:.1f}s.')

    def _on_home_result(self, future) -> None:
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.state = ParkingState.DONE
            self.get_logger().info('Returned to original start pose after humanoid parking.')
        else:
            self.get_logger().warn(f'Return-home goal failed with status={result.status}.')

    def _robot_pose_in_world(self) -> tuple[float, float, float] | None:
        pose = self._lookup_pose_stamped()
        if pose is None:
            return None
        p = pose.pose.position
        return p.x, p.y, _yaw_from_quaternion(pose.pose.orientation)

    def _lookup_pose_stamped(self) -> PoseStamped | None:
        for candidate in candidate_base_frames(self.base_frame):
            pose = self._lookup_pose_stamped_in_frame(candidate)
            if pose is not None:
                if candidate != self.base_frame and candidate != self.last_base_frame_fallback:
                    self.get_logger().warn(
                        f'Configured base frame "{self.base_frame}" is unavailable; '
                        f'using "{candidate}" for parking transforms.'
                    )
                    self.last_base_frame_fallback = candidate
                return pose

        return None

    def _lookup_pose_stamped_in_frame(self, base_frame: str) -> PoseStamped | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return None

        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _publish_markers(self) -> None:
        if self.latest_humanoid_pose is None:
            return

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        humanoid = self.latest_humanoid_pose.pose.position

        circle_center = self.parking_pose.pose.position if self.parking_pose else humanoid
        markers.markers.append(self._circle_marker(0, circle_center, now))

        if self.parking_pose is not None:
            start = self.start_pose_map.pose.position if self.start_pose_map else None
            line_points = []
            if start is not None:
                line_points.append(Point(x=start.x, y=start.y, z=0.06))
            target = self.parking_pose.pose.position
            line_points.append(Point(x=target.x, y=target.y, z=0.06))
            line_points.append(Point(x=humanoid.x, y=humanoid.y, z=0.06))
            markers.markers.append(self._line_marker(1, line_points, now))

        markers.markers.append(self._sphere_marker(2, humanoid, now))
        self.marker_pub.publish(markers)

    def _green_marker(self, marker_id: int, marker_type: int, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = 'humanoid_parking'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def _circle_marker(self, marker_id: int, center, stamp) -> Marker:
        marker = self._green_marker(marker_id, Marker.LINE_STRIP, stamp)
        marker.scale.x = 0.06
        radius = 0.45
        for i in range(49):
            angle = 2.0 * math.pi * i / 48
            marker.points.append(
                Point(
                    x=center.x + radius * math.cos(angle),
                    y=center.y + radius * math.sin(angle),
                    z=0.06,
                )
            )
        return marker

    def _line_marker(self, marker_id: int, points: list[Point], stamp) -> Marker:
        marker = self._green_marker(marker_id, Marker.LINE_STRIP, stamp)
        marker.scale.x = 0.08
        marker.points = points
        return marker

    def _sphere_marker(self, marker_id: int, point, stamp) -> Marker:
        marker = self._green_marker(marker_id, Marker.SPHERE, stamp)
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = 0.2
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        return marker


def main() -> None:
    rclpy.init()
    node = HumanoidParkingNode()
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
