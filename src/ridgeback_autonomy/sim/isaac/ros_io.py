"""In-process ROS 2 I/O for the Isaac runner (rclpy side).

Owns everything that is NOT a GPU sensor path: /clock, cmd_vel
subscriptions (TwistStamped per the Nav2 contract, plus a tolerant plain
Twist), odometry + odom->base_link TF, and the exact ground-truth pose.
GPU sensors (RTX lidar, camera) publish through OmniGraph bridge helpers
instead (sensors.py, P4).

Runs on the system rclpy/CycloneDDS that the sourced workspace provides
(the bridge loads system ROS when it is sourced before launch). The node
lives in the robot namespace and remaps /tf -> tf like every node in
this stack.

TF ownership note: odom->base_link belongs to the EKF include (matching
the real platform, where robot_localization owns it). The runner can
publish it directly with odom_tf=True for standalone/debug runs without
the EKF.
"""
from __future__ import annotations

import math

import numpy as np


def _quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class LidarScanAssembler:
    """Bin the bridge's RTX lidar PointCloud2 into the contract LaserScan.

    The 6.0.1 bridge laser_scan writer mislabels ROTARY sensors with an
    azimuth ROI (hardcoded 360-deg FOV — see sensors.py), but its
    point_cloud output is sensor-frame correct. A published bin only ever
    comes from the current completed tick (updated == t) — a bin that
    wasn't refreshed by either half-arc cloud this tick is +inf, never a
    stale hit carried over from an earlier sweep (measured to reduce
    rotation-induced map smear vs. carrying bins across sweeps).

    UST-10LX geometry: 1081 bins, -135..+135 deg, 0.25 deg step, 40 Hz.
    """

    N_BINS = 1081
    ANGLE_MIN = -3.0 * math.pi / 4.0
    ANGLE_INC = math.radians(0.25)
    RANGE_MIN = 0.06
    RANGE_MAX = 10.0
    SCAN_PERIOD = 1.0 / 40.0

    def __init__(self, node, index: int):
        from functools import partial

        from rclpy.qos import QoSProfile
        from sensor_msgs.msg import LaserScan, PointCloud2

        self._LaserScan = LaserScan
        self._frame = f"lidar2d_{index}_laser"
        self._ranges = np.full(self.N_BINS, np.inf, dtype=np.float32)
        self._updated = np.full(self.N_BINS, -1.0, dtype=np.float64)
        self._half_stamp = [None, None]
        self._pub = node.create_publisher(
            LaserScan, f"sensors/lidar2d_{index}/scan", QoSProfile(depth=10))
        # two half-arc clouds per lidar (sensors.py: the rotary model only
        # fires 180 deg of drum transit per tick; two prims cover the arc)
        node.create_subscription(
            PointCloud2, f"sensors/lidar2d_{index}/points",
            partial(self._on_cloud, 0), 10)
        node.create_subscription(
            PointCloud2, f"sensors/lidar2d_{index}/points_l",
            partial(self._on_cloud, 1), 10)

    def _on_cloud(self, half, msg):
        from sensor_msgs_py import point_cloud2 as pc2

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._half_stamp[half] = t
        pts = pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        x = pts["x"].astype(np.float64)
        y = pts["y"].astype(np.float64)
        r = np.hypot(x, y)
        ok = r > self.RANGE_MIN * 0.5   # zero-range = invalid slot padding
        if ok.any():
            r = r[ok]
            bins = np.round((np.arctan2(y[ok], x[ok]) - self.ANGLE_MIN)
                            / self.ANGLE_INC).astype(int)
            good = (bins >= 0) & (bins < self.N_BINS)
            self._ranges[bins[good]] = r[good]
            self._updated[bins[good]] = t
        # both prims capture per the same tick and stamp identically —
        # publish once per completed pair so a scan never mixes two
        # capture instants (a half-stale seam smears SLAM under rotation)
        if self._half_stamp[0] != self._half_stamp[1]:
            return

        out = self._ranges.copy()
        out[self._updated != t] = np.inf
        scan = self._LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self._frame
        scan.angle_min = self.ANGLE_MIN
        scan.angle_max = self.ANGLE_MIN + self.ANGLE_INC * (self.N_BINS - 1)
        scan.angle_increment = self.ANGLE_INC
        scan.scan_time = self.SCAN_PERIOD
        scan.time_increment = 0.0
        scan.range_min = self.RANGE_MIN
        scan.range_max = self.RANGE_MAX
        scan.ranges = out.tolist()
        self._pub.publish(scan)


class RosIO:
    def __init__(self, namespace: str, base_frame: str = "base_link",
                 odom_frame: str = "odom", odom_tf: bool = False,
                 imu_frame: str = "imu_0_link"):
        import rclpy
        from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_srvs.srv import Trigger
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

        rclpy.init()
        self._rclpy = rclpy
        self.node = rclpy.create_node(
            "isaac_sim_runner", namespace=namespace,
            # tf remap convention used across this stack
            cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
            automatically_declare_parameters_from_overrides=True,
        )
        self._base_frame = base_frame
        self._odom_frame = odom_frame
        self._imu_frame = imu_frame
        self._odom_tf = odom_tf

        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_stamp_wall = None   # set by runner via sim-time now()

        # /clock is global (un-namespaced) by contract
        self._clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        # raw wheel odometry — the EKF include turns this into
        # platform/odom/filtered (+ TF), same shape as the real platform
        self._odom_pub = self.node.create_publisher(
            Odometry, "platform/odom", 10)
        self._imu_pub = self.node.create_publisher(
            Imu, "sensors/imu_0/data_raw", 10)
        self._gt_pub = self.node.create_publisher(
            PoseStamped, "ground_truth/pose", 10)
        # The planar Isaac drive rig does not expose the URDF wheel joints as
        # articulation DOFs. Publish their (static, always-zero) positions so
        # robot_state_publisher can still provide all four wheel-link TFs.
        self._joint_state_pub = self.node.create_publisher(
            JointState, "joint_states", 10)
        self._tf = TransformBroadcaster(self.node)
        self._static_tf = StaticTransformBroadcaster(self.node)

        self._new_cmd = None
        self.node.create_subscription(
            TwistStamped, "cmd_vel", self._on_twist_stamped, 10)
        # tolerant plain-Twist fallback on the same topic (logs once)
        self._warned_plain = False
        self.node.create_subscription(Twist, "cmd_vel", self._on_twist, 10)

        # in-session benchmark reset (P7): teleport the robot back to spawn +
        # re-zero odom without relaunching the sim, for probe --repeat N. The
        # callback only raises a flag; the actual rig teleport happens on the
        # main loop (physics/articulation ops are not thread-safe from the
        # executor callback thread).
        self._reset_request = False
        self.node.create_service(Trigger, "sim/reset", self._on_reset)

        self._msgs = dict(Odometry=Odometry, PoseStamped=PoseStamped,
                          Clock=Clock, JointState=JointState)
        # contract LaserScan assembled from the bridge's point clouds
        # (see LidarScanAssembler for why the bridge's own laser_scan
        # output cannot be used)
        self._scan_assemblers = [LidarScanAssembler(self.node, i)
                                 for i in (0, 1)]
        self._publish_base_link_shim()

    # ---- subscriptions -----------------------------------------------------

    def _on_twist_stamped(self, msg):
        t = msg.twist
        self._new_cmd = (t.linear.x, t.linear.y, t.angular.z)

    def _on_twist(self, msg):
        if not self._warned_plain:
            self.node.get_logger().warn(
                "plain Twist on cmd_vel — contract is TwistStamped; accepting")
            self._warned_plain = True
        self._new_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)

    def take_cmd(self):
        """Return and clear the newest cmd (vx, vy, wz), or None."""
        cmd, self._new_cmd = self._new_cmd, None
        return cmd

    def _on_reset(self, request, response):
        self._reset_request = True
        response.success = True
        response.message = "reset queued"
        return response

    def take_reset(self) -> bool:
        """True once if a reset was requested since the last call (consumed
        by the main loop, which does the actual teleport)."""
        r, self._reset_request = self._reset_request, False
        return r

    # ---- publications ------------------------------------------------------

    def _stamp(self, sim_time: float):
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = int(sim_time)
        t.nanosec = int((sim_time - int(sim_time)) * 1e9)
        return t

    def publish_clock(self, sim_time: float):
        msg = self._msgs["Clock"]()
        msg.clock = self._stamp(sim_time)
        self._clock_pub.publish(msg)

    def publish_wheel_joint_states(self, sim_time: float):
        msg = self._msgs["JointState"]()
        msg.header.stamp = self._stamp(sim_time)
        msg.name = [
            "front_left_wheel_joint", "front_right_wheel_joint",
            "rear_left_wheel_joint", "rear_right_wheel_joint",
        ]
        msg.position = [0.0] * 4
        self._joint_state_pub.publish(msg)

    def publish_odom(self, sim_time: float, odom_state, body_twist):
        stamp = self._stamp(sim_time)

        msg = self._msgs["Odometry"]()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = odom_state.x
        msg.pose.pose.position.y = odom_state.y
        qx, qy, qz, qw = _quat_from_yaw(odom_state.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = body_twist[0]
        msg.twist.twist.linear.y = body_twist[1]
        msg.twist.twist.angular.z = body_twist[2]
        # non-zero diagonal covariances so the EKF weights it sanely
        for i, v in ((0, 1e-3), (7, 1e-3), (35, 1e-3)):
            msg.pose.covariance[i] = v
            msg.twist.covariance[i] = v
        self._odom_pub.publish(msg)

        if self._odom_tf:
            from geometry_msgs.msg import TransformStamped
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = odom_state.x
            tf.transform.translation.y = odom_state.y
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf.sendTransform(tf)

    def publish_imu(self, sim_time: float, wz: float, ax: float, ay: float):
        """Raw IMU (no orientation estimate, like a real driver's
        data_raw): body-frame gyro z and planar accel + gravity."""
        from sensor_msgs.msg import Imu
        msg = Imu()
        msg.header.stamp = self._stamp(sim_time)
        msg.header.frame_id = self._imu_frame
        msg.orientation_covariance[0] = -1.0     # no orientation
        msg.angular_velocity.z = wz
        msg.angular_velocity_covariance[8] = 1e-4
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = 9.81
        msg.linear_acceleration_covariance[0] = 1e-2
        msg.linear_acceleration_covariance[4] = 1e-2
        msg.linear_acceleration_covariance[8] = 1e-2
        self._imu_pub.publish(msg)

    def publish_ground_truth(self, sim_time: float, x, y, yaw):
        msg = self._msgs["PoseStamped"]()
        msg.header.stamp = self._stamp(sim_time)
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._gt_pub.publish(msg)

    def _publish_base_link_shim(self):
        """Identity base_link -> <ns>/robot/base_link (perception default)."""
        from geometry_msgs.msg import TransformStamped
        ns = self.node.get_namespace().strip("/")
        tf = TransformStamped()
        tf.header.frame_id = self._base_frame
        tf.child_frame_id = f"{ns}/robot/base_link"
        tf.transform.rotation.w = 1.0
        self._static_tf.sendTransform(tf)

    # ---- loop glue -----------------------------------------------------------

    def spin_once(self):
        # drain the ready queue, not a single callback: four half-arc
        # cloud streams (~28 Hz each) plus cmd/clock would otherwise
        # backlog behind a one-callback-per-render-frame budget and the
        # scan assembler's stamp pairing compares stale halves
        for _ in range(32):
            self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        self.node.destroy_node()
        self._rclpy.shutdown()
