#!/usr/bin/env python3
"""SLAM-only front+rear LaserScan merger.

This is a SLAM-input merge ONLY:
  - /sensors/lidar2d_0/scan and /sensors/lidar2d_1/scan are consumed
    read-only and keep publishing unchanged to every other consumer
    (Nav2 costmaps, collision_monitor, RViz) exactly as before.
  - Output publishes to a NEW topic (default sensors/scan_slam_merged);
    nothing subscribes to it unless slam.launch.py's slam_source:=merged
    explicitly points slam_toolbox's scan_topic at it.
  - No stale-bin reuse, no fake interpolation: a merge is built ONLY from
    the current front scan plus (if paired within tolerance) the current
    rear scan. A bin neither scan currently observes stays +inf. Front and
    rear are re-paired fresh on every front-scan arrival -- there is no
    persistent state carried between merges besides the last few raw scans
    kept only to find a same-instant rear partner.

Synchronization: front-scan arrival triggers a merge attempt. The nearest
buffered rear scan within --sync-tolerance-sec is used; if none qualifies,
the merge still publishes using front-only data (bins rear would have
filled stay +inf) rather than silently drop the cycle or reuse stale rear
data.

Common frame: motion-compensates the rear scan from ITS OWN capture time
into the front scan's capture time using the independent odom->base_link
chain (NEVER slam_toolbox's own map->base_link estimate, which would be
circular) at each scan's own timestamp -- never "latest" TF:

    p_base_at_t_front =
        inverse(T_odom_base(t_front)) * T_odom_base(t_rear)
        * T_base_lidar_rear * p_rear

Front points only need the static T_base_lidar_front extrinsic (already
at the reference time by construction). Both project onto one shared
angular grid at front's native angle_increment; a genuine bin collision
(both sensors observed the same bin) keeps the NEARER (smaller-range)
return, matching one physical surface being closer to one sensor than
the other, not an average.
"""
import math
from collections import deque

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ScanMergerNode(Node):
    def __init__(self):
        super().__init__('scan_merger_node')

        self.declare_parameter('front_topic', 'sensors/lidar2d_0/scan')
        self.declare_parameter('rear_topic', 'sensors/lidar2d_1/scan')
        self.declare_parameter('output_topic', 'sensors/scan_slam_merged')
        self.declare_parameter('output_frame_id', 'base_link')
        self.declare_parameter('sync_tolerance_sec', 0.02)
        self.declare_parameter('log_period_sec', 5.0)
        # Static extrinsics (base_link -> lidar) -- provably constant across
        # all yaw, so looked up once here rather than per scan.
        self.declare_parameter('front_lidar_xyz_yaw',
                               [0.3922, 0.0, 0.0, 0.0])
        self.declare_parameter('rear_lidar_xyz_yaw',
                               [-0.3922, 0.0, 0.0, math.pi])

        front_topic = self.get_parameter('front_topic').value
        rear_topic = self.get_parameter('rear_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.tolerance = float(self.get_parameter('sync_tolerance_sec').value)
        log_period = float(self.get_parameter('log_period_sec').value)
        fx, fy, _, fyaw = self.get_parameter('front_lidar_xyz_yaw').value
        rx, ry, _, ryaw = self.get_parameter('rear_lidar_xyz_yaw').value
        self.front_pose = (fx, fy, fyaw)
        self.rear_pose = (rx, ry, ryaw)
        # Merged points are expressed in base_link, not in a lidar frame, so a
        # return at the sensor's own range_max lands up to |lidar offset|
        # further out (10.00 m ahead of the front lidar = 10.39 m from
        # base_link). Publishing the sensor's range_max verbatim would make
        # consumers discard that outermost shell -- slam_toolbox drops any
        # reading above range_max -- so merged mode would see LESS than
        # front_only. Pad by the largest lidar offset.
        self._range_pad = max(math.hypot(fx, fy), math.hypot(rx, ry))

        sensor_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

        self._rear_buffer = deque(maxlen=20)
        # Pending front scans awaiting a merge decision. A single-threaded
        # executor does not guarantee front/rear callback ORDER within a
        # cycle even when both sensors are stamp-identical -- triggering
        # the merge attempt only from _on_front would race whichever
        # callback happens to run first and silently pair every front scan
        # against the PREVIOUS cycle's rear (one whole sensor period stale).
        # Buffer front scans and re-attempt the merge from BOTH callbacks.
        self._front_pending = deque(maxlen=20)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(LaserScan, output_topic, sensor_qos)
        self.create_subscription(LaserScan, rear_topic, self._on_rear, sensor_qos)
        self.create_subscription(LaserScan, front_topic, self._on_front, sensor_qos)
        self.create_timer(max(log_period, 1.0), self._log_summary)

        self._n_front = 0
        self._n_paired = 0
        self._n_front_only = 0
        self._n_rear_dropped = 0
        self._deltas = deque(maxlen=500)
        self._tf_misses = 0

        self.get_logger().info(
            f'scan_merger_node: {front_topic} + {rear_topic} '
            f'-> {output_topic}, frame={self.output_frame_id}, '
            f'sync_tolerance={self.tolerance * 1000:.0f}ms. '
            f'{front_topic}/{rear_topic} keep publishing unchanged to every '
            f'other consumer.')

    def _on_rear(self, msg: LaserScan):
        self._rear_buffer.append((stamp_s(msg.header.stamp), msg))
        self._drain_pending()

    def _nearest_rear(self, t_front):
        if not self._rear_buffer:
            return None, None
        t_rear, msg = min(self._rear_buffer, key=lambda item: abs(item[0] - t_front))
        return (t_rear, msg) if abs(t_rear - t_front) <= self.tolerance else (None, None)

    def _odom_base(self, t):
        try:
            tr = self._tf_buffer.lookup_transform(
                'odom', 'base_link',
                Time(seconds=int(t), nanoseconds=int((t % 1) * 1e9)))
        except tf2_ros.TransformException:
            return None
        return (tr.transform.translation.x, tr.transform.translation.y,
               yaw_of(tr.transform.rotation))

    @staticmethod
    def _compose(a, b):
        ax, ay, ayaw = a
        bx, by, byaw = b
        c, s = math.cos(ayaw), math.sin(ayaw)
        return (ax + c * bx - s * by, ay + s * bx + c * by, ayaw + byaw)

    @staticmethod
    def _inverse(a):
        x, y, yaw = a
        c, s = math.cos(yaw), math.sin(yaw)
        return (-(c * x + s * y), -(-s * x + c * y), -yaw)

    def _scan_points_in_frame(self, msg: LaserScan, lidar_pose):
        """Points from a raw LaserScan, in the frame `lidar_pose` (x,y,yaw)
        is expressed relative to (i.e. apply lidar_pose as lidar->frame)."""
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        valid = (np.isfinite(ranges) & (ranges >= msg.range_min)
                & (ranges <= msg.range_max))
        idx = np.nonzero(valid)[0]
        if idx.size == 0:
            return np.empty((0, 2))
        angles = msg.angle_min + idx * msg.angle_increment
        lx = ranges[valid] * np.cos(angles)
        ly = ranges[valid] * np.sin(angles)
        px, py, pyaw = lidar_pose
        c, s = math.cos(pyaw), math.sin(pyaw)
        return np.column_stack((px + c * lx - s * ly, py + s * lx + c * ly))

    def _on_front(self, front_msg: LaserScan):
        self._n_front += 1
        self._front_pending.append((stamp_s(front_msg.header.stamp), front_msg))
        self._drain_pending()

    def _drain_pending(self):
        """Try to resolve every buffered, not-yet-published front scan.
        Called from BOTH callbacks so pairing succeeds regardless of which
        topic's message the executor happens to process first this cycle
        (see _front_pending's docstring note in __init__)."""
        while self._front_pending:
            t_front, front_msg = self._front_pending[0]
            t_rear, rear_msg = self._nearest_rear(t_front)
            # A same-or-later rear sample exists in the buffer, or this
            # front entry has aged past 2x tolerance with nothing arriving
            # -- either way it's time to resolve (merge or front-only).
            age = self.get_clock().now().nanoseconds * 1e-9 - t_front
            if rear_msg is None and age < 2 * self.tolerance:
                break   # give the rear callback a chance to still arrive
            self._front_pending.popleft()
            self._publish_merge(t_front, front_msg, t_rear, rear_msg)

    def _publish_merge(self, t_front, front_msg, t_rear, rear_msg):
        front_points = self._scan_points_in_frame(front_msg, self.front_pose)
        rear_points = np.empty((0, 2))

        if rear_msg is not None:
            delta = t_front - t_rear
            self._deltas.append(delta)
            if abs(delta) < 1e-6:
                rear_points = self._scan_points_in_frame(rear_msg, self.rear_pose)
                self._n_paired += 1
            else:
                odom_front = self._odom_base(t_front)
                odom_rear = self._odom_base(t_rear)
                if odom_front is None or odom_rear is None:
                    # No odom chain -> the rear scan cannot be brought into
                    # the front's capture time, so it is dropped whole. That
                    # publish is front-only data; counting it as "paired"
                    # would report a healthy merge that never happened.
                    self._tf_misses += 1
                    self._n_rear_dropped += 1
                else:
                    # motion-compensate rear (captured at t_rear) into the
                    # base_link frame AT t_front, via the odom chain only
                    rear_local = self._scan_points_in_frame(rear_msg, self.rear_pose)
                    correction = self._compose(self._inverse(odom_front), odom_rear)
                    cx, cy, cyaw = correction
                    c, s = math.cos(cyaw), math.sin(cyaw)
                    rear_points = np.column_stack((
                        cx + c * rear_local[:, 0] - s * rear_local[:, 1],
                        cy + s * rear_local[:, 0] + c * rear_local[:, 1],
                    )) if rear_local.shape[0] else rear_local
                    self._n_paired += 1
        else:
            self._n_front_only += 1

        increment = front_msg.angle_increment
        n_bins = int(round(2.0 * math.pi / increment))
        angle_min = -math.pi
        ranges = np.full(n_bins, np.inf, dtype=np.float64)

        for points in (front_points, rear_points):
            if points.shape[0] == 0:
                continue
            r = np.hypot(points[:, 0], points[:, 1])
            ang = np.arctan2(points[:, 1], points[:, 0])
            bin_idx = np.mod(np.round((ang - angle_min) / increment).astype(int),
                            n_bins)
            # "nearest valid range wins" on a genuine bin collision
            np.minimum.at(ranges, bin_idx, r)

        out = LaserScan()
        out.header.stamp = front_msg.header.stamp   # t_front is the reference time
        out.header.frame_id = self.output_frame_id
        out.angle_min = angle_min
        out.angle_max = angle_min + (n_bins - 1) * increment
        out.angle_increment = increment
        out.time_increment = 0.0
        out.scan_time = front_msg.scan_time
        out.range_min = front_msg.range_min
        out.range_max = front_msg.range_max + self._range_pad
        out.ranges = ranges.tolist()
        self._pub.publish(out)

    def _log_summary(self):
        if self._deltas:
            arr = np.abs(np.array(self._deltas))
            stats = (f'delta_abs_mean={arr.mean() * 1000:.1f}ms '
                    f'delta_abs_p95={np.percentile(arr, 95) * 1000:.1f}ms '
                    f'delta_abs_max={arr.max() * 1000:.1f}ms')
        else:
            stats = 'no paired samples yet'
        self.get_logger().info(
            f'scan_merger: front={self._n_front} paired={self._n_paired} '
            f'front_only(no rear in tolerance)={self._n_front_only} '
            f'rear_dropped(no odom TF)={self._n_rear_dropped} '
            f'tf_misses={self._tf_misses} {stats}')


def main():
    rclpy.init()
    node = ScanMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
