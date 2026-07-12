#!/usr/bin/env python3
"""Publish a live localization-error HUD panel (GT pose vs SLAM estimate).

Isaac-only: the sim runner publishes the robot's ground-truth pose on
``ground_truth/pose``; SLAM's estimate is the ``map -> base_link`` TF. This node
reports the planar translation error and the yaw error between them, live, as an
``rviz_2d_overlay_msgs/OverlayText`` on ``hud/localization`` for the HUD
aggregator to render. The gz path has no ground-truth pose publisher, so this
node is launched only under ``sim:=isaac``.

The error is the same quantity ``tools/isaac/slam_quality_probe.py`` scores after
a run as ``pose_rmse_m`` / ``pose_max_m``; this is the live view of it, so drift
the probe would only report at the end is visible while the run is happening —
the kind of thing worth eyeballing in RViz.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from rviz_2d_overlay_msgs.msg import OverlayText
from tf2_msgs.msg import TFMessage
import tf2_ros

# Non-breaking space: the overlay renders as HTML, which collapses runs of
# normal spaces and breaks the monospace columns.
NBSP = ' '


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _ang_norm(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LocalizationOverlayNode(Node):
    def __init__(self):
        super().__init__('localization_overlay_node')

        self.declare_parameter('gt_topic', 'ground_truth/pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('panel_topic', 'hud/localization')
        self.declare_parameter('publish_rate_hz', 2.0)

        gt_topic = self.get_parameter('gt_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        panel_topic = self.get_parameter('panel_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._gt = None            # latest ground truth (x, y, yaw)
        self._max_trans = 0.0
        self._max_yaw = 0.0

        # tf2_ros.TransformListener hardcodes the absolute /tf topics; TF in this
        # stack flows on the robot namespace, so feed the buffer from the
        # relative tf/tf_static topics directly (same approach as the probe).
        self._tf = tf2_ros.Buffer()
        self.create_subscription(TFMessage, 'tf', self._on_tf, 100)
        self.create_subscription(
            TFMessage, 'tf_static', self._on_tf_static,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))

        sensor_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseStamped, gt_topic, self._on_gt, sensor_qos)

        self.pub = self.create_publisher(OverlayText, panel_topic, 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    def _on_tf(self, msg):
        for tr in msg.transforms:
            self._tf.set_transform(tr, 'localization_overlay')

    def _on_tf_static(self, msg):
        for tr in msg.transforms:
            self._tf.set_transform_static(tr, 'localization_overlay')

    def _on_gt(self, msg):
        self._gt = (msg.pose.position.x, msg.pose.position.y,
                    _yaw_of(msg.pose.orientation))

    def _slam_pose(self):
        # Time() == latest available transform (no clock dependency).
        try:
            tr = self._tf.lookup_transform(self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return None
        t = tr.transform.translation
        return (t.x, t.y, _yaw_of(tr.transform.rotation))

    def _tick(self):
        m = OverlayText()
        m.action = OverlayText.ADD
        m.text = self._panel_text()
        self.pub.publish(m)

    def _panel_text(self):
        title = 'LOCALIZATION'
        if self._gt is None:
            return self._pad(f'{title}\n waiting for ground truth...')
        slam = self._slam_pose()
        if slam is None:
            return self._pad(f'{title}\n waiting for map->base_link...')
        trans = math.hypot(slam[0] - self._gt[0], slam[1] - self._gt[1])
        yaw_deg = math.degrees(_ang_norm(slam[2] - self._gt[2]))
        self._max_trans = max(self._max_trans, trans)
        self._max_yaw = max(self._max_yaw, abs(yaw_deg))
        lines = [
            title,
            f' trans err {trans:5.2f} m  (max {self._max_trans:4.2f})',
            f' yaw err  {yaw_deg:+5.1f} deg (max {self._max_yaw:4.1f})',
        ]
        return self._pad('\n'.join(lines))

    @staticmethod
    def _pad(text):
        return text.replace(' ', NBSP)


def main():
    rclpy.init()
    node = LocalizationOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
