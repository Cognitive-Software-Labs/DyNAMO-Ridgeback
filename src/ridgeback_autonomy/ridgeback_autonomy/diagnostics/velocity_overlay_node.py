#!/usr/bin/env python3
"""Publish a screen-anchored OverlayText with the four-stage velocity chain.

Renders as a fixed HUD in the RViz 3D view (top-left corner) using a monospace
font so the columns line up. Requires the rviz_2d_overlay_plugins package
(built from source in this workspace).

Stages tracked:
  planned    -> cmd_vel_nav      (MPPI raw output, TwistStamped)
  capped     -> cmd_vel_smoothed (velocity_smoother output, TwistStamped)
  controller -> cmd_vel          (collision_monitor output, TwistStamped)
  actual     -> platform/odom/filtered (realized velocity, nav_msgs/Odometry)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rviz_2d_overlay_msgs.msg import OverlayText


class VelocityOverlayNode(Node):
    def __init__(self):
        super().__init__('velocity_overlay_node')

        self.declare_parameter('text_size', 11.0)
        self.declare_parameter('horizontal_distance', 10)
        self.declare_parameter('vertical_distance', 10)
        self.declare_parameter('overlay_width', 320)
        self.declare_parameter('overlay_height', 140)
        self.declare_parameter('font', 'DejaVu Sans Mono')
        self.declare_parameter('marker_topic', 'velocity_overlay')
        self.declare_parameter('publish_rate_hz', 10.0)

        self.text_size = float(self.get_parameter('text_size').value)
        self.h_dist = int(self.get_parameter('horizontal_distance').value)
        self.v_dist = int(self.get_parameter('vertical_distance').value)
        self.width = int(self.get_parameter('overlay_width').value)
        self.height = int(self.get_parameter('overlay_height').value)
        self.font = self.get_parameter('font').value
        marker_topic = self.get_parameter('marker_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.latest = {
            'planned': (0.0, 0.0, 0.0),
            'capped': (0.0, 0.0, 0.0),
            'controller': (0.0, 0.0, 0.0),
            'actual': (0.0, 0.0, 0.0),
        }

        self.create_subscription(TwistStamped, 'cmd_vel_nav',
                                 lambda m: self._on_twist('planned', m), sensor_qos)
        self.create_subscription(TwistStamped, 'cmd_vel_smoothed',
                                 lambda m: self._on_twist('capped', m), sensor_qos)
        self.create_subscription(TwistStamped, 'cmd_vel',
                                 lambda m: self._on_twist('controller', m), sensor_qos)
        self.create_subscription(Odometry, 'platform/odom/filtered',
                                 self._on_odom, sensor_qos)

        self.pub = self.create_publisher(OverlayText, marker_topic, 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    def _on_twist(self, key, msg):
        t = msg.twist
        self.latest[key] = (t.linear.x, t.linear.y, t.angular.z)

    def _on_odom(self, msg):
        t = msg.twist.twist
        self.latest['actual'] = (t.linear.x, t.linear.y, t.angular.z)

    def _tick(self):
        m = OverlayText()
        m.action = OverlayText.ADD
        m.width = self.width
        m.height = self.height
        m.horizontal_alignment = OverlayText.LEFT
        m.vertical_alignment = OverlayText.TOP
        m.horizontal_distance = self.h_dist
        m.vertical_distance = self.v_dist
        m.text_size = self.text_size
        m.font = self.font
        m.bg_color.r = 0.0
        m.bg_color.g = 0.0
        m.bg_color.b = 0.0
        m.bg_color.a = 0.5
        m.fg_color.r = 0.1
        m.fg_color.g = 1.0
        m.fg_color.b = 0.1
        m.fg_color.a = 1.0
        m.text = self._format_text()
        self.pub.publish(m)

    # Column layout (monospace): stage label + three right-aligned value cols.
    LABEL_W = 11
    VAL_W = 8

    def _format_text(self):
        def row(label, vx, vy, wz):
            return (f'{label:<{self.LABEL_W}}'
                    f'{vx:>{self.VAL_W}}{vy:>{self.VAL_W}}{wz:>{self.VAL_W}}')

        def data_row(label, vals):
            vx, vy, wz = vals
            return row(label, f'{vx:+.2f}', f'{vy:+.2f}', f'{wz:+.2f}')

        header = row('stage', 'vx', 'vy', 'wz')
        rule = '-' * (self.LABEL_W + 3 * self.VAL_W)
        rows = [data_row(k, self.latest[k])
                for k in ('planned', 'capped', 'controller', 'actual')]
        text = '\n'.join([header, rule, *rows])
        # The overlay renders as HTML (QStaticText), which collapses runs of
        # normal spaces and breaks column alignment. Use non-breaking spaces so
        # the monospace padding is preserved.
        return text.replace(' ', ' ')


def main():
    rclpy.init()
    node = VelocityOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
