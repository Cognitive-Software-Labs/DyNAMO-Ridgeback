#!/usr/bin/env python3
"""General HUD aggregator: merge labeled panels into one screen overlay.

Producers (velocity, coverage, …) each publish their section as an
``rviz_2d_overlay_msgs/OverlayText`` on their own panel topic. This node
subscribes to an ordered list of those topics, keeps the latest text from each,
and renders a single combined ``OverlayText`` on ``hud_overlay`` — stacked
top-down in the configured order. Adding a new metric to the HUD is just a new
publisher plus its topic in the ``panels`` list; no RViz changes needed.

The HUD owns the container style (font, colours, position, size); it reads only
each panel's ``.text`` field. Panels are expected to pad columns with
non-breaking spaces (the overlay renders as HTML, which collapses normal runs of
spaces) — this node joins them verbatim.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from rviz_2d_overlay_msgs.msg import OverlayText


class HudNode(Node):
    def __init__(self):
        super().__init__('hud_node')

        self.declare_parameter('panels', ['hud/velocity', 'hud/coverage'])
        self.declare_parameter('marker_topic', 'hud_overlay')
        self.declare_parameter('text_size', 11.0)
        self.declare_parameter('horizontal_distance', 10)
        self.declare_parameter('vertical_distance', 10)
        self.declare_parameter('overlay_width', 360)
        self.declare_parameter('font', 'DejaVu Sans Mono')
        self.declare_parameter('publish_rate_hz', 5.0)

        self.panels = list(self.get_parameter('panels').value)
        marker_topic = self.get_parameter('marker_topic').value
        self.text_size = float(self.get_parameter('text_size').value)
        self.h_dist = int(self.get_parameter('horizontal_distance').value)
        self.v_dist = int(self.get_parameter('vertical_distance').value)
        self.width = int(self.get_parameter('overlay_width').value)
        self.font = self.get_parameter('font').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Latest text per panel topic, preserving declared order.
        self._texts = {topic: '' for topic in self.panels}
        for topic in self.panels:
            # Bind the topic name so the callback knows which panel it is.
            self.create_subscription(
                OverlayText, topic,
                lambda msg, t=topic: self._on_panel(t, msg), qos)

        self.pub = self.create_publisher(OverlayText, marker_topic, 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    def _on_panel(self, topic, msg):
        self._texts[topic] = msg.text or ''

    def _tick(self):
        sections = [self._texts[t] for t in self.panels if self._texts[t]]
        text = '\n\n'.join(sections)

        # Auto-size height from the line count so panels can grow freely.
        # The overlay renders each line at ~1.8x the font size; under-counting
        # the per-line height makes the deficit accumulate down the panel and
        # clips the lower lines, so track the font size and add padding.
        line_count = text.count('\n') + 1 if text else 1
        line_px = self.text_size * 1.8 + 4.0
        height = int(line_count * line_px + 16)

        m = OverlayText()
        m.action = OverlayText.ADD
        m.width = self.width
        m.height = height
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
        m.text = text
        self.pub.publish(m)


def main():
    rclpy.init()
    node = HudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
