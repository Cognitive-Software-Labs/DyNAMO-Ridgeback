#!/usr/bin/env python3
"""General HUD aggregator: merge labeled panels into one screen overlay.

Producers (velocity, coverage, …) each publish their section as an
``rviz_2d_overlay_msgs/OverlayText`` on their own panel topic. This node
subscribes to an ordered list of those topics, keeps the latest text from each,
and renders a single combined ``OverlayText`` on ``hud_overlay`` — stacked
top-down in the configured order. Adding a new metric to the HUD is just a new
publisher plus its topic in the ``panels`` list; no RViz changes needed.

The HUD owns the container style (font, colours, position, size); it reads only
each panel's ``.text`` field, and joins them verbatim.

The overlay plugin renders with ``QStaticText``, which auto-detects its format:
plain text unless the string carries HTML tags. Panels are plain by default and
separate lines with ``\\n``. A panel that wants per-line colour has to emit
``<span>`` markup, which flips the whole overlay to rich text -- and there ``\\n``
stops breaking lines and runs of spaces collapse. ``rich_text`` switches this node
to that mode: it joins with ``<br/>`` and sizes off ``<br/>``. Panels feeding a
``rich_text`` HUD must pad columns with ``&nbsp;``, not spaces.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from rviz_2d_overlay_msgs.msg import OverlayText


LINE_BREAK = '<br/>'

# Per-line height as a multiple of the font size, plus a constant. The plain
# figure is the long-standing one tuned against the plain-text path. Rich text
# lays out tighter -- measured at 1.71x the font size against the 1.8x + 4 the
# plain path assumes -- and reusing the plain figure leaves roughly a third of
# the panel empty below the last row.
PLAIN_LINE_FACTOR, PLAIN_LINE_PAD = 1.8, 4.0
RICH_LINE_FACTOR, RICH_LINE_PAD = 1.72, 0.0
PANEL_PADDING_PX = 16
RICH_PANEL_PADDING_PX = 8

_HORIZONTAL_ALIGNMENTS = {
    'left': OverlayText.LEFT,
    'right': OverlayText.RIGHT,
    'center': OverlayText.CENTER,
}
_VERTICAL_ALIGNMENTS = {
    'top': OverlayText.TOP,
    'bottom': OverlayText.BOTTOM,
    'center': OverlayText.CENTER,
}


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
        # Which corner to pin to. horizontal_distance/vertical_distance are
        # measured from whichever border the alignment selects, so the inset
        # works unchanged on any side.
        self.declare_parameter('horizontal_alignment', 'left')
        self.declare_parameter('vertical_alignment', 'top')
        self.declare_parameter('rich_text', False)

        self.panels = list(self.get_parameter('panels').value)
        marker_topic = self.get_parameter('marker_topic').value
        self.text_size = float(self.get_parameter('text_size').value)
        self.h_dist = int(self.get_parameter('horizontal_distance').value)
        self.v_dist = int(self.get_parameter('vertical_distance').value)
        self.width = int(self.get_parameter('overlay_width').value)
        self.font = self.get_parameter('font').value
        self.rich_text = bool(self.get_parameter('rich_text').value)
        self.h_align = self._alignment(
            'horizontal_alignment', _HORIZONTAL_ALIGNMENTS, OverlayText.LEFT)
        self.v_align = self._alignment(
            'vertical_alignment', _VERTICAL_ALIGNMENTS, OverlayText.TOP)
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

    def _alignment(self, parameter_name: str, options: dict, fallback: int) -> int:
        raw = str(self.get_parameter(parameter_name).value).strip().lower()
        if raw in options:
            return options[raw]
        self.get_logger().warn(
            f'Unknown {parameter_name} "{raw}"; expected one of '
            f'{", ".join(sorted(options))}. Falling back to the default.')
        return fallback

    def _on_panel(self, topic, msg):
        self._texts[topic] = msg.text or ''

    def _tick(self):
        sections = [self._texts[t] for t in self.panels if self._texts[t]]
        # In rich text a newline is whitespace, not a break, so a panel's own
        # line breaks have to be converted too -- not just the joins.
        separator = LINE_BREAK * 2 if self.rich_text else '\n\n'
        text = separator.join(sections)
        if self.rich_text:
            text = text.replace('\n', LINE_BREAK)

        # Auto-size height from the line count so panels can grow freely.
        # Under-counting the per-line height makes the deficit accumulate down
        # the panel and clips the lower rows, so the figure is per-render-mode
        # rather than one value that has to be safe for the looser of the two.
        breaks = text.count(LINE_BREAK) if self.rich_text else text.count('\n')
        line_count = breaks + 1 if text else 1
        if self.rich_text:
            line_px = self.text_size * RICH_LINE_FACTOR + RICH_LINE_PAD
            padding = RICH_PANEL_PADDING_PX
        else:
            line_px = self.text_size * PLAIN_LINE_FACTOR + PLAIN_LINE_PAD
            padding = PANEL_PADDING_PX
        height = int(line_count * line_px + padding)

        m = OverlayText()
        m.action = OverlayText.ADD
        m.width = self.width
        m.height = height
        m.horizontal_alignment = self.h_align
        m.vertical_alignment = self.v_align
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
