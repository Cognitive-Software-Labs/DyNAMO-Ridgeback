#!/usr/bin/env python3
"""Publish a live exploration-coverage HUD panel.

Subscribes to the SLAM occupancy grid, compares it against a hand-authored
ground-truth map for the current world (installed under
``share/ridgeback_autonomy/sim/ground_truth_maps/``), and publishes a compact
text panel (``rviz_2d_overlay_msgs/OverlayText``) on ``hud/coverage`` for the HUD
aggregator to render.

Two numbers are reported (see ``sim/ground_truth_maps/README.md``):
  * complete  — slam_found_free / all gt-free cells (how much is discovered)
  * accuracy  — coverage_pct, over the explored gt-free area only

Worlds without a ground-truth map (e.g. ``hospital``) just show ``n/a`` — the
node never crashes.
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from rviz_2d_overlay_msgs.msg import OverlayText

from ridgeback_autonomy.common.coverage_utils import (
    align_grids,
    compute_stats,
    occupancy_msg_to_grid,
    pgm_to_grid,
)

# Non-breaking space: the overlay renders as HTML, which collapses runs of
# normal spaces and breaks the monospace columns.
NBSP = ' '


class CoverageOverlayNode(Node):
    def __init__(self):
        super().__init__('coverage_overlay_node')

        self.declare_parameter('world', '')
        self.declare_parameter('ground_truth_dir', '')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('panel_topic', 'hud/coverage')
        self.declare_parameter('publish_rate_hz', 1.0)

        self.world = self.get_parameter('world').value or ''
        gt_dir = self.get_parameter('ground_truth_dir').value or ''
        if not gt_dir:
            # Default to the maps installed with the package, so this works on
            # any machine (not just the source tree).
            gt_dir = os.path.join(
                get_package_share_directory('ridgeback_autonomy'),
                'sim', 'ground_truth_maps')
        map_topic = self.get_parameter('map_topic').value
        panel_topic = self.get_parameter('panel_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        # Load the ground-truth map once (cached). Missing GT is not fatal.
        self._gt_grid = None
        self._gt_meta = None
        self._gt_reason = ''
        gt_pgm = os.path.join(gt_dir, f'{self.world}.pgm') if gt_dir and self.world else ''
        if not gt_pgm:
            self._gt_reason = 'no ground truth dir/world set'
        elif not os.path.exists(gt_pgm):
            self._gt_reason = f'no ground truth for {self.world}'
        else:
            try:
                self._gt_grid, self._gt_meta = pgm_to_grid(gt_pgm)
                self.get_logger().info(f'Loaded ground truth: {gt_pgm}')
            except Exception as exc:  # noqa: BLE001 - report, never crash the HUD
                self._gt_reason = 'ground truth load failed'
                self.get_logger().error(f'Failed to load ground truth {gt_pgm}: {exc}')

        self._latest_map = None

        # SLAM latches the map with transient-local reliable QoS.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)

        self.pub = self.create_publisher(OverlayText, panel_topic, 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    def _on_map(self, msg):
        self._latest_map = msg

    def _tick(self):
        m = OverlayText()
        m.action = OverlayText.ADD
        m.text = self._panel_text()
        self.pub.publish(m)

    def _panel_text(self):
        title = f'COVERAGE  ({self.world})' if self.world else 'COVERAGE'

        if self._gt_grid is None:
            return self._pad(f'{title}\n n/a ({self._gt_reason})')

        if self._latest_map is None:
            return self._pad(f'{title}\n waiting for map...')

        slam_grid, slam_meta = occupancy_msg_to_grid(self._latest_map)
        slam_crop, gt_crop = align_grids(slam_grid, slam_meta, self._gt_grid, self._gt_meta)
        if slam_crop is None:
            return self._pad(f'{title}\n no map/ground-truth overlap yet')

        s = compute_stats(slam_crop, gt_crop)
        total = s['gt_free_cells']
        found = s['slam_found_free']
        complete = found / total * 100 if total else 0.0

        lines = [
            title,
            f' complete   {complete:5.1f}%',
            f' accuracy   {s["coverage_pct"]:5.1f}%',
            f' found      {self._k(found)} / {self._k(total)}',
            f' unexplored {self._k(s["unexplored_cells"])}',
        ]
        return self._pad('\n'.join(lines))

    @staticmethod
    def _k(n):
        return f'{n / 1000:.0f}k' if n >= 1000 else str(int(n))

    @staticmethod
    def _pad(text):
        return text.replace(' ', NBSP)


def main():
    rclpy.init()
    node = CoverageOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
