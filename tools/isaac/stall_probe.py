#!/usr/bin/env python3
"""Diagnose the collision_monitor stall (docs/isaac/open-issues.md §1).

Answers, in one attach, the questions that section lists as untried:

1. What is `local_costmap/published_footprint` ACTUALLY publishing? Every
   "the returns are inside the footprint" claim on record compared against an
   *assumed* 0.9325 x 0.7932 hull; the topic has never been read.
2. What does `collision_monitor/footprint_approach` project? The polygon is
   `action_type: approach`, so it is swept forward along cmd_vel for
   `time_before_collision` -- "inside the footprint" is not the static hull.
3. Do the near returns exist BEFORE the assembler? The raw bridge clouds
   (`points`, `points_l`) are reported per half-arc prim, so a phantom that
   only appears post-binning separates an assembler bug from a sensor one.
4. Is the cmd_vel chain breaking where §1 says it is?

Reports scan hits in the SENSOR frame (range/bearing, TF-independent) and,
when `tf_static` resolves, in `base_link`, so a hit can be tested against the
published hull rather than an assumed one.

Usage (workspace sourced, matching ROS_DOMAIN_ID/RMW as the stack):
    python3 tools/isaac/stall_probe.py [--namespace r100_0001]
                                       [--seconds 25] [--near 1.0]
"""
from __future__ import annotations

import argparse
import math
from collections import defaultdict

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PolygonStamped, Twist, TwistStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry

# The hull every previous claim assumed, for side-by-side comparison.
ASSUMED_HULL = (0.9325, 0.7932)


def poly_extent(points):
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    return min(xs), max(xs), min(ys), max(ys)


def point_in_poly(px, py, points):
    """Standard ray-crossing test; polygon is a list of geometry_msgs Point32."""
    inside = False
    n = len(points)
    for i in range(n):
        x1, y1 = points[i].x, points[i].y
        x2, y2 = points[(i + 1) % n].x, points[(i + 1) % n].y
        if (y1 > py) != (y2 > py):
            xint = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if px < xint:
                inside = not inside
    return inside


class StallProbe(Node):
    def __init__(self, ns: str, near: float):
        super().__init__("stall_probe", namespace=ns,
                         cli_args=["--ros-args", "-r", "/tf:=tf",
                                   "-r", "/tf_static:=tf_static"])
        self.near = near
        self.counts = defaultdict(int)
        self.footprints = {}        # topic -> last polygon (points list)
        self.footprint_frames = {}
        self.scan_hits = {}         # topic -> list of (bin, deg, range)
        self.scan_frames = {}
        self.cloud_hits = {}        # topic -> list of (deg, range)
        self.cmd_last = {}
        self.odom_first = None
        self.odom_last = None

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        for topic in ("local_costmap/published_footprint",
                      "global_costmap/published_footprint",
                      "collision_monitor/footprint_approach"):
            # published_footprint is volatile, the monitor's viz polygon too;
            # subscribe twice rather than guess the durability.
            self.create_subscription(
                PolygonStamped, topic,
                lambda m, t=topic: self._on_poly(t, m), 10)
            self.create_subscription(
                PolygonStamped, topic,
                lambda m, t=topic: self._on_poly(t, m), latched)

        for i in (0, 1):
            self.create_subscription(
                LaserScan, f"sensors/lidar2d_{i}/scan",
                lambda m, t=f"lidar2d_{i}/scan": self._on_scan(t, m), 10)
            for suffix in ("points", "points_l"):
                topic = f"sensors/lidar2d_{i}/{suffix}"
                self.create_subscription(
                    PointCloud2, topic,
                    lambda m, t=topic: self._on_cloud(t, m), 10)

        for topic in ("cmd_vel_nav", "cmd_vel_smoothed", "cmd_vel"):
            self.create_subscription(
                TwistStamped, topic,
                lambda m, t=topic: self._on_cmd(t, m.twist), 10)
            self.create_subscription(
                Twist, topic,
                lambda m, t=topic + "(plain)": self._on_cmd(t, m), 10)

        self.create_subscription(
            Odometry, "platform/odom/filtered", self._on_odom, 10)

    # --- callbacks -------------------------------------------------------
    def _on_poly(self, topic, msg):
        self.counts[topic] += 1
        if msg.polygon.points:
            self.footprints[topic] = list(msg.polygon.points)
            self.footprint_frames[topic] = msg.header.frame_id

    def _on_scan(self, topic, msg):
        self.counts[topic] += 1
        r = np.asarray(msg.ranges, dtype=np.float64)
        idx = np.nonzero(np.isfinite(r) & (r > 0.0) & (r < self.near))[0]
        if idx.size:
            hits = [(int(i),
                     math.degrees(msg.angle_min + i * msg.angle_increment),
                     float(r[i])) for i in idx]
            # keep the worst (most populated) frame seen, that is the one
            # tripping min_points
            prev = self.scan_hits.get(topic)
            if prev is None or len(hits) > len(prev):
                self.scan_hits[topic] = hits
        elif topic not in self.scan_hits:
            self.scan_hits[topic] = []
        self.scan_frames[topic] = msg.header.frame_id

    def _on_cloud(self, topic, msg):
        self.counts[topic] += 1
        pts = pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        x = np.asarray(pts["x"], dtype=np.float64)
        y = np.asarray(pts["y"], dtype=np.float64)
        if x.size == 0:
            return
        rr = np.hypot(x, y)
        # mirror the assembler's invalid-slot filter (ros_io.py: r > 0.03)
        m = (rr > 0.03) & (rr < self.near)
        if m.any():
            hits = [(math.degrees(math.atan2(yy, xx)), float(d))
                    for xx, yy, d in zip(x[m], y[m], rr[m])]
            prev = self.cloud_hits.get(topic)
            if prev is None or len(hits) > len(prev):
                self.cloud_hits[topic] = hits
        elif topic not in self.cloud_hits:
            self.cloud_hits[topic] = []

    def _on_cmd(self, topic, tw):
        self.counts[topic] += 1
        self.cmd_last[topic] = (tw.linear.x, tw.linear.y, tw.angular.z)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if self.odom_first is None:
            self.odom_first = (p.x, p.y)
        self.odom_last = (p.x, p.y)

def lidar_offset(index):
    """Static front/rear lidar pose in base_link, per docs/isaac/robot-model.md
    (commit a33111c2: parent base_link, xyz [+-0.3922, 0, 0.179], rear yaw pi).
    Used only as a fallback when tf_static has not been captured."""
    return (0.3922, 0.0, 0.0) if index == 0 else (-0.3922, 0.0, math.pi)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="r100_0001")
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--near", type=float, default=1.0,
                    help="report returns closer than this (m)")
    args = ap.parse_args()

    rclpy.init()
    node = StallProbe(args.namespace, args.near)
    start = node.get_clock().now().nanoseconds
    try:
        while (node.get_clock().now().nanoseconds - start) < args.seconds * 1e9:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    print("\n================ STALL PROBE ================")
    print(f"namespace /{args.namespace}   window {args.seconds:.0f}s wall"
          f"   near threshold {args.near} m\n")

    print("-- message counts --")
    for topic in sorted(node.counts):
        print(f"  {topic:48s} {node.counts[topic]:6d}")
    if not node.counts:
        print("  (nothing received -- check ROS_DOMAIN_ID / RMW / daemon)")

    print("\n-- published footprint (THE test that was skipped) --")
    if not node.footprints:
        print("  NO footprint received on any topic")
    for topic, pts in node.footprints.items():
        xmin, xmax, ymin, ymax = poly_extent(pts)
        frame = node.footprint_frames.get(topic, "?")
        print(f"  {topic}  frame={frame}  {len(pts)} vertices")
        print(f"    x [{xmin:+.4f}, {xmax:+.4f}]  ->  length {xmax - xmin:.4f}")
        print(f"    y [{ymin:+.4f}, {ymax:+.4f}]  ->  width  {ymax - ymin:.4f}")
        print(f"    assumed hull was {ASSUMED_HULL[0]:.4f} x "
              f"{ASSUMED_HULL[1]:.4f}"
              f"   delta {(xmax - xmin) - ASSUMED_HULL[0]:+.4f} x "
              f"{(ymax - ymin) - ASSUMED_HULL[1]:+.4f}")
        print("    vertices: "
              + " ".join(f"({p.x:+.3f},{p.y:+.3f})" for p in pts))

    print("\n-- near returns, assembled scans (sensor frame) --")
    for topic in sorted(node.scan_hits):
        hits = node.scan_hits[topic]
        frame = node.scan_frames.get(topic, "?")
        print(f"  {topic}  frame={frame}  worst frame: {len(hits)} hits "
              f"< {args.near} m")
        for b, deg, rng in hits[:40]:
            idx = 0 if "lidar2d_0" in topic else 1
            ox, oy, oyaw = lidar_offset(idx)
            th = math.radians(deg) + oyaw
            bx = ox + rng * math.cos(th)
            by = oy + rng * math.sin(th)
            marks = []
            # Compare against the base_link-frame polygon only. The costmaps
            # publish their footprint in odom/map, and testing a base_link
            # point against those is only accidentally right while the robot
            # sits at the origin with zero yaw — which is exactly the stalled
            # case, so the bug hides itself.
            fp = node.footprints.get("collision_monitor/footprint_approach")
            if fp:
                marks.append("IN-FOOTPRINT" if point_in_poly(bx, by, fp)
                             else "outside-footprint")
            print(f"    bin {b:4d}  {deg:+8.2f} deg  r={rng:.4f}  "
                  f"base_link=({bx:+.4f},{by:+.4f})  {' '.join(marks)}")
        if len(hits) > 40:
            print(f"    ... {len(hits) - 40} more")

    print("\n-- near returns, RAW bridge clouds (pre-assembler) --")
    if not node.cloud_hits:
        print("  no clouds received")
    for topic in sorted(node.cloud_hits):
        hits = node.cloud_hits[topic]
        print(f"  {topic}: worst frame {len(hits)} hits < {args.near} m")
        for deg, rng in hits[:20]:
            print(f"    {deg:+8.2f} deg  r={rng:.4f}")
        if len(hits) > 20:
            print(f"    ... {len(hits) - 20} more")

    print("\n-- cmd_vel chain --")
    for topic in ("cmd_vel_nav", "cmd_vel_smoothed", "cmd_vel"):
        n = node.counts.get(topic, 0) + node.counts.get(topic + "(plain)", 0)
        last = node.cmd_last.get(topic) or node.cmd_last.get(topic + "(plain)")
        last_s = (f"last=({last[0]:+.3f},{last[1]:+.3f},{last[2]:+.3f})"
                  if last else "last=none")
        print(f"  {topic:20s} {n:5d} msgs   {last_s}")

    if node.odom_first and node.odom_last:
        dx = node.odom_last[0] - node.odom_first[0]
        dy = node.odom_last[1] - node.odom_first[1]
        print(f"\n-- odom displacement: {math.hypot(dx, dy):.4f} m "
              f"(dx {dx:+.4f}, dy {dy:+.4f})")
    else:
        print("\n-- odom displacement: no odometry received")
    print("=============================================\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
