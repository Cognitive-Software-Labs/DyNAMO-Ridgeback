#!/usr/bin/env python3
"""Demo capture: drive a scripted loop, record camera frames, slam map,
GT + odom trajectories, scan snapshot. Saves PNGs to the scratchpad."""
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, LaserScan

OUT = Path(__file__).parent

# drive plan: (vx, vy, wz, seconds)
PLAN = [
    (0.4, 0.0, 0.0, 6.0),      # forward down the corridor
    (0.3, 0.0, 0.6, 5.0),      # sweeping left arc
    (0.0, 0.3, 0.0, 4.0),      # pure strafe (holonomic showoff)
    (0.3, 0.0, -0.6, 5.0),     # arc back right
    (0.0, 0.0, 0.9, 5.0),      # spin in place (fills the map around)
    (0.35, 0.0, 0.2, 8.0),     # cruise
    (0.0, 0.0, 0.0, 3.0),      # stop, settle
]


class Demo:
    def __init__(self, node):
        self.node = node
        self.map_msg = None
        self.frames = {}           # tag -> (stamp, Image)
        self.scan = None
        self.gt_path = []
        self.odom_path = []
        self.cmd_pub = node.create_publisher(TwistStamped, "cmd_vel", 10)
        node.create_subscription(OccupancyGrid, "map", self.on_map, 10)
        node.create_subscription(Image, "sensors/camera_0/color/image",
                                 self.on_img, 5)
        node.create_subscription(LaserScan, "sensors/lidar2d_0/scan",
                                 self.on_scan, 5)
        node.create_subscription(PoseStamped, "ground_truth/pose",
                                 self.on_gt, 20)
        node.create_subscription(Odometry, "platform/odom/filtered",
                                 self.on_odom, 20)
        self.want_frame = None

    def on_map(self, m):
        self.map_msg = m

    def on_img(self, m):
        if self.want_frame:
            self.frames[self.want_frame] = m
            self.want_frame = None

    def on_scan(self, m):
        self.scan = m

    def on_gt(self, m):
        self.gt_path.append((m.pose.position.x, m.pose.position.y))

    def on_odom(self, m):
        self.odom_path.append((m.pose.pose.position.x, m.pose.pose.position.y))

    def drive(self):
        for i, (vx, vy, wz, dur) in enumerate(PLAN):
            self.want_frame = f"seg{i}"
            t0 = time.time()
            while time.time() - t0 < dur:
                msg = TwistStamped()
                msg.twist.linear.x = vx
                msg.twist.linear.y = vy
                msg.twist.angular.z = wz
                self.cmd_pub.publish(msg)
                rclpy.spin_once(self.node, timeout_sec=0.05)
            print(f"segment {i} done ({vx},{vy},{wz}) x{dur}s", flush=True)
        # let slam integrate the last poses
        t0 = time.time()
        while time.time() - t0 < 4.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)


def save_camera(demo):
    from PIL import Image as PILImage
    saved = []
    for tag, m in demo.frames.items():
        if m.encoding != "rgb8":
            continue
        img = PILImage.frombytes("RGB", (m.width, m.height), bytes(m.data))
        p = OUT / f"demo_cam_{tag}.png"
        img.save(p)
        saved.append(p.name)
    print("camera frames:", saved, flush=True)


def save_map(demo):
    from PIL import Image as PILImage, ImageDraw
    m = demo.map_msg
    if m is None:
        print("NO MAP", flush=True)
        return
    w, h = m.info.width, m.info.height
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    px = bytearray(w * h)
    for i, c in enumerate(m.data):
        px[i] = 128 if c < 0 else (255 if c <= 50 else 0)
    img = PILImage.frombytes("L", (w, h), bytes(px)).convert("RGB")

    def to_px(pt):
        return ((pt[0] - ox) / res, (pt[1] - oy) / res)

    d = ImageDraw.Draw(img)
    if len(demo.gt_path) > 1:
        d.line([to_px(p) for p in demo.gt_path[::5]], fill=(0, 180, 0), width=2)
    if len(demo.odom_path) > 1:
        d.line([to_px(p) for p in demo.odom_path[::5]], fill=(220, 60, 60), width=1)
    # map frame y-up -> image y-down
    img = img.transpose(PILImage.FLIP_TOP_BOTTOM)
    img = img.resize((w * 3, h * 3), PILImage.NEAREST)
    p = OUT / "demo_map.png"
    img.save(p)
    print(f"map saved {w}x{h} -> {p.name}", flush=True)


def save_scan(demo):
    from PIL import Image as PILImage, ImageDraw
    s = demo.scan
    if s is None:
        return
    size = 640
    img = PILImage.new("RGB", (size, size), (16, 16, 24))
    d = ImageDraw.Draw(img)
    cx = cy = size // 2
    scale = (size / 2 - 10) / s.range_max
    d.ellipse([cx - 3, cy - 3, cx + 3, cy + 3], fill=(80, 160, 255))
    n_fin = 0
    for i, r in enumerate(s.ranges):
        if not (s.range_min <= r <= s.range_max) or math.isinf(r):
            continue
        n_fin += 1
        a = s.angle_min + i * s.angle_increment
        x = cx + r * math.cos(a) * scale
        y = cy - r * math.sin(a) * scale
        d.point((x, y), fill=(255, 120, 40))
    for rr in (2.5, 5.0, 7.5, 10.0):
        rp = rr * scale
        d.ellipse([cx - rp, cy - rp, cx + rp, cy + rp], outline=(40, 40, 60))
    p = OUT / "demo_scan.png"
    img.save(p)
    print(f"scan saved ({n_fin} finite pts) -> {p.name}", flush=True)


def main():
    rclpy.init()
    node = rclpy.create_node("demo_capture", namespace="r100_0001")
    demo = Demo(node)
    # wait for first map + image
    t0 = time.time()
    while (demo.map_msg is None) and time.time() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.2)
    demo.drive()
    save_camera(demo)
    save_map(demo)
    save_scan(demo)
    gt = demo.gt_path[-1] if demo.gt_path else (0, 0)
    od = demo.odom_path[-1] if demo.odom_path else (0, 0)
    err = math.hypot(gt[0] - od[0], gt[1] - od[1])
    print(f"final GT ({gt[0]:.3f},{gt[1]:.3f}) vs EKF ({od[0]:.3f},{od[1]:.3f})"
          f" err {err*100:.1f} cm over {len(demo.gt_path)} samples", flush=True)
    print("DEMO CAPTURE DONE", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
