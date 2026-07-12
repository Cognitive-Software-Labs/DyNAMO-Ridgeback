#!/usr/bin/env python3
"""SLAM quality probe: drive a closed loop, score the run against analytic GT.

Drives a GT-pose-feedback waypoint loop through mock_hospital that RETURNS TO
START — the scripted demo drive never revisited mapped space, so loop closure
was never actually exercised and "map looks crisp" stayed subjective. This
harness makes it numeric:

  - pose_rmse_m / pose_max_m   slam estimate (map->base_link TF) vs GT pose
  - loop_error_*               slam-vs-GT error over the final 10 s back at
                               the start pose — the end-at-start closure figure
  - map_iou / wall_precision / wall_recall
                               final /map occupied cells vs the analytic GT
                               grid from gt_occupancy.py (GT dilated 1 cell
                               for the 0.05 m discretization; include
                               footprints masked out)
  - closures_accepted/rejected karto's "Closing loop..." / "REJECTED!" lines
                               from the slam launch log (--slam-log)
  - tf_correction_events       map->odom jumps > 3 cm / 0.015 rad between
                               consecutive 10 Hz samples — proxy for graph
                               corrections, independent of log availability

Artifacts per --tag: <out>/<tag>_metrics.json + <tag>_overlay.png (slam map,
GT walls in blue, GT trajectory green, slam trajectory red).

Run under a sourced workspace against a live sim + slam stack:

    python3 tools/isaac/slam_quality_probe.py --tag B \
        --gt-grid <gt_hospital.npz> --slam-log <slam_launch.log> --out <dir>

Rotation is capped at WZ_MAX=0.4 rad/s (frame-quantized sensor pose smears
scale with wz; nav-realistic rates are the regime under test). Waypoints are
clearance-checked against the GT grid before the first command is sent.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
import tf2_ros

sys.path.insert(0, str(Path(__file__).parent))
from gt_occupancy import check_waypoints, load_grid  # noqa: E402

# Closed loop in mock_hospital known-free space: east down the corridor,
# through the x=11.8 north door into the NE room, a westward sweep inside,
# back out and west along the corridor to the spawn pose. Verified >=0.8 m
# clearance against the SDF wall boxes (actual minimum 1.33 m).
WAYPOINTS = [
    (9.0, 0.0), (11.8, 0.0), (11.8, 3.3), (9.0, 3.4),
    (11.8, 3.3), (11.8, 0.0), (2.0, 0.0), (0.0, 0.0),
]
FINAL_YAW = 0.0          # face the original heading again
VX_MAX = 0.35
WZ_MAX = 0.4
ARRIVE_M = 0.15
ALIGN_RAD = 0.05
SETTLE_S = 12.0          # stationary tail; loop error scored on last 10 s
LOOP_WINDOW_S = 10.0
JUMP_TRANS_M = 0.03      # map->odom deltas above these count as corrections
JUMP_YAW_RAD = 0.015


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_norm(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Probe:
    def __init__(self, node):
        self.node = node
        self.gt = None                   # latest (t, x, y, yaw)
        self.gt_path = []
        self.slam_path = []              # (t, x, y, yaw) from map->base_link
        self.odom_corr = []              # (t, x, y, yaw) map->odom
        self.map_msg = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(
            seconds=30.0))
        self.cmd_pub = node.create_publisher(TwistStamped, "cmd_vel", 10)
        node.create_subscription(PoseStamped, "ground_truth/pose",
                                 self.on_gt, 20)
        node.create_subscription(OccupancyGrid, "map", self.on_map, 10)
        # tf2_ros.TransformListener hardcodes absolute /tf; our TF flows on
        # the robot namespace, so feed the buffer from relative topics.
        node.create_subscription(TFMessage, "tf", self.on_tf, 100)
        node.create_subscription(
            TFMessage, "tf_static", self.on_tf_static,
            rclpy.qos.QoSProfile(
                depth=10,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

    def on_gt(self, m):
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        self.gt = (t, m.pose.position.x, m.pose.position.y,
                   yaw_of(m.pose.orientation))
        self.gt_path.append(self.gt)

    def on_map(self, m):
        self.map_msg = m

    def on_tf(self, m):
        for tr in m.transforms:
            self.tf_buffer.set_transform(tr, "probe")

    def on_tf_static(self, m):
        for tr in m.transforms:
            self.tf_buffer.set_transform_static(tr, "probe")

    def now_s(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def lookup(self, parent, child):
        try:
            tr = self.tf_buffer.lookup_transform(parent, child, Time())
        except tf2_ros.TransformException:
            return None
        tt = tr.transform.translation
        return (Time.from_msg(tr.header.stamp).nanoseconds * 1e-9,
                tt.x, tt.y, yaw_of(tr.transform.rotation))

    def sample_slam(self):
        est = self.lookup("map", "base_link")
        if est is not None:
            self.slam_path.append((self.now_s(),) + est[1:])
        corr = self.lookup("map", "odom")
        if corr is not None:
            if not self.odom_corr or corr[1:] != self.odom_corr[-1][1:]:
                self.odom_corr.append((self.now_s(),) + corr[1:])

    def send(self, vx, wz):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.angular.z = wz
        self.cmd_pub.publish(msg)

    # -- control -------------------------------------------------------------

    def spin_for(self, sim_seconds, sample=True):
        t0 = self.now_s()
        last = t0
        while self.now_s() - t0 < sim_seconds:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if sample and self.now_s() - last >= 0.1:
                last = self.now_s()
                self.sample_slam()

    def rotate_to(self, target_yaw, timeout_s=40.0):
        t0 = self.now_s()
        last_sample = t0
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.gt is None:
                continue
            err = ang_norm(target_yaw - self.gt[3])
            if abs(err) < ALIGN_RAD:
                self.send(0.0, 0.0)
                return True
            if self.now_s() - t0 > timeout_s:
                return False
            self.send(0.0, max(-WZ_MAX, min(WZ_MAX, 2.0 * err)))
            if self.now_s() - last_sample >= 0.1:
                last_sample = self.now_s()
                self.sample_slam()

    def drive_to(self, wx, wy):
        dist0 = math.hypot(wx - self.gt[1], wy - self.gt[2])
        heading = math.atan2(wy - self.gt[2], wx - self.gt[1])
        if not self.rotate_to(heading):
            return False, "rotate timeout"
        t0 = self.now_s()
        timeout = 30.0 + 4.0 * dist0
        last_sample = t0
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            dx, dy = wx - self.gt[1], wy - self.gt[2]
            dist = math.hypot(dx, dy)
            if dist < ARRIVE_M:
                self.send(0.0, 0.0)
                return True, ""
            if self.now_s() - t0 > timeout:
                self.send(0.0, 0.0)
                return False, f"drive timeout at dist {dist:.2f}"
            err = ang_norm(math.atan2(dy, dx) - self.gt[3])
            vx = 0.1 if abs(err) > 0.5 else min(VX_MAX, 0.3 + 0.8 * dist)
            self.send(vx, max(-WZ_MAX, min(WZ_MAX, 1.5 * err)))
            if self.now_s() - last_sample >= 0.1:
                last_sample = self.now_s()
                self.sample_slam()


# -- metrics ------------------------------------------------------------------

def dilate(mask):
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    out[1:, 1:] |= mask[:-1, :-1]
    out[1:, :-1] |= mask[:-1, 1:]
    out[:-1, 1:] |= mask[1:, :-1]
    out[:-1, :-1] |= mask[1:, 1:]
    return out


def pose_errors(slam_path, gt_path):
    """Planar error of each slam sample vs time-interpolated GT."""
    if not slam_path or len(gt_path) < 2:
        return np.array([])
    gt = np.array(gt_path)          # columns: t, x, y, yaw
    errs = []
    for t, x, y, _ in slam_path:
        if t < gt[0, 0] or t > gt[-1, 0]:
            continue
        gx = np.interp(t, gt[:, 0], gt[:, 1])
        gy = np.interp(t, gt[:, 0], gt[:, 2])
        errs.append(math.hypot(x - gx, y - gy))
    return np.array(errs)


def map_metrics(map_msg, gt_grid, gt_ignore, gt_origin, gt_res):
    """Resample the slam map onto the GT lattice, compare occupied cells.

    GT walls are dilated one cell (0.05 m discretization slack); include
    footprints and never-observed cells are excluded from every count.
    """
    info = map_msg.info
    data = np.array(map_msg.data, dtype=np.int8).reshape(
        info.height, info.width)
    ny, nx = gt_grid.shape
    # GT cell centers -> slam map indices (map frame == world frame: slam
    # starts at the spawn pose, which is the world origin with yaw 0)
    xs = gt_origin[0] + (np.arange(nx) + 0.5) * gt_res
    ys = gt_origin[1] + (np.arange(ny) + 0.5) * gt_res
    ix = np.floor((xs - info.origin.position.x) / info.resolution).astype(int)
    iy = np.floor((ys - info.origin.position.y) / info.resolution).astype(int)
    in_x = (ix >= 0) & (ix < info.width)
    in_y = (iy >= 0) & (iy < info.height)
    inside = in_y[:, None] & in_x[None, :]
    vals = np.full((ny, nx), -1, dtype=np.int8)
    vals[inside] = data[np.clip(iy, 0, info.height - 1)[:, None],
                        np.clip(ix, 0, info.width - 1)[None, :]][inside]

    observed = vals >= 0
    slam_occ = vals > 50
    gt_occ = gt_grid == 100
    valid = observed & ~gt_ignore
    gt_d = dilate(gt_occ)
    slam_d = dilate(slam_occ)

    s = slam_occ & valid
    g = gt_occ & valid
    inter = (slam_occ & gt_d & valid).sum()
    union = ((slam_occ | gt_occ) & valid).sum()
    return {
        "map_iou": round(float(inter / union), 4) if union else 0.0,
        "wall_precision": round(float((s & gt_d).sum() / s.sum()), 4)
                          if s.sum() else 0.0,
        "wall_recall": round(float((g & slam_d).sum() / g.sum()), 4)
                       if g.sum() else 0.0,
        "slam_occupied_cells": int(s.sum()),
        "gt_occupied_cells_observed": int(g.sum()),
        "observed_cells": int(valid.sum()),
    }


def closure_counts(log_path):
    if not log_path or not Path(log_path).exists():
        return None
    text = Path(log_path).read_text(errors="replace")
    begins = len(re.findall(r"Closing loop", text))
    rejected = len(re.findall(r"REJECTED!", text))
    return {"closures_accepted": max(0, begins - rejected),
            "closures_rejected": rejected}


def correction_events(odom_corr):
    events = 0
    max_jump = 0.0
    for a, b in zip(odom_corr, odom_corr[1:]):
        d = math.hypot(b[1] - a[1], b[2] - a[2])
        if d > JUMP_TRANS_M or abs(ang_norm(b[3] - a[3])) > JUMP_YAW_RAD:
            events += 1
            max_jump = max(max_jump, d)
    return events, max_jump


# -- rendering ----------------------------------------------------------------

def render_overlay(map_msg, gt_grid, gt_ignore, gt_origin, gt_res,
                   gt_path, slam_path, out_png, scale=3):
    from PIL import Image, ImageDraw
    info = map_msg.info
    w, h = info.width, info.height
    data = np.array(map_msg.data, dtype=np.int8).reshape(h, w)
    img = np.full((h, w, 3), 128, dtype=np.uint8)
    img[(data >= 0) & (data <= 50)] = (255, 255, 255)
    img[data > 50] = (0, 0, 0)

    # GT walls in translucent blue on the slam map lattice
    ny, nx = gt_grid.shape
    oy_i, ox_i = np.nonzero((gt_grid == 100) & ~gt_ignore)
    gx = gt_origin[0] + (ox_i + 0.5) * gt_res
    gy = gt_origin[1] + (oy_i + 0.5) * gt_res
    mx = np.floor((gx - info.origin.position.x) / info.resolution).astype(int)
    my = np.floor((gy - info.origin.position.y) / info.resolution).astype(int)
    ok = (mx >= 0) & (mx < w) & (my >= 0) & (my < h)
    img[my[ok], mx[ok]] = (img[my[ok], mx[ok]] * 0.4 +
                           np.array((70, 110, 255)) * 0.6).astype(np.uint8)

    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)

    def to_px(x, y):
        return ((x - info.origin.position.x) / info.resolution,
                (y - info.origin.position.y) / info.resolution)

    if len(gt_path) > 1:
        d.line([to_px(p[1], p[2]) for p in gt_path[::5]],
               fill=(0, 180, 0), width=2)
    if len(slam_path) > 1:
        d.line([to_px(p[1], p[2]) for p in slam_path[::2]],
               fill=(220, 40, 40), width=1)
    for wx, wy in WAYPOINTS:
        px, py = to_px(wx, wy)
        d.ellipse([px - 2, py - 2, px + 2, py + 2], outline=(255, 140, 0))

    im = im.transpose(Image.FLIP_TOP_BOTTOM)
    im = im.resize((w * scale, h * scale), Image.NEAREST)
    im.save(out_png)


# -- main ---------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tag", required=True)
    ap.add_argument("--gt-grid", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--slam-log", type=Path, default=None)
    ap.add_argument("--namespace", default="r100_0001")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    gt_grid, gt_ignore, gt_origin, gt_res = load_grid(args.gt_grid)
    problems = check_waypoints(gt_grid, gt_origin, gt_res,
                               [(0.0, 0.0)] + WAYPOINTS, clearance=0.8)
    if problems:
        print("WAYPOINT CHECK FAILED:", *problems, sep="\n  ")
        return 2

    rclpy.init()
    node = rclpy.create_node(
        "slam_quality_probe", namespace=args.namespace,
        parameter_overrides=[rclpy.parameter.Parameter(
            "use_sim_time", value=True)])
    probe = Probe(node)

    print("waiting for GT pose, map and map->base_link TF...", flush=True)
    t0 = time.time()
    while time.time() - t0 < 180:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (probe.gt is not None and probe.map_msg is not None
                and probe.lookup("map", "base_link") is not None):
            break
    else:
        print("PROBE ABORT: stack never became ready", flush=True)
        return 3

    metrics = {"tag": args.tag, "waypoints": WAYPOINTS,
               "wz_max": WZ_MAX, "vx_max": VX_MAX,
               "aborted": False, "abort_reason": ""}
    drive_t0 = probe.now_s()
    probe.gt_path.clear()

    for i, (wx, wy) in enumerate(WAYPOINTS):
        ok, why = probe.drive_to(wx, wy)
        print(f"waypoint {i} ({wx},{wy}): {'ok' if ok else why} "
              f"sim_t={probe.now_s() - drive_t0:.1f}s", flush=True)
        if not ok:
            metrics["aborted"] = True
            metrics["abort_reason"] = f"waypoint {i}: {why}"
            break
    if not metrics["aborted"]:
        if not probe.rotate_to(FINAL_YAW):
            metrics["aborted"] = True
            metrics["abort_reason"] = "final align timeout"
    probe.send(0.0, 0.0)
    settle_t0 = probe.now_s()
    probe.spin_for(SETTLE_S)
    metrics["drive_duration_s"] = round(probe.now_s() - drive_t0, 1)

    errs = pose_errors(probe.slam_path, probe.gt_path)
    metrics["n_pose_samples"] = int(errs.size)
    metrics["pose_rmse_m"] = round(float(np.sqrt(np.mean(errs ** 2))), 4) \
        if errs.size else None
    metrics["pose_max_m"] = round(float(errs.max()), 4) if errs.size else None

    tail = [p for p in probe.slam_path
            if p[0] >= settle_t0 + SETTLE_S - LOOP_WINDOW_S]
    tail_errs = pose_errors(tail, probe.gt_path)
    metrics["loop_error_mean_m"] = round(float(tail_errs.mean()), 4) \
        if tail_errs.size else None
    metrics["loop_error_max_m"] = round(float(tail_errs.max()), 4) \
        if tail_errs.size else None

    if probe.map_msg is not None:
        metrics.update(map_metrics(probe.map_msg, gt_grid, gt_ignore,
                                   gt_origin, gt_res))
        render_overlay(probe.map_msg, gt_grid, gt_ignore, gt_origin, gt_res,
                       probe.gt_path, probe.slam_path,
                       args.out / f"{args.tag}_overlay.png")
    closures = closure_counts(args.slam_log)
    metrics["closure_log_available"] = closures is not None
    if closures:
        metrics.update(closures)
    ev, max_jump = correction_events(probe.odom_corr)
    metrics["tf_correction_events"] = ev
    metrics["tf_max_jump_m"] = round(max_jump, 4)

    out_json = args.out / f"{args.tag}_metrics.json"
    out_json.write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2), flush=True)
    print(f"PROBE DONE -> {out_json}", flush=True)
    rclpy.shutdown()
    return 1 if metrics["aborted"] else 0


if __name__ == "__main__":
    sys.exit(main())
