#!/usr/bin/env python3
"""Scan-vs-GT geometry check: static fit at current yaw, then spin and
measure scan-content rotation rate vs GT yaw rate. PASS = static median
fit error ~ rangeAccuracy (<=0.08 m) and shift ratio ~ -1.0."""
import math, sys, time
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TwistStamped

sys.path.insert(0, "tools/isaac")
from gt_occupancy import load_grid

GT_NPZ = sys.argv[1]
SPIN_S = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0

grid, ignore, origin, res = load_grid(GT_NPZ)
occ = grid == 100

def raycast(px, py, a):
    dx, dy = math.cos(a), math.sin(a)
    r = 0.06
    while r < 10.2:
        x, y = px + r * dx, py + r * dy
        ix = int((x - origin[0]) / res); iy = int((y - origin[1]) / res)
        if 0 <= ix < occ.shape[1] and 0 <= iy < occ.shape[0] and occ[iy, ix]:
            return r
        r += 0.02
    return float('inf')

rclpy.init()
n = rclpy.create_node("scan_geom_check", namespace="r100_0001")
scans, gts = [], []
meta = {}
def on_scan(m):
    meta['angle_min'] = m.angle_min
    meta['angle_inc'] = m.angle_increment
    scans.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                  np.array(m.ranges, dtype=float)))
n.create_subscription(LaserScan, "sensors/lidar2d_0/scan", on_scan, 50)
def on_gt(m):
    q = m.pose.orientation
    gts.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                m.pose.position.x, m.pose.position.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
n.create_subscription(PoseStamped, "ground_truth/pose", on_gt, 50)
pub = n.create_publisher(TwistStamped, "cmd_vel", 10)

t0 = time.time()
while (not scans or not gts) and time.time() - t0 < 60:
    rclpy.spin_once(n, timeout_sec=0.1)
if not scans:
    print("NO SCANS"); sys.exit(1)

# --- static fit ------------------------------------------------------------
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.05)
st, ranges = scans[-1]
for st_c, r_c in reversed(scans):
    if (np.isfinite(r_c) & (r_c > 0.05)).sum() > 200:
        st, ranges = st_c, r_c
        break
_, px, py, yw = gts[-1]
lx = px + 0.3922*math.cos(yw); ly = py + 0.3922*math.sin(yw)
inc = meta['angle_inc']
a0 = meta['angle_min']
finite = np.isfinite(ranges) & (ranges > 0.05)
idx = np.nonzero(finite)[0][::4]
errs = []
for i in idx:
    a = a0 + i*inc
    r_exp = raycast(lx, ly, yw + a)
    if math.isinf(r_exp): continue
    errs.append(abs(ranges[i] - r_exp))
print(f"n_bins={len(ranges)} angle_min={a0:.4f} inc={inc:.6f} "
      f"finite={finite.sum()} ({100*finite.sum()/len(ranges):.0f}%)")
print(f"STATIC yaw={yw:.3f}: median_fit_err={np.median(errs):.3f} m "
      f"(n={len(errs)})")

# --- spin ------------------------------------------------------------------
scans.clear(); gts.clear()
t0 = time.time()
while time.time() - t0 < SPIN_S:
    m = TwistStamped(); m.twist.angular.z = 0.3
    pub.publish(m); rclpy.spin_once(n, timeout_sec=0.02)
pub.publish(TwistStamped())

g = np.array(gts); yaw_u = np.unwrap(g[:, 3])
def prep(r):
    r = r.copy(); bad = ~np.isfinite(r); r[bad] = 0.0
    return r, ~bad
ratios = []
inc = meta['angle_inc']
pairs = 0
for i in range(0, len(scans) - 1, max(1, len(scans)//8)):
    for j in range(i + 1, len(scans)):
        if scans[j][0] - scans[i][0] >= 1.0:
            (t1, r1), (t2, r2) = scans[i], scans[j]
            if t1 < g[0,0] or t2 > g[-1,0]: break
            y1 = np.interp(t1, g[:,0], yaw_u); y2 = np.interp(t2, g[:,0], yaw_u)
            dyaw = y2 - y1
            if abs(dyaw) < 0.1: break
            a1, m1 = prep(r1); a2, m2 = prep(r2)
            best, bs = -1e9, 0
            for s in range(-int(0.6/inc), int(0.6/inc) + 1):
                a2s = np.roll(a2, s); m2s = np.roll(m2, s)
                if s > 0: m2s[:s] = False       # non-wrapping array:
                elif s < 0: m2s[s:] = False     # kill rolled-in bins
                mm = m1 & m2s
                if mm.sum() < 100: continue
                score = -np.median(np.abs(a1[mm] - a2s[mm]))
                if score > best: best, bs = score, s
            ratios.append(bs*inc/dyaw)
            pairs += 1
            break
    if pairs >= 6: break
print("SPIN shift/dyaw ratios:", [round(x, 3) for x in ratios])
print("EXPECT +1.0 (align-shift equals dyaw); -1.33 = 270-into-360 packing bug")
med = float(np.median(ratios)) if ratios else float('nan')
ok = len(errs) > 50 and np.median(errs) <= 0.08 and abs(med - 1.0) < 0.12
print("VERDICT:", "PASS" if ok else "FAIL")
rclpy.shutdown()
