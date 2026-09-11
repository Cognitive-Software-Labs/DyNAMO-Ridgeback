#!/usr/bin/env python3
"""Predict what the 2D lidars see OF THE ROBOT ITSELF, from the robot USD.

Slices the robot's *visible render* meshes at the scan plane (the RTX lidar
raytraces render geometry, not colliders — so `riser_link`'s collision Cube,
which straddles the plane, is correctly invisible to it) and ray-casts from
each lidar over the full 270-deg contract arc. Output is a self-occlusion
range profile: for every bearing, the range at which the robot's own body
would answer, or `inf` if the arc is clear.

This is the offline half of `docs/isaac/open-issues.md` §1. The stall's
phantom returns land at base_link y ~ +0.39 == the body half-width, so
"the lidar sees its own body" needs a geometric verdict, not an AABB guess:
the lidars sit recessed in a notch, so the cross-section at the scan plane
is NOT the 0.932 x 0.790 hull.

    isaac_venv/bin/python3 tools/isaac/self_occlusion_check.py
    isaac_venv/bin/python3 tools/isaac/self_occlusion_check.py --plane-z 0.2264

Pure pxr — no SimulationApp, no GPU, no physics.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
ROBOT = (REPO / "src/ridgeback_autonomy/sim/isaac/usd/robots/ridgeback_r100"
              / "ridgeback_r100.usda")

# Contract scan geometry (mirrors ros_io.LidarScanAssembler).
N_BINS = 1081
ANGLE_MIN = -3.0 * math.pi / 4.0
ANGLE_INC = math.radians(0.25)


def visible_render_meshes(stage):
    """Render-purpose, visible Mesh prims — what the RTX lidar can hit.

    Excludes colliders (`collisions/` scopes and `UsdPhysics`-only Cubes) and
    anything marked invisible or guide/proxy purpose.
    """
    from pxr import UsdGeom

    out = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        path = prim.GetPath().pathString
        if "/collisions/" in path:
            continue
        img = UsdGeom.Imageable(prim)
        if img.ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        purpose = img.ComputePurpose()
        if purpose in (UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy):
            continue
        out.append(prim)
    return out


def slice_prim(prim, plane_z, xform_cache):
    """Plane-crossing segments [(x0,y0,x1,y1)] for one mesh, in base_link."""
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not pts or not counts or not idx:
        return []

    m = xform_cache.GetLocalToWorldTransform(prim)
    P = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
    M = np.array(m, dtype=np.float64).reshape(4, 4)
    pw = (np.c_[P, np.ones(len(P))] @ M)[:, :3]

    # fan-triangulate
    tri = []
    k = 0
    for c in counts:
        f = idx[k:k + c]
        for i in range(1, c - 1):
            tri.append((f[0], f[i], f[i + 1]))
        k += c
    if not tri:
        return []
    T = pw[np.asarray(tri, dtype=np.int64)]        # (ntri, 3, 3)

    tz = T[:, :, 2]
    crosses = (tz.min(1) <= plane_z) & (tz.max(1) >= plane_z)
    segs = []
    for t in np.nonzero(crosses)[0]:
        v = T[t]
        hits = []
        for i in range(3):
            p, q = v[i], v[(i + 1) % 3]
            dp, dq = p[2] - plane_z, q[2] - plane_z
            if (dp <= 0 <= dq) or (dq <= 0 <= dp):
                denom = dp - dq
                if abs(denom) < 1e-12:
                    hits.append((p[0], p[1]))
                    hits.append((q[0], q[1]))
                else:
                    s = dp / denom
                    hits.append((p[0] + s * (q[0] - p[0]),
                                 p[1] + s * (q[1] - p[1])))
        if len(hits) >= 2:
            segs.append((hits[0][0], hits[0][1], hits[1][0], hits[1][1]))
    return segs


def cast(segs, origin, bearings, self_skip=0.02):
    """Nearest segment hit per bearing. segs: (n,4) array."""
    if len(segs) == 0:
        return np.full(len(bearings), np.inf)
    S = np.asarray(segs, dtype=np.float64)
    ax, ay, bx, by = S[:, 0], S[:, 1], S[:, 2], S[:, 3]
    ex, ey = bx - ax, by - ay
    ox, oy = origin
    out = np.full(len(bearings), np.inf)
    for j, th in enumerate(bearings):
        dx, dy = math.cos(th), math.sin(th)
        # solve o + t*d = a + u*e  ->  cross products
        denom = dx * ey - dy * ex
        ok = np.abs(denom) > 1e-15
        if not ok.any():
            continue
        qx, qy = ax - ox, ay - oy
        t = np.where(ok, (qx * ey - qy * ex) / np.where(ok, denom, 1.0), -1.0)
        u = np.where(ok, (qx * dy - qy * dx) / np.where(ok, denom, 1.0), -1.0)
        valid = ok & (t > self_skip) & (u >= 0.0) & (u <= 1.0)
        if valid.any():
            out[j] = t[valid].min()
    return out


def main():
    from pxr import Usd, UsdGeom

    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=Path, default=ROBOT)
    ap.add_argument("--plane-z", type=float, default=None,
                    help="default: the front lidar's z read from the USD")
    ap.add_argument("--step", type=int, default=1,
                    help="report every Nth bin")
    args = ap.parse_args()

    stage = Usd.Stage.Open(str(args.robot), Usd.Stage.LoadAll)
    xc = UsdGeom.XformCache()

    lidars = {}
    for prim in stage.Traverse():
        if prim.GetName() in ("lidar2d_0_laser", "lidar2d_1_laser"):
            m = xc.GetLocalToWorldTransform(prim)
            tr = m.ExtractTranslation()
            rot = m.ExtractRotationMatrix()
            yaw = math.atan2(rot[0][1], rot[0][0])
            lidars[prim.GetName()] = (tr[0], tr[1], tr[2], yaw)
    if not lidars:
        raise SystemExit("no lidar2d_*_laser prims found")

    plane_z = (args.plane_z if args.plane_z is not None
               else lidars["lidar2d_0_laser"][2])

    meshes = visible_render_meshes(stage)
    print(f"robot: {args.robot.name}")
    print(f"scan plane z = {plane_z:.4f} (base_link)")
    print(f"visible render meshes: {len(meshes)}")

    per_prim = {}
    for prim in meshes:
        s = slice_prim(prim, plane_z, xc)
        if s:
            per_prim[prim.GetPath().pathString] = s
    all_segs = [s for segs in per_prim.values() for s in segs]
    print(f"plane-crossing segments: {len(all_segs)}\n")

    print("-- contributing meshes (cross-section extent at the plane) --")
    if not per_prim:
        print("  NONE — no visible geometry crosses the scan plane")
    for p, s in sorted(per_prim.items(), key=lambda kv: -len(kv[1])):
        S = np.asarray(s)
        xs = np.r_[S[:, 0], S[:, 2]]
        ys = np.r_[S[:, 1], S[:, 3]]
        print(f"  {len(s):5d} segs  x[{xs.min():+.4f},{xs.max():+.4f}]  "
              f"y[{ys.min():+.4f},{ys.max():+.4f}]  {p}")

    for name in sorted(lidars):
        lx, ly, lz, lyaw = lidars[name]
        bearings = ANGLE_MIN + ANGLE_INC * np.arange(N_BINS)
        world_bearings = bearings + lyaw
        # A lidar cannot range its own housing: those returns sit at 0.02 m,
        # well inside the UST-10LX 0.06 m range_min, and the sensor model
        # discards them. Cast against everything else.
        own_link = name.replace("_laser", "_link")
        segs = [s for p, ss in per_prim.items() if f"/{own_link}/" not in p
                for s in ss]
        r = cast(segs, (lx, ly), world_bearings)
        finite = np.isfinite(r)
        print(f"\n-- {name} at ({lx:+.4f},{ly:+.4f}) yaw {math.degrees(lyaw):+.1f} "
              f"deg --")
        print(f"   self-occluded bins: {finite.sum()} / {N_BINS}")
        if not finite.any():
            print("   arc is CLEAR of the robot's own body")
            continue
        idx = np.nonzero(finite)[0]
        # contiguous runs
        runs = np.split(idx, np.nonzero(np.diff(idx) != 1)[0] + 1)
        for run in runs:
            d0 = math.degrees(bearings[run[0]])
            d1 = math.degrees(bearings[run[-1]])
            print(f"   bins {run[0]:4d}..{run[-1]:4d}  "
                  f"{d0:+7.2f}..{d1:+7.2f} deg  "
                  f"r {r[run].min():.4f}..{r[run].max():.4f} m")
        # asymmetry check
        left = finite[N_BINS // 2:].sum()
        right = finite[:N_BINS // 2].sum()
        print(f"   left(+theta) {left} bins  vs  right(-theta) {right} bins"
              + ("   <-- ASYMMETRIC" if left != right else "   (symmetric)"))


if __name__ == "__main__":
    main()
