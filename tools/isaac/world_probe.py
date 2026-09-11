#!/usr/bin/env python3
"""What does the 2D lidar plane actually hit near a point in a world USD?

`generate_gt_map.py` answers this for a whole world and reduces it to an
occupancy grid, which loses *which prim* produced a cell and silently drops
meshes wider than `--max-extent`. This tool keeps the attribution: it slices
the world at the scan plane, reports every contributing prim within a radius
of a query point, and optionally ray-casts from a lidar pose over the contract
arc so the prediction can be compared bin-for-bin against a live scan.

Built for `docs/isaac/open-issues.md` §1: the GT map reads `free` around the
`warehouse_full` spawn, yet the front lidar returns 0.41-0.55 m on the robot's
left. One of those two is wrong, and a grid cell cannot say which prim it came
from.

    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 \
        tools/isaac/world_probe.py warehouse_full --at 0,0 --radius 1.5

    # predict the front lidar's returns from the robot's spawn pose
    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 \
        tools/isaac/world_probe.py warehouse_full --at 0,0 --radius 3 \
        --lidar 0.3922,0,0 --arc 120,135

Unlike the GT generator this applies NO max-extent skip by default, so a large
merged shell mesh that the map drops still shows up here.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "src/ridgeback_autonomy/sim/isaac"))

from gt_occupancy import LIDAR_PLANE_Z  # noqa: E402  (one owner for the plane)


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("world", help="world name (worlds.py) or explicit .usd path")
    ap.add_argument("--at", default="0,0", help="query centre x,y in metres")
    ap.add_argument("--radius", type=float, default=1.5,
                    help="report geometry within this radius of --at (m)")
    ap.add_argument("--plane-z", type=float, default=LIDAR_PLANE_Z,
                    help="world-frame scan plane height (m)")
    ap.add_argument("--lidar", default=None,
                    help="x,y,yaw_deg of a lidar to ray-cast from (metres/deg)")
    ap.add_argument("--arc", default="-135,135",
                    help="bearing range deg_lo,deg_hi for --lidar")
    ap.add_argument("--arc-step", type=float, default=0.25,
                    help="bearing step deg (contract is 0.25)")
    ap.add_argument("--max-extent", type=float, default=0.0,
                    help="skip meshes whose XY span exceeds this; 0 = no skip")
    ap.add_argument("--load-frames", type=int, default=600)
    ap.add_argument("--top", type=int, default=25, help="prims to list")
    return ap.parse_args()


def slice_near(stage, plane_u, cx, cy, radius_u, max_extent_u):
    """{prim_path: [(x0,y0,x1,y1)]} for segments within radius of (cx,cy).

    Mirrors generate_gt_map._slice_segments (instance proxies, PointInstancer
    expansion) but keeps prim attribution and prunes by bounding box first so
    a 100k-prim stock env stays cheap.
    """
    from pxr import Usd, UsdGeom

    tc = Usd.TimeCode.Default()
    bbox = UsdGeom.BBoxCache(tc, [UsdGeom.Tokens.default_,
                                  UsdGeom.Tokens.render])
    out = {}
    stats = {"instancers": 0, "instances": 0, "skipped": 0, "considered": 0}

    def keep(x, y):
        return (x - cx) ** 2 + (y - cy) ** 2 <= radius_u ** 2

    def slice_tris(T, path):
        tz = T[:, :, 2]
        crosses = (tz.min(1) <= plane_u) & (tz.max(1) >= plane_u)
        for t in np.nonzero(crosses)[0]:
            v = T[t]
            hits = []
            for i in range(3):
                p, q = v[i], v[(i + 1) % 3]
                dp, dq = p[2] - plane_u, q[2] - plane_u
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
                if keep(*hits[0]) or keep(*hits[1]):
                    out.setdefault(path, []).append(
                        (hits[0][0], hits[0][1], hits[1][0], hits[1][1]))

    def tris_of(prim, extra=None):
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get()
        counts = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            return None
        l2w = UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(tc)
        if extra is not None:
            l2w = l2w * extra
        P = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        M = np.array(l2w, dtype=np.float64).reshape(4, 4)
        pw = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        if max_extent_u > 0 and np.ptp(pw[:, :2], axis=0).max() > max_extent_u:
            stats["skipped"] += 1
            return None
        # cheap reject: mesh nowhere near the query disc, or misses the plane
        if (pw[:, 2].min() > plane_u) or (pw[:, 2].max() < plane_u):
            return None
        if (pw[:, 0].min() - cx > radius_u or cx - pw[:, 0].max() > radius_u or
                pw[:, 1].min() - cy > radius_u or cy - pw[:, 1].max() > radius_u):
            return None
        tri = []
        k = 0
        for c in counts:
            f = idx[k:k + c]
            for i in range(1, c - 1):
                tri.append((f[0], f[i], f[i + 1]))
            k += c
        if not tri:
            return None
        stats["considered"] += 1
        return pw[np.asarray(tri, dtype=np.int64)]

    it = iter(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
    for prim in it:
        img = UsdGeom.Imageable(prim)
        if img and img.ComputeVisibility(tc) == UsdGeom.Tokens.invisible:
            it.PruneChildren()
            continue
        if prim.IsA(UsdGeom.Mesh):
            T = tris_of(prim)
            if T is not None:
                slice_tris(T, prim.GetPath().pathString)
        elif prim.IsA(UsdGeom.PointInstancer):
            pi = UsdGeom.PointInstancer(prim)
            protos = pi.GetPrototypesRel().GetTargets()
            proto_idx = pi.GetProtoIndicesAttr().Get()
            xforms = pi.ComputeInstanceTransformsAtTime(tc, tc)
            if not protos or proto_idx is None or xforms is None:
                it.PruneChildren()
                continue
            stats["instancers"] += 1
            # Prune instances whose translation is far from the query disc
            # before touching prototype geometry (stock envs have 10k+).
            instancer_l2w = img.ComputeLocalToWorldTransform(tc)
            for inst, pidx in enumerate(proto_idx):
                X = np.array(xforms[inst] * instancer_l2w,
                             dtype=np.float64).reshape(4, 4)
                tx, ty = X[3, 0], X[3, 1]
                proto_prim = stage.GetPrimAtPath(protos[pidx])
                pr_range = bbox.ComputeWorldBound(proto_prim).ComputeAlignedRange()
                pad = 0.0
                if not pr_range.IsEmpty():
                    d = pr_range.GetMax() - pr_range.GetMin()
                    pad = math.hypot(d[0], d[1])
                if math.hypot(tx - cx, ty - cy) > radius_u + pad:
                    continue
                stats["instances"] += 1
                m_proto_root_inv = UsdGeom.Imageable(
                    proto_prim).ComputeLocalToWorldTransform(tc).GetInverse()
                extra = m_proto_root_inv * xforms[inst] * instancer_l2w
                for sub in Usd.PrimRange(proto_prim,
                                         Usd.TraverseInstanceProxies()):
                    if sub.IsA(UsdGeom.Mesh):
                        T = tris_of(sub, extra=extra)
                        if T is not None:
                            slice_tris(T, f"{prim.GetPath()}[{inst}]"
                                          f"{sub.GetPath()}")
            it.PruneChildren()
    return out, stats


def cast(segs, origin, bearings):
    if len(segs) == 0:
        return np.full(len(bearings), np.inf)
    S = np.asarray(segs, dtype=np.float64)
    ax, ay, bx, by = S[:, 0], S[:, 1], S[:, 2], S[:, 3]
    ex, ey = bx - ax, by - ay
    ox, oy = origin
    qx, qy = ax - ox, ay - oy
    out = np.full(len(bearings), np.inf)
    for j, th in enumerate(bearings):
        dx, dy = math.cos(th), math.sin(th)
        denom = dx * ey - dy * ex
        ok = np.abs(denom) > 1e-15
        if not ok.any():
            continue
        safe = np.where(ok, denom, 1.0)
        t = (qx * ey - qy * ex) / safe
        u = (qx * dy - qy * dx) / safe
        valid = ok & (t > 1e-6) & (u >= 0.0) & (u <= 1.0)
        if valid.any():
            out[j] = t[valid].min()
    return out


def main():
    args = parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    code = 1
    try:
        code = run(app, args)
    except Exception:
        import traceback
        traceback.print_exc()
    app.close()
    sys.exit(code)


def run(app, args) -> int:
    import omni.usd
    from pxr import UsdGeom
    from worlds import get_assets_root, resolve_world

    world_path = resolve_world(args.world, get_assets_root())
    print(f"loading world: {world_path}", flush=True)
    ctx = omni.usd.get_context()
    ctx.open_stage(world_path)
    stage = ctx.get_stage()
    for i in range(args.load_frames):
        app.update()
        _, _, loading = ctx.get_stage_loading_status()
        if i >= 30 and not loading:
            break
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    print(f"stage loaded after {i + 1} frames, metersPerUnit={mpu}", flush=True)

    cx_m, cy_m = (float(v) for v in args.at.split(","))
    plane_u = args.plane_z / mpu
    cx, cy = cx_m / mpu, cy_m / mpu
    radius_u = args.radius / mpu

    per_prim, stats = slice_near(stage, plane_u, cx, cy, radius_u,
                                 args.max_extent / mpu)
    n = sum(len(v) for v in per_prim.values())
    print(f"\nscan plane z = {args.plane_z} m   query ({cx_m},{cy_m}) "
          f"r={args.radius} m")
    print(f"meshes sliced: {stats['considered']}  instancers: "
          f"{stats['instancers']}  instances near: {stats['instances']}  "
          f"oversized skipped: {stats['skipped']}")
    print(f"plane-crossing segments within radius: {n}\n")

    if not per_prim:
        print("NOTHING crosses the scan plane within the query radius.")
    else:
        print(f"-- contributing prims (nearest first), distances in metres --")
        rows = []
        for p, segs in per_prim.items():
            S = np.asarray(segs) * mpu
            mxs = (S[:, 0] + S[:, 2]) / 2.0
            mys = (S[:, 1] + S[:, 3]) / 2.0
            d = np.hypot(mxs - cx_m, mys - cy_m)
            rows.append((d.min(), len(segs), S[:, [0, 2]].min(),
                         S[:, [0, 2]].max(), S[:, [1, 3]].min(),
                         S[:, [1, 3]].max(), p))
        rows.sort()
        for dmin, cnt, x0, x1, y0, y1, p in rows[:args.top]:
            short = p if len(p) <= 88 else "..." + p[-85:]
            print(f"  d={dmin:6.3f}  {cnt:5d} segs  "
                  f"x[{x0:+.3f},{x1:+.3f}] y[{y0:+.3f},{y1:+.3f}]\n"
                  f"            {short}")
        if len(rows) > args.top:
            print(f"  ... {len(rows) - args.top} more prims")

    if args.lidar:
        lx_m, ly_m, lyaw_d = (float(v) for v in args.lidar.split(","))
        d0, d1 = (float(v) for v in args.arc.split(","))
        bearings_deg = np.arange(d0, d1 + 1e-9, args.arc_step)
        world_b = np.radians(bearings_deg + lyaw_d)
        all_segs = [s for v in per_prim.values() for s in v]
        r = cast(all_segs, (lx_m / mpu, ly_m / mpu), world_b) * mpu
        finite = np.isfinite(r)
        print(f"\n-- predicted returns from lidar ({lx_m},{ly_m}) "
              f"yaw {lyaw_d} deg, arc {d0}..{d1} deg --")
        print(f"   {finite.sum()} / {len(r)} bearings hit geometry")
        for b, rr in zip(bearings_deg[finite], r[finite]):
            th = math.radians(b + lyaw_d)
            print(f"   {b:+8.2f} deg  r={rr:.4f}  "
                  f"world=({lx_m + rr * math.cos(th):+.4f},"
                  f"{ly_m + rr * math.sin(th):+.4f})")
    return 0


if __name__ == "__main__":
    main()
