#!/usr/bin/env python3
"""Analytic ground-truth occupancy map from an Isaac world USD.

The gz-era GT maps were captured by driving SLAM; the Isaac stock environments
(warehouse/office/hospital) have no SDF for gt_occupancy.py to parse. This tool
generates the reference map analytically from the USD stage by *slicing visual
geometry at the 2D lidar plane* — exactly what the RTX lidar raytraces (render
meshes, not colliders). For every triangle that straddles plane-z it emits the
plane-crossing segment; those segments rasterize to occupied cells. Free space
is then the flood-fill reachable from an interior seed; everything the flood
can't reach (outside walls, sealed voids) is unknown.

Deliberately does NOT play physics or touch the occupancy-map generator
extension: playing this stage trips a 6.0.1 omni.graph.core crash, and the
slice is a pure-USD read (open, stream, ComputeLocalToWorldTransform, done),
so it is deterministic and contention-immune. Runs as its own headless
SimulationApp only to get the asset resolver + S3 streaming for stock worlds.

    source install/setup.bash
    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 \
        tools/isaac/generate_gt_map.py warehouse \
        --out src/ridgeback_autonomy/sim/ground_truth_maps

Outputs, per world, into --out:
    <world>.pgm + <world>.yaml   ROS map_server pair (coverage_overlay_node)
    <world>.png                  preview (occupied black, unknown tan)
    <world>.npz                  grid/ignore/origin/resolution (coverage_ceiling.py)

The seed defaults to (0,0); the stock envs are centred near the origin so it
lands on open floor, but pass --origin x,y for a world whose (0,0) is inside a
wall/shelf (else the map comes back mostly unknown). mock_hospital and
g1_distance_calibration keep their exact SDF path (gt_occupancy.py) — this tool
is for the mesh-only stock worlds.

Plane default 0.418 m = the front UST-10LX height in the committed robot USD,
matching gt_occupancy.py.
"""
from __future__ import annotations

import argparse
import sys
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "src/ridgeback_autonomy/sim/isaac"))

LIDAR_PLANE_Z = 0.418
RESOLUTION = 0.05
PX_FREE = 254
PX_OCC = 0
PX_UNKNOWN = 205


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("world", help="world name (worlds.py) or explicit .usd path")
    ap.add_argument("--out", type=Path,
                    default=Path("src/ridgeback_autonomy/sim/ground_truth_maps"),
                    help="output directory")
    ap.add_argument("--name", default=None, help="output stem (default: world)")
    ap.add_argument("--cell-size", type=float, default=RESOLUTION)
    ap.add_argument("--plane-z", type=float, default=LIDAR_PLANE_Z)
    ap.add_argument("--origin", default=None,
                    help="flood-fill seed x,y in world metres; default = the "
                         "centroid of the dominant geometry cluster (nearest "
                         "free cell is used if it lands on a wall)")
    ap.add_argument("--cluster-coarse", type=float, default=2.0,
                    help="coarse bin size (m) for the dominant-cluster crop "
                         "that isolates the building from far stray geometry")
    ap.add_argument("--margin", type=float, default=1.0,
                    help="metres of padding added around the sliced extent")
    ap.add_argument("--max-extent", type=float, default=150.0,
                    help="skip any mesh whose world XY span exceeds this "
                         "(metres) — environment skydomes/backdrops cross the "
                         "lidar plane hundreds of m out and would blow up the "
                         "grid; real building geometry is well under this")
    ap.add_argument("--load-frames", type=int, default=600,
                    help="max app.update() frames to wait for S3 assets to "
                         "stream (no physics play — pure load pump)")
    return ap.parse_args()


def _slice_segments(stage, plane_z, max_extent):
    """World-space plane-crossing segments [(x0,y0,x1,y1), ...] from every
    visible mesh triangle that straddles plane_z. Descends native instance
    proxies and expands PointInstancers. Meshes whose world XY span exceeds
    max_extent (skydomes/backdrops) are skipped so they can't blow up bounds."""
    from pxr import Gf, Usd, UsdGeom

    segs = []
    skipped = [0]
    stats = {"instancers": 0, "instances": 0}
    tc = Usd.TimeCode.Default()

    def slice_mesh(points_world):
        """points_world: (n,3) float64; iterate the mesh's triangles already
        fanned by the caller into consecutive triples."""
        z = points_world[:, 2]
        tri = points_world.reshape(-1, 3, 3)
        tz = z.reshape(-1, 3)
        crosses = (tz.min(1) <= plane_z) & (tz.max(1) >= plane_z)
        for t in np.nonzero(crosses)[0]:
            v = tri[t]
            hits = []
            for i in range(3):
                p, q = v[i], v[(i + 1) % 3]
                dp, dq = p[2] - plane_z, q[2] - plane_z
                if (dp <= 0 <= dq) or (dq <= 0 <= dp):
                    denom = dp - dq
                    if abs(denom) < 1e-12:
                        hits.append((p[0], p[1])); hits.append((q[0], q[1]))
                    else:
                        s = dp / denom
                        hits.append((p[0] + s * (q[0] - p[0]),
                                     p[1] + s * (q[1] - p[1])))
            if len(hits) >= 2:
                segs.append((hits[0][0], hits[0][1], hits[1][0], hits[1][1]))

    def fan_triangulate(pts_local, counts, idx, m):
        """Transform local mesh points by m (vectorised — meshes can be 100k+
        verts), fan-triangulate its faces, return a flat (3*ntri, 3) world
        array. pxr uses row vectors: worldRow = [x y z 1] @ M."""
        P = np.array([[p[0], p[1], p[2]] for p in pts_local], dtype=np.float64)
        M = np.array(m, dtype=np.float64).reshape(4, 4)
        pw = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        if np.ptp(pw[:, :2], axis=0).max() > max_extent:
            skipped[0] += 1                # skydome / backdrop — not an obstacle
            return None
        tri_idx = []
        k = 0
        for c in counts:
            f = idx[k:k + c]
            k += c
            for j in range(1, c - 1):
                tri_idx.extend((f[0], f[j], f[j + 1]))
        return pw[tri_idx] if tri_idx else None

    def handle_mesh(prim, extra=None):
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get()
        counts = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            return
        l2w = UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(tc)
        if extra is not None:
            l2w = l2w * extra          # instancer per-instance world xform
        tris = fan_triangulate(pts, counts, list(idx), l2w)
        if tris is not None:
            slice_mesh(tris)

    it = iter(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
    for prim in it:
        img = UsdGeom.Imageable(prim)
        if img and img.ComputeVisibility(tc) == UsdGeom.Tokens.invisible:
            it.PruneChildren()
            continue
        if prim.IsA(UsdGeom.Mesh):
            handle_mesh(prim)
        elif prim.IsA(UsdGeom.PointInstancer):
            pi = UsdGeom.PointInstancer(prim)
            protos = pi.GetPrototypesRel().GetTargets()
            proto_idx = pi.GetProtoIndicesAttr().Get()
            xforms = pi.ComputeInstanceTransformsAtTime(tc, tc)
            if not protos or proto_idx is None or xforms is None:
                continue
            stats["instancers"] += 1
            stats["instances"] += len(proto_idx)
            # World xform for an instanced mesh:
            #   p_world = p_local * M_sub_in_proto * X_inst * M_instancer
            # handle_mesh multiplies by ComputeLocalToWorldTransform(sub) =
            # M_sub_world = M_sub_in_proto * M_proto_root, so `extra` must undo
            # M_proto_root first — else the prototype's own authored placement
            # is double-counted and instances fling metres away.
            instancer_l2w = UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(tc)
            for inst, pidx in enumerate(proto_idx):
                proto_prim = stage.GetPrimAtPath(protos[pidx])
                m_proto_root_inv = UsdGeom.Imageable(
                    proto_prim).ComputeLocalToWorldTransform(tc).GetInverse()
                extra = m_proto_root_inv * xforms[inst] * instancer_l2w
                for sub in Usd.PrimRange(proto_prim,
                                         Usd.TraverseInstanceProxies()):
                    if sub.IsA(UsdGeom.Mesh):
                        handle_mesh(sub, extra=extra)
            it.PruneChildren()          # instancer geometry handled explicitly
    return segs, skipped[0], stats


def _dominant_bounds(mx, my, coarse):
    """Bounds (xmin,ymin,xmax,ymax) of the spatially largest connected cluster
    of segment midpoints. Isolates the real building from far stray geometry
    (which survives a count-based percentile clip when heavily tessellated):
    walls form one big connected run of coarse bins; junk forms tiny detached
    blobs, so the component covering the most *bins* is the building."""
    from scipy.ndimage import binary_dilation, label

    x0, y0 = mx.min(), my.min()
    gx = ((mx - x0) / coarse).astype(int)
    gy = ((my - y0) / coarse).astype(int)
    occ = np.zeros((gy.max() + 1, gx.max() + 1), dtype=bool)
    occ[gy, gx] = True
    # bridge sub-cell wall gaps so a building doesn't split into fragments
    lab, n = label(binary_dilation(occ, iterations=1))
    sizes = np.bincount(lab.ravel())
    sizes[0] = 0
    ys_, xs_ = np.nonzero(lab == sizes.argmax())
    return (x0 + xs_.min() * coarse, y0 + ys_.min() * coarse,
            x0 + (xs_.max() + 1) * coarse, y0 + (ys_.max() + 1) * coarse)


def _rasterize(segs, cell, bounds):
    """Segments -> (occupied bool grid [iy,ix], origin (xmin,ymin)). Segments
    outside `bounds` (xmin,ymin,xmax,ymax) are clipped by the in-range mask."""
    xmin, ymin, xmax, ymax = bounds
    nx = int(np.ceil((xmax - xmin) / cell))
    ny = int(np.ceil((ymax - ymin) / cell))
    occ = np.zeros((ny, nx), dtype=bool)
    for x0, y0, x1, y1 in segs:
        n = max(1, int(np.hypot(x1 - x0, y1 - y0) / (cell * 0.5)) + 1)
        xs_ = np.linspace(x0, x1, n)
        ys_ = np.linspace(y0, y1, n)
        ix = ((xs_ - xmin) / cell).astype(int)
        iy = ((ys_ - ymin) / cell).astype(int)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        occ[iy[ok], ix[ok]] = True
    return occ, (xmin, ymin)


def _flood_free(occ, origin, seed_xy, cell):
    """Free = flood-fill of non-occupied cells reachable from seed; the rest is
    unknown. Nearest non-occupied cell is used if the seed lands on a wall."""
    ny, nx = occ.shape
    sx = int((seed_xy[0] - origin[0]) / cell)
    sy = int((seed_xy[1] - origin[1]) / cell)
    sx = min(max(sx, 0), nx - 1)
    sy = min(max(sy, 0), ny - 1)
    if occ[sy, sx]:                      # nudge to the nearest open cell
        openc = np.argwhere(~occ)
        if openc.size == 0:
            return np.zeros_like(occ)
        d = np.abs(openc[:, 0] - sy) + np.abs(openc[:, 1] - sx)
        sy, sx = openc[d.argmin()]
    free = np.zeros_like(occ)
    q = deque([(int(sy), int(sx))])
    free[sy, sx] = True
    while q:
        y, x = q.popleft()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny_, nx_ = y + dy, x + dx
            if 0 <= ny_ < ny and 0 <= nx_ < nx and not free[ny_, nx_] \
                    and not occ[ny_, nx_]:
                free[ny_, nx_] = True
                q.append((ny_, nx_))
    return free


def generate(app, args) -> int:
    import omni.usd
    from pxr import UsdGeom

    from worlds import get_assets_root, resolve_world

    world_path = resolve_world(args.world, get_assets_root())
    print(f"loading world: {world_path}", flush=True)
    ctx = omni.usd.get_context()
    ctx.open_stage(world_path)
    stage = ctx.get_stage()
    # Pump updates (NO timeline.play — physics play crashes omni.graph.core on
    # some stages) until the payloads/S3 references finish streaming. The
    # third field of get_stage_loading_status() is the count still loading;
    # keep a min-frames floor so streaming has queued before we trust a 0.
    for i in range(args.load_frames):
        app.update()
        _, _, loading = ctx.get_stage_loading_status()
        if i >= 30 and not loading:
            break
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    print(f"stage loaded after {i + 1} frames, metersPerUnit={mpu}", flush=True)

    plane = args.plane_z / mpu           # slice in stage units
    segs, skipped, stats = _slice_segments(stage, plane, args.max_extent / mpu)
    print(f"sliced {len(segs)} plane-crossing segments "
          f"({skipped} oversized meshes skipped; {stats['instancers']} "
          f"instancers, {stats['instances']} instances)", flush=True)
    if not segs:
        print("no geometry crosses the lidar plane — wrong plane-z or empty "
              "stage; aborting", flush=True)
        return 1

    # Bounds = the dominant connected cluster of segments (the building), not
    # raw min/max: stock envs strand the real interior in hundreds of metres of
    # far stray geometry, and a count-based percentile can't drop a heavily
    # tessellated far prop. Crop spatially instead.
    a = np.array(segs, dtype=np.float64)
    xs = np.concatenate([a[:, 0], a[:, 2]])
    ys = np.concatenate([a[:, 1], a[:, 3]])
    mx = 0.5 * (a[:, 0] + a[:, 2])
    my = 0.5 * (a[:, 1] + a[:, 3])
    m_u = args.margin / mpu
    xlo, ylo, xhi, yhi = _dominant_bounds(mx, my, args.cluster_coarse / mpu)
    print(f"segment extent raw x[{xs.min():.1f},{xs.max():.1f}] "
          f"y[{ys.min():.1f},{ys.max():.1f}] -> dominant cluster "
          f"x[{xlo:.1f},{xhi:.1f}] y[{ylo:.1f},{yhi:.1f}] (stage units)",
          flush=True)
    cell_u = args.cell_size / mpu
    bounds = (xlo - m_u, ylo - m_u, xhi + m_u, yhi + m_u)
    occ, origin_u = _rasterize(segs, cell_u, bounds)
    ox_m, oy_m = origin_u[0] * mpu, origin_u[1] * mpu       # -> metres
    if args.origin:
        seed = tuple(float(v) for v in args.origin.split(","))
    else:
        # centre of the cropped building (median midpoint inside bounds), so
        # the flood starts inside — the building is not always near (0,0).
        inb = (mx >= xlo) & (mx <= xhi) & (my >= ylo) & (my <= yhi)
        seed = (float(np.median(mx[inb])) * mpu,
                float(np.median(my[inb])) * mpu)
    free = _flood_free(occ, (ox_m, oy_m), seed, args.cell_size)
    unknown = ~occ & ~free
    ny, nx = occ.shape
    print(f"grid {nx}x{ny}: occ={int(occ.sum())} free={int(free.sum())} "
          f"unknown={int(unknown.sum())} origin=({ox_m:.3f},{oy_m:.3f})",
          flush=True)
    return _write_outputs(args, occ, free, unknown, nx, ny, ox_m, oy_m)


def _write_outputs(args, occ, free, unknown, nx, ny, xmin, ymin) -> int:
    """.npz keeps the bottom-first (origin = lower-left) convention
    gt_occupancy.load_grid expects; the ROS pgm is flipped to top-first."""
    from PIL import Image

    stem = args.name or args.world
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    res_m = args.cell_size

    grid = np.zeros((ny, nx), dtype=np.uint8)
    grid[occ] = 100
    np.savez_compressed(
        out / f"{stem}.npz", grid=grid, ignore=unknown,
        origin=np.array([xmin, ymin]), resolution=res_m, plane_z=args.plane_z)

    px = np.full((ny, nx), PX_UNKNOWN, dtype=np.uint8)
    px[free] = PX_FREE
    px[occ] = PX_OCC
    Image.fromarray(np.flipud(px), mode="L").save(out / f"{stem}.pgm")
    (out / f"{stem}.yaml").write_text(
        f"image: {stem}.pgm\n"
        f"mode: trinary\n"
        f"resolution: {res_m:.4f}\n"
        f"origin: [{xmin:.4f}, {ymin:.4f}, 0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n")

    rgb = np.full((ny, nx, 3), 255, dtype=np.uint8)
    rgb[unknown] = (255, 200, 120)
    rgb[occ] = (0, 0, 0)
    Image.fromarray(rgb[::-1]).resize((nx * 2, ny * 2), Image.NEAREST).save(
        out / f"{stem}.png")

    print(f"wrote {out}/{stem}.{{pgm,yaml,png,npz}} — {nx}x{ny} @ {res_m} m, "
          f"origin ({xmin:.3f},{ymin:.3f})", flush=True)
    return 0


def main():
    args = parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    code = 1
    try:
        code = generate(app, args)
    except Exception:
        import traceback
        traceback.print_exc()
        print("GT MAP FAILED", flush=True)
    app.close()
    sys.exit(code)


if __name__ == "__main__":
    main()
