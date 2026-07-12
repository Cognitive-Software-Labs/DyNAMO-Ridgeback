#!/usr/bin/env python3
"""Analytic ground-truth occupancy grid straight from an SDF world.

Rasterizes every *visual* geom whose vertical extent crosses the 2D lidar's
scan plane (RTX lidar raytraces render geometry, not colliders — see
sdf2usd.py). No sim, no capture drive: the SDF is the declared source of
truth, so the grid is exact and regeneratable. Convention matches
nav_msgs/OccupancyGrid: row-major, index [iy, ix], origin = lower-left cell
corner in world coords, 100 = occupied, 0 = free.

Includes (e.g. the vendored G1) are opaque meshes this parser cannot see
into; their footprint is emitted as an *ignore mask* circle instead, so map
metrics skip those cells rather than mis-scoring them.

Pure python + numpy (parse layer of sdf2usd has no pxr dependency):

    python3 tools/isaac/gt_occupancy.py \
        src/ridgeback_autonomy/sim/worlds/mock_hospital.sdf \
        /tmp/gt_hospital.npz --png /tmp/gt_hospital.png

The lidar plane default (0.418 m) is the front UST-10LX height in the
committed robot USD (0.342 m above base_link) plus the resting spawn_z
(0.076 m).
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sdf2usd import parse_world  # noqa: E402  (dependency-free parse layer)

LIDAR_PLANE_Z = 0.418
RESOLUTION = 0.05


def _geom_world_pose(model_pose, geom_pose):
    """Compose model ∘ geom planar pose -> (x, y, z, yaw).

    Every geom in the in-repo worlds is upright (roll = pitch = 0); refuse
    anything else rather than rasterize a wrong footprint.
    """
    for p, what in ((model_pose, "model"), (geom_pose, "geom")):
        if abs(p[3]) > 1e-9 or abs(p[4]) > 1e-9:
            raise ValueError(f"non-upright {what} pose unsupported: {p}")
    myaw = model_pose[5]
    c, s = math.cos(myaw), math.sin(myaw)
    gx, gy, gz = geom_pose[:3]
    return (
        model_pose[0] + c * gx - s * gy,
        model_pose[1] + s * gx + c * gy,
        model_pose[2] + gz,
        myaw + geom_pose[5],
    )


def _footprint_at_plane(kind, size, wz, plane_z):
    """Half-extents (hx, hy) of the geom's cross-section at plane_z, or None
    if the geom does not reach the plane. Spheres return the chord radius."""
    if kind == "box":
        if abs(plane_z - wz) > size[2] / 2:
            return None
        return size[0] / 2, size[1] / 2
    r = size[0]
    dz = plane_z - wz
    if abs(dz) >= r:
        return None
    chord = math.sqrt(r * r - dz * dz)
    return chord, chord


def build_grid(world, plane_z=LIDAR_PLANE_Z, resolution=RESOLUTION,
               margin=0.5, ignore_radius=0.7):
    """Rasterize -> (grid uint8 [iy, ix], ignore bool, origin (ox, oy)).

    grid: 100 occupied / 0 free. ignore: cells whose occupancy the SDF
    cannot predict (include footprints), to be masked out of metrics.
    """
    # world extent from all rasterized footprints + margin
    rects = []       # (cx, cy, hx, hy, yaw)
    for model in world.models:
        for g in model.geoms:
            if g.is_collision:
                continue  # lidar sees visuals only
            x, y, z, yaw = _geom_world_pose(model.pose, g.pose)
            half = _footprint_at_plane(g.kind, g.size, z, plane_z)
            if half is None:
                continue
            rects.append((x, y, half[0], half[1], yaw))
    if not rects:
        raise ValueError("nothing crosses the lidar plane — wrong plane_z?")

    def extent(r, axis):
        c, s = math.cos(r[4]), math.sin(r[4])
        return (abs(c) * r[2] + abs(s) * r[3] if axis == 0
                else abs(s) * r[2] + abs(c) * r[3])

    xs_lo = min(r[0] - extent(r, 0) for r in rects) - margin
    xs_hi = max(r[0] + extent(r, 0) for r in rects) + margin
    ys_lo = min(r[1] - extent(r, 1) for r in rects) - margin
    ys_hi = max(r[1] + extent(r, 1) for r in rects) + margin
    nx = int(math.ceil((xs_hi - xs_lo) / resolution))
    ny = int(math.ceil((ys_hi - ys_lo) / resolution))
    grid = np.zeros((ny, nx), dtype=np.uint8)
    ignore = np.zeros((ny, nx), dtype=bool)

    # cell-center coordinate matrices (built once, reused per footprint)
    cxs = xs_lo + (np.arange(nx) + 0.5) * resolution
    cys = ys_lo + (np.arange(ny) + 0.5) * resolution

    def paint(cx, cy, hx, hy, yaw, target):
        r = math.hypot(hx, hy)
        ix0 = max(0, int((cx - r - xs_lo) / resolution))
        ix1 = min(nx, int((cx + r - xs_lo) / resolution) + 1)
        iy0 = max(0, int((cy - r - ys_lo) / resolution))
        iy1 = min(ny, int((cy + r - ys_lo) / resolution) + 1)
        if ix0 >= ix1 or iy0 >= iy1:
            return
        gx = cxs[ix0:ix1][None, :] - cx
        gy = cys[iy0:iy1][:, None] - cy
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx = c * gx - s * gy      # into the geom's local frame
        ly = s * gx + c * gy
        target[iy0:iy1, ix0:ix1] |= (np.abs(lx) <= hx) & (np.abs(ly) <= hy)

    occ = np.zeros((ny, nx), dtype=bool)
    for cx, cy, hx, hy, yaw in rects:
        paint(cx, cy, hx, hy, yaw, occ)
    grid[occ] = 100

    for inc in world.includes:
        paint(inc.pose[0], inc.pose[1], ignore_radius, ignore_radius, 0.0,
              ignore)

    return grid, ignore, (xs_lo, ys_lo)


def check_waypoints(grid, origin, resolution, waypoints, clearance=0.8,
                    step=0.05):
    """Verify every point along the waypoint polyline keeps `clearance`
    metres from occupied cells. Returns a list of violation strings."""
    ny, nx = grid.shape
    occ_iy, occ_ix = np.nonzero(grid == 100)
    occ_x = origin[0] + (occ_ix + 0.5) * resolution
    occ_y = origin[1] + (occ_iy + 0.5) * resolution
    problems = []
    pts = []
    for a, b in zip(waypoints, waypoints[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(d / step))
        pts += [(a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n)
                for t in range(n + 1)]
    for px, py in pts:
        dmin = float(np.min(np.hypot(occ_x - px, occ_y - py)))
        if dmin < clearance:
            problems.append(
                f"({px:.2f},{py:.2f}) clearance {dmin:.2f} < {clearance}")
    return problems


def save_png(grid, ignore, path, scale=3):
    from PIL import Image
    ny, nx = grid.shape
    img = np.full((ny, nx, 3), 255, dtype=np.uint8)
    img[grid == 100] = (0, 0, 0)
    img[ignore] = (255, 200, 120)
    im = Image.fromarray(img[::-1])            # world y-up -> image y-down
    im = im.resize((nx * scale, ny * scale), Image.NEAREST)
    im.save(path)


def load_grid(npz_path):
    d = np.load(npz_path)
    return (d["grid"], d["ignore"], (float(d["origin"][0]),
            float(d["origin"][1])), float(d["resolution"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("sdf", type=Path)
    ap.add_argument("out", type=Path, help="output .npz")
    ap.add_argument("--plane-z", type=float, default=LIDAR_PLANE_Z)
    ap.add_argument("--resolution", type=float, default=RESOLUTION)
    ap.add_argument("--png", type=Path, help="optional preview PNG")
    args = ap.parse_args()

    world = parse_world(args.sdf)
    grid, ignore, origin = build_grid(
        world, plane_z=args.plane_z, resolution=args.resolution)
    np.savez_compressed(
        args.out, grid=grid, ignore=ignore, origin=np.array(origin),
        resolution=args.resolution, plane_z=args.plane_z)
    ny, nx = grid.shape
    print(f"wrote {args.out}: {nx}x{ny} @ {args.resolution} m, origin "
          f"({origin[0]:.2f},{origin[1]:.2f}), occupied "
          f"{int((grid == 100).sum())} cells, ignore {int(ignore.sum())}")
    if args.png:
        save_png(grid, ignore, args.png)
        print(f"preview: {args.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
