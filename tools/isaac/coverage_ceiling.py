#!/usr/bin/env python3
"""Analytic coverage ceiling for a range-limited lidar in a known world.

Answers: what fraction of the ground-truth free space can a lidar of a given
range even *observe*, before any SLAM/exploration quality is considered? For
every traversable GT-free cell (a candidate robot vantage), it raycasts a full
360 deg of line-of-sight out to `--range` metres and marks the free cells that
vantage can see; the union over all vantages is the observable fraction.

360 deg is a deliberate UPPER BOUND for a narrower real sensor (e.g. the Isaac
270 deg UST-10LX): a robot rotates while exploring, so cumulatively it
approaches 360 deg from most positions. If even this generous bound falls below
the coverage target, the target is unreachable by the sensor and no amount of
SLAM tuning will close it; if it clears the target, a coverage shortfall is a
SLAM/exploration problem, not a sensor one.

    python3 tools/isaac/coverage_ceiling.py <gt_grid.npz> --range 10
    # build the grid first with gt_occupancy.py if you don't have the .npz

Result for mock_hospital (2026-07-13): 10 m -> 90.8%, 25 m -> 91.4% — the
Isaac 10 m lidar is NOT the coverage cap in this small world (gz's 25 m sees
only 0.6 pts more), so the isaac exploration plateau is drift/stall, not range.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gt_occupancy import load_grid  # noqa: E402


def observable_fraction(npz: Path, max_range_m: float, robot_radius_m: float = 0.35,
                        vantage_step_m: float = 0.4) -> dict:
    grid, ignore, origin, res = load_grid(npz)
    ny, nx = grid.shape
    free = (grid == 0) & ~ignore
    occ = (grid == 100)
    rng_cells = int(max_range_m / res)

    # Vantages = free cells the robot footprint fits in, subsampled for speed.
    from scipy.ndimage import binary_erosion
    rob_r = max(1, int(round(robot_radius_m / res)))
    vant = binary_erosion(free, iterations=rob_r)   # border treated as blocked
    vy, vx = np.nonzero(vant)
    step = max(1, int(vantage_step_m / res))
    keep = (vy % step == 0) & (vx % step == 0)
    vy, vx = vy[keep], vx[keep]

    observed = np.zeros_like(free)
    ang = np.linspace(0, 2 * math.pi, 360, endpoint=False)
    cos, sin = np.cos(ang), np.sin(ang)
    steps = np.arange(rng_cells)
    for cy, cx in zip(vy, vx):
        for a in range(360):
            xs = (cx + steps * cos[a]).astype(int)
            ys = (cy + steps * sin[a]).astype(int)
            inb = (xs >= 0) & (xs < nx) & (ys >= 0) & (ys < ny)
            xs, ys = xs[inb], ys[inb]
            if xs.size == 0:
                continue
            hit = np.nonzero(occ[ys, xs])[0]
            end = hit[0] if hit.size else xs.size
            observed[ys[:end], xs[:end]] = True

    total = int(free.sum())
    seen = int((observed & free).sum())
    return {"gt_free": total, "vantages": int(vy.size), "observable": seen,
            "ceiling_pct": round(100.0 * seen / total, 1) if total else 0.0}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("npz", type=Path, help="GT occupancy grid from gt_occupancy.py")
    ap.add_argument("--range", type=float, default=10.0, help="lidar max range (m)")
    args = ap.parse_args()
    r = observable_fraction(args.npz, args.range)
    print(f"GT-free cells:      {r['gt_free']}")
    print(f"vantages sampled:   {r['vantages']}")
    print(f"observable free:    {r['observable']}")
    print(f"OBSERVABLE CEILING: {r['ceiling_pct']}%  (360deg LOS, {args.range:.0f}m)")


if __name__ == "__main__":
    main()
