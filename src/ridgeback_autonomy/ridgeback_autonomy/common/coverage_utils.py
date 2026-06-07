"""Coverage comparison helpers: SLAM occupancy grid vs a ground-truth map.

Both grids are normalised to the same value space:
    -1  unknown
     0  free
     1  occupied

They are spatially aligned in world coordinates (via their resolution/origin
metadata) and compared cell-by-cell. Coverage is measured over ground-truth
FREE cells only.

Used by ``coverage_overlay_node`` to score a SLAM map against a ground-truth
map. Kept ROS-agnostic: ``occupancy_msg_to_grid`` duck-types the OccupancyGrid
message rather than importing nav_msgs.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml


# ─────────────────────── normalised grid values ──────────────────────────────
UNKNOWN = -1
FREE = 0
OCCUPIED = 1


# ──────────────────────── map loading / conversion ───────────────────────────

def pgm_to_grid(pgm_path: Path):
    """Load a PGM + companion YAML and return a normalised int8 grid + meta dict.

    PGM pixel convention (ROS default, negate=0): pixels >= 200 are free,
    <= 50 occupied, the rest unknown.
    """
    pgm_path = Path(pgm_path)
    img = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Cannot load: {pgm_path}")

    yaml_path = pgm_path.with_suffix('.yaml')
    if not yaml_path.exists():
        raise FileNotFoundError(f"No companion .yaml: {yaml_path}")
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    grid = np.full(img.shape, UNKNOWN, dtype=np.int8)
    grid[img >= 200] = FREE
    grid[img <= 50] = OCCUPIED

    return grid, meta


def occupancy_msg_to_grid(msg):
    """Convert a nav_msgs/OccupancyGrid to a normalised int8 grid + meta dict.

    OccupancyGrid values: -1 unknown, 0 free, 1-100 occupied. The data array is
    row-major with row 0 at world y_min (bottom); we flipud so row 0 = world
    y_max (top), matching the PGM convention assumed by the alignment code.
    """
    w, h = msg.info.width, msg.info.height
    raw = np.array(msg.data, dtype=np.int8).reshape((h, w))

    grid = np.full((h, w), UNKNOWN, dtype=np.int8)
    grid[raw == 0] = FREE
    grid[raw > 0] = OCCUPIED  # 1–100 all treated as occupied

    grid = np.flipud(grid)  # bottom-first → top-first

    meta = {
        'resolution': msg.info.resolution,
        'origin': [
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            0.0,
        ],
    }
    return grid, meta


# ──────────────────────── spatial alignment ──────────────────────────────────

def world_extent(grid, meta):
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    h, w = grid.shape
    return ox, oy, ox + w * res, oy + h * res


def align_grids(slam_grid, slam_meta, gt_grid, gt_meta):
    """Crop both grids to their overlapping world region at the ground-truth
    resolution. Returns (slam_crop, gt_crop) or (None, None) if no overlap.
    """
    res = gt_meta['resolution']

    # Resample SLAM grid if its resolution differs.
    if not np.isclose(slam_meta['resolution'], res, rtol=0.01):
        scale = slam_meta['resolution'] / res
        new_w = int(round(slam_grid.shape[1] * scale))
        new_h = int(round(slam_grid.shape[0] * scale))
        # Nearest-neighbour so we don't invent intermediate occupancy values.
        slam_f = slam_grid.astype(np.float32)
        slam_f = cv2.resize(slam_f, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        slam_grid = slam_f.astype(np.int8)
        slam_meta = dict(slam_meta, resolution=res)

    sx0, sy0, sx1, sy1 = world_extent(slam_grid, slam_meta)
    gx0, gy0, gx1, gy1 = world_extent(gt_grid, gt_meta)

    ix0 = max(sx0, gx0)
    iy0 = max(sy0, gy0)
    ix1 = min(sx1, gx1)
    iy1 = min(sy1, gy1)

    if ix0 >= ix1 or iy0 >= iy1:
        return None, None

    def crop(grid, meta, x0, y0, x1, y1):
        ox, oy = meta['origin'][0], meta['origin'][1]
        h, w = grid.shape
        c0 = max(0, int(round((x0 - ox) / res)))
        c1 = min(w, int(round((x1 - ox) / res)))
        r0 = max(0, h - int(round((y1 - oy) / res)))
        r1 = min(h, h - int(round((y0 - oy) / res)))
        return grid[r0:r1, c0:c1]

    sc = crop(slam_grid, slam_meta, ix0, iy0, ix1, iy1)
    gc = crop(gt_grid, gt_meta, ix0, iy0, ix1, iy1)

    rh = min(sc.shape[0], gc.shape[0])
    rw = min(sc.shape[1], gc.shape[1])
    return sc[:rh, :rw], gc[:rh, :rw]


# ──────────────────────── coverage metric ────────────────────────────────────

def compute_stats(slam_crop, gt_crop):
    """Compare two normalised grids cell-by-cell. Returns a dict of statistics.

    Two headline numbers come out of this (don't conflate them):
      * coverage_pct  — accuracy over the *explored* gt-free area
                        (slam_found_free / gt-free cells slam has reached).
      * completeness  — slam_found_free / gt_free_cells (computed by callers),
                        how much of all reachable free space is discovered.
    """
    diff = slam_crop.astype(np.int16) - gt_crop.astype(np.int16)

    gt_known = gt_crop != UNKNOWN
    slam_known = slam_crop != UNKNOWN
    both_known = gt_known & slam_known

    gt_free = gt_crop == FREE

    # Coverage (accuracy): how much of explored gt-free did slam also mark free?
    gt_free_known = gt_free & slam_known
    slam_found_free = gt_free_known & (slam_crop == FREE)

    coverage_pct = (
        slam_found_free.sum() / gt_free_known.sum() * 100
        if gt_free_known.sum() > 0 else 0.0
    )

    agreed = both_known & (diff == 0)
    agreement_pct = agreed.sum() / both_known.sum() * 100 if both_known.sum() > 0 else 0.0

    # slam=free where gt=occupied  → diff = -1 ; slam=occupied where gt=free → +1
    false_free = both_known & (diff == -1)
    false_occ = both_known & (diff == 1)

    # Unknown in slam over gt-free area = unexplored reachable space.
    unexplored = gt_free & (slam_crop == UNKNOWN)

    return {
        'coverage_pct': float(coverage_pct),
        'agreement_pct': float(agreement_pct),
        'gt_free_cells': int(gt_free.sum()),
        'slam_found_free': int(slam_found_free.sum()),
        'unexplored_cells': int(unexplored.sum()),
        'false_free_cells': int(false_free.sum()),
        'false_occ_cells': int(false_occ.sum()),
        'both_known_cells': int(both_known.sum()),
        'agreed_cells': int(agreed.sum()),
    }
