#!/usr/bin/env python3
"""
Coverage analyzer: Compare SLAM map against ground-truth map.

Computes what fraction of the reachable space was discovered by the explorer.

Both maps are aligned in world coordinates using their .yaml metadata before
comparison, so different map extents / origins are handled correctly.

Usage:
    python3 analyze_coverage.py <slam_map.pgm> <ground_truth.pgm> [--output out.png]

The script auto-discovers the matching .yaml for each .pgm (same basename).

Example:
    python3 analyze_coverage.py ~/test_coverage.pgm warehouse.pgm --output cov.png
"""

import argparse
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("ERROR: cv2 or numpy not available.")
    print("Install with: pip install opencv-python numpy")
    sys.exit(1)

try:
    import yaml as _yaml
except ImportError:
    print("ERROR: pyyaml not available.")
    print("Install with: pip install pyyaml")
    sys.exit(1)


# ─────────────────────────── map loading ────────────────────────────

def load_map_with_meta(pgm_path: Path):
    """
    Load a .pgm file and its companion .yaml.

    Returns
    -------
    img : np.ndarray  (H x W, uint8, grayscale)
    meta : dict       keys: resolution (m/px), origin (x, y, theta)
    """
    pgm_path = Path(pgm_path)
    yaml_path = pgm_path.with_suffix('.yaml')

    img = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Cannot load image: {pgm_path}")

    if not yaml_path.exists():
        print(f"WARNING: No .yaml found at {yaml_path}; assuming resolution=0.05, origin=(0,0)")
        meta = {'resolution': 0.05, 'origin': [0.0, 0.0, 0.0]}
    else:
        with open(yaml_path) as f:
            meta = _yaml.safe_load(f)

    return img, meta


# ─────────────────────── spatial alignment ──────────────────────────

def world_extent(img, meta):
    """
    Return the world-coordinate bounding box of a map.

    ROS convention: origin = world coords of the *bottom-left* pixel.
    Image row 0 is the TOP of the map (y_max); last row is the bottom (y_min).

    Returns (x_min, y_min, x_max, y_max) in metres.
    """
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    h, w = img.shape
    x_min = ox
    y_min = oy
    x_max = ox + w * res
    y_max = oy + h * res
    return x_min, y_min, x_max, y_max


def align_maps(slam_img, slam_meta, gt_img, gt_meta):
    """
    Crop / pad both maps so they cover exactly the same world region.

    Returns two images of identical shape, in the intersection of their extents.
    If the maps don't overlap at all, raises ValueError.
    """
    res = gt_meta['resolution']          # use ground-truth resolution as reference
    slam_res = slam_meta['resolution']

    # Resample SLAM map if resolutions differ
    if not np.isclose(slam_res, res, rtol=0.01):
        scale = slam_res / res
        new_w = int(round(slam_img.shape[1] * scale))
        new_h = int(round(slam_img.shape[0] * scale))
        slam_img = cv2.resize(slam_img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        slam_meta = dict(slam_meta)
        slam_meta['resolution'] = res

    sx_min, sy_min, sx_max, sy_max = world_extent(slam_img, slam_meta)
    gx_min, gy_min, gx_max, gy_max = world_extent(gt_img, gt_meta)

    # Intersection
    ix_min = max(sx_min, gx_min)
    iy_min = max(sy_min, gy_min)
    ix_max = min(sx_max, gx_max)
    iy_max = min(sy_max, gy_max)

    if ix_min >= ix_max or iy_min >= iy_max:
        raise ValueError(
            f"Maps do not overlap in world coordinates!\n"
            f"  SLAM:  x=[{sx_min:.1f}, {sx_max:.1f}]  y=[{sy_min:.1f}, {sy_max:.1f}]\n"
            f"  GT:    x=[{gx_min:.1f}, {gx_max:.1f}]  y=[{gy_min:.1f}, {gy_max:.1f}]"
        )

    def crop(img, meta, x0, y0, x1, y1):
        ox, oy = meta['origin'][0], meta['origin'][1]
        h, w = img.shape
        # pixel indices (note: row 0 = y_max in world coords)
        col0 = int(round((x0 - ox) / res))
        col1 = int(round((x1 - ox) / res))
        # rows are inverted: row for world-y maps to (h - (y - oy)/res)
        row0 = h - int(round((y1 - oy) / res))   # y1 (top) → smaller row index
        row1 = h - int(round((y0 - oy) / res))   # y0 (bottom) → larger row index
        col0, col1 = max(0, col0), min(w, col1)
        row0, row1 = max(0, row0), min(h, row1)
        return img[row0:row1, col0:col1]

    slam_crop = crop(slam_img, slam_meta, ix_min, iy_min, ix_max, iy_max)
    gt_crop   = crop(gt_img,   gt_meta,   ix_min, iy_min, ix_max, iy_max)

    # Final shape reconciliation (rounding artefacts of ≤1 px)
    rh = min(slam_crop.shape[0], gt_crop.shape[0])
    rw = min(slam_crop.shape[1], gt_crop.shape[1])
    slam_crop = slam_crop[:rh, :rw]
    gt_crop   = gt_crop[:rh, :rw]

    overlap_m2 = (ix_max - ix_min) * (iy_max - iy_min)
    total_m2   = (sx_max - sx_min) * (sy_max - sy_min)
    print(f"[align] World overlap: {overlap_m2:.1f} m² of {total_m2:.1f} m² SLAM extent")

    return slam_crop, gt_crop


# ────────────────────────── coverage logic ──────────────────────────

def analyze_coverage(slam_map, ground_truth_map,
                     threshold_free=200, threshold_occupied=100):
    """
    Analyze coverage against ground-truth.

    Both maps must already be spatially aligned (same shape, same world extent).

    Args
    ----
    slam_map        : aligned SLAM image (uint8)
    ground_truth_map: aligned ground-truth image (uint8)
    threshold_free  : pixels >= this value are "free"
    threshold_occupied: pixels <= this value are "occupied"

    Returns
    -------
    dict with coverage statistics and BGR analysis_map for visualization.
    """
    # Ground-truth masks
    gt_reachable = ground_truth_map >= threshold_free

    # SLAM masks
    slam_discovered = slam_map >= threshold_free   # seen as free space

    # Coverage metrics
    total_reachable = int(np.sum(gt_reachable))
    discovered      = int(np.sum(slam_discovered & gt_reachable))
    missed          = total_reachable - discovered
    coverage_pct    = (discovered / total_reachable * 100) if total_reachable > 0 else 0.0

    # Colour visualization (BGR)
    vis = np.zeros((*slam_map.shape, 3), dtype=np.uint8)
    vis[:, :, 0] = ground_truth_map   # grey background
    vis[:, :, 1] = ground_truth_map
    vis[:, :, 2] = ground_truth_map

    vis[slam_discovered & gt_reachable] = [0, 200, 0]   # green  = discovered
    vis[gt_reachable & ~slam_discovered] = [0, 0, 200]   # red    = missed

    return {
        'coverage_pct':          coverage_pct,
        'total_reachable_cells': total_reachable,
        'discovered_cells':      discovered,
        'missed_cells':          missed,
        'analysis_map':          vis,
    }


# ─────────────────────────── CLI entry ──────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Analyze exploration coverage against ground-truth (spatially aligned).')
    parser.add_argument('slam_map',        help='SLAM-generated .pgm file')
    parser.add_argument('ground_truth_map', help='Ground-truth .pgm file')
    parser.add_argument('--output', default=None,
                        help='Save analysis visualization to PNG')
    parser.add_argument('--threshold-free', type=int, default=200,
                        help='Pixel value threshold for "free" (default: 200)')
    parser.add_argument('--threshold-occupied', type=int, default=100,
                        help='Pixel value threshold for "occupied" (default: 100)')

    args = parser.parse_args()

    # Load
    slam_img, slam_meta = load_map_with_meta(args.slam_map)
    gt_img,   gt_meta   = load_map_with_meta(args.ground_truth_map)

    print(f"[load] SLAM map:        {slam_img.shape[1]}x{slam_img.shape[0]} px  "
          f"res={slam_meta['resolution']}  origin={slam_meta['origin'][:2]}")
    print(f"[load] Ground-truth:    {gt_img.shape[1]}x{gt_img.shape[0]} px  "
          f"res={gt_meta['resolution']}  origin={gt_meta['origin'][:2]}")

    # Align in world coordinates
    try:
        slam_aligned, gt_aligned = align_maps(slam_img, slam_meta, gt_img, gt_meta)
    except ValueError as e:
        print(f"\nERROR: {e}")
        print("\nHint: Check that the 'origin' values in both .yaml files are correct.")
        sys.exit(1)

    # Compute coverage
    result = analyze_coverage(slam_aligned, gt_aligned,
                              args.threshold_free, args.threshold_occupied)

    # Report
    print("\n=== Coverage Analysis ===")
    print(f"Coverage: {result['coverage_pct']:.1f}%")
    print(f"Discovered: {result['discovered_cells']:,} / {result['total_reachable_cells']:,} cells")
    print(f"Missed: {result['missed_cells']:,} cells")
    print()
    pct = result['coverage_pct']
    if pct >= 90:
        print("  ✓ Excellent coverage (>90%)")
    elif pct >= 75:
        print("  ◐ Good coverage (75–90%)")
    elif pct >= 50:
        print("  ◑ Moderate coverage (50–75%)")
    else:
        print("  ✗ Poor coverage (<50%) — check map setup or explorer config")

    if args.output:
        cv2.imwrite(args.output, result['analysis_map'])
        print(f"\n✓ Visualization saved: {args.output}")
        print("  Green = discovered reachable space")
        print("  Red   = missed reachable space")
        print("  Grey  = obstacles / unknown / out-of-overlap")

    return 0 if result['coverage_pct'] >= 75 else 1


if __name__ == '__main__':
    sys.exit(main())
