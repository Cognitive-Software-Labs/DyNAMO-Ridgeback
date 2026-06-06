#!/usr/bin/env python3
"""
Compare any map image (screenshot, exported PNG, PGM) against a ground-truth PGM.

Unlike analyze_coverage.py this script does NOT require a .yaml companion —
it works on raw images by normalising both to {free, occupied, unknown} and
finding the best spatial alignment via ORB feature matching before computing
cell-level similarity.

Normalisation thresholds (adjustable via flags):
    pixel >= free_thresh     → free      (0)
    pixel <= occupied_thresh → occupied  (1)
    otherwise                → unknown   (-1)

Usage:
    python3 compare_image.py <explored_image> <ground_truth.pgm>
    python3 compare_image.py screenshot.png mock_hospital.pgm
    python3 compare_image.py screenshot.png mock_hospital.pgm --output result.png
    python3 compare_image.py screenshot.png mock_hospital.pgm --crop 0 30 0 0
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: opencv not available.  pip install opencv-python")
    sys.exit(1)


UNKNOWN  = -1
FREE     =  0
OCCUPIED =  1


# ──────────────────────── normalisation ──────────────────────────────────────

def normalise(img_gray, free_thresh=180, occupied_thresh=80):
    """
    Convert a grayscale image to a {-1, 0, 1} int8 grid.

    Works for both PGM maps and screenshots:
      - PGM:        254=free, 0=occupied, 127=unknown
      - Screenshot: light-gray=free, black=wall, dark-bg=unknown
    """
    grid = np.full(img_gray.shape, UNKNOWN, dtype=np.int8)
    grid[img_gray >= free_thresh]  = FREE
    grid[img_gray <= occupied_thresh] = OCCUPIED
    return grid


def crop_to_known(grid):
    """Return the bounding box of cells that are not unknown."""
    known = grid != UNKNOWN
    rows  = np.any(known, axis=1)
    cols  = np.any(known, axis=0)
    r0, r1 = np.where(rows)[0][[0, -1]]
    c0, c1 = np.where(cols)[0][[0, -1]]
    return grid[r0:r1+1, c0:c1+1]


# ──────────────────────── alignment ──────────────────────────────────────────

def align_by_features(src_gray, dst_gray):
    """
    Find a homography from src to dst using ORB keypoints.
    Returns the warped src image aligned to dst, or None if not enough matches.
    """
    orb = cv2.ORB_create(2000)
    kp1, des1 = orb.detectAndCompute(src_gray, None)
    kp2, des2 = orb.detectAndCompute(dst_gray, None)

    if des1 is None or des2 is None or len(kp1) < 4 or len(kp2) < 4:
        return None

    bf      = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    matches = bf.knnMatch(des1, des2, k=2)

    good = [m for m, n in matches if m.distance < 0.75 * n.distance]
    if len(good) < 8:
        return None

    src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    if H is None:
        return None

    h, w = dst_gray.shape
    warped = cv2.warpPerspective(src_gray, H, (w, h), flags=cv2.INTER_NEAREST,
                                 borderValue=127)
    return warped


def align_by_resize(src_gray, dst_gray):
    """Fallback: simply resize src to match dst dimensions."""
    return cv2.resize(src_gray, (dst_gray.shape[1], dst_gray.shape[0]),
                      interpolation=cv2.INTER_NEAREST)


# ──────────────────────── similarity metric ──────────────────────────────────

def compute_similarity(slam_grid, gt_grid):
    """
    Cell-by-cell comparison on cells where both grids have a known value.

    Returns a dict with all statistics.
    """
    both_known = (slam_grid != UNKNOWN) & (gt_grid != UNKNOWN)
    gt_free    = gt_grid == FREE

    diff = slam_grid.astype(np.int16) - gt_grid.astype(np.int16)

    agreed     = both_known & (diff == 0)
    false_occ  = both_known & (diff ==  1)   # slam=occ, gt=free
    false_free = both_known & (diff == -1)   # slam=free, gt=occ

    gt_free_known    = gt_free & (slam_grid != UNKNOWN)
    slam_found_free  = gt_free_known & (slam_grid == FREE)
    unexplored       = gt_free & (slam_grid == UNKNOWN)

    total      = int(both_known.sum())
    coverage   = (slam_found_free.sum() / gt_free_known.sum() * 100
                  if gt_free_known.sum() > 0 else 0.0)
    agreement  = agreed.sum() / total * 100 if total > 0 else 0.0

    return {
        'coverage_pct':     float(coverage),
        'agreement_pct':    float(agreement),
        'gt_free_cells':    int(gt_free.sum()),
        'slam_found_free':  int(slam_found_free.sum()),
        'unexplored':       int(unexplored.sum()),
        'false_occ':        int(false_occ.sum()),
        'false_free':       int(false_free.sum()),
        'both_known':       total,
        'agreed':           int(agreed.sum()),
        'diff':             diff,
    }


# ──────────────────────── visualisation ──────────────────────────────────────

def make_vis(slam_grid, gt_grid):
    h, w = gt_grid.shape
    vis  = np.full((h, w, 3), 180, dtype=np.uint8)
    vis[(gt_grid == FREE)  & (slam_grid == FREE)]  = [0, 200, 0]    # green  agreed free
    vis[(gt_grid == FREE)  & (slam_grid != FREE)]  = [0, 0, 200]    # red    missed
    vis[(gt_grid == OCCUPIED) & (slam_grid == OCCUPIED)] = [60, 60, 60]  # dark  agreed wall
    return vis


# ──────────────────────────────── main ───────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Compare any map image against a ground-truth PGM.'
    )
    parser.add_argument('explored_image',   help='Screenshot or exported map image')
    parser.add_argument('ground_truth_pgm', help='Ground-truth .pgm file')
    parser.add_argument('--output', default=None,
                        help='Save result PNG (default: <gt_name>_image_compare.png)')
    parser.add_argument('--free-thresh',     type=int, default=180,
                        help='Pixels >= this are FREE (default: 180)')
    parser.add_argument('--occupied-thresh', type=int, default=80,
                        help='Pixels <= this are OCCUPIED (default: 80)')
    parser.add_argument('--crop', nargs=4, type=int, metavar=('TOP','BOTTOM','LEFT','RIGHT'),
                        default=[0, 0, 0, 0],
                        help='Crop pixels from each edge of the explored image before processing '
                             '(useful to remove HUD / UI elements)')
    parser.add_argument('--no-feature-align', action='store_true',
                        help='Skip ORB feature matching and just resize (faster, less accurate)')
    args = parser.parse_args()

    # ── load ──
    exp_path = Path(args.explored_image).expanduser().resolve()
    gt_path  = Path(args.ground_truth_pgm).expanduser().resolve()

    exp_img = cv2.imread(str(exp_path), cv2.IMREAD_GRAYSCALE)
    gt_img  = cv2.imread(str(gt_path),  cv2.IMREAD_GRAYSCALE)

    if exp_img is None:
        print(f"ERROR: cannot load {exp_path}"); sys.exit(1)
    if gt_img is None:
        print(f"ERROR: cannot load {gt_path}");  sys.exit(1)

    # ── crop UI elements from screenshot ──
    t, b, l, r = args.crop
    b = exp_img.shape[0] - b if b > 0 else exp_img.shape[0]
    r = exp_img.shape[1] - r if r > 0 else exp_img.shape[1]
    exp_img = exp_img[t:b, l:r]

    print(f"Explored image : {exp_img.shape[1]}x{exp_img.shape[0]} px  ({exp_path.name})")
    print(f"Ground truth   : {gt_img.shape[1]}x{gt_img.shape[0]} px  ({gt_path.name})")

    # ── normalise both to {-1, 0, 1} ──
    exp_grid = normalise(exp_img, args.free_thresh, args.occupied_thresh)
    gt_grid  = normalise(gt_img,  args.free_thresh, args.occupied_thresh)

    print(f"\nNormalised explored image:")
    print(f"  free={int((exp_grid == FREE).sum()):,}  "
          f"occupied={int((exp_grid == OCCUPIED).sum()):,}  "
          f"unknown={int((exp_grid == UNKNOWN).sum()):,}")
    print(f"Normalised ground truth:")
    print(f"  free={int((gt_grid == FREE).sum()):,}  "
          f"occupied={int((gt_grid == OCCUPIED).sum()):,}  "
          f"unknown={int((gt_grid == UNKNOWN).sum()):,}")

    # ── align explored image onto ground truth ──
    aligned_img = None
    if not args.no_feature_align:
        print("\nAttempting ORB feature alignment…")
        aligned_img = align_by_features(exp_img, gt_img)
        if aligned_img is not None:
            print("  Feature alignment succeeded.")
        else:
            print("  Not enough feature matches — falling back to resize.")

    if aligned_img is None:
        print("  Using resize alignment.")
        aligned_img = align_by_resize(exp_img, gt_img)

    aligned_grid = normalise(aligned_img, args.free_thresh, args.occupied_thresh)

    # ── compare ──
    s = compute_similarity(aligned_grid, gt_grid)

    pct     = s['coverage_pct']
    bar_len = 40
    filled  = int(bar_len * pct / 100)
    bar     = '█' * filled + '░' * (bar_len - filled)
    status  = ('✓ EXCELLENT' if pct >= 90 else
               '◐ GOOD'      if pct >= 75 else
               '◑ MODERATE'  if pct >= 50 else
               '✗ LOW')

    result = (
        f"\n{'='*60}\n"
        f"  SIMILARITY RESULT\n"
        f"  explored : {exp_path.name}\n"
        f"  gt       : {gt_path.name}\n"
        f"{'='*60}\n"
        f"\n  [{bar}] {pct:5.1f}%  {status}\n"
        f"\n  Coverage of ground-truth free space\n"
        f"    Found free  : {s['slam_found_free']:>8,}  /  {s['gt_free_cells']:,} cells\n"
        f"    Unexplored  : {s['unexplored']:>8,} cells\n"
        f"\n  Cell-level agreement  (explored - gt,  known cells only)\n"
        f"    Agreed (diff=0)         : {s['agreed']:>8,}  /  {s['both_known']:,}"
        f"  →  {s['agreement_pct']:.1f}%\n"
        f"    diff=+1 (exp=wall, gt=free)  : {s['false_occ']:>8,}\n"
        f"    diff=-1 (exp=free, gt=wall)  : {s['false_free']:>8,}\n"
        f"{'='*60}"
    )

    print(result)

    # ── save results ──
    out_txt = gt_path.with_name(gt_path.stem + '_image_compare.txt')
    out_txt.write_text(result)
    print(f"\nResult saved → {out_txt}")

    out_png = Path(args.output).expanduser().resolve() if args.output else \
              gt_path.with_name(gt_path.stem + '_image_compare.png')
    vis = make_vis(aligned_grid, gt_grid)
    cv2.imwrite(str(out_png), vis)
    print(f"Visual saved  → {out_png}")
    print("  green = both say free  |  red = gt=free but exp missed  |  dark = both say wall")


if __name__ == '__main__':
    main()
