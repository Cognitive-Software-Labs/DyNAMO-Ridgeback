#!/usr/bin/env python3
"""
Live coverage monitor — subscribes to the SLAM occupancy grid and prints a
final cell-by-cell comparison against a hand-authored ground-truth map when
the user presses Ctrl+C (i.e. when exploration is done).

Both grids are normalised to the same value space:
    -1  unknown
     0  free
     1  occupied

They are spatially aligned in world coordinates (via their .yaml metadata)
and subtracted.  Where both maps agree the difference is 0; disagreements ±1.
Coverage = fraction of ground-truth-free cells the SLAM run also marked free.

Usage (source your workspace first):
    python3 live_coverage_monitor.py <ground_truth.pgm>
    python3 live_coverage_monitor.py <ground_truth.pgm> --topic /r100_0001/map
    python3 live_coverage_monitor.py <ground_truth.pgm> --save-image /tmp/result.png

Press Ctrl+C when exploration is finished — final stats are printed then.

Hospital example:
    python3 live_coverage_monitor.py \\
        ~/Documents/DyNAMO/DyNAMO-Ridgeback/config/ground_truth_maps/mock_hospital.pgm
"""

import argparse
import sys
import threading
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: opencv not available.  pip install opencv-python")
    sys.exit(1)

try:
    import yaml as _yaml
except ImportError:
    print("ERROR: pyyaml not available.  pip install pyyaml")
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid
except ImportError:
    print("ERROR: rclpy not found — source your ROS 2 workspace first.")
    sys.exit(1)


# ─────────────────────── normalised grid values ──────────────────────────────
UNKNOWN  = -1
FREE     =  0
OCCUPIED =  1


# ──────────────────────── map loading / conversion ───────────────────────────

def pgm_to_grid(pgm_path: Path):
    """
    Load a PGM + YAML and return a normalised int8 grid and meta dict.

    PGM pixel convention (ROS default, negate=0):
        254 / 205  → free     → 0
          0        → occupied → 1
        127        → unknown  → -1
    Threshold: pixels >= 200 are free, <= 50 are occupied, rest unknown.
    """
    img = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Cannot load: {pgm_path}")

    yaml_path = pgm_path.with_suffix('.yaml')
    if not yaml_path.exists():
        raise FileNotFoundError(f"No companion .yaml: {yaml_path}")
    with open(yaml_path) as f:
        meta = _yaml.safe_load(f)

    grid = np.full(img.shape, UNKNOWN, dtype=np.int8)
    grid[img >= 200] = FREE
    grid[img <= 50]  = OCCUPIED

    return grid, meta


def occupancy_msg_to_grid(msg: OccupancyGrid):
    """
    Convert a nav_msgs/OccupancyGrid to a normalised int8 grid and meta dict.

    OccupancyGrid values:
        -1   → UNKNOWN
         0   → FREE
        100  → OCCUPIED
        1-99 → treated as OCCUPIED (partially occupied)

    The data array is row-major with row 0 at world y_min (bottom of world).
    After reshape we flipud so row 0 = world y_max (top), matching the PGM
    convention assumed by the alignment code.
    """
    w, h = msg.info.width, msg.info.height
    raw = np.array(msg.data, dtype=np.int8).reshape((h, w))

    grid = np.full((h, w), UNKNOWN, dtype=np.int8)
    grid[raw == 0]  = FREE
    grid[raw > 0]   = OCCUPIED   # 1–100 all treated as occupied

    grid = np.flipud(grid)        # bottom-first → top-first

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
    """
    Crop both grids to their overlapping world region at the ground-truth
    resolution.  Returns (slam_crop, gt_crop) or (None, None) if no overlap.
    """
    res = gt_meta['resolution']

    # Resample SLAM grid if its resolution differs
    if not np.isclose(slam_meta['resolution'], res, rtol=0.01):
        scale   = slam_meta['resolution'] / res
        new_w   = int(round(slam_grid.shape[1] * scale))
        new_h   = int(round(slam_grid.shape[0] * scale))
        # Nearest-neighbour so we don't invent intermediate occupancy values
        slam_f  = slam_grid.astype(np.float32)
        slam_f  = cv2.resize(slam_f, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        slam_grid = slam_f.astype(np.int8)
        slam_meta = dict(slam_meta, resolution=res)

    sx0, sy0, sx1, sy1 = world_extent(slam_grid, slam_meta)
    gx0, gy0, gx1, gy1 = world_extent(gt_grid,   gt_meta)

    ix0 = max(sx0, gx0);  iy0 = max(sy0, gy0)
    ix1 = min(sx1, gx1);  iy1 = min(sy1, gy1)

    if ix0 >= ix1 or iy0 >= iy1:
        return None, None

    def crop(grid, meta, x0, y0, x1, y1):
        ox, oy = meta['origin'][0], meta['origin'][1]
        h, w   = grid.shape
        c0 = max(0, int(round((x0 - ox) / res)))
        c1 = min(w, int(round((x1 - ox) / res)))
        r0 = max(0, h - int(round((y1 - oy) / res)))
        r1 = min(h, h - int(round((y0 - oy) / res)))
        return grid[r0:r1, c0:c1]

    sc = crop(slam_grid, slam_meta, ix0, iy0, ix1, iy1)
    gc = crop(gt_grid,   gt_meta,   ix0, iy0, ix1, iy1)

    rh = min(sc.shape[0], gc.shape[0])
    rw = min(sc.shape[1], gc.shape[1])
    return sc[:rh, :rw], gc[:rh, :rw]


# ──────────────────────── coverage metric ────────────────────────────────────

def compute_stats(slam_crop, gt_crop):
    """
    Compare two normalised grids cell-by-cell.

    Only cells where BOTH grids have a known value (not -1) contribute to the
    agreement metrics.  Coverage is measured over ground-truth FREE cells only.

    Returns a dict with all statistics.
    """
    diff = slam_crop.astype(np.int16) - gt_crop.astype(np.int16)

    # Masks
    gt_known   = gt_crop   != UNKNOWN
    slam_known = slam_crop != UNKNOWN
    both_known = gt_known & slam_known

    gt_free     = gt_crop == FREE
    gt_occupied = gt_crop == OCCUPIED

    # Coverage: how much of gt-free did slam also mark free?
    gt_free_known   = gt_free & slam_known          # gt says free AND slam has an opinion
    slam_found_free = gt_free_known & (slam_crop == FREE)

    coverage_pct = (
        slam_found_free.sum() / gt_free_known.sum() * 100
        if gt_free_known.sum() > 0 else 0.0
    )

    # Overall agreement on known cells
    agreed     = both_known & (diff == 0)
    agreement_pct = agreed.sum() / both_known.sum() * 100 if both_known.sum() > 0 else 0.0

    # Disagreement breakdown (on cells where diff != 0 and both known)
    disagreed  = both_known & (diff != 0)
    # slam=free where gt=occupied  → diff = 0 - 1 = -1  (slam mapped hallway, gt says wall)
    false_free = both_known & (diff == -1)
    # slam=occupied where gt=free  → diff = 1 - 0 = +1  (slam mapped wall, gt says hallway)
    false_occ  = both_known & (diff ==  1)

    # Unknown in slam over gt-free area = unexplored reachable space
    unexplored = gt_free & (slam_crop == UNKNOWN)

    return {
        'coverage_pct':        coverage_pct,
        'agreement_pct':       agreement_pct,
        'gt_free_cells':       int(gt_free.sum()),
        'slam_found_free':     int(slam_found_free.sum()),
        'unexplored_cells':    int(unexplored.sum()),
        'false_free_cells':    int(false_free.sum()),
        'false_occ_cells':     int(false_occ.sum()),
        'both_known_cells':    int(both_known.sum()),
        'agreed_cells':        int(agreed.sum()),
        'diff':                diff,
    }


def make_visualization(slam_crop, gt_crop, diff):
    """
    BGR image:
        green   slam=free  & gt=free   (correctly explored)
        red     slam≠free  & gt=free   (missed / wrong)
        blue    slam=occ   & gt=occ    (wall agreed)
        grey                           (unknown / outside overlap)
    """
    h, w = gt_crop.shape
    vis  = np.full((h, w, 3), 180, dtype=np.uint8)  # grey background

    both_free = (gt_crop == FREE)  & (slam_crop == FREE)
    missed    = (gt_crop == FREE)  & (slam_crop != FREE)
    both_occ  = (gt_crop == OCCUPIED) & (slam_crop == OCCUPIED)

    vis[both_free] = [0, 200, 0]    # green
    vis[missed]    = [0, 0, 200]    # red
    vis[both_occ]  = [80, 80, 80]   # dark grey (wall)

    return vis


# ──────────────────────────── ROS 2 node ─────────────────────────────────────

class CoverageMonitor(Node):

    def __init__(self, gt_grid, gt_meta, map_topic: str, save_image):
        super().__init__('live_coverage_monitor')

        self._gt_grid   = gt_grid
        self._gt_meta   = gt_meta
        self._save_image = save_image

        self._latest_msg = None
        self._lock       = threading.Lock()

        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, 1)

        print(
            f"Monitoring {map_topic}\n"
            f"  GT: {gt_grid.shape[1]}x{gt_grid.shape[0]} cells  "
            f"res={gt_meta['resolution']}  origin={gt_meta['origin'][:2]}\n"
            f"  GT free cells: {int((gt_grid == FREE).sum()):,}\n\n"
            f"Press Ctrl+C when exploration is done to see final results."
        )

    def _map_cb(self, msg: OccupancyGrid):
        with self._lock:
            self._latest_msg = msg

    def compute_final(self):
        """Called on shutdown — runs one final comparison on the latest map."""
        with self._lock:
            msg = self._latest_msg

        if msg is None:
            print("\nNo map was received — nothing to compare.")
            return

        slam_grid, slam_meta = occupancy_msg_to_grid(msg)
        slam_crop, gt_crop   = align_grids(slam_grid, slam_meta, self._gt_grid, self._gt_meta)

        if slam_crop is None:
            print("\nMaps never overlapped — check topic and ground-truth origin.")
            return

        s = compute_stats(slam_crop, gt_crop)

        pct     = s['coverage_pct']
        bar_len = 40
        filled  = int(bar_len * pct / 100)
        bar     = '█' * filled + '░' * (bar_len - filled)
        status  = ('✓ EXCELLENT' if pct >= 90 else
                   '◐ GOOD'      if pct >= 75 else
                   '◑ MODERATE'  if pct >= 50 else
                   '✗ LOW')

        print(
            f"\n{'='*60}\n"
            f"  FINAL COVERAGE RESULT\n"
            f"{'='*60}\n"
            f"\n  [{bar}] {pct:5.1f}%  {status}\n"
            f"\n  Coverage of ground-truth free space\n"
            f"    Autonomously found free : {s['slam_found_free']:>8,}  /  {s['gt_free_cells']:,} cells\n"
            f"    Still unexplored        : {s['unexplored_cells']:>8,} cells\n"
            f"\n  Cell-level agreement  (slam - gt,  on known cells)\n"
            f"    Agreed  (diff = 0)      : {s['agreed_cells']:>8,}  /  {s['both_known_cells']:,}"
            f"  →  {s['agreement_pct']:.1f}%\n"
            f"    diff = +1  (slam=wall, gt=free)  : {s['false_occ_cells']:>8,}\n"
            f"    diff = -1  (slam=free, gt=wall)  : {s['false_free_cells']:>8,}\n"
            f"\n  SLAM map size : {slam_grid.shape[1]}x{slam_grid.shape[0]} cells  "
            f"res={slam_meta['resolution']:.3f}\n"
            f"{'='*60}"
        )

        if self._save_image is not None:
            vis = make_visualization(slam_crop, gt_crop, s['diff'])
            cv2.imwrite(str(self._save_image), vis)
            print(
                f"\n  Visualization saved → {self._save_image}\n"
                f"    green = autonomously discovered free space\n"
                f"    red   = missed (gt=free, slam did not find it)\n"
                f"    grey  = walls / unknown"
            )


# ──────────────────────────────── CLI ────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Live coverage monitor: cell-by-cell SLAM vs ground-truth comparison.'
    )
    parser.add_argument('ground_truth',
                        help='Path to ground-truth .pgm (companion .yaml must exist)')
    parser.add_argument('--topic', default='/r100_0001/map',
                        help='OccupancyGrid topic (default: /r100_0001/map)')
    parser.add_argument('--output', default=None,
                        help='Path to save the result text file (default: next to ground_truth .pgm)')
    parser.add_argument('--save-image', metavar='FILE', default=None,
                        help='Path to save a PNG visualization of the final coverage')
    args = parser.parse_args()

    gt_path = Path(args.ground_truth).expanduser().resolve()
    gt_grid, gt_meta = pgm_to_grid(gt_path)

    # Default output file sits next to the ground-truth PGM
    out_path = Path(args.output).expanduser().resolve() if args.output else \
               gt_path.with_name(gt_path.stem + '_coverage_result.txt')

    save_image = Path(args.save_image).expanduser().resolve() if args.save_image else \
                 gt_path.with_name(gt_path.stem + '_coverage_result.png')

    rclpy.init()
    node = CoverageMonitor(gt_grid, gt_meta, args.topic, save_image)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Run final comparison and capture output to file
        import io, contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            node.compute_final()
        result_text = buf.getvalue()

        print(result_text)                        # also show in terminal
        out_path.write_text(result_text)
        print(f"\nResult saved to: {out_path}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
