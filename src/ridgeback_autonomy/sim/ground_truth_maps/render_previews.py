#!/usr/bin/env python3
"""Render PNG previews of the ground-truth maps.

PGM occupancy maps don't preview in most image viewers or on GitHub. This
converts every ``<world>.pgm`` in the package maps dir to a same-named
``<world>.png`` next to it (committed so they show up on GitHub). Re-run after
editing or adding a map.

Usage (from the repo root):
    python3 src/ridgeback_autonomy/sim/ground_truth_maps/render_previews.py
    python3 src/ridgeback_autonomy/sim/ground_truth_maps/render_previews.py --maps-dir <dir>
"""

import argparse
from pathlib import Path

from PIL import Image

# This script lives in the maps dir; default to its own location.
DEFAULT_MAPS_DIR = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser(description='Render PNG previews of ground-truth maps')
    ap.add_argument('--maps-dir', type=Path, default=DEFAULT_MAPS_DIR,
                    help='Directory of <world>.pgm maps (default: package maps dir)')
    args = ap.parse_args()

    pgms = sorted(args.maps_dir.glob('*.pgm'))
    if not pgms:
        print(f'No .pgm files in {args.maps_dir}')
        return

    for pgm in pgms:
        png = pgm.with_suffix('.png')
        Image.open(pgm).save(png)
        print(f'{pgm.name} -> {png.name}')


if __name__ == '__main__':
    main()
