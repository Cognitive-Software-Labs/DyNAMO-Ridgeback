#!/usr/bin/env python3
"""Compare a tape survey of the physical robot with the Nav2 footprint (S8).

Usage:
  python3 tools/hardware/compare_measured_outline.py SURVEY.yaml --output DIR

The survey gives the hull's overall length and width, and each point's
horizontal distance back from the front face and in from the left face (both
measured square to the hull; a point outside the left face is negative), its
height above the floor and its tape uncertainty. The four face midpoints are
checked from the hull size; survey the rounded corners and every attachment. base_link is taken as the
hull's centre, with HULL_CENTRE_UNCERTAINTY_M added for the model's 9 mm
front/rear asymmetry. A point passes when it lies inside the footprint plus the
configured padding by at least its combined uncertainty. Read-only.
"""
import argparse
import json
import math
from pathlib import Path
import re
import sys

import yaml

ROOT = Path(__file__).resolve().parents[2]
NAV2 = ROOT / 'src/ridgeback_autonomy/config/nav2_params.yaml'
HULL_CENTRE_UNCERTAINTY_M = 0.005


def footprint(nav2_text):
    """Local costmap footprint polygon and padding from nav2_params.yaml text."""

    params = yaml.safe_load(nav2_text)['local_costmap']['local_costmap']['ros__parameters']
    points = [tuple(map(float, pair)) for pair in
              re.findall(r'\[\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*\]', params['footprint'])]
    return points, float(params.get('footprint_padding', 0.0))


def to_base_link(hull, point):
    x = hull['length_m'] / 2 - point['from_front_m']
    y = hull['width_m'] / 2 - point['from_left_m']
    return x, y


def signed_margin(polygon, x, y):
    """Distance to the polygon boundary: positive inside, negative outside."""

    inside = False
    nearest = math.inf
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
        nearest = min(nearest, math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)))
    return nearest if inside else -nearest


def evaluate(survey, polygon, padding):
    hull = survey['hull']
    hull_u = float(hull.get('uncertainty_m', 0.0))
    rows = []
    # Face midpoints only: the hull's corners are rounded (the footprint chamfers
    # follow them), so its bounding-box corners are not points on the robot.
    # Survey the rounded corners as ordinary points.
    corners = [('hull front', 0.0, hull['width_m'] / 2), ('hull rear', hull['length_m'], hull['width_m'] / 2),
               ('hull left', hull['length_m'] / 2, 0.0), ('hull right', hull['length_m'] / 2, hull['width_m'])]
    points = [{'name': n, 'from_front_m': f, 'from_left_m': s, 'height_m': None, 'uncertainty_m': hull_u}
              for n, f, s in corners] + list(survey.get('points', []))
    for point in points:
        x, y = to_base_link(hull, point)
        u = math.hypot(float(point.get('uncertainty_m', 0.0)), hull_u / 2, HULL_CENTRE_UNCERTAINTY_M)
        margin = signed_margin(polygon, x, y) + padding
        rows.append({'name': point['name'], 'x_m': round(x, 4), 'y_m': round(y, 4),
                     'height_m': point.get('height_m'), 'margin_m': round(margin, 4),
                     'uncertainty_m': round(u, 4), 'passed': margin - u >= 0.0})
    return rows


def plot(rows, polygon, padding, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(figsize=(7, 6))
    xs, ys = zip(*(polygon + polygon[:1]))
    axes.plot(xs, ys, 'k-', label='Nav2 footprint')
    for row in rows:
        colour = 'tab:green' if row['passed'] else 'tab:red'
        axes.add_patch(plt.Circle((row['x_m'], row['y_m']), row['uncertainty_m'], color=colour, alpha=0.3))
        axes.plot(row['x_m'], row['y_m'], 'o', color=colour)
        axes.annotate(row['name'], (row['x_m'], row['y_m']), fontsize=7, xytext=(3, 3),
                      textcoords='offset points')
    axes.set_aspect('equal')
    axes.set_xlabel('base_link x (m, forward)')
    axes.set_ylabel('base_link y (m, left)')
    axes.set_title(f'Measured points vs footprint (padding {padding * 1000:.0f} mm)')
    axes.legend(loc='lower right')
    figure.savefig(path, dpi=150, bbox_inches='tight')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('survey', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    survey = yaml.safe_load(args.survey.read_text())
    polygon, padding = footprint(NAV2.read_text())
    rows = evaluate(survey, polygon, padding)
    args.output.mkdir(parents=True, exist_ok=False)
    for row in rows:
        print(f"{'PASS' if row['passed'] else 'FAIL'}  {row['name']:24} x {row['x_m']:+.3f} y {row['y_m']:+.3f}  "
              f"margin {row['margin_m'] * 1000:+.0f} mm (uncertainty {row['uncertainty_m'] * 1000:.0f} mm)")
    passed = all(row['passed'] for row in rows)
    print('OUTLINE PASS' if passed else 'OUTLINE FAIL')
    (args.output / 'report.json').write_text(json.dumps(
        {'survey': survey, 'padding_m': padding, 'footprint': polygon, 'rows': rows, 'passed': passed}, indent=1))
    plot(rows, polygon, padding, args.output / 'outline.png')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
