"""Cut the regression fixture for G1 localization out of capture_g1_views.py recordings.

Usage: make_g1_view_fixture.py OUT.npz NAME=CAPTURE.npz[:SX,SY[:X0,Y0,X1,Y1]] ...
For each view it keeps one depth frame cropped around G1's detection box (with the
intrinsics shifted to the crop), the camera pose, the box, the surface estimate the
mission logged for that view (SX, SY; omitted for none) and G1's ground truth, which
the tests use only to measure the error. A box given on the command line replaces a
capture without one (the detector's box from the original run at the same pose).
"""
import sys

import numpy as np

MARGIN_X, MARGIN_ABOVE, MARGIN_BELOW = 40, 20, 80


def main():
    out, views = sys.argv[1], {}
    for arg in sys.argv[2:]:
        name, spec = arg.split('=', 1)
        parts = spec.split(':')
        d = np.load(parts[0])
        surface = [float(v) for v in parts[1].split(',')] if len(parts) > 1 and parts[1] else [np.nan, np.nan]
        box = ([float(v) for v in parts[2].split(',')] if len(parts) > 2
               else [float(v) for v in d['boxes'][-1]])
        depth = d['depth_mm'][-1]
        x0 = max(0, int(box[0]) - MARGIN_X)
        x1 = min(depth.shape[1], int(np.ceil(box[2])) + MARGIN_X)
        y0 = max(0, int(box[1]) - MARGIN_ABOVE)
        y1 = min(depth.shape[0], int(np.ceil(box[3])) + MARGIN_BELOW)
        fx, fy, cx, cy = d['intrinsics']
        views[f'{name}_depth_mm'] = depth[y0:y1, x0:x1]
        views[f'{name}_intrinsics'] = np.array([fx, fy, cx - x0, cy - y0])
        views[f'{name}_box'] = np.array([box[0] - x0, box[1] - y0, box[2] - x0, box[3] - y0])
        views[f'{name}_camera_rotation'] = d['camera_rotation']
        views[f'{name}_camera_translation'] = d['camera_translation']
        views[f'{name}_surface'] = np.array(surface)
        views[f'{name}_g1_truth'] = d['g1_truth'][:2]
    np.savez_compressed(out, **views)


if __name__ == '__main__':
    main()
