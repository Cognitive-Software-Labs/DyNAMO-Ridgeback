"""Cut the G1 heading-sweep regression fixture out of the 2026-11-01 Gazebo captures.

Usage: make_heading_sweep_fixture.py FRAMES_DIR OUT.npz

FRAMES_DIR is artifacts/exploration/g1_heading_2026-11-01/frames: 72 views of a G1 at
(3, 0) turned in 15 deg steps, from three robot poses (far 2.7 m, near 1.6 m, oblique
1.9 m from the camera), with each view's colour and depth frame and ``index.json``. The
robot camera is level, 0.28 m ahead of base_link and 1.0345 m up (the 2026-11-01
evaluate.py there used the same transform). For each view this keeps one depth frame in
millimetres with everything farther than 0.7 m from G1's axis zeroed (the estimator reads
0.6 m around it), cropped to what is left; the intrinsics shifted to the crop; how many
image rows lie below the crop (the image's bottom row sets how low the camera sees); the
camera pose in the map; and G1's ground-truth yaw, which the tests use only to measure
the heading error.
"""
import json
import math
import os
import sys

import numpy as np

CAMERA_IN_BASE = (0.28035, -0.0115, 1.0345)
G1_XY = (3.0, 0.0)
KEEP_RADIUS_M = 0.7


def main():
    frames, out_path = sys.argv[1], sys.argv[2]
    out = {}
    for row in json.load(open(os.path.join(frames, 'index.json'))):
        name = row['name']
        capture = np.load(os.path.join(frames, name + '.npz'))
        k = capture['K']
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        x, y, yaw = row['robot']
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[s, 0.0, c], [-c, 0.0, s], [0.0, -1.0, 0.0]])   # level optical frame
        translation = np.array([x + c * CAMERA_IN_BASE[0] - s * CAMERA_IN_BASE[1],
                                y + s * CAMERA_IN_BASE[0] + c * CAMERA_IN_BASE[1], CAMERA_IN_BASE[2]])
        depth = np.asarray(capture['depth'], dtype=float)
        v, u = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
        valid = np.isfinite(depth) & (depth > 0.0)
        z = np.where(valid, depth, 0.0)
        points = np.stack(((u - cx) * z / fx, (v - cy) * z / fy, z), axis=-1) @ rotation.T + translation
        keep = valid & (np.hypot(points[..., 0] - G1_XY[0], points[..., 1] - G1_XY[1]) <= KEEP_RADIUS_M)
        rows, cols = np.nonzero(keep)
        v0, v1, u0, u1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
        out[f'{name}_depth_mm'] = np.where(keep, np.round(depth * 1000.0), 0)[v0:v1, u0:u1].astype(np.uint16)
        out[f'{name}_intrinsics'] = np.array([fx, fy, cx - u0, cy - v0])
        out[f'{name}_image_rows_below'] = np.array(depth.shape[0] - v1)
        out[f'{name}_camera_rotation'] = rotation
        out[f'{name}_camera_translation'] = translation
        out[f'{name}_g1_yaw_deg'] = np.array(float(row['g1_yaw_deg']))
    np.savez_compressed(out_path, **out)
    print(f'{out_path}: {len(out) // 6} views, {os.path.getsize(out_path) / 1e3:.0f} kB')


if __name__ == '__main__':
    main()
