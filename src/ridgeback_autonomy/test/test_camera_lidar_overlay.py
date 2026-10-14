"""Camera-LiDAR overlay check, on a synthetic scene with known geometry."""
from importlib.machinery import SourceFileLoader
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[3]
WIDTH, HEIGHT = 640, 480
INFO = SimpleNamespace(k=[380.0, 0.0, 320.0, 0.0, 380.0, 240.0, 0.0, 0.0, 1.0],
                       width=WIDTH, height=HEIGHT)
# LiDAR body frame (x forward, y left, z up) -> camera optical frame; the scan
# plane sits 0.8 m below the camera and 0.1 m behind it.
LIDAR_TO_OPTICAL = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
LIDAR_IN_OPTICAL = np.array([0.0, 0.8, -0.1])


def load_tool():
    loader = SourceFileLoader('camera_lidar_overlay', str(ROOT / 'tools/camera_lidar_overlay'))
    spec = importlib.util.spec_from_loader('camera_lidar_overlay', loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


TOOL = load_tool()


def wall_distance(x, y):
    """Optical-frame Z of a zig-zag back wall, so lateral shifts change depth."""

    return 3.0 + 0.6 * np.abs(((x / 0.5) % 2.0) - 1.0)


def scene_depth():
    """Aligned depth of the wall: solve Z = wall(X) along each pixel ray."""

    u = np.arange(WIDTH, dtype=np.float64)
    ray_x = (u - INFO.k[2]) / INFO.k[0]
    z = np.full(WIDTH, 3.3)
    for _ in range(60):
        z = wall_distance(ray_x * z, None)
    return np.tile(z, (HEIGHT, 1))


def scan_of_wall(yaw_error_deg=0.0):
    """A LaserScan whose beams hit the wall, measured with a yaw-rotated mount."""

    angles = np.radians(np.arange(-60.0, 60.0, 0.25))
    ranges = []
    for angle in angles:
        true_angle = angle + math.radians(yaw_error_deg)
        direction = LIDAR_TO_OPTICAL @ np.array([math.cos(true_angle), math.sin(true_angle), 0.0])
        t = 3.3
        for _ in range(60):
            point = LIDAR_IN_OPTICAL + t * direction
            t = (wall_distance(point[0], None) - LIDAR_IN_OPTICAL[2]) / direction[2]
        ranges.append(t)
    return SimpleNamespace(ranges=ranges, angle_min=float(angles[0]),
                           angle_increment=math.radians(0.25), range_min=0.05, range_max=30.0)


def project(scan):
    return TOOL.scan_points_optical(scan, LIDAR_TO_OPTICAL, LIDAR_IN_OPTICAL)


def test_aligned_scan_lands_on_the_depth_surface():
    points, valid = project(scan_of_wall())
    intrinsics = TOOL.intrinsics_from_camera_info(INFO)
    *_, residual = TOOL.depth_residuals(points, valid, intrinsics, scene_depth())
    summary = TOOL.residual_summary(residual)
    assert summary['beams'] > 100
    assert summary['inlier_fraction'] > 0.95
    assert abs(summary['median_m']) < 0.01
    assert TOOL.yaw_sweep(points, valid, intrinsics, scene_depth())['best_yaw_deg'] == 0.0


@pytest.mark.parametrize('error_deg', [1.5, -1.5])
def test_yaw_sweep_recovers_a_mount_yaw_error(error_deg):
    points, valid = project(scan_of_wall(yaw_error_deg=error_deg))
    intrinsics = TOOL.intrinsics_from_camera_info(INFO)
    *_, residual = TOOL.depth_residuals(points, valid, intrinsics, scene_depth())
    assert TOOL.residual_summary(residual)['inlier_fraction'] < 0.9
    best = TOOL.yaw_sweep(points, valid, intrinsics, scene_depth())['best_yaw_deg']
    assert best == pytest.approx(error_deg, abs=0.25)


def test_positive_yaw_moves_points_to_image_left():
    ahead = np.array([[0.0, 0.0, 2.0]])
    turned = TOOL.yaw_about_optical_y(ahead, math.radians(10.0))
    assert turned[0, 0] < 0.0
    assert turned[0, 2] == pytest.approx(2.0 * math.cos(math.radians(10.0)))


def test_missing_depth_is_reported_not_counted_as_agreement():
    points, valid = project(scan_of_wall())
    intrinsics = TOOL.intrinsics_from_camera_info(INFO)
    *_, residual = TOOL.depth_residuals(points, valid, intrinsics, np.zeros((HEIGHT, WIDTH)))
    summary = TOOL.residual_summary(residual)
    assert summary['with_depth'] == 0 and summary['inlier_fraction'] is None


def test_decoders_handle_row_padding_and_channel_order():
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    rgb[..., 0] = 200  # red
    padded = np.zeros((2, 12), dtype=np.uint8)
    padded[:, :9] = rgb.reshape(2, 9)
    bgr = TOOL.decode_color('rgb8', 2, 3, 12, padded.tobytes())
    assert bgr.shape == (2, 3, 3) and (bgr[..., 2] == 200).all() and (bgr[..., 0] == 0).all()
    depth = np.array([[1000, 2500, 0, 0]], dtype=np.uint16)
    assert TOOL.decode_depth_m('16UC1', 1, 2, 8, depth.tobytes()).tolist() == [[1.0, 2.5]]
