from __future__ import annotations

import math

import numpy as np
import pytest
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from ridgeback_common.models import Detection, DetectionBatch
from ridgeback_localization.core.pointcloud_ranging import (
    add_pointcloud_measurements,
    compute_pointcloud_measurement,
    extract_organized_xyz,
    focus_bbox,
)


def _flat_cloud(points: list[tuple[float, float, float]]):
    return point_cloud2.create_cloud_xyz32(Header(), points)


def test_extract_organized_xyz_recovers_isaac_row_major_camera_cloud(
) -> None:
    points = [(float(i), float(i + 1), float(i + 2)) for i in range(6)]
    cloud = _flat_cloud(points)

    xyz = extract_organized_xyz(cloud, (2, 3))

    assert xyz.shape == (2, 3, 3)
    assert xyz[0, 0] == pytest.approx(points[0])
    assert xyz[1, 2] == pytest.approx(points[-1])


def test_extract_organized_xyz_rejects_arbitrary_flat_cloud() -> None:
    cloud = _flat_cloud([(1.0, 2.0, 3.0)] * 5)

    with pytest.raises(ValueError, match='does not match RGB image'):
        extract_organized_xyz(cloud, (2, 3))


def test_focus_bbox_crops_to_the_upper_centre_patch() -> None:
    assert focus_bbox((10, 20, 110, 220)) == (34, 56, 86, 144)


def test_compute_pointcloud_measurement_returns_nearest_inlier() -> None:
    pointcloud_xyz = np.zeros((4, 4, 3), dtype=np.float32)
    pointcloud_xyz[:, :, 0] = 2.0
    pointcloud_xyz[:, :, 1] = np.array([
        [0.20, 0.10, 0.00, -0.10],
        [0.15, 0.05, -0.05, -0.15],
        [0.20, 0.10, 0.00, -0.10],
        [0.15, 0.05, -0.05, -0.15],
    ], dtype=np.float32)

    measurement = compute_pointcloud_measurement(
        pointcloud_xyz,
        bbox_xyxy=(0, 0, 4, 4),
        rotation=np.eye(3, dtype=np.float32),
        translation=np.zeros(3, dtype=np.float32),
    )

    assert measurement is not None
    lateral_m, forward_m, distance_m = measurement
    assert abs(lateral_m) <= 0.05
    assert math.isclose(forward_m, 1.75, rel_tol=1e-5)
    assert distance_m <= 1.76


def test_compute_pointcloud_measurement_supports_direct_cloud_coordinates() -> None:
    pointcloud_xyz = np.zeros((4, 4, 3), dtype=np.float32)
    pointcloud_xyz[:, :, 0] = 2.0
    pointcloud_xyz[:, :, 1] = np.array([
        [0.20, 0.10, 0.00, -0.10],
        [0.15, 0.05, -0.05, -0.15],
        [0.20, 0.10, 0.00, -0.10],
        [0.15, 0.05, -0.05, -0.15],
    ], dtype=np.float32)

    measurement = compute_pointcloud_measurement(
        pointcloud_xyz,
        bbox_xyxy=(0, 0, 4, 4),
        rotation=None,
        translation=None,
    )

    assert measurement is not None
    lateral_m, forward_m, distance_m = measurement
    assert abs(lateral_m) <= 0.05
    assert math.isclose(forward_m, 1.75, rel_tol=1e-5)
    assert distance_m <= 1.76


def test_add_pointcloud_measurements_falls_back_when_transform_path_rejects_points() -> None:
    pointcloud_xyz = np.zeros((4, 4, 3), dtype=np.float32)
    pointcloud_xyz[:, :, 0] = 2.0
    pointcloud_xyz[:, :, 1] = np.array([
        [0.20, 0.10, 0.00, -0.10],
        [0.15, 0.05, -0.05, -0.15],
        [0.20, 0.10, 0.00, -0.10],
        [0.15, 0.05, -0.05, -0.15],
    ], dtype=np.float32)
    batch = DetectionBatch(
        image_width=4,
        image_height=4,
        detections=[Detection(bbox_xyxy=(0, 0, 4, 4), label='humanoid robot', score=0.9)],
    )
    # Rotate the cloud so every transformed point lands behind the vehicle,
    # which forces the fallback path to use the raw organized cloud instead.
    bad_rotation = np.diag([-1.0, -1.0, 1.0]).astype(np.float32)

    add_pointcloud_measurements(
        batch,
        pointcloud_xyz,
        bad_rotation,
        np.zeros(3, dtype=np.float32),
    )

    detection = batch.detections[0]
    assert detection.pointcloud_distance_m is not None
    assert math.isclose(detection.pointcloud_forward_m, 1.75, rel_tol=1e-5)
