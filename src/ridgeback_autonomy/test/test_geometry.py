from __future__ import annotations

import math

import numpy as np

from ridgeback_autonomy.common.models import CameraConfig, Detection, DetectionBatch, LidarScanPoints
from ridgeback_autonomy.perception.core.geometry import (
    ROBOT_FRONT_OFFSET_M,
    add_depth_measurements,
    add_pointcloud_measurements,
    add_rgb_measurements,
    compute_lidar_measurement,
    compute_pointcloud_measurement,
    compute_weighted_planar_distance,
    focus_bbox,
    map_bbox_between_images,
    planar_measurement_from_vehicle_front,
    rotate_camera_to_vehicle_frame,
)


def test_ground_truth_components_share_the_front_reference() -> None:
    """Forward, lateral, and distance must agree; each is measured off the front.

    A mixed tuple (origin-relative forward next to a front-relative distance)
    reads as a target that is farther ahead than it is away, and inflates every
    benchmark association cost by the offset.
    """

    forward_m, lateral_m, distance_m = planar_measurement_from_vehicle_front(
        5.02, -1.11, 0.0)

    assert math.isclose(forward_m, 5.02 - ROBOT_FRONT_OFFSET_M, rel_tol=1e-9)
    assert math.isclose(lateral_m, -1.11, rel_tol=1e-9)
    assert math.isclose(distance_m, math.hypot(forward_m, lateral_m), rel_tol=1e-12)
    assert distance_m >= abs(forward_m)


def test_camera_lateral_is_left_positive() -> None:
    """The rgb path must agree in sign with lidar, pointcloud, mask, and truth.

    Camera-optical +X is image-right, so a target left of centre has a negative
    optical X and must come back as a positive (left) vehicle lateral.
    """

    lateral_left, _, _ = rotate_camera_to_vehicle_frame(-0.4, 0.0, 1.0, 0.0)
    lateral_right, _, _ = rotate_camera_to_vehicle_frame(0.4, 0.0, 1.0, 0.0)

    assert lateral_left > 0.0
    assert lateral_right < 0.0


def test_rgb_measurement_lateral_matches_ground_truth_sign() -> None:
    """End-to-end sign check: a target left of centre reports the same sign as truth."""

    camera_config = CameraConfig(
        depth_hfov_deg=90.0,
        depth_vfov_deg=90.0,
        pitch_deg=0.0,
        height_m=0.85,
    )
    # Bounding box left of the image centre (image is 100 wide, centre 50).
    batch = DetectionBatch(
        image_width=100,
        image_height=100,
        detections=[Detection(bbox_xyxy=(10, 40, 30, 80), label='humanoid robot', score=0.9)],
    )

    add_rgb_measurements(batch, camera_config)
    rgb_lateral_m = batch.detections[0].rgb_lateral_m

    assert rgb_lateral_m is not None
    # Truth for a target off to the left is +Y in the base frame (REP-103).
    _, truth_lateral_m, _ = planar_measurement_from_vehicle_front(3.0, 1.0, 0.0)
    assert math.copysign(1.0, rgb_lateral_m) == math.copysign(1.0, truth_lateral_m)


def test_focus_bbox_and_mapping() -> None:
    bbox = focus_bbox((10, 20, 110, 220))

    assert bbox == (34, 56, 86, 144)
    assert map_bbox_between_images(bbox, 200, 400, 100, 200) == (17, 28, 43, 72)


def test_compute_weighted_planar_distance_for_center_pixel() -> None:
    depth_meters = np.full((10, 10), 4.0, dtype=np.float32)
    camera_config = CameraConfig(
        depth_hfov_deg=90.0,
        depth_vfov_deg=90.0,
        pitch_deg=0.0,
        height_m=0.85,
    )

    distance = compute_weighted_planar_distance(
        depth_meters,
        (5, 5, 6, 6),
        camera_config,
        depth_max_meters=10.0,
    )

    assert distance is not None
    assert math.isclose(distance, 3.75, rel_tol=1e-6)


def test_add_depth_measurements_falls_back_to_full_bbox_when_focus_has_no_depth() -> None:
    depth_meters = np.full((10, 10), np.nan, dtype=np.float32)
    depth_meters[2:8, 2:8] = 4.0
    # Carve out the focused center patch so the fallback path must use the
    # full bbox instead of the narrower focus ROI.
    depth_meters[3:6, 3:7] = np.nan

    batch = DetectionBatch(
        image_width=10,
        image_height=10,
        detections=[Detection(bbox_xyxy=(2, 2, 8, 8), label='humanoid robot', score=0.9)],
    )
    camera_config = CameraConfig(
        depth_hfov_deg=90.0,
        depth_vfov_deg=90.0,
        pitch_deg=0.0,
        height_m=0.85,
    )

    add_depth_measurements(
        batch,
        camera_config,
        depth_max_meters=10.0,
        sensor_depth_meters=depth_meters,
        mono_depth_meters=None,
    )

    assert batch.detections[0].sensor_depth_distance_m is not None


def test_compute_lidar_measurement_returns_vehicle_frame_distance() -> None:
    forward = np.array([2.0, 2.1, 1.9], dtype=np.float32)
    lateral = np.array([-0.1, 0.0, 0.1], dtype=np.float32)
    planar = np.hypot(forward, lateral)
    bearing = np.arctan2(lateral, forward)
    scan_points = LidarScanPoints(
        forward_m=forward,
        lateral_m=lateral,
        planar_distance_m=planar,
        bearing_rad=bearing,
        valid=np.array([True, True, True]),
    )

    measurement = compute_lidar_measurement(
        scan_points,
        bbox_xyxy=(45, 0, 55, 20),
        image_width=100,
        camera_hfov_rad=math.radians(90.0),
    )

    assert measurement is not None
    lateral_m, forward_m, distance_m = measurement
    assert abs(lateral_m) < 0.11
    assert math.isclose(forward_m, 1.75, rel_tol=1e-5)
    assert math.isclose(distance_m, math.hypot(lateral_m, forward_m), rel_tol=1e-6)


def test_compute_lidar_measurement_supports_camera_frame_transformations() -> None:
    rotation = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ], dtype=np.float32)
    translation = np.array([0.3, 0.0, 0.85], dtype=np.float32)

    forward = np.array([2.3, 2.3, 2.3, 6.3], dtype=np.float32)
    lateral = np.array([-0.05, 0.0, 0.05, 0.0], dtype=np.float32)
    planar = np.hypot(forward, lateral)
    bearing = np.arctan2(lateral, forward)
    points_xyz = np.stack([forward, lateral, np.full_like(forward, 0.85)], axis=1)

    scan_points = LidarScanPoints(
        forward_m=forward,
        lateral_m=lateral,
        planar_distance_m=planar,
        bearing_rad=bearing,
        valid=np.array([True, True, True, True]),
        points_xyz=points_xyz,
    )

    measurement = compute_lidar_measurement(
        scan_points,
        bbox_xyxy=(45, 0, 55, 20),
        image_width=100,
        camera_hfov_rad=math.radians(90.0),
        rotation=rotation,
        translation=translation,
    )

    assert measurement is not None
    lateral_m, forward_m, distance_m = measurement
    assert abs(lateral_m) < 1e-5
    assert math.isclose(forward_m, 2.05, rel_tol=1e-5)
    assert math.isclose(distance_m, 2.05, rel_tol=1e-5)


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
