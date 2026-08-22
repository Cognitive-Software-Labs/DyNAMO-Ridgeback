from __future__ import annotations

import math

import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from ridgeback_autonomy.common.models import CameraConfig, DetectionBatch, LidarScanPoints


ROBOT_FRONT_OFFSET_M = 0.25

DEPTH_FOCUS_X_MIN = 0.24
DEPTH_FOCUS_X_MAX = 0.76
DEPTH_FOCUS_Y_MIN = 0.18
DEPTH_FOCUS_Y_MAX = 0.62

LIDAR_MIN_RANGE_METERS = 0.05
LIDAR_MAX_METERS = 10.0
LIDAR_HALF_WINDOW_MARGIN_DEG = 0.75
LIDAR_HALF_WINDOW_SCALE = 0.25
LIDAR_HALF_WINDOW_FLOOR_DEG = 1.0
LIDAR_CLOSE_PERCENTILE = 30.0
LIDAR_INLIER_DISTANCE_MARGIN_M = 0.20
LIDAR_MIN_VALID_RAYS = 2

POINTCLOUD_MAX_METERS = 10.0
POINTCLOUD_FRONT_PERCENTILE = 25.0
POINTCLOUD_INLIER_AHEAD_MARGIN_M = 0.10
POINTCLOUD_INLIER_BEHIND_MARGIN_M = 0.35
POINTCLOUD_MIN_VALID_POINTS = 10
POINTCLOUD_MIN_INLIERS = 3


def apply_vehicle_front_offset(lateral_m, forward_m):
    return lateral_m, forward_m - ROBOT_FRONT_OFFSET_M


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_vehicle_planar(dx_world: float, dy_world: float, yaw_rad: float):
    forward_m = math.cos(yaw_rad) * dx_world + math.sin(yaw_rad) * dy_world
    lateral_m = -math.sin(yaw_rad) * dx_world + math.cos(yaw_rad) * dy_world
    return forward_m, lateral_m


def planar_measurement_from_vehicle_front(
    dx_world: float,
    dy_world: float,
    yaw_rad: float,
) -> tuple[float, float, float]:
    """World-frame offset -> ``(forward_m, lateral_m, distance_m)`` off the robot front.

    All three components share one reference point -- the base origin plus
    ``ROBOT_FRONT_OFFSET_M`` -- which is what every estimator publishes against,
    so ground truth and estimates are directly comparable component by component
    (the benchmark association matches them in the planar plane, not just on
    distance). Lateral is offset-invariant: the front offset is purely forward.
    """

    forward_m, lateral_m = world_to_vehicle_planar(dx_world, dy_world, yaw_rad)
    lateral_m, forward_m = apply_vehicle_front_offset(lateral_m, forward_m)
    return forward_m, lateral_m, math.hypot(lateral_m, forward_m)


def rotate_camera_to_vehicle_frame(x_cam, y_cam, z_cam, pitch_rad: float):
    cos_pitch = math.cos(pitch_rad)
    sin_pitch = math.sin(pitch_rad)

    # Camera-optical +X points image-right; vehicle lateral is base +Y, which is
    # left-positive (REP-103). Negating keeps this family on the same convention
    # as the lidar, pointcloud, and mask paths -- and as the ground truth the
    # benchmark associates against.
    lateral_vehicle = -x_cam
    vertical_vehicle = cos_pitch * y_cam + sin_pitch * z_cam
    forward_vehicle = -sin_pitch * y_cam + cos_pitch * z_cam
    return lateral_vehicle, vertical_vehicle, forward_vehicle


def focus_bbox(bbox_xyxy: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox_xyxy
    width = max(x2 - x1, 1)
    height = max(y2 - y1, 1)

    focus_x1 = x1 + int(math.floor(width * DEPTH_FOCUS_X_MIN))
    focus_x2 = x1 + int(math.ceil(width * DEPTH_FOCUS_X_MAX))
    focus_y1 = y1 + int(math.floor(height * DEPTH_FOCUS_Y_MIN))
    focus_y2 = y1 + int(math.ceil(height * DEPTH_FOCUS_Y_MAX))

    focus_x1 = max(x1, min(x2 - 1, focus_x1))
    focus_y1 = max(y1, min(y2 - 1, focus_y1))
    focus_x2 = max(focus_x1 + 1, min(x2, focus_x2))
    focus_y2 = max(focus_y1 + 1, min(y2, focus_y2))
    return focus_x1, focus_y1, focus_x2, focus_y2


def map_bbox_between_images(
    bbox_xyxy: tuple[int, int, int, int],
    src_width: int,
    src_height: int,
    dst_width: int,
    dst_height: int,
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox_xyxy
    scale_x = dst_width / max(src_width, 1)
    scale_y = dst_height / max(src_height, 1)

    mapped_x1 = int(math.floor(x1 * scale_x))
    mapped_y1 = int(math.floor(y1 * scale_y))
    mapped_x2 = int(math.ceil(x2 * scale_x))
    mapped_y2 = int(math.ceil(y2 * scale_y))

    mapped_x1 = max(0, min(dst_width - 1, mapped_x1))
    mapped_y1 = max(0, min(dst_height - 1, mapped_y1))
    mapped_x2 = max(mapped_x1 + 1, min(dst_width, mapped_x2))
    mapped_y2 = max(mapped_y1 + 1, min(dst_height, mapped_y2))
    return mapped_x1, mapped_y1, mapped_x2, mapped_y2


def add_rgb_measurements(batch: DetectionBatch, camera_config: CameraConfig) -> DetectionBatch:
    if not batch.detected:
        return batch

    hfov = camera_config.depth_hfov_deg
    vfov = camera_config.depth_vfov_deg
    height_m = camera_config.height_m
    pitch = math.radians(camera_config.pitch_deg)

    fx = batch.image_width / (2.0 * math.tan(math.radians(hfov / 2.0)))
    fy = batch.image_height / (2.0 * math.tan(math.radians(vfov / 2.0)))
    cx0 = batch.image_width / 2.0
    cy0 = batch.image_height / 2.0

    for detection in batch.detections:
        x1, _, x2, y2 = detection.bbox_xyxy
        u = (x1 + x2) / 2.0
        v = float(y2)
        x_cam = (u - cx0) / fx
        y_cam = (v - cy0) / fy
        z_cam = 1.0
        lateral_world, vertical_world, forward_world = rotate_camera_to_vehicle_frame(
            x_cam,
            y_cam,
            z_cam,
            pitch,
        )

        if vertical_world <= 0:
            continue

        t = height_m / vertical_world
        lateral_m = lateral_world * t
        forward_m = forward_world * t
        lateral_m, forward_m = apply_vehicle_front_offset(lateral_m, forward_m)

        detection.rgb_lateral_m = float(lateral_m)
        detection.rgb_forward_m = float(forward_m)
        detection.rgb_distance_m = float(math.hypot(lateral_m, forward_m))

    return batch


def add_depth_measurements(
    batch: DetectionBatch,
    camera_config: CameraConfig,
    depth_max_meters: float,
    sensor_depth_meters: np.ndarray | None,
    mono_depth_meters: np.ndarray | None,
) -> DetectionBatch:
    for detection in batch.detections:
        detection.focus_bbox_xyxy = focus_bbox(detection.bbox_xyxy)
        detection.sensor_depth_distance_m = None
        detection.mono_depth_distance_m = None

    if not batch.detected:
        return batch

    add_depth_source_measurements(
        batch,
        camera_config,
        depth_max_meters,
        sensor_depth_meters,
        'sensor_depth_distance_m',
    )
    add_depth_source_measurements(
        batch,
        camera_config,
        depth_max_meters,
        mono_depth_meters,
        'mono_depth_distance_m',
    )
    return batch


def add_depth_source_measurements(
    batch: DetectionBatch,
    camera_config: CameraConfig,
    depth_max_meters: float,
    depth_meters: np.ndarray | None,
    target_key: str,
) -> None:
    if depth_meters is None:
        return

    depth_h, depth_w = depth_meters.shape[:2]
    for detection in batch.detections:
        focus = detection.focus_bbox_xyxy or focus_bbox(detection.bbox_xyxy)
        candidate_bboxes = [
            map_bbox_between_images(
                focus,
                batch.image_width,
                batch.image_height,
                depth_w,
                depth_h,
            ),
            map_bbox_between_images(
                detection.bbox_xyxy,
                batch.image_width,
                batch.image_height,
                depth_w,
                depth_h,
            ),
        ]

        distance = None
        for candidate_bbox in candidate_bboxes:
            distance = compute_weighted_planar_distance(
                depth_meters,
                candidate_bbox,
                camera_config,
                depth_max_meters,
            )
            if distance is not None:
                break
        setattr(detection, target_key, distance)


def compute_weighted_planar_distance(
    depth_meters: np.ndarray,
    bbox_xyxy: tuple[int, int, int, int],
    camera_config: CameraConfig,
    depth_max_meters: float,
) -> float | None:
    x1, y1, x2, y2 = bbox_xyxy
    roi = depth_meters[y1:y2, x1:x2]
    if roi.size == 0:
        return None

    valid = np.isfinite(roi) & (roi > 0.0)
    if depth_max_meters > 0.0:
        valid &= roi <= depth_max_meters
    if not np.any(valid):
        return None

    roi_h, roi_w = roi.shape
    xs = np.arange(roi_w, dtype=np.float32)
    ys = np.arange(roi_h, dtype=np.float32)
    center_x = (roi_w - 1) / 2.0
    center_y = (roi_h - 1) / 2.0
    sigma_x = max(roi_w * 0.25, 1e-6)
    sigma_y = max(roi_h * 0.25, 1e-6)

    x_weights = np.exp(-0.5 * np.square((xs - center_x) / sigma_x))
    y_weights = np.exp(-0.5 * np.square((ys - center_y) / sigma_y))
    weights = np.outer(y_weights, x_weights).astype(np.float32)

    img_h, img_w = depth_meters.shape[:2]
    fx = img_w / (2.0 * math.tan(math.radians(camera_config.depth_hfov_deg / 2.0)))
    fy = img_h / (2.0 * math.tan(math.radians(camera_config.depth_vfov_deg / 2.0)))
    cx0 = img_w / 2.0
    cy0 = img_h / 2.0
    pitch = math.radians(camera_config.pitch_deg)

    us = np.arange(x1, x2, dtype=np.float32)
    vs = np.arange(y1, y2, dtype=np.float32)
    grid_u, grid_v = np.meshgrid(us, vs)

    z_cam = roi.astype(np.float32)
    x_cam = ((grid_u - cx0) / fx) * z_cam
    y_cam = ((grid_v - cy0) / fy) * z_cam
    lateral_world, _, forward_world = rotate_camera_to_vehicle_frame(
        x_cam,
        y_cam,
        z_cam,
        pitch,
    )
    lateral_world, forward_world = apply_vehicle_front_offset(lateral_world, forward_world)

    valid &= forward_world > 0.0
    if not np.any(valid):
        return None

    planar_distance = np.sqrt(np.square(lateral_world) + np.square(forward_world))
    weights *= valid.astype(np.float32)
    weight_sum = float(np.sum(weights))
    if weight_sum <= 0.0:
        return None

    return float(np.sum(planar_distance * weights) / weight_sum)


def extract_scan_points_base(
    scan_msg: LaserScan,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> LidarScanPoints:
    ranges = np.asarray(scan_msg.ranges, dtype=np.float32)
    if ranges.size == 0:
        raise ValueError('LaserScan contains no ranges.')

    valid = np.isfinite(ranges)
    safe_ranges = ranges.copy()
    safe_ranges[~valid] = 0.0

    range_min = max(float(scan_msg.range_min), LIDAR_MIN_RANGE_METERS)
    if range_min > 0.0:
        valid &= safe_ranges >= range_min

    range_max = float(scan_msg.range_max)
    if math.isfinite(range_max) and range_max > 0.0:
        valid &= safe_ranges <= min(range_max, LIDAR_MAX_METERS)
    else:
        valid &= safe_ranges <= LIDAR_MAX_METERS

    if not np.any(valid):
        raise ValueError('LaserScan contains no valid ranges.')

    angles = scan_msg.angle_min + (
        np.arange(ranges.size, dtype=np.float32) * np.float32(scan_msg.angle_increment)
    )
    points_scan = np.stack((
        safe_ranges * np.cos(angles),
        safe_ranges * np.sin(angles),
        np.zeros_like(safe_ranges),
    ), axis=1)
    points_base = points_scan @ rotation.T + translation

    forward_m = points_base[:, 0]
    lateral_m = points_base[:, 1]
    planar_distance_m = np.hypot(forward_m, lateral_m)
    bearing_rad = np.arctan2(lateral_m, forward_m)

    valid &= np.isfinite(forward_m) & np.isfinite(lateral_m) & np.isfinite(planar_distance_m)
    valid &= forward_m > 0.0
    valid &= planar_distance_m <= LIDAR_MAX_METERS
    if not np.any(valid):
        raise ValueError('LaserScan points do not contain valid forward returns.')

    return LidarScanPoints(
        forward_m=forward_m,
        lateral_m=lateral_m,
        planar_distance_m=planar_distance_m,
        bearing_rad=bearing_rad,
        valid=valid,
        points_xyz=points_base,
    )


def add_lidar_measurements(
    batch: DetectionBatch,
    scan_points: LidarScanPoints | None,
    camera_hfov_rad: float,
    rotation: np.ndarray | None = None,
    translation: np.ndarray | None = None,
) -> DetectionBatch:
    for detection in batch.detections:
        detection.lidar_lateral_m = None
        detection.lidar_forward_m = None
        detection.lidar_distance_m = None

    if not batch.detected or scan_points is None:
        return batch

    for detection in batch.detections:
        measurement = compute_lidar_measurement(
            scan_points,
            detection.bbox_xyxy,
            batch.image_width,
            camera_hfov_rad,
            rotation,
            translation,
        )
        if measurement is None:
            continue

        lateral_m, forward_m, distance_m = measurement
        detection.lidar_lateral_m = lateral_m
        detection.lidar_forward_m = forward_m
        detection.lidar_distance_m = distance_m

    return batch


def compute_lidar_measurement(
    scan_points: LidarScanPoints,
    bbox_xyxy: tuple[int, int, int, int],
    image_width: int,
    camera_hfov_rad: float,
    rotation: np.ndarray | None = None,
    translation: np.ndarray | None = None,
) -> tuple[float, float, float] | None:
    center_bearing_rad, half_window_rad = compute_camera_bearing_window(
        bbox_xyxy,
        image_width,
        camera_hfov_rad,
    )

    if rotation is None or translation is None or scan_points.points_xyz is None:
        return None

    # 1. Transform LiDAR base frame points into camera optical frame
    # points_xyz is shape (N, 3), translation is (3,), rotation is (3, 3)
    # points_cam = (points_base - translation) @ rotation
    points_cam = (scan_points.points_xyz - translation) @ rotation

    # 2. Compute horizontal bearing of points in the camera frame
    # In camera optical frame, X is lateral (right) and Z is forward (depth)
    bearings_cam = np.arctan2(points_cam[:, 0], points_cam[:, 2])

    # 3. Apply bounding box gate to camera-relative bearing angles
    bearing_delta = wrap_angle(bearings_cam - center_bearing_rad)
    gate = scan_points.valid & (points_cam[:, 2] > 0.0) & (np.abs(bearing_delta) <= half_window_rad)
    if int(np.count_nonzero(gate)) < LIDAR_MIN_VALID_RAYS:
        return None

    gated_points_base = scan_points.points_xyz[gate]
    gated_points_cam = points_cam[gate]

    # 4. Compute camera-relative planar distance for percentile filtering
    gated_planar = np.hypot(gated_points_cam[:, 0], gated_points_cam[:, 2])

    # 5. Extract close points on the target object
    anchor_distance_m = float(np.percentile(gated_planar, LIDAR_CLOSE_PERCENTILE))
    inliers = gated_planar <= anchor_distance_m + LIDAR_INLIER_DISTANCE_MARGIN_M
    if not np.any(inliers):
        return None

    # 6. Compute target median position in the camera frame
    target_cam_x = float(np.median(gated_points_cam[inliers, 0]))
    target_cam_y = float(np.median(gated_points_cam[inliers, 1]))
    target_cam_z = float(np.median(gated_points_cam[inliers, 2]))
    target_cam = np.array([target_cam_x, target_cam_y, target_cam_z], dtype=np.float32)

    # 7. Project target median back to base link frame
    target_base = rotation @ target_cam + translation
    lateral_m = float(target_base[1])
    forward_m = float(target_base[0])

    # Apply vehicle front offset and compute final distance in the base frame
    lateral_m, forward_m = apply_vehicle_front_offset(lateral_m, forward_m)
    distance_m = float(math.hypot(lateral_m, forward_m))

    if forward_m <= 0.0 or distance_m > LIDAR_MAX_METERS:
        return None
    return lateral_m, forward_m, distance_m


def compute_camera_bearing_window(
    bbox_xyxy: tuple[int, int, int, int],
    image_width: int,
    camera_hfov_rad: float,
) -> tuple[float, float]:
    x1, _, x2, _ = bbox_xyxy
    fx = image_width / (2.0 * math.tan(camera_hfov_rad / 2.0))
    cx0 = image_width / 2.0

    left_bearing_rad = math.atan2(x1 - cx0, fx)
    right_bearing_rad = math.atan2((x2 - 1) - cx0, fx)
    center_bearing_rad = 0.5 * (left_bearing_rad + right_bearing_rad)
    half_window_rad = 0.5 * abs(right_bearing_rad - left_bearing_rad)

    margin_rad = max(
        math.radians(LIDAR_HALF_WINDOW_MARGIN_DEG),
        half_window_rad * LIDAR_HALF_WINDOW_SCALE,
    )
    half_window_rad = max(
        half_window_rad + margin_rad,
        math.radians(LIDAR_HALF_WINDOW_FLOOR_DEG),
    )
    return center_bearing_rad, half_window_rad


def wrap_angle(angle_rad):
    return np.arctan2(np.sin(angle_rad), np.cos(angle_rad))


def extract_organized_xyz(msg: PointCloud2, image_shape: tuple[int, int]) -> np.ndarray:
    image_height, image_width = image_shape
    if msg.height <= 1 or msg.width <= 1:
        raise ValueError('Point cloud is not organized.')
    if msg.height != image_height or msg.width != image_width:
        raise ValueError(
            'Point cloud shape does not match RGB image '
            f'({msg.width}x{msg.height} vs {image_width}x{image_height}).'
        )

    xyz_points = point_cloud2.read_points_numpy(
        msg,
        field_names=['x', 'y', 'z'],
        skip_nans=False,
    )
    if xyz_points.ndim != 2 or xyz_points.shape[0] != msg.width * msg.height or xyz_points.shape[1] != 3:
        raise ValueError('Unexpected PointCloud2 layout for x/y/z fields.')

    return xyz_points.reshape((msg.height, msg.width, 3)).astype(np.float32, copy=False)


def add_pointcloud_measurements(
    batch: DetectionBatch,
    pointcloud_xyz: np.ndarray | None,
    rotation: np.ndarray | None,
    translation: np.ndarray | None,
) -> DetectionBatch:
    for detection in batch.detections:
        detection.focus_bbox_xyxy = focus_bbox(detection.bbox_xyxy)
        detection.pointcloud_lateral_m = None
        detection.pointcloud_forward_m = None
        detection.pointcloud_distance_m = None

    if not batch.detected or pointcloud_xyz is None:
        return batch

    for detection in batch.detections:
        measurement = compute_pointcloud_measurement(
            pointcloud_xyz,
            detection.focus_bbox_xyxy or focus_bbox(detection.bbox_xyxy),
            rotation,
            translation,
        )
        if measurement is None and (rotation is not None or translation is not None):
            # Some simulator-organized point clouds are already expressed in a
            # base-like camera frame even when the frame id suggests an optical
            # transform. Fall back to the raw cloud instead of dropping the
            # measurement entirely.
            measurement = compute_pointcloud_measurement(
                pointcloud_xyz,
                detection.focus_bbox_xyxy or focus_bbox(detection.bbox_xyxy),
                None,
                None,
            )
        if measurement is None:
            continue

        lateral_m, forward_m, distance_m = measurement
        detection.pointcloud_lateral_m = lateral_m
        detection.pointcloud_forward_m = forward_m
        detection.pointcloud_distance_m = distance_m

    return batch


def compute_pointcloud_measurement(
    pointcloud_xyz: np.ndarray,
    bbox_xyxy: tuple[int, int, int, int],
    rotation: np.ndarray | None,
    translation: np.ndarray | None,
) -> tuple[float, float, float] | None:
    x1, y1, x2, y2 = bbox_xyxy
    roi_points = pointcloud_xyz[y1:y2, x1:x2, :].reshape(-1, 3)
    valid = np.all(np.isfinite(roi_points), axis=1)
    if not np.any(valid):
        return None

    valid_points = roi_points[valid]
    if valid_points.shape[0] < POINTCLOUD_MIN_VALID_POINTS:
        return None

    if rotation is not None and translation is not None:
        points_vehicle = valid_points @ rotation.T + translation
    else:
        points_vehicle = valid_points

    lateral_m = points_vehicle[:, 1]
    forward_m = points_vehicle[:, 0]
    lateral_m, forward_m = apply_vehicle_front_offset(lateral_m, forward_m)

    planar_distance = np.hypot(lateral_m, forward_m)
    valid = np.isfinite(planar_distance) & (forward_m > 0.0)
    valid &= planar_distance <= POINTCLOUD_MAX_METERS
    if not np.any(valid):
        return None

    lateral_m = lateral_m[valid]
    forward_m = forward_m[valid]
    planar_distance = planar_distance[valid]

    anchor_forward_m = float(np.percentile(forward_m, POINTCLOUD_FRONT_PERCENTILE))
    inliers = (
        (forward_m >= anchor_forward_m - POINTCLOUD_INLIER_AHEAD_MARGIN_M)
        & (forward_m <= anchor_forward_m + POINTCLOUD_INLIER_BEHIND_MARGIN_M)
    )
    if int(np.count_nonzero(inliers)) < POINTCLOUD_MIN_INLIERS:
        return None

    lateral_inliers = lateral_m[inliers]
    forward_inliers = forward_m[inliers]
    distance_inliers = planar_distance[inliers]
    nearest_index = int(np.argmin(distance_inliers))
    return (
        float(lateral_inliers[nearest_index]),
        float(forward_inliers[nearest_index]),
        float(distance_inliers[nearest_index]),
    )
