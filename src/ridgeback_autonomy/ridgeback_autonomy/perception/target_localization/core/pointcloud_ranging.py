"""The ``pointcloud`` estimator: range straight off the organized ``PointCloud2``.

The one path that reads the cloud the sensor publishes rather than deprojecting
a depth image itself, which is why it is also the reference the mask stack's
deprojection was validated against
(``docs/history/pointcloud_provenance_test.md``).
"""

from __future__ import annotations

import math

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from ridgeback_autonomy.common.models import DetectionBatch
from ridgeback_autonomy.perception.target_localization.core.ranging_defaults import (
    FRONT_PERCENTILE as POINTCLOUD_FRONT_PERCENTILE,
    INLIER_AHEAD_MARGIN_M as POINTCLOUD_INLIER_AHEAD_MARGIN_M,
    INLIER_BEHIND_MARGIN_M as POINTCLOUD_INLIER_BEHIND_MARGIN_M,
    MAX_RANGE_M as POINTCLOUD_MAX_METERS,
    MIN_VALID_SAMPLES as POINTCLOUD_MIN_VALID_POINTS,
)
from ridgeback_autonomy.perception.target_localization.core.vehicle_frame import apply_vehicle_front_offset


# The ROI crop inside a detection box: a centre patch, biased upward, so the
# torso rather than the floor under the feet drives the range.
DEPTH_FOCUS_X_MIN = 0.24
DEPTH_FOCUS_X_MAX = 0.76
DEPTH_FOCUS_Y_MIN = 0.18
DEPTH_FOCUS_Y_MAX = 0.62

POINTCLOUD_MIN_INLIERS = 3


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
