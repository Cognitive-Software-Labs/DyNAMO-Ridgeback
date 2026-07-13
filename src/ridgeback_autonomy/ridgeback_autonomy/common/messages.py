from __future__ import annotations

import math
from typing import Any

import numpy as np
from sensor_msgs.msg import Image

from ridgeback_autonomy.common.models import Detection, DetectionBatch
from ridgeback_autonomy.msg import G1Detections, G1Measurements


MISSING_FLOAT = float('nan')


def optional_float(value: float | None) -> float:
    return float(value) if value is not None else MISSING_FLOAT


def build_detections_message(batch: DetectionBatch, header) -> G1Detections:
    msg = G1Detections()
    _populate_identity_fields(msg, batch, header)
    return msg


def build_measurements_message(batch: DetectionBatch, header) -> G1Measurements:
    msg = G1Measurements()
    _populate_identity_fields(msg, batch, header)

    for detection in batch.detections:
        msg.rgb_lateral_m.append(optional_float(detection.rgb_lateral_m))
        msg.rgb_forward_m.append(optional_float(detection.rgb_forward_m))
        msg.rgb_distance_m.append(optional_float(detection.rgb_distance_m))
        msg.sensor_depth_distance_m.append(optional_float(detection.sensor_depth_distance_m))
        msg.mono_depth_distance_m.append(optional_float(detection.mono_depth_distance_m))
        msg.pointcloud_lateral_m.append(optional_float(detection.pointcloud_lateral_m))
        msg.pointcloud_forward_m.append(optional_float(detection.pointcloud_forward_m))
        msg.pointcloud_distance_m.append(optional_float(detection.pointcloud_distance_m))
        msg.lidar_lateral_m.append(optional_float(detection.lidar_lateral_m))
        msg.lidar_forward_m.append(optional_float(detection.lidar_forward_m))
        msg.lidar_distance_m.append(optional_float(detection.lidar_distance_m))
        msg.projective_ranging_lateral_m.append(optional_float(detection.projective_ranging_lateral_m))
        msg.projective_ranging_forward_m.append(optional_float(detection.projective_ranging_forward_m))
        msg.projective_ranging_distance_m.append(optional_float(detection.projective_ranging_distance_m))
        msg.euclidean_reconstruction_lateral_m.append(optional_float(detection.euclidean_reconstruction_lateral_m))
        msg.euclidean_reconstruction_forward_m.append(optional_float(detection.euclidean_reconstruction_forward_m))
        msg.euclidean_reconstruction_distance_m.append(optional_float(detection.euclidean_reconstruction_distance_m))
        msg.polar_profiling_lateral_m.append(optional_float(detection.polar_profiling_lateral_m))
        msg.polar_profiling_forward_m.append(optional_float(detection.polar_profiling_forward_m))
        msg.polar_profiling_distance_m.append(optional_float(detection.polar_profiling_distance_m))

    return msg


def batch_from_detections_message(msg: G1Detections) -> DetectionBatch:
    batch = DetectionBatch(
        image_width=int(msg.image_width),
        image_height=int(msg.image_height),
    )
    bboxes = decode_bbox_quads(msg.bbox_xyxy)
    for index, bbox in enumerate(bboxes):
        batch.detections.append(
            Detection(
                bbox_xyxy=bbox,
                label=msg.labels[index] if index < len(msg.labels) else '',
                score=float(msg.scores[index]) if index < len(msg.scores) else 0.0,
            )
        )
    return batch


def batch_from_measurements_message(msg: G1Measurements) -> DetectionBatch:
    batch = batch_from_detections_message(msg)

    for index, detection in enumerate(batch.detections):
        detection.rgb_lateral_m = decode_optional_float(msg.rgb_lateral_m, index)
        detection.rgb_forward_m = decode_optional_float(msg.rgb_forward_m, index)
        detection.rgb_distance_m = decode_optional_float(msg.rgb_distance_m, index)
        detection.sensor_depth_distance_m = decode_optional_float(msg.sensor_depth_distance_m, index)
        detection.mono_depth_distance_m = decode_optional_float(msg.mono_depth_distance_m, index)
        detection.pointcloud_lateral_m = decode_optional_float(msg.pointcloud_lateral_m, index)
        detection.pointcloud_forward_m = decode_optional_float(msg.pointcloud_forward_m, index)
        detection.pointcloud_distance_m = decode_optional_float(msg.pointcloud_distance_m, index)
        detection.lidar_lateral_m = decode_optional_float(msg.lidar_lateral_m, index)
        detection.lidar_forward_m = decode_optional_float(msg.lidar_forward_m, index)
        detection.lidar_distance_m = decode_optional_float(msg.lidar_distance_m, index)
        detection.projective_ranging_lateral_m = decode_optional_float(msg.projective_ranging_lateral_m, index)
        detection.projective_ranging_forward_m = decode_optional_float(msg.projective_ranging_forward_m, index)
        detection.projective_ranging_distance_m = decode_optional_float(msg.projective_ranging_distance_m, index)
        detection.euclidean_reconstruction_lateral_m = decode_optional_float(msg.euclidean_reconstruction_lateral_m, index)
        detection.euclidean_reconstruction_forward_m = decode_optional_float(msg.euclidean_reconstruction_forward_m, index)
        detection.euclidean_reconstruction_distance_m = decode_optional_float(msg.euclidean_reconstruction_distance_m, index)
        detection.polar_profiling_lateral_m = decode_optional_float(msg.polar_profiling_lateral_m, index)
        detection.polar_profiling_forward_m = decode_optional_float(msg.polar_profiling_forward_m, index)
        detection.polar_profiling_distance_m = decode_optional_float(msg.polar_profiling_distance_m, index)

    return batch


def first_finite_positive(values) -> float | None:
    for value in values:
        value = float(value)
        if math.isfinite(value) and value > 0.0:
            return value
    return None


def decode_bbox_quads(values) -> list[tuple[int, int, int, int]]:
    bboxes: list[tuple[int, int, int, int]] = []
    bbox_values = list(values)
    for index in range(0, len(bbox_values) - 3, 4):
        bboxes.append((
            int(round(float(bbox_values[index]))),
            int(round(float(bbox_values[index + 1]))),
            int(round(float(bbox_values[index + 2]))),
            int(round(float(bbox_values[index + 3]))),
        ))
    return bboxes


def snapshot_measurements_message(msg: G1Measurements) -> dict[str, Any]:
    return {
        'detected': bool(msg.detected),
        'count': int(msg.count),
        'bboxes': [list(bbox) for bbox in decode_bbox_quads(msg.bbox_xyxy)],
        'rgb_distance_m': first_finite_positive(msg.rgb_distance_m),
        'sensor_depth_distance_m': first_finite_positive(msg.sensor_depth_distance_m),
        'mono_depth_distance_m': first_finite_positive(msg.mono_depth_distance_m),
        'lidar_distance_m': first_finite_positive(msg.lidar_distance_m),
        'pointcloud_distance_m': first_finite_positive(msg.pointcloud_distance_m),
        'projective_ranging_distance_m': first_finite_positive(msg.projective_ranging_distance_m),
        'euclidean_reconstruction_distance_m': first_finite_positive(msg.euclidean_reconstruction_distance_m),
        'polar_profiling_distance_m': first_finite_positive(msg.polar_profiling_distance_m),
    }


def build_float32_image_message(image: np.ndarray, header) -> Image:
    image = np.asarray(image, dtype=np.float32)
    msg = Image()
    msg.header = header
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = '32FC1'
    msg.is_bigendian = False
    msg.step = int(image.shape[1] * image.dtype.itemsize)
    msg.data = image.tobytes()
    return msg


def _populate_identity_fields(message, batch: DetectionBatch, header) -> None:
    message.header = header
    message.detected = batch.detected
    message.count = batch.count
    message.image_width = batch.image_width
    message.image_height = batch.image_height

    for detection in batch.detections:
        x1, y1, x2, y2 = detection.bbox_xyxy
        message.bbox_xyxy.extend([float(x1), float(y1), float(x2), float(y2)])
        message.labels.append(detection.label)
        message.scores.append(float(detection.score))


def decode_optional_float(values, index: int) -> float | None:
    if index >= len(values):
        return None
    value = float(values[index])
    return value if math.isfinite(value) else None
