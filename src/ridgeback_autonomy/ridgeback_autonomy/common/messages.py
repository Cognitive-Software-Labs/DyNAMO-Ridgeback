from __future__ import annotations

import math
from typing import Any

import numpy as np
from sensor_msgs.msg import Image

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.common.models import Detection, DetectionBatch
from ridgeback_autonomy.msg import PolarBeams, TargetDetections, TargetMeasurements


MISSING_FLOAT = float('nan')


def optional_float(value: float | None) -> float:
    return float(value) if value is not None else MISSING_FLOAT


def optional_status(value: int | None) -> int:
    """Encode a miss-reason code as ``uint8``; ``None`` -> ``UNSET`` sentinel."""

    return int(value) if value is not None else int(MissReason.UNSET)


def decode_optional_status(values, index: int) -> int | None:
    """Decode a ``uint8`` status back to a code, ``UNSET``/absent -> ``None``."""

    if index >= len(values):
        return None
    value = int(values[index])
    return None if value == int(MissReason.UNSET) else value


def build_detections_message(batch: DetectionBatch, header) -> TargetDetections:
    msg = TargetDetections()
    _populate_identity_fields(msg, batch, header)
    return msg


def build_measurements_message(batch: DetectionBatch, header) -> TargetMeasurements:
    msg = TargetMeasurements()
    _populate_identity_fields(msg, batch, header)

    for detection in batch.detections:
        msg.pointcloud_lateral_m.append(optional_float(detection.pointcloud_lateral_m))
        msg.pointcloud_forward_m.append(optional_float(detection.pointcloud_forward_m))
        msg.pointcloud_distance_m.append(optional_float(detection.pointcloud_distance_m))
        msg.projective_ranging_lateral_m.append(optional_float(detection.projective_ranging_lateral_m))
        msg.projective_ranging_forward_m.append(optional_float(detection.projective_ranging_forward_m))
        msg.projective_ranging_distance_m.append(optional_float(detection.projective_ranging_distance_m))
        msg.euclidean_reconstruction_lateral_m.append(optional_float(detection.euclidean_reconstruction_lateral_m))
        msg.euclidean_reconstruction_forward_m.append(optional_float(detection.euclidean_reconstruction_forward_m))
        msg.euclidean_reconstruction_distance_m.append(optional_float(detection.euclidean_reconstruction_distance_m))
        msg.polar_profiling_lateral_m.append(optional_float(detection.polar_profiling_lateral_m))
        msg.polar_profiling_forward_m.append(optional_float(detection.polar_profiling_forward_m))
        msg.polar_profiling_distance_m.append(optional_float(detection.polar_profiling_distance_m))
        msg.projective_ranging_status.append(optional_status(detection.projective_ranging_status))
        msg.euclidean_reconstruction_status.append(optional_status(detection.euclidean_reconstruction_status))
        msg.polar_profiling_status.append(optional_status(detection.polar_profiling_status))

    return msg


def build_polar_beams_message(records, scan_msg, header) -> PolarBeams:
    """The frame's polar beam indices, unioned across its detections.

    ``header`` is the detection/measurement stamp so a consumer can key this the
    way it keys the silhouette artifact; the scan identity travels separately
    because the indices are only valid against that one array.

    The union is what makes this different from the ray markers, which draw the
    nearest detection alone to keep the 3D view readable
    (``docs/target_localization/polar_profiling.md`` Section 4). A 2D panel has no such
    clutter problem, so every ranged detection is shown.
    """

    msg = PolarBeams()
    msg.header = header
    msg.scan_stamp = scan_msg.header.stamp
    msg.scan_frame_id = scan_msg.header.frame_id
    msg.beam_count = len(scan_msg.ranges)
    msg.selected = union_beam_indices(record.selected for record in records)
    msg.merged = union_beam_indices(record.merged for record in records)
    return msg


def union_beam_indices(index_arrays) -> np.ndarray:
    """Sorted, de-duplicated ``uint32`` union of per-detection beam indices."""

    arrays = [np.asarray(indices, dtype=np.uint32).ravel() for indices in index_arrays]
    if not arrays:
        return np.empty(0, dtype=np.uint32)
    return np.unique(np.concatenate(arrays)).astype(np.uint32)


def polar_beam_booleans(msg: PolarBeams, beam_count: int):
    """``(used, dropped)`` per-beam booleans over a scan of ``beam_count`` beams.

    ``None`` when the message was recorded against a differently sized scan: the
    indices would land on the wrong beams, and a misaligned highlight is worse
    than none. ``dropped`` is selected-but-discarded -- the set that tells a
    frame the estimator threw away apart from a frame that saw nothing.
    """

    if int(msg.beam_count) != int(beam_count):
        return None

    selected_indices = np.asarray(msg.selected, dtype=np.intp)
    merged_indices = np.asarray(msg.merged, dtype=np.intp)
    for indices in (selected_indices, merged_indices):
        if indices.size and int(indices.max()) >= beam_count:
            return None

    selected = np.zeros(beam_count, dtype=bool)
    used = np.zeros(beam_count, dtype=bool)
    selected[selected_indices] = True
    used[merged_indices] = True
    return used, selected & ~used


def batch_from_detections_message(msg: TargetDetections) -> DetectionBatch:
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


def batch_from_measurements_message(msg: TargetMeasurements) -> DetectionBatch:
    batch = batch_from_detections_message(msg)

    for index, detection in enumerate(batch.detections):
        detection.pointcloud_lateral_m = decode_optional_float(msg.pointcloud_lateral_m, index)
        detection.pointcloud_forward_m = decode_optional_float(msg.pointcloud_forward_m, index)
        detection.pointcloud_distance_m = decode_optional_float(msg.pointcloud_distance_m, index)
        detection.projective_ranging_lateral_m = decode_optional_float(msg.projective_ranging_lateral_m, index)
        detection.projective_ranging_forward_m = decode_optional_float(msg.projective_ranging_forward_m, index)
        detection.projective_ranging_distance_m = decode_optional_float(msg.projective_ranging_distance_m, index)
        detection.euclidean_reconstruction_lateral_m = decode_optional_float(msg.euclidean_reconstruction_lateral_m, index)
        detection.euclidean_reconstruction_forward_m = decode_optional_float(msg.euclidean_reconstruction_forward_m, index)
        detection.euclidean_reconstruction_distance_m = decode_optional_float(msg.euclidean_reconstruction_distance_m, index)
        detection.polar_profiling_lateral_m = decode_optional_float(msg.polar_profiling_lateral_m, index)
        detection.polar_profiling_forward_m = decode_optional_float(msg.polar_profiling_forward_m, index)
        detection.polar_profiling_distance_m = decode_optional_float(msg.polar_profiling_distance_m, index)
        detection.projective_ranging_status = decode_optional_status(msg.projective_ranging_status, index)
        detection.euclidean_reconstruction_status = decode_optional_status(
            msg.euclidean_reconstruction_status, index)
        detection.polar_profiling_status = decode_optional_status(msg.polar_profiling_status, index)

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


def snapshot_measurements_message(msg: TargetMeasurements) -> dict[str, Any]:
    return {
        'detected': bool(msg.detected),
        'count': int(msg.count),
        'bboxes': [list(bbox) for bbox in decode_bbox_quads(msg.bbox_xyxy)],
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


def build_bgr8_image_message(image: np.ndarray, header) -> Image:
    """Wrap an OpenCV BGR frame as an ``Image`` for RViz to display.

    Sibling of ``build_float32_image_message``. ``bgr8`` is the encoding the
    frame is already in, so this is a header plus a buffer copy with no colour
    conversion -- it sits on a per-frame render path.
    """

    image = np.ascontiguousarray(image, dtype=np.uint8)
    msg = Image()
    msg.header = header
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.step = int(image.shape[1] * 3)
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
