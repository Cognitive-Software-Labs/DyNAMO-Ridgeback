"""Bounded sensor samples taken inside an unshortened live measurement window.

The quota recorder ends a trial's window once ``capture_batches`` raw batches
are frozen, so the live result it scores is the sample. Sampled-window capture
keeps the window at ``capture_sec`` and freezes only the first
``capture_batches`` raw batches delivered inside it. The trial therefore has
two live populations that must not be confused:

- the full window, scored exactly as an uncaptured run scores it, and
- the sampled batches' own live measurements, scored separately with the same
  functions, which is the only live result a replay of the sample can match.

ROS-free, so the campaign evidence checks and tests can read what the runner
wrote without a node.
"""

from __future__ import annotations

from typing import Any, Iterable

from ridgeback_common.miss_reason import MissReason
from ridgeback_localization.estimator_registry import (
    DEPTH_PATH_ESTIMATORS,
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_POSITION_ATTRS,
    ESTIMATOR_STATUS_FIELD_KEYS,
)


SENSOR_CAPTURE_MODE_QUOTA = 'quota'
SENSOR_CAPTURE_MODE_SAMPLED_WINDOW = 'sampled_window'
SENSOR_CAPTURE_MODES = (SENSOR_CAPTURE_MODE_QUOTA, SENSOR_CAPTURE_MODE_SAMPLED_WINDOW)

# Written beside the full-window CSVs of the run that took the sample.
SAMPLED_REFERENCE_DIRNAME = 'sampled_reference'
SAMPLED_WINDOWS_NAME = 'sampled_windows.json'
SAMPLED_WINDOWS_SCHEMA = 'ridgeback-sampled-windows/1'

# Statuses with which the live node says it produced no aligned depth for the
# frame: the frame failed before the depth source ran, or the source returned
# nothing. Any other depth-path status means a frame was produced and published.
NO_DEPTH_STATUSES = frozenset(int(reason) for reason in (
    MissReason.NO_CAMERA_INFO,
    MissReason.GRID_MISMATCH,
    MissReason.NO_COLOR_FRAME,
    MissReason.TF_MISS_EXTRINSIC,
    MissReason.NO_DEPTH_FRAME,
    MissReason.UNSET,
))


def parse_sensor_capture_mode(value: Any) -> str:
    mode = str(value).strip()
    if mode not in SENSOR_CAPTURE_MODES:
        raise ValueError(
            f'Unknown sensor_capture_mode "{mode}"; expected one of '
            f'{", ".join(SENSOR_CAPTURE_MODES)}.')
    return mode


def _optional_float(value: Any) -> float | None:
    return None if value is None else float(value)


def live_measurement_record(
    event,
    estimators: Iterable[str],
) -> dict[str, Any]:
    """Serialize one live measurement event exactly as the node published it.

    Detection order is the node's own, which is the raw detector order, so the
    record can be checked against the frozen raw batch box by box.
    """

    selected = tuple(estimators)
    detections = []
    for detection in event.detections:
        values = {}
        for estimator in selected:
            position = ESTIMATOR_POSITION_ATTRS.get(estimator)
            status_attr = ESTIMATOR_STATUS_FIELD_KEYS.get(estimator)
            values[estimator] = {
                'distance_m': _optional_float(
                    getattr(detection, ESTIMATOR_FIELD_KEYS[estimator])),
                'forward_m': (
                    None if position is None
                    else _optional_float(getattr(detection, position[0]))),
                'lateral_m': (
                    None if position is None
                    else _optional_float(getattr(detection, position[1]))),
                'status': (
                    None if status_attr is None
                    else getattr(detection, status_attr)),
            }
        detections.append({
            'bbox_xyxy': [int(value) for value in detection.bbox_xyxy],
            'label': str(detection.label),
            'score': float(detection.score),
            'estimators': values,
        })
    return {
        'stamp_ns': int(event.stamp_ns),
        'frame_id': str(event.key[0]),
        'detected': bool(event.detected),
        'count': int(event.count),
        'image_width': int(event.image_width),
        'image_height': int(event.image_height),
        'detections': detections,
    }


def live_record_expects_depth(record: dict | None) -> bool:
    """Whether the live node published aligned depth for this measurement.

    Unknown until the live record arrives, so a missing record keeps the
    recorder waiting. Polar profiling never decides it: it needs no depth.
    """

    if record is None:
        return True
    for detection in record['detections']:
        for estimator, values in detection['estimators'].items():
            if estimator not in DEPTH_PATH_ESTIMATORS:
                continue
            status = values.get('status')
            if status is not None and int(status) not in NO_DEPTH_STATUSES:
                return True
    return False


def live_record_expects_scan(record: dict | None) -> bool:
    """Whether a polar outcome proves the worker selected a scan."""

    if record is None:
        return False
    present = {int(reason) for reason in (
        MissReason.OK, MissReason.TF_MISS_SCAN, MissReason.SCAN_INVALID,
        MissReason.NO_BEAMS_IN_VIEW, MissReason.TOO_FEW_RAYS_SELECTED,
        MissReason.TOO_FEW_RAYS_MERGED,
    )}
    return any(
        detection['estimators'].get('polar_profiling', {}).get('status') in present
        for detection in record['detections'])


def live_record_matches_raw_batch(record: dict, event: dict) -> bool:
    """Whether a live record measured exactly the frozen raw batch, box by box."""

    if int(record['stamp_ns']) != int(event['stamp_ns']):
        return False
    if len(record['detections']) != len(event['detections']):
        return False
    return all(
        [int(value) for value in live['bbox_xyxy']]
        == [int(value) for value in raw['bbox_xyxy']]
        for live, raw in zip(record['detections'], event['detections']))
