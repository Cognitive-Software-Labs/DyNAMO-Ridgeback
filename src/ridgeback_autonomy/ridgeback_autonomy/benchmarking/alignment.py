from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from ridgeback_autonomy.common.messages import decode_bbox_quads, first_finite_positive
from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.msg import G1Measurements

from ridgeback_autonomy.benchmarking.estimators import (
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_STATUS_FIELD_KEYS,
)


MeasurementEventKey = tuple[Any, ...]


@dataclass
class EventPreview:
    color_bgr: np.ndarray | None = None
    color_stamp_ns: int | None = None
    color_delta_ms: float | None = None
    color_nearest_stamp_ns: int | None = None
    color_nearest_delta_ms: float | None = None
    sensor_depth_bgr: np.ndarray | None = None
    sensor_depth_stamp_ns: int | None = None
    sensor_depth_delta_ms: float | None = None
    sensor_depth_nearest_stamp_ns: int | None = None
    sensor_depth_nearest_delta_ms: float | None = None
    depth_anything_bgr: np.ndarray | None = None
    depth_anything_stamp_ns: int | None = None
    depth_anything_delta_ms: float | None = None
    depth_anything_nearest_stamp_ns: int | None = None
    depth_anything_nearest_delta_ms: float | None = None


@dataclass(frozen=True)
class PreviewMatchResult:
    image_bgr: np.ndarray | None
    matched_stamp_ns: int | None
    matched_delta_ms: float | None
    nearest_stamp_ns: int | None
    nearest_delta_ms: float | None


@dataclass
class MeasurementEvent:
    key: MeasurementEventKey
    stamp_ns: int
    detected: bool
    count: int
    bboxes: tuple[tuple[int, int, int, int], ...]
    image_width: int
    image_height: int
    estimates: dict[str, float | None] = field(default_factory=dict)
    estimate_statuses: dict[str, int] = field(default_factory=dict)
    preview: EventPreview = field(default_factory=EventPreview)


def stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def measurement_message_key(msg: G1Measurements) -> MeasurementEventKey:
    return (
        msg.header.frame_id,
        int(msg.header.stamp.sec),
        int(msg.header.stamp.nanosec),
        int(msg.count),
        tuple(float(value) for value in msg.bbox_xyxy),
    )


def extract_public_estimator_values(msg: G1Measurements) -> dict[str, float | None]:
    return {
        estimator: first_finite_positive(getattr(msg, field_key))
        for estimator, field_key in ESTIMATOR_FIELD_KEYS.items()
    }


def extract_estimator_statuses(msg: G1Measurements) -> dict[str, int]:
    """Per-estimator status code for the first detection (count == 1 frames).

    Mask estimators carry an explicit ``*_status`` array from the node; the
    legacy estimators have none, so a coarse ``OK``/``UNSET`` is inferred from
    whether they published a finite distance.
    """

    statuses: dict[str, int] = {}
    for estimator, field_key in ESTIMATOR_FIELD_KEYS.items():
        status_key = ESTIMATOR_STATUS_FIELD_KEYS.get(estimator)
        if status_key is not None:
            values = getattr(msg, status_key)
            statuses[estimator] = int(values[0]) if len(values) > 0 else int(MissReason.UNSET)
        else:
            value = first_finite_positive(getattr(msg, field_key))
            statuses[estimator] = int(MissReason.OK if value is not None else MissReason.UNSET)
    return statuses


def ensure_measurement_event(
    events: dict[MeasurementEventKey, MeasurementEvent],
    msg: G1Measurements,
) -> MeasurementEvent:
    key = measurement_message_key(msg)
    event = events.get(key)
    if event is not None:
        return event

    event = MeasurementEvent(
        key=key,
        stamp_ns=stamp_to_nanoseconds(msg.header.stamp),
        detected=bool(msg.detected),
        count=int(msg.count),
        bboxes=tuple(decode_bbox_quads(msg.bbox_xyxy)),
        image_width=int(msg.image_width),
        image_height=int(msg.image_height),
    )
    events[key] = event
    return event


def update_measurement_event(
    event: MeasurementEvent,
    msg: G1Measurements,
    allowed_estimators: set[str],
) -> None:
    event.detected = bool(msg.detected)
    event.count = int(msg.count)
    event.stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
    event.bboxes = tuple(decode_bbox_quads(msg.bbox_xyxy))
    event.image_width = int(msg.image_width)
    event.image_height = int(msg.image_height)

    for estimator, value in extract_public_estimator_values(msg).items():
        if estimator in allowed_estimators:
            event.estimates[estimator] = value

    for estimator, status in extract_estimator_statuses(msg).items():
        if estimator in allowed_estimators:
            event.estimate_statuses[estimator] = status


def has_all_selected_estimates(
    event: MeasurementEvent,
    selected_estimators: tuple[str, ...],
) -> bool:
    return all(event.estimates.get(estimator) is not None for estimator in selected_estimators)


def event_has_panel_preview(event: MeasurementEvent, estimator: str) -> bool:
    if estimator == 'sensor_depth':
        return event.preview.sensor_depth_bgr is not None
    if estimator == 'depth_anything':
        return event.preview.depth_anything_bgr is not None
    return event.preview.color_bgr is not None


def event_has_all_panel_previews(
    event: MeasurementEvent,
    selected_estimators: tuple[str, ...],
) -> bool:
    return all(event_has_panel_preview(event, estimator) for estimator in selected_estimators)


def find_exact_preview_match(
    preview_buffer: Mapping[int, np.ndarray],
    event_stamp_ns: int,
) -> PreviewMatchResult:
    nearest_stamp_ns, nearest_delta_ms = nearest_preview_metadata(preview_buffer, event_stamp_ns)
    preview = preview_buffer.get(event_stamp_ns)
    if preview is None:
        return PreviewMatchResult(
            image_bgr=None,
            matched_stamp_ns=None,
            matched_delta_ms=None,
            nearest_stamp_ns=nearest_stamp_ns,
            nearest_delta_ms=nearest_delta_ms,
        )

    return PreviewMatchResult(
        image_bgr=preview,
        matched_stamp_ns=event_stamp_ns,
        matched_delta_ms=0.0,
        nearest_stamp_ns=nearest_stamp_ns,
        nearest_delta_ms=nearest_delta_ms,
    )


def find_nearest_preview_match(
    preview_buffer: Mapping[int, np.ndarray],
    event_stamp_ns: int,
    tolerance_ns: int,
) -> PreviewMatchResult:
    nearest_stamp_ns, nearest_delta_ms = nearest_preview_metadata(preview_buffer, event_stamp_ns)
    if nearest_stamp_ns is None or nearest_delta_ms is None:
        return PreviewMatchResult(
            image_bgr=None,
            matched_stamp_ns=None,
            matched_delta_ms=None,
            nearest_stamp_ns=None,
            nearest_delta_ms=None,
        )

    if nearest_delta_ms > (tolerance_ns / 1_000_000.0):
        return PreviewMatchResult(
            image_bgr=None,
            matched_stamp_ns=None,
            matched_delta_ms=None,
            nearest_stamp_ns=nearest_stamp_ns,
            nearest_delta_ms=nearest_delta_ms,
        )

    return PreviewMatchResult(
        image_bgr=preview_buffer[nearest_stamp_ns],
        matched_stamp_ns=nearest_stamp_ns,
        matched_delta_ms=nearest_delta_ms,
        nearest_stamp_ns=nearest_stamp_ns,
        nearest_delta_ms=nearest_delta_ms,
    )


def nearest_preview_metadata(
    preview_buffer: Mapping[int, np.ndarray],
    event_stamp_ns: int,
) -> tuple[int | None, float | None]:
    if not preview_buffer:
        return None, None

    nearest_stamp_ns = min(
        preview_buffer.keys(),
        key=lambda stamp_ns: (abs(stamp_ns - event_stamp_ns), stamp_ns),
    )
    nearest_delta_ms = abs(nearest_stamp_ns - event_stamp_ns) / 1_000_000.0
    return nearest_stamp_ns, nearest_delta_ms
