from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from ridgeback_autonomy.common.messages import (
    batch_from_measurements_message,
    decode_bbox_quads,
    first_finite_positive,
)
from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.common.models import Detection
from ridgeback_autonomy.msg import G1Measurements

from ridgeback_autonomy.benchmarking.estimators import (
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_POSITION_ATTRS,
    ESTIMATOR_STATUS_FIELD_KEYS,
)


MeasurementEventKey = tuple[Any, ...]


@dataclass
class EventPreview:
    """The colour frame a trial's collage panels are drawn on.

    One image for the whole event: every estimator's panel is the same colour
    frame annotated differently, so there is nothing per-estimator to buffer.
    """

    color_bgr: np.ndarray | None = None
    color_stamp_ns: int | None = None
    color_delta_ms: float | None = None
    color_nearest_stamp_ns: int | None = None
    color_nearest_delta_ms: float | None = None


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
    # Scalar per-frame estimates (first finite detection): the single-instance
    # view used by the collage and the single-robot scoring path.
    estimates: dict[str, float | None] = field(default_factory=dict)
    estimate_statuses: dict[str, int] = field(default_factory=dict)
    # Per-detection measurements, merged across the pointcloud / mask
    # messages that share this frame's alignment key. Index-aligned with
    # ``bboxes``; the multi-instance scoring path reads these + associates them
    # to ground truth. Empty until the first measurement message is merged.
    detections: list[Detection] = field(default_factory=list)
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
    pointcloud row has none, so a coarse ``OK``/``UNSET`` is inferred from
    whether it published a finite distance.
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


def detection_status(detection: Detection, estimator: str) -> int:
    """One estimator's status code on ONE detection.

    The per-detection counterpart of ``extract_estimator_statuses``, which only
    ever reads index 0. Mask estimators carry an explicit ``*_status`` decoded
    per detection; the pointcloud row publishes none, so the same coarse
    ``OK``/``UNSET`` is inferred from whether THIS detection got a distance.
    """

    status_key = ESTIMATOR_STATUS_FIELD_KEYS.get(estimator)
    if status_key is not None:
        code = getattr(detection, status_key)
        return int(code) if code is not None else int(MissReason.UNSET)
    value = getattr(detection, ESTIMATOR_FIELD_KEYS[estimator])
    return int(MissReason.OK if value is not None else MissReason.UNSET)


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

    merge_measurement_detections(event, msg, allowed_estimators)


def merge_measurement_detections(
    event: MeasurementEvent,
    msg: G1Measurements,
    allowed_estimators: set[str],
) -> None:
    """Merge this message's per-detection values into the event's detection table.

    The pointcloud and mask messages of one frame share the alignment key (same
    count + bbox tuple), so their detections are index-aligned. Each message only
    carries its own node's estimators; we copy exactly the ``allowed_estimators``
    so a later message never erases an earlier one's fields.
    """

    batch = batch_from_measurements_message(msg)
    if len(event.detections) != len(batch.detections):
        # First message for this frame (or the count changed): seed the table
        # with identity-only detections, then merge this message's estimators.
        event.detections = [
            Detection(bbox_xyxy=det.bbox_xyxy, label=det.label, score=det.score)
            for det in batch.detections
        ]
    for target, source in zip(event.detections, batch.detections):
        for estimator in allowed_estimators:
            _copy_estimator_fields(target, source, estimator)


def _copy_estimator_fields(target: Detection, source: Detection, estimator: str) -> None:
    distance_attr = ESTIMATOR_FIELD_KEYS[estimator]
    setattr(target, distance_attr, getattr(source, distance_attr))
    position = ESTIMATOR_POSITION_ATTRS.get(estimator)
    if position is not None:
        forward_attr, lateral_attr = position
        setattr(target, forward_attr, getattr(source, forward_attr))
        setattr(target, lateral_attr, getattr(source, lateral_attr))
    # The mask estimators' status rides along with their values: the reason
    # tallies read it off this table, so dropping it here reports a working
    # estimator as UNSET on every box.
    status_attr = ESTIMATOR_STATUS_FIELD_KEYS.get(estimator)
    if status_attr is not None:
        setattr(target, status_attr, getattr(source, status_attr))


def has_all_selected_estimates(
    event: MeasurementEvent,
    selected_estimators: tuple[str, ...],
) -> bool:
    return all(event.estimates.get(estimator) is not None for estimator in selected_estimators)


def event_has_panel_preview(event: MeasurementEvent) -> bool:
    """Whether this event's collage panels can be drawn at all.

    One question for the whole event rather than one per estimator: every panel
    is the same colour frame, so either all of them render or none do.
    """

    return event.preview.color_bgr is not None


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
