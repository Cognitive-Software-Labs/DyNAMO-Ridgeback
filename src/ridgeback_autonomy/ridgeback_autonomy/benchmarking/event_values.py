"""Pure value accessors shared by live capture and offline replay."""

from __future__ import annotations

from typing import Any

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_STATUS_FIELD_KEYS,
)


def detection_status(detection: Any, estimator: str) -> int:
    """Return one estimator's status code for one detection-like value."""

    status_key = ESTIMATOR_STATUS_FIELD_KEYS.get(estimator)
    if status_key is not None:
        code = getattr(detection, status_key)
        return int(code) if code is not None else int(MissReason.UNSET)
    value = getattr(detection, ESTIMATOR_FIELD_KEYS[estimator])
    return int(MissReason.OK if value is not None else MissReason.UNSET)


def has_all_selected_estimates(
    event: Any,
    selected_estimators: tuple[str, ...],
) -> bool:
    return all(event.estimates.get(estimator) is not None for estimator in selected_estimators)


def event_has_panel_preview(event: Any) -> bool:
    """Whether this event's collage panels can be drawn at all."""

    return event.preview.color_bgr is not None
