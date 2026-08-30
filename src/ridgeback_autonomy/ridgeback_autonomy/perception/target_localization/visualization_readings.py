"""Select one coherent estimator snapshot for every visualization surface.

This module owns the producer-liveness and observation-age gates, the
same-batch/aged split, and the single reading map shared by the HUD and marker
renderers.  It deliberately has no ROS node or TF dependencies: callers hand
it cached messages and times, and both renderers consume its immutable result.
"""

from __future__ import annotations

from dataclasses import dataclass

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_POSITION_ATTRS,
    nearest_instance_index,
)


def _array_value(values, index: int) -> float | None:
    if values is not None and len(values) > index and values[index] == values[index]:
        return float(values[index])
    return None


def estimator_reading(
    msg,
    estimator: str,
    index: int,
) -> tuple[float | None, float | None, float | None]:
    """Return ``(forward, lateral, distance)`` for one estimator detection."""

    position_attrs = ESTIMATOR_POSITION_ATTRS.get(estimator)
    forward = lateral = None
    if position_attrs is not None:
        forward_attr, lateral_attr = position_attrs
        forward = _array_value(getattr(msg, forward_attr, None), index)
        lateral = _array_value(getattr(msg, lateral_attr, None), index)
    distance = _array_value(
        getattr(msg, ESTIMATOR_FIELD_KEYS[estimator], None), index)
    return forward, lateral, distance


def merged_distance_reader(batch: list):
    """Build an ``(estimator, index) -> distance`` reader across producers."""

    def read_distance(estimator: str, index: int) -> float | None:
        for msg in batch:
            if not msg.detected or index >= msg.count:
                continue
            _, _, distance = estimator_reading(msg, estimator, index)
            if distance is not None:
                return distance
        return None

    return read_distance


def nearest_detection_index(batch: list) -> int:
    """Return the detection every visualization surface should describe."""

    count = max((msg.count for msg in batch if msg.detected), default=0)
    nearest = nearest_instance_index(count, merged_distance_reader(batch))
    return 0 if nearest is None else nearest


def stamp_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def partition_measurements(
    cached,
    now_nanoseconds: int,
    liveness_budget_s: float,
    max_observation_age_s: float,
) -> tuple[list, list]:
    """Split cached messages into ``(same_batch, aged)`` and discard stale data.

    Liveness is measured from receipt time, because mask inference can make a
    valid observation old before it arrives.  Scene validity is measured from
    the observation stamp.  Surviving messages with the newest stamp form the
    only batch whose detection indices may be merged; older messages may still
    supply an estimator reading, but are ranked only against themselves.
    """

    live = []
    for entry in cached:
        if entry is None:
            continue
        msg, receipt_nanoseconds = entry
        if now_nanoseconds - receipt_nanoseconds > liveness_budget_s * 1_000_000_000:
            continue
        stamp = stamp_nanoseconds(msg.header.stamp)
        if now_nanoseconds - stamp > max_observation_age_s * 1_000_000_000:
            continue
        live.append((msg, stamp))

    if not live:
        return [], []

    newest = max(stamp for _, stamp in live)
    dated = [
        (msg, (now_nanoseconds - stamp) / 1_000_000_000, stamp)
        for msg, stamp in live
    ]
    same_batch = [(msg, age_s) for msg, age_s, stamp in dated if stamp == newest]
    aged = [(msg, age_s) for msg, age_s, stamp in dated if stamp != newest]
    return same_batch, aged


def batch_messages(entries: list) -> list:
    """Drop ages from ``(message, age_seconds)`` entries."""

    return [msg for msg, _ in entries]


@dataclass(frozen=True)
class EstimatorReading:
    """One estimator answer rendered by both the HUD and marker layers.

    ``stamp`` remains the source message stamp so marker placement can use the
    robot pose from the observation instant rather than its latest pose.
    """

    distance_m: float
    age_s: float
    aged: bool
    forward_m: float | None
    lateral_m: float | None
    stamp: object


def collect_readings(
    same_batch: list,
    aged: list,
    nearest: int,
    estimators: tuple[str, ...],
) -> dict[str, EstimatorReading]:
    """Return the single estimator snapshot every surface renders.

    A fresh reading wins over an aged reading for the same estimator.  An aged
    message is ranked against itself because its detection indices belong to a
    different detector batch.
    """

    readings: dict[str, EstimatorReading] = {}
    for msg, age_s in same_batch:
        if not msg.detected or nearest >= msg.count:
            continue
        for estimator in estimators:
            forward, lateral, distance = estimator_reading(msg, estimator, nearest)
            if distance is not None:
                readings[estimator] = EstimatorReading(
                    distance_m=distance,
                    age_s=age_s,
                    aged=False,
                    forward_m=forward,
                    lateral_m=lateral,
                    stamp=msg.header.stamp,
                )

    for msg, age_s in aged:
        index = nearest_detection_index([msg])
        if not msg.detected or index >= msg.count:
            continue
        for estimator in estimators:
            if estimator in readings:
                continue
            forward, lateral, distance = estimator_reading(msg, estimator, index)
            if distance is not None:
                readings[estimator] = EstimatorReading(
                    distance_m=distance,
                    age_s=age_s,
                    aged=True,
                    forward_m=forward,
                    lateral_m=lateral,
                    stamp=msg.header.stamp,
                )

    return readings
