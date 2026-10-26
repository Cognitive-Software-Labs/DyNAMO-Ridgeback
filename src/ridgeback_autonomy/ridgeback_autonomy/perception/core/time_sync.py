from __future__ import annotations

from collections.abc import Iterable
from typing import TypeVar


T = TypeVar('T')


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def select_closest_timestamped(
    samples: Iterable[tuple[float, T]],
    target_stamp_s: float,
    tolerance_s: float,
) -> tuple[T, float] | None:
    """Return the nearest sample and signed sample-target delta."""
    if target_stamp_s <= 0.0 or tolerance_s < 0.0:
        return None

    closest: tuple[T, float] | None = None
    closest_abs_dt = float('inf')
    for sample_stamp_s, sample in samples:
        if sample_stamp_s <= 0.0:
            continue
        dt_s = sample_stamp_s - target_stamp_s
        abs_dt_s = abs(dt_s)
        if abs_dt_s < closest_abs_dt:
            closest = (sample, dt_s)
            closest_abs_dt = abs_dt_s

    if closest is None or closest_abs_dt > tolerance_s:
        return None
    return closest


def prune_timestamped_buffer(
    samples,
    newest_stamp_s: float,
    max_age_s: float,
) -> None:
    if newest_stamp_s <= 0.0 or max_age_s <= 0.0:
        return
    oldest_allowed_s = newest_stamp_s - max_age_s
    while samples and samples[0][0] < oldest_allowed_s:
        samples.popleft()
