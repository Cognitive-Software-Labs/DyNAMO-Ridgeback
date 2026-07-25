from __future__ import annotations

import statistics
from collections import Counter

from ridgeback_autonomy.benchmarking.alignment import (
    MeasurementEvent,
    event_has_all_panel_previews,
    has_all_selected_estimates,
)
from ridgeback_autonomy.common.miss_reason import MissReason, reason_name


def usable_aligned_events(
    events: dict,
    selected_estimators: tuple[str, ...],
) -> list[MeasurementEvent]:
    # No count==1 gate: multi-robot frames are kept and scored per instance via
    # the association path. An event is usable if it detected something and at
    # least one detection carries every selected estimator (has_all_selected).
    return sorted(
        (
            event for event in events.values()
            if event.detected and has_all_selected_estimates(event, selected_estimators)
        ),
        key=lambda event: event.stamp_ns,
    )


def compute_status_histogram(
    events: dict,
    selected_estimators: tuple[str, ...],
) -> dict[str, dict[int, int]]:
    """Per-estimator count of status codes over **all** captured events.

    Runs over every ``count == 1`` event (not just the usable-aligned ones),
    so misses are visible: an estimator that never produced a value still shows
    its reason tally. Codes are ``MissReason`` values; a missing/absent status
    is counted as ``UNSET``.
    """

    histogram: dict[str, Counter] = {estimator: Counter() for estimator in selected_estimators}
    for event in events.values():
        if int(event.count) != 1:
            continue
        for estimator in selected_estimators:
            code = event.estimate_statuses.get(estimator)
            histogram[estimator][int(code if code is not None else MissReason.UNSET)] += 1
    return {estimator: dict(counter) for estimator, counter in histogram.items()}


def merge_status_histograms(
    aggregate: dict[str, dict[int, int]],
    trial: dict[str, dict[int, int]],
) -> None:
    """Fold one trial's histogram into a running aggregate, in place."""

    for estimator, code_counts in trial.items():
        target = aggregate.setdefault(estimator, {})
        for code, count in code_counts.items():
            target[code] = target.get(code, 0) + count


def format_status_tally(
    histogram: dict[str, dict[int, int]],
    selected_estimators: tuple[str, ...],
) -> str:
    """A compact ``projective 20/20, polar 0/20 (SCAN_INVALID)`` line naming the
    culprit of a skipped trial."""

    parts: list[str] = []
    for estimator in selected_estimators:
        code_counts = histogram.get(estimator, {})
        total = sum(code_counts.values())
        ok = code_counts.get(int(MissReason.OK), 0)
        short = estimator.split('_')[0]
        if ok == total:
            parts.append(f'{short} {ok}/{total}')
        else:
            misses = {code: count for code, count in code_counts.items() if code != int(MissReason.OK)}
            dominant = max(misses, key=misses.get) if misses else int(MissReason.UNSET)
            parts.append(f'{short} {ok}/{total} ({reason_name(dominant)})')
    return ', '.join(parts)


def compute_trial_medians(
    usable_events: list[MeasurementEvent],
    selected_estimators: tuple[str, ...],
) -> dict[str, float]:
    return {
        estimator: float(statistics.median([
            event.estimates[estimator]
            for event in usable_events
            if event.estimates.get(estimator) is not None
        ]))
        for estimator in selected_estimators
    }


def choose_representative_event(
    usable_events: list[MeasurementEvent],
    selected_estimators: tuple[str, ...],
    trial_medians: dict[str, float],
) -> MeasurementEvent:
    return min(
        usable_events,
        key=lambda event: (
            0 if event_has_all_panel_previews(event, selected_estimators) else 1,
            sum(
                abs(event.estimates[estimator] - trial_medians[estimator])
                for estimator in selected_estimators
            ),
            event.stamp_ns,
        ),
    )
