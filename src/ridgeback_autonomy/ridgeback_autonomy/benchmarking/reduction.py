from __future__ import annotations

import statistics
from collections import Counter

from ridgeback_autonomy.benchmarking.alignment import (
    MeasurementEvent,
    detection_status,
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


def usable_events_by_estimator(
    events: dict,
    selected_estimators: tuple[str, ...],
) -> dict[str, list[MeasurementEvent]]:
    """Per-estimator usable events: detected frames where THIS estimator has a
    finite value on some detection.

    Each estimator is scored on its own event set, so a scene that blinds one
    estimator by design (occluder on the scan plane, target beyond the depth
    clamp) yields a per-estimator miss with a reason instead of discarding the
    whole trial for everyone.
    """

    by_estimator: dict[str, list[MeasurementEvent]] = {
        estimator: [] for estimator in selected_estimators}
    for event in sorted(events.values(), key=lambda event: event.stamp_ns):
        if not event.detected:
            continue
        for estimator in selected_estimators:
            # estimates[e] is the first finite value across the message's
            # detections, so "present" == "some detection carries it".
            if event.estimates.get(estimator) is not None:
                by_estimator[estimator].append(event)
    return by_estimator


def union_usable_events(
    usable_by_estimator: dict[str, list[MeasurementEvent]],
) -> list[MeasurementEvent]:
    """All events usable for at least one estimator, stamp-ordered."""

    seen: dict[int, MeasurementEvent] = {}
    for events in usable_by_estimator.values():
        for event in events:
            seen[id(event)] = event
    return sorted(seen.values(), key=lambda event: event.stamp_ns)


def compute_status_histogram(
    events: dict,
    selected_estimators: tuple[str, ...],
) -> dict[str, dict[int, int]]:
    """Per-estimator count of status codes over every captured BOX.

    The unit is one detection (box-observation), not one frame, so a scene with
    two robots contributes both boxes per frame. Counting frames instead meant
    multi-robot scenes were either invisible (every frame held 2 boxes, so none
    qualified) or sampled only on the frames where the detector had already
    lost one -- exactly the failures, and nothing else.

    Single-robot scenes are unaffected: one box per frame makes detections and
    frames the same number. Codes are ``MissReason`` values; an absent status
    counts as ``UNSET``.
    """

    histogram: dict[str, Counter] = {estimator: Counter() for estimator in selected_estimators}
    for event in events.values():
        for detection in event.detections:
            for estimator in selected_estimators:
                histogram[estimator][detection_status(detection, estimator)] += 1
    return {estimator: dict(counter) for estimator, counter in histogram.items()}


def dominant_miss_reason(code_counts: dict[int, int] | None) -> str | None:
    """The reason that best explains why an estimator produced nothing.

    A specific reason beats ``UNSET``: ``UNSET`` means the frame never reached
    this estimator's node, which is usually the majority of the tally and never
    the story. Returns ``None`` when nothing failed (all ``OK`` / empty).
    """

    misses = {
        code: count for code, count in (code_counts or {}).items()
        if code != int(MissReason.OK)
    }
    if not misses:
        return None
    specific = {
        code: count for code, count in misses.items()
        if code != int(MissReason.UNSET)
    }
    if specific:
        return reason_name(max(specific, key=specific.get))
    return reason_name(int(MissReason.UNSET))


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
    usable_by_estimator: dict[str, list[MeasurementEvent]],
) -> dict[str, float]:
    """Median per estimator over ITS OWN usable events.

    Partial by design: an estimator with no usable events has no key, and its
    absence is scored as a per-estimator miss (reason in the status histogram).
    """

    return {
        estimator: float(statistics.median(
            [event.estimates[estimator] for event in events]))
        for estimator, events in usable_by_estimator.items()
        if events
    }


def choose_representative_event(
    usable_events: list[MeasurementEvent],
    selected_estimators: tuple[str, ...],
    trial_medians: dict[str, float],
) -> MeasurementEvent:
    """Pick the collage frame from the union of usable events.

    Prefer frames with panel previews, then frames covering the most
    estimators, then closeness to the trial medians over the estimators the
    frame actually carries.
    """

    def rank(event: MeasurementEvent):
        present = tuple(
            estimator for estimator in selected_estimators
            if estimator in trial_medians and event.estimates.get(estimator) is not None
        )
        preview_scope = present or selected_estimators
        return (
            0 if event_has_all_panel_previews(event, preview_scope) else 1,
            -len(present),
            sum(abs(event.estimates[estimator] - trial_medians[estimator])
                for estimator in present),
            event.stamp_ns,
        )

    return min(usable_events, key=rank)
