from __future__ import annotations

import statistics

from ridgeback_autonomy.benchmarking.alignment import (
    MeasurementEvent,
    event_has_all_panel_previews,
    has_all_selected_estimates,
)


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
