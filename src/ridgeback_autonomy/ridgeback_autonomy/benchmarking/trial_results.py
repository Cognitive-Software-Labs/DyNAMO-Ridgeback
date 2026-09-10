"""Pure construction of benchmark trials, rows, and collage annotations."""

from __future__ import annotations

from collections import Counter
from typing import Any

from ridgeback_autonomy.benchmarking.association import GtPoint, assign_to_ground_truth
from ridgeback_autonomy.benchmarking.event_values import detection_status
from ridgeback_autonomy.benchmarking.reduction import dominant_miss_reason
from ridgeback_autonomy.benchmarking.scoring import (
    OUTCOME_DETECTOR_MISS,
    OUTCOME_NO_VALUE,
    SceneScore,
    build_display_instance_estimate,
    build_instance_estimate,
)
from ridgeback_autonomy.benchmarking.scenarios import Scene
from ridgeback_autonomy.benchmarking.simulation import GroundTruthInstance


def build_trials(scenes: tuple[Scene, ...] | list[Scene], repeats: int) -> list[dict[str, Any]]:
    """Expand scenes and repeat overrides into stable trial ids."""

    trials = []
    for scene in scenes:
        effective_repeats = scene.repeats_override or repeats
        for repeat_index in range(effective_repeats):
            trial_id = (
                scene.id
                if effective_repeats == 1
                else f'{scene.id}_rep{repeat_index + 1}'
            )
            trials.append({
                'trial_id': trial_id,
                'repeat_index': repeat_index + 1,
                'scene': scene,
            })
    return trials


def compute_instance_status_histogram(
    captured_events,
    ground_truth_instances: list[GroundTruthInstance],
    selected_estimators: tuple[str, ...],
) -> dict[int, dict[str, dict[int, int]]]:
    """Count statuses only where that estimator locates the same detection.

    A status is per detected box, while a trial row is per ground-truth target.
    The two granularities may be joined only through the named estimator's own
    association. Borrowing another estimator's locator or assuming box order
    would manufacture identity precisely when the estimator produced no value.
    Unmatched statuses intentionally remain only in the observation histogram.
    """

    ground_truth_points = [
        GtPoint(
            index=truth.index,
            forward_m=truth.forward_m,
            lateral_m=truth.lateral_m,
            distance_m=truth.distance_m,
        )
        for truth in ground_truth_instances
    ]
    counts: dict[int, dict[str, Counter]] = {
        truth.index: {estimator: Counter() for estimator in selected_estimators}
        for truth in ground_truth_instances
    }
    for event in captured_events:
        for estimator in selected_estimators:
            estimates = [
                build_instance_estimate(detection, index, estimator)
                for index, detection in enumerate(event.detections)
            ]
            assignment = assign_to_ground_truth(estimates, ground_truth_points)
            matches = assignment.matches
            if not matches and len(ground_truth_points) == 1 and len(event.detections) == 1:
                # No estimator locator exists, but there is still only one
                # possible box-to-target identity in this event.
                matches = ((ground_truth_points[0].index, 0),)
            for ground_truth_index, detection_index in matches:
                counts[ground_truth_index][estimator][detection_status(
                    event.detections[detection_index], estimator)] += 1

    return {
        ground_truth_index: {
            estimator: dict(code_counts)
            for estimator, code_counts in per_estimator.items()
        }
        for ground_truth_index, per_estimator in counts.items()
    }


def build_trial_result(
    trial: dict[str, Any],
    scene: Scene,
    ground_truth_instances: list[GroundTruthInstance],
    scene_score: SceneScore,
    selected_estimators: tuple[str, ...],
    estimator_display_names: dict[str, str],
    usable_by_estimator: dict[str, list],
    image_path: str,
    frames_captured: int,
    captured_events=(),
) -> dict[str, Any]:
    """Build one scored-or-missed row per target instance and estimator."""

    instance_status_histogram = compute_instance_status_histogram(
        captured_events, ground_truth_instances, selected_estimators)
    rows: dict[str, list[dict[str, Any]]] = {
        estimator: [] for estimator in selected_estimators}
    for ground_truth in ground_truth_instances:
        medians = scene_score.medians.get(ground_truth.index, {})
        outcomes = scene_score.outcomes.get(ground_truth.index, {})
        for estimator in selected_estimators:
            estimate = medians.get(estimator)
            if estimate is not None:
                absolute_error = abs(estimate - ground_truth.distance_m)
                relative_error = (
                    absolute_error / ground_truth.distance_m
                    if ground_truth.distance_m > 0.0 else None
                )
            else:
                absolute_error = None
                relative_error = None
            outcome = outcomes.get(estimator, OUTCOME_DETECTOR_MISS)
            if outcome == OUTCOME_NO_VALUE:
                # Use only statuses joined through this estimator's association
                # (or the unambiguous one-target/one-box case). An empty result
                # is intentionally unknown.
                code_counts = instance_status_histogram[ground_truth.index][estimator]
                miss_reason = dominant_miss_reason(code_counts)
            else:
                miss_reason = None
            rows[estimator].append({
                'trial_id': trial['trial_id'],
                'repeat_index': trial['repeat_index'],
                'scene_id': scene.id,
                'instance_index': ground_truth.index,
                'spawn_world_x': ground_truth.world_x,
                'spawn_world_y': ground_truth.world_y,
                'spawn_yaw_rad': scene.robots[ground_truth.index].yaw,
                'true_forward_m': ground_truth.forward_m,
                'true_lateral_m': ground_truth.lateral_m,
                'true_distance_m': ground_truth.distance_m,
                'estimator': estimator_display_names[estimator],
                'outcome': outcome,
                'miss_reason': miss_reason,
                'trial_estimate_m': estimate,
                'abs_error_m': absolute_error,
                'rel_error': relative_error,
                'usable_aligned_events': len(usable_by_estimator.get(estimator, [])),
                'frames_captured': frames_captured,
                'image_path': image_path,
            })

    return {
        'rows': rows,
        'missed_count': len(scene_score.detector_missed),
        'outcome_counts': {
            estimator: scene_score.outcome_counts(estimator)
            for estimator in selected_estimators
        },
        'extra_count': scene_score.extra_count,
    }


def build_box_annotations(
    representative_event,
    ground_truth_instances: list[GroundTruthInstance],
    selected_estimators: tuple[str, ...],
) -> list[dict | None]:
    """Associate displayed boxes with truth instances for collage labels only."""

    estimates = [
        build_display_instance_estimate(detection, index, selected_estimators)
        for index, detection in enumerate(representative_event.detections)
    ]
    ground_truth_points = [
        GtPoint(
            index=truth.index,
            forward_m=truth.forward_m,
            lateral_m=truth.lateral_m,
            distance_m=truth.distance_m,
        )
        for truth in ground_truth_instances
    ]
    assignment = assign_to_ground_truth(estimates, ground_truth_points)
    true_by_index = {
        truth.index: truth.distance_m for truth in ground_truth_instances}
    annotations: list[dict | None] = [None] * len(representative_event.detections)
    for ground_truth_index, detection_index in assignment.matches:
        annotations[detection_index] = {
            'instance_index': ground_truth_index,
            'true_distance_m': true_by_index[ground_truth_index],
        }
    return annotations


def dominant_miss_reasons(
    selected_estimators: tuple[str, ...],
    status_histogram: dict[str, dict[int, int]],
    trial_medians: dict[str, float],
) -> dict[str, str]:
    """Return the dominant reason for every selected estimator without a median."""

    reasons = {}
    for estimator in selected_estimators:
        if estimator in trial_medians:
            continue
        reason = dominant_miss_reason(status_histogram.get(estimator))
        if reason is not None:
            reasons[estimator] = reason
    return reasons
