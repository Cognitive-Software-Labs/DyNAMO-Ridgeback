"""Per-instance scoring for multi-robot scenes.

Association runs once per frame (sensor-only, estimate-driven -- see
``association.py``); the matched detection's per-estimator distance is collected
across frames and reduced to a median per (ground-truth instance, estimator). A
ground-truth robot never matched in any frame is MISSED (occluded/undetected)
and is not scored -- only counted -- so occlusion cannot flatter the MAE.

Pure module (no ROS); the runner supplies decoded per-detection measurements.
"""

from __future__ import annotations

import statistics

from ridgeback_autonomy.benchmarking.association import (
    GtPoint,
    InstanceEstimate,
    assign_to_ground_truth,
)
from ridgeback_autonomy.benchmarking.estimators import (
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_POSITION_ATTRS,
)


# Preference order for the planar locator used to associate a detection to a GT:
# the most position-reliable estimators first. Only estimators actually selected
# for the run are considered; if none provide a finite planar position the
# instance falls back to a 1-D distance locator.
LOCATOR_PRIORITY = (
    'lidar',
    'pointcloud',
    'projective_ranging',
    'euclidean_reconstruction',
    'rgb',
    'polar_profiling',
)


def build_instance_estimate(detection, index: int, selected_estimators) -> InstanceEstimate:
    """Sensor-side locator for one detection: planar from the best selected
    position estimator, else a distance fallback."""

    forward = lateral = distance = None
    for estimator in LOCATOR_PRIORITY:
        if estimator not in selected_estimators or estimator not in ESTIMATOR_POSITION_ATTRS:
            continue
        forward_attr, lateral_attr = ESTIMATOR_POSITION_ATTRS[estimator]
        candidate_forward = getattr(detection, forward_attr)
        candidate_lateral = getattr(detection, lateral_attr)
        if candidate_forward is not None and candidate_lateral is not None:
            forward, lateral = float(candidate_forward), float(candidate_lateral)
            break
    for estimator in selected_estimators:
        candidate = getattr(detection, ESTIMATOR_FIELD_KEYS[estimator])
        if candidate is not None:
            distance = float(candidate)
            break
    return InstanceEstimate(index=index, forward_m=forward, lateral_m=lateral, distance_m=distance)


def score_scene(usable_events, gt_instances, selected_estimators):
    """Median per (GT instance, estimator) across frames + missed/extra counts.

    Returns ``(instance_medians, missed_gt, extra_count)`` where
    ``instance_medians[gt_index][estimator]`` is a float or ``None``,
    ``missed_gt`` is the tuple of GT indices never matched, and ``extra_count``
    is the peak number of unmatched detections in any single frame.
    """

    gt_points = [
        GtPoint(index=gt.index, forward_m=gt.forward_m, lateral_m=gt.lateral_m,
                distance_m=gt.distance_m)
        for gt in gt_instances
    ]
    collected = {gt.index: {est: [] for est in selected_estimators} for gt in gt_instances}
    matched_any = {gt.index: False for gt in gt_instances}
    extra_per_frame: list[int] = []

    for event in usable_events:
        instances = [
            build_instance_estimate(detection, index, selected_estimators)
            for index, detection in enumerate(event.detections)
        ]
        assignment = assign_to_ground_truth(instances, gt_points)
        extra_per_frame.append(len(assignment.extra_detections))
        for gt_index, det_index in assignment.matches:
            matched_any[gt_index] = True
            detection = event.detections[det_index]
            for estimator in selected_estimators:
                value = getattr(detection, ESTIMATOR_FIELD_KEYS[estimator])
                if value is not None:
                    collected[gt_index][estimator].append(float(value))

    instance_medians = {
        gt_index: {
            estimator: (statistics.median(values) if values else None)
            for estimator, values in per_estimator.items()
        }
        for gt_index, per_estimator in collected.items()
    }
    missed_gt = tuple(sorted(gt.index for gt in gt_instances if not matched_any[gt.index]))
    extra_count = max(extra_per_frame) if extra_per_frame else 0
    return instance_medians, missed_gt, extra_count
