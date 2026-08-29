"""Per-instance scoring for multi-robot scenes.

Every estimator is associated INDEPENDENTLY: each one locates the detections
with its own estimate, runs its own assignment against the ground-truth robots
(see ``association.py``), and the matched detection's distance is collected
across frames and reduced to a median per (ground-truth instance, estimator).

Independence is the point. A shared locator would let one bad estimator decide
the assignment for all of them, so a single gross localization error would erase
the instance from every estimator's score -- including the estimators that were
right. Instead each estimator carries its own outcome per instance:

``scored``          matched; median + error recorded.
``gate_miss``       produced values, but none landed within the association gate
                    of this instance (only reachable in multi-robot scenes,
                    where the gate runs). A gross localization error.
``no_value``        produced no value on any frame; the reason is in the status
                    histogram (``run.json``'s ``reason_histogram``, and the
                    "Why boxes went unmeasured" section of ``summary.md``).
``detector_miss``   no estimator matched this instance in any frame -- occluded
                    or never detected, so nobody had a chance at it.

A ground-truth robot that is not ``scored`` is counted, never graded, so
occlusion cannot flatter the MAE.

Pure module (no ROS); the runner supplies decoded per-detection measurements.
"""

from __future__ import annotations

from dataclasses import dataclass
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


OUTCOME_SCORED = 'scored'
OUTCOME_GATE_MISS = 'gate_miss'
OUTCOME_NO_VALUE = 'no_value'
OUTCOME_DETECTOR_MISS = 'detector_miss'

MISS_OUTCOMES = (OUTCOME_DETECTOR_MISS, OUTCOME_GATE_MISS, OUTCOME_NO_VALUE)


@dataclass(frozen=True)
class SceneScore:
    """Result of scoring one scene's capture window.

    ``medians[gt_index][estimator]`` is a float or ``None``;
    ``outcomes[gt_index][estimator]`` is one of the ``OUTCOME_*`` constants.
    ``detector_missed`` lists the GT indices no estimator ever matched.
    ``extra_count`` is the worst-case number of detections in a single frame
    that NO estimator could attribute to a robot -- a detector-level concept, so
    it counts a box only when every estimator failed to place it. Scoring it per
    estimator would let one bad localizer inflate the detector's phantom count.
    """

    medians: dict[int, dict[str, float | None]]
    outcomes: dict[int, dict[str, str]]
    detector_missed: tuple[int, ...]
    extra_count: int

    def outcome_counts(self, estimator: str) -> dict[str, int]:
        """Per-outcome tally for one estimator across the scene's instances.

        The single source for miss totals: the summary's
        ``missed_instance_count`` is the sum of the three miss outcomes here,
        not a separately counted number that could drift from them.
        """

        counts = {outcome: 0 for outcome in MISS_OUTCOMES}
        counts[OUTCOME_SCORED] = 0
        for per_estimator in self.outcomes.values():
            outcome = per_estimator.get(estimator)
            if outcome is not None:
                counts[outcome] += 1
        return counts


def build_instance_estimate(detection, index: int, estimator: str) -> InstanceEstimate:
    """One detection's locator AS SEEN BY ``estimator``.

    Planar when that estimator placed this detection, else its own scalar
    distance -- the 1-D fallback, which today is reached only when a
    position-capable row produced nothing on this box. Never borrows another
    estimator's position: the association has to reflect what THIS estimator
    believes, or its errors would be hidden behind a better one.
    """

    forward = lateral = None
    position_attrs = ESTIMATOR_POSITION_ATTRS.get(estimator)
    if position_attrs is not None:
        forward_attr, lateral_attr = position_attrs
        candidate_forward = getattr(detection, forward_attr)
        candidate_lateral = getattr(detection, lateral_attr)
        if candidate_forward is not None and candidate_lateral is not None:
            forward, lateral = float(candidate_forward), float(candidate_lateral)

    value = getattr(detection, ESTIMATOR_FIELD_KEYS[estimator])
    distance = float(value) if value is not None else None
    return InstanceEstimate(index=index, forward_m=forward, lateral_m=lateral, distance_m=distance)


def build_display_instance_estimate(detection, index: int, selected_estimators) -> InstanceEstimate:
    """A single locator for one detection, for COLLAGE LABELS ONLY.

    The collage draws one frame with one box-to-instance mapping, so it needs a
    single locator where scoring deliberately uses one per estimator. Takes the
    first selected estimator that offers a usable locator, in canonical order.
    Never used for grading -- picking one estimator to speak for the rest is
    exactly what ``score_scene`` avoids.
    """

    for estimator in selected_estimators:
        estimate = build_instance_estimate(detection, index, estimator)
        if estimate.forward_m is not None or estimate.distance_m is not None:
            return estimate
    return InstanceEstimate(index=index)


def score_scene(
    usable_by_estimator,
    gt_instances,
    selected_estimators,
    detector_fired: bool = True,
) -> SceneScore:
    """Score every (instance, estimator) pair over that estimator's usable events.

    ``usable_by_estimator`` maps estimator -> the frames where it produced a
    value, so an estimator blinded by design (occluder on the scan plane, target
    past the depth clamp) simply has an empty list and is charged a
    ``no_value`` miss rather than dragging the whole scene down with it.

    ``detector_fired`` says whether the detector reported ANY detection during
    the capture window. When it did not, no estimator could have produced a
    value, so every outcome is a ``detector_miss`` -- charging them all
    ``no_value`` would blame the estimators for a detection failure.
    """

    gt_points = [
        GtPoint(index=gt.index, forward_m=gt.forward_m, lateral_m=gt.lateral_m,
                distance_m=gt.distance_m)
        for gt in gt_instances
    ]
    collected = {gt.index: {est: [] for est in selected_estimators} for gt in gt_instances}
    matched_any = {gt.index: False for gt in gt_instances}
    # Per frame (keyed by identity, since the same event object is shared across
    # the estimators that could use it): how many boxes it held, and which of
    # them at least one estimator managed to attribute to a robot.
    frame_detection_counts: dict[int, int] = {}
    frame_attributed: dict[int, set[int]] = {}

    for estimator in selected_estimators:
        for event in usable_by_estimator.get(estimator, ()):
            instances = [
                build_instance_estimate(detection, index, estimator)
                for index, detection in enumerate(event.detections)
            ]
            assignment = assign_to_ground_truth(instances, gt_points)
            frame_detection_counts[id(event)] = len(event.detections)
            attributed = frame_attributed.setdefault(id(event), set())
            for gt_index, det_index in assignment.matches:
                attributed.add(det_index)
                value = getattr(
                    event.detections[det_index], ESTIMATOR_FIELD_KEYS[estimator])
                if value is None:
                    # Matched on this estimator's locator but carrying no
                    # distance: nothing to grade, and the reason is already in
                    # the status histogram.
                    continue
                matched_any[gt_index] = True
                collected[gt_index][estimator].append(float(value))

    medians = {
        gt_index: {
            estimator: (statistics.median(values) if values else None)
            for estimator, values in per_estimator.items()
        }
        for gt_index, per_estimator in collected.items()
    }
    outcomes = {
        gt_index: {
            estimator: _classify(
                median=per_estimator[estimator],
                had_values=bool(usable_by_estimator.get(estimator)),
                instance_found=matched_any[gt_index],
                detector_fired=detector_fired,
            )
            for estimator in selected_estimators
        }
        for gt_index, per_estimator in medians.items()
    }
    detector_missed = tuple(
        sorted(gt.index for gt in gt_instances if not matched_any[gt.index]))
    unattributed = [
        count - len(frame_attributed.get(frame_key, ()))
        for frame_key, count in frame_detection_counts.items()
    ]
    return SceneScore(
        medians=medians,
        outcomes=outcomes,
        detector_missed=detector_missed,
        extra_count=max(unattributed) if unattributed else 0,
    )


def _classify(
    median: float | None,
    had_values: bool,
    instance_found: bool,
    detector_fired: bool,
) -> str:
    """Which of the four outcomes one (instance, estimator) pair earned."""

    if median is not None:
        return OUTCOME_SCORED
    if not detector_fired:
        # Nothing was detected all window: no estimator could have spoken, so
        # this is the detector's miss, not theirs.
        return OUTCOME_DETECTOR_MISS
    if not had_values:
        # The estimator produced nothing anywhere, so it never had a shot at
        # this instance regardless of where the instance was.
        return OUTCOME_NO_VALUE
    if not instance_found:
        # Nobody matched this instance: a detector-level failure (occlusion),
        # not this estimator's fault.
        return OUTCOME_DETECTOR_MISS
    # Someone else located it and this estimator had values to offer, so its
    # own estimate landed outside the association gate.
    return OUTCOME_GATE_MISS
