"""Benchmark-only scoring assignment: pair each sensor-produced instance to a
ground-truth robot so its distance can be graded.

Separation of concerns (load-bearing): the estimators are sensor-only and never
see the true poses. Truth enters ONLY here, as the assignment target + the
reference distance -- never to detect, separate, or locate instances. The
assignment is driven by the *estimate*: each detected instance carries its own
estimated planar position (from a reference estimator) and is matched to the
nearest true robot. A GT with no assigned instance is MISSED (occluded); an
instance matching no GT (or beyond the sanity gate) is EXTRA.

Pure module (no ROS / numpy), unit-testable against synthetic estimates + poses.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


# If the nearest GT is farther than this from an instance's estimate, the pair
# is rejected (EXTRA) rather than forced -- stops a wildly-wrong estimate from
# mis-scoring against an unrelated robot. Only applied when the scene HAS an
# unrelated robot: see ``assign_to_ground_truth``.
ASSIGN_MAX_GATE_M = 1.5


@dataclass(frozen=True)
class InstanceEstimate:
    """One sensor-detected instance's estimated position (its locator).

    ``forward_m``/``lateral_m`` come from a reference estimator when available;
    ``distance_m`` is the fallback locator (1-D) when the reference estimator
    reports no planar position. ``index`` is the detection index in the frame.
    """

    index: int
    forward_m: float | None = None
    lateral_m: float | None = None
    distance_m: float | None = None


@dataclass(frozen=True)
class GtPoint:
    """One ground-truth robot's true planar position (the scoring reference)."""

    index: int
    forward_m: float
    lateral_m: float
    distance_m: float


@dataclass(frozen=True)
class Assignment:
    matches: tuple[tuple[int, int], ...]      # (gt_index, detection_index)
    missed_gt: tuple[int, ...]                # GT indices with no detection
    extra_detections: tuple[int, ...]         # detection indices matching no GT


def _pair_cost(instance: InstanceEstimate, gt: GtPoint) -> float | None:
    """Planar estimate<->truth distance; falls back to 1-D distance difference.

    Returns ``None`` when the instance carries no usable locator at all.
    """

    if instance.forward_m is not None and instance.lateral_m is not None:
        return math.hypot(instance.forward_m - gt.forward_m, instance.lateral_m - gt.lateral_m)
    if instance.distance_m is not None:
        return abs(instance.distance_m - gt.distance_m)
    return None


def assign_to_ground_truth(
    instances: list[InstanceEstimate],
    gts: list[GtPoint],
    max_gate_m: float = ASSIGN_MAX_GATE_M,
) -> Assignment:
    """Greedy nearest-estimate assignment of instances to ground-truth robots.

    All (instance, gt) pairs within the gate are ranked by cost ascending and
    assigned first-come, each instance/GT used at most once. Deterministic: ties
    break by (gt_index, detection_index).

    The gate is an ASSOCIATION device, not an outlier filter. It exists to stop a
    wildly-wrong estimate from being scored against an unrelated robot, so it
    only applies when the scene contains an unrelated robot to confuse it with.
    With a single GT the estimate is assigned however far off it is, and its
    error is graded in full -- gating there would quietly drop the worst trials
    out of the MAE instead of reporting them.
    """

    gate_m = max_gate_m if len(gts) > 1 else math.inf

    candidates: list[tuple[float, int, int]] = []
    for gt in gts:
        for instance in instances:
            cost = _pair_cost(instance, gt)
            if cost is None or cost > gate_m:
                continue
            candidates.append((cost, gt.index, instance.index))
    candidates.sort(key=lambda item: (item[0], item[1], item[2]))

    used_gt: set[int] = set()
    used_det: set[int] = set()
    matches: list[tuple[int, int]] = []
    for _cost, gt_index, det_index in candidates:
        if gt_index in used_gt or det_index in used_det:
            continue
        used_gt.add(gt_index)
        used_det.add(det_index)
        matches.append((gt_index, det_index))

    matches.sort()
    missed_gt = tuple(sorted(gt.index for gt in gts if gt.index not in used_gt))
    extra_detections = tuple(
        sorted(instance.index for instance in instances if instance.index not in used_det))
    return Assignment(
        matches=tuple(matches),
        missed_gt=missed_gt,
        extra_detections=extra_detections,
    )
