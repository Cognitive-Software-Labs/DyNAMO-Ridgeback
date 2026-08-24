from __future__ import annotations

from ridgeback_autonomy.benchmarking.association import (
    ASSIGN_MAX_GATE_M,
    Assignment,
    GtPoint,
    InstanceEstimate,
    assign_to_ground_truth,
)


def test_two_separated_instances_match_one_to_one() -> None:
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),
        GtPoint(index=1, forward_m=3.0, lateral_m=1.5, distance_m=3.35),
    ]
    instances = [
        InstanceEstimate(index=0, forward_m=2.05, lateral_m=0.02, distance_m=2.05),
        InstanceEstimate(index=1, forward_m=2.95, lateral_m=1.48, distance_m=3.3),
    ]

    result = assign_to_ground_truth(instances, gts)

    assert result == Assignment(matches=((0, 0), (1, 1)), missed_gt=(), extra_detections=())


def test_inter_robot_occlusion_far_robot_missed() -> None:
    # Two robots on the same bearing; only the near one is detected. The far one
    # must be MISSED, not mis-assigned -- and no truth was used to detect it.
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),   # near
        GtPoint(index=1, forward_m=4.0, lateral_m=0.0, distance_m=4.0),   # far (occluded)
    ]
    instances = [InstanceEstimate(index=0, forward_m=2.03, lateral_m=0.0, distance_m=2.03)]

    result = assign_to_ground_truth(instances, gts)

    assert result.matches == ((0, 0),)
    assert result.missed_gt == (1,)
    assert result.extra_detections == ()


def test_estimate_beyond_gate_is_extra_and_gt_missed() -> None:
    # Two GTs, so the gate is live: an estimate 3 m from the nearer robot could
    # be mis-scored against either one, and is rejected instead.
    gts = [
        GtPoint(index=0, forward_m=5.0, lateral_m=0.0, distance_m=5.0),
        GtPoint(index=1, forward_m=8.0, lateral_m=0.0, distance_m=8.0),
    ]
    instances = [InstanceEstimate(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0)]

    result = assign_to_ground_truth(instances, gts)

    assert result.matches == ()
    assert result.missed_gt == (0, 1)
    assert result.extra_detections == (0,)


def test_single_gt_is_ungated_so_a_far_estimate_still_matches() -> None:
    # The gate disambiguates between robots; with only one there is nothing to
    # confuse, so the estimate is assigned and its error graded in full rather
    # than dropped out of the MAE.
    gts = [GtPoint(index=0, forward_m=5.0, lateral_m=0.0, distance_m=5.0)]
    instances = [InstanceEstimate(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0)]

    result = assign_to_ground_truth(instances, gts)

    assert result.matches == ((0, 0),)
    assert result.missed_gt == ()
    assert result.extra_detections == ()


def test_distance_only_locator_matches_on_distance() -> None:
    # sensor_depth-style instance: no planar position, only a distance.
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),
        GtPoint(index=1, forward_m=4.0, lateral_m=0.0, distance_m=4.0),
    ]
    instances = [InstanceEstimate(index=0, forward_m=None, lateral_m=None, distance_m=3.9)]

    result = assign_to_ground_truth(instances, gts)

    assert result.matches == ((1, 0),)
    assert result.missed_gt == (0,)


def test_duplicate_detection_nearest_wins_other_is_extra() -> None:
    gts = [GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0)]
    instances = [
        InstanceEstimate(index=0, forward_m=2.4, lateral_m=0.0, distance_m=2.4),
        InstanceEstimate(index=1, forward_m=2.05, lateral_m=0.0, distance_m=2.05),
    ]

    result = assign_to_ground_truth(instances, gts)

    assert result.matches == ((0, 1),)          # nearer detection (index 1) wins
    assert result.extra_detections == (0,)


def test_no_detections_all_gt_missed() -> None:
    gts = [GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0)]

    result = assign_to_ground_truth([], gts)

    assert result.matches == ()
    assert result.missed_gt == (0,)
    assert result.extra_detections == ()


def test_gate_default_value() -> None:
    assert ASSIGN_MAX_GATE_M == 1.5
