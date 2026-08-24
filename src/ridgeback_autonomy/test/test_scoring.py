from __future__ import annotations

from types import SimpleNamespace

import pytest

from ridgeback_autonomy.benchmarking.association import GtPoint
from ridgeback_autonomy.benchmarking.scoring import (
    OUTCOME_DETECTOR_MISS,
    OUTCOME_GATE_MISS,
    OUTCOME_NO_VALUE,
    OUTCOME_SCORED,
    build_display_instance_estimate,
    build_instance_estimate,
    score_scene,
)
from ridgeback_autonomy.common.models import Detection


def _lidar_detection(forward, lateral, distance):
    return Detection(
        bbox_xyxy=(0, 0, 10, 10), label='humanoid robot', score=0.9,
        lidar_forward_m=forward, lidar_lateral_m=lateral, lidar_distance_m=distance,
    )


def _event(detections):
    return SimpleNamespace(detections=detections)


def _lidar_only(events):
    """The per-estimator usable-event map for a lidar-only selection."""

    return {'lidar': events}


def test_single_robot_median_over_frames() -> None:
    gts = [GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0)]
    events = [
        _event([_lidar_detection(2.0, 0.0, 1.9)]),
        _event([_lidar_detection(2.0, 0.0, 2.0)]),
        _event([_lidar_detection(2.0, 0.0, 2.1)]),
    ]

    score = score_scene(_lidar_only(events), gts, ('lidar',))

    assert score.medians[0]['lidar'] == pytest.approx(2.0)
    assert score.outcomes[0]['lidar'] == OUTCOME_SCORED
    assert score.detector_missed == ()
    assert score.extra_count == 0


def test_two_robots_scored_independently() -> None:
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),
        GtPoint(index=1, forward_m=3.0, lateral_m=1.5, distance_m=3.35),
    ]
    events = [
        _event([_lidar_detection(2.0, 0.0, 2.02), _lidar_detection(3.0, 1.5, 3.30)]),
        _event([_lidar_detection(2.0, 0.0, 1.98), _lidar_detection(3.0, 1.5, 3.40)]),
    ]

    score = score_scene(_lidar_only(events), gts, ('lidar',))

    assert score.medians[0]['lidar'] == pytest.approx(2.0)
    assert score.medians[1]['lidar'] == pytest.approx(3.35)
    assert score.detector_missed == ()
    assert score.extra_count == 0


def test_occluded_far_robot_missed_not_scored() -> None:
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),   # near
        GtPoint(index=1, forward_m=4.0, lateral_m=0.0, distance_m=4.0),   # far, occluded
    ]
    events = [_event([_lidar_detection(2.0, 0.0, 2.0)]) for _ in range(3)]

    score = score_scene(_lidar_only(events), gts, ('lidar',))

    assert score.medians[0]['lidar'] == pytest.approx(2.0)
    assert score.medians[1]['lidar'] is None
    assert score.outcomes[1]['lidar'] == OUTCOME_DETECTOR_MISS
    assert score.detector_missed == (1,)


def test_distance_only_locator_used_when_estimator_has_no_planar_position() -> None:
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),
        GtPoint(index=1, forward_m=4.0, lateral_m=0.0, distance_m=4.0),
    ]
    near = Detection(bbox_xyxy=(0, 0, 10, 10), label='r', score=0.9, sensor_depth_distance_m=2.05)
    events = [_event([near])]

    score = score_scene({'sensor_depth': events}, gts, ('sensor_depth',))

    assert score.medians[0]['sensor_depth'] == pytest.approx(2.05)
    assert score.medians[1]['sensor_depth'] is None
    assert score.detector_missed == (1,)


def test_build_instance_estimate_uses_the_named_estimator_only() -> None:
    detection = _lidar_detection(2.5, -0.3, 2.52)

    lidar = build_instance_estimate(detection, index=0, estimator='lidar')
    # A different estimator on the same detection has no values of its own and
    # must NOT borrow lidar's position.
    polar = build_instance_estimate(detection, index=0, estimator='polar_profiling')

    assert lidar.forward_m == pytest.approx(2.5)
    assert lidar.lateral_m == pytest.approx(-0.3)
    assert lidar.distance_m == pytest.approx(2.52)
    assert polar.forward_m is None
    assert polar.lateral_m is None
    assert polar.distance_m is None


def test_estimator_without_values_is_a_no_value_miss() -> None:
    gts = [GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0)]
    detection = Detection(
        bbox_xyxy=(0, 0, 10, 10), label='r', score=0.9,
        lidar_distance_m=2.05, lidar_forward_m=2.05, lidar_lateral_m=0.0)
    events = [_event([detection])]

    # sensor_depth never produced a usable frame, so it has no event list.
    score = score_scene(
        {'lidar': events, 'sensor_depth': []}, gts, ('lidar', 'sensor_depth'))

    assert score.detector_missed == ()
    assert score.medians[0]['lidar'] == pytest.approx(2.05)
    assert score.medians[0]['sensor_depth'] is None
    assert score.outcomes[0]['sensor_depth'] == OUTCOME_NO_VALUE
    assert score.outcome_counts('sensor_depth')[OUTCOME_NO_VALUE] == 1
    assert score.outcome_counts('lidar')[OUTCOME_SCORED] == 1


def test_one_estimator_far_off_does_not_erase_the_instance_for_the_others() -> None:
    # The regression guard for per-estimator association: in a multi-robot scene
    # polar_profiling is 4 m off on both detections, so it earns a gate miss --
    # while lidar, which located them correctly, keeps its scored errors.
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),
        GtPoint(index=1, forward_m=5.0, lateral_m=0.0, distance_m=5.0),
    ]

    def detection(forward, distance, polar_forward, polar_distance):
        return Detection(
            bbox_xyxy=(0, 0, 10, 10), label='r', score=0.9,
            lidar_forward_m=forward, lidar_lateral_m=0.0, lidar_distance_m=distance,
            polar_profiling_forward_m=polar_forward, polar_profiling_lateral_m=0.0,
            polar_profiling_distance_m=polar_distance,
        )

    events = [_event([
        detection(2.0, 2.0, 9.5, 9.5),
        detection(5.0, 5.0, 12.5, 12.5),
    ])]
    usable = {'lidar': events, 'polar_profiling': events}

    score = score_scene(usable, gts, ('lidar', 'polar_profiling'))

    assert score.medians[0]['lidar'] == pytest.approx(2.0)
    assert score.medians[1]['lidar'] == pytest.approx(5.0)
    assert score.outcomes[0]['lidar'] == OUTCOME_SCORED
    assert score.medians[0]['polar_profiling'] is None
    assert score.outcomes[0]['polar_profiling'] == OUTCOME_GATE_MISS
    assert score.outcomes[1]['polar_profiling'] == OUTCOME_GATE_MISS
    # The instance was located by lidar, so this is polar's failure, not the
    # detector's -- nothing here is a detector miss.
    assert score.detector_missed == ()
    assert score.outcome_counts('polar_profiling')[OUTCOME_GATE_MISS] == 2
    assert score.outcome_counts('lidar')[OUTCOME_SCORED] == 2


def test_extra_count_is_a_detector_metric_not_an_estimator_one() -> None:
    # extra = boxes NO estimator could place. A badly-localizing estimator must
    # not inflate it, or the detector gets blamed for the estimator's error.
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),
        GtPoint(index=1, forward_m=5.0, lateral_m=0.0, distance_m=5.0),
    ]

    def detection(forward, distance, polar_forward, polar_distance):
        return Detection(
            bbox_xyxy=(0, 0, 10, 10), label='r', score=0.9,
            lidar_forward_m=forward, lidar_lateral_m=0.0, lidar_distance_m=distance,
            polar_profiling_forward_m=polar_forward, polar_profiling_lateral_m=0.0,
            polar_profiling_distance_m=polar_distance,
        )

    # Both boxes are real; polar is 4 m off on each, lidar places both.
    events = [_event([detection(2.0, 2.0, 9.5, 9.5), detection(5.0, 5.0, 12.5, 12.5)])]
    score = score_scene(
        {'lidar': events, 'polar_profiling': events}, gts, ('lidar', 'polar_profiling'))
    assert score.extra_count == 0

    # A third box no estimator can attribute IS a phantom.
    phantom = [_event([
        detection(2.0, 2.0, 2.0, 2.0),
        detection(5.0, 5.0, 5.0, 5.0),
        detection(30.0, 30.0, 30.0, 30.0),
    ])]
    score = score_scene(
        {'lidar': phantom, 'polar_profiling': phantom}, gts, ('lidar', 'polar_profiling'))
    assert score.extra_count == 1


def test_single_robot_scene_is_ungated_so_large_errors_stay_scored() -> None:
    # Truth 2.0 m, estimator reports 6.3 m. With one robot there is nothing to
    # confuse it with, so the 4.3 m error is graded rather than dropped.
    gts = [GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0)]
    events = [_event([_lidar_detection(6.3, 0.0, 6.3)])]

    score = score_scene(_lidar_only(events), gts, ('lidar',))

    assert score.medians[0]['lidar'] == pytest.approx(6.3)
    assert score.outcomes[0]['lidar'] == OUTCOME_SCORED
    assert score.detector_missed == ()


def test_nothing_detected_is_a_detector_miss_not_an_estimator_miss() -> None:
    gts = [GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0)]

    score = score_scene({'lidar': []}, gts, ('lidar',), detector_fired=False)

    assert score.outcomes[0]['lidar'] == OUTCOME_DETECTOR_MISS
    assert score.detector_missed == (0,)
    assert score.outcome_counts('lidar') == {
        OUTCOME_SCORED: 0,
        OUTCOME_DETECTOR_MISS: 1,
        OUTCOME_GATE_MISS: 0,
        OUTCOME_NO_VALUE: 0,
    }


def test_display_estimate_falls_back_across_selected_estimators() -> None:
    # Collage-only helper: the first selected estimator carrying a locator wins.
    detection = _lidar_detection(2.5, -0.3, 2.52)

    estimate = build_display_instance_estimate(
        detection, index=0, selected_estimators=('polar_profiling', 'lidar'))

    assert estimate.forward_m == pytest.approx(2.5)
    assert estimate.distance_m == pytest.approx(2.52)
