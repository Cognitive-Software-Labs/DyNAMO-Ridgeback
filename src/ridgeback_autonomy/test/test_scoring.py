from __future__ import annotations

from types import SimpleNamespace

import pytest

from ridgeback_autonomy.benchmarking.association import GtPoint
from ridgeback_autonomy.benchmarking.scoring import build_instance_estimate, score_scene
from ridgeback_autonomy.common.models import Detection


def _lidar_detection(forward, lateral, distance):
    return Detection(
        bbox_xyxy=(0, 0, 10, 10), label='humanoid robot', score=0.9,
        lidar_forward_m=forward, lidar_lateral_m=lateral, lidar_distance_m=distance,
    )


def _event(detections):
    return SimpleNamespace(detections=detections)


def test_single_robot_median_over_frames() -> None:
    gts = [GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0)]
    events = [
        _event([_lidar_detection(2.0, 0.0, 1.9)]),
        _event([_lidar_detection(2.0, 0.0, 2.0)]),
        _event([_lidar_detection(2.0, 0.0, 2.1)]),
    ]

    medians, missed, extra = score_scene(events, gts, ('lidar',))

    assert medians[0]['lidar'] == pytest.approx(2.0)
    assert missed == ()
    assert extra == 0


def test_two_robots_scored_independently() -> None:
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),
        GtPoint(index=1, forward_m=3.0, lateral_m=1.5, distance_m=3.35),
    ]
    events = [
        _event([_lidar_detection(2.0, 0.0, 2.02), _lidar_detection(3.0, 1.5, 3.30)]),
        _event([_lidar_detection(2.0, 0.0, 1.98), _lidar_detection(3.0, 1.5, 3.40)]),
    ]

    medians, missed, extra = score_scene(events, gts, ('lidar',))

    assert medians[0]['lidar'] == pytest.approx(2.0)
    assert medians[1]['lidar'] == pytest.approx(3.35)
    assert missed == ()
    assert extra == 0


def test_occluded_far_robot_missed_not_scored() -> None:
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),   # near
        GtPoint(index=1, forward_m=4.0, lateral_m=0.0, distance_m=4.0),   # far, occluded
    ]
    events = [_event([_lidar_detection(2.0, 0.0, 2.0)]) for _ in range(3)]

    medians, missed, extra = score_scene(events, gts, ('lidar',))

    assert medians[0]['lidar'] == pytest.approx(2.0)
    assert medians[1]['lidar'] is None
    assert missed == (1,)


def test_distance_only_locator_used_when_no_planar_estimator() -> None:
    gts = [
        GtPoint(index=0, forward_m=2.0, lateral_m=0.0, distance_m=2.0),
        GtPoint(index=1, forward_m=4.0, lateral_m=0.0, distance_m=4.0),
    ]
    near = Detection(bbox_xyxy=(0, 0, 10, 10), label='r', score=0.9, sensor_depth_distance_m=2.05)
    events = [_event([near])]

    medians, missed, extra = score_scene(events, gts, ('sensor_depth',))

    assert medians[0]['sensor_depth'] == pytest.approx(2.05)
    assert medians[1]['sensor_depth'] is None
    assert missed == (1,)


def test_build_instance_estimate_prefers_planar_locator() -> None:
    detection = _lidar_detection(2.5, -0.3, 2.52)

    estimate = build_instance_estimate(detection, index=0, selected_estimators=('lidar',))

    assert estimate.forward_m == pytest.approx(2.5)
    assert estimate.lateral_m == pytest.approx(-0.3)
    assert estimate.distance_m == pytest.approx(2.52)
