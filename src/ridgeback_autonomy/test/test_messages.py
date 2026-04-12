from __future__ import annotations

import math

import pytest
from std_msgs.msg import Header

from ridgeback_autonomy.common.messages import (
    batch_from_detections_message,
    build_detections_message,
    build_measurements_message,
    snapshot_measurements_message,
)
from ridgeback_autonomy.common.models import Detection, DetectionBatch


def test_build_detections_message_serializes_identity_fields() -> None:
    batch = DetectionBatch(
        image_width=640,
        image_height=480,
        detections=[
            Detection(
                bbox_xyxy=(1, 2, 30, 40),
                label='humanoid robot',
                score=0.9,
            ),
        ],
    )

    msg = build_detections_message(batch, Header())

    assert msg.count == 1
    assert list(msg.bbox_xyxy) == [1.0, 2.0, 30.0, 40.0]
    assert list(msg.labels) == ['humanoid robot']
    assert list(msg.scores) == pytest.approx([0.9], rel=1e-6)
    assert msg.image_width == 640
    assert msg.image_height == 480


def test_batch_from_detections_message_round_trips_detection_identity() -> None:
    batch = DetectionBatch(
        image_width=320,
        image_height=240,
        detections=[
            Detection(
                bbox_xyxy=(5, 6, 20, 30),
                label='humanoid robot',
                score=0.75,
            ),
        ],
    )

    decoded = batch_from_detections_message(build_detections_message(batch, Header()))

    assert decoded.count == 1
    assert decoded.detections[0].bbox_xyxy == (5, 6, 20, 30)
    assert decoded.detections[0].label == 'humanoid robot'
    assert decoded.detections[0].score == pytest.approx(0.75, rel=1e-6)


def test_build_measurements_message_serializes_parallel_arrays() -> None:
    batch = DetectionBatch(
        image_width=640,
        image_height=480,
        detections=[
            Detection(
                bbox_xyxy=(1, 2, 30, 40),
                label='humanoid robot',
                score=0.9,
                rgb_lateral_m=0.1,
                rgb_forward_m=2.0,
                rgb_distance_m=2.002,
                sensor_depth_distance_m=1.95,
                mono_depth_distance_m=None,
                pointcloud_lateral_m=0.0,
                pointcloud_forward_m=1.8,
                pointcloud_distance_m=1.8,
            ),
            Detection(
                bbox_xyxy=(50, 60, 100, 140),
                label='humanoid robot',
                score=0.8,
                lidar_lateral_m=-0.2,
                lidar_forward_m=3.0,
                lidar_distance_m=3.006,
            ),
        ],
    )

    msg = build_measurements_message(batch, Header())

    assert msg.count == 2
    assert list(msg.bbox_xyxy) == [1.0, 2.0, 30.0, 40.0, 50.0, 60.0, 100.0, 140.0]
    assert math.isclose(msg.rgb_distance_m[0], 2.002, rel_tol=1e-6)
    assert math.isnan(msg.rgb_distance_m[1])
    assert math.isnan(msg.mono_depth_distance_m[0])
    assert math.isclose(msg.pointcloud_distance_m[0], 1.8, rel_tol=1e-6)
    assert math.isclose(msg.lidar_distance_m[1], 3.006, rel_tol=1e-6)


def test_snapshot_measurements_message_uses_first_finite_positive_values() -> None:
    batch = DetectionBatch(
        image_width=320,
        image_height=240,
        detections=[
            Detection(
                bbox_xyxy=(5, 6, 20, 30),
                label='humanoid robot',
                score=0.75,
                rgb_distance_m=2.5,
                sensor_depth_distance_m=2.2,
                mono_depth_distance_m=None,
                lidar_distance_m=2.1,
                pointcloud_distance_m=None,
            ),
        ],
    )

    msg = build_measurements_message(batch, Header())
    snapshot = snapshot_measurements_message(msg)

    assert snapshot['detected'] is True
    assert snapshot['count'] == 1
    assert snapshot['bboxes'] == [[5, 6, 20, 30]]
    assert snapshot['rgb_distance_m'] == pytest.approx(2.5, rel=1e-6)
    assert snapshot['sensor_depth_distance_m'] == pytest.approx(2.2, rel=1e-6)
    assert snapshot['mono_depth_distance_m'] is None
    assert snapshot['lidar_distance_m'] == pytest.approx(2.1, rel=1e-6)
    assert snapshot['pointcloud_distance_m'] is None
