from __future__ import annotations

import math

import pytest
from std_msgs.msg import Header

from ridgeback_autonomy.common.messages import (
    batch_from_detections_message,
    batch_from_measurements_message,
    build_detections_message,
    build_measurements_message,
    decode_optional_status,
    optional_status,
    snapshot_measurements_message,
)
from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.common.models import Detection, DetectionBatch
from ridgeback_autonomy.common.stamps import stamp_key, stamp_to_nanoseconds


@pytest.mark.parametrize('sec,nanosec,expected', [
    (0, 0, 0),
    (1, 1, 1_000_000_001),
    (-1, 999_999_999, -1),
    (2_000_000_000, 123_456_789, 2_000_000_000_123_456_789),
])
def test_stamp_conversion_preserves_nanosecond_precision(sec, nanosec, expected):
    from builtin_interfaces.msg import Time

    stamp = Time(sec=sec, nanosec=nanosec)
    assert stamp_key(stamp) == (sec, nanosec)
    assert stamp_to_nanoseconds(stamp) == expected


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
                pointcloud_lateral_m=0.0,
                pointcloud_forward_m=1.8,
                pointcloud_distance_m=1.8,
            ),
            Detection(
                bbox_xyxy=(50, 60, 100, 140),
                label='humanoid robot',
                score=0.8,
                polar_profiling_lateral_m=-0.2,
                polar_profiling_forward_m=3.0,
                polar_profiling_distance_m=3.006,
                projective_ranging_lateral_m=-0.1,
                projective_ranging_forward_m=3.1,
                projective_ranging_distance_m=3.102,
                euclidean_reconstruction_lateral_m=-0.15,
                euclidean_reconstruction_forward_m=3.05,
                euclidean_reconstruction_distance_m=3.054,
            ),
        ],
    )

    msg = build_measurements_message(batch, Header())

    assert msg.count == 2
    assert list(msg.bbox_xyxy) == [1.0, 2.0, 30.0, 40.0, 50.0, 60.0, 100.0, 140.0]
    assert math.isclose(msg.pointcloud_distance_m[0], 1.8, rel_tol=1e-6)
    assert math.isnan(msg.pointcloud_distance_m[1])
    assert math.isnan(msg.polar_profiling_distance_m[0])
    assert math.isclose(msg.polar_profiling_distance_m[1], 3.006, rel_tol=1e-6)
    assert math.isnan(msg.projective_ranging_distance_m[0])
    assert math.isclose(msg.projective_ranging_distance_m[1], 3.102, rel_tol=1e-6)
    assert math.isnan(msg.euclidean_reconstruction_distance_m[0])
    assert math.isclose(msg.euclidean_reconstruction_distance_m[1], 3.054, rel_tol=1e-6)


def test_snapshot_measurements_message_uses_first_finite_positive_values() -> None:
    batch = DetectionBatch(
        image_width=320,
        image_height=240,
        detections=[
            Detection(
                bbox_xyxy=(5, 6, 20, 30),
                label='humanoid robot',
                score=0.75,
                pointcloud_distance_m=None,
                polar_profiling_distance_m=2.1,
                projective_ranging_distance_m=2.3,
                euclidean_reconstruction_distance_m=None,
            ),
        ],
    )

    msg = build_measurements_message(batch, Header())
    snapshot = snapshot_measurements_message(msg)

    assert snapshot['detected'] is True
    assert snapshot['count'] == 1
    assert snapshot['bboxes'] == [[5, 6, 20, 30]]
    assert snapshot['pointcloud_distance_m'] is None
    assert snapshot['polar_profiling_distance_m'] == pytest.approx(2.1, rel=1e-6)
    assert snapshot['projective_ranging_distance_m'] == pytest.approx(2.3, rel=1e-6)
    assert snapshot['euclidean_reconstruction_distance_m'] is None


def test_optional_status_encode_decode() -> None:
    assert optional_status(None) == int(MissReason.UNSET)
    assert optional_status(int(MissReason.SCAN_INVALID)) == int(MissReason.SCAN_INVALID)
    assert decode_optional_status([int(MissReason.OK)], 0) == int(MissReason.OK)
    assert decode_optional_status([int(MissReason.UNSET)], 0) is None  # sentinel -> None
    assert decode_optional_status([], 0) is None  # absent -> None


def test_measurements_message_round_trips_estimator_statuses() -> None:
    batch = DetectionBatch(
        image_width=640,
        image_height=480,
        detections=[
            Detection(
                bbox_xyxy=(1, 2, 30, 40), label='humanoid robot', score=0.9,
                projective_ranging_status=int(MissReason.OK),
                euclidean_reconstruction_status=int(MissReason.TOO_FEW_AFTER_ISOLATION),
                polar_profiling_status=int(MissReason.SCAN_INVALID),
            ),
        ],
    )

    msg = build_measurements_message(batch, Header())
    assert list(msg.projective_ranging_status) == [int(MissReason.OK)]

    decoded = batch_from_measurements_message(msg).detections[0]
    assert decoded.projective_ranging_status == int(MissReason.OK)
    assert decoded.euclidean_reconstruction_status == int(
        MissReason.TOO_FEW_AFTER_ISOLATION)
    assert decoded.polar_profiling_status == int(MissReason.SCAN_INVALID)


def test_unset_status_decodes_to_none() -> None:
    batch = DetectionBatch(
        image_width=640,
        image_height=480,
        detections=[Detection(bbox_xyxy=(1, 2, 30, 40), label='humanoid robot', score=0.9)],
    )

    decoded = batch_from_measurements_message(build_measurements_message(batch, Header()))

    assert decoded.detections[0].projective_ranging_status is None
