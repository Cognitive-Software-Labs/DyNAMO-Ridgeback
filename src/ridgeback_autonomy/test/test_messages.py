from __future__ import annotations

import math

import numpy as np
import pytest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from ridgeback_autonomy.common.markers import PolarBeamRecord
from ridgeback_autonomy.common.messages import (
    batch_from_detections_message,
    batch_from_measurements_message,
    build_detections_message,
    build_measurements_message,
    build_polar_beams_message,
    decode_optional_status,
    optional_status,
    polar_beam_booleans,
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


def scan_message(beam_count: int, sec: int = 7, nanosec: int = 500) -> LaserScan:
    scan = LaserScan()
    scan.header.frame_id = 'lidar2d_0_laser'
    scan.header.stamp.sec = sec
    scan.header.stamp.nanosec = nanosec
    scan.ranges = [1.0] * beam_count
    return scan


def beam_record(index: int, selected, merged) -> PolarBeamRecord:
    return PolarBeamRecord(
        detection_index=index,
        selected=np.array(selected, dtype=np.intp),
        merged=np.array(merged, dtype=np.intp),
        in_bbox=np.array(selected, dtype=np.intp),
    )


def test_polar_beams_message_unions_across_detections() -> None:
    # Two robots at different ranges each keep their own band. The single-band
    # union the overlay used to compute would have dropped the far one entirely.
    records = [
        beam_record(0, [1, 2, 3], [2, 3]),
        beam_record(1, [3, 8, 9], [8, 9]),
    ]
    header = Header()
    header.stamp.sec = 42
    header.frame_id = 'camera_0_color_optical_frame'

    msg = build_polar_beams_message(records, scan_message(12), header)

    assert list(msg.selected) == [1, 2, 3, 8, 9]  # sorted, de-duplicated
    assert list(msg.merged) == [2, 3, 8, 9]
    assert msg.beam_count == 12
    # Two stamps: the frame this explains, and the scan the indices index into.
    assert msg.header.stamp.sec == 42
    assert (msg.scan_stamp.sec, msg.scan_stamp.nanosec) == (7, 500)
    assert msg.scan_frame_id == 'lidar2d_0_laser'


def test_polar_beams_message_with_no_records_is_empty_not_absent() -> None:
    msg = build_polar_beams_message([], scan_message(12), Header())

    assert list(msg.selected) == []
    assert list(msg.merged) == []
    assert msg.beam_count == 12


def test_polar_beam_booleans_round_trip_marks_used_and_dropped() -> None:
    records = [beam_record(0, [1, 2, 3], [2, 3])]

    msg = build_polar_beams_message(records, scan_message(6), Header())
    used, dropped = polar_beam_booleans(msg, 6)

    assert list(np.flatnonzero(used)) == [2, 3]
    # Selected but discarded by the range band -- the distinction the panel
    # exists to draw, and the one a used-only highlight cannot express.
    assert list(np.flatnonzero(dropped)) == [1]


def test_polar_beam_booleans_refuse_a_mismatched_beam_count() -> None:
    # Indexing a differently sized scan would shift the highlight onto beams the
    # estimator never touched. A missing highlight is the required failure mode.
    msg = build_polar_beams_message(
        [beam_record(0, [1, 2], [2])], scan_message(6), Header())

    assert polar_beam_booleans(msg, 7) is None
    assert polar_beam_booleans(msg, 6) is not None
