from __future__ import annotations

import math

import pytest
from std_msgs.msg import Header

from ridgeback_autonomy.benchmarking.alignment import measurement_message_key
from ridgeback_autonomy.common.messages import (
    batch_from_detections_message,
    batch_from_measurements_message,
    build_detections_message,
    build_measurements_message,
)
from ridgeback_autonomy.common.models import Detection, DetectionBatch
from ridgeback_autonomy.perception.core.intrinsics import CameraIntrinsics
from ridgeback_autonomy.perception.g1_mask_measurement_node import (
    CAMERA_PITCH_DEG_DEFAULT,
    ROBOT_FRONT_OFFSET_M_DEFAULT,
    grid_mismatch_warning,
    optical_to_vehicle_planar,
)


def test_vehicle_adapter_zero_pitch_forward_is_z_minus_offset() -> None:
    lateral, forward, distance = optical_to_vehicle_planar(
        (0.5, -0.2, 3.0), pitch_rad=0.0, front_offset_m=0.25)

    assert lateral == pytest.approx(0.5)
    assert forward == pytest.approx(2.75)
    assert distance == pytest.approx(math.hypot(0.5, 2.75))


def test_vehicle_adapter_pitch_folds_optical_y_into_forward() -> None:
    pitch_rad = math.radians(10.0)
    # A point straight ahead of a downward-pitched camera: optical Y (down)
    # contributes against forward, optical Z contributes with cos(pitch).
    lateral, forward, _ = optical_to_vehicle_planar(
        (0.0, 0.3, 2.0), pitch_rad=pitch_rad, front_offset_m=0.0)

    assert lateral == 0.0
    assert forward == pytest.approx(
        -math.sin(pitch_rad) * 0.3 + math.cos(pitch_rad) * 2.0)


def test_defaults_mirror_legacy_constants_by_value() -> None:
    assert ROBOT_FRONT_OFFSET_M_DEFAULT == 0.25
    assert CAMERA_PITCH_DEG_DEFAULT == 0.0


def test_grid_mismatch_warning_none_when_grids_match() -> None:
    intrinsics = CameraIntrinsics(
        fx=443.5, fy=443.5, cx=319.5, cy=239.5, width=640, height=480)
    batch = DetectionBatch(image_width=640, image_height=480, detections=[])

    assert grid_mismatch_warning(intrinsics, batch) is None


def test_grid_mismatch_warning_names_both_grids() -> None:
    # The Isaac Sim grid against the Gazebo detection grid: the realistic
    # mismatch a mis-wired camera_info topic would produce.
    intrinsics = CameraIntrinsics(
        fx=443.5, fy=443.5, cx=639.5, cy=359.5, width=1280, height=720)
    batch = DetectionBatch(image_width=640, image_height=480, detections=[])

    warning = grid_mismatch_warning(intrinsics, batch)

    assert warning is not None
    assert '(720, 1280)' in warning
    assert '(480, 640)' in warning


def test_measurements_from_detections_batch_preserve_alignment_key() -> None:
    header = Header()
    header.frame_id = 'camera_0_color_optical'
    header.stamp.sec = 7
    header.stamp.nanosec = 42
    batch = DetectionBatch(
        image_width=640,
        image_height=480,
        detections=[
            Detection(bbox_xyxy=(10, 20, 40, 60), label='humanoid robot', score=0.9),
        ],
    )
    detections_msg = build_detections_message(batch, header)

    # What the mask node does: rebuild the batch from the detections message,
    # fill path fields, publish measurements with the same header.
    rebuilt = batch_from_detections_message(detections_msg)
    rebuilt.detections[0].projective_ranging_distance_m = 2.5
    rebuilt.detections[0].euclidean_reconstruction_distance_m = 2.6
    measurements_msg = build_measurements_message(rebuilt, detections_msg.header)

    assert measurement_message_key(measurements_msg) == (
        'camera_0_color_optical', 7, 42, 1, (10.0, 20.0, 40.0, 60.0))


def test_missing_path_results_encode_as_nan_and_decode_as_none() -> None:
    batch = DetectionBatch(
        image_width=640,
        image_height=480,
        detections=[
            Detection(
                bbox_xyxy=(1, 2, 30, 40),
                label='humanoid robot',
                score=0.9,
                projective_ranging_lateral_m=0.1,
                projective_ranging_forward_m=2.0,
                projective_ranging_distance_m=2.002,
                # euclidean_reconstruction left unset: the sparse-mask skip case.
            ),
        ],
    )

    msg = build_measurements_message(batch, Header())
    decoded = batch_from_measurements_message(msg)

    assert msg.projective_ranging_distance_m[0] == pytest.approx(2.002, rel=1e-6)
    assert math.isnan(msg.euclidean_reconstruction_distance_m[0])
    assert decoded.detections[0].projective_ranging_distance_m == pytest.approx(2.002, rel=1e-6)
    assert decoded.detections[0].euclidean_reconstruction_distance_m is None
