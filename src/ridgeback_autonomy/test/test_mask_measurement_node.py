from __future__ import annotations

import math

import numpy as np
import pytest
from sensor_msgs.msg import Image
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
from ridgeback_autonomy.perception.core.mask import MaskPrecision, mask_from_array
from ridgeback_autonomy.perception.g1_mask_measurement_node import (
    CAMERA_PITCH_DEG_DEFAULT,
    MASK_GATE_BOX,
    MASK_GATE_SILHOUETTE,
    ROBOT_FRONT_OFFSET_M_DEFAULT,
    ColorFrameBuffer,
    encode_mask_debug_image,
    fill_path_measurements,
    grid_mismatch_warning,
    optical_to_vehicle_planar,
    resolve_mask_gate,
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


def test_resolve_mask_gate_accepts_both_gates() -> None:
    assert resolve_mask_gate('box') == MASK_GATE_BOX
    assert resolve_mask_gate(' silhouette ') == MASK_GATE_SILHOUETTE


def test_resolve_mask_gate_rejects_unknown_value() -> None:
    with pytest.raises(ValueError, match='box, silhouette'):
        resolve_mask_gate('tight')


def color_image(sec: int, nanosec: int) -> Image:
    msg = Image()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    return msg


def stamp(sec: int, nanosec: int):
    header = Header()
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    return header.stamp


def test_color_frame_buffer_exact_stamp_hit() -> None:
    buffer = ColorFrameBuffer(depth=3)
    target = color_image(10, 500)
    buffer.store(color_image(10, 400))
    buffer.store(target)
    buffer.store(color_image(10, 600))

    assert buffer.lookup(stamp(10, 500)) is target


def test_color_frame_buffer_miss_returns_none() -> None:
    buffer = ColorFrameBuffer(depth=3)
    buffer.store(color_image(10, 400))

    # Nearby but not exact: the lookup is exact-stamp, no tolerance.
    assert buffer.lookup(stamp(10, 401)) is None


def test_color_frame_buffer_evicts_oldest_beyond_depth() -> None:
    buffer = ColorFrameBuffer(depth=2)
    buffer.store(color_image(1, 0))
    buffer.store(color_image(2, 0))
    buffer.store(color_image(3, 0))

    assert len(buffer) == 2
    assert buffer.lookup(stamp(1, 0)) is None  # aged out
    assert buffer.lookup(stamp(3, 0)) is not None


# --- fill_path_measurements with tight masks: the silhouette-gate consumption
# path, exercised end-to-end with a fake producer's blobs. ---

FILL_HEIGHT, FILL_WIDTH = 60, 80
FILL_INTRINSICS = CameraIntrinsics(
    fx=100.0, fy=100.0, cx=40.0, cy=30.0, width=FILL_WIDTH, height=FILL_HEIGHT)
OBJECT_SLICE = (slice(20, 40), slice(30, 50))
OBJECT_DEPTH_M = 2.0


def build_fill_batch(count: int = 1) -> DetectionBatch:
    return DetectionBatch(
        image_width=FILL_WIDTH,
        image_height=FILL_HEIGHT,
        detections=[
            Detection(bbox_xyxy=(30, 20, 50, 40), label='humanoid robot', score=0.9)
            for _ in range(count)
        ],
    )


def build_fill_depth() -> np.ndarray:
    depth = np.full((FILL_HEIGHT, FILL_WIDTH), 4.0, dtype=np.float32)
    depth[OBJECT_SLICE] = OBJECT_DEPTH_M
    return depth


def tight_blob() -> np.ndarray:
    blob = np.zeros((FILL_HEIGHT, FILL_WIDTH), dtype=bool)
    blob[OBJECT_SLICE] = True
    return blob


def forbidden_isolation(*args, **kwargs):
    raise AssertionError('isolation recipe must not run on the tight branch')


def test_fill_with_tight_masks_runs_paths_without_isolation_recipes() -> None:
    batch = build_fill_batch()
    masks = [mask_from_array(tight_blob(), MaskPrecision.TIGHT)]

    fill_path_measurements(
        batch,
        masks,
        FILL_INTRINSICS,
        build_fill_depth(),
        None,  # no scan: polar profiling simply skipped
        pitch_rad=0.0,
        front_offset_m=0.25,
        isolation_2d=forbidden_isolation,
        isolation_3d=forbidden_isolation,
    )

    detection = batch.detections[0]
    # Object plate at 2.0 m, zero pitch: forward = Z - front offset.
    assert detection.projective_ranging_forward_m == pytest.approx(
        OBJECT_DEPTH_M - 0.25, abs=1e-6)
    assert detection.euclidean_reconstruction_forward_m == pytest.approx(
        OBJECT_DEPTH_M - 0.25, abs=0.01)
    assert detection.polar_profiling_distance_m is None


def test_fill_skips_none_mask_entries_fields_stay_unset() -> None:
    batch = build_fill_batch(count=2)
    masks = [None, mask_from_array(tight_blob(), MaskPrecision.TIGHT)]

    fill_path_measurements(
        batch,
        masks,
        FILL_INTRINSICS,
        build_fill_depth(),
        None,
        pitch_rad=0.0,
        front_offset_m=0.25,
        isolation_2d=forbidden_isolation,
        isolation_3d=forbidden_isolation,
    )

    # The empty-segmentation detection drops; the other still fills.
    assert batch.detections[0].projective_ranging_distance_m is None
    assert batch.detections[0].euclidean_reconstruction_distance_m is None
    assert batch.detections[1].projective_ranging_distance_m is not None


def test_encode_mask_debug_image_unions_masks_and_skips_none() -> None:
    mask_a = np.zeros((4, 6), dtype=bool)
    mask_a[1, 2] = True
    mask_b = np.zeros((4, 6), dtype=bool)
    mask_b[3, 5] = True
    header = Header()
    header.frame_id = 'camera_0_color_optical'
    header.stamp.sec = 7

    msg = encode_mask_debug_image(
        [mask_from_array(mask_a, MaskPrecision.TIGHT),
         None,
         mask_from_array(mask_b, MaskPrecision.TIGHT)],
        4, 6, header)

    assert msg.encoding == 'mono8'
    assert (msg.height, msg.width, msg.step) == (4, 6, 6)
    assert msg.header.frame_id == 'camera_0_color_optical'
    decoded = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(4, 6)
    assert decoded[1, 2] == 255 and decoded[3, 5] == 255
    assert int(np.count_nonzero(decoded)) == 2
