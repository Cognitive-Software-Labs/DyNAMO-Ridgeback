from __future__ import annotations

import math

import numpy as np
import pytest
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from ridgeback_autonomy.benchmarking.alignment import measurement_message_key
from ridgeback_autonomy.common.markers import PolarBeamRecord
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
    BASE_FRAME_DEFAULT,
    MASK_GATE_BOX,
    MASK_GATE_SILHOUETTE,
    MAX_BOX_FRAME_FRACTION,
    ROBOT_FRONT_OFFSET_M_DEFAULT,
    StampedMessageBuffer,
    box_within_frame_fraction,
    encode_mask_debug_image,
    fill_path_measurements,
    grid_mismatch_warning,
    nearest_beam_record,
    optical_to_base_planar,
    resolve_enabled_estimators,
    resolve_mask_gate,
)


# The optical -> base rotation of a level (zero-pitch, zero-roll) camera:
# base X (forward) = optical Z, base Y (left) = -optical X, base Z (up) =
# -optical Y. What TF publishes for the benchmark camera.
LEVEL_OPTICAL_TO_BASE = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])
ZERO_TRANSLATION = np.zeros(3)


def test_base_adapter_level_camera_forward_is_z_minus_offset() -> None:
    lateral, forward, distance = optical_to_base_planar(
        (0.5, -0.2, 3.0), LEVEL_OPTICAL_TO_BASE, ZERO_TRANSLATION,
        front_offset_m=0.25)

    # Optical X right = base -Y: lateral is left-positive (REP-103), matching
    # the ground truth and the lidar/pointcloud rows.
    assert lateral == pytest.approx(-0.5)
    assert forward == pytest.approx(2.75)
    assert distance == pytest.approx(math.hypot(0.5, 2.75))


def test_base_adapter_applies_camera_mounting_translation() -> None:
    # Camera mounted forward/left/up of the base origin: the translation must
    # enter the planar coordinate -- this is the fix under test.
    translation = np.array([0.311, 0.018, 1.158])
    lateral, forward, _ = optical_to_base_planar(
        (0.0, 0.0, 3.0), LEVEL_OPTICAL_TO_BASE, translation,
        front_offset_m=0.25)

    assert forward == pytest.approx(3.0 + 0.311 - 0.25)
    assert lateral == pytest.approx(0.018)


def test_defaults_mirror_legacy_constants_by_value() -> None:
    # The node's front-offset default mirrors the legacy geometry constant by
    # value (the two stacks never import each other). The pure core-module
    # mirrors are covered in test_mirrored_constants; this one needs the
    # ROS-importing node module, so it lives here.
    from ridgeback_autonomy.perception.core import geometry

    assert ROBOT_FRONT_OFFSET_M_DEFAULT == geometry.ROBOT_FRONT_OFFSET_M
    assert ROBOT_FRONT_OFFSET_M_DEFAULT == 0.25
    assert BASE_FRAME_DEFAULT == 'base_link'


def test_box_within_frame_fraction_accepts_normal_box() -> None:
    # A person-sized box on a 640x480 frame: well under the gate.
    assert box_within_frame_fraction((260, 120, 380, 400), 480, 640) is True


def test_box_within_frame_fraction_rejects_near_full_frame_box() -> None:
    # The OWLv2 failure mode: a box spanning almost the whole frame.
    assert box_within_frame_fraction((2, 2, 638, 478), 480, 640) is False


def test_box_within_frame_fraction_boundary_at_max_fraction() -> None:
    # Exactly MAX_BOX_FRAME_FRACTION of the area is still accepted (<=).
    width, height = 640, 480
    box_w = int(round(width * MAX_BOX_FRAME_FRACTION))
    assert box_within_frame_fraction((0, 0, box_w, height), height, width) is True
    assert box_within_frame_fraction((0, 0, box_w + 2, height), height, width) is False


def test_box_within_frame_fraction_degenerate_frame_is_rejected() -> None:
    assert box_within_frame_fraction((0, 0, 10, 10), 0, 0) is False


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


def test_stamped_buffer_exact_stamp_hit() -> None:
    buffer = StampedMessageBuffer(depth=3)
    target = color_image(10, 500)
    buffer.store(color_image(10, 400))
    buffer.store(target)
    buffer.store(color_image(10, 600))

    assert buffer.lookup(stamp(10, 500)) is target


def test_stamped_buffer_miss_returns_none() -> None:
    buffer = StampedMessageBuffer(depth=3)
    buffer.store(color_image(10, 400))

    # Nearby but not exact: the lookup is exact-stamp, no tolerance.
    assert buffer.lookup(stamp(10, 401)) is None


def test_stamped_buffer_evicts_oldest_beyond_depth() -> None:
    buffer = StampedMessageBuffer(depth=2)
    buffer.store(color_image(1, 0))
    buffer.store(color_image(2, 0))
    buffer.store(color_image(3, 0))

    assert len(buffer) == 2
    assert buffer.lookup(stamp(1, 0)) is None  # aged out
    assert buffer.lookup(stamp(3, 0)) is not None


def test_stamped_buffer_nearest_picks_closest_within_tolerance() -> None:
    # The scan match: free-running stamps, pick the nearest inside the window.
    buffer = StampedMessageBuffer(depth=5)
    buffer.store(color_image(10, 0))            # 40 ms before target
    closest = color_image(10, 30_000_000)       # 10 ms before target
    buffer.store(closest)
    buffer.store(color_image(10, 90_000_000))   # 50 ms after target

    hit = buffer.lookup_nearest(stamp(10, 40_000_000), tolerance_s=0.05)

    assert hit is closest


def test_stamped_buffer_nearest_miss_outside_tolerance_returns_none() -> None:
    buffer = StampedMessageBuffer(depth=5)
    buffer.store(color_image(10, 0))            # 100 ms away

    assert buffer.lookup_nearest(stamp(10, 100_000_000), tolerance_s=0.05) is None


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
        camera_rotation=LEVEL_OPTICAL_TO_BASE,
        camera_translation=ZERO_TRANSLATION,
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


def build_fill_scan(near_ratios, near_z: float = 2.0, wall_z: float = 5.0, count: int = 41):
    """Scan points across the box and past it; ``near_ratios`` marks the object.

    u = 100*(X/Z) + 40, so a ratio of X/Z maps to a column directly and the box
    (columns 30..50) spans ratios -0.10..0.10. Y = 0 puts every beam on row 30,
    inside the box's rows, so the vertical test never masks the horizontal one.
    """

    ratios = np.linspace(-0.2, 0.2, count)
    z = np.where(near_ratios(ratios), near_z, wall_z)
    points = np.stack((ratios * z, np.zeros(count), z), axis=1)
    return points, np.ones(count, dtype=bool)


def test_fill_records_the_beams_polar_reduced() -> None:
    batch = build_fill_batch()
    masks = [mask_from_array(tight_blob(), MaskPrecision.TIGHT)]
    scan_points = build_fill_scan(lambda ratios: np.abs(ratios) <= 0.05)
    records: list = []

    fill_path_measurements(
        batch, masks, FILL_INTRINSICS, build_fill_depth(), scan_points,
        camera_rotation=LEVEL_OPTICAL_TO_BASE,
        camera_translation=ZERO_TRANSLATION,
        front_offset_m=0.25,
        isolation_2d=forbidden_isolation,
        isolation_3d=forbidden_isolation,
        beam_records=records,
    )

    assert len(records) == 1
    record = records[0]
    assert record.detection_index == 0
    # The near plate merged; the wall beams that shared the box did not.
    points = scan_points[0]
    assert np.allclose(points[record.merged][:, 2], 2.0)
    dropped = np.setdiff1d(record.selected, record.merged)
    assert dropped.size > 0
    assert np.allclose(points[dropped][:, 2], 5.0)
    # Tight mask equals the box here, so the wedge spans the same beams.
    assert np.array_equal(record.in_bbox, record.selected)


def test_fill_records_beams_even_when_polar_produces_no_estimate() -> None:
    # A lone near return with the rest of the box on a far wall: the near-band
    # merge keeps one beam, below the floor, so polar declines. The beams are
    # still what a viewer needs -- that is how "declined" is told apart from
    # "latched onto the near thing".
    batch = build_fill_batch()
    masks = [mask_from_array(tight_blob(), MaskPrecision.TIGHT)]
    scan_points = build_fill_scan(
        lambda ratios: np.isclose(ratios, 0.0, atol=1e-9), near_z=1.0)
    records: list = []

    fill_path_measurements(
        batch, masks, FILL_INTRINSICS, build_fill_depth(), scan_points,
        camera_rotation=LEVEL_OPTICAL_TO_BASE,
        camera_translation=ZERO_TRANSLATION,
        front_offset_m=0.25,
        isolation_2d=forbidden_isolation,
        isolation_3d=forbidden_isolation,
        beam_records=records,
    )

    assert batch.detections[0].polar_profiling_distance_m is None
    assert len(records) == 1
    assert records[0].merged.size == 0
    assert records[0].selected.size > 0


def beam_record(detection_index: int) -> PolarBeamRecord:
    return PolarBeamRecord(
        detection_index=detection_index,
        selected=np.array([1, 2]),
        merged=np.array([1]),
        in_bbox=np.array([1, 2]),
    )


def test_the_drawn_record_is_the_nearest_detections() -> None:
    batch = build_fill_batch(count=2)
    batch.detections[0].polar_profiling_distance_m = 4.0
    batch.detections[1].polar_profiling_distance_m = 2.0

    record = nearest_beam_record(batch, [beam_record(0), beam_record(1)])

    assert record.detection_index == 1


def test_records_are_matched_by_detection_index_not_position() -> None:
    # Detection 0's segmentation came back empty, so it recorded no beams and the
    # list is one short. Indexing it positionally would draw the far robot.
    batch = build_fill_batch(count=3)
    batch.detections[1].polar_profiling_distance_m = 5.0
    batch.detections[2].polar_profiling_distance_m = 3.0

    record = nearest_beam_record(batch, [beam_record(1), beam_record(2)])

    assert record.detection_index == 2


def test_a_batch_with_no_estimates_still_draws_its_beams() -> None:
    # Every path declined, so nothing is rankable -- but a frame with beams and
    # no estimate is exactly the failure the ray layers exist to show.
    batch = build_fill_batch(count=2)

    record = nearest_beam_record(batch, [beam_record(0), beam_record(1)])

    assert record.detection_index == 0
    assert nearest_beam_record(batch, []) is None


def test_fill_skips_none_mask_entries_fields_stay_unset() -> None:
    batch = build_fill_batch(count=2)
    masks = [None, mask_from_array(tight_blob(), MaskPrecision.TIGHT)]

    fill_path_measurements(
        batch,
        masks,
        FILL_INTRINSICS,
        build_fill_depth(),
        None,
        camera_rotation=LEVEL_OPTICAL_TO_BASE,
        camera_translation=ZERO_TRANSLATION,
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


def _status_fixture():
    from ridgeback_autonomy.common.models import Detection, DetectionBatch
    from ridgeback_autonomy.perception.core.intrinsics import CameraIntrinsics
    from ridgeback_autonomy.perception.core.mask import rasterize_bbox

    intrinsics = CameraIntrinsics(fx=100.0, fy=100.0, cx=40.0, cy=30.0, width=80, height=60)
    depth = np.full((60, 80), 4.0, dtype=np.float32)
    depth[20:40, 30:50] = 2.0
    batch = DetectionBatch(
        image_width=80, image_height=60,
        detections=[Detection(bbox_xyxy=(25, 15, 55, 45), label='r', score=0.9)])
    masks = [rasterize_bbox((25, 15, 55, 45), 60, 80)]
    return intrinsics, depth, batch, masks


def test_fill_path_measurements_stamps_ok_and_scan_reason() -> None:
    from ridgeback_autonomy.common.miss_reason import MissReason
    from ridgeback_autonomy.perception.g1_mask_measurement_node import fill_path_measurements

    intrinsics, depth, batch, masks = _status_fixture()
    fill_path_measurements(
        batch, masks, intrinsics, depth, None,
        camera_rotation=np.eye(3), camera_translation=np.zeros(3), front_offset_m=0.0,
        isolation_2d=None, isolation_3d=None, scan_reason=MissReason.NO_SCAN)

    det = batch.detections[0]
    assert det.projective_ranging_status == int(MissReason.OK)
    assert det.euclidean_reconstruction_status == int(MissReason.OK)
    assert det.polar_profiling_status == int(MissReason.NO_SCAN)


def test_fill_path_measurements_no_depth_stamps_no_depth_frame() -> None:
    from ridgeback_autonomy.common.miss_reason import MissReason
    from ridgeback_autonomy.perception.g1_mask_measurement_node import fill_path_measurements

    intrinsics, _depth, batch, masks = _status_fixture()
    fill_path_measurements(
        batch, masks, intrinsics, None, None,
        camera_rotation=np.eye(3), camera_translation=np.zeros(3), front_offset_m=0.0,
        isolation_2d=None, isolation_3d=None, scan_reason=MissReason.TF_MISS_SCAN)

    det = batch.detections[0]
    assert det.projective_ranging_status == int(MissReason.NO_DEPTH_FRAME)
    assert det.euclidean_reconstruction_status == int(MissReason.NO_DEPTH_FRAME)
    assert det.polar_profiling_status == int(MissReason.TF_MISS_SCAN)


def test_fill_path_measurements_skips_none_mask() -> None:
    from ridgeback_autonomy.perception.g1_mask_measurement_node import fill_path_measurements

    intrinsics, depth, batch, _masks = _status_fixture()
    fill_path_measurements(
        batch, [None], intrinsics, depth, None,
        camera_rotation=np.eye(3), camera_translation=np.zeros(3), front_offset_m=0.0,
        isolation_2d=None, isolation_3d=None)

    # A None mask is already status-stamped by masks_for_batch; fill leaves it.
    assert batch.detections[0].projective_ranging_status is None


def test_fill_with_no_depth_still_lets_polar_fill_the_same_frame() -> None:
    # The depth paths and polar profiling are independent: a source that
    # produced nothing at this stamp costs the two depth rows, not the scan row.
    from ridgeback_autonomy.common.miss_reason import MissReason

    batch = build_fill_batch()
    masks = [mask_from_array(tight_blob(), MaskPrecision.TIGHT)]

    fill_path_measurements(
        batch, masks, FILL_INTRINSICS, None,
        build_fill_scan(lambda ratios: np.abs(ratios) <= 0.05),
        camera_rotation=LEVEL_OPTICAL_TO_BASE,
        camera_translation=ZERO_TRANSLATION,
        front_offset_m=0.25,
        isolation_2d=forbidden_isolation,
        isolation_3d=forbidden_isolation,
    )

    detection = batch.detections[0]
    assert detection.projective_ranging_status == int(MissReason.NO_DEPTH_FRAME)
    assert detection.euclidean_reconstruction_status == int(MissReason.NO_DEPTH_FRAME)
    assert detection.polar_profiling_status == int(MissReason.OK)
    assert detection.polar_profiling_forward_m == pytest.approx(2.0 - 0.25, abs=0.01)


# --- Estimator subset: a path outside ``enabled`` is never run, so it has no
# value AND no status -- the run simply has no such row, exactly as an
# unselected legacy estimator has none. ---


def test_fill_runs_only_the_enabled_paths_leaving_the_rest_unset() -> None:
    from ridgeback_autonomy.common.miss_reason import MissReason

    batch = build_fill_batch()
    masks = [mask_from_array(tight_blob(), MaskPrecision.TIGHT)]

    # Depth AND scan both available: only the selection keeps the depth rows out.
    fill_path_measurements(
        batch, masks, FILL_INTRINSICS, build_fill_depth(),
        build_fill_scan(lambda ratios: np.abs(ratios) <= 0.05),
        camera_rotation=LEVEL_OPTICAL_TO_BASE,
        camera_translation=ZERO_TRANSLATION,
        front_offset_m=0.25,
        isolation_2d=forbidden_isolation,
        isolation_3d=forbidden_isolation,
        enabled=frozenset({'polar_profiling'}),
    )

    detection = batch.detections[0]
    assert detection.polar_profiling_status == int(MissReason.OK)
    assert detection.polar_profiling_forward_m == pytest.approx(2.0 - 0.25, abs=0.01)
    # Not "missed for want of depth" -- never asked.
    assert detection.projective_ranging_status is None
    assert detection.euclidean_reconstruction_status is None
    assert detection.projective_ranging_distance_m is None
    assert detection.euclidean_reconstruction_distance_m is None


def test_fill_splits_the_two_depth_paths_independently() -> None:
    from ridgeback_autonomy.common.miss_reason import MissReason

    batch = build_fill_batch()
    masks = [mask_from_array(tight_blob(), MaskPrecision.TIGHT)]

    fill_path_measurements(
        batch, masks, FILL_INTRINSICS, build_fill_depth(), None,
        camera_rotation=LEVEL_OPTICAL_TO_BASE,
        camera_translation=ZERO_TRANSLATION,
        front_offset_m=0.25,
        isolation_2d=forbidden_isolation,
        isolation_3d=forbidden_isolation,
        enabled=frozenset({'projective_ranging'}),
    )

    detection = batch.detections[0]
    assert detection.projective_ranging_status == int(MissReason.OK)
    assert detection.euclidean_reconstruction_status is None
    # The scan row is not stamped with its scan_reason either.
    assert detection.polar_profiling_status is None


def test_fill_without_depth_leaves_a_disabled_depth_row_unset_not_missing() -> None:
    # The NO_DEPTH_FRAME stamp follows the selection too: a row that was not
    # asked for must not be reported as a depth miss, or the benchmark's
    # miss-reason tally would blame a source the run never used.
    from ridgeback_autonomy.common.miss_reason import MissReason

    batch = build_fill_batch()
    masks = [mask_from_array(tight_blob(), MaskPrecision.TIGHT)]

    fill_path_measurements(
        batch, masks, FILL_INTRINSICS, None, None,
        camera_rotation=LEVEL_OPTICAL_TO_BASE,
        camera_translation=ZERO_TRANSLATION,
        front_offset_m=0.25,
        isolation_2d=forbidden_isolation,
        isolation_3d=forbidden_isolation,
        enabled=frozenset({'euclidean_reconstruction'}),
    )

    detection = batch.detections[0]
    assert detection.euclidean_reconstruction_status == int(MissReason.NO_DEPTH_FRAME)
    assert detection.projective_ranging_status is None


def test_fill_records_no_beams_when_polar_is_not_enabled() -> None:
    batch = build_fill_batch()
    masks = [mask_from_array(tight_blob(), MaskPrecision.TIGHT)]
    records: list = []

    fill_path_measurements(
        batch, masks, FILL_INTRINSICS, build_fill_depth(),
        build_fill_scan(lambda ratios: np.abs(ratios) <= 0.05),
        camera_rotation=LEVEL_OPTICAL_TO_BASE,
        camera_translation=ZERO_TRANSLATION,
        front_offset_m=0.25,
        isolation_2d=forbidden_isolation,
        isolation_3d=forbidden_isolation,
        beam_records=records,
        enabled=frozenset({'projective_ranging', 'euclidean_reconstruction'}),
    )

    assert records == []


def test_resolve_enabled_estimators_keeps_only_the_mask_rows() -> None:
    # One comma-separated list selects across both stacks; this node keeps the
    # keys it owns and ignores the legacy ones.
    assert resolve_enabled_estimators('all') == frozenset({
        'projective_ranging', 'euclidean_reconstruction', 'polar_profiling'})
    assert resolve_enabled_estimators('rgb,lidar,polar_profiling') == frozenset(
        {'polar_profiling'})


def test_resolve_enabled_estimators_rejects_a_selection_with_no_mask_row() -> None:
    # Not a quiet no-op: the node would publish empty measurements forever.
    with pytest.raises(ValueError, match='no mask estimator'):
        resolve_enabled_estimators('rgb,lidar')


def test_resolve_enabled_estimators_rejects_an_unknown_key() -> None:
    with pytest.raises(ValueError, match='Unsupported estimator'):
        resolve_enabled_estimators('polar_profiling,not_an_estimator')


# --- Depth acquisition in the node: the source is pulled at the detection
# stamp, on a stream this process buffered itself. ---


class _StubDepthSource:
    """Injected in place of a real depth source: scripted produce(), no model."""

    def __init__(self, input_kind: str = 'depth') -> None:
        self.input_kind = input_kind
        self.produce_calls: list = []
        self.frame = None

    def produce(self, msg):
        self.produce_calls.append(msg)
        return self.frame


@pytest.fixture
def ros_context():
    rclpy = pytest.importorskip('rclpy')
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def _mask_node(source, **parameters):
    from rclpy.parameter import Parameter
    from ridgeback_autonomy.perception.g1_mask_measurement_node import (
        G1MaskMeasurementNode,
    )

    return G1MaskMeasurementNode(
        depth_source=source,
        parameter_overrides=[
            Parameter(name, value=value) for name, value in parameters.items()
        ],
    )


def depth_image(sec: int, nanosec: int) -> Image:
    msg = Image()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.encoding = '32FC1'
    msg.height, msg.width = FILL_HEIGHT, FILL_WIDTH
    return msg


def test_depth_input_is_the_depth_stream_matched_on_the_exact_stamp(ros_context) -> None:
    node = _mask_node(_StubDepthSource('depth'))
    try:
        assert node.depth_input_buffer is node.depth_buffer

        buffered = depth_image(7, 42)
        node.depth_callback(buffered)

        assert node.depth_input_buffer.lookup(stamp(7, 42)) is buffered
        # One tick off is a miss -- the rule is exact-stamp, no tolerance -- and
        # a miss is what lands as NO_DEPTH_FRAME.
        assert node.depth_input_buffer.lookup(stamp(7, 43)) is None
    finally:
        node.destroy_node()


def test_monocular_source_reads_the_color_frame_and_no_depth_stream(ros_context) -> None:
    node = _mask_node(_StubDepthSource('color'))
    try:
        # A color-input source subscribes to no depth stream at all.
        assert node.depth_buffer is None
        assert node.depth_input_buffer is node.color_buffer
        # The box gate loads no segmenter, so the color buffer exists purely for
        # the depth source here.
        assert node.segmenter is None

        buffered = color_image(7, 42)
        node.color_callback(buffered)

        assert node.depth_input_buffer.lookup(stamp(7, 42)) is buffered
    finally:
        node.destroy_node()


def test_depth_for_batch_converts_the_matched_frame_once(ros_context) -> None:
    source = _StubDepthSource('depth')
    node = _mask_node(source)
    try:
        batch = build_fill_batch()
        depth_m = build_fill_depth()
        source.frame = (depth_m, Header())
        matched = depth_image(7, 42)

        assert node.depth_for_batch(matched, batch) is depth_m
        # Nothing buffered at the stamp: the source is never asked, and the
        # caller reads the None as NO_DEPTH_FRAME.
        assert node.depth_for_batch(None, batch) is None
        assert source.produce_calls == [matched]
    finally:
        node.destroy_node()


def test_depth_for_batch_none_when_the_source_declines_the_frame(ros_context) -> None:
    # A model that is unavailable or mid-cooldown, or an unsupported encoding:
    # the source returns None, which means "no usable depth at this stamp".
    source = _StubDepthSource('depth')
    node = _mask_node(source)
    try:
        source.frame = None

        assert node.depth_for_batch(depth_image(7, 42), build_fill_batch()) is None
    finally:
        node.destroy_node()


def test_depth_for_batch_rejects_a_grid_the_masks_cannot_index(ros_context) -> None:
    source = _StubDepthSource('depth')
    node = _mask_node(source)
    try:
        source.frame = (np.zeros((FILL_HEIGHT + 1, FILL_WIDTH), dtype=np.float32),
                        Header())

        assert node.depth_for_batch(depth_image(7, 42), build_fill_batch()) is None
    finally:
        node.destroy_node()


def test_aligned_depth_debug_carries_the_detection_stamp(ros_context) -> None:
    node = _mask_node(_StubDepthSource('depth'))
    published: list = []
    node.aligned_depth_debug_pub.publish = lambda msg: published.append(msg)
    node.aligned_depth_debug_pub.get_subscription_count = lambda: 1
    try:
        header = Header()
        header.frame_id = 'camera_0_color_optical'
        header.stamp.sec, header.stamp.nanosec = 7, 42

        node.publish_aligned_depth_debug(build_fill_depth(), header)

        assert len(published) == 1
        assert published[0].encoding == '32FC1'
        assert published[0].header.frame_id == 'camera_0_color_optical'
        assert (published[0].header.stamp.sec,
                published[0].header.stamp.nanosec) == (7, 42)
    finally:
        node.destroy_node()


def test_aligned_depth_debug_skips_the_encode_with_no_subscriber(ros_context) -> None:
    node = _mask_node(_StubDepthSource('depth'))
    published: list = []
    node.aligned_depth_debug_pub.publish = lambda msg: published.append(msg)
    node.aligned_depth_debug_pub.get_subscription_count = lambda: 0
    try:
        node.publish_aligned_depth_debug(build_fill_depth(), Header())
        # Nothing produced either: a frame with no depth has nothing to show.
        node.aligned_depth_debug_pub.get_subscription_count = lambda: 1
        node.publish_aligned_depth_debug(None, Header())

        assert published == []
    finally:
        node.destroy_node()


# --- The subset drives what the node subscribes to and builds: a run pays for
# the inputs its selected paths read, and nothing else. ---


def subscribed_topics(node) -> set[str]:
    return {subscription.topic_name for subscription in node.subscriptions}


def test_polar_only_run_builds_no_depth_machinery(ros_context) -> None:
    # Injecting a source and selecting no depth path: the selection wins. This
    # is the case that matters under depth_source:=monocular, where building
    # the source is what loads Depth-Anything.
    node = _mask_node(_StubDepthSource('depth'), enabled_estimators='polar_profiling')
    try:
        assert node.needs_depth is False
        assert node.depth_source is None
        assert node.depth_buffer is None
        assert node.depth_input_buffer is None
        assert node.aligned_depth_debug_pub is None
        assert not any(
            topic.endswith('depth/image') for topic in subscribed_topics(node))
        # The scan side is fully wired.
        assert node.scan_buffer is not None
        assert node.ray_marker_pub is not None
    finally:
        node.destroy_node()


def test_depth_only_run_builds_no_scan_machinery(ros_context) -> None:
    node = _mask_node(
        _StubDepthSource('depth'),
        enabled_estimators='projective_ranging,euclidean_reconstruction')
    try:
        assert node.needs_scan is False
        assert node.scan_buffer is None
        assert node.ray_marker_pub is None
        assert not any(
            topic.endswith('scan') for topic in subscribed_topics(node))
        assert node.depth_input_buffer is node.depth_buffer
        assert node.aligned_depth_debug_pub is not None
    finally:
        node.destroy_node()


def test_polar_only_run_subscribes_to_no_color_stream_for_depth(ros_context) -> None:
    # A color-input (monocular) source with no depth path selected: the color
    # buffer exists only for readers that want it, and neither the absent depth
    # source nor the box gate does. Without this the worker would hand a color
    # frame to a depth source that does not exist.
    node = _mask_node(_StubDepthSource('color'), enabled_estimators='polar_profiling')
    try:
        assert node.color_buffer is None
        assert node.depth_input_buffer is None
    finally:
        node.destroy_node()


def test_default_run_fills_all_three_rows(ros_context) -> None:
    node = _mask_node(_StubDepthSource('depth'))
    try:
        assert node.enabled_estimators == frozenset({
            'projective_ranging', 'euclidean_reconstruction', 'polar_profiling'})
        assert node.needs_depth and node.needs_scan
    finally:
        node.destroy_node()
