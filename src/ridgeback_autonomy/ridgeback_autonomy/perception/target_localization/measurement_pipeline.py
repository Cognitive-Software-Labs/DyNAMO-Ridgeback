"""Pure batch-local measurement and debug-artifact helpers."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image

from ridgeback_autonomy.common.markers import PolarBeamRecord
from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.target_localization.core.box_gate import (
    MAX_BOX_FRAME_FRACTION,
    box_within_frame_fraction,
)
from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    MASK_DEPTH_GATE_DEFAULT,
    prepare_depth_region,
    valid_depth,
)
from ridgeback_autonomy.perception.target_localization.core.euclidean_reconstruction import (
    localize_prepared_euclidean_reconstruction,
)
from ridgeback_autonomy.perception.target_localization.core.polar_profiling import (
    localize_projected_polar_profiling,
    project_scan_to_image,
    select_bbox_beams,
)
from ridgeback_autonomy.perception.target_localization.core.projective_ranging import (
    localize_prepared_projective_ranging,
)
from ridgeback_autonomy.perception.target_localization.core.ranging_defaults import (
    MIN_VALID_SAMPLES as MIN_VALID_PIXELS_DEFAULT,
)
from ridgeback_autonomy.perception.target_localization.core.vehicle_frame import (
    optical_to_base_planar,
)
from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    ESTIMATOR_FIELD_KEYS,
    MASK_ESTIMATORS,
    MASK_GATES,
    nearest_instance_index,
    parse_estimators,
    selected_mask_estimators,
)


def resolve_mask_gate(value) -> str:
    """Validate the ``mask_gate`` parameter value (``box`` | ``silhouette``)."""

    gate = str(value).strip()
    if gate not in MASK_GATES:
        supported = ', '.join(MASK_GATES)
        raise ValueError(f'Unknown mask_gate "{gate}". Expected one of: {supported}')
    return gate


def resolve_enabled_estimators(value) -> frozenset:
    """The ``enabled_estimators`` parameter as this node's mask-row subset.

    A selection containing no mask row is a launch error rather than a node
    that publishes empty measurements forever.
    """

    enabled = frozenset(selected_mask_estimators(parse_estimators(value)))
    if not enabled:
        supported = ', '.join(sorted(MASK_ESTIMATORS))
        raise ValueError(
            f'enabled_estimators "{value}" selects no mask estimator; this node '
            f'fills only: {supported}.')
    return enabled


def set_mask_estimator_status(
    detection,
    reason: MissReason,
    enabled=MASK_ESTIMATORS,
) -> None:
    """Stamp one miss reason on the enabled mask-estimator status fields."""

    if 'projective_ranging' in enabled:
        detection.projective_ranging_status = int(reason)
    if 'euclidean_reconstruction' in enabled:
        detection.euclidean_reconstruction_status = int(reason)
    if 'polar_profiling' in enabled:
        detection.polar_profiling_status = int(reason)


def fill_path_measurements(
    batch,
    masks,
    intrinsics,
    depth_m,
    scan_points,
    *,
    camera_rotation: np.ndarray,
    camera_translation: np.ndarray,
    front_offset_m: float,
    isolation_2d,
    isolation_3d,
    depth_max: float = MASK_DEPTH_GATE_DEFAULT,
    min_valid_pixels: int = MIN_VALID_PIXELS_DEFAULT,
    scan_reason: MissReason = MissReason.NO_SCAN,
    beam_records: list | None = None,
    enabled=MASK_ESTIMATORS,
) -> None:
    """Run every enabled path for each index-aligned detection and region, in place.

    A ``None`` region is a deliberate no-fallback miss and stays unset. Paths
    outside ``enabled`` are never run or status-stamped. Frame-wide depth
    validity and scan projection are each computed lazily at most once, then
    shared across detections; optional beam records are by-products of polar
    work that was already selected, never a reason to run it.

    Both depth estimators receive the **same** ``PreparedDepthRegion``, prepared
    once per detection. Preparing it twice would cost a second pass and, worse,
    leave the two rows free to disagree about which pixels were selected.

    ``min_valid_pixels`` is the sufficiency floor for **both** depth rows. Their
    pre-isolation guards count the same array -- the prepared region's
    ``valid_masked`` -- so the two names ("pixels", "points") describe one
    quantity, and giving each row its own floor would let one report on a
    selection the other rejected, which is the same disagreement preparing the
    region once exists to prevent.
    """

    wants_projective = 'projective_ranging' in enabled
    wants_euclidean = 'euclidean_reconstruction' in enabled
    wants_polar = 'polar_profiling' in enabled
    frame_valid_depth = None
    scan_projection = None

    for index, (detection, mask) in enumerate(zip(batch.detections, masks)):
        if mask is None:
            continue

        if depth_m is None:
            if wants_projective:
                detection.projective_ranging_status = int(MissReason.NO_DEPTH_FRAME)
            if wants_euclidean:
                detection.euclidean_reconstruction_status = int(MissReason.NO_DEPTH_FRAME)
        else:
            prepared = None
            if wants_projective or wants_euclidean:
                if frame_valid_depth is None:
                    frame_valid_depth = valid_depth(depth_m, depth_max)
                prepared = prepare_depth_region(mask, depth_m, frame_valid_depth)

            if wants_projective:
                result_a, reason_a = localize_prepared_projective_ranging(
                    prepared, intrinsics, isolation=isolation_2d,
                    min_valid_pixels=min_valid_pixels)
                detection.projective_ranging_status = int(reason_a)
                if result_a is not None:
                    (
                        detection.projective_ranging_lateral_m,
                        detection.projective_ranging_forward_m,
                        detection.projective_ranging_distance_m,
                    ) = optical_to_base_planar(
                        result_a.xyz_optical, camera_rotation, camera_translation,
                        front_offset_m)

            if wants_euclidean:
                result_b, reason_b = localize_prepared_euclidean_reconstruction(
                    prepared, intrinsics, isolation=isolation_3d,
                    min_valid_points=min_valid_pixels)
                detection.euclidean_reconstruction_status = int(reason_b)
                if result_b is not None:
                    (
                        detection.euclidean_reconstruction_lateral_m,
                        detection.euclidean_reconstruction_forward_m,
                        detection.euclidean_reconstruction_distance_m,
                    ) = optical_to_base_planar(
                        result_b.xyz_optical, camera_rotation, camera_translation,
                        front_offset_m)

        if not wants_polar:
            continue

        if scan_points is None:
            detection.polar_profiling_status = int(scan_reason)
        else:
            points_optical, valid = scan_points
            if scan_projection is None:
                scan_projection = project_scan_to_image(
                    points_optical, valid, intrinsics)
            attempt = localize_projected_polar_profiling(scan_projection, mask)
            result_c = attempt.result
            detection.polar_profiling_status = int(attempt.reason)
            if result_c is not None:
                # Polar profiling recovers optical X/Z only. Optical Y is
                # unobservable and substituted with zero before the full
                # camera-to-base extrinsic is applied.
                x_optical, z_optical = (float(value) for value in result_c.xz_optical)
                (
                    detection.polar_profiling_lateral_m,
                    detection.polar_profiling_forward_m,
                    detection.polar_profiling_distance_m,
                ) = optical_to_base_planar(
                    (x_optical, 0.0, z_optical), camera_rotation,
                    camera_translation, front_offset_m)

            if beam_records is not None:
                if result_c is not None:
                    selected, merged = result_c.selected_beams, result_c.merged_beams
                else:
                    selected = attempt.selected_beams
                    merged = np.empty(0, dtype=np.intp)
                beam_records.append(PolarBeamRecord(
                    detection_index=index,
                    selected=selected,
                    merged=merged,
                    in_bbox=select_bbox_beams(scan_projection, detection.bbox_xyxy),
                ))


def nearest_beam_record(batch, beam_records):
    """The nearest ranked detection's ray record, or the first useful miss.

    Match by ``detection_index`` because empty segmentations do not create beam
    records. If no estimator ranked, the first record still visualizes why.
    """

    if not beam_records:
        return None

    def read_distance(estimator: str, index: int) -> float | None:
        return getattr(batch.detections[index], ESTIMATOR_FIELD_KEYS[estimator], None)

    nearest = nearest_instance_index(batch.count, read_distance)
    for record in beam_records:
        if record.detection_index == nearest:
            return record
    return beam_records[0]


def encode_mask_debug_image(masks, image_height: int, image_width: int, header) -> Image:
    """Union of the frame's mask regions as a ``mono8`` Image (255 = object).

    Each region is blitted into the one output buffer. Materializing a
    full-frame mask per detection first would allocate the whole image once per
    detection to produce the same bytes.
    """

    union = np.zeros((image_height, image_width), dtype=np.uint8)
    for mask in masks:
        if mask is not None:
            mask.blit_into(union, 255)
    msg = Image()
    msg.header = header
    msg.height = image_height
    msg.width = image_width
    msg.encoding = 'mono8'
    msg.is_bigendian = 0
    msg.step = image_width
    msg.data = union.tobytes()
    return msg


def grid_mismatch_warning(intrinsics, batch) -> str | None:
    """Warning when ``camera_info`` and detection grids differ, else ``None``.

    A mismatch means masks cannot safely index either aligned depth or the
    projected scan, so every path for the frame must be skipped.
    """

    if (intrinsics.height, intrinsics.width) == (batch.image_height, batch.image_width):
        return None
    return (
        f'camera_info grid ({intrinsics.height}, {intrinsics.width}) does not '
        f'match the detection grid ({batch.image_height}, {batch.image_width}); '
        'masks cannot index the projection. Check the camera_info_topic parameter.'
    )
