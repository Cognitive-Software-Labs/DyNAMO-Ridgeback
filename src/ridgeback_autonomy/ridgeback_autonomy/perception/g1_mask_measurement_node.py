#!/usr/bin/env python3
"""Mask-based measurement node: the projective ranging / euclidean
reconstruction / polar profiling benchmark rows.

Consumes detections plus the camera stream the depth source reads and
the 2D LiDAR scan, builds one mask per detection, and runs the localization
paths per mask: the two depth paths
(``perception/core/projective_ranging.py`` / ``euclidean_reconstruction.py``)
against the aligned depth frame, and polar profiling
(``perception/core/polar_profiling.py``) against the scan projected into the
camera optical frame via TF. Each path's result is converted from the camera
optical frame to the base-frame planar-distance convention every benchmark
row shares (ground truth included) through the optical -> base extrinsics
from TF -- the camera's mounting pose, translation included, is modeled
exactly -- and published as ``G1Measurements`` with
the identity fields of the source detections message -- so the benchmark
runner can merge them into the same aligned event as the pointcloud
measurements.

Which of the three rows a run fills is the ``enabled_estimators`` parameter,
the same name and ``all`` default ``g1_pointcloud_measurement_node`` carries,
so one comma-separated list selects across both stacks. A path that
was not selected is never run: its fields stay NaN, its status stays ``UNSET``,
and the inputs only it needs are never subscribed to -- a polar-only run builds
no depth source at all (so ``depth_source:=monocular`` loads no model), and a
depth-only run never touches the scan.

The mask front-end is the ``mask_gate`` parameter: ``box`` rasterizes each
detection box into a ``rect`` mask (no model, no extra input); ``silhouette``
prompts a segmentation model (``perception/core/segmentation.py``) with the
boxes on the exact color frame the detections were made on, producing
``tight`` masks. Masks never cross the wire either way
(``mask_component.md`` Section 5.3) -- the segmenter runs in this process.

The aligned depth frame is produced here, on demand, at the detection stamp:
the ``depth_source`` parameter picks a strategy from
``perception/core/depth_sources.py``, this node buffers that strategy's input
stream raw and converts only the frame the detections were made on. Every
frame is received and none is discarded blind, so the exact-stamp match is a
lookup into a buffer this process filled rather than an intersection with some
other process's thinning. Nothing downstream branches on which source ran.
Polar profiling needs no depth frame -- only the scan, the mask, and the
color-grid intrinsics -- so it runs independently of depth availability.
Deliberately independent of the pointcloud estimator
(``perception/core/pointcloud_ranging.py``): constants are mirrored by value,
never imported.
"""

from __future__ import annotations

import math
import threading
import time
import traceback
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from ridgeback_autonomy.benchmarking.estimators import (
    DEPTH_PATH_ESTIMATORS,
    ESTIMATOR_FIELD_KEYS,
    MASK_ESTIMATORS,
    nearest_instance_index,
    parse_estimators,
    selected_mask_estimators,
)
from ridgeback_autonomy.common.markers import PolarBeamRecord, build_polar_ray_markers
from ridgeback_autonomy.common.messages import (
    batch_from_detections_message,
    build_measurements_message,
)
from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.common.tf_utils import lookup_transform_components
from ridgeback_autonomy.msg import G1Detections, G1Measurements
from ridgeback_autonomy.perception.core.depth_common import (
    DEPTH_GATE_DISABLED,
    MASK_DEPTH_GATE_DEFAULT,
    resolve_depth_gate,
)
from ridgeback_autonomy.perception.core.depth_sources import (
    DEPTH_ANYTHING_MODEL_ID_DEFAULT,
    DEPTH_SOURCE_STEREOSCOPIC,
    build_depth_source,
    encode_depth_message,
)
from ridgeback_autonomy.perception.core.image_utils import convert_color_image_message
from ridgeback_autonomy.perception.core.intrinsics import intrinsics_from_camera_info
from ridgeback_autonomy.perception.core.isolation_2d import (
    ISOLATION_2D_DEFAULT,
    ISOLATION_2D_RECIPES,
)
from ridgeback_autonomy.perception.core.isolation_3d import (
    BASE_ABOVE_FLOOR_M_DEFAULT,
    ISOLATION_3D_DEFAULT,
    ISOLATION_3D_RECIPES,
    build_isolation_3d,
    camera_floor_geometry,
)
from ridgeback_autonomy.perception.core.mask import (
    MaskPrecision,
    mask_from_array,
    rasterize_detection,
)
from ridgeback_autonomy.perception.core.projective_ranging import localize_projective_ranging
from ridgeback_autonomy.perception.core.euclidean_reconstruction import localize_euclidean_reconstruction
from ridgeback_autonomy.perception.core.polar_profiling import (
    beams_in_bbox,
    localize_polar_profiling,
    scan_points_optical,
    select_beams,
)
from ridgeback_autonomy.perception.core.segmentation import (
    SEGMENTATION_MIN_PREDICTED_IOU_DEFAULT,
    SEGMENTATION_MODEL_DEFAULT,
    SamBoxSegmenter,
)


RAW_DETECTIONS_TOPIC = 'detections/g1/raw'
MASK_MEASUREMENTS_TOPIC = 'measurements/g1/mask'
COLOR_TOPIC_DEFAULT = 'sensors/camera_0/color/image'
DEPTH_TOPIC_DEFAULT = 'sensors/camera_0/depth/image'
CAMERA_INFO_TOPIC_DEFAULT = 'sensors/camera_0/color/camera_info'
MASK_DEBUG_TOPIC = 'debug/g1/mask'
ALIGNED_DEPTH_DEBUG_TOPIC = 'debug/g1/mask/aligned_depth'
RAY_MARKER_TOPIC = 'visualization/g1/polar_rays'

# Markers expire rather than being explicitly deleted, so they vanish on their
# own when detections stop. Mirrors the estimate rings' lifetime.
RAY_MARKER_LIFETIME_SEC = 1.5

ROBOT_FRONT_OFFSET_M_DEFAULT = 0.25  # mirrors vehicle_frame.ROBOT_FRONT_OFFSET_M
BASE_FRAME_DEFAULT = 'base_link'

MASK_GATE_BOX = 'box'
MASK_GATE_SILHOUETTE = 'silhouette'
MASK_GATES = (MASK_GATE_BOX, MASK_GATE_SILHOUETTE)

# ~0.5 s of color frames at 30 fps -- comfortably above the detector latency
# (~200 ms at 5 FPS), so the exact-stamp lookup only misses when the pipeline
# is genuinely stalled.
COLOR_BUFFER_DEPTH_DEFAULT = 15

# A detector box covering more than this fraction of the frame is almost always
# a failure (OWLv2 occasionally boxes the whole scene at close range); masking
# with it isolates the background wall and poisons every path. Detections that
# fail this gate are skipped (fields stay NaN, trial drops) rather than measured
# against the room -- the observed close-range outlier trials.
MAX_BOX_FRAME_FRACTION = 0.60

# The depth source's input frames and the scans are matched to the detection
# stamp, not paired latest-wins, so the mask and the depth/scan it reads come
# from the same instant. The depth input shares the color frame's stamp (exact
# match); the scan free-runs at ~40 Hz, so it is matched to the nearest buffered
# stamp within SCAN_MATCH_TOLERANCE_S. Buffer depths span the detector latency
# (~200 ms) plus jitter.
DEPTH_MATCH_BUFFER_DEPTH = 15
SCAN_MATCH_BUFFER_DEPTH = 20
SCAN_MATCH_TOLERANCE_S_DEFAULT = 0.05


def optical_to_base_planar(
    xyz_optical,
    rotation: np.ndarray,
    translation: np.ndarray,
    front_offset_m: float,
) -> tuple[float, float, float]:
    """Camera-optical point -> ``(lateral_m, forward_m, distance_m)`` in the base frame.

    ``rotation`` / ``translation`` are the camera-optical -> base extrinsics
    from TF, so the camera's mounting pose (translation included) is modeled
    exactly instead of assuming the optical center sits at the base origin.
    Lateral is base +Y (left-positive, REP-103), matching the ground truth and
    the legacy lidar/pointcloud rows; the robot front offset is subtracted
    from base forward (+X) before the planar distance.
    """

    point_base = (
        np.asarray(rotation, dtype=np.float64)
        @ np.asarray(xyz_optical, dtype=np.float64)
        + np.asarray(translation, dtype=np.float64)
    )
    lateral_m = float(point_base[1])
    forward_m = float(point_base[0]) - front_offset_m
    distance_m = math.hypot(lateral_m, forward_m)
    return lateral_m, forward_m, distance_m


def resolve_mask_gate(value) -> str:
    """Validate the ``mask_gate`` parameter value (``box`` | ``silhouette``)."""

    gate = str(value).strip()
    if gate not in MASK_GATES:
        supported = ', '.join(MASK_GATES)
        raise ValueError(f'Unknown mask_gate "{gate}". Expected one of: {supported}')
    return gate


def resolve_enabled_estimators(value) -> frozenset:
    """The ``enabled_estimators`` parameter as this node's mask-row subset.

    Shares ``parse_estimators`` with the legacy stack, so one comma-separated
    list can select across both and each node keeps the keys it owns. A value
    naming only non-mask estimators leaves nothing for this node to fill, which
    is a launch mistake rather than a quiet no-op -- the node would publish
    empty measurements forever -- so it raises.
    """

    enabled = frozenset(selected_mask_estimators(parse_estimators(value)))
    if not enabled:
        supported = ', '.join(sorted(MASK_ESTIMATORS))
        raise ValueError(
            f'enabled_estimators "{value}" selects no mask estimator; this node '
            f'fills only: {supported}.')
    return enabled


def box_within_frame_fraction(
    bbox_xyxy,
    image_height: int,
    image_width: int,
    max_fraction: float = MAX_BOX_FRAME_FRACTION,
) -> bool:
    """True if the detector box covers at most ``max_fraction`` of the frame.

    A near-full-frame box (see ``MAX_BOX_FRAME_FRACTION``) fails the gate; the
    caller then skips that detection instead of masking the whole scene. Pure so
    it can be unit-tested without a node (mirrors ``grid_mismatch_warning``).
    """

    frame_area = float(image_height) * float(image_width)
    if frame_area <= 0.0:
        return False
    x1, y1, x2, y2 = bbox_xyxy
    box_area = float(max(0, x2 - x1)) * float(max(0, y2 - y1))
    return box_area <= max_fraction * frame_area


def stamp_key(stamp) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


class StampedMessageBuffer:
    """Stamp-keyed rolling buffer of recent messages, matched by header stamp.

    Used for the color frame (the silhouette gate's prompt image and the
    monocular depth source's input), the raw depth stream, and the scan.
    Color and depth share the exact color stamp, so they match with
    ``lookup`` (no tolerance); the scan free-runs, so it matches with
    ``lookup_nearest`` within a tolerance window. A miss means the message aged
    out (or never arrived); the caller skips that source's paths rather than
    pairing whatever arrived most recently, which would smear distance under
    motion or mislabel the silhouette benchmark row.
    """

    def __init__(self, depth: int) -> None:
        self.depth = int(depth)
        self._msgs: OrderedDict[tuple[int, int], object] = OrderedDict()

    def __len__(self) -> int:
        return len(self._msgs)

    def store(self, msg) -> None:
        key = stamp_key(msg.header.stamp)
        self._msgs[key] = msg
        self._msgs.move_to_end(key)
        while len(self._msgs) > self.depth:
            self._msgs.popitem(last=False)

    def lookup(self, stamp):
        """Exact stamp match, or ``None``."""

        return self._msgs.get(stamp_key(stamp))

    def lookup_nearest(self, stamp, tolerance_s: float):
        """Buffered message closest to ``stamp`` within ``tolerance_s``, or ``None``."""

        sec, nanosec = stamp_key(stamp)
        target_ns = sec * 1_000_000_000 + nanosec
        tol_ns = int(tolerance_s * 1_000_000_000)
        best = None
        best_delta = None
        for (key_sec, key_nanosec), msg in self._msgs.items():
            delta = abs(key_sec * 1_000_000_000 + key_nanosec - target_ns)
            if delta <= tol_ns and (best_delta is None or delta < best_delta):
                best_delta = delta
                best = msg
        return best

    def stamps_ns(self) -> list[int]:
        """Buffered stamps as nanoseconds, oldest first. Diagnostics only."""

        return [sec * 1_000_000_000 + nanosec for sec, nanosec in self._msgs]


class DepthMatchDiagnostics:
    """Counts why the depth input lookup hits or misses, for one run.

    Answers three competing explanations for a ``NO_DEPTH_FRAME`` with one
    log line, without changing any matching behaviour:

    - **reception loss** -- ``depth`` received well below ``color``. The frame
      was published but this process never got it.
    - **stamp mismatch** -- both streams received at the same rate, the target
      stamp sits inside the buffered span, and the nearest buffered stamp is a
      near-constant offset away. The streams are not stamped alike.
    - **lag** -- the target is *newer* than everything buffered, so the depth
      frame simply had not arrived yet when the detection was processed.
    """

    def __init__(self) -> None:
        self.color_rx = 0
        self.depth_rx = 0
        self.hits = 0
        self.misses = 0
        self.miss_target_newer = 0
        self.miss_target_older = 0
        self.miss_target_inside = 0
        self.miss_nearest_delta_ms: list[float] = []
        self.empty_buffer_misses = 0

    def record_lookup(self, hit: bool, target_ns: int, buffered_ns: list[int]) -> None:
        if hit:
            self.hits += 1
            return
        self.misses += 1
        if not buffered_ns:
            self.empty_buffer_misses += 1
            return
        if target_ns > max(buffered_ns):
            self.miss_target_newer += 1
        elif target_ns < min(buffered_ns):
            self.miss_target_older += 1
        else:
            self.miss_target_inside += 1
        nearest = min(abs(stamp - target_ns) for stamp in buffered_ns)
        self.miss_nearest_delta_ms.append(nearest / 1e6)

    def summary(self) -> str:
        lookups = self.hits + self.misses
        ratio = (self.depth_rx / self.color_rx) if self.color_rx else float('nan')
        hit_rate = (self.hits / lookups) if lookups else float('nan')
        lines = [
            f'depth-match: rx color={self.color_rx} depth={self.depth_rx} '
            f'(depth/color={ratio:.2f}) | lookups={lookups} hit={self.hits} '
            f'miss={self.misses} (hit_rate={hit_rate:.2f})',
        ]
        if self.misses:
            deltas = sorted(self.miss_nearest_delta_ms)
            if deltas:
                median = deltas[len(deltas) // 2]
                spread = (
                    f'min={deltas[0]:.1f} med={median:.1f} max={deltas[-1]:.1f}')
            else:
                spread = 'n/a'
            lines.append(
                f'  miss placement: target_newer={self.miss_target_newer} '
                f'target_inside={self.miss_target_inside} '
                f'target_older={self.miss_target_older} '
                f'empty_buffer={self.empty_buffer_misses}')
            lines.append(f'  nearest buffered stamp delta (ms): {spread}')
        return '\n'.join(lines)


def set_mask_estimator_status(
    detection,
    reason: MissReason,
    enabled=MASK_ESTIMATORS,
) -> None:
    """Stamp one miss reason on the enabled mask-estimator status fields.

    ``enabled`` is the run's mask-estimator subset; a path that was never asked
    to run keeps its ``UNSET`` status, because that is what happened -- the node
    did not miss, it never looked. Same shape as the legacy rows, which carry no
    status field at all and infer ``UNSET`` from a NaN distance.
    """

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
    scan_reason: MissReason = MissReason.NO_SCAN,
    beam_records: list | None = None,
    enabled=MASK_ESTIMATORS,
) -> None:
    """Run every enabled path for each (detection, mask) pair, in place.

    ``camera_rotation`` / ``camera_translation`` are the camera-optical ->
    base extrinsics (TF at the detection stamp). ``masks`` is index-aligned
    with ``batch.detections``; a ``None`` mask (already status-stamped by
    ``masks_for_batch``) skips that detection entirely -- its fields stay NaN
    and the trial drops, per the no-fallback convention. Each estimator's
    ``*_status`` records why it missed (or ``OK``); ``scan_reason`` is the
    reason polar carries when the scan itself never resolved. The
    ``tight | rect`` fork lives inside the paths themselves; this function is
    gate-agnostic.

    ``enabled`` is the run's mask-estimator subset. A path outside it is not
    run and not stamped, so its fields stay NaN and its status stays ``UNSET``
    -- the run simply has no such row, exactly as an unselected legacy
    estimator has none.

    ``depth_max`` is the working depth gate the two depth paths clean against
    (polar never sees it -- it reads the scan, which has its own clip). It is
    not a validity rule: how far a reading can be believed is the depth
    source's ``usable_max_m``, and the caller passes the tighter of the two.

    ``beam_records`` is an optional out-list collecting one ``PolarBeamRecord``
    per detection that had a scan, for visualization. Passing nothing keeps the
    previous behaviour exactly; the beams are a by-product of work already done,
    never a reason to run a path.
    """

    wants_projective = 'projective_ranging' in enabled
    wants_euclidean = 'euclidean_reconstruction' in enabled
    wants_polar = 'polar_profiling' in enabled

    for index, (detection, mask) in enumerate(zip(batch.detections, masks)):
        if mask is None:
            continue

        if depth_m is None:
            if wants_projective:
                detection.projective_ranging_status = int(MissReason.NO_DEPTH_FRAME)
            if wants_euclidean:
                detection.euclidean_reconstruction_status = int(MissReason.NO_DEPTH_FRAME)
        else:
            if wants_projective:
                result_a, reason_a = localize_projective_ranging(
                    depth_m, mask, intrinsics, isolation=isolation_2d,
                    depth_max=depth_max)
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
                result_b, reason_b = localize_euclidean_reconstruction(
                    depth_m, mask, intrinsics, isolation=isolation_3d,
                    depth_max=depth_max)
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
            result_c, reason_c = localize_polar_profiling(
                points_optical, valid, mask, intrinsics)
            detection.polar_profiling_status = int(reason_c)
            if result_c is not None:
                # Polar profiling recovers only (X, Z); Y is unobservable and
                # substituted with 0. Optical Y folds into base forward only
                # through the camera pitch, which is 0 for this benchmark, so
                # the substitution is exact here.
                x_optical, z_optical = (float(value) for value in result_c.xz_optical)
                (
                    detection.polar_profiling_lateral_m,
                    detection.polar_profiling_forward_m,
                    detection.polar_profiling_distance_m,
                ) = optical_to_base_planar(
                    (x_optical, 0.0, z_optical), camera_rotation,
                    camera_translation, front_offset_m)

            if beam_records is not None:
                # A miss is the case worth seeing, so record the beams either
                # way: on success the estimator's own two sets, on failure the
                # selection it rejected, with nothing marked as used.
                if result_c is not None:
                    selected, merged = result_c.selected_beams, result_c.merged_beams
                else:
                    selected = select_beams(points_optical, valid, mask, intrinsics)
                    merged = np.empty(0, dtype=np.intp)
                beam_records.append(PolarBeamRecord(
                    detection_index=index,
                    selected=selected,
                    merged=merged,
                    in_bbox=beams_in_bbox(
                        points_optical, valid, detection.bbox_xyxy, intrinsics),
                ))


def nearest_beam_record(batch, beam_records):
    """The record for the detection the ray layers speak for, or ``None``.

    Ranked by the same rule the estimate rings use, so both surfaces land on the
    same robot. This node only fills its own three estimators, so the canonical
    order falls through to them here -- two near-equal robots can still split the
    ray and ring layers for a frame, which is the cost of not coupling the nodes.

    Matched on ``detection_index`` rather than list position: detections whose
    segmentation came back empty never record beams, so the two diverge. A batch
    that ranks to nothing still draws its first record -- a frame with beams and
    no estimate is exactly the failure worth seeing.
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
    """Union of the frame's masks as a ``mono8`` Image (255 = object).

    The debug artifact for the overlay panel: the pixels downstream actually
    consumed this frame. ``None`` entries (empty segmentations) contribute
    nothing.
    """

    union = np.zeros((image_height, image_width), dtype=np.uint8)
    for mask in masks:
        if mask is not None:
            union[mask.data] = 255
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
    """Warning when the ``camera_info`` grid differs from the detection grid.

    The masks are rasterized on the detection grid, while the depth indexing
    and the scan projection are bounded by the ``camera_info`` grid. When the
    two differ the masks cannot index either, so the caller skips every path
    for the frame (fields stay NaN) and warns. Returns ``None`` when the
    grids match.
    """

    if (intrinsics.height, intrinsics.width) == (batch.image_height, batch.image_width):
        return None
    return (
        f'camera_info grid ({intrinsics.height}, {intrinsics.width}) does not '
        f'match the detection grid ({batch.image_height}, {batch.image_width}); '
        'masks cannot index the projection. Check the camera_info_topic parameter.'
    )


class G1MaskMeasurementNode(Node):
    def __init__(self, depth_source=None, **node_kwargs) -> None:
        # ``node_kwargs`` reaches rclpy's Node: tests pass
        # ``parameter_overrides`` to stand the node up on a chosen
        # configuration without a launch file or CLI arguments.
        super().__init__('g1_mask_measurement_node', **node_kwargs)

        self.declare_parameter('detections_topic', RAW_DETECTIONS_TOPIC)
        self.declare_parameter('measurement_topic', MASK_MEASUREMENTS_TOPIC)
        # Which of the three mask rows this run fills, same parameter name and
        # "all" default as g1_pointcloud_measurement_node. Non-mask keys
        # in the value are ignored (a run selects across both stacks with one
        # list); the paths not selected are never run, so their fields stay NaN
        # and their statuses stay UNSET.
        self.declare_parameter('enabled_estimators', 'all')
        self.declare_parameter('depth_source', DEPTH_SOURCE_STEREOSCOPIC)
        # The stereo source's input. On a real D435 this must be the driver's
        # ``aligned_depth_to_color`` stream: the source converts units, it does
        # not align. Unused when depth_source is monocular.
        self.declare_parameter('depth_topic', DEPTH_TOPIC_DEFAULT)
        # The working depth gate for the two depth paths. Unbounded by
        # default, because the only ceiling a mask measurement needs is
        # whatever its depth source declares it can resolve.
        #
        # This gate is about how much of the scene to admit, not about whether
        # a reading is believable; that ceiling is the depth source's
        # ``usable_max_m`` and the two are combined in effective_depth_max().
        # Unbounded by default. It used to sit at 10 m, where it doubled as the
        # background suppressor keeping euclidean's percentile anchor inside
        # the regime that anchor is correct in -- load-bearing behaviour from a
        # constant nobody picked for that job, and the reason an object past
        # the gate came back as TOO_FEW_VALID_PIXELS, blaming the mask for a
        # range decision. The isolation default is mode-anchored now
        # (``foreground_isolation_3d.md`` Section 2b), so nothing depends on
        # the gate being tight.
        self.declare_parameter('depth_max_meters', DEPTH_GATE_DISABLED)
        self.declare_parameter('camera_info_topic', CAMERA_INFO_TOPIC_DEFAULT)
        self.declare_parameter('depth_anything_model_id', DEPTH_ANYTHING_MODEL_ID_DEFAULT)
        self.declare_parameter('depth_anything_device', '')
        self.declare_parameter('scan_topic', 'sensors/lidar2d_0/scan')
        self.declare_parameter('scan_match_tolerance_s', SCAN_MATCH_TOLERANCE_S_DEFAULT)
        self.declare_parameter('ray_marker_topic', RAY_MARKER_TOPIC)
        self.declare_parameter('ray_marker_lifetime_sec', RAY_MARKER_LIFETIME_SEC)
        self.declare_parameter('base_frame', BASE_FRAME_DEFAULT)
        self.declare_parameter('front_offset_m', ROBOT_FRONT_OFFSET_M_DEFAULT)
        self.declare_parameter('isolation_2d', ISOLATION_2D_DEFAULT)
        self.declare_parameter('isolation_3d', ISOLATION_3D_DEFAULT)
        # Height of the base origin above the floor, added to the TF
        # camera-above-base height to place the euclidean floor crop per frame.
        self.declare_parameter('base_above_floor_m', BASE_ABOVE_FLOOR_M_DEFAULT)
        self.declare_parameter('mask_gate', MASK_GATE_BOX)
        self.declare_parameter('color_topic', COLOR_TOPIC_DEFAULT)
        self.declare_parameter('color_buffer_depth', COLOR_BUFFER_DEPTH_DEFAULT)
        self.declare_parameter('segmentation_model', SEGMENTATION_MODEL_DEFAULT)
        # Silhouette confidence floor. Settable via --ros-args only for
        # now, same status as segmentation_model -- fold into the launch-arg
        # work later.
        self.declare_parameter(
            'segmentation_min_iou', SEGMENTATION_MIN_PREDICTED_IOU_DEFAULT)
        self.declare_parameter('mask_debug_topic', MASK_DEBUG_TOPIC)
        self.declare_parameter('aligned_depth_debug_topic', ALIGNED_DEPTH_DEBUG_TOPIC)
        # Periodic accounting of depth-input lookup hits and misses. Off by
        # default: it is a run-time investigation aid, not part of the pipeline.
        self.declare_parameter('depth_match_debug', False)
        self.declare_parameter('depth_match_debug_period_s', 5.0)

        self.enabled_estimators = resolve_enabled_estimators(
            self.get_parameter('enabled_estimators').value)
        # The two input axes the selection collapses: the depth paths need an
        # aligned depth frame (and so a depth source, its input stream, and the
        # debug republish), polar profiling needs the scan (and so the ray
        # markers). A run that selects only one side pays for only one side --
        # notably, polar-only under depth_source:=monocular never loads
        # Depth-Anything.
        self.needs_depth = bool(self.enabled_estimators & DEPTH_PATH_ESTIMATORS)
        self.needs_scan = 'polar_profiling' in self.enabled_estimators

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.scan_match_tolerance_s = float(
            self.get_parameter('scan_match_tolerance_s').value)
        self.ray_marker_lifetime_sec = float(
            self.get_parameter('ray_marker_lifetime_sec').value)
        self.front_offset_m = float(self.get_parameter('front_offset_m').value)
        self.isolation_2d = self.resolve_recipe(
            'isolation_2d', ISOLATION_2D_RECIPES)
        # The 3D recipe is rebuilt per frame with the TF-derived floor pose, so
        # store the validated name (not a pre-built callable) and the offset.
        self.isolation_3d_name = self.resolve_recipe_name(
            'isolation_3d', ISOLATION_3D_RECIPES)
        self.base_above_floor_m = float(
            self.get_parameter('base_above_floor_m').value)
        self.mask_gate = resolve_mask_gate(self.get_parameter('mask_gate').value)

        # How the aligned depth frame is obtained for a given stamp. The source
        # is pulled per detection batch rather than fed by a producer topic, so
        # the frame it converts is always the one the detections were made on.
        # ``None`` when no depth path is selected: the source names the stream
        # to subscribe to, so building one nothing would read is what drags in
        # the input topic (and, for the monocular source, the model).
        # Seam: tests inject a stub source (an ``input_kind`` and a scripted
        # ``produce``) so the node stands up without a model, mirroring
        # G1DetectorNode(detector=...).
        self.depth_source = None
        if self.needs_depth:
            self.depth_source = depth_source or build_depth_source(
                str(self.get_parameter('depth_source').value),
                self.get_logger(),
                model_id=str(self.get_parameter('depth_anything_model_id').value),
                device=str(self.get_parameter('depth_anything_device').value),
                # Node clock so the monocular cooldown respects use_sim_time.
                now_fn=lambda: self.get_clock().now().nanoseconds / 1e9,
            )
        self.depth_max_gate_m = resolve_depth_gate(
            self.get_parameter('depth_max_meters').value)

        # Two readers want the exact color frame the detections were made on:
        # the silhouette gate prompts the segmenter with it, and the monocular
        # depth source runs its network on it. One buffer serves both, matched
        # on the same stamp. A box-gated stereo run subscribes to neither the
        # color stream nor a model -- rasterization is model-free.
        self.color_buffer: StampedMessageBuffer | None = None
        self.segmenter: SamBoxSegmenter | None = None
        self.last_segmentation_log_monotonic = 0.0
        self.last_oversized_log_monotonic = 0.0
        self.segmentation_min_iou = 0.0
        if (self.mask_gate == MASK_GATE_SILHOUETTE
                or (self.depth_source is not None
                    and self.depth_source.input_kind == 'color')):
            self.color_buffer = StampedMessageBuffer(
                int(self.get_parameter('color_buffer_depth').value))
        if self.mask_gate == MASK_GATE_SILHOUETTE:
            self.segmentation_min_iou = float(
                self.get_parameter('segmentation_min_iou').value)
            self.segmenter = SamBoxSegmenter(
                str(self.get_parameter('segmentation_model').value),
                self.get_logger(),
            )
            # Load eagerly: a missing perception_venv fails at startup with
            # the actionable RuntimeError (caught in main), not per frame in
            # the worker.
            self.segmenter.load()

        # TF serves two extrinsics per frame: scan -> optical (polar profiling
        # projects the scan into the camera frame) and optical -> base (every
        # path's result is converted to the base-frame planar convention with
        # the camera's true mounting pose, translation included).
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.last_scan_tf_fallback: str | None = None
        self.last_base_tf_fallback: str | None = None

        self.latest_detections_msg: G1Detections | None = None
        # Depth input and scan are matched to the detection stamp, not
        # paired latest-wins, so they are buffered rather than kept as a single
        # slot. Depth = exact stamp; scan = nearest within scan_match_tolerance_s.
        # Buffered raw: receiving is cheap, and only the frame at the detection
        # stamp is ever converted.
        self.depth_buffer: StampedMessageBuffer | None = None
        if self.depth_source is not None and self.depth_source.input_kind == 'depth':
            self.depth_buffer = StampedMessageBuffer(DEPTH_MATCH_BUFFER_DEPTH)
        # Where the depth source's input lands: the depth stream for the stereo
        # source, the shared color buffer for the monocular one, and nothing at
        # all with no depth path selected -- a silhouette-gated polar-only run
        # has a color buffer for the segmenter that is not a depth input.
        # Chosen on ``is None``, not truthiness -- an empty buffer has len() == 0.
        self.depth_input_buffer: StampedMessageBuffer | None = None
        if self.needs_depth:
            self.depth_input_buffer = (
                self.color_buffer if self.depth_buffer is None else self.depth_buffer)
        self.scan_buffer: StampedMessageBuffer | None = None
        if self.needs_scan:
            self.scan_buffer = StampedMessageBuffer(SCAN_MATCH_BUFFER_DEPTH)
        self.latest_camera_info: CameraInfo | None = None
        self.processing_lock = threading.Lock()
        self.process_event = threading.Event()
        self.stop_event = threading.Event()
        self.last_skip_warning: str | None = None

        self.create_subscription(
            G1Detections,
            str(self.get_parameter('detections_topic').value),
            self.detections_callback,
            10,
        )
        if self.depth_buffer is not None:
            self.create_subscription(
                Image,
                str(self.get_parameter('depth_topic').value),
                self.depth_callback,
                qos_profile_sensor_data,
            )
        # Intrinsics of the color grid, which the aligned depth frame shares.
        # Static, so the latest-wins slot stays correct.
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        if self.scan_buffer is not None:
            self.create_subscription(
                LaserScan,
                str(self.get_parameter('scan_topic').value),
                self.scan_callback,
                qos_profile_sensor_data,
            )
        if self.color_buffer is not None:
            self.create_subscription(
                Image,
                str(self.get_parameter('color_topic').value),
                self.color_callback,
                qos_profile_sensor_data,
            )

        self.measurement_pub = self.create_publisher(
            G1Measurements,
            str(self.get_parameter('measurement_topic').value),
            10,
        )

        # Debug-only artifact (mask_component.md Section 5.3): the union of
        # the tight masks actually consumed this frame, so the overlay panel
        # can show exactly what downstream saw. Never consumed off the wire
        # by measurement code; only exists on the silhouette gate (the rect
        # union is derivable from the detections message).
        self.mask_debug_pub = None
        if self.mask_gate == MASK_GATE_SILHOUETTE:
            self.mask_debug_pub = self.create_publisher(
                Image,
                str(self.get_parameter('mask_debug_topic').value),
                qos_profile_sensor_data,
            )

        # The aligned depth frame the paths just read, for the overlay panel.
        # Debug-only, and detection-gated: nothing is produced on frames without
        # a detection, because nothing is converted on them either. Absent with
        # no depth path selected, so the overlay's aligned panel is dark rather
        # than fed by a topic nothing writes.
        self.aligned_depth_debug_pub = None
        if self.needs_depth:
            self.aligned_depth_debug_pub = self.create_publisher(
                Image,
                str(self.get_parameter('aligned_depth_debug_topic').value),
                qos_profile_sensor_data,
            )

        # Which beams polar profiling reduced, and the box they were drawn from,
        # as RViz markers. Published for every frame polar runs on: the three
        # layers are separate marker namespaces, so RViz's own per-namespace
        # checkboxes do the enabling and disabling without a round trip through
        # this node.
        self.ray_marker_pub = None
        if self.needs_scan:
            self.ray_marker_pub = self.create_publisher(
                MarkerArray,
                str(self.get_parameter('ray_marker_topic').value),
                10,
            )

        # Diagnostics timer runs on the executor, so it observes exactly the
        # thread that would be starved if reception is the problem. Only the
        # depth paths do a stamp lookup worth accounting for.
        self.depth_match_diagnostics: DepthMatchDiagnostics | None = None
        if bool(self.get_parameter('depth_match_debug').value):
            if self.needs_depth:
                self.depth_match_diagnostics = DepthMatchDiagnostics()
                self.create_timer(
                    float(self.get_parameter('depth_match_debug_period_s').value),
                    self.log_depth_match_diagnostics)
            else:
                self.get_logger().warn(
                    'depth_match_debug is set but no depth path is enabled; '
                    'there is no depth lookup to account for.')

        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.worker_thread.start()

    def log_depth_match_diagnostics(self) -> None:
        with self.processing_lock:
            summary = self.depth_match_diagnostics.summary()
        self.get_logger().info(summary)

    def effective_depth_max(self) -> float:
        """The depth cutoff to clean against: the tighter of gate and source ceiling.

        Resolved per batch rather than once at startup because the monocular
        source only learns its ceiling from the checkpoint config once the
        model finishes loading, which is lazy and retried after a failure. A
        source that declares no ceiling (the test stubs) contributes none and
        leaves the gate alone.
        """

        usable_max_m = getattr(self.depth_source, 'usable_max_m', float('inf'))
        return min(self.depth_max_gate_m, float(usable_max_m))

    def resolve_recipe(self, parameter_name: str, registry: dict):
        return registry[self.resolve_recipe_name(parameter_name, registry)]

    def resolve_recipe_name(self, parameter_name: str, registry: dict) -> str:
        key = str(self.get_parameter(parameter_name).value).strip()
        if key not in registry:
            supported = ', '.join(sorted(registry))
            raise ValueError(
                f'Unknown {parameter_name} recipe "{key}". Expected one of: {supported}')
        return key

    def detections_callback(self, detections_msg: G1Detections) -> None:
        with self.processing_lock:
            self.latest_detections_msg = detections_msg
        self.process_event.set()

    def depth_callback(self, depth_msg: Image) -> None:
        with self.processing_lock:
            self.depth_buffer.store(depth_msg)
            if self.depth_match_diagnostics is not None:
                self.depth_match_diagnostics.depth_rx += 1

    def scan_callback(self, scan_msg: LaserScan) -> None:
        with self.processing_lock:
            self.scan_buffer.store(scan_msg)

    def camera_info_callback(self, camera_info: CameraInfo) -> None:
        self.latest_camera_info = camera_info

    def color_callback(self, color_msg: Image) -> None:
        with self.processing_lock:
            self.color_buffer.store(color_msg)
            if self.depth_match_diagnostics is not None:
                self.depth_match_diagnostics.color_rx += 1

    def processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.process_event.wait(timeout=0.1):
                continue
            self.process_event.clear()

            while not self.stop_event.is_set():
                with self.processing_lock:
                    detections_msg = self.latest_detections_msg
                    self.latest_detections_msg = None
                    depth_input_msg = None
                    scan_msg = None
                    if detections_msg is not None:
                        # Match the mask's own instant: the depth source's input
                        # shares the color stamp (exact), the free-running scan
                        # matches the nearest within tolerance. A miss -> None ->
                        # that input's paths skip.
                        stamp = detections_msg.header.stamp
                        if self.depth_input_buffer is not None:
                            depth_input_msg = self.depth_input_buffer.lookup(stamp)
                            if self.depth_match_diagnostics is not None:
                                sec, nanosec = stamp_key(stamp)
                                self.depth_match_diagnostics.record_lookup(
                                    depth_input_msg is not None,
                                    sec * 1_000_000_000 + nanosec,
                                    self.depth_input_buffer.stamps_ns())
                        if self.scan_buffer is not None:
                            scan_msg = self.scan_buffer.lookup_nearest(
                                stamp, self.scan_match_tolerance_s)

                if detections_msg is None:
                    break

                try:
                    self.process_measurements(
                        detections_msg, depth_input_msg, scan_msg,
                        self.latest_camera_info)
                except Exception:  # noqa: BLE001 - worker must survive any frame
                    self.get_logger().error(
                        'Mask measurement frame failed:\n' + traceback.format_exc())

                if not self.process_event.is_set():
                    break
                self.process_event.clear()

    def process_measurements(
        self,
        detections_msg: G1Detections,
        depth_input_msg: Image | None,
        scan_msg: LaserScan | None,
        camera_info: CameraInfo | None,
    ) -> None:
        """Publish one measurements message per detections message.

        Always publishes -- when a path's input (the depth source's frame,
        scan, or intrinsics) is not available yet its fields stay NaN, which downstream
        reads as "no estimate" (same convention as the other measurement
        nodes). The depth paths and polar profiling are independent: either can
        fill while the other's input is missing. A ``camera_info`` grid that
        does not match the detection grid skips every path for the frame --
        the masks cannot index the projection.
        """

        batch = batch_from_detections_message(detections_msg)

        if batch.detected and camera_info is not None:
            intrinsics = intrinsics_from_camera_info(camera_info)
            warning = grid_mismatch_warning(intrinsics, batch)
            if warning is not None:
                self.log_skip_warning(warning)
                self.stamp_frame_reason(batch, MissReason.GRID_MISMATCH)
            else:
                # Extrinsic before masks: a TF miss skips (and never overwrites)
                # the per-detection mask statuses, and spares the segmenter a
                # forward pass on a frame no path could use.
                camera_extrinsic = self.camera_extrinsic_for_batch(detections_msg)
                if camera_extrinsic is None:
                    self.stamp_frame_reason(batch, MissReason.TF_MISS_EXTRINSIC)
                else:
                    masks = self.masks_for_batch(detections_msg, batch)
                    if masks is None:
                        self.stamp_frame_reason(batch, MissReason.NO_COLOR_FRAME)
                    else:
                        if self.mask_debug_pub is not None:
                            self.mask_debug_pub.publish(encode_mask_debug_image(
                                masks, batch.image_height, batch.image_width,
                                detections_msg.header))
                        camera_rotation, camera_translation = camera_extrinsic
                        # The euclidean floor crop tracks the live mount: derive
                        # its height/pitch from the same extrinsic and build the
                        # recipe for this frame.
                        camera_height_m, camera_pitch_deg = camera_floor_geometry(
                            camera_rotation, camera_translation,
                            self.base_above_floor_m)
                        isolation_3d = build_isolation_3d(
                            self.isolation_3d_name, camera_height_m, camera_pitch_deg)
                        depth_m = self.depth_for_batch(depth_input_msg, batch)
                        self.publish_aligned_depth_debug(
                            depth_m, detections_msg.header)
                        scan_points, scan_reason = self.scan_points_for_batch(
                            detections_msg, scan_msg)
                        beam_records: list[PolarBeamRecord] = []
                        fill_path_measurements(
                            batch,
                            masks,
                            intrinsics,
                            depth_m,
                            scan_points,
                            camera_rotation=camera_rotation,
                            camera_translation=camera_translation,
                            front_offset_m=self.front_offset_m,
                            isolation_2d=self.isolation_2d,
                            isolation_3d=isolation_3d,
                            depth_max=self.effective_depth_max(),
                            scan_reason=scan_reason,
                            beam_records=beam_records,
                            enabled=self.enabled_estimators,
                        )
                        self.publish_ray_markers(
                            nearest_beam_record(batch, beam_records), scan_msg)
        elif batch.detected:
            self.log_skip_warning(
                'No camera_info received yet; publishing measurements without '
                'path estimates.'
            )
            self.stamp_frame_reason(batch, MissReason.NO_CAMERA_INFO)

        self.measurement_pub.publish(
            build_measurements_message(batch, detections_msg.header))

    def publish_ray_markers(self, beam_record, scan_msg) -> None:
        """Draw the nearest detection's polar beams, given a scan to draw from.

        Stamped from the scan rather than the detection so the markers carry the
        stamp of the data they depict; the two are matched to within
        ``scan_match_tolerance_s`` and RViz interpolates the transform.
        """

        if self.ray_marker_pub is None or scan_msg is None or beam_record is None:
            return
        markers = build_polar_ray_markers(
            scan_msg,
            beam_record,
            scan_msg.header.stamp,
            Duration(seconds=self.ray_marker_lifetime_sec).to_msg(),
        )
        if markers:
            self.ray_marker_pub.publish(MarkerArray(markers=markers))

    def stamp_frame_reason(self, batch, reason: MissReason) -> None:
        """Stamp a frame-level miss reason on the enabled mask estimators of
        every detection, so a frame that short-circuits before path work still
        reports why rather than a bare ``UNSET``."""

        for detection in batch.detections:
            set_mask_estimator_status(detection, reason, self.enabled_estimators)

    def masks_for_batch(self, detections_msg: G1Detections, batch) -> list | None:
        """One mask per detection for the configured gate, or ``None``.

        ``box``: rasterize each detection box (never fails). ``silhouette``:
        prompt the segmenter with all boxes on the stamp-matched color frame
        -- one forward per frame, one decode per box. A missing color frame
        returns ``None`` (skip the frame's paths, never downgrade to rect --
        the run *is* the gate axis); an empty segmentation yields a ``None``
        entry for that detection only. Detections whose box covers more than
        ``MAX_BOX_FRAME_FRACTION`` of the frame are gated to a ``None`` entry
        before masking, so a runaway detector box is never rasterized
        or segmented into the background wall.
        """

        accepted = [
            box_within_frame_fraction(
                detection.bbox_xyxy, batch.image_height, batch.image_width)
            for detection in batch.detections
        ]
        self.log_oversized_skip(accepted.count(False))

        if self.mask_gate == MASK_GATE_BOX:
            box_masks: list = []
            for detection, keep in zip(batch.detections, accepted):
                if keep:
                    box_masks.append(rasterize_detection(
                        detection, batch.image_height, batch.image_width))
                else:
                    set_mask_estimator_status(
                        detection, MissReason.MASK_OVERSIZED_BOX,
                        self.enabled_estimators)
                    box_masks.append(None)
            return box_masks

        with self.processing_lock:
            color_msg = self.color_buffer.lookup(detections_msg.header.stamp)
        if color_msg is None:
            self.log_skip_warning(
                'Silhouette gate: no color frame buffered for the detection '
                'stamp (aged out or not arrived); skipping paths for this '
                'frame. Check the color_topic parameter and buffer depth.'
            )
            return None

        # The segmenter wants RGB; the shared decoder returns BGR. Only accepted
        # boxes are prompted; oversized ones map straight to ``None``.
        rgb = np.ascontiguousarray(
            convert_color_image_message(color_msg)[:, :, ::-1])
        prompt_boxes = [
            detection.bbox_xyxy
            for detection, keep in zip(batch.detections, accepted)
            if keep
        ]
        blobs: list = []
        if prompt_boxes:
            started = time.perf_counter()
            blobs = self.segmenter.segment_boxes(
                rgb, prompt_boxes, min_predicted_iou=self.segmentation_min_iou)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self.log_segmentation_latency(elapsed_ms, len(blobs))

        # Re-align the segmenter's per-prompt blobs back to full detection order.
        blob_iter = iter(blobs)
        masks: list = []
        for detection, keep in zip(batch.detections, accepted):
            if not keep:
                set_mask_estimator_status(
                    detection, MissReason.MASK_OVERSIZED_BOX, self.enabled_estimators)
                masks.append(None)
                continue
            blob = next(blob_iter)
            if blob is None:
                set_mask_estimator_status(
                    detection, MissReason.MASK_EMPTY_SEGMENTATION,
                    self.enabled_estimators)
                masks.append(None)
            else:
                masks.append(mask_from_array(blob, MaskPrecision.TIGHT))
        return masks

    def log_segmentation_latency(self, elapsed_ms: float, mask_count: int) -> None:
        now = time.monotonic()
        if now - self.last_segmentation_log_monotonic < 5.0:
            return
        self.last_segmentation_log_monotonic = now
        self.get_logger().info(
            f'Silhouette segmentation: {elapsed_ms:.1f} ms for {mask_count} mask(s).')

    def log_oversized_skip(self, count: int) -> None:
        """Warn (throttled) that oversized detector boxes were gated out."""

        if count <= 0:
            return
        now = time.monotonic()
        if now - self.last_oversized_log_monotonic < 5.0:
            return
        self.last_oversized_log_monotonic = now
        self.get_logger().warn(
            f'Skipping {count} detection(s) whose box exceeds '
            f'{int(MAX_BOX_FRAME_FRACTION * 100)}% of the frame (likely a '
            'detector failure); their path estimates stay unset.')

    def depth_for_batch(self, depth_input_msg: Image | None, batch):
        """Aligned depth in meters on the batch grid, or ``None`` if unusable.

        ``depth_input_msg`` is the depth source's own input at the detection
        stamp -- a raw camera depth frame for the stereo source, the color
        frame for the monocular one -- so the conversion (a unit decode, or a
        network forward pass) happens here, once, for the frame the detections
        were made on.

        Every "no usable depth at this stamp" outcome funnels through ``None``,
        which the caller stamps as ``NO_DEPTH_FRAME``: nothing buffered at the
        stamp, an unsupported encoding, a monocular model that is unavailable
        or mid-cooldown, or a grid the masks cannot index.
        """

        if depth_input_msg is None:
            return None
        try:
            frame = self.depth_source.produce(depth_input_msg)
        except ValueError as exc:
            self.log_skip_warning(f'Aligned depth frame skipped: {exc}')
            return None
        if frame is None:
            return None
        depth_m, _ = frame
        if depth_m.shape != (batch.image_height, batch.image_width):
            self.log_skip_warning(
                f'Aligned depth grid {depth_m.shape} does not match the '
                f'detection grid ({batch.image_height}, {batch.image_width}); '
                'masks cannot index it. Check the depth_topic parameter.'
            )
            return None
        return depth_m

    def publish_aligned_depth_debug(self, depth_m, header) -> None:
        """Republish the aligned depth frame the paths just read.

        Debug-only artifact for the overlay panel; no measurement code consumes
        it off the wire. The encode is skipped when nobody is subscribed, so a
        headless benchmark run pays nothing for it.
        """

        if depth_m is None or self.aligned_depth_debug_pub is None:
            return
        if self.aligned_depth_debug_pub.get_subscription_count() <= 0:
            return
        self.aligned_depth_debug_pub.publish(encode_depth_message(depth_m, header))

    def camera_extrinsic_for_batch(self, detections_msg: G1Detections):
        """Camera-optical -> base ``(rotation, translation)`` from TF, or ``None``.

        Looked up at the detection stamp for the frame the detections (and
        masks) live in. Without it the paths cannot be expressed in the base
        planar convention, so the caller skips every path for the frame --
        fields stay NaN, never a camera-at-origin approximation.
        """

        try:
            rotation, translation, self.last_base_tf_fallback = lookup_transform_components(
                self.tf_buffer,
                self.base_frame,
                detections_msg.header.frame_id,
                Time.from_msg(detections_msg.header.stamp),
                self.get_logger(),
                self.last_base_tf_fallback,
            )
        except TransformException as exc:
            self.log_skip_warning(
                f'Camera -> base extrinsic unavailable (TF): {exc}; '
                'skipping path estimates for this frame.')
            return None
        return rotation, translation

    def scan_points_for_batch(self, detections_msg: G1Detections, scan_msg: LaserScan | None):
        """Scan in the camera optical frame ``(points, valid)``, or ``None``.

        Looks up the scan -> optical extrinsic from TF at the detection stamp
        (the frame the masks live in) and projects the polar scan into it. A
        missing scan, an unavailable transform, or an empty scan yields
        ``None`` -- polar profiling is simply skipped for that frame.
        """

        if scan_msg is None:
            return None, MissReason.NO_SCAN
        try:
            rotation, translation, self.last_scan_tf_fallback = lookup_transform_components(
                self.tf_buffer,
                detections_msg.header.frame_id,
                scan_msg.header.frame_id,
                Time.from_msg(detections_msg.header.stamp),
                self.get_logger(),
                self.last_scan_tf_fallback,
            )
        except TransformException as exc:
            self.log_skip_warning(f'Polar profiling scan skipped (TF): {exc}')
            return None, MissReason.TF_MISS_SCAN
        try:
            return scan_points_optical(scan_msg, rotation, translation), MissReason.OK
        except ValueError as exc:
            self.log_skip_warning(f'Polar profiling scan skipped: {exc}')
            return None, MissReason.SCAN_INVALID

    def log_skip_warning(self, warning: str) -> None:
        if warning == self.last_skip_warning:
            return
        self.last_skip_warning = warning
        self.get_logger().warn(warning)

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.process_event.set()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    try:
        node = G1MaskMeasurementNode()
    except RuntimeError as exc:
        # Silhouette gate without the venv (no transformers/torch). Fail
        # cleanly with a clear message instead of dumping a traceback --
        # same pattern as g1_detector_node.
        get_logger('g1_mask_measurement').fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
