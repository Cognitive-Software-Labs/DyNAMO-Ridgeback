#!/usr/bin/env python3
"""Mask-based measurement node: the projective ranging / euclidean
reconstruction / polar profiling benchmark rows.

Consumes detections plus the aligned depth frame (``aligned_depth_node``) and
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
runner can merge them into the same aligned event as the camera and lidar
measurements.

The mask front-end is the ``mask_gate`` parameter: ``box`` rasterizes each
detection box into a ``rect`` mask (no model, no extra input); ``silhouette``
prompts a segmentation model (``perception/core/segmentation.py``) with the
boxes on the exact color frame the detections were made on, producing
``tight`` masks. Masks never cross the wire either way
(``mask_component.md`` Section 5.3) -- the segmenter runs in this process.

The depth source (stereoscopic vs monocular) is whatever ``aligned_depth_node``
was configured to produce; this node never branches on it. Polar profiling
needs no depth frame -- only the scan, the mask, and the color-grid
intrinsics -- so it runs independently of depth availability. Deliberately
independent of the legacy estimator stack (``geometry.py`` /
``g1_camera_measurement_node``): constants are mirrored by value, never
imported.
"""

from __future__ import annotations

import math
import threading
import time
import traceback
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from ridgeback_autonomy.common.messages import (
    batch_from_detections_message,
    build_measurements_message,
)
from ridgeback_autonomy.common.tf_utils import lookup_transform_components
from ridgeback_autonomy.msg import G1Detections, G1Measurements
from ridgeback_autonomy.perception.aligned_depth_node import (
    ALIGNED_CAMERA_INFO_TOPIC,
    ALIGNED_DEPTH_TOPIC,
    decode_depth_to_meters,
)
from ridgeback_autonomy.perception.core.image_utils import convert_color_image_message
from ridgeback_autonomy.perception.core.intrinsics import intrinsics_from_camera_info
from ridgeback_autonomy.perception.core.isolation_2d import (
    ISOLATION_2D_DEFAULT,
    ISOLATION_2D_RECIPES,
)
from ridgeback_autonomy.perception.core.isolation_3d import (
    ISOLATION_3D_DEFAULT,
    ISOLATION_3D_RECIPES,
)
from ridgeback_autonomy.perception.core.mask import (
    MaskPrecision,
    mask_from_array,
    rasterize_detection,
)
from ridgeback_autonomy.perception.core.projective_ranging import localize_projective_ranging
from ridgeback_autonomy.perception.core.euclidean_reconstruction import localize_euclidean_reconstruction
from ridgeback_autonomy.perception.core.polar_profiling import (
    localize_polar_profiling,
    scan_points_optical,
)
from ridgeback_autonomy.perception.core.segmentation import (
    SEGMENTATION_MODEL_DEFAULT,
    SamBoxSegmenter,
)


RAW_DETECTIONS_TOPIC = 'detections/g1/raw'
MASK_MEASUREMENTS_TOPIC = 'measurements/g1/mask'
COLOR_TOPIC_DEFAULT = 'sensors/camera_0/color/image'
MASK_DEBUG_TOPIC = 'debug/g1/mask'

ROBOT_FRONT_OFFSET_M_DEFAULT = 0.25  # mirrors geometry.ROBOT_FRONT_OFFSET_M
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
# against the room -- audit C6, the 2026-07-17 pos_003/024/032 outlier trials.
MAX_BOX_FRAME_FRACTION = 0.60

# Aligned-depth frames and scans are matched to the detection stamp, not paired
# latest-wins, so the mask and the depth/scan it reads come from the same
# instant (audit C3). Depth inherits the color frame's stamp (exact match); the
# scan free-runs at ~40 Hz, so it is matched to the nearest buffered stamp
# within SCAN_MATCH_TOLERANCE_S. Buffer depths span the detector latency
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

    Used for the silhouette color frame, the aligned depth frame, and the scan.
    Color and depth inherit the exact color stamp, so they match with
    ``lookup`` (no tolerance); the scan free-runs, so it matches with
    ``lookup_nearest`` within a tolerance window. A miss means the message aged
    out (or never arrived); the caller skips that source's paths rather than
    pairing whatever arrived most recently, which would smear distance under
    motion (audit C3) or mislabel the silhouette benchmark row.
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
) -> None:
    """Run every available path for each (detection, mask) pair, in place.

    ``camera_rotation`` / ``camera_translation`` are the camera-optical ->
    base extrinsics (TF at the detection stamp). ``masks`` is index-aligned
    with ``batch.detections``; a ``None`` mask (empty segmentation) skips that
    detection entirely -- its fields stay NaN and the trial drops, per the
    no-fallback convention. The ``tight | rect`` fork lives inside the paths
    themselves; this function is gate-agnostic.
    """

    for detection, mask in zip(batch.detections, masks):
        if mask is None:
            continue

        if depth_m is not None:
            result_a = localize_projective_ranging(
                depth_m, mask, intrinsics, isolation=isolation_2d)
            if result_a is not None:
                (
                    detection.projective_ranging_lateral_m,
                    detection.projective_ranging_forward_m,
                    detection.projective_ranging_distance_m,
                ) = optical_to_base_planar(
                    result_a.xyz_optical, camera_rotation, camera_translation,
                    front_offset_m)

            result_b = localize_euclidean_reconstruction(
                depth_m, mask, intrinsics, isolation=isolation_3d)
            if result_b is not None:
                (
                    detection.euclidean_reconstruction_lateral_m,
                    detection.euclidean_reconstruction_forward_m,
                    detection.euclidean_reconstruction_distance_m,
                ) = optical_to_base_planar(
                    result_b.xyz_optical, camera_rotation, camera_translation,
                    front_offset_m)

        if scan_points is not None:
            points_optical, valid = scan_points
            result_c = localize_polar_profiling(
                points_optical, valid, mask, intrinsics)
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
        'masks cannot index the projection. Check the aligned camera_info topic.'
    )


class G1MaskMeasurementNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_mask_measurement_node')

        self.declare_parameter('detections_topic', RAW_DETECTIONS_TOPIC)
        self.declare_parameter('measurement_topic', MASK_MEASUREMENTS_TOPIC)
        self.declare_parameter('aligned_depth_topic', ALIGNED_DEPTH_TOPIC)
        self.declare_parameter('aligned_camera_info_topic', ALIGNED_CAMERA_INFO_TOPIC)
        self.declare_parameter('scan_topic', 'sensors/lidar2d_0/scan')
        self.declare_parameter('scan_match_tolerance_s', SCAN_MATCH_TOLERANCE_S_DEFAULT)
        self.declare_parameter('base_frame', BASE_FRAME_DEFAULT)
        self.declare_parameter('front_offset_m', ROBOT_FRONT_OFFSET_M_DEFAULT)
        self.declare_parameter('isolation_2d', ISOLATION_2D_DEFAULT)
        self.declare_parameter('isolation_3d', ISOLATION_3D_DEFAULT)
        self.declare_parameter('mask_gate', MASK_GATE_BOX)
        self.declare_parameter('color_topic', COLOR_TOPIC_DEFAULT)
        self.declare_parameter('color_buffer_depth', COLOR_BUFFER_DEPTH_DEFAULT)
        self.declare_parameter('segmentation_model', SEGMENTATION_MODEL_DEFAULT)
        self.declare_parameter('mask_debug_topic', MASK_DEBUG_TOPIC)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.scan_match_tolerance_s = float(
            self.get_parameter('scan_match_tolerance_s').value)
        self.front_offset_m = float(self.get_parameter('front_offset_m').value)
        self.isolation_2d = self.resolve_recipe(
            'isolation_2d', ISOLATION_2D_RECIPES)
        self.isolation_3d = self.resolve_recipe(
            'isolation_3d', ISOLATION_3D_RECIPES)
        self.mask_gate = resolve_mask_gate(self.get_parameter('mask_gate').value)

        # The silhouette gate needs the exact color frame the detections were
        # made on, and the segmentation model. The box gate subscribes to
        # nothing extra and loads nothing -- rasterization is model-free.
        self.color_buffer: StampedMessageBuffer | None = None
        self.segmenter: SamBoxSegmenter | None = None
        self.last_segmentation_log_monotonic = 0.0
        self.last_oversized_log_monotonic = 0.0
        if self.mask_gate == MASK_GATE_SILHOUETTE:
            self.color_buffer = StampedMessageBuffer(
                int(self.get_parameter('color_buffer_depth').value))
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
        # Depth and scan are matched to the detection stamp (audit C3), not
        # paired latest-wins, so they are buffered rather than kept as a single
        # slot. Depth = exact stamp; scan = nearest within scan_match_tolerance_s.
        self.depth_buffer = StampedMessageBuffer(DEPTH_MATCH_BUFFER_DEPTH)
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
        self.create_subscription(
            Image,
            str(self.get_parameter('aligned_depth_topic').value),
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('aligned_camera_info_topic').value),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self.scan_callback,
            qos_profile_sensor_data,
        )
        if self.mask_gate == MASK_GATE_SILHOUETTE:
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

        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.worker_thread.start()

    def resolve_recipe(self, parameter_name: str, registry: dict):
        key = str(self.get_parameter(parameter_name).value).strip()
        if key not in registry:
            supported = ', '.join(sorted(registry))
            raise ValueError(
                f'Unknown {parameter_name} recipe "{key}". Expected one of: {supported}')
        return registry[key]

    def detections_callback(self, detections_msg: G1Detections) -> None:
        with self.processing_lock:
            self.latest_detections_msg = detections_msg
        self.process_event.set()

    def depth_callback(self, depth_msg: Image) -> None:
        with self.processing_lock:
            self.depth_buffer.store(depth_msg)

    def scan_callback(self, scan_msg: LaserScan) -> None:
        with self.processing_lock:
            self.scan_buffer.store(scan_msg)

    def camera_info_callback(self, camera_info: CameraInfo) -> None:
        self.latest_camera_info = camera_info

    def color_callback(self, color_msg: Image) -> None:
        with self.processing_lock:
            self.color_buffer.store(color_msg)

    def processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.process_event.wait(timeout=0.1):
                continue
            self.process_event.clear()

            while not self.stop_event.is_set():
                with self.processing_lock:
                    detections_msg = self.latest_detections_msg
                    self.latest_detections_msg = None
                    depth_msg = None
                    scan_msg = None
                    if detections_msg is not None:
                        # Match the mask's own instant: depth inherits the color
                        # stamp (exact), the free-running scan the nearest within
                        # tolerance. A miss -> None -> that source's paths skip.
                        stamp = detections_msg.header.stamp
                        depth_msg = self.depth_buffer.lookup(stamp)
                        scan_msg = self.scan_buffer.lookup_nearest(
                            stamp, self.scan_match_tolerance_s)

                if detections_msg is None:
                    break

                try:
                    self.process_measurements(
                        detections_msg, depth_msg, scan_msg, self.latest_camera_info)
                except Exception:  # noqa: BLE001 - worker must survive any frame
                    self.get_logger().error(
                        'Mask measurement frame failed:\n' + traceback.format_exc())

                if not self.process_event.is_set():
                    break
                self.process_event.clear()

    def process_measurements(
        self,
        detections_msg: G1Detections,
        depth_msg: Image | None,
        scan_msg: LaserScan | None,
        camera_info: CameraInfo | None,
    ) -> None:
        """Publish one measurements message per detections message.

        Always publishes -- when a path's input (aligned depth, scan, or
        intrinsics) is not available yet its fields stay NaN, which downstream
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
            else:
                masks = self.masks_for_batch(detections_msg, batch)
                if masks is not None:
                    if self.mask_debug_pub is not None:
                        self.mask_debug_pub.publish(encode_mask_debug_image(
                            masks, batch.image_height, batch.image_width,
                            detections_msg.header))
                    camera_extrinsic = self.camera_extrinsic_for_batch(detections_msg)
                    if camera_extrinsic is not None:
                        camera_rotation, camera_translation = camera_extrinsic
                        depth_m = self.decode_depth_for_batch(depth_msg, batch)
                        scan_points = self.scan_points_for_batch(detections_msg, scan_msg)
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
                            isolation_3d=self.isolation_3d,
                        )
        elif batch.detected:
            self.log_skip_warning(
                'No camera_info received yet; publishing measurements without '
                'path estimates.'
            )

        self.measurement_pub.publish(
            build_measurements_message(batch, detections_msg.header))

    def masks_for_batch(self, detections_msg: G1Detections, batch) -> list | None:
        """One mask per detection for the configured gate, or ``None``.

        ``box``: rasterize each detection box (never fails). ``silhouette``:
        prompt the segmenter with all boxes on the stamp-matched color frame
        -- one forward per frame, one decode per box. A missing color frame
        returns ``None`` (skip the frame's paths, never downgrade to rect --
        the run *is* the gate axis); an empty segmentation yields a ``None``
        entry for that detection only. Detections whose box covers more than
        ``MAX_BOX_FRAME_FRACTION`` of the frame are gated to a ``None`` entry
        before masking (audit C6), so a runaway detector box is never rasterized
        or segmented into the background wall.
        """

        accepted = [
            box_within_frame_fraction(
                detection.bbox_xyxy, batch.image_height, batch.image_width)
            for detection in batch.detections
        ]
        self.log_oversized_skip(accepted.count(False))

        if self.mask_gate == MASK_GATE_BOX:
            return [
                rasterize_detection(detection, batch.image_height, batch.image_width)
                if keep else None
                for detection, keep in zip(batch.detections, accepted)
            ]

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
            blobs = self.segmenter.segment_boxes(rgb, prompt_boxes)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self.log_segmentation_latency(elapsed_ms, len(blobs))

        # Re-align the segmenter's per-prompt blobs back to full detection order.
        blob_iter = iter(blobs)
        masks: list = []
        for keep in accepted:
            if not keep:
                masks.append(None)
                continue
            blob = next(blob_iter)
            masks.append(
                None if blob is None else mask_from_array(blob, MaskPrecision.TIGHT))
        return masks

    def log_segmentation_latency(self, elapsed_ms: float, mask_count: int) -> None:
        now = time.monotonic()
        if now - self.last_segmentation_log_monotonic < 5.0:
            return
        self.last_segmentation_log_monotonic = now
        self.get_logger().info(
            f'Silhouette segmentation: {elapsed_ms:.1f} ms for {mask_count} mask(s).')

    def log_oversized_skip(self, count: int) -> None:
        """Warn (throttled) that oversized detector boxes were gated out (C6)."""

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

    def decode_depth_for_batch(self, depth_msg: Image | None, batch):
        """Aligned depth in meters on the batch grid, or ``None`` if unusable."""

        if depth_msg is None:
            return None
        try:
            depth_m = decode_depth_to_meters(depth_msg)
        except ValueError as exc:
            self.log_skip_warning(f'Aligned depth frame skipped: {exc}')
            return None
        if depth_m.shape != (batch.image_height, batch.image_width):
            self.log_skip_warning(
                f'Aligned depth grid {depth_m.shape} does not match the '
                f'detection grid ({batch.image_height}, {batch.image_width}); '
                'masks cannot index it. Check the aligned_depth_node input topics.'
            )
            return None
        return depth_m

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
            return None
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
            return None
        try:
            return scan_points_optical(scan_msg, rotation, translation)
        except ValueError as exc:
            self.log_skip_warning(f'Polar profiling scan skipped: {exc}')
            return None

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
