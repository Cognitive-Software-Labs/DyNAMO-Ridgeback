#!/usr/bin/env python3
"""Mask-based measurement node: the projective ranging / euclidean
reconstruction / polar profiling benchmark rows.

Consumes detections plus the camera stream the depth source reads and
the 2D LiDAR scan, builds one mask per detection, and runs the localization
paths per mask: the two depth paths
(``target_localization/core/projective_ranging.py`` / ``euclidean_reconstruction.py``)
against the aligned depth frame, and polar profiling
(``target_localization/core/polar_profiling.py``) against the scan projected into the
camera optical frame via TF. Each path's result is converted from the camera
optical frame to the base-frame planar-distance convention every benchmark
row shares (ground truth included) through the optical -> base extrinsics
from TF -- the camera's mounting pose, translation included, is modeled
exactly -- and published as ``TargetMeasurements`` with
the identity fields of the source detections message -- so the benchmark
runner can merge them into the same aligned event as the pointcloud
measurements.

Which of the three rows a run fills is the ``enabled_estimators`` parameter,
the same name and ``all`` default ``target_pointcloud_measurement_node`` carries,
so one comma-separated list selects across both stacks. A path that
was not selected is never run: its fields stay NaN, its status stays ``UNSET``,
and the inputs only it needs are never subscribed to -- a polar-only run builds
no depth source at all (so ``depth_source:=monocular`` loads no model), and a
depth-only run never touches the scan.

The mask front-end is the ``mask_gate`` parameter: ``box`` rasterizes each
detection box into a ``rect`` mask (no model, no extra input); ``silhouette``
prompts a segmentation model (``target_localization/core/segmentation.py``) with the
boxes on the exact color frame the detections were made on, producing
``tight`` masks. Masks never cross the wire either way
(``docs/target_localization/mask_representation.md`` Section 5.3) -- the segmenter runs in this process.

The aligned depth frame is produced here, on demand, at the detection stamp:
the ``depth_source`` parameter picks a strategy from
``target_localization/core/depth_sources.py``, this node buffers that strategy's input
stream raw and converts only the frame the detections were made on. Every
frame is received and none is discarded blind, so the exact-stamp match is a
lookup into a buffer this process filled rather than an intersection with some
other process's thinning. Nothing downstream branches on which source ran.
Polar profiling needs no depth frame -- only the scan, the mask, and the
color-grid intrinsics -- so it runs independently of depth availability.
Deliberately independent of the pointcloud estimator
(``target_localization/core/pointcloud_ranging.py``): shared defaults come from
``core/ranging_defaults.py``, never from the sibling estimator.
"""

from __future__ import annotations

import importlib
import threading
import time
import traceback

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

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    DEPTH_PATH_ESTIMATORS,
    MASK_GATE_BOX,
    MASK_GATE_SILHOUETTE,
)
from ridgeback_autonomy.common.markers import PolarBeamRecord, build_polar_ray_markers
from ridgeback_autonomy.common.messages import (
    batch_from_detections_message,
    build_measurements_message,
)
from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.common.stamps import stamp_key
from ridgeback_autonomy.common.tf_utils import lookup_transform_components
from ridgeback_autonomy.msg import TargetDetections, TargetMeasurements
from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    DEPTH_GATE_DISABLED,
    resolve_depth_gate,
)
from ridgeback_autonomy.perception.target_localization.core.depth_sources import (
    DEPTH_ANYTHING_MODEL_ID_DEFAULT,
    DEPTH_SOURCE_STEREOSCOPIC,
    build_depth_source,
    encode_depth_message,
)
from ridgeback_autonomy.perception.target_localization.core.image_utils import decode_color_to_rgb
from ridgeback_autonomy.perception.target_localization.core.intrinsics import intrinsics_from_camera_info
from ridgeback_autonomy.perception.target_localization.core.isolation_2d import (
    ISOLATION_2D_DEFAULT,
    ISOLATION_2D_RECIPES,
)
from ridgeback_autonomy.perception.target_localization.core.isolation_3d import (
    BASE_ABOVE_FLOOR_M_DEFAULT,
    ISOLATION_3D_DEFAULT,
    ISOLATION_3D_RECIPES,
    build_isolation_3d,
    camera_floor_geometry,
)
from ridgeback_autonomy.perception.target_localization.core.mask import (
    MaskPrecision,
    region_from_blob,
    region_from_detection,
)
from ridgeback_autonomy.perception.target_localization.core.polar_profiling import (
    scan_points_optical,
)
from ridgeback_autonomy.perception.target_localization.core.segmentation import (
    SEGMENTATION_MIN_PREDICTED_IOU_DEFAULT,
    SEGMENTATION_MODEL_DEFAULT,
    SamBoxSegmenter,
)
from ridgeback_autonomy.perception.target_localization.core.vehicle_frame import (
    ROBOT_FRONT_OFFSET_M,
)
from ridgeback_autonomy.perception.target_localization.contracts import (
    ALIGNED_DEPTH_DEBUG_TOPIC,
    MASK_DEBUG_TOPIC,
    MASK_MEASUREMENTS_TOPIC,
    POLAR_RAYS_TOPIC,
    RAW_DETECTIONS_TOPIC,
)
from ridgeback_autonomy.perception.target_localization.measurement_pipeline import (
    MAX_BOX_FRAME_FRACTION,
    box_within_frame_fraction,
    encode_mask_debug_image,
    fill_path_measurements,
    grid_mismatch_warning,
    nearest_beam_record,
    optical_to_base_planar,
    resolve_enabled_estimators,
    resolve_mask_gate,
    set_mask_estimator_status,
)
from ridgeback_autonomy.perception.target_localization.synchronization import (
    COLOR_BUFFER_DEPTH_DEFAULT,
    DEPTH_MATCH_BUFFER_DEPTH,
    SCAN_MATCH_BUFFER_DEPTH,
    SCAN_MATCH_TOLERANCE_S_DEFAULT,
    DepthMatchDiagnostics,
    PreparedColorFrame,
    StampedMessageBuffer,
)


COLOR_TOPIC_DEFAULT = 'sensors/camera_0/color/image'
DEPTH_TOPIC_DEFAULT = 'sensors/camera_0/depth/image'
CAMERA_INFO_TOPIC_DEFAULT = 'sensors/camera_0/color/camera_info'
RAY_MARKER_LIFETIME_SEC = 1.5
BASE_FRAME_DEFAULT = 'base_link'


class TargetMaskMeasurementNode(Node):
    def __init__(self, depth_source=None, **node_kwargs) -> None:
        # ``node_kwargs`` reaches rclpy's Node: tests pass
        # ``parameter_overrides`` to stand the node up on a chosen
        # configuration without a launch file or CLI arguments.
        super().__init__('target_mask_measurement_node', **node_kwargs)

        self.declare_parameter('detections_topic', RAW_DETECTIONS_TOPIC)
        self.declare_parameter('measurement_topic', MASK_MEASUREMENTS_TOPIC)
        # Which of the three mask rows this run fills, same parameter name and
        # "all" default as target_pointcloud_measurement_node. Non-mask keys
        # in the value are ignored (a run selects across both stacks with one
        # list); the paths not selected are never run, so their fields stay NaN
        # and their statuses stay UNSET.
        self.declare_parameter('enabled_estimators', 'all')
        self.declare_parameter('depth_source', DEPTH_SOURCE_STEREOSCOPIC)
        # The stereo source's input. On hardware this must be the RealSense
        # driver's ``aligned_depth_to_color`` stream: the source converts
        # units, it does not align. Unused when depth_source is monocular.
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
        # (``docs/target_localization/euclidean_reconstruction.md`` Section 2.3), so nothing depends on
        # the gate being tight.
        self.declare_parameter('depth_max_meters', DEPTH_GATE_DISABLED)
        self.declare_parameter('camera_info_topic', CAMERA_INFO_TOPIC_DEFAULT)
        self.declare_parameter('depth_anything_model_id', DEPTH_ANYTHING_MODEL_ID_DEFAULT)
        self.declare_parameter('depth_anything_device', '')
        self.declare_parameter('scan_topic', 'sensors/lidar2d_0/scan')
        self.declare_parameter('scan_match_tolerance_s', SCAN_MATCH_TOLERANCE_S_DEFAULT)
        self.declare_parameter('ray_marker_topic', POLAR_RAYS_TOPIC)
        self.declare_parameter('ray_marker_lifetime_sec', RAY_MARKER_LIFETIME_SEC)
        self.declare_parameter('base_frame', BASE_FRAME_DEFAULT)
        self.declare_parameter('front_offset_m', ROBOT_FRONT_OFFSET_M)
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
        # Periodic accounting of depth-input delivery and bounded worker-stage
        # timing. Off by default: it is an investigation aid, not pipeline work.
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

        # This existing default-off switch now owns both exact-depth delivery
        # accounting and the bounded stage timings used by the model-concurrency
        # sweep. Construct it before model setup so eager SlimSAM load time can
        # be separated from warm per-batch work.
        self.depth_match_diagnostics: DepthMatchDiagnostics | None = None
        if bool(self.get_parameter('depth_match_debug').value):
            if self.needs_depth:
                self.depth_match_diagnostics = DepthMatchDiagnostics()
            else:
                self.get_logger().warn(
                    'depth_match_debug is set but no depth path is enabled; '
                    'there is no depth lookup or depth-path timing to account for.')

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
        # TargetDetectorNode(detector=...).
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
            load_started_ns = (
                time.monotonic_ns()
                if self.depth_match_diagnostics is not None else None)
            try:
                self.segmenter.load()
            finally:
                if load_started_ns is not None:
                    self.depth_match_diagnostics.record_stage(
                        'slimsam_load', time.monotonic_ns() - load_started_ns)

        # TF serves two extrinsics per frame: scan -> optical (polar profiling
        # projects the scan into the camera frame) and optical -> base (every
        # path's result is converted to the base-frame planar convention with
        # the camera's true mounting pose, translation included).
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.last_scan_tf_fallback: str | None = None
        self.last_base_tf_fallback: str | None = None

        self.latest_detections_msg: TargetDetections | None = None
        self.latest_detections_received_monotonic_ns: int | None = None
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
            TargetDetections,
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
            TargetMeasurements,
            str(self.get_parameter('measurement_topic').value),
            10,
        )

        # Debug-only artifact (docs/target_localization/mask_representation.md Section 5.3): the union of
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

        # The timer runs on the executor, so it observes exactly the thread
        # that would be starved if reception is the problem.
        if self.depth_match_diagnostics is not None:
            self.create_timer(
                float(self.get_parameter('depth_match_debug_period_s').value),
                self.log_depth_match_diagnostics)

        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.worker_thread.start()

    def log_depth_match_diagnostics(self) -> None:
        with self.processing_lock:
            summary = self.depth_match_diagnostics.summary()
        self.get_logger().info(summary)

    def record_stage_timing(self, name: str, started_ns: int | None) -> None:
        """Record one diagnostic stage without affecting the default-off path."""

        if started_ns is None or self.depth_match_diagnostics is None:
            return
        elapsed_ns = time.monotonic_ns() - started_ns
        with self.processing_lock:
            self.depth_match_diagnostics.record_stage(name, elapsed_ns)

    def synchronize_cuda_for_timing(self) -> None:
        """Make optional GPU stage timings honest, and account for the cost."""

        if self.depth_match_diagnostics is None:
            return
        started_ns = time.monotonic_ns()
        try:
            torch = importlib.import_module('torch')
            if torch.cuda.is_available():
                torch.cuda.synchronize()
        except Exception:  # noqa: BLE001 - diagnostics must never break inference
            # A CPU-only run or unavailable CUDA runtime needs no barrier. The
            # model call itself remains authoritative and will report failures.
            pass
        self.record_stage_timing('cuda_sync', started_ns)

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

    def detections_callback(self, detections_msg: TargetDetections) -> None:
        diagnostics = self.depth_match_diagnostics
        waiting_started_ns = (
            time.monotonic_ns() if diagnostics is not None else None)
        with self.processing_lock:
            acquired_ns = (
                time.monotonic_ns() if diagnostics is not None else None)
            replaced_pending = self.latest_detections_msg is not None
            self.latest_detections_msg = detections_msg
            self.latest_detections_received_monotonic_ns = acquired_ns
            if diagnostics is not None:
                diagnostics.record_detection_arrival(
                    replaced_pending=replaced_pending,
                    now_ns=acquired_ns,
                )
                diagnostics.record_lock_timing(
                    'detections_callback',
                    acquired_ns - waiting_started_ns,
                    time.monotonic_ns() - acquired_ns,
                )
        self.process_event.set()

    def depth_callback(self, depth_msg: Image) -> None:
        diagnostics = self.depth_match_diagnostics
        waiting_started_ns = (
            time.monotonic_ns() if diagnostics is not None else None)
        with self.processing_lock:
            acquired_ns = (
                time.monotonic_ns() if diagnostics is not None else None)
            self.depth_buffer.store(depth_msg)
            if diagnostics is not None:
                sec, nanosec = stamp_key(depth_msg.header.stamp)
                diagnostics.record_depth_arrival(
                    sec * 1_000_000_000 + nanosec,
                    now_ns=acquired_ns,
                )
                diagnostics.record_lock_timing(
                    'depth_callback',
                    acquired_ns - waiting_started_ns,
                    time.monotonic_ns() - acquired_ns,
                )

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
                diagnostics = self.depth_match_diagnostics
                waiting_started_ns = (
                    time.monotonic_ns() if diagnostics is not None else None)
                with self.processing_lock:
                    acquired_ns = (
                        time.monotonic_ns() if diagnostics is not None else None)
                    detections_msg = self.latest_detections_msg
                    self.latest_detections_msg = None
                    detections_received_ns = (
                        self.latest_detections_received_monotonic_ns)
                    self.latest_detections_received_monotonic_ns = None
                    depth_input_msg = None
                    scan_msg = None
                    if detections_msg is not None:
                        if (diagnostics is not None
                                and detections_received_ns is not None):
                            diagnostics.record_detection_dequeue(
                                acquired_ns - detections_received_ns)
                        # Match the mask's own instant: the depth source's input
                        # shares the color stamp (exact), the free-running scan
                        # matches the nearest within tolerance. A miss -> None ->
                        # that input's paths skip.
                        stamp = detections_msg.header.stamp
                        if self.depth_input_buffer is not None:
                            depth_input_msg = self.depth_input_buffer.lookup(stamp)
                            if diagnostics is not None:
                                sec, nanosec = stamp_key(stamp)
                                diagnostics.record_lookup(
                                    depth_input_msg is not None,
                                    sec * 1_000_000_000 + nanosec,
                                    self.depth_input_buffer.stamps_ns(),
                                    now_ns=time.monotonic_ns())
                        if self.scan_buffer is not None:
                            scan_msg = self.scan_buffer.lookup_nearest(
                                stamp, self.scan_match_tolerance_s)
                    if diagnostics is not None:
                        diagnostics.record_lock_timing(
                            'worker_snapshot',
                            acquired_ns - waiting_started_ns,
                            time.monotonic_ns() - acquired_ns,
                        )

                if detections_msg is None:
                    break

                processing_started_ns = (
                    time.monotonic_ns() if diagnostics is not None else None)
                stamp_sec, stamp_nanosec = stamp_key(detections_msg.header.stamp)
                target_stamp_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
                try:
                    self.process_measurements(
                        detections_msg, depth_input_msg, scan_msg,
                        self.latest_camera_info)
                    if diagnostics is not None:
                        completed_ns = time.monotonic_ns()
                        received_ns = (
                            detections_received_ns
                            if detections_received_ns is not None
                            else processing_started_ns)
                        with self.processing_lock:
                            diagnostics.record_batch_completed(
                                target_stamp_ns,
                                completed_ns - received_ns,
                            )
                except Exception:  # noqa: BLE001 - worker must survive any frame
                    if diagnostics is not None:
                        with self.processing_lock:
                            diagnostics.record_batch_failed(target_stamp_ns)
                    self.get_logger().error(
                        'Mask measurement frame failed:\n' + traceback.format_exc())
                finally:
                    if diagnostics is not None:
                        processing_elapsed_ns = (
                            time.monotonic_ns() - processing_started_ns)
                        with self.processing_lock:
                            diagnostics.record_worker_processing(
                                processing_elapsed_ns)

                if not self.process_event.is_set():
                    break
                self.process_event.clear()

    def process_measurements(
        self,
        detections_msg: TargetDetections,
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
                    color_hint = None
                    if (self.depth_source is not None
                            and self.depth_source.input_kind == 'color'):
                        color_hint = depth_input_msg
                    masks, prepared_color = self.masks_for_batch(
                        detections_msg, batch, color_hint=color_hint)
                    if masks is None:
                        self.stamp_frame_reason(batch, MissReason.NO_COLOR_FRAME)
                    else:
                        self.publish_mask_debug(
                            masks,
                            batch.image_height,
                            batch.image_width,
                            detections_msg.header,
                        )
                        camera_rotation, camera_translation = camera_extrinsic
                        # The euclidean floor crop tracks the live mount: derive
                        # its height/pitch from the same extrinsic and build the
                        # recipe for this frame.
                        camera_height_m, camera_pitch_deg = camera_floor_geometry(
                            camera_rotation, camera_translation,
                            self.base_above_floor_m)
                        isolation_3d = build_isolation_3d(
                            self.isolation_3d_name, camera_height_m, camera_pitch_deg)
                        depth_m = self.depth_for_batch(
                            depth_input_msg, batch, prepared_color=prepared_color)
                        self.publish_aligned_depth_debug(
                            depth_m, detections_msg.header)
                        scan_points, scan_reason = self.scan_points_for_batch(
                            detections_msg, scan_msg)
                        beam_records = self.ray_marker_records()
                        reduction_started_ns = (
                            time.monotonic_ns()
                            if self.depth_match_diagnostics is not None else None)
                        try:
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
                        finally:
                            self.record_stage_timing(
                                'estimator_reduction', reduction_started_ns)
                        if beam_records is not None:
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

    def ray_marker_records(self) -> list[PolarBeamRecord] | None:
        """Allocate debug records only while the ray topic has a subscriber."""

        if self.ray_marker_pub is None:
            return None
        if self.ray_marker_pub.get_subscription_count() <= 0:
            return None
        return []

    def publish_ray_markers(self, beam_record, scan_msg) -> None:
        """Draw the nearest detection's polar beams, given a scan to draw from.

        Stamped from the scan rather than the detection so the markers carry the
        stamp of the data they depict; the two are matched to within
        ``scan_match_tolerance_s`` and RViz interpolates the transform.
        """

        if self.ray_marker_pub is None or scan_msg is None or beam_record is None:
            return
        if self.ray_marker_pub.get_subscription_count() <= 0:
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

    def masks_for_batch(
        self,
        detections_msg: TargetDetections,
        batch,
        *,
        color_hint: Image | None = None,
    ) -> tuple[list | None, PreparedColorFrame | None]:
        """One mask region per detection plus an optional prepared RGB frame.

        ``box``: the detection box becomes its own window (never fails).
        ``silhouette``:
        prompt the segmenter with all boxes on the stamp-matched color frame
        -- one forward and one RGB preparation per frame. A missing color frame
        returns ``None`` (skip the frame's paths, never downgrade to rect --
        the run *is* the gate axis); an empty segmentation yields a ``None``
        entry for that detection only. Detections whose box covers more than
        ``MAX_BOX_FRAME_FRACTION`` of the frame are gated to a ``None`` entry
        before masking, so a runaway detector box is never rasterized
        or segmented into the background wall. A matching ``color_hint`` is
        reused without taking the color-buffer lock; otherwise this method
        retains the late exact-stamp buffer lookup.
        """

        accepted = [
            box_within_frame_fraction(
                detection.bbox_xyxy, batch.image_height, batch.image_width)
            for detection in batch.detections
        ]
        self.log_oversized_skip(accepted.count(False))

        if self.mask_gate == MASK_GATE_BOX:
            regions_started_ns = (
                time.monotonic_ns()
                if self.depth_match_diagnostics is not None else None)
            box_masks: list = []
            try:
                for detection, keep in zip(batch.detections, accepted):
                    if keep:
                        box_masks.append(region_from_detection(
                            detection, batch.image_height, batch.image_width))
                    else:
                        set_mask_estimator_status(
                            detection, MissReason.MASK_OVERSIZED_BOX,
                            self.enabled_estimators)
                        box_masks.append(None)
            finally:
                self.record_stage_timing(
                    'mask_region_prepare', regions_started_ns)
            return box_masks, None

        color_msg = None
        if (color_hint is not None
                and stamp_key(color_hint.header.stamp)
                == stamp_key(detections_msg.header.stamp)):
            color_msg = color_hint
        else:
            with self.processing_lock:
                color_msg = self.color_buffer.lookup(detections_msg.header.stamp)
        if color_msg is None:
            self.log_skip_warning(
                'Silhouette gate: no color frame buffered for the detection '
                'stamp (aged out or not arrived); skipping paths for this '
                'frame. Check the color_topic parameter and buffer depth.'
            )
            return None, None

        # Both inference backends consume this one contiguous RGB object. Only
        # accepted boxes are prompted; oversized ones map straight to ``None``.
        rgb_started_ns = (
            time.monotonic_ns()
            if self.depth_match_diagnostics is not None else None)
        try:
            prepared_color = PreparedColorFrame(
                message=color_msg,
                rgb=np.ascontiguousarray(decode_color_to_rgb(color_msg)),
            )
        finally:
            self.record_stage_timing('rgb_prepare', rgb_started_ns)
        prompt_boxes = [
            detection.bbox_xyxy
            for detection, keep in zip(batch.detections, accepted)
            if keep
        ]
        blobs: list = []
        if prompt_boxes:
            started = time.perf_counter()
            self.synchronize_cuda_for_timing()
            model_started_ns = (
                time.monotonic_ns()
                if self.depth_match_diagnostics is not None else None)
            try:
                blobs = self.segmenter.segment_boxes(
                    prepared_color.rgb, prompt_boxes,
                    min_predicted_iou=self.segmentation_min_iou)
                self.synchronize_cuda_for_timing()
            finally:
                self.record_stage_timing('slimsam', model_started_ns)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self.log_segmentation_latency(elapsed_ms, len(blobs))

        # Re-align the segmenter's per-prompt blobs back to full detection order.
        regions_started_ns = (
            time.monotonic_ns()
            if self.depth_match_diagnostics is not None else None)
        blob_iter = iter(blobs)
        masks: list = []
        try:
            for detection, keep in zip(batch.detections, accepted):
                if not keep:
                    set_mask_estimator_status(
                        detection, MissReason.MASK_OVERSIZED_BOX,
                        self.enabled_estimators)
                    masks.append(None)
                    continue
                blob = next(blob_iter)
                if blob is None:
                    set_mask_estimator_status(
                        detection, MissReason.MASK_EMPTY_SEGMENTATION,
                        self.enabled_estimators)
                    masks.append(None)
                else:
                    # Cropped to the blob's own extent and copied, so the frame-sized
                    # model output is free to expire at the end of this loop.
                    masks.append(region_from_blob(blob, MaskPrecision.TIGHT))
        finally:
            self.record_stage_timing('mask_region_prepare', regions_started_ns)
        return masks, prepared_color

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

    def depth_for_batch(
        self,
        depth_input_msg: Image | None,
        batch,
        *,
        prepared_color: PreparedColorFrame | None = None,
    ):
        """Aligned depth in meters on the batch grid, or ``None`` if unusable.

        ``depth_input_msg`` is the depth source's own input at the detection
        stamp -- a raw camera depth frame for the stereo source, the color
        frame for the monocular one -- so the conversion (a unit decode, or a
        network forward pass) happens here, once, for the frame the detections
        were made on. With silhouette + monocular, ``prepared_color`` carries
        that same exact RGB array from segmentation into the depth source.

        Every "no usable depth at this stamp" outcome funnels through ``None``,
        which the caller stamps as ``NO_DEPTH_FRAME``: nothing buffered at the
        stamp, an unsupported encoding, a monocular model that is unavailable
        or mid-cooldown, or a grid the masks cannot index.
        """

        if depth_input_msg is None and prepared_color is None:
            return None
        is_monocular = self.depth_source.input_kind == 'color'
        stage_name = 'depth_anything' if is_monocular else 'stereo_depth'
        if is_monocular:
            self.synchronize_cuda_for_timing()
        started_ns = (
            time.monotonic_ns()
            if self.depth_match_diagnostics is not None else None)
        try:
            produce_from_rgb = getattr(self.depth_source, 'produce_from_rgb', None)
            if (prepared_color is not None
                    and self.depth_source.input_kind == 'color'
                    and callable(produce_from_rgb)):
                frame = produce_from_rgb(
                    prepared_color.rgb, prepared_color.message.header)
            elif depth_input_msg is not None:
                frame = self.depth_source.produce(depth_input_msg)
            else:
                return None
        except ValueError as exc:
            self.log_skip_warning(f'Aligned depth frame skipped: {exc}')
            return None
        finally:
            if is_monocular:
                self.synchronize_cuda_for_timing()
            self.record_stage_timing(stage_name, started_ns)
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

    def publish_mask_debug(
        self,
        masks,
        image_height: int,
        image_width: int,
        header,
    ) -> None:
        """Publish the consumed mask union only when something subscribes."""

        if self.mask_debug_pub is None:
            return
        if self.mask_debug_pub.get_subscription_count() <= 0:
            return
        self.mask_debug_pub.publish(
            encode_mask_debug_image(masks, image_height, image_width, header))

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

    def camera_extrinsic_for_batch(self, detections_msg: TargetDetections):
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

    def scan_points_for_batch(self, detections_msg: TargetDetections, scan_msg: LaserScan | None):
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
        if self.depth_match_diagnostics is not None and rclpy.ok():
            with self.processing_lock:
                summary = self.depth_match_diagnostics.summary()
            self.get_logger().info('Final ' + summary)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    try:
        node = TargetMaskMeasurementNode()
    except RuntimeError as exc:
        # Silhouette gate without the venv (no transformers/torch). Fail
        # cleanly with a clear message instead of dumping a traceback --
        # same pattern as target_detector_node.
        get_logger('target_mask_measurement').fatal(str(exc))
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
