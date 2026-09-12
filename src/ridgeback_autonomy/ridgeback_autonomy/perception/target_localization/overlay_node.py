#!/usr/bin/env python3

from __future__ import annotations

from collections import OrderedDict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    DEPTH_PATH_ESTIMATORS,
    parse_estimators,
    parse_mask_gate,
    uses_mask_estimators,
)
from ridgeback_autonomy.common.messages import (
    batch_from_measurements_message,
    build_bgr8_image_message,
    polar_beam_booleans,
)
from ridgeback_autonomy.common.stamps import stamp_key
from ridgeback_autonomy.common.tf_utils import lookup_transform_components
from ridgeback_autonomy.msg import PolarBeams, TargetMeasurements
from ridgeback_autonomy.perception.target_localization.core.image_utils import (
    convert_color_image_message,
    convert_depth_to_meters_message,
    decode_image_message,
)
from ridgeback_autonomy.perception.target_localization.core.intrinsics import (
    intrinsics_from_camera_info,
    project_points,
)
from ridgeback_autonomy.perception.target_localization.core.polar_profiling import scan_points_optical
from ridgeback_autonomy.perception.target_localization.core.rendering import (
    PANEL_MAX_COLS_DEFAULT,
    RgbdOverlayRenderer,
    ScanHighlight,
)
from ridgeback_autonomy.perception.target_localization.contracts import (
    ALIGNED_DEPTH_DEBUG_TOPIC,
    GROUND_TRUTH_TOPIC,
    MASK_DEBUG_TOPIC,
    MASK_MEASUREMENTS_TOPIC,
    OVERLAY_IMAGE_TOPIC,
    POINTCLOUD_MEASUREMENTS_TOPIC,
    POLAR_BEAMS_TOPIC,
)
from ridgeback_autonomy.perception.target_localization.ground_truth import truth_reading


# The mask node's debug republish of the depth frame its paths read. It only
# converts depth on frames that carry a detection, so this panel updates on
# detection frames and holds its last one in between, rather than tracking the
# camera stream.
COLOR_CAMERA_INFO_TOPIC = 'sensors/camera_0/color/camera_info'
SCAN_TOPIC = 'sensors/lidar2d_0/scan'
DEPTH_MAX_METERS_DEFAULT = 10.0

# The estimator fields each measurement pipeline populates. render_latest builds
# the batch from whichever pipeline drove the frame, then merges the other in
# by stamp so the label block can show every selected estimator's line.
POINTCLOUD_FIELDS = (
    'pointcloud_lateral_m', 'pointcloud_forward_m', 'pointcloud_distance_m',
)
MASK_FIELDS = (
    'projective_ranging_lateral_m', 'projective_ranging_forward_m', 'projective_ranging_distance_m',
    'euclidean_reconstruction_lateral_m', 'euclidean_reconstruction_forward_m',
    'euclidean_reconstruction_distance_m',
    'polar_profiling_lateral_m', 'polar_profiling_forward_m', 'polar_profiling_distance_m',
)


class TargetOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__('target_overlay_node')

        self.declare_parameter('measurement_topic', POINTCLOUD_MEASUREMENTS_TOPIC)
        self.declare_parameter('mask_measurement_topic', MASK_MEASUREMENTS_TOPIC)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('mask_debug_topic', MASK_DEBUG_TOPIC)
        self.declare_parameter('aligned_depth_topic', ALIGNED_DEPTH_DEBUG_TOPIC)
        self.declare_parameter('color_camera_info_topic', COLOR_CAMERA_INFO_TOPIC)
        self.declare_parameter('scan_topic', SCAN_TOPIC)
        self.declare_parameter('polar_beams_topic', POLAR_BEAMS_TOPIC)
        self.declare_parameter('ground_truth_topic', GROUND_TRUTH_TOPIC)
        self.declare_parameter('depth_max_meters', DEPTH_MAX_METERS_DEFAULT)
        # The run config: which estimators, the aligned-depth source, and the
        # mask gate. These drive which panels are built and which labels drawn.
        self.declare_parameter('estimators', 'all')
        self.declare_parameter('depth_source', 'stereoscopic')
        self.declare_parameter('mask_gate', 'box')
        self.declare_parameter('overlay_image_topic', OVERLAY_IMAGE_TOPIC)
        # The RViz strip is far wider than it is tall, where a single row fills
        # it instead of letterboxing to a third of the width.
        self.declare_parameter('max_cols', PANEL_MAX_COLS_DEFAULT)
        # The per-detection label block on the RGB panel. Off for benchmark runs,
        # where the HUD carries the numbers as text RViz draws at full size and
        # the block otherwise covers the robot it annotates.
        self.declare_parameter('rgb_panel_labels', True)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_max_meters = float(self.get_parameter('depth_max_meters').value)
        self.rgb_panel_labels = bool(self.get_parameter('rgb_panel_labels').value)

        self.estimators = parse_estimators(str(self.get_parameter('estimators').value))
        self.depth_source = str(self.get_parameter('depth_source').value).strip() or 'stereoscopic'
        self.mask_gate = parse_mask_gate(str(self.get_parameter('mask_gate').value))
        self.wants_aligned = bool(set(self.estimators) & DEPTH_PATH_ESTIMATORS)
        self.wants_mask = uses_mask_estimators(self.estimators)
        self.wants_polar = 'polar_profiling' in self.estimators

        self.renderer = RgbdOverlayRenderer(
            self.depth_max_meters, self.estimators, self.depth_source, self.mask_gate,
            max_cols=max(1, int(self.get_parameter('max_cols').value)),
            rgb_panel_labels=self.rgb_panel_labels)

        self.latest_measurements_msg: TargetMeasurements | None = None
        self.latest_color_msg: Image | None = None
        self.latest_aligned_depth_msg: Image | None = None
        self.latest_color_info: CameraInfo | None = None
        self.latest_truth_msg: PointStamped | None = None
        self.pointcloud_cache: OrderedDict[tuple, TargetMeasurements] = OrderedDict()
        self.mask_cache: OrderedDict[tuple, TargetMeasurements] = OrderedDict()
        self.mask_debug_cache: OrderedDict[tuple[int, int], Image] = OrderedDict()
        self.polar_beams_cache: OrderedDict[tuple[int, int], PolarBeams] = OrderedDict()
        # Scans are kept by identity rather than latest-wins because a beams
        # message names the one its indices belong to, and that is rarely the
        # newest by the time the measurement it explains has been rendered.
        self.scan_cache: OrderedDict[tuple[str, int, int], LaserScan] = OrderedDict()
        self.last_matched_silhouette = None
        self.last_scan_tf_fallback: str | None = None
        self.last_color_warning: str | None = None
        self.last_aligned_depth_warning: str | None = None
        self.last_mask_debug_warning: str | None = None
        self.last_scan_tf_warning: str | None = None
        self.last_scan_decode_warning: str | None = None
        self.last_beam_count_warning: str | None = None

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.create_subscription(
            TargetMeasurements, self.get_parameter('measurement_topic').value,
            self.measurement_callback, 10)
        self.create_subscription(
            TargetMeasurements, self.get_parameter('mask_measurement_topic').value,
            self.mask_measurement_callback, 10)
        self.create_subscription(
            Image, self.color_topic, self.color_callback, qos_profile=qos_profile_sensor_data)
        # Silent outside the benchmark (nothing publishes it), so this needs no
        # exploration-vs-benchmark gate.
        # Depth 1: latest value wins, and a deeper queue would only let this
        # node read a previous trial's truth behind a backlog.
        self.create_subscription(
            PointStamped, self.get_parameter('ground_truth_topic').value,
            self.ground_truth_callback, 1)

        if self.wants_aligned:
            self.create_subscription(
                Image, self.get_parameter('aligned_depth_topic').value,
                self.aligned_depth_callback, qos_profile=qos_profile_sensor_data)
        if self.wants_mask:
            self.create_subscription(
                Image, self.get_parameter('mask_debug_topic').value,
                self.mask_debug_callback, qos_profile=qos_profile_sensor_data)
        if self.wants_polar:
            self.create_subscription(
                CameraInfo, self.get_parameter('color_camera_info_topic').value,
                self.color_info_callback, qos_profile=qos_profile_sensor_data)
            self.create_subscription(
                LaserScan, self.get_parameter('scan_topic').value,
                self.scan_callback, qos_profile=qos_profile_sensor_data)
            # The beams the measuring node actually reduced. Subscribing is what
            # makes it publish, and what makes this panel show the run it is
            # drawn on rather than a locally re-derived guess at it.
            self.create_subscription(
                PolarBeams, self.get_parameter('polar_beams_topic').value,
                self.polar_beams_callback, 10)

        # The composite is a topic so RViz can hold it alongside the 3D view.
        self.overlay_pub = self.create_publisher(
            Image, str(self.get_parameter('overlay_image_topic').value),
            qos_profile_sensor_data)

    # -- measurement inputs --

    def measurement_callback(self, msg: TargetMeasurements) -> None:
        self.cache_measurement(self.pointcloud_cache, msg)
        self.latest_measurements_msg = msg
        self.render_latest()

    def mask_measurement_callback(self, msg: TargetMeasurements) -> None:
        self.cache_measurement(self.mask_cache, msg)
        self.latest_measurements_msg = msg
        self.render_latest()

    def cache_measurement(self, cache: OrderedDict, msg: TargetMeasurements) -> None:
        self.cache_by_key(cache, self.measurement_message_key(msg), msg)

    # Deep enough to cover the lag between a measurement and the artifacts that
    # explain it, shallow enough that a stalled consumer cannot grow unbounded.
    CACHE_DEPTH = 32

    def cache_by_key(self, cache: OrderedDict, key, msg) -> None:
        cache[key] = msg
        cache.move_to_end(key)
        while len(cache) > self.CACHE_DEPTH:
            cache.popitem(last=False)

    # -- image / scan inputs --

    def color_callback(self, color_msg: Image) -> None:
        self.latest_color_msg = color_msg

    def aligned_depth_callback(self, aligned_depth_msg: Image) -> None:
        self.latest_aligned_depth_msg = aligned_depth_msg

    def color_info_callback(self, info_msg: CameraInfo) -> None:
        self.latest_color_info = info_msg

    def scan_callback(self, scan_msg: LaserScan) -> None:
        self.cache_by_key(
            self.scan_cache, self.scan_key(scan_msg.header), scan_msg)

    def ground_truth_callback(self, msg: PointStamped) -> None:
        self.latest_truth_msg = msg

    def current_truth(self) -> tuple[float, float, float] | None:
        """The benchmark ground truth while it is being republished, else None.

        Age-gated on the message stamp by ``truth_reading``, so the reference
        line disappears between trials instead of lying next to a teleported
        target -- and so a message that waited in the queue counts as the old
        data it is rather than as a fresh arrival.
        """

        reading = truth_reading(self.latest_truth_msg, self.get_clock().now().nanoseconds)
        if reading is None:
            return None
        return (reading.lateral_m, reading.forward_m, reading.distance_m)

    def mask_debug_callback(self, mask_msg: Image) -> None:
        key = stamp_key(mask_msg.header.stamp)
        self.cache_by_key(self.mask_debug_cache, key, mask_msg)
        self.rerender_if_pending(key)

    def polar_beams_callback(self, beams_msg: PolarBeams) -> None:
        key = stamp_key(beams_msg.header.stamp)
        self.cache_by_key(self.polar_beams_cache, key, beams_msg)
        self.rerender_if_pending(key)

    def rerender_if_pending(self, key: tuple[int, int]) -> None:
        """Re-render when a per-frame artifact for the pending stamp arrives.

        Both the silhouette image and the beam indices lag their measurements
        message -- by segmentation latency and by the estimator reduction
        respectively -- so the frame is first rendered without them. Without
        this the panel would keep its fallback content until the next
        measurement arrived.
        """

        measurements_msg = self.latest_measurements_msg
        if measurements_msg is None:
            return
        if key == stamp_key(measurements_msg.header.stamp):
            self.render_latest()

    # -- rendering --

    def render_latest(self) -> None:
        if self.latest_measurements_msg is None or self.latest_color_msg is None:
            return

        try:
            frame = convert_color_image_message(self.latest_color_msg)
            self.last_color_warning = None
        except ValueError as exc:
            self.log_warning_once(
                'last_color_warning',
                f'Cannot decode color image ({self.latest_color_msg.encoding}): {exc}')
            return

        aligned_depth_meters = self.decode_depth(
            self.latest_aligned_depth_msg, 'last_aligned_depth_warning', 'aligned depth')

        batch = batch_from_measurements_message(self.latest_measurements_msg)
        self.merge_measurements(batch)

        scan_uv, scan_in_view, scan_highlight = self.project_published_scan()
        annotated = self.renderer.render(
            frame,
            batch,
            aligned_depth_meters=aligned_depth_meters,
            published_mask=self.match_mask_debug(self.latest_measurements_msg),
            scan_uv=scan_uv,
            scan_in_view=scan_in_view,
            scan_highlight=scan_highlight,
            truth=self.current_truth(),
        )
        self.overlay_pub.publish(
            build_bgr8_image_message(annotated, self.latest_color_msg.header))

    def decode_depth(self, depth_msg: Image | None, warning_attr: str, label: str):
        if depth_msg is None:
            return None
        try:
            meters = convert_depth_to_meters_message(depth_msg)
            setattr(self, warning_attr, None)
            return meters
        except ValueError as exc:
            self.log_warning_once(
                warning_attr, f'Cannot decode {label} image ({depth_msg.encoding}): {exc}')
            return None

    def merge_measurements(self, batch) -> None:
        """Fill the batch with every pipeline's fields so the label block can
        show every selected estimator, not only the pipeline that drove this
        frame. The driving message's own fields are already present."""

        primary = self.latest_measurements_msg
        for cache, fields in (
                (self.pointcloud_cache, POINTCLOUD_FIELDS),
                (self.mask_cache, MASK_FIELDS)):
            other = self.match_measurement(cache, primary)
            if other is not None and other is not primary:
                self.merge_fields(batch, other, fields)

    def merge_fields(self, batch, other_msg: TargetMeasurements, field_names) -> None:
        other = batch_from_measurements_message(other_msg)
        for detection, other_detection in zip(batch.detections, other.detections):
            for name in field_names:
                setattr(detection, name, getattr(other_detection, name))

    def project_published_scan(self):
        """``(uv, in_view, highlight)`` for the scan the measurement was made on.

        Not the latest scan: the beams message names the array its indices index
        into, and applying them to a newer scan would put the highlight on the
        wrong beams -- silently, and worst exactly when the platform is moving.
        A scan that has aged out of the cache, or a frame with no beams message
        yet, therefore draws nothing at all. A missing highlight is the required
        failure mode here; a misaligned one is not.
        """

        if (not self.wants_polar or self.latest_color_info is None
                or self.latest_measurements_msg is None):
            return None, None, None
        beams_msg = self.polar_beams_cache.get(
            stamp_key(self.latest_measurements_msg.header.stamp))
        if beams_msg is None:
            return None, None, None
        scan_msg = self.scan_cache.get(
            (beams_msg.scan_frame_id, *stamp_key(beams_msg.scan_stamp)))
        if scan_msg is None:
            return None, None, None
        try:
            rotation, translation, self.last_scan_tf_fallback = lookup_transform_components(
                self.tf_buffer,
                self.latest_measurements_msg.header.frame_id,
                scan_msg.header.frame_id,
                Time.from_msg(self.latest_measurements_msg.header.stamp),
                self.get_logger(),
                self.last_scan_tf_fallback,
            )
        except TransformException as exc:
            self.log_warning_once(
                'last_scan_tf_warning', f'Polar overlay scan skipped (TF): {exc}')
            return None, None, None
        try:
            points_optical, valid = scan_points_optical(
                scan_msg, rotation, translation)
        except ValueError as exc:
            self.log_warning_once(
                'last_scan_decode_warning', f'Polar overlay scan skipped: {exc}')
            return None, None, None
        intrinsics = intrinsics_from_camera_info(self.latest_color_info)
        uv, in_view = project_points(points_optical, intrinsics)
        return uv, (valid & in_view), self.beam_highlight(beams_msg, scan_msg)

    def beam_highlight(self, beams_msg: PolarBeams, scan_msg: LaserScan):
        """The per-beam states, or ``None`` if they cannot be trusted here."""

        booleans = polar_beam_booleans(beams_msg, len(scan_msg.ranges))
        if booleans is None:
            self.log_warning_once(
                'last_beam_count_warning',
                f'Polar beams recorded against {beams_msg.beam_count} beams but '
                f'the matched scan has {len(scan_msg.ranges)}; drawing the scan '
                'without a highlight. Check that one scan producer is running.')
            return None
        self.last_beam_count_warning = None
        used, dropped = booleans
        return ScanHighlight(used=used, dropped=dropped)

    def match_mask_debug(self, measurements_msg: TargetMeasurements):
        """The silhouette artifact for the rendered stamp, else the newest one."""

        mask_msg = self.mask_debug_cache.get(stamp_key(measurements_msg.header.stamp))
        if mask_msg is None:
            return self.last_matched_silhouette
        try:
            mask = decode_image_message(mask_msg)
            self.last_mask_debug_warning = None
        except ValueError as exc:
            self.log_warning_once(
                'last_mask_debug_warning',
                f'Cannot decode mask debug image ({mask_msg.encoding}): {exc}')
            return self.last_matched_silhouette
        self.last_matched_silhouette = mask > 0
        return self.last_matched_silhouette

    # The pointcloud and mask measurements run at different rates and rarely
    # share an exact detection stamp, so pair within this window on equal count.
    MATCH_MAX_DT_SEC = 1.0

    def match_measurement(self, cache: OrderedDict, primary_msg: TargetMeasurements):
        if not cache:
            return None
        exact = cache.get(self.measurement_message_key(primary_msg))
        if exact is not None:
            return exact
        target = self.stamp_seconds(primary_msg.header.stamp)
        best_msg = None
        best_dt = None
        for msg in cache.values():
            if int(msg.count) != int(primary_msg.count):
                continue
            dt = abs(self.stamp_seconds(msg.header.stamp) - target)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_msg = msg
        if best_msg is None or best_dt > self.MATCH_MAX_DT_SEC:
            return None
        return best_msg

    @staticmethod
    def stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def scan_key(header) -> tuple[str, int, int]:
        """Frame plus stamp: what a beams message names its scan by."""

        return (header.frame_id, *stamp_key(header.stamp))

    def measurement_message_key(self, msg: TargetMeasurements) -> tuple:
        return (
            msg.header.frame_id,
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
            int(msg.count),
            tuple(float(value) for value in msg.bbox_xyxy),
        )

    def log_warning_once(self, attribute_name: str, warning: str) -> None:
        if warning == getattr(self, attribute_name):
            return
        setattr(self, attribute_name, warning)
        self.get_logger().warn(warning)

def main() -> None:
    rclpy.init()
    node = TargetOverlayNode()
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
