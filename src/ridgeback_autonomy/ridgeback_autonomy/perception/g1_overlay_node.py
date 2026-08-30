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

from ridgeback_autonomy.perception.estimators import (
    DEPTH_PATH_ESTIMATORS,
    parse_estimators,
    parse_mask_gate,
    uses_mask_estimators,
)
from ridgeback_autonomy.common.messages import (
    batch_from_measurements_message,
    build_bgr8_image_message,
)
from ridgeback_autonomy.common.tf_utils import lookup_transform_components
from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.perception.core.image_utils import (
    convert_color_image_message,
    convert_depth_to_meters_message,
    decode_image_message,
)
from ridgeback_autonomy.perception.core.intrinsics import (
    intrinsics_from_camera_info,
    project_points,
)
from ridgeback_autonomy.perception.core.polar_profiling import scan_points_optical
from ridgeback_autonomy.perception.core.rendering import (
    PANEL_MAX_COLS_DEFAULT,
    RgbdOverlayRenderer,
)
from ridgeback_autonomy.perception.ground_truth import GROUND_TRUTH_TOPIC, truth_reading


POINTCLOUD_MEASUREMENTS_TOPIC = 'measurements/g1/pointcloud'
MASK_MEASUREMENTS_TOPIC = 'measurements/g1/mask'
MASK_DEBUG_TOPIC = 'debug/g1/mask'
OVERLAY_IMAGE_TOPIC = 'debug/g1/overlay'
# The mask node's debug republish of the depth frame its paths read. It only
# converts depth on frames that carry a detection, so this panel updates on
# detection frames and holds its last one in between, rather than tracking the
# camera stream.
ALIGNED_DEPTH_TOPIC = 'debug/g1/mask/aligned_depth'
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


class G1OverlayNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_overlay_node')

        self.declare_parameter('measurement_topic', POINTCLOUD_MEASUREMENTS_TOPIC)
        self.declare_parameter('mask_measurement_topic', MASK_MEASUREMENTS_TOPIC)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('mask_debug_topic', MASK_DEBUG_TOPIC)
        self.declare_parameter('aligned_depth_topic', ALIGNED_DEPTH_TOPIC)
        self.declare_parameter('color_camera_info_topic', COLOR_CAMERA_INFO_TOPIC)
        self.declare_parameter('scan_topic', SCAN_TOPIC)
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

        self.latest_measurements_msg: G1Measurements | None = None
        self.latest_color_msg: Image | None = None
        self.latest_aligned_depth_msg: Image | None = None
        self.latest_color_info: CameraInfo | None = None
        self.latest_scan_msg: LaserScan | None = None
        self.latest_truth_msg: PointStamped | None = None
        self.pointcloud_cache: OrderedDict[tuple, G1Measurements] = OrderedDict()
        self.mask_cache: OrderedDict[tuple, G1Measurements] = OrderedDict()
        self.mask_debug_cache: OrderedDict[tuple[int, int], Image] = OrderedDict()
        self.last_matched_silhouette = None
        self.last_scan_tf_fallback: str | None = None
        self.last_color_warning: str | None = None
        self.last_aligned_depth_warning: str | None = None
        self.last_mask_debug_warning: str | None = None
        self.last_scan_tf_warning: str | None = None
        self.last_scan_decode_warning: str | None = None

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.create_subscription(
            G1Measurements, self.get_parameter('measurement_topic').value,
            self.measurement_callback, 10)
        self.create_subscription(
            G1Measurements, self.get_parameter('mask_measurement_topic').value,
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

        # The composite is a topic so RViz can hold it alongside the 3D view.
        self.overlay_pub = self.create_publisher(
            Image, str(self.get_parameter('overlay_image_topic').value),
            qos_profile_sensor_data)

    # -- measurement inputs --

    def measurement_callback(self, msg: G1Measurements) -> None:
        self.cache_measurement(self.pointcloud_cache, msg)
        self.latest_measurements_msg = msg
        self.render_latest()

    def mask_measurement_callback(self, msg: G1Measurements) -> None:
        self.cache_measurement(self.mask_cache, msg)
        self.latest_measurements_msg = msg
        self.render_latest()

    def cache_measurement(self, cache: OrderedDict, msg: G1Measurements) -> None:
        key = self.measurement_message_key(msg)
        cache[key] = msg
        cache.move_to_end(key)
        while len(cache) > 32:
            cache.popitem(last=False)

    # -- image / scan inputs --

    def color_callback(self, color_msg: Image) -> None:
        self.latest_color_msg = color_msg

    def aligned_depth_callback(self, aligned_depth_msg: Image) -> None:
        self.latest_aligned_depth_msg = aligned_depth_msg

    def color_info_callback(self, info_msg: CameraInfo) -> None:
        self.latest_color_info = info_msg

    def scan_callback(self, scan_msg: LaserScan) -> None:
        self.latest_scan_msg = scan_msg

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
        key = (mask_msg.header.stamp.sec, mask_msg.header.stamp.nanosec)
        self.mask_debug_cache[key] = mask_msg
        self.mask_debug_cache.move_to_end(key)
        while len(self.mask_debug_cache) > 32:
            self.mask_debug_cache.popitem(last=False)
        # The silhouette artifact lags its measurements message by the
        # segmentation latency, so re-render when the matching mask arrives,
        # otherwise the panel keeps its fallback content.
        measurements_msg = self.latest_measurements_msg
        if measurements_msg is not None and key == (
                measurements_msg.header.stamp.sec, measurements_msg.header.stamp.nanosec):
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

        scan_uv, scan_in_view, scan_points_optical = self.project_scan()
        annotated = self.renderer.render(
            frame,
            batch,
            aligned_depth_meters=aligned_depth_meters,
            published_mask=self.match_mask_debug(self.latest_measurements_msg),
            scan_uv=scan_uv,
            scan_in_view=scan_in_view,
            scan_points_optical=scan_points_optical,
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

    def merge_fields(self, batch, other_msg: G1Measurements, field_names) -> None:
        other = batch_from_measurements_message(other_msg)
        for detection, other_detection in zip(batch.detections, other.detections):
            for name in field_names:
                setattr(detection, name, getattr(other_detection, name))

    def project_scan(self):
        if not self.wants_polar:
            return None, None, None
        if (self.latest_scan_msg is None or self.latest_color_info is None
                or self.latest_measurements_msg is None):
            return None, None, None
        try:
            rotation, translation, self.last_scan_tf_fallback = lookup_transform_components(
                self.tf_buffer,
                self.latest_measurements_msg.header.frame_id,
                self.latest_scan_msg.header.frame_id,
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
                self.latest_scan_msg, rotation, translation)
        except ValueError as exc:
            self.log_warning_once(
                'last_scan_decode_warning', f'Polar overlay scan skipped: {exc}')
            return None, None, None
        intrinsics = intrinsics_from_camera_info(self.latest_color_info)
        uv, in_view = project_points(points_optical, intrinsics)
        return uv, (valid & in_view), points_optical

    def match_mask_debug(self, measurements_msg: G1Measurements):
        """The silhouette artifact for the rendered stamp, else the newest one."""

        mask_msg = self.mask_debug_cache.get(
            (measurements_msg.header.stamp.sec, measurements_msg.header.stamp.nanosec))
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

    def match_measurement(self, cache: OrderedDict, primary_msg: G1Measurements):
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

    def measurement_message_key(self, msg: G1Measurements) -> tuple:
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
    node = G1OverlayNode()
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
