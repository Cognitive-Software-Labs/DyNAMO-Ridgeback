#!/usr/bin/env python3

from __future__ import annotations

import time
from collections import OrderedDict

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from ridgeback_autonomy.benchmarking.estimators import (
    DEPTH_PATH_ESTIMATORS,
    GROUND_TRUTH_TOPIC,
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


CAMERA_MEASUREMENTS_TOPIC = 'measurements/g1/camera'
LIDAR_MEASUREMENTS_TOPIC = 'measurements/g1/lidar'
MASK_MEASUREMENTS_TOPIC = 'measurements/g1/mask'
MONO_DEPTH_DEBUG_TOPIC = 'debug/g1/camera/mono_depth'
MASK_DEBUG_TOPIC = 'debug/g1/mask'
OVERLAY_IMAGE_TOPIC = 'debug/g1/overlay'
ALIGNED_DEPTH_TOPIC = 'perception/aligned_depth/image'
COLOR_CAMERA_INFO_TOPIC = 'sensors/camera_0/color/camera_info'
SCAN_TOPIC = 'sensors/lidar2d_0/scan'
DEPTH_MAX_METERS_DEFAULT = 10.0
RENDER_FPS_DEFAULT = 15.0

# The benchmark runner republishes the trial ground truth at ~1 Hz during a
# capture window. The age gate drops the reference line shortly after capture
# ends, so a stale truth never sits next to a teleported target between trials.
TRUTH_MAX_AGE_S = 3.0

# The estimator fields each measurement pipeline populates. render_latest builds
# the batch from whichever pipeline drove the frame, then merges the others in
# by stamp so the label block can show every selected estimator's line.
CAMERA_FIELDS = (
    'rgb_lateral_m', 'rgb_forward_m', 'rgb_distance_m',
    'sensor_depth_distance_m', 'mono_depth_distance_m',
    'pointcloud_lateral_m', 'pointcloud_forward_m', 'pointcloud_distance_m',
)
LIDAR_FIELDS = ('lidar_lateral_m', 'lidar_forward_m', 'lidar_distance_m')
MASK_FIELDS = (
    'projective_ranging_lateral_m', 'projective_ranging_forward_m', 'projective_ranging_distance_m',
    'euclidean_reconstruction_lateral_m', 'euclidean_reconstruction_forward_m',
    'euclidean_reconstruction_distance_m',
    'polar_profiling_lateral_m', 'polar_profiling_forward_m', 'polar_profiling_distance_m',
)


class G1OverlayNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_overlay_node')

        self.declare_parameter('measurement_topic', CAMERA_MEASUREMENTS_TOPIC)
        self.declare_parameter('lidar_measurement_topic', LIDAR_MEASUREMENTS_TOPIC)
        self.declare_parameter('mask_measurement_topic', MASK_MEASUREMENTS_TOPIC)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('depth_topic', 'sensors/camera_0/depth/image')
        self.declare_parameter('mono_depth_debug_topic', MONO_DEPTH_DEBUG_TOPIC)
        self.declare_parameter('mask_debug_topic', MASK_DEBUG_TOPIC)
        self.declare_parameter('aligned_depth_topic', ALIGNED_DEPTH_TOPIC)
        self.declare_parameter('color_camera_info_topic', COLOR_CAMERA_INFO_TOPIC)
        self.declare_parameter('scan_topic', SCAN_TOPIC)
        self.declare_parameter('ground_truth_topic', GROUND_TRUTH_TOPIC)
        self.declare_parameter('depth_max_meters', DEPTH_MAX_METERS_DEFAULT)
        self.declare_parameter('render_fps', RENDER_FPS_DEFAULT)
        self.declare_parameter('window_name', 'G1 Perception')
        # The run config: which estimators, the aligned-depth source, and the
        # mask gate. These drive which panels are built and which labels drawn.
        self.declare_parameter('estimators', 'all')
        self.declare_parameter('depth_source', 'stereoscopic')
        self.declare_parameter('mask_gate', 'box')
        self.declare_parameter('overlay_image_topic', OVERLAY_IMAGE_TOPIC)
        # The standalone OpenCV window. Kept on by default so existing workflows
        # are unchanged; benchmark runs turn it off, because the composite is
        # published for RViz there and a floating window would sit over it.
        self.declare_parameter('show_window', True)
        # Panels per row. A tall 3-wide grid suits the standalone window; the
        # RViz strip is far wider than it is tall, where a single row fills it
        # instead of letterboxing to a third of the width.
        self.declare_parameter('max_cols', PANEL_MAX_COLS_DEFAULT)
        # The per-detection label block on the RGB panel. Off for benchmark runs,
        # where the HUD carries the numbers as text RViz draws at full size and
        # the block otherwise covers the robot it annotates.
        self.declare_parameter('rgb_panel_labels', True)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_max_meters = float(self.get_parameter('depth_max_meters').value)
        self.render_fps = max(1.0, float(self.get_parameter('render_fps').value))
        self.window_name = self.get_parameter('window_name').value
        self.show_window = bool(self.get_parameter('show_window').value)
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
        self.latest_depth_msg: Image | None = None
        self.latest_mono_depth_msg: Image | None = None
        self.latest_aligned_depth_msg: Image | None = None
        self.latest_color_info: CameraInfo | None = None
        self.latest_scan_msg: LaserScan | None = None
        self.latest_truth_msg: PointStamped | None = None
        self.latest_truth_monotonic = 0.0
        self.camera_cache: OrderedDict[tuple, G1Measurements] = OrderedDict()
        self.lidar_cache: OrderedDict[tuple, G1Measurements] = OrderedDict()
        self.mask_cache: OrderedDict[tuple, G1Measurements] = OrderedDict()
        self.mask_debug_cache: OrderedDict[tuple[int, int], Image] = OrderedDict()
        self.last_matched_silhouette = None
        self.last_scan_tf_fallback: str | None = None
        self.last_color_warning: str | None = None
        self.last_depth_warning: str | None = None
        self.last_mono_depth_warning: str | None = None
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
            G1Measurements, self.get_parameter('lidar_measurement_topic').value,
            self.lidar_measurement_callback, 10)
        self.create_subscription(
            G1Measurements, self.get_parameter('mask_measurement_topic').value,
            self.mask_measurement_callback, 10)
        self.create_subscription(
            Image, self.color_topic, self.color_callback, qos_profile=qos_profile_sensor_data)
        # Silent outside the benchmark (nothing publishes it), so this needs no
        # exploration-vs-benchmark gate.
        self.create_subscription(
            PointStamped, self.get_parameter('ground_truth_topic').value,
            self.ground_truth_callback, 10)

        if 'sensor_depth' in self.estimators:
            self.create_subscription(
                Image, self.get_parameter('depth_topic').value,
                self.depth_callback, qos_profile=qos_profile_sensor_data)
        if 'depth_anything' in self.estimators:
            self.create_subscription(
                Image, self.get_parameter('mono_depth_debug_topic').value,
                self.mono_depth_callback, qos_profile=qos_profile_sensor_data)
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

        self.render_timer = self.create_timer(1.0 / self.render_fps, self.render_callback)

        # The composite as a topic, so RViz can hold it alongside the 3D view and
        # one window contains the whole picture. Published whatever the window
        # setting, since the two are independent sinks for the same frame.
        self.overlay_pub = self.create_publisher(
            Image, str(self.get_parameter('overlay_image_topic').value),
            qos_profile_sensor_data)

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1280, 720)
            cv2.moveWindow(self.window_name, 60, 60)

    # -- measurement inputs --

    def measurement_callback(self, msg: G1Measurements) -> None:
        self.cache_measurement(self.camera_cache, msg)
        self.latest_measurements_msg = msg
        self.render_latest()

    def mask_measurement_callback(self, msg: G1Measurements) -> None:
        self.cache_measurement(self.mask_cache, msg)
        self.latest_measurements_msg = msg
        self.render_latest()

    def lidar_measurement_callback(self, msg: G1Measurements) -> None:
        self.cache_measurement(self.lidar_cache, msg)

    def cache_measurement(self, cache: OrderedDict, msg: G1Measurements) -> None:
        key = self.measurement_message_key(msg)
        cache[key] = msg
        cache.move_to_end(key)
        while len(cache) > 32:
            cache.popitem(last=False)

    # -- image / scan inputs --

    def color_callback(self, color_msg: Image) -> None:
        self.latest_color_msg = color_msg

    def depth_callback(self, depth_msg: Image) -> None:
        self.latest_depth_msg = depth_msg

    def mono_depth_callback(self, mono_depth_msg: Image) -> None:
        self.latest_mono_depth_msg = mono_depth_msg

    def aligned_depth_callback(self, aligned_depth_msg: Image) -> None:
        self.latest_aligned_depth_msg = aligned_depth_msg

    def color_info_callback(self, info_msg: CameraInfo) -> None:
        self.latest_color_info = info_msg

    def scan_callback(self, scan_msg: LaserScan) -> None:
        self.latest_scan_msg = scan_msg

    def ground_truth_callback(self, msg: PointStamped) -> None:
        self.latest_truth_msg = msg
        self.latest_truth_monotonic = time.monotonic()

    def current_truth(self) -> tuple[float, float, float] | None:
        """The benchmark ground truth while it is being republished, else None.

        Packed as x=lateral, y=forward, z=distance (the runner's
        ``ground_truth_point_message``). Age-gated so the reference line
        disappears between trials instead of lying next to a teleported target.
        """

        if self.latest_truth_msg is None:
            return None
        if time.monotonic() - self.latest_truth_monotonic > TRUTH_MAX_AGE_S:
            return None
        point = self.latest_truth_msg.point
        return (point.x, point.y, point.z)

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

    def render_callback(self) -> None:
        # Pumps the highgui event loop; pointless with no window to pump.
        if self.show_window:
            cv2.waitKey(1)

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

        sensor_depth_meters = self.decode_depth(
            self.latest_depth_msg, 'last_depth_warning', 'depth')
        mono_depth_meters = self.decode_depth(
            self.latest_mono_depth_msg, 'last_mono_depth_warning', 'mono-depth debug')
        aligned_depth_meters = self.decode_depth(
            self.latest_aligned_depth_msg, 'last_aligned_depth_warning', 'aligned depth')

        batch = batch_from_measurements_message(self.latest_measurements_msg)
        self.merge_measurements(batch)

        scan_uv, scan_in_view, scan_points_optical = self.project_scan()
        annotated = self.renderer.render(
            frame,
            batch,
            sensor_depth_meters=sensor_depth_meters,
            mono_depth_meters=mono_depth_meters,
            aligned_depth_meters=aligned_depth_meters,
            published_mask=self.match_mask_debug(self.latest_measurements_msg),
            scan_uv=scan_uv,
            scan_in_view=scan_in_view,
            scan_points_optical=scan_points_optical,
            truth=self.current_truth(),
        )
        self.overlay_pub.publish(
            build_bgr8_image_message(annotated, self.latest_color_msg.header))
        if self.show_window:
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)

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
                (self.camera_cache, CAMERA_FIELDS),
                (self.lidar_cache, LIDAR_FIELDS),
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

    # Camera, lidar, and mask measurements run at different rates and rarely
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

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


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
