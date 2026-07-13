#!/usr/bin/env python3
"""Mask-based measurement node: the Path A/B benchmark rows.

Consumes detections plus the aligned depth frame (``aligned_depth_node``),
rasterizes each detection box into a ``rect`` mask, and runs both
localization paths (``perception/core/path_a.py`` / ``path_b.py``) per mask.
Results are converted from the camera optical frame to the vehicle-frame
planar-distance convention every benchmark row shares (ground truth
included), and published as ``G1Measurements`` with the identity fields of
the source detections message -- so the benchmark runner can merge them into
the same aligned event as the camera and lidar measurements.

The depth source (stereo vs Depth-Anything) is whatever ``aligned_depth_node``
was configured to produce; this node never branches on it. Deliberately
independent of the legacy estimator stack (``geometry.py`` /
``g1_camera_measurement_node``): constants are mirrored by value, never
imported.
"""

from __future__ import annotations

import math
import threading
import traceback

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ridgeback_autonomy.common.messages import (
    batch_from_detections_message,
    build_measurements_message,
)
from ridgeback_autonomy.msg import G1Detections, G1Measurements
from ridgeback_autonomy.perception.aligned_depth_node import (
    ALIGNED_CAMERA_INFO_TOPIC,
    ALIGNED_DEPTH_TOPIC,
    decode_depth_to_meters,
)
from ridgeback_autonomy.perception.core.intrinsics import intrinsics_from_camera_info
from ridgeback_autonomy.perception.core.isolation_2d import (
    ISOLATION_2D_DEFAULT,
    ISOLATION_2D_RECIPES,
)
from ridgeback_autonomy.perception.core.isolation_3d import (
    ISOLATION_3D_DEFAULT,
    ISOLATION_3D_RECIPES,
)
from ridgeback_autonomy.perception.core.mask import rasterize_detection
from ridgeback_autonomy.perception.core.path_a import localize_path_a
from ridgeback_autonomy.perception.core.path_b import localize_path_b


RAW_DETECTIONS_TOPIC = 'detections/g1/raw'
MASK_MEASUREMENTS_TOPIC = 'measurements/g1/mask'

ROBOT_FRONT_OFFSET_M_DEFAULT = 0.25  # mirrors geometry.ROBOT_FRONT_OFFSET_M
CAMERA_PITCH_DEG_DEFAULT = 0.0  # mirrors config/camera_config.json pitch_deg


def optical_to_vehicle_planar(
    xyz_optical,
    pitch_rad: float,
    front_offset_m: float,
) -> tuple[float, float, float]:
    """Camera-optical point -> ``(lateral_m, forward_m, distance_m)``.

    The vehicle-frame planar convention every benchmark row (and the ground
    truth) uses: optical X right is vehicle lateral; optical Y/Z fold into
    vehicle forward through the camera pitch; the robot front offset is
    subtracted from forward before the planar distance.
    """

    x, y, z = (float(value) for value in xyz_optical)
    lateral_m = x
    forward_m = -math.sin(pitch_rad) * y + math.cos(pitch_rad) * z - front_offset_m
    distance_m = math.hypot(lateral_m, forward_m)
    return lateral_m, forward_m, distance_m


class G1MaskMeasurementNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_mask_measurement_node')

        self.declare_parameter('detections_topic', RAW_DETECTIONS_TOPIC)
        self.declare_parameter('measurement_topic', MASK_MEASUREMENTS_TOPIC)
        self.declare_parameter('aligned_depth_topic', ALIGNED_DEPTH_TOPIC)
        self.declare_parameter('aligned_camera_info_topic', ALIGNED_CAMERA_INFO_TOPIC)
        self.declare_parameter('pitch_deg', CAMERA_PITCH_DEG_DEFAULT)
        self.declare_parameter('front_offset_m', ROBOT_FRONT_OFFSET_M_DEFAULT)
        self.declare_parameter('isolation_2d', ISOLATION_2D_DEFAULT)
        self.declare_parameter('isolation_3d', ISOLATION_3D_DEFAULT)

        self.pitch_rad = math.radians(float(self.get_parameter('pitch_deg').value))
        self.front_offset_m = float(self.get_parameter('front_offset_m').value)
        self.isolation_2d = self.resolve_recipe(
            'isolation_2d', ISOLATION_2D_RECIPES)
        self.isolation_3d = self.resolve_recipe(
            'isolation_3d', ISOLATION_3D_RECIPES)

        self.latest_detections_msg: G1Detections | None = None
        self.latest_depth_msg: Image | None = None
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

        self.measurement_pub = self.create_publisher(
            G1Measurements,
            str(self.get_parameter('measurement_topic').value),
            10,
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
            self.latest_depth_msg = depth_msg

    def camera_info_callback(self, camera_info: CameraInfo) -> None:
        self.latest_camera_info = camera_info

    def processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.process_event.wait(timeout=0.1):
                continue
            self.process_event.clear()

            while not self.stop_event.is_set():
                with self.processing_lock:
                    detections_msg = self.latest_detections_msg
                    depth_msg = self.latest_depth_msg
                    self.latest_detections_msg = None

                if detections_msg is None:
                    break

                try:
                    self.process_measurements(
                        detections_msg, depth_msg, self.latest_camera_info)
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
        camera_info: CameraInfo | None,
    ) -> None:
        """Publish one measurements message per detections message.

        Always publishes -- when the aligned depth frame or intrinsics are
        not available yet the path fields stay NaN, which downstream reads
        as "no estimate" (same convention as the other measurement nodes).
        """

        batch = batch_from_detections_message(detections_msg)

        if batch.detected and depth_msg is not None and camera_info is not None:
            try:
                depth_m = decode_depth_to_meters(depth_msg)
            except ValueError as exc:
                self.log_skip_warning(f'Aligned depth frame skipped: {exc}')
                depth_m = None
            if depth_m is not None:
                if depth_m.shape == (batch.image_height, batch.image_width):
                    self.fill_path_measurements(batch, depth_m, camera_info)
                else:
                    self.log_skip_warning(
                        f'Aligned depth grid {depth_m.shape} does not match the '
                        f'detection grid ({batch.image_height}, {batch.image_width}); '
                        'masks cannot index it. Check the aligned_depth_node input topics.'
                    )
        elif batch.detected:
            self.log_skip_warning(
                'No aligned depth frame / camera_info received yet; publishing '
                'measurements without path estimates.'
            )

        self.measurement_pub.publish(
            build_measurements_message(batch, detections_msg.header))

    def fill_path_measurements(self, batch, depth_m, camera_info: CameraInfo) -> None:
        intrinsics = intrinsics_from_camera_info(camera_info)
        for detection in batch.detections:
            mask = rasterize_detection(detection, batch.image_height, batch.image_width)

            result_a = localize_path_a(
                depth_m, mask, intrinsics, isolation=self.isolation_2d)
            if result_a is not None:
                (
                    detection.path_a_lateral_m,
                    detection.path_a_forward_m,
                    detection.path_a_distance_m,
                ) = optical_to_vehicle_planar(
                    result_a.xyz_optical, self.pitch_rad, self.front_offset_m)

            result_b = localize_path_b(
                depth_m, mask, intrinsics, isolation=self.isolation_3d)
            if result_b is not None:
                (
                    detection.path_b_lateral_m,
                    detection.path_b_forward_m,
                    detection.path_b_distance_m,
                ) = optical_to_vehicle_planar(
                    result_b.xyz_optical, self.pitch_rad, self.front_offset_m)

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
    node = G1MaskMeasurementNode()
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
