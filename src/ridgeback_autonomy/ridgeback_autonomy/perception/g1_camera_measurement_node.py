#!/usr/bin/env python3

from __future__ import annotations

import os
import threading

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from ridgeback_autonomy.common.camera_config import load_camera_config
from ridgeback_autonomy.common.messages import (
    batch_from_detections_message,
    build_float32_image_message,
    build_measurements_message,
)
from ridgeback_autonomy.common.tf_utils import lookup_transform_components
from ridgeback_autonomy.msg import G1Detections, G1Measurements
from ridgeback_autonomy.perception.core.depth_anything import (
    DEPTH_ANYTHING_ENABLED_DEFAULT,
    DEPTH_ANYTHING_MODEL_ID_DEFAULT,
    DepthAnythingEstimator,
)
from ridgeback_autonomy.perception.core.detection import resolve_torch_device
from ridgeback_autonomy.perception.core.geometry import (
    add_depth_measurements,
    add_pointcloud_measurements,
    add_rgb_measurements,
    extract_organized_xyz,
)
from ridgeback_autonomy.perception.core.image_utils import (
    bgr_frame_to_pil,
    convert_color_image_message,
    convert_depth_to_meters_message,
)


RAW_DETECTIONS_TOPIC = 'detections/g1/raw'
CAMERA_MEASUREMENTS_TOPIC = 'measurements/g1/camera'
MONO_DEPTH_DEBUG_TOPIC = 'debug/g1/camera/mono_depth'
DEPTH_MAX_METERS_DEFAULT = 10.0


class G1CameraMeasurementNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_camera_measurement_node')

        pkg_share = get_package_share_directory('ridgeback_autonomy')
        default_config = os.path.join(pkg_share, 'config', 'camera_config.json')
        default_base_frame = self.default_base_frame()
        default_depth_anything_device = resolve_torch_device()

        self.declare_parameter('camera_config_path', default_config)
        self.declare_parameter('detections_topic', RAW_DETECTIONS_TOPIC)
        self.declare_parameter('measurement_topic', CAMERA_MEASUREMENTS_TOPIC)
        self.declare_parameter('mono_depth_debug_topic', MONO_DEPTH_DEBUG_TOPIC)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('depth_topic', 'sensors/camera_0/depth/image')
        self.declare_parameter('pointcloud_topic', 'sensors/camera_0/points')
        self.declare_parameter('base_frame', default_base_frame)
        self.declare_parameter('depth_max_meters', DEPTH_MAX_METERS_DEFAULT)
        self.declare_parameter('depth_anything_enabled', DEPTH_ANYTHING_ENABLED_DEFAULT)
        self.declare_parameter('depth_anything_model_id', DEPTH_ANYTHING_MODEL_ID_DEFAULT)
        self.declare_parameter('depth_anything_device', default_depth_anything_device)

        self.camera_config_path = self.get_parameter('camera_config_path').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.measurement_topic = self.get_parameter('measurement_topic').value
        self.mono_depth_debug_topic = self.get_parameter('mono_depth_debug_topic').value
        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.base_frame = self.get_parameter('base_frame').value or default_base_frame
        self.depth_max_meters = float(self.get_parameter('depth_max_meters').value)
        self.depth_anything_enabled = bool(self.get_parameter('depth_anything_enabled').value)
        self.depth_anything_model_id = self.get_parameter('depth_anything_model_id').value
        self.depth_anything_device = self.get_parameter('depth_anything_device').value

        self.camera_config = load_camera_config(self.camera_config_path)
        self.last_pointcloud_warning = None
        self.last_pointcloud_shape_warning = None
        self.last_depth_warning = None
        self.last_color_warning = None
        self.last_base_frame_fallback = None

        self.depth_anything = DepthAnythingEstimator(
            enabled=self.depth_anything_enabled,
            model_id=self.depth_anything_model_id,
            device=self.depth_anything_device,
            logger=self.get_logger(),
        )
        if self.depth_anything_enabled:
            self.depth_anything.load()

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.latest_detections_msg: G1Detections | None = None
        self.latest_color_msg: Image | None = None
        self.latest_depth_msg: Image | None = None
        self.latest_depth_meters: np.ndarray | None = None
        self.latest_pointcloud_msg: PointCloud2 | None = None
        self.latest_pointcloud_xyz: np.ndarray | None = None
        self.latest_pointcloud_rotation: np.ndarray | None = None
        self.latest_pointcloud_translation: np.ndarray | None = None
        self.processing_lock = threading.Lock()
        self.process_event = threading.Event()
        self.depth_process_event = threading.Event()
        self.pointcloud_process_event = threading.Event()
        self.stop_event = threading.Event()

        self.create_subscription(
            G1Detections,
            self.detections_topic,
            self.detections_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )

        self.measurement_pub = self.create_publisher(
            G1Measurements,
            self.measurement_topic,
            10,
        )
        self.mono_depth_pub = self.create_publisher(
            Image,
            self.mono_depth_debug_topic,
            10,
        )

        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.depth_worker_thread = threading.Thread(target=self.depth_processing_loop, daemon=True)
        self.pointcloud_worker_thread = threading.Thread(
            target=self.pointcloud_processing_loop,
            daemon=True,
        )
        self.worker_thread.start()
        self.depth_worker_thread.start()
        self.pointcloud_worker_thread.start()

    def default_base_frame(self) -> str:
        namespace_name = self.get_namespace().strip('/')
        if namespace_name:
            return f'{namespace_name}/robot/base_link'
        return 'robot/base_link'

    def detections_callback(self, detections_msg: G1Detections) -> None:
        with self.processing_lock:
            self.latest_detections_msg = detections_msg
        self.process_event.set()

    def color_callback(self, color_msg: Image) -> None:
        with self.processing_lock:
            self.latest_color_msg = color_msg

    def depth_callback(self, depth_msg: Image) -> None:
        with self.processing_lock:
            self.latest_depth_msg = depth_msg
        self.depth_process_event.set()

    def pointcloud_callback(self, pointcloud_msg: PointCloud2) -> None:
        with self.processing_lock:
            self.latest_pointcloud_msg = pointcloud_msg
        self.pointcloud_process_event.set()

    def depth_processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.depth_process_event.wait(timeout=0.1):
                continue
            self.depth_process_event.clear()

            while not self.stop_event.is_set():
                with self.processing_lock:
                    depth_msg = self.latest_depth_msg
                    self.latest_depth_msg = None

                if depth_msg is None:
                    break

                try:
                    depth_meters = convert_depth_to_meters_message(depth_msg)
                    self.last_depth_warning = None
                except ValueError as exc:
                    depth_meters = None
                    self.log_warning_once(
                        'last_depth_warning',
                        f'Cannot decode depth image ({depth_msg.encoding}): {exc}',
                    )

                with self.processing_lock:
                    self.latest_depth_meters = depth_meters

                if not self.depth_process_event.is_set():
                    break
                self.depth_process_event.clear()

    def pointcloud_processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.pointcloud_process_event.wait(timeout=0.1):
                continue
            self.pointcloud_process_event.clear()

            while not self.stop_event.is_set():
                with self.processing_lock:
                    pointcloud_msg = self.latest_pointcloud_msg
                    self.latest_pointcloud_msg = None

                if pointcloud_msg is None:
                    break

                pointcloud_xyz = None
                rotation = None
                translation = None
                try:
                    pointcloud_xyz = extract_organized_xyz(
                        pointcloud_msg,
                        (int(pointcloud_msg.height), int(pointcloud_msg.width)),
                    )
                    rotation, translation = self.lookup_transform(pointcloud_msg)
                    self.last_pointcloud_warning = None
                except TransformException as exc:
                    self.log_warning_once(
                        'last_pointcloud_warning',
                        'Point cloud TF lookup failed; falling back to direct organized '
                        f'cloud coordinates ({exc}).',
                    )
                except ValueError as exc:
                    self.log_warning_once('last_pointcloud_warning', str(exc))

                with self.processing_lock:
                    self.latest_pointcloud_xyz = pointcloud_xyz
                    self.latest_pointcloud_rotation = rotation
                    self.latest_pointcloud_translation = translation

                if not self.pointcloud_process_event.is_set():
                    break
                self.pointcloud_process_event.clear()

    def processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.process_event.wait(timeout=0.1):
                continue
            self.process_event.clear()

            while not self.stop_event.is_set():
                with self.processing_lock:
                    detections_msg = self.latest_detections_msg
                    color_msg = self.latest_color_msg
                    depth_meters = self.latest_depth_meters
                    pointcloud_xyz = self.latest_pointcloud_xyz
                    pointcloud_rotation = self.latest_pointcloud_rotation
                    pointcloud_translation = self.latest_pointcloud_translation

                if detections_msg is None:
                    break

                self.process_measurements(
                    detections_msg,
                    color_msg,
                    depth_meters,
                    pointcloud_xyz,
                    pointcloud_rotation,
                    pointcloud_translation,
                )

                if not self.process_event.is_set():
                    break
                self.process_event.clear()

    def process_measurements(
        self,
        detections_msg: G1Detections,
        color_msg: Image | None,
        depth_meters: np.ndarray | None,
        pointcloud_xyz: np.ndarray | None,
        pointcloud_rotation: np.ndarray | None,
        pointcloud_translation: np.ndarray | None,
    ) -> None:
        batch = batch_from_detections_message(detections_msg)
        blank_depth = np.zeros((batch.image_height, batch.image_width), dtype=np.float32)
        mono_depth_debug = blank_depth
        mono_depth_measurements = None

        if self.depth_anything_enabled and batch.detected and color_msg is not None:
            try:
                frame = convert_color_image_message(color_msg)
                self.last_color_warning = None
            except ValueError as exc:
                self.log_warning_once(
                    'last_color_warning',
                    f'Cannot decode color image ({color_msg.encoding}): {exc}',
                )
            else:
                mono_depth_predicted, _ = self.depth_anything.predict(
                    bgr_frame_to_pil(frame),
                    frame.shape[:2],
                )
                if mono_depth_predicted is not None:
                    mono_depth_debug = mono_depth_predicted
                    mono_depth_measurements = mono_depth_predicted

        add_rgb_measurements(batch, self.camera_config)
        add_depth_measurements(
            batch,
            self.camera_config,
            self.depth_max_meters,
            depth_meters,
            mono_depth_measurements,
        )

        pointcloud_shape_matches = (
            pointcloud_xyz is not None
            and pointcloud_xyz.shape[:2] == (batch.image_height, batch.image_width)
        )
        if pointcloud_shape_matches:
            self.last_pointcloud_shape_warning = None
            add_pointcloud_measurements(
                batch,
                pointcloud_xyz,
                pointcloud_rotation,
                pointcloud_translation,
            )
        elif pointcloud_xyz is not None and pointcloud_xyz.shape[:2] != (batch.image_height, batch.image_width):
            self.log_warning_once(
                'last_pointcloud_shape_warning',
                'Point cloud shape does not match detection image '
                f'({pointcloud_xyz.shape[1]}x{pointcloud_xyz.shape[0]} vs '
                f'{batch.image_width}x{batch.image_height}).',
            )
            add_pointcloud_measurements(batch, None, None, None)
        else:
            add_pointcloud_measurements(batch, None, None, None)

        self.measurement_pub.publish(build_measurements_message(batch, detections_msg.header))
        self.mono_depth_pub.publish(build_float32_image_message(mono_depth_debug, detections_msg.header))

    def lookup_transform(self, pointcloud_msg: PointCloud2):
        rotation, translation, self.last_base_frame_fallback = lookup_transform_components(
            self.tf_buffer,
            self.base_frame,
            pointcloud_msg.header.frame_id,
            Time.from_msg(pointcloud_msg.header.stamp),
            self.get_logger(),
            self.last_base_frame_fallback,
        )
        return rotation, translation

    def log_warning_once(self, attribute_name: str, warning: str) -> None:
        if warning == getattr(self, attribute_name):
            return
        setattr(self, attribute_name, warning)
        self.get_logger().warn(warning)

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.process_event.set()
        self.depth_process_event.set()
        self.pointcloud_process_event.set()
        for thread_name in ('worker_thread', 'depth_worker_thread', 'pointcloud_worker_thread'):
            thread = getattr(self, thread_name, None)
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = G1CameraMeasurementNode()
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
