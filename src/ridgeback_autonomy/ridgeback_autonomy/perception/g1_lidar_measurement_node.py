#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from ridgeback_autonomy.common.camera_config import load_camera_config
from ridgeback_autonomy.common.messages import (
    batch_from_detections_message,
    build_measurements_message,
)
from ridgeback_autonomy.common.tf_utils import lookup_transform_components
from ridgeback_autonomy.msg import G1Detections, G1Measurements
from ridgeback_autonomy.perception.core.geometry import (
    add_lidar_measurements,
    extract_scan_points_base,
)


RAW_DETECTIONS_TOPIC = 'detections/g1/raw'
LIDAR_MEASUREMENTS_TOPIC = 'measurements/g1/lidar'


class G1LidarMeasurementNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_lidar_measurement_node')

        pkg_share = get_package_share_directory('ridgeback_autonomy')
        default_config = os.path.join(pkg_share, 'config', 'camera_config.json')
        default_base_frame = self.default_base_frame()

        self.declare_parameter('camera_config_path', default_config)
        self.declare_parameter('detections_topic', RAW_DETECTIONS_TOPIC)
        self.declare_parameter('measurement_topic', LIDAR_MEASUREMENTS_TOPIC)
        self.declare_parameter('scan_topic', 'sensors/lidar2d_0/scan')
        self.declare_parameter('base_frame', default_base_frame)

        self.camera_config_path = self.get_parameter('camera_config_path').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.measurement_topic = self.get_parameter('measurement_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.base_frame = self.get_parameter('base_frame').value or default_base_frame

        self.camera_config = load_camera_config(self.camera_config_path)
        self.camera_hfov_rad = math.radians(self.camera_config.depth_hfov_deg)
        self.last_lidar_warning = None
        self.last_base_frame_fallback = None

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.latest_detections_msg: G1Detections | None = None
        self.latest_scan_msg: LaserScan | None = None
        self.latest_scan_points = None
        self.processing_lock = threading.Lock()
        self.process_event = threading.Event()
        self.scan_process_event = threading.Event()
        self.stop_event = threading.Event()

        self.create_subscription(
            G1Detections,
            self.detections_topic,
            self.detections_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.measurement_pub = self.create_publisher(
            G1Measurements,
            self.measurement_topic,
            10,
        )

        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.scan_worker_thread = threading.Thread(target=self.scan_processing_loop, daemon=True)
        self.worker_thread.start()
        self.scan_worker_thread.start()

    def default_base_frame(self) -> str:
        return 'base_link'

    def detections_callback(self, detections_msg: G1Detections) -> None:
        with self.processing_lock:
            self.latest_detections_msg = detections_msg
        self.process_event.set()

    def scan_callback(self, scan_msg: LaserScan) -> None:
        with self.processing_lock:
            self.latest_scan_msg = scan_msg
        self.scan_process_event.set()

    def scan_processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.scan_process_event.wait(timeout=0.1):
                continue
            self.scan_process_event.clear()

            while not self.stop_event.is_set():
                with self.processing_lock:
                    scan_msg = self.latest_scan_msg
                    self.latest_scan_msg = None

                if scan_msg is None:
                    break

                scan_points = None
                try:
                    scan_points = self.extract_scan_points(scan_msg)
                    self.last_lidar_warning = None
                except (TransformException, ValueError) as exc:
                    self.log_lidar_warning(str(exc))

                with self.processing_lock:
                    self.latest_scan_points = scan_points

                if not self.scan_process_event.is_set():
                    break
                self.scan_process_event.clear()

    def processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.process_event.wait(timeout=0.1):
                continue
            self.process_event.clear()

            while not self.stop_event.is_set():
                with self.processing_lock:
                    detections_msg = self.latest_detections_msg
                    scan_points = self.latest_scan_points

                if detections_msg is None:
                    break

                self.process_measurements(detections_msg, scan_points)

                if not self.process_event.is_set():
                    break
                self.process_event.clear()

    def process_measurements(self, detections_msg: G1Detections, scan_points) -> None:
        batch = batch_from_detections_message(detections_msg)
        rotation = None
        translation = None
        if scan_points is not None:
            try:
                rotation, translation, self.last_base_frame_fallback = lookup_transform_components(
                    self.tf_buffer,
                    self.base_frame,
                    detections_msg.header.frame_id,
                    Time.from_msg(detections_msg.header.stamp),
                    self.get_logger(),
                    self.last_base_frame_fallback,
                )
            except TransformException as exc:
                self.get_logger().error(
                    f'Lidar measurement skipped: TF lookup from '
                    f'"{detections_msg.header.frame_id}" to "{self.base_frame}" failed: {exc}. '
                    f'In simulation, ensure the camera_0_color_optical_tf '
                    f'static_transform_publisher is running (requires use_sim_time:=true). '
                    f'On real hardware, ensure the RealSense driver is publishing camera TF.'
                )
        add_lidar_measurements(
            batch,
            scan_points,
            self.camera_hfov_rad,
            rotation,
            translation,
        )
        self.measurement_pub.publish(build_measurements_message(batch, detections_msg.header))

    def extract_scan_points(self, scan_msg: LaserScan):
        rotation, translation, self.last_base_frame_fallback = lookup_transform_components(
            self.tf_buffer,
            self.base_frame,
            scan_msg.header.frame_id,
            Time.from_msg(scan_msg.header.stamp),
            self.get_logger(),
            self.last_base_frame_fallback,
        )
        return extract_scan_points_base(scan_msg, rotation, translation)

    def log_lidar_warning(self, warning: str) -> None:
        if warning == self.last_lidar_warning:
            return
        self.last_lidar_warning = warning
        self.get_logger().warn(warning)

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.process_event.set()
        self.scan_process_event.set()
        for thread_name in ('worker_thread', 'scan_worker_thread'):
            thread = getattr(self, thread_name, None)
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = G1LidarMeasurementNode()
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
