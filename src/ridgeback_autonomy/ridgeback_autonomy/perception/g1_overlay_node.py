#!/usr/bin/env python3

from __future__ import annotations

from collections import OrderedDict

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ridgeback_autonomy.common.messages import batch_from_measurements_message
from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.perception.core.image_utils import (
    convert_color_image_message,
    convert_depth_to_meters_message,
)
from ridgeback_autonomy.perception.core.rendering import RgbdOverlayRenderer


CAMERA_MEASUREMENTS_TOPIC = 'measurements/g1/camera'
LIDAR_MEASUREMENTS_TOPIC = 'measurements/g1/lidar'
MONO_DEPTH_DEBUG_TOPIC = 'debug/g1/camera/mono_depth'
DEPTH_MAX_METERS_DEFAULT = 10.0
RENDER_FPS_DEFAULT = 15.0


class G1OverlayNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_overlay_node')

        self.declare_parameter('measurement_topic', CAMERA_MEASUREMENTS_TOPIC)
        self.declare_parameter('lidar_measurement_topic', LIDAR_MEASUREMENTS_TOPIC)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('depth_topic', 'sensors/camera_0/depth/image')
        self.declare_parameter('mono_depth_debug_topic', MONO_DEPTH_DEBUG_TOPIC)
        self.declare_parameter('depth_max_meters', DEPTH_MAX_METERS_DEFAULT)
        self.declare_parameter('render_fps', RENDER_FPS_DEFAULT)
        self.declare_parameter('window_name', 'G1 Perception')

        self.measurement_topic = self.get_parameter('measurement_topic').value
        self.lidar_measurement_topic = self.get_parameter('lidar_measurement_topic').value
        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.mono_depth_debug_topic = self.get_parameter('mono_depth_debug_topic').value
        self.depth_max_meters = float(self.get_parameter('depth_max_meters').value)
        self.render_fps = max(1.0, float(self.get_parameter('render_fps').value))
        self.window_name = self.get_parameter('window_name').value

        self.renderer = RgbdOverlayRenderer(self.depth_max_meters)
        self.latest_measurements_msg: G1Measurements | None = None
        self.lidar_measurement_cache: OrderedDict[tuple, G1Measurements] = OrderedDict()
        self.latest_color_msg: Image | None = None
        self.latest_depth_msg: Image | None = None
        self.latest_mono_depth_msg: Image | None = None
        self.last_color_warning: str | None = None
        self.last_depth_warning: str | None = None
        self.last_mono_depth_warning: str | None = None

        self.measurement_subscription = self.create_subscription(
            G1Measurements,
            self.measurement_topic,
            self.measurement_callback,
            10,
        )
        self.lidar_measurement_subscription = self.create_subscription(
            G1Measurements,
            self.lidar_measurement_topic,
            self.lidar_measurement_callback,
            10,
        )
        self.color_subscription = self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.mono_depth_subscription = self.create_subscription(
            Image,
            self.mono_depth_debug_topic,
            self.mono_depth_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.render_timer = self.create_timer(1.0 / self.render_fps, self.render_callback)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        cv2.moveWindow(self.window_name, 60, 60)

    def measurement_callback(self, measurements_msg: G1Measurements) -> None:
        self.latest_measurements_msg = measurements_msg
        self.render_latest()

    def lidar_measurement_callback(self, measurements_msg: G1Measurements) -> None:
        key = self.measurement_message_key(measurements_msg)
        self.lidar_measurement_cache[key] = measurements_msg
        self.lidar_measurement_cache.move_to_end(key)
        while len(self.lidar_measurement_cache) > 32:
            self.lidar_measurement_cache.popitem(last=False)

    def color_callback(self, color_msg: Image) -> None:
        self.latest_color_msg = color_msg

    def depth_callback(self, depth_msg: Image) -> None:
        self.latest_depth_msg = depth_msg

    def mono_depth_callback(self, mono_depth_msg: Image) -> None:
        self.latest_mono_depth_msg = mono_depth_msg

    def render_callback(self) -> None:
        cv2.waitKey(1)

    def render_latest(self) -> None:
        if self.latest_measurements_msg is None or self.latest_color_msg is None:
            return

        try:
            frame = convert_color_image_message(self.latest_color_msg)
            self.last_color_warning = None
        except ValueError as exc:
            self.log_warning_once('last_color_warning', f'Cannot decode color image ({self.latest_color_msg.encoding}): {exc}')
            return

        sensor_depth_meters = None
        mono_depth_meters = None
        if self.latest_depth_msg is not None:
            try:
                sensor_depth_meters = convert_depth_to_meters_message(self.latest_depth_msg)
                self.last_depth_warning = None
            except ValueError as exc:
                self.log_warning_once('last_depth_warning', f'Cannot decode depth image ({self.latest_depth_msg.encoding}): {exc}')

        if self.latest_mono_depth_msg is not None:
            try:
                mono_depth_meters = convert_depth_to_meters_message(self.latest_mono_depth_msg)
                self.last_mono_depth_warning = None
            except ValueError as exc:
                self.log_warning_once(
                    'last_mono_depth_warning',
                    f'Cannot decode mono-depth debug image ({self.latest_mono_depth_msg.encoding}): {exc}',
                )

        batch = batch_from_measurements_message(self.latest_measurements_msg)
        self.merge_lidar_measurements(batch, self.latest_measurements_msg)
        annotated = self.renderer.render(
            frame,
            sensor_depth_meters,
            mono_depth_meters,
            batch,
            None,
            None,
        )
        cv2.imshow(self.window_name, annotated)
        cv2.waitKey(1)

    def merge_lidar_measurements(
        self,
        batch,
        camera_msg: G1Measurements,
    ) -> None:
        lidar_msg = self.lidar_measurement_cache.get(self.measurement_message_key(camera_msg))
        if lidar_msg is None:
            return

        lidar_batch = batch_from_measurements_message(lidar_msg)
        for detection, lidar_detection in zip(batch.detections, lidar_batch.detections):
            detection.lidar_lateral_m = lidar_detection.lidar_lateral_m
            detection.lidar_forward_m = lidar_detection.lidar_forward_m
            detection.lidar_distance_m = lidar_detection.lidar_distance_m

    def measurement_message_key(
        self,
        msg: G1Measurements,
    ) -> tuple:
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
