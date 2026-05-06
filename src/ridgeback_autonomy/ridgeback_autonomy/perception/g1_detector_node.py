#!/usr/bin/env python3

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ridgeback_autonomy.common.messages import build_detections_message
from ridgeback_autonomy.msg import G1Detections
from ridgeback_autonomy.perception.core.detection import (
    DETECTION_MODEL_DEFAULT,
    DETECTION_THRESHOLD,
    OwlV2Detector,
    parse_owl_detections,
)
from ridgeback_autonomy.perception.core.image_utils import (
    bgr_frame_to_pil,
    convert_color_image_message,
)


RAW_DETECTIONS_TOPIC = 'detections/g1/raw'
DETECTOR_FPS_DEFAULT = 5.0


class G1DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_detector_node')

        self.declare_parameter('detection_model', DETECTION_MODEL_DEFAULT)
        self.declare_parameter('detection_threshold', DETECTION_THRESHOLD)
        self.declare_parameter('detector_fps', DETECTOR_FPS_DEFAULT)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('detections_topic', RAW_DETECTIONS_TOPIC)

        self.detection_model = self.get_parameter('detection_model').value
        self.detection_threshold = float(self.get_parameter('detection_threshold').value)
        self.detector_fps = max(0.1, float(self.get_parameter('detector_fps').value))
        self.detector_period = 1.0 / self.detector_fps
        self.color_topic = self.get_parameter('color_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value

        self.detector = OwlV2Detector(self.detection_model, self.get_logger())
        self.detector.load()
        self.last_detection_time = 0.0
        self.latest_color_msg: Image | None = None
        self.processing_lock = threading.Lock()
        self.process_event = threading.Event()
        self.stop_event = threading.Event()

        self.detections_pub = self.create_publisher(
            G1Detections,
            self.detections_topic,
            10,
        )
        self.create_subscription(
            Image,
            self.color_topic,
            self.on_color_image,
            qos_profile_sensor_data,
        )
        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info(f'Subscribed to color stream: {self.color_topic}')
        self.get_logger().info(f'Publishing raw detections on: {self.detections_topic}')
        self.get_logger().info(f'Running detector at up to {self.detector_fps:.1f} FPS')

    def on_color_image(self, color_msg: Image) -> None:
        with self.processing_lock:
            self.latest_color_msg = color_msg
        self.process_event.set()

    def processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.process_event.wait(timeout=0.1):
                continue
            self.process_event.clear()

            next_allowed_time = self.last_detection_time + self.detector_period
            wait_time = next_allowed_time - time.monotonic()
            if wait_time > 0.0 and self.stop_event.wait(wait_time):
                break

            with self.processing_lock:
                color_msg = self.latest_color_msg
                self.latest_color_msg = None

            if color_msg is None:
                continue

            self.process_color_image(color_msg)
            self.last_detection_time = time.monotonic()

    def process_color_image(self, color_msg: Image) -> None:
        try:
            frame = convert_color_image_message(color_msg)
        except ValueError as exc:
            self.get_logger().warn(f'Cannot decode color image ({color_msg.encoding}): {exc}')
            return

        outputs = self.detector.detect(bgr_frame_to_pil(frame))
        batch = parse_owl_detections(
            outputs,
            frame.shape[1],
            frame.shape[0],
            self.detection_threshold,
        )
        self.detections_pub.publish(build_detections_message(batch, color_msg.header))

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.process_event.set()
        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = G1DetectorNode()
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
