#!/usr/bin/env python3
"""Publish an isolated contract fixture on domain 99; never source robot DDS.

Run alongside camera_contract_check --backend gz --namespace latency_smoke
--base-frame base_link --timing --warmup 2 --duration 3 --no-scans.
"""
import time
import signal
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

running = True
def stop(*_args):
    global running
    running = False
signal.signal(signal.SIGTERM, stop)
rclpy.init()
node = rclpy.create_node('sensor_latency_fixture')
prefix = '/latency_smoke/sensors/camera_0/'
pubs = [node.create_publisher(kind, prefix + topic, qos_profile_sensor_data)
        for kind, topic in [(Image, 'color/image'), (Image, 'depth/image'), (CameraInfo, 'color/camera_info')]]
tfpub = node.create_publisher(TFMessage, '/latency_smoke/tf', 10)
color = Image(height=480, width=640, encoding='rgb8', step=1920, data=bytes(640*480*3))
depth = Image(height=480, width=640, encoding='16UC1', step=1280, data=bytes(640*480*2))
info = CameraInfo(height=480, width=640)
transform = TransformStamped(child_frame_id='camera_0_color_optical_frame')
transform.header.frame_id = 'base_link'
transform.transform.rotation.w = 1.
end = time.monotonic() + 15
try:
    while running and time.monotonic() < end:
        tick = time.monotonic()
        stamp = node.get_clock().now().to_msg()
        for message, pub in zip((color, depth, info), pubs):
            message.header.stamp = stamp
            message.header.frame_id = transform.child_frame_id
            pub.publish(message)
        transform.header.stamp = stamp
        tfpub.publish(TFMessage(transforms=[transform]))
        time.sleep(max(0, 1/30 - (time.monotonic() - tick)))
finally:
    node.destroy_node()
    rclpy.shutdown()
