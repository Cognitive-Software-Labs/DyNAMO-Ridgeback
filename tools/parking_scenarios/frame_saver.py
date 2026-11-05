"""Save the robot camera's colour frames about once a second, for offline detector review.

Usage: frame_saver.py OUT_DIR [PERIOD_S] [jpg|png]; png is lossless and about 6x larger.
Files are named by the image stamp (ns), so they line up with the recorded detections.
Validation only: nothing reads them back at runtime.
"""
import os, signal, sys
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

out, period = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
ext = sys.argv[3] if len(sys.argv) > 3 else 'jpg'
os.makedirs(out, exist_ok=True)
rclpy.init()
# A blocking spin never runs Python signal handlers; spin_once below lets SIGTERM stop it.
signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
with open(os.path.join(os.path.dirname(os.path.abspath(out)), 'frame_saver.pid'), 'w') as f:
    f.write(str(os.getpid()))
node = rclpy.create_node('parking_frame_saver', namespace='r100_0001')
last = [0]


def save(msg):
    stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    if stamp - last[0] < period * 1e9:
        return
    last[0] = stamp
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding)
    if channels is None:
        return
    img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
    img = img.reshape(msg.height, msg.width, channels)
    if msg.encoding.startswith('rgb'):
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if channels == 4 else cv2.COLOR_RGB2BGR)
    elif channels == 4:
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    cv2.imwrite(f'{out}/{stamp}.{ext}', img, [cv2.IMWRITE_JPEG_QUALITY, 85] if ext == 'jpg' else [])


node.create_subscription(Image, 'sensors/camera_0/color/image', save, qos_profile_sensor_data)
try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
except (KeyboardInterrupt, Exception):
    pass
