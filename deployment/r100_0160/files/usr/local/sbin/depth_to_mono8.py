#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

class DepthToMono(Node):
    def __init__(self):
        super().__init__('depth_to_mono8')
        self.pub = self.create_publisher(Image, '/r100_0160/camera/depth/image_rect_mono8', qos_profile_sensor_data)
        self.sub = self.create_subscription(Image, '/r100_0160/camera/depth/image_rect_raw', self.cb, qos_profile_sensor_data)
        self.min_mm = 400.0
        self.max_mm = 4000.0
    def cb(self, msg):
        if msg.encoding not in ('16UC1', 'mono16'):
            return
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
        clipped = np.clip(arr.astype(np.float32), self.min_mm, self.max_mm)
        mono = ((self.max_mm - clipped) * 255.0 / (self.max_mm - self.min_mm)).astype(np.uint8)
        mono[arr == 0] = 0
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = 'mono8'
        out.is_bigendian = 0
        out.step = msg.width
        out.data = mono.tobytes()
        self.pub.publish(out)

def main():
    rclpy.init()
    node = DepthToMono()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
