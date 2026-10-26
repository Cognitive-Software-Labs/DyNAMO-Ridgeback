#!/usr/bin/env python3

from __future__ import annotations

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class Camera1PointcloudFrameNode(Node):
    """Republish camera_1 Gazebo point clouds with the frame their data uses."""

    def __init__(self) -> None:
        super().__init__('camera1_pointcloud_frame_node')

        self.declare_parameter('raw_topic', 'sensors/camera_1/points')
        self.declare_parameter('corrected_topic', 'sensors/camera_1/points_corrected')
        self.declare_parameter('corrected_frame_id', 'camera_1_link')

        self.raw_topic = str(self.get_parameter('raw_topic').value)
        self.corrected_topic = str(self.get_parameter('corrected_topic').value)
        self.corrected_frame_id = str(self.get_parameter('corrected_frame_id').value)
        self._logged_first_message = False

        self.publisher = self.create_publisher(
            PointCloud2,
            self.corrected_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.raw_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Camera 1 point cloud frame correction: '
            f'{self.raw_topic} -> {self.corrected_topic}, '
            f'frame_id={self.corrected_frame_id}'
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        corrected = PointCloud2()
        corrected.header = Header()
        corrected.header.stamp = msg.header.stamp
        corrected.header.frame_id = self.corrected_frame_id
        corrected.height = msg.height
        corrected.width = msg.width
        corrected.fields = msg.fields
        corrected.is_bigendian = msg.is_bigendian
        corrected.point_step = msg.point_step
        corrected.row_step = msg.row_step
        corrected.data = msg.data
        corrected.is_dense = msg.is_dense

        self.publisher.publish(corrected)

        if not self._logged_first_message:
            self.get_logger().info(
                'Republished first corrected camera_1 cloud: '
                f'raw_frame={msg.header.frame_id}, '
                f'corrected_frame={self.corrected_frame_id}, '
                f'size={msg.width}x{msg.height}, point_step={msg.point_step}'
            )
            self._logged_first_message = True


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Camera1PointcloudFrameNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
