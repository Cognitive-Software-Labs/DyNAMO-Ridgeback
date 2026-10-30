#!/usr/bin/env python3
"""Republish Gazebo 2D LiDAR scans with their corrupted rays marked invalid.

The installed gz-rendering GPU rays do not measure two bins of the 1081-bin,
270-degree scan correctly. The first ray (-135 degrees) carries a range from
about 90 degrees away, next to the -45 degree cube-map face seam; SLAM
integrated those endpoints as phantom obstacles, one inside the robot's start
footprint. Bin 360, on that -45 degree seam, returns phantom short ranges in
roughly 10-26% of scans, which vetoed parking legs. NaN marks a sample
erroneous (REP 117); SLAM, Nav2 and this repository's scan consumers already
skip non-finite ranges. Nothing else in the scan changes.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


# Bin indices of the 1081-bin scan that gz-rendering corrupts: -135 and -45 degrees.
CORRUPTED_BINS = (0, 360)


def invalidate_first_ray(msg):
    """Mark the corrupted bins erroneous in place and return ``msg``."""
    for index in CORRUPTED_BINS:
        if index < len(msg.ranges):
            msg.ranges[index] = math.nan
    return msg


class LidarFirstRayGuard(Node):
    def __init__(self):
        super().__init__('lidar_first_ray_guard')
        self.declare_parameter('scans', ['lidar2d_0'])
        self._relays = []
        for name in self.get_parameter('scans').value:
            # Same reliable depth-10 profile as the bridge it replaces.
            publisher = self.create_publisher(LaserScan, f'{name}/scan', 10)
            self._relays.append(self.create_subscription(
                LaserScan, f'{name}/scan_gz',
                lambda msg, publisher=publisher: publisher.publish(invalidate_first_ray(msg)),
                qos_profile_sensor_data))


def main():
    rclpy.init()
    node = LidarFirstRayGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
