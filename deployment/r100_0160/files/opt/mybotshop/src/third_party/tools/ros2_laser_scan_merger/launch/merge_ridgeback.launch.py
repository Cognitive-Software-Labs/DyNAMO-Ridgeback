#!/usr/bin/env python3
"""Ridgeback scan merger with independently recovering ROS child processes.

This is the locally maintained copy of MyBotShop's merge_ridgeback.launch.py.
The installed service still supplies the calibrated params_ridgeback.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    namespace = os.environ.get("MMP_NS", "r100_0160")
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    config = os.path.join(
        get_package_share_directory("ros2_laser_scan_merger"),
        "config",
        "params_ridgeback.yaml",
    )

    merger = Node(
        namespace=namespace,
        package="ros2_laser_scan_merger",
        executable="ros2_laser_scan_merger",
        parameters=[config],
        remappings=remappings,
        output="screen",
        respawn=True,
        respawn_delay=2,
        additional_env={"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"},
    )
    converter = Node(
        namespace=namespace,
        name="pointcloud_to_laserscan",
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        parameters=[config],
        remappings=remappings + [("scan", "sensors/scan")],
        output="screen",
        respawn=True,
        respawn_delay=2,
        additional_env={"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"},
    )
    return LaunchDescription([merger, converter])
