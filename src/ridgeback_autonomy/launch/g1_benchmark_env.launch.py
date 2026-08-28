"""Persistent simulator, visualization, transforms, and detector for G1 benchmarks."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ridgeback_autonomy.benchmarking.launch_common import (
    RAW_DETECTIONS_TOPIC,
    perception_venv_actions,
)


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    launch_dir = os.path.join(pkg_this, 'launch')
    includes_dir = os.path.join(launch_dir, 'includes')
    rviz_config = os.path.join(pkg_this, 'sim', 'rviz', 'benchmark.rviz')

    namespace = LaunchConfiguration('namespace')
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    color_topic = LaunchConfiguration('color_topic')
    exploration_rviz = LaunchConfiguration('exploration_rviz')

    return LaunchDescription([
        *perception_venv_actions(pkg_this),
        # Shared with g1_benchmark_config.launch.py. Keep these defaults
        # byte-identical: the first declaration inherited by a wrapper wins.
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='g1_distance_calibration'),
        DeclareLaunchArgument('color_topic', default_value='sensors/camera_0/color/image'),
        DeclareLaunchArgument(
            'setup_path',
            default_value=os.path.expanduser('~/clearpath/'),
        ),
        DeclareLaunchArgument(
            'exploration_rviz',
            default_value='true',
            description='Launch the persistent benchmark RViz2 window',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(exploration_rviz),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(includes_dir, 'simulation.launch.py')
            ),
            launch_arguments={
                'setup_path': setup_path,
                'world': world,
                'clearpath_rviz': 'false',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(includes_dir, 'camera_optical_tf.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                # This include declares the argument without a default.
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        Node(
            package='ridgeback_autonomy',
            executable='g1_detector_node',
            name='g1_detector',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'color_topic': color_topic,
                'detections_topic': RAW_DETECTIONS_TOPIC,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
    ])
