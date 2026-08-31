"""Persistent simulator, visualization, transforms, and target detector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ridgeback_autonomy.perception.target_localization.launch import (
    RAW_DETECTIONS_TOPIC,
    SIMULATION_CAMERA_INPUTS,
    perception_venv_actions,
    resolved_camera_inputs,
)


def build_detector(context, *args, **kwargs):
    """Resolve the compatibility color override before starting the detector."""

    inputs = resolved_camera_inputs(context, 'color_topic')
    return [Node(
        package='ridgeback_autonomy',
        executable='target_detector_node',
        name='target_detector',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'color_topic': inputs.color_image_topic,
            'detections_topic': RAW_DETECTIONS_TOPIC,
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )]


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
        # Shared with target_benchmark_config.launch.py. Keep these defaults
        # byte-identical: the first declaration inherited by a wrapper wins.
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='target_distance_calibration'),
        DeclareLaunchArgument(
            'color_topic', default_value=SIMULATION_CAMERA_INPUTS.color_image_topic),
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

        OpaqueFunction(function=build_detector),
    ])
