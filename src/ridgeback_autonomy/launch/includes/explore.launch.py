import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    explorer = LaunchConfiguration('explorer')

    explore_lite_params_file = os.path.join(pkg_this, 'config', 'explore_lite_params.yaml')
    frontier_explorer_params_file = os.path.join(pkg_this, 'config', 'frontier_explorer_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('explorer', default_value='explore_lite',
                              description='Which explorer to use: "explore_lite" or "custom"'),

        # Option 1: explore_lite (default)
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            namespace=namespace,
            output='screen',
            parameters=[
                explore_lite_params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
            condition=launch.conditions.LaunchConfigurationEquals('explorer', 'explore_lite'),
        ),

        # Option 2: custom frontier explorer
        Node(
            package='ridgeback_slam_exploration',
            executable='frontier_explorer_node.py',
            name='frontier_explorer_node',
            namespace=namespace,
            output='screen',
            parameters=[
                frontier_explorer_params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
            condition=launch.conditions.LaunchConfigurationEquals('explorer', 'custom'),
        ),
    ])
