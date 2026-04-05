import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_slam_exploration')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file = os.path.join(pkg_this, 'config', 'slam_toolbox_params.yaml')

    # Same TF remapping pattern used by Nav2 nodes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'),
                  ('/map', 'map'), ('/map_metadata', 'map_metadata')]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace,
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time, 'use_lifecycle_manager': True},
            ],
            remappings=remappings,
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            namespace=namespace,
            parameters=[
                {'autostart': True},
                {'node_names': ['slam_toolbox']},
                {'use_sim_time': use_sim_time},
                {'bond_timeout': 20.0},
                {'bond_respawn_max_duration': 0.0},
            ],
        ),
    ])
