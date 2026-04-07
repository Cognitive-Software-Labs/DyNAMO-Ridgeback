import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_slam_exploration')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')

    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'setup_path',
            default_value=os.path.expanduser('~/clearpath/'),
            description='Path to clearpath config directory containing robot.yaml',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse',
            description='Gazebo world to load',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz',
        ),
        # Add our worlds directory so Gazebo can find custom worlds (e.g. hospital)
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_this, 'worlds'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_clearpath_gz, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'setup_path': setup_path,
                'world': world,
                'rviz': rviz,
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
