"""Compatibility entrypoint: persistent environment plus one benchmark config."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('ridgeback_autonomy'),
        'launch',
    )
    shutdown_on_complete = LaunchConfiguration('shutdown_on_complete')

    return LaunchDescription([
        DeclareLaunchArgument(
            'shutdown_on_complete',
            default_value='false',
            description='Shut the wrapper down when the benchmark runner exits',
        ),
        # The config layer comes first so it reads the public world before the
        # persistent layer hands the simulator its native name under the same
        # launch argument. Order does not delay anything: the config layer's
        # nodes wait on the environment's readiness gate either way.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'target_benchmark_config.launch.py')
            ),
            launch_arguments={
                'shutdown_on_complete': shutdown_on_complete,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'target_benchmark_env.launch.py')
            ),
        ),
    ])
