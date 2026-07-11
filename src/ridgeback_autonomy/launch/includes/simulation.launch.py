import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution, LaunchConfiguration,
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')

    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    clearpath_rviz = LaunchConfiguration('clearpath_rviz')
    sim = LaunchConfiguration('sim')

    return LaunchDescription([
        # transient dispatch during the Isaac port: gz stays the default
        # until P8 removes it
        DeclareLaunchArgument(
            'sim',
            default_value='gz',
            choices=['gz', 'isaac'],
            description='Simulation backend',
        ),
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
            'clearpath_rviz',
            default_value='false',
            description='Launch the Clearpath-provided RViz instance',
        ),
        DeclareLaunchArgument(
            'gz_gui',
            default_value='true',
            description='Launch the Gazebo GUI window',
        ),
        DeclareLaunchArgument(
            'headless_rendering',
            default_value='false',
            description='Render server sensors via EGL without an X display '
                        '(GPU rendering for non-seat/SSH sessions; see ISSUES.md)',
        ),
        # Add our worlds and models directories so Gazebo can find them
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_this, 'sim', 'worlds'),
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_this, 'sim', 'models'),
        ),
        AppendEnvironmentVariable(
            'GZ_GUI_PLUGIN_PATH',
            os.path.join(os.path.abspath(os.path.join(pkg_this, '..', '..')), 'lib', 'ridgeback_autonomy'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_clearpath_gz, 'launch', 'simulation.launch.py')
            ),
            condition=UnlessCondition(
                EqualsSubstitution(sim, 'isaac')),
            launch_arguments={
                'setup_path': setup_path,
                'world': world,
                'rviz': clearpath_rviz,
                'use_sim_time': 'true',
                'gz_gui': LaunchConfiguration('gz_gui'),
                'headless_rendering': LaunchConfiguration('headless_rendering'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_this, 'launch', 'includes',
                             'simulation_isaac.launch.py')
            ),
            condition=IfCondition(EqualsSubstitution(sim, 'isaac')),
            launch_arguments={
                'setup_path': setup_path,
                'world': world,
                # headless_rendering arg maps to Isaac headless; the
                # gz_gui concept has no Isaac equivalent (P8 retires it)
            }.items(),
        ),
    ])
