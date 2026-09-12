"""Gazebo provider for the backend-neutral Ridgeback autonomy stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from ridgeback_autonomy.common.camera_profiles import (
    CAMERA_PROFILE_CHOICES,
    DEFAULT_CAMERA_PROFILE,
    resolve_camera_profile,
)


def _configure_camera_profile(context):
    profile = resolve_camera_profile(
        LaunchConfiguration('camera_profile').perform(context))
    return [
        SetLaunchConfiguration('sim_camera_width', str(profile.width)),
        SetLaunchConfiguration('sim_camera_height', str(profile.height)),
        SetLaunchConfiguration(
            'sim_camera_horizontal_fov', str(profile.horizontal_fov_rad)),
    ]


def generate_launch_description():
    pkg_adapter = get_package_share_directory('ridgeback_autonomy_gz')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')

    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    gz_gui = LaunchConfiguration('gz_gui')
    headless_rendering = LaunchConfiguration('headless_rendering')

    return LaunchDescription([
        DeclareLaunchArgument('setup_path', default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument('world', default_value='mock_hospital'),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('clearpath_rviz', default_value='false'),
        DeclareLaunchArgument('gz_gui', default_value='true'),
        DeclareLaunchArgument('headless_rendering', default_value='false'),
        DeclareLaunchArgument(
            'camera_profile', default_value=DEFAULT_CAMERA_PROFILE,
            choices=list(CAMERA_PROFILE_CHOICES),
            description='Nominal D455 simulation profile'),
        OpaqueFunction(function=_configure_camera_profile),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_adapter, 'sim', 'worlds')),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_adapter, 'sim', 'models')),
        AppendEnvironmentVariable(
            'GZ_GUI_PLUGIN_PATH',
            os.path.join(os.path.abspath(os.path.join(pkg_adapter, '..', '..')),
                         'lib', 'ridgeback_autonomy_gz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_clearpath_gz, 'launch', 'simulation.launch.py')),
            launch_arguments={
                'setup_path': setup_path,
                'world': world,
                'rviz': LaunchConfiguration('clearpath_rviz'),
                'use_sim_time': 'true',
                'headless_rendering': PythonExpression([
                    "'false' if '", gz_gui, "' == 'true' and '",
                    headless_rendering, "' == 'false' else 'true'",
                ]),
            }.items(),
        ),
    ])
