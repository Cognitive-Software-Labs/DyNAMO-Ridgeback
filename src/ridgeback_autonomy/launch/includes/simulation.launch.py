"""Compatibility dispatcher for the optional backend adapter packages.

New callers select ``backend``. The old ``sim`` argument remains as the default
source so existing ``sim:=gz`` and ``sim:=isaac`` commands keep working during
the migration. Package lookup happens only for the selected backend; the core
package therefore has no runtime dependency on unselected simulators.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from ridgeback_common.camera_profiles import (
    CAMERA_PROFILE_CHOICES,
    DEFAULT_CAMERA_PROFILE,
    DEFAULT_DEPTH_FIDELITY,
    DEPTH_FIDELITY_CHOICES,
)


ADAPTER_PACKAGES = {
    'gz': 'ridgeback_autonomy_gz',
    'isaac': 'ridgeback_autonomy_isaac',
    'hardware': 'ridgeback_autonomy_hardware',
}

# Adapter argument names map to their public compatibility-launch names. Most
# are identical; hardware deliberately exposes ``start_hardware_platform`` to
# callers while its adapter retains the shorter ``start_platform`` name.
ADAPTER_LAUNCH_ARGUMENTS = {
    'gz': {
        'setup_path': 'setup_path',
        'world': 'world',
        'namespace': 'namespace',
        'clearpath_rviz': 'clearpath_rviz',
        'gz_gui': 'gz_gui',
        'headless_rendering': 'headless_rendering',
        'camera_profile': 'camera_profile',
    },
    'isaac': {
        'setup_path': 'setup_path',
        'world': 'world',
        'namespace': 'namespace',
        'rtf': 'rtf',
        'headless': 'headless',
        'livestream': 'livestream',
        'odom_noise': 'odom_noise',
        'noise_seed': 'noise_seed',
        'camera': 'camera',
        'sim_mode': 'sim_mode',
        'sensor_hz': 'sensor_hz',
        'camera_profile': 'camera_profile',
        'depth_fidelity': 'depth_fidelity',
    },
    'hardware': {
        'setup_path': 'setup_path',
        'namespace': 'namespace',
        'start_platform': 'start_hardware_platform',
    },
}


def _include_selected_backend(context):
    backend = LaunchConfiguration('backend').perform(context)
    package = ADAPTER_PACKAGES[backend]
    adapter_launch = os.path.join(
        get_package_share_directory(package), 'launch', 'backend.launch.py')
    arguments = {
        adapter_name: LaunchConfiguration(public_name)
        for adapter_name, public_name in ADAPTER_LAUNCH_ARGUMENTS[backend].items()
    }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(adapter_launch),
        launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    legacy_sim = LaunchConfiguration('sim')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='gz', choices=['gz', 'isaac'],
                              description='Deprecated compatibility alias for backend'),
        DeclareLaunchArgument(
            'backend', default_value=legacy_sim, choices=sorted(ADAPTER_PACKAGES),
            description='I/O provider: Gazebo, Isaac Sim, or physical hardware'),
        DeclareLaunchArgument(
            'setup_path',
            default_value=PythonExpression([
                "'/etc/clearpath/' if '", LaunchConfiguration('backend'),
                "' == 'hardware' else '", os.path.expanduser('~/clearpath/'), "'",
            ]),
        ),
        DeclareLaunchArgument('world', default_value='initial_test_world'),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('clearpath_rviz', default_value='false'),
        DeclareLaunchArgument('gz_gui', default_value='true'),
        DeclareLaunchArgument('headless_rendering', default_value='false'),
        DeclareLaunchArgument('rtf', default_value='1.0'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('livestream', default_value='false'),
        DeclareLaunchArgument('odom_noise', default_value='1.0'),
        DeclareLaunchArgument('noise_seed', default_value='0'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_profile', default_value=DEFAULT_CAMERA_PROFILE,
                              choices=CAMERA_PROFILE_CHOICES),
        DeclareLaunchArgument('depth_fidelity', default_value=DEFAULT_DEPTH_FIDELITY,
                              choices=DEPTH_FIDELITY_CHOICES),
        DeclareLaunchArgument('sim_mode', default_value='realtime'),
        DeclareLaunchArgument('sensor_hz', default_value='40.0'),
        DeclareLaunchArgument('start_hardware_platform', default_value='false'),
        OpaqueFunction(function=_include_selected_backend),
    ])
