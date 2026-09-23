"""Hardware provider with an attach-only safe default.

The Clearpath services normally own platform, sensor, and camera bringup on the
robot. This adapter starts nothing unless ``start_platform`` is explicitly set;
the common launch readiness gates then verify the external ROS contract.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ridgeback_autonomy.namespace_resolution import resolve_namespace


def _resolve_hardware_namespace(context):
    resolved = resolve_namespace(
        backend='hardware',
        requested=LaunchConfiguration('namespace').perform(context),
        setup_path=LaunchConfiguration('setup_path').perform(context),
    )
    return [
        SetLaunchConfiguration('namespace', resolved),
        LogInfo(msg=f'Hardware backend using ROS namespace: {resolved}'),
    ]


def generate_launch_description():
    start_platform = LaunchConfiguration('start_platform')
    setup_path = LaunchConfiguration('setup_path')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS namespace override; empty reads setup_path/robot.yaml'),
        OpaqueFunction(function=_resolve_hardware_namespace),
        DeclareLaunchArgument(
            'start_platform', default_value='false', choices=['true', 'false'],
            description='Explicitly start Clearpath platform/control bringup; '
                        'false safely attaches to externally managed hardware'),
        LogInfo(
            msg='Hardware backend: attaching to existing platform and sensor bringup',
            condition=UnlessCondition(start_platform),
        ),
        LogInfo(
            msg='Hardware backend: explicitly starting Clearpath platform bringup',
            condition=IfCondition(start_platform),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('clearpath_common'), 'launch', 'platform.launch.py'])),
            condition=IfCondition(start_platform),
            launch_arguments={
                'setup_path': setup_path,
                'namespace': namespace,
                'use_sim_time': 'false',
                'enable_ekf': 'true',
            }.items(),
        ),
    ])
