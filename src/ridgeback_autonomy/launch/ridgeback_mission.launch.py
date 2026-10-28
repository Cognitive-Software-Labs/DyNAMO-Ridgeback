"""Gazebo/Isaac integration launch for the parking mission.

Frontier motion is off unless ``exploration_enabled`` is set; the mission
manager drives only with ``parking_navigation_enabled``. The wrapper reuses the complete exploration
bringup and adds exactly one Phase 2D mission manager when enabled.
Exploration starts the production explorer and its MissionGuard, which own
``mission/pause`` and active-goal cancellation. The optional stationary
handoff stub represents an idle explorer only, so the two are mutually
exclusive: requesting both fails the launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ridgeback_autonomy.mission.parking_rviz import parking_layout


def _reject_conflicting_handoff(context):
    exploration = LaunchConfiguration('exploration_enabled').perform(context)
    mock = LaunchConfiguration('stationary_mock_handoff_enabled').perform(context)
    if exploration == 'true' and mock == 'true':
        # Both would serve mission/pause; the mock would also claim an idle
        # explorer while the real one owns a Nav2 goal.
        raise RuntimeError(
            'exploration_enabled:=true and stationary_mock_handoff_enabled:=true '
            'are mutually exclusive; the real explorer provides mission/pause')
    return []


def _parking_rviz(context):
    """One RViz with the shared exploration layout plus the parking plan display."""

    if LaunchConfiguration('exploration_rviz').perform(context) != 'true':
        return []
    source = os.path.join(
        get_package_share_directory('ridgeback_autonomy'), 'sim', 'rviz', 'exploration.rviz')
    return [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=LaunchConfiguration('namespace'),
        arguments=['-d', parking_layout(source, 'parking/visualization')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )]


def generate_launch_description():
    package_share = get_package_share_directory('ridgeback_autonomy')
    exploration_launch = os.path.join(
        package_share, 'launch', 'ridgeback_exploration.launch.py')

    backend = LaunchConfiguration('backend')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mission_enabled = LaunchConfiguration('mission_management_enabled')
    mock_handoff_enabled = LaunchConfiguration('stationary_mock_handoff_enabled')
    exploration_enabled = LaunchConfiguration('exploration_enabled')

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='gz', choices=['gz', 'isaac'],
            description='Simulation backend; hardware is intentionally excluded'),
        DeclareLaunchArgument(
            'world', default_value='depot',
            description='Existing backend world to load'),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('setup_path', default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument('exploration_rviz', default_value='true'),
        DeclareLaunchArgument('target_localization_enabled', default_value='true'),
        DeclareLaunchArgument('localization_mode', default_value='local'),
        DeclareLaunchArgument('estimators', default_value='all'),
        DeclareLaunchArgument('detector_debug', default_value='false'),
        DeclareLaunchArgument('gz_gui', default_value='true'),
        DeclareLaunchArgument('headless_rendering', default_value='false'),
        DeclareLaunchArgument(
            'mission_management_enabled', default_value='false',
            choices=['true', 'false'],
            description='Start one non-driving mission manager'),
        DeclareLaunchArgument(
            'stationary_mock_handoff_enabled', default_value='false',
            choices=['true', 'false'],
            description=(
                'Provide an idle test-only mission/pause acknowledgment when the '
                'production explorer is intentionally absent')),
        DeclareLaunchArgument(
            'exploration_enabled', default_value='false',
            choices=['true', 'false'],
            description=(
                'Start the production frontier explorer and its MissionGuard; '
                'mutually exclusive with stationary_mock_handoff_enabled')),
        OpaqueFunction(function=_reject_conflicting_handoff),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument(
            'robot_base_frame', default_value='base_link',
            description='Observed TF base frame; independent of ROS namespace'),
        DeclareLaunchArgument('handoff_timeout_s', default_value='10.0'),
        DeclareLaunchArgument('target_localization_timeout_s', default_value='5.0'),
        DeclareLaunchArgument(
            'parking_navigation_enabled', default_value='false',
            choices=['true', 'false'],
            description=(
                'After a path-feasible parking pose: drive to pre-parking and the '
                'final pose, wait, and return to the start pose')),

        # Evaluated before the include: its launch arguments are set in this
        # context, so afterwards exploration_rviz would always read 'false'.
        OpaqueFunction(function=_parking_rviz),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exploration_launch),
            launch_arguments={
                'backend': backend,
                'world': LaunchConfiguration('world'),
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'setup_path': LaunchConfiguration('setup_path'),
                # RViz is started below with the parking display added.
                'exploration_rviz': 'false',
                'target_localization_enabled': LaunchConfiguration(
                    'target_localization_enabled'),
                'localization_mode': LaunchConfiguration('localization_mode'),
                'base_frame': LaunchConfiguration('robot_base_frame'),
                'estimators': LaunchConfiguration('estimators'),
                'detector_debug': LaunchConfiguration('detector_debug'),
                'gz_gui': LaunchConfiguration('gz_gui'),
                'headless_rendering': LaunchConfiguration('headless_rendering'),
                # Stationary unless exploration is requested explicitly; the
                # explorer is then gated by its own MissionGuard.
                'autonomous_motion_enabled': exploration_enabled,
            }.items(),
        ),

        Node(
            package='ridgeback_autonomy',
            executable='stationary_mission_guard',
            name='stationary_mission_guard',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", mission_enabled, "' == 'true' and '",
                mock_handoff_enabled, "' == 'true' and '",
                exploration_enabled, "' != 'true'",
            ])),
        ),

        Node(
            package='ridgeback_autonomy',
            executable='mission_manager_node.py',
            name='mission_manager_node',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_frame': LaunchConfiguration('map_frame'),
                'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                'handoff_timeout_s': LaunchConfiguration('handoff_timeout_s'),
                'target_localization_timeout_s': LaunchConfiguration(
                    'target_localization_timeout_s'),
                'parking_navigation_enabled': LaunchConfiguration('parking_navigation_enabled'),
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=IfCondition(mission_enabled),
        ),
    ])
