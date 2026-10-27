"""Non-driving Gazebo/Isaac integration launch for the parking mission prefix.

This wrapper deliberately forces frontier motion off.  It reuses the complete
exploration bringup and adds exactly one Phase 2D mission manager when enabled.
The optional stationary handoff stub represents an idle explorer only; active
goal cancellation remains the responsibility of the production MissionGuard.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('ridgeback_autonomy')
    exploration_launch = os.path.join(
        package_share, 'launch', 'ridgeback_exploration.launch.py')

    backend = LaunchConfiguration('backend')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mission_enabled = LaunchConfiguration('mission_management_enabled')
    mock_handoff_enabled = LaunchConfiguration('stationary_mock_handoff_enabled')

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
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument(
            'robot_base_frame', default_value='base_link',
            description='Observed TF base frame; independent of ROS namespace'),
        DeclareLaunchArgument('handoff_timeout_s', default_value='10.0'),
        DeclareLaunchArgument('target_localization_timeout_s', default_value='5.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exploration_launch),
            launch_arguments={
                'backend': backend,
                'world': LaunchConfiguration('world'),
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'setup_path': LaunchConfiguration('setup_path'),
                'exploration_rviz': LaunchConfiguration('exploration_rviz'),
                'target_localization_enabled': LaunchConfiguration(
                    'target_localization_enabled'),
                'localization_mode': LaunchConfiguration('localization_mode'),
                'base_frame': LaunchConfiguration('robot_base_frame'),
                'estimators': LaunchConfiguration('estimators'),
                'detector_debug': LaunchConfiguration('detector_debug'),
                'gz_gui': LaunchConfiguration('gz_gui'),
                'headless_rendering': LaunchConfiguration('headless_rendering'),
                # Phase 2D.5 is stationary by construction.  This is not a
                # caller-controlled argument in this wrapper.
                'autonomous_motion_enabled': 'false',
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
                mock_handoff_enabled, "' == 'true'",
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
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=IfCondition(mission_enabled),
        ),
    ])
