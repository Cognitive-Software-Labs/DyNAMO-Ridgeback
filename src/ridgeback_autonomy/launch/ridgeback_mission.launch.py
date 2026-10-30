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
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ridgeback_autonomy.mission.parking_rviz import parking_layout
from ridgeback_autonomy.mission.simulation_config import demo_world, simulation_setup_path


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


def _configure_simulation(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    backend = value('backend')
    demo = value('demo_scene_enabled') == 'true'
    selected = simulation_setup_path(get_package_share_directory('ridgeback_autonomy'),
        value('setup_path'), backend=backend, namespace=value('namespace'), demo=demo)
    if value('use_sim_time') != 'true':
        raise ValueError('The simulation-only mission requires use_sim_time:=true')
    if demo:
        if value('world') != demo_world(backend):
            raise ValueError(f'{backend} demo requires world:={demo_world(backend)}; '
                             'other worlds may already contain a G1')
        if value('mission_management_enabled') != 'true':
            raise ValueError('demo_scene_enabled requires mission_management_enabled:=true')
        if value('demo_plan_only') == 'true' and (
                value('exploration_enabled') != 'false'
                or value('stationary_mock_handoff_enabled') != 'true'
                or value('parking_navigation_enabled') != 'false'):
            raise ValueError('demo_plan_only requires exploration_enabled:=false, '
                             'stationary_mock_handoff_enabled:=true, parking_navigation_enabled:=false')
    actions = [SetLaunchConfiguration('setup_path', selected),
               LogInfo(msg=f'Simulation robot declaration: {selected}robot.yaml; '
                       f'mission odometry: {value("odometry_topic")}')]
    if demo:
        domain = int(value('demo_domain_id'))
        if not 1 <= domain <= 100:
            raise ValueError('demo_domain_id must be in [1, 100]')
        share = get_package_share_directory('ridgeback_autonomy')
        for name, setting in {
            'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp', 'ROS_DOMAIN_ID': str(domain),
            'ROS_LOCALHOST_ONLY': '1',
            'CYCLONEDDS_URI': 'file://' + os.path.join(share, 'config', 'parking_demo_cyclonedds.xml'),
            'GZ_PARTITION': f'ridgeback-parking-{os.getuid()}-{domain}', 'GZ_IP': '127.0.0.1',
        }.items():
            actions.append(SetEnvironmentVariable(name, setting))
        actions.append(LogInfo(msg=f'Parking demo: loopback-only CycloneDDS domain {domain}'))
    return actions


def _after_gate(event, context, actions):
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        return [LogInfo(msg=f'Parking startup gate failed (exit {event.returncode}); shutting down'),
                EmitEvent(event=Shutdown(reason='parking startup prerequisites failed'))]
    return actions


def _startup_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    if value('mission_management_enabled') != 'true':
        return []
    demo = value('demo_scene_enabled') == 'true'
    hold = demo and value('exploration_enabled') == 'true'
    common = ['--backend', value('backend'), '--odometry-topic', value('odometry_topic'),
              '--timeout', value('startup_timeout_s')]
    if hold:
        common += ['--hold-explorer']
    def gate(phase):
        return Node(package='ridgeback_autonomy', executable='parking_simulation_startup',
            name='parking_startup_' + phase, namespace=value('namespace'),
            arguments=['--phase', phase, *common, *(['--demo'] if demo else [])],
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')], output='screen')
    prepare, release = gate('prepare'), gate('release')
    manager = Node(
        package='ridgeback_autonomy', executable='mission_manager_node.py',
        name='mission_manager_node', namespace=value('namespace'),
        parameters=[{'use_sim_time': True,
            'startup_paused': demo,
            **{name: LaunchConfiguration(name) for name in (
                'map_frame', 'robot_base_frame', 'handoff_timeout_s',
                'target_localization_timeout_s', 'parking_navigation_enabled', 'odometry_topic')}}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')], output='screen')
    return [RegisterEventHandler(OnProcessExit(target_action=prepare,
                on_exit=lambda event, context: _after_gate(event, context, [manager, release]))),
            RegisterEventHandler(OnProcessExit(target_action=release,
                on_exit=lambda event, context: _after_gate(event, context, []))), prepare]


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
        DeclareLaunchArgument('demo_scene_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'world', default_value=PythonExpression([
                "'parking_demo' if '", backend, "' == 'isaac' and '",
                LaunchConfiguration('demo_scene_enabled'), "' == 'true' else "
                "'initial_test_world' if '", backend, "' == 'isaac' else 'depot'"]),
            description='Existing backend world to load'),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('setup_path', default_value='',
                             description='Empty selects this checkout\'s clearpath/robot.yaml'),
        DeclareLaunchArgument('odometry_topic', default_value=PythonExpression([
            "'platform/odom' if '", backend, "' == 'isaac' else 'platform/odom/filtered'"]),
            description='Mission settlement/clearance odometry; Nav2 keeps filtered odometry'),
        DeclareLaunchArgument('demo_plan_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('demo_domain_id', default_value='83'),
        DeclareLaunchArgument('startup_timeout_s', default_value='300.0'),
        DeclareLaunchArgument('localization_required', default_value='false'),
        DeclareLaunchArgument('exploration_rviz', default_value='true'),
        DeclareLaunchArgument('target_localization_enabled', default_value='true'),
        DeclareLaunchArgument('localization_mode', default_value='local'),
        DeclareLaunchArgument('estimators', default_value='all'),
        DeclareLaunchArgument('detector_debug', default_value='false'),
        DeclareLaunchArgument(
            'slam_source', default_value=PythonExpression([
                "'merged' if '", backend, "' == 'gz' else 'front_only'"]),
            choices=['front_only', 'merged'],
            description=(
                'SLAM input for this demo. Gazebo maps both LiDARs: front-only SLAM '
                'never observes the floor behind the spawn pose, so every parking '
                'route stayed unknown there. Isaac and the shared exploration '
                'default keep front_only')),
        DeclareLaunchArgument('gz_gui', default_value='true'),
        DeclareLaunchArgument('headless_rendering', default_value='false'),
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Isaac: run without the sim GUI window (gz ignores)'),
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

        OpaqueFunction(function=_configure_simulation),

        # Evaluated before the include: its launch arguments are set in this
        # context, so afterwards exploration_rviz would always read 'false'.
        OpaqueFunction(function=_parking_rviz),

        # The existing backend uses deferred process-exit callbacks whose
        # substitutions need this launch context after the include returns.
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
                'headless': LaunchConfiguration('headless'),
                'slam_source': LaunchConfiguration('slam_source'),
                # Stationary unless exploration is requested explicitly; the
                # explorer is then gated by its own MissionGuard.
                'autonomous_motion_enabled': exploration_enabled,
                # Existing MissionGuard startup latch prevents all frontier
                # goals until scene + Nav2 + the manager's start pose are ready.
                'exploration_startup_paused': PythonExpression([
                    "'true' if '", LaunchConfiguration('demo_scene_enabled'),
                    "' == 'true' and '", exploration_enabled, "' == 'true' else 'false'"]),
                'localization_required': LaunchConfiguration('localization_required'),
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

        OpaqueFunction(function=_startup_nodes),
    ])
