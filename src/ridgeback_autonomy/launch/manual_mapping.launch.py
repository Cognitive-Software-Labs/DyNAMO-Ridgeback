#!/usr/bin/env python3
"""
Manual SLAM mapping launch for ground-truth map generation.

This brings up:
- Gazebo simulation
- SLAM (slam_toolbox)
- Keyboard teleop

Excludes Nav2 and frontier exploration so the operator can manually teleoperate
through the entire reachable space to collect ground-truth maps.

Usage:
    ros2 launch ridgeback_autonomy manual_mapping.launch.py world:=hospital
    ros2 launch ridgeback_autonomy manual_mapping.launch.py world:=warehouse
    ros2 launch ridgeback_autonomy manual_mapping.launch.py world:=office

After driving through the entire space, save the map:
    ros2 run nav2_map_server map_saver_cli -f ground_truth_<world>

The map will be saved as:
    ground_truth_<world>.pgm
    ground_truth_<world>.yaml
"""

import os
from launch import LaunchDescription
import launch.conditions
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    pkg_clearpath_control = get_package_share_directory('clearpath_control')
    launch_dir = os.path.join(pkg_this, 'launch')
    includes_dir = os.path.join(launch_dir, 'includes')
    workspace_root = os.path.abspath(os.path.join(pkg_this, '..', '..', '..', '..'))
    fastrtps_profile_abs = os.path.join(workspace_root, 'fastrtps_no_shm.xml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    mapping_rviz = LaunchConfiguration('mapping_rviz')

    rviz_config = os.path.join(pkg_this, 'sim', 'rviz', 'exploration.rviz')

    # Readiness gates replace fixed startup timers: SLAM starts once scan +
    # odometry exist; the drive controller is activated once controller_manager
    # is up (it loads platform_velocity_controller inactive at startup).
    gate_slam = ExecuteProcess(
        cmd=['ros2', 'run', 'ridgeback_autonomy', 'launch_wait',
             '--topic', ['/', namespace, '/sensors/lidar2d_0/scan'],
             '--topic', ['/', namespace, '/platform/odom/filtered'],
             '--timeout', '45'],
        name='gate_slam', output='screen',
    )
    gate_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'ridgeback_autonomy', 'launch_wait',
             '--service', ['/', namespace, '/controller_manager/switch_controller'],
             '--timeout', '30'],
        name='gate_controller', output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastrtps_profile_abs),
        
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('setup_path',
                              default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='World to load: hospital, warehouse, or office'),
        DeclareLaunchArgument('mapping_rviz', default_value='true',
                              description='Launch RViz for map visualization during mapping'),

        # RViz2 for mapping visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(mapping_rviz),
        ),

        # 1. Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(includes_dir, 'simulation.launch.py')
            ),
            launch_arguments={
                'setup_path': setup_path,
                'world': world,
                'clearpath_rviz': 'false',
                'gz_gui': 'true',
            }.items(),
        ),

        # 2. SLAM — start once the sim is publishing scan + odometry.
        gate_slam,
        RegisterEventHandler(OnProcessExit(
            target_action=gate_slam,
            on_exit=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(includes_dir, 'slam.launch.py')
                    ),
                    launch_arguments={
                        'setup_path': setup_path,
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
            ],
        )),

        # Activate the mecanum drive controller (starts inactive in sim) once
        # controller_manager is up.
        gate_controller,
        RegisterEventHandler(OnProcessExit(
            target_action=gate_controller,
            on_exit=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/r100_0001/controller_manager/switch_controller',
                        'controller_manager_msgs/srv/SwitchController',
                        "{activate_controllers: ['platform_velocity_controller'], "
                        "deactivate_controllers: [], strictness: 2}",
                    ],
                    output='screen',
                ),
            ],
        )),

        # 3. Interactive marker teleop (RViz drag widget) + twist_mux
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_clearpath_control, 'launch', 'teleop_base.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

    ])


