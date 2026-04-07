import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_slam_exploration')
    launch_dir = os.path.join(pkg_this, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    rviz_config = os.path.join(pkg_this, 'rviz', 'exploration.rviz')

    fastrtps_config = os.path.join(pkg_this, 'config', 'fastrtps_no_shm.xml')

    return LaunchDescription([
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastrtps_config),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('setup_path',
                              default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(rviz),
        ),

        # 1. Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'simulation.launch.py')
            ),
            launch_arguments={
                'setup_path': setup_path,
                'world': world,
            }.items(),
        ),

        # 2. Launch SLAM (delayed to let sim fully start and publish TF)
        TimerAction(
            period=20.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'slam.launch.py')
                    ),
                    launch_arguments={
                        'setup_path': setup_path,
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
            ],
        ),

        # 3. Launch Nav2 (delayed to let SLAM start publishing map)
        TimerAction(
            period=30.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'nav2.launch.py')
                    ),
                    launch_arguments={
                        'namespace': namespace,
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
            ],
        ),

        # 4. Launch explore_lite (delayed to let Nav2 fully start)
        TimerAction(
            period=45.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'explore.launch.py')
                    ),
                    launch_arguments={
                        'namespace': namespace,
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
            ],
        ),
    ])
