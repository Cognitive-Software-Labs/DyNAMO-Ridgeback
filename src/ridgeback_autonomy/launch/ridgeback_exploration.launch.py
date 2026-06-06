import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    launch_dir = os.path.join(pkg_this, 'launch')
    includes_dir = os.path.join(launch_dir, 'includes')
    workspace_root = os.path.abspath(os.path.join(pkg_this, '..', '..', '..', '..'))
    perception_venv_path = os.path.join(workspace_root, 'perception_venv')
    perception_venv_bin = os.path.join(perception_venv_path, 'bin')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    exploration_rviz = LaunchConfiguration('exploration_rviz')
    g1_perception_enabled = LaunchConfiguration('g1_perception_enabled')
    depth_anything_enabled = LaunchConfiguration('depth_anything_enabled')
    mppi_visualize = LaunchConfiguration('mppi_visualize')
    explorer = LaunchConfiguration('explorer')

    rviz_config = os.path.join(pkg_this, 'sim', 'rviz', 'exploration.rviz')

    return LaunchDescription([
        SetEnvironmentVariable('VIRTUAL_ENV', perception_venv_path),
        SetEnvironmentVariable(
            'PATH',
            os.pathsep.join([perception_venv_bin, os.environ.get('PATH', '')]),
        ),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('setup_path',
                              default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument('world', default_value='mock_hospital'),
        DeclareLaunchArgument('exploration_rviz', default_value='true',
                              description='Launch the exploration RViz2 config'),
        DeclareLaunchArgument('g1_perception_enabled', default_value='true',
                              description='Launch the G1 perception stack'),
        DeclareLaunchArgument('depth_anything_enabled', default_value='false',
                              description='Enable Depth-Anything in the camera measurement node'),
        DeclareLaunchArgument('mppi_visualize', default_value='false',
                              description='Publish MPPI trajectory visualization topics'),
        DeclareLaunchArgument('explorer', default_value='explore_lite',
                              description='Which explorer to use: "explore_lite" or "custom"'),

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
            condition=launch.conditions.IfCondition(exploration_rviz),
        ),

        Node(
            package='ridgeback_autonomy',
            executable='g1_detector_node',
            name='g1_detector',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(g1_perception_enabled),
        ),

        Node(
            package='ridgeback_autonomy',
            executable='g1_camera_measurement_node',
            name='g1_camera_measurement',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'depth_anything_enabled': depth_anything_enabled,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(g1_perception_enabled),
        ),

        Node(
            package='ridgeback_autonomy',
            executable='g1_lidar_measurement_node',
            name='g1_lidar_measurement',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(g1_perception_enabled),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(includes_dir, 'camera_optical_tf.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        Node(
            package='ridgeback_autonomy',
            executable='g1_overlay_node',
            name='g1_overlay',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(g1_perception_enabled),
        ),

        # 1. Launch Gazebo simulation
        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(includes_dir, 'simulation.launch.py')
                    ),
            launch_arguments={
                'setup_path': setup_path,
                'world': world,
                'clearpath_rviz': 'false',
            }.items(),
        ),

        # 2. Launch SLAM (delayed to let sim fully start and publish TF)
        TimerAction(
            period=20.0,
            actions=[
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
        ),

        # 3. Launch Nav2 (delayed to let SLAM start publishing map)
        TimerAction(
            period=80.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(includes_dir, 'nav2.launch.py')
                    ),
                    launch_arguments={
                        'namespace': namespace,
                        'use_sim_time': use_sim_time,
                        'mppi_visualize': mppi_visualize,
                    }.items(),
                ),
            ],
        ),

        # 3b. Re-trigger Nav2 lifecycle STARTUP to recover from the
        # odom→base_link TF race that sometimes leaves servers inactive.
        # If activation already succeeded the service returns false harmlessly.
        TimerAction(
            period=92.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/r100_0001/lifecycle_manager_navigation/manage_nodes',
                        'nav2_msgs/srv/ManageLifecycleNodes',
                        '{command: 2}',
                    ],
                    output='log',
                ),
            ],
        ),

        # 4. Launch explorer (delayed to let Nav2 fully start)
        TimerAction(
            period=105.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(includes_dir, 'explore.launch.py')
                    ),
                    launch_arguments={
                        'namespace': namespace,
                        'use_sim_time': use_sim_time,
                        'explorer': explorer,
                    }.items(),
                ),
            ],
        ),

        # Velocity chain overlay marker (planned/capped/controller/actual).
        Node(
            package='ridgeback_autonomy',
            executable='velocity_overlay_node',
            name='velocity_overlay_node',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
    ])
