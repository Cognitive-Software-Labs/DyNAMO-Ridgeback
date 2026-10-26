import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    launch_dir = os.path.join(pkg_this, 'launch')
    includes_dir = os.path.join(launch_dir, 'includes')
    workspace_root = os.path.abspath(os.path.join(pkg_this, '..', '..', '..', '..'))
    perception_venv_path = os.path.join(workspace_root, 'perception_venv')
    perception_venv_bin = os.path.join(perception_venv_path, 'bin')
    g1_model_sdf = os.path.join(pkg_this, 'sim', 'models', 'g1', 'model.sdf')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')

    def _launch_wait(name, *conditions, timeout, remap_tf=False):
        cmd = [
                'ros2',
                'run',
                'ridgeback_autonomy',
                'launch_wait',
                *conditions,
                '--timeout',
                str(timeout),
        ]
        if remap_tf:
            cmd.extend([
                '--ros-args',
                '-r',
                ['/tf:=/', namespace, '/tf'],
                '-r',
                ['/tf_static:=/', namespace, '/tf_static'],
            ])
        return ExecuteProcess(cmd=cmd, name=name, output='screen')

    gate_sim_ready = _launch_wait(
        'gate_mission_supervisor_sim_ready',
        '--topic', ['/', namespace, '/sensors/lidar2d_0/scan'],
        '--topic', ['/', namespace, '/platform/odom/filtered'],
        '--topic', ['/', namespace, '/platform_velocity_controller/controller_state'],
        '--topic', ['/', namespace, '/sensors/camera_0/color/image'],
        timeout=120,
    )
    gate_nav2 = _launch_wait(
        'gate_mission_supervisor_nav2',
        '--topic', ['/', namespace, '/map'],
        timeout=180,
    )
    gate_map_to_base = _launch_wait(
        'gate_mission_supervisor_map_to_base_link',
        '--tf', 'map', 'base_link',
        timeout=180,
        remap_tf=True,
    )
    gate_explorer = _launch_wait(
        'gate_mission_supervisor_explorer',
        '--topic', ['/', namespace, '/global_costmap/costmap'],
        timeout=180,
    )

    spawn_g1 = ExecuteProcess(
        cmd=[
            '/opt/ros/jazzy/lib/ros_gz_sim/create',
            '-world', world,
            '-file', g1_model_sdf,
            '-name', 'mission_supervisor_demo_g1',
            '-x', '2.065',
            '-y', '-5.797',
            '-z', '0.000000',
            '-Y', '3.141593',
        ],
        name='spawn_mission_supervisor_demo_g1',
        output='screen',
    )
    remove_builtin_g1 = ExecuteProcess(
        cmd=[
            'gz',
            'service',
            '-s', '/world/mock_hospital/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', 'name: "default_g1_far_right_room"',
        ],
        name='remove_default_mock_hospital_g1',
        output='screen',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='mission_supervisor_demo_rviz',
        namespace=namespace,
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('VIRTUAL_ENV', perception_venv_path),
        SetEnvironmentVariable(
            'PATH',
            os.pathsep.join([perception_venv_bin, os.environ.get('PATH', '')]),
        ),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_this, 'config', 'mission_thesis_view.rviz'),
        ),
        DeclareLaunchArgument(
            'setup_path',
            default_value=os.path.expanduser('~/clearpath/'),
        ),
        DeclareLaunchArgument('world', default_value='mock_hospital'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_clearpath_gz, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'setup_path': setup_path,
                'world': world,
                'rviz': 'false',
                'use_sim_time': use_sim_time,
                'x': '0.000000',
                'y': '-2.200000',
                'z': '0.300000',
                'yaw': '0.000000',
            }.items(),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_clearpath_alias_tf',
            namespace=namespace,
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', PythonExpression(["'", namespace, "/robot/base_link'"]),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
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
            executable='g1_detector_node',
            name='g1_detector',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),

        Node(
            package='ridgeback_autonomy',
            executable='g1_camera_measurement_node',
            name='g1_camera_measurement',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'depth_anything_enabled': False,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),

        Node(
            package='ridgeback_autonomy',
            executable='g1_lidar_measurement_node',
            name='g1_lidar_measurement',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),

        Node(
            package='ridgeback_autonomy',
            executable='g1_overlay_node',
            name='g1_overlay',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),

        Node(
            package='ridgeback_autonomy',
            executable='g1_estimate_viz_node',
            name='g1_estimate_viz',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),

        gate_sim_ready,
        RegisterEventHandler(OnProcessExit(
            target_action=gate_sim_ready,
            on_exit=[
                rviz_node,
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(includes_dir, 'slam.launch.py')
                    ),
                    launch_arguments={
                        'setup_path': setup_path,
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
                remove_builtin_g1,
                spawn_g1,
                gate_nav2,
            ],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_nav2,
            on_exit=[gate_map_to_base],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_map_to_base,
            on_exit=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(includes_dir, 'nav2.launch.py')
                    ),
                    launch_arguments={
                        'namespace': namespace,
                        'use_sim_time': use_sim_time,
                        'mppi_visualize': 'false',
                    }.items(),
                ),
                Node(
                    package='ridgeback_autonomy',
                    executable='mission_manager_node',
                    name='mission_manager_node',
                    namespace=namespace,
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'stop_exploration_on_g1': True,
                        'g1_foot_midpoint_debug_enabled': False,
                    }],
                    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                    output='screen',
                ),
                gate_explorer,
            ],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_explorer,
            on_exit=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(includes_dir, 'explore.launch.py')
                    ),
                    launch_arguments={
                        'namespace': namespace,
                        'use_sim_time': use_sim_time,
                        'explorer': 'custom',
                    }.items(),
                ),
            ],
        )),
    ])
