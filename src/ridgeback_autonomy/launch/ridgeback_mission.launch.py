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
from launch_ros.parameter_descriptions import ParameterValue


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
    rviz_config = LaunchConfiguration('rviz_config')
    enable_translating_g1_observation = LaunchConfiguration(
        'enable_translating_g1_observation'
    )
    parking_candidate_arrows_enabled = LaunchConfiguration(
        'parking_candidate_arrows_enabled'
    )
    parking_candidate_labels_enabled = LaunchConfiguration(
        'parking_candidate_labels_enabled'
    )
    parking_candidate_scores_enabled = LaunchConfiguration(
        'parking_candidate_scores_enabled'
    )

    def _launch_wait(name, *conditions, timeout):
        return ExecuteProcess(
            cmd=[
                'ros2',
                'run',
                'ridgeback_autonomy',
                'launch_wait',
                *conditions,
                '--timeout',
                str(timeout),
            ],
            name=name,
            output='screen',
        )

    gate_sim_ready = _launch_wait(
        'gate_mission_exploration_sim_ready',
        '--topic', ['/', namespace, '/sensors/lidar2d_0/scan'],
        '--topic', ['/', namespace, '/platform/odom/filtered'],
        '--topic', ['/', namespace, '/sensors/camera_0/color/image'],
        timeout=60,
    )
    gate_nav2 = _launch_wait(
        'gate_mission_exploration_nav2',
        '--topic', ['/', namespace, '/map'],
        timeout=60,
    )
    gate_explorer = _launch_wait(
        'gate_mission_exploration_explorer',
        '--topic', ['/', namespace, '/global_costmap/costmap'],
        timeout=60,
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
        DeclareLaunchArgument('world', default_value='park_robot'),
        DeclareLaunchArgument(
            'enable_translating_g1_observation',
            default_value='false',
            description=(
                'Experimental translating A/B G1 observation fallback; the '
                'canonical mission centers in place by default.'
            ),
        ),
        DeclareLaunchArgument(
            'parking_candidate_arrows_enabled',
            default_value='true',
            description='Show all parking candidate decision arrows in RViz.',
        ),
        DeclareLaunchArgument(
            'parking_candidate_labels_enabled',
            default_value='false',
            description='Show candidate index, status, and rejection reason labels.',
        ),
        DeclareLaunchArgument(
            'parking_candidate_scores_enabled',
            default_value='false',
            description=(
                'Show actual candidate ranking metrics; no synthetic score is used.'
            ),
        ),

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

        Node(
            package='rviz2',
            executable='rviz2',
            name='ridgeback_mission_rviz',
            namespace=namespace,
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),

        gate_sim_ready,
        RegisterEventHandler(OnProcessExit(
            target_action=gate_sim_ready,
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
                gate_nav2,
            ],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_nav2,
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
                        'enable_translating_g1_observation': ParameterValue(
                            enable_translating_g1_observation,
                            value_type=bool,
                        ),
                        'parking_candidate_arrows_enabled': ParameterValue(
                            parking_candidate_arrows_enabled,
                            value_type=bool,
                        ),
                        'parking_candidate_labels_enabled': ParameterValue(
                            parking_candidate_labels_enabled,
                            value_type=bool,
                        ),
                        'parking_candidate_scores_enabled': ParameterValue(
                            parking_candidate_scores_enabled,
                            value_type=bool,
                        ),
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
