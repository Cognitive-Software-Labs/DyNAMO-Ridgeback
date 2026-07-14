import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution, EqualsSubstitution, LaunchConfiguration, PythonExpression,
)
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
    estimate_viz = LaunchConfiguration('estimate_viz')
    depth_anything_enabled = LaunchConfiguration('depth_anything_enabled')
    mppi_visualize = LaunchConfiguration('mppi_visualize')
    coverage_overlay_enabled = LaunchConfiguration('coverage_overlay_enabled')
    sim = LaunchConfiguration('sim')
    sim_ready_timeout = LaunchConfiguration('sim_ready_timeout')
    rtf = LaunchConfiguration('rtf')
    headless = LaunchConfiguration('headless')
    livestream = LaunchConfiguration('livestream')
    odom_noise = LaunchConfiguration('odom_noise')
    camera = LaunchConfiguration('camera')

    rviz_config = os.path.join(pkg_this, 'sim', 'rviz', 'exploration.rviz')

    def _launch_wait(name, *conditions, timeout):
        # timeout may be an int (fixed gate) or a launch substitution (the
        # sim-dependent readiness window); pass substitutions through verbatim.
        timeout_arg = (str(timeout) if isinstance(timeout, (int, float))
                       else timeout)
        return ExecuteProcess(
            cmd=['ros2', 'run', 'ridgeback_autonomy', 'launch_wait',
                 *conditions, '--timeout', timeout_arg],
            name=name, output='screen',
        )

    # Readiness gates replace fixed startup timers: each blocks until its stage's
    # prerequisites exist, then OnProcessExit fires the next stage. A
    # `/<ns>/map` publisher implies the full map->odom->base_link TF chain is
    # alive, so Nav2 only activates once TF is ready — no activation race, hence
    # no blind lifecycle re-startup is needed.
    gate_slam = _launch_wait(
        'gate_slam',
        '--topic', ['/', namespace, '/sensors/lidar2d_0/scan'],
        '--topic', ['/', namespace, '/platform/odom/filtered'],
        timeout=sim_ready_timeout,
    )
    gate_nav2 = _launch_wait(
        'gate_nav2', '--topic', ['/', namespace, '/map'], timeout=60,
    )
    gate_explorer = _launch_wait(
        'gate_explorer', '--topic', ['/', namespace, '/global_costmap/costmap'],
        timeout=60,
    )

    # DDS middleware, set here so `ros2 launch` is consistent with the
    # start_exploration.sh path (mismatched RMWs can't communicate). Respect an
    # explicit override; otherwise default to CycloneDDS with the repo's tuned
    # profile (loopback, raised participant limit, large socket buffers).
    rmw_impl = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    dds_env = [SetEnvironmentVariable('RMW_IMPLEMENTATION', rmw_impl)]
    if rmw_impl == 'rmw_cyclonedds_cpp':
        cyclonedds_uri = os.environ.get(
            'CYCLONEDDS_URI',
            'file://' + os.path.join(workspace_root, 'cyclonedds.xml'),
        )
        dds_env.append(SetEnvironmentVariable('CYCLONEDDS_URI', cyclonedds_uri))

    return LaunchDescription([
        *dds_env,
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
        DeclareLaunchArgument(
            'sim', default_value='gz', choices=['gz', 'isaac'],
            description='Simulation backend, forwarded down the include chain'),
        DeclareLaunchArgument(
            'sim_ready_timeout',
            default_value=PythonExpression(
                ["'300' if '", sim, "' == 'isaac' else '45'"]),
            description='Seconds the first readiness gate waits for the sim to '
                        'publish scan+odom (Isaac cold-boots slower than gz)'),
        DeclareLaunchArgument(
            'rtf', default_value='1.0',
            description='Isaac real-time factor; 0 = unthrottled (gz ignores)'),
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Isaac: run without the sim GUI window (gz ignores)'),
        DeclareLaunchArgument(
            'livestream', default_value='false',
            description='Isaac: WebRTC livestream (gz ignores)'),
        DeclareLaunchArgument(
            'odom_noise', default_value='1.0',
            description='Isaac: odometry drift scale; 0 = perfect (gz ignores)'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Isaac: attach D455 camera; false = lidar-only '
                        '(saves GPU/RTF; gz ignores)'),
        DeclareLaunchArgument('exploration_rviz', default_value='true',
                              description='Launch the exploration RViz2 config'),
        DeclareLaunchArgument('g1_perception_enabled', default_value='true',
                              description='Launch the full G1 perception/positioning stack '
                                          '(detection + camera/lidar measurement + overlay); '
                                          'requires perception_venv'),
        DeclareLaunchArgument('estimate_viz', default_value='false',
                              description='Launch the g1_estimate_viz_node RViz marker publisher'),
        DeclareLaunchArgument('depth_anything_enabled', default_value='false',
                              description='Enable Depth-Anything in the camera measurement node'),
        DeclareLaunchArgument('mppi_visualize', default_value='false',
                              description='Publish MPPI trajectory visualization topics'),
        DeclareLaunchArgument('coverage_overlay_enabled', default_value='true',
                              description='Publish the live exploration-coverage HUD panel'),
        DeclareLaunchArgument('headless_rendering', default_value='false',
                              description='Render Gazebo server sensors via EGL without an X '
                                          'display (GPU rendering for SSH sessions; ISSUES.md)'),

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
            executable='g1_estimate_viz_node',
            name='g1_estimate_viz',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(
                AndSubstitution(g1_perception_enabled, estimate_viz)
            ),
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
                'sim': sim,
                'setup_path': setup_path,
                'world': world,
                'clearpath_rviz': 'false',
                'headless_rendering': LaunchConfiguration('headless_rendering'),
                'rtf': rtf,
                'headless': headless,
                'livestream': livestream,
                'odom_noise': odom_noise,
                'camera': camera,
            }.items(),
        ),

        # Event-driven bringup. gate_slam runs now (alongside the sim) and waits
        # for the scan + odometry to appear; each gate's exit fires the next
        # stage and the following gate.
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
                        'mppi_visualize': mppi_visualize,
                    }.items(),
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
                    }.items(),
                ),
            ],
        )),

        # Velocity chain overlay panel (planned/capped/controller/actual) -> hud/velocity.
        Node(
            package='ridgeback_autonomy',
            executable='velocity_overlay_node',
            name='velocity_overlay_node',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),

        # Live exploration-coverage panel -> hud/coverage.
        Node(
            package='ridgeback_autonomy',
            executable='coverage_overlay_node',
            name='coverage_overlay_node',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'world': world,
                'map_topic': 'map',
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(coverage_overlay_enabled),
        ),

        # Localization-error panel (GT pose vs SLAM map->base_link) ->
        # hud/localization. Isaac-only: only the Isaac runner publishes
        # ground_truth/pose, so gz launches without it (the panel stays empty
        # and the HUD aggregator drops it).
        Node(
            package='ridgeback_autonomy',
            executable='localization_overlay_node',
            name='localization_overlay_node',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(
                EqualsSubstitution(sim, 'isaac')),
        ),

        # General HUD aggregator: merges the panels into one screen overlay.
        Node(
            package='ridgeback_autonomy',
            executable='hud_node',
            name='hud_node',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'panels': ['hud/velocity', 'hud/coverage', 'hud/localization'],
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
    ])
