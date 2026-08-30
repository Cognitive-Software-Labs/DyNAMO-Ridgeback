import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ridgeback_autonomy.perception.estimators import (
    parse_estimators,
    selected_mask_estimators,
    selected_pointcloud_estimators,
    uses_mask_estimators,
    uses_pointcloud_estimators,
)
from ridgeback_autonomy.perception.g1_launch import (
    SIMULATION_CAMERA_INPUTS,
    distance_hud_node,
    estimate_viz_node,
    mask_measurement_node,
    overlay_node,
    perception_venv_actions,
    pointcloud_measurement_node,
    resolved_camera_inputs,
)


# The distance HUD is a second overlay from the velocity/coverage one, and needs
# its own aggregator: hud_node renders through QStaticText, which switches the
# whole overlay to rich text as soon as any tag appears. The g1 panel is
# unconditionally rich (per-estimator span colours, <br/> breaks) while the
# velocity and coverage panels line their columns up with runs of spaces, which
# rich text collapses. One node cannot serve both contracts.
HUD_G1_MARKER_TOPIC = 'hud_g1_overlay'

# The wide layout is four cells of HUD_WIDE_CELL_COLUMNS each, so ~40 columns.
# text_size is in POINTS, so columns-to-pixels follows the display scaling --
# measured at 12.6 px/column on one monitor and 14.4 on another, and the overlay
# clips rather than wraps, so this covers the wider of the two (40 x 14.4 = 576)
# with room for the insets. Too narrow silently drops the last estimator's whole
# column, which reads as that row never reporting rather than as a layout fault.
HUD_G1_OVERLAY_WIDTH = 660


def build_g1_perception_nodes(context, *args, **kwargs):
    """The measurement and display stack for the estimator rows this run selected.

    An OpaqueFunction because the selection has to be read as a string --
    ``parse_estimators`` decides which measurement nodes exist at all, and a
    substitution cannot be branched on until a context resolves it.
    """

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    depth_source = LaunchConfiguration('depth_source')
    mask_gate = LaunchConfiguration('mask_gate')
    estimate_viz = LaunchConfiguration('estimate_viz')
    camera_inputs = resolved_camera_inputs(
        context, 'color_topic', 'camera_info_topic',
        'depth_topic', 'pointcloud_topic')

    # The mask node's own base_frame default is the bare string "base_link",
    # unlike the pointcloud and viz nodes which derive a namespaced frame from
    # get_namespace(). Under this namespace that default resolves to a frame
    # nothing publishes and polar profiling's scan->base lookup fails silently,
    # so the frame is passed rather than defaulted.
    base_frame = [namespace, '/robot/base_link']

    selected_estimators = parse_estimators(
        LaunchConfiguration('estimators').perform(context).strip()
    )
    estimators = ','.join(selected_estimators)

    nodes = [Node(
        package='ridgeback_autonomy',
        executable='g1_detector_node',
        name='g1_detector',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            'color_topic': camera_inputs.color_image_topic,
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )]

    if uses_pointcloud_estimators(selected_estimators):
        nodes.append(pointcloud_measurement_node(
            namespace=namespace,
            use_sim_time=use_sim_time,
            enabled_estimators=','.join(
                selected_pointcloud_estimators(selected_estimators)),
            base_frame=base_frame,
            color_topic=camera_inputs.color_image_topic,
            pointcloud_topic=camera_inputs.organized_points_topic,
        ))

    if uses_mask_estimators(selected_estimators):
        nodes.append(mask_measurement_node(
            namespace=namespace,
            use_sim_time=use_sim_time,
            enabled_estimators=','.join(
                selected_mask_estimators(selected_estimators)),
            base_frame=base_frame,
            depth_source=depth_source,
            mask_gate=mask_gate,
            color_topic=camera_inputs.color_image_topic,
            camera_info_topic=camera_inputs.color_camera_info_topic,
            depth_topic=camera_inputs.aligned_depth_topic,
        ))

    # Rings on the floor plan plus the distance panel that names them. Both
    # surfaces are filtered from the one selected set, so a ring can never
    # appear without a column to name it, and a column is only shown for a path
    # this run actually launched -- an unproduced row could only ever print
    # "--", the same mark an estimator that ran and found nothing prints.
    nodes.append(estimate_viz_node(
        namespace=namespace,
        use_sim_time=use_sim_time,
        estimators=estimators,
        # Exploration has no truth source, so the row layout's truth and error
        # columns would be permanently blank.
        hud_layout='wide',
        condition=launch.conditions.IfCondition(estimate_viz),
    ))

    # Four cells of HUD_WIDE_CELL_COLUMNS each, so ~40 columns; 40 x 14.4 px =
    # 576 plus insets.
    nodes.append(distance_hud_node(
        namespace=namespace,
        use_sim_time=use_sim_time,
        name='hud_g1_node',
        overlay_width=HUD_G1_OVERLAY_WIDTH,
        marker_topic=HUD_G1_MARKER_TOPIC,
        # The viz node is the only producer of the panel this aggregator merges.
        condition=launch.conditions.IfCondition(estimate_viz),
    ))

    # The camera overlay picks its panels from the same set, so it grids one
    # frame per path that has a producer here. Labels stay on, unlike the
    # benchmark: there is no collage carrying the numbers separately.
    nodes.append(overlay_node(
        namespace=namespace,
        use_sim_time=use_sim_time,
        estimators=estimators,
        color_topic=camera_inputs.color_image_topic,
        depth_source=depth_source,
        mask_gate=mask_gate,
    ))

    return nodes


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    launch_dir = os.path.join(pkg_this, 'launch')
    includes_dir = os.path.join(launch_dir, 'includes')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    exploration_rviz = LaunchConfiguration('exploration_rviz')
    g1_perception_enabled = LaunchConfiguration('g1_perception_enabled')
    mppi_visualize = LaunchConfiguration('mppi_visualize')
    explorer = LaunchConfiguration('explorer')
    coverage_overlay_enabled = LaunchConfiguration('coverage_overlay_enabled')

    rviz_config = os.path.join(pkg_this, 'sim', 'rviz', 'exploration.rviz')

    def _launch_wait(name, *conditions, timeout):
        return ExecuteProcess(
            cmd=['ros2', 'run', 'ridgeback_autonomy', 'launch_wait',
                 *conditions, '--timeout', str(timeout)],
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
        timeout=45,
    )
    gate_nav2 = _launch_wait(
        'gate_nav2', '--topic', ['/', namespace, '/map'], timeout=60,
    )
    gate_explorer = _launch_wait(
        'gate_explorer', '--topic', ['/', namespace, '/global_costmap/costmap'],
        timeout=60,
    )

    return LaunchDescription([
        *perception_venv_actions(pkg_this),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('setup_path',
                              default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument('world', default_value='mock_hospital'),
        DeclareLaunchArgument(
            'color_topic', default_value=SIMULATION_CAMERA_INPUTS.color_image_topic),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=SIMULATION_CAMERA_INPUTS.color_camera_info_topic,
        ),
        DeclareLaunchArgument(
            'depth_topic', default_value=SIMULATION_CAMERA_INPUTS.aligned_depth_topic),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value=SIMULATION_CAMERA_INPUTS.organized_points_topic or '',
        ),
        DeclareLaunchArgument('exploration_rviz', default_value='true',
                              description='Launch the exploration RViz2 config'),
        DeclareLaunchArgument('g1_perception_enabled', default_value='true',
                              description='Launch the full G1 perception/positioning stack '
                                          '(detection + the selected measurement rows + '
                                          'rings, distance HUD and camera overlay); '
                                          'requires perception_venv'),
        DeclareLaunchArgument('estimate_viz', default_value='true',
                              description='Publish the estimator rings and the wide distance HUD'),
        DeclareLaunchArgument('estimators', default_value='all',
                              description='Which distance estimator rows to run: "all" or a '
                                          'comma-separated subset of pointcloud, '
                                          'projective_ranging, euclidean_reconstruction, '
                                          'polar_profiling'),
        DeclareLaunchArgument('depth_source', default_value='stereoscopic',
                              description='Aligned depth source for the mask rows: '
                                          'stereoscopic or monocular'),
        DeclareLaunchArgument('mask_gate', default_value='box',
                              description='Mask front-end: box (no segmentation model) or '
                                          'silhouette'),
        DeclareLaunchArgument('mppi_visualize', default_value='false',
                              description='Publish MPPI trajectory visualization topics'),
        DeclareLaunchArgument('explorer', default_value='explore_lite',
                              description='Which explorer to use: "explore_lite" or "custom"'),
        DeclareLaunchArgument('coverage_overlay_enabled', default_value='true',
                              description='Publish the live exploration-coverage HUD panel'),

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

        # Detector, measurement nodes, rings, distance HUD and camera overlay
        # are gated as one unit: they are all downstream of detections.
        OpaqueFunction(
            function=build_g1_perception_nodes,
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
                        'explorer': explorer,
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

        # General HUD aggregator: merges the panels into one screen overlay.
        Node(
            package='ridgeback_autonomy',
            executable='hud_node',
            name='hud_node',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'panels': ['hud/velocity', 'hud/coverage'],
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
    ])
