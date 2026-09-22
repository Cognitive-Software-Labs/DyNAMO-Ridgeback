from ridgeback_localization.environment import compute_environment
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.conditions
from launch.substitutions import (
    EqualsSubstitution, LaunchConfiguration, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ridgeback_common.camera_profiles import (
    CAMERA_PROFILE_CHOICES,
    DEFAULT_CAMERA_PROFILE,
    DEFAULT_DEPTH_FIDELITY,
    DEPTH_FIDELITY_CHOICES,
)

from ridgeback_localization.estimator_registry import (
    parse_estimators,
    selected_mask_estimators,
    selected_pointcloud_estimators,
    uses_mask_estimators,
    uses_pointcloud_estimators,
)
from ridgeback_autonomy.localization_launch import (
    cyclonedds_actions,
    distance_hud_node,
    estimate_viz_node,
    mask_measurement_node,
    overlay_node,
    pointcloud_measurement_node,
    REALSENSE_CAMERA_INPUTS,
    resolved_camera_inputs,
    SIMULATION_CAMERA_INPUTS,
)


# The distance HUD is a second overlay from the velocity/coverage one, and needs
# its own aggregator: hud_node renders through QStaticText, which switches the
# whole overlay to rich text as soon as any tag appears. The target panel is
# unconditionally rich (per-estimator span colours, <br/> breaks) while the
# velocity and coverage panels line their columns up with runs of spaces, which
# rich text collapses. One node cannot serve both contracts.
HUD_TARGET_MARKER_TOPIC = 'hud_target_overlay'

# The wide layout is four cells of HUD_WIDE_CELL_COLUMNS each, so ~40 columns.
# text_size is in POINTS, so columns-to-pixels follows the display scaling --
# measured at 12.6 px/column on one monitor and 14.4 on another, and the overlay
# clips rather than wraps, so this covers the wider of the two (40 x 14.4 = 576)
# with room for the insets. Too narrow silently drops the last estimator's whole
# column, which reads as that row never reporting rather than as a layout fault.
HUD_TARGET_OVERLAY_WIDTH = 660


def build_target_localization_nodes(context, *args, **kwargs):
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
    camera_backend = (
        'realsense'
        if LaunchConfiguration('backend').perform(context) == 'hardware'
        else 'simulation'
    )
    camera_inputs = resolved_camera_inputs(
        context, 'color_topic', 'camera_info_topic',
        'depth_topic', 'pointcloud_topic', backend=camera_backend)

    # The mask node's own base_frame default is the bare string "base_link",
    # unlike the pointcloud and viz nodes which derive a namespaced frame from
    # get_namespace(). Under this namespace that default resolves to a frame
    # nothing publishes and polar profiling's scan->base lookup fails silently,
    # so the frame is passed rather than defaulted.
    base_frame = LaunchConfiguration('base_frame').perform(context).strip()
    if not base_frame:
        raise ValueError('Pass the observed base_frame explicitly on hardware')
    mode = LaunchConfiguration('localization_mode').perform(context)

    selected_estimators = parse_estimators(
        LaunchConfiguration('estimators').perform(context).strip()
    )
    estimators = ','.join(selected_estimators)

    nodes = []
    if mode == 'local':
        nodes = [Node(
            package='ridgeback_localization',
            executable='target_detector_node',
            additional_env=compute_environment() if mode == 'local' else {},
            name='target_detector',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'color_topic': camera_inputs.color_image_topic,
                'target_labels': ParameterValue(LaunchConfiguration('target_labels'), value_type=str),
                # Typed explicitly: the node declares a double, so an integer
                # spelling like ``detector_fps:=10`` would otherwise be rejected.
                'detector_fps': ParameterValue(
                    LaunchConfiguration('detector_fps'), value_type=float),
                'detector_debug': LaunchConfiguration('detector_debug'),
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
                scan_topic=LaunchConfiguration('scan_topic'),
            ))

    if mode == 'external':
        nodes = [Node(package='ridgeback_autonomy', executable='localization_status',
            namespace=namespace, parameters=[{'use_sim_time': use_sim_time,
                'health_timeout': ParameterValue(LaunchConfiguration('health_timeout'), value_type=float)}],
            output='screen')]
    else:
        nodes.append(Node(package='ridgeback_localization', executable='localization_health',
            namespace=namespace, parameters=[{
                'use_sim_time': use_sim_time, 'mode': 'local', 'estimators': estimators,
                'target_labels': ParameterValue(LaunchConfiguration('target_labels'), value_type=str),
                'color_topic': camera_inputs.color_image_topic,
                'depth_topic': camera_inputs.aligned_depth_topic,
                'camera_info_topic': camera_inputs.color_camera_info_topic,
                'pointcloud_topic': camera_inputs.organized_points_topic or '',
                'scan_topic': LaunchConfiguration('scan_topic'), 'depth_source': depth_source, 'mask_gate': mask_gate,
                **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                   for name in ('startup_timeout', 'input_timeout', 'progress_timeout', 'source_age')},
            }], output='screen'))

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
        base_frame=base_frame,
        condition=launch.conditions.IfCondition(estimate_viz),
    ))

    # Four cells of HUD_WIDE_CELL_COLUMNS each, so ~40 columns; 40 x 14.4 px =
    # 576 plus insets.
    nodes.append(distance_hud_node(
        namespace=namespace,
        use_sim_time=use_sim_time,
        name='hud_target_node',
        overlay_width=HUD_TARGET_OVERLAY_WIDTH,
        marker_topic=HUD_TARGET_MARKER_TOPIC,
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
    target_localization_enabled = LaunchConfiguration('target_localization_enabled')
    mppi_visualize = LaunchConfiguration('mppi_visualize')
    coverage_overlay_enabled = LaunchConfiguration('coverage_overlay_enabled')
    autonomous_motion_enabled = LaunchConfiguration('autonomous_motion_enabled')
    backend = LaunchConfiguration('backend')
    legacy_sim = LaunchConfiguration('sim')
    sim_ready_timeout = LaunchConfiguration('sim_ready_timeout')
    rtf = LaunchConfiguration('rtf')
    headless = LaunchConfiguration('headless')
    livestream = LaunchConfiguration('livestream')
    odom_noise = LaunchConfiguration('odom_noise')
    noise_seed = LaunchConfiguration('noise_seed')
    camera = LaunchConfiguration('camera')
    sim_mode = LaunchConfiguration('sim_mode')
    sensor_hz = LaunchConfiguration('sensor_hz')
    slam_source = LaunchConfiguration('slam_source')

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

    def _backend_default(simulation_value, hardware_value):
        return PythonExpression([
            "'", hardware_value, "' if '", backend,
            "' == 'hardware' else '", simulation_value, "'",
        ])

    return LaunchDescription([
        *cyclonedds_actions(pkg_this),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument(
            'sim', default_value='gz', choices=['gz', 'isaac'],
            description='Deprecated compatibility alias for backend'),
        DeclareLaunchArgument(
            'backend', default_value=legacy_sim,
            choices=['gz', 'isaac', 'hardware'],
            description='I/O provider for the shared autonomy stack'),
        DeclareLaunchArgument(
            'use_sim_time', default_value=_backend_default('true', 'false')),
        DeclareLaunchArgument('setup_path',
                              default_value=_backend_default(
                                  os.path.expanduser('~/clearpath/'),
                                  '/etc/clearpath/')),
        DeclareLaunchArgument('world', default_value='initial_test_world'),
        DeclareLaunchArgument(
            'sim_ready_timeout',
            default_value=PythonExpression(
                ["'300' if '", backend, "' == 'isaac' else '45'"]),
            description='Seconds the first readiness gate waits for scan+odom'),
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
            'noise_seed', default_value='0',
            description='Isaac: reproducible odometry/IMU noise seed '
                        '(gz/hardware ignore)'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Isaac: attach D455 camera; false = lidar-only '
                        '(saves GPU/RTF; gz ignores)'),
        DeclareLaunchArgument(
            'camera_profile', default_value=DEFAULT_CAMERA_PROFILE,
            choices=CAMERA_PROFILE_CHOICES,
            description='Gazebo/Isaac nominal D455 render profile; hardware '
                        'uses its externally managed active profile'),
        DeclareLaunchArgument(
            'depth_fidelity', default_value=DEFAULT_DEPTH_FIDELITY,
            choices=DEPTH_FIDELITY_CHOICES,
            description='Isaac depth source: ideal renderer or D455-like '
                        'stereo disparity/noise/range artifacts; other '
                        'backends ignore this simulator-only option'),
        DeclareLaunchArgument(
            'sim_mode', default_value='realtime',
            description='Isaac: realtime | deterministic timing (gz ignores)'),
        DeclareLaunchArgument(
            'sensor_hz', default_value='40.0',
            description='Isaac: lidar rate / deterministic fixed dt (gz ignores)'),
        DeclareLaunchArgument(
            'slam_source',
            default_value='front_only',
            choices=['front_only', 'merged'],
            description='front_only or merged (slam_toolbox reads '
                        "scan_merger_node.py's SLAM-only front+rear merge "
                        'instead of raw front; Nav2/collision_monitor '
                        'always keep the raw front+rear topics unchanged '
                        'either way). The default is identical on every '
                        'backend; merged scans remain an explicit opt-in.'),
        DeclareLaunchArgument(
            'color_topic', default_value=_backend_default(
                SIMULATION_CAMERA_INPUTS.color_image_topic,
                REALSENSE_CAMERA_INPUTS.color_image_topic)),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=_backend_default(
                SIMULATION_CAMERA_INPUTS.color_camera_info_topic,
                REALSENSE_CAMERA_INPUTS.color_camera_info_topic),
        ),
        DeclareLaunchArgument(
            'depth_topic', default_value=_backend_default(
                SIMULATION_CAMERA_INPUTS.aligned_depth_topic,
                REALSENSE_CAMERA_INPUTS.aligned_depth_topic)),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value=_backend_default(
                SIMULATION_CAMERA_INPUTS.organized_points_topic or '',
                REALSENSE_CAMERA_INPUTS.organized_points_topic or ''),
        ),
        DeclareLaunchArgument('exploration_rviz', default_value='true',
                              description='Launch the exploration RViz2 config'),
        DeclareLaunchArgument('target_localization_enabled', default_value='true',
                              description='Launch the full target-localization stack '
                                          '(detection + the selected measurement rows + '
                                          'rings, distance HUD and camera overlay); '
                                          'requires perception_venv'),
        DeclareLaunchArgument('localization_mode', default_value='local', choices=['local', 'external']),
        DeclareLaunchArgument('localization_required', default_value=PythonExpression([
            "'true' if '", LaunchConfiguration('localization_mode'), "' == 'external' else 'false'"])),
        DeclareLaunchArgument('base_frame', default_value=PythonExpression(["'' if '", backend, "' == 'hardware' else '", namespace, "/robot/base_link'"])),
        DeclareLaunchArgument('scan_topic', default_value='sensors/lidar2d_0/scan'),
        DeclareLaunchArgument('target_labels', default_value='humanoid robot'),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in {
            'startup_timeout':'120.0', 'input_timeout':'3.0', 'progress_timeout':'15.0',
            'source_age':'15.0', 'health_timeout':'3.0', 'cancellation_timeout':'5.0'}.items()],
        DeclareLaunchArgument('estimate_viz', default_value='true',
                              description='Publish the estimator rings and the wide distance HUD'),
        DeclareLaunchArgument('estimators', default_value=_backend_default(
                                  'all',
                                  'projective_ranging,euclidean_reconstruction'),
                              description='Which distance estimator rows to run: "all" or a '
                                          'comma-separated subset of pointcloud, '
                                          'projective_ranging, euclidean_reconstruction, '
                                          'polar_profiling'),
        DeclareLaunchArgument('detector_fps', default_value='10.0',
                              description='Upper bound on detection rate, in frames per '
                                          'second; every measurement row inherits this '
                                          'cadence'),
        DeclareLaunchArgument('detector_debug', default_value='false',
                              description='Default-off detector evidence logging: achieved '
                                          'cadence, superseded frames, and bounded cold/warm '
                                          'percentiles for the throttle wait, decode, '
                                          'inference, parse, publish and CUDA synchronization'),
        DeclareLaunchArgument('depth_source', default_value='stereoscopic',
                              description='Aligned depth source for the mask rows: '
                                          'stereoscopic or monocular'),
        DeclareLaunchArgument('mask_gate', default_value='box',
                              description='Mask front-end: box (no segmentation model) or '
                                          'silhouette'),
        DeclareLaunchArgument('mppi_visualize', default_value='false',
                              description='Publish MPPI trajectory visualization topics'),
        DeclareLaunchArgument(
            'coverage_overlay_enabled',
            default_value=_backend_default('true', 'false'),
            description='Publish the live exploration-coverage HUD panel; '
                        'hardware defaults false because no simulation truth map exists'),
        DeclareLaunchArgument(
            'autonomous_motion_enabled',
            default_value=_backend_default('true', 'false'),
            choices=['true', 'false'],
            description='Allow frontier goals to command motion; hardware '
                        'defaults false and requires explicit operator opt-in'),
        DeclareLaunchArgument(
            'start_hardware_platform', default_value='false',
            choices=['true', 'false'],
            description='Hardware only: explicitly start Clearpath platform '
                        'bringup; false attaches to existing robot services'),
        DeclareLaunchArgument('headless_rendering', default_value='false',
                              description='Render Gazebo server sensors via EGL without an X '
                                          'display (GPU rendering for SSH sessions; '
                                          'docs/troubleshooting.md)'),

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
            function=build_target_localization_nodes,
            condition=launch.conditions.IfCondition(target_localization_enabled),
        ),

        # 1. Launch the selected I/O provider. The dispatcher resolves only the
        # selected optional package, so hardware never loads simulator code.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(includes_dir, 'simulation.launch.py')
            ),
            launch_arguments={
                'sim': legacy_sim,
                'backend': backend,
                'setup_path': setup_path,
                'world': world,
                'namespace': namespace,
                'clearpath_rviz': 'false',
                'headless_rendering': LaunchConfiguration('headless_rendering'),
                'rtf': rtf,
                'headless': headless,
                'livestream': livestream,
                'odom_noise': odom_noise,
                'noise_seed': noise_seed,
                'camera': camera,
                'camera_profile': LaunchConfiguration('camera_profile'),
                'depth_fidelity': LaunchConfiguration('depth_fidelity'),
                'sim_mode': sim_mode,
                'sensor_hz': sensor_hz,
                'start_hardware_platform': LaunchConfiguration(
                    'start_hardware_platform'),
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
                        'slam_source': slam_source,
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
                        'localization_required': LaunchConfiguration('localization_required'),
                        'health_timeout': LaunchConfiguration('health_timeout'),
                        'localization_progress_timeout': LaunchConfiguration('progress_timeout'),
                        'cancellation_timeout': LaunchConfiguration('cancellation_timeout'),
                    }.items(),
                    condition=launch.conditions.IfCondition(
                        autonomous_motion_enabled),
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
            condition=launch.conditions.IfCondition(PythonExpression([
                "'", backend, "' != 'hardware' and '",
                coverage_overlay_enabled, "' == 'true'",
            ])),
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
                EqualsSubstitution(backend, 'isaac')),
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
