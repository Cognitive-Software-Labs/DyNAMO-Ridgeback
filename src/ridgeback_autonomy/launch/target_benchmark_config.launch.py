"""Per-configuration measurement, visualization, and runner benchmark layer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ridgeback_autonomy.benchmarking.paths import default_output_directory
from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    parse_estimators,
    selected_mask_estimators,
    selected_pointcloud_estimators,
    uses_mask_estimators,
    uses_pointcloud_estimators,
)
from ridgeback_autonomy.perception.target_localization.launch import (
    ALIGNED_DEPTH_DEBUG_TOPIC,
    CONFIG_LAUNCH_ARGUMENT_NAMES,
    MASK_MEASUREMENTS_TOPIC,
    POINTCLOUD_MEASUREMENTS_TOPIC,
    RAW_DETECTIONS_TOPIC,
    SIMULATION_CAMERA_INPUTS,
    cyclonedds_actions,
    distance_hud_node,
    estimate_viz_node,
    mask_measurement_node,
    overlay_node,
    perception_venv_actions,
    pointcloud_measurement_node,
    resolved_camera_inputs,
    workspace_root_from_package_share,
)
from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    DEPTH_GATE_DISABLED,
    NEAREST_MODE_BIN_WIDTH_M_DEFAULT,
    NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT,
)
from ridgeback_autonomy.perception.target_localization.core.isolation_3d import (
    FLOOR_MARGIN_M_DEFAULT,
    ISOLATION_3D_DEFAULT,
)
from ridgeback_autonomy.perception.target_localization.core.ranging_defaults import (
    FRONT_PERCENTILE,
    INLIER_AHEAD_MARGIN_M,
    INLIER_BEHIND_MARGIN_M,
    MIN_VALID_SAMPLES,
    NEAR_SURFACE_BAND_M,
)


ENV_READY_TIMEOUT_SEC = 300.0


def build_benchmark_nodes(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    repeats = LaunchConfiguration('repeats')
    output_dir = LaunchConfiguration('output_dir')
    settle_sec = LaunchConfiguration('settle_sec')
    capture_sec = LaunchConfiguration('capture_sec')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame = LaunchConfiguration('base_frame')
    camera_inputs = resolved_camera_inputs(
        context, 'color_topic', 'camera_info_topic',
        'depth_topic', 'pointcloud_topic')

    selected_estimators = parse_estimators(
        LaunchConfiguration('estimators').perform(context).strip()
    )
    selected_pointcloud = selected_pointcloud_estimators(selected_estimators)
    selected_mask = selected_mask_estimators(selected_estimators)
    needs_pointcloud = uses_pointcloud_estimators(selected_estimators)
    needs_mask = uses_mask_estimators(selected_estimators)

    nodes = []

    if needs_pointcloud:
        nodes.append(pointcloud_measurement_node(
            namespace=namespace,
            use_sim_time=use_sim_time,
            enabled_estimators=','.join(selected_pointcloud),
            base_frame=base_frame,
            color_topic=camera_inputs.color_image_topic,
            pointcloud_topic=camera_inputs.organized_points_topic,
        ))

    if needs_mask:
        nodes.append(mask_measurement_node(
            namespace=namespace,
            use_sim_time=use_sim_time,
            enabled_estimators=','.join(selected_mask),
            base_frame=base_frame,
            depth_source=LaunchConfiguration('depth_source'),
            depth_topic=camera_inputs.aligned_depth_topic,
            camera_info_topic=camera_inputs.color_camera_info_topic,
            scan_topic=scan_topic,
            isolation_2d=LaunchConfiguration('isolation_2d'),
            isolation_2d_bin_width_m=LaunchConfiguration('isolation_2d_bin_width_m'),
            isolation_2d_band_m=LaunchConfiguration('isolation_2d_band_m'),
            isolation_2d_min_bin_fraction=LaunchConfiguration(
                'isolation_2d_min_bin_fraction'),
            min_valid_pixels=LaunchConfiguration('min_valid_pixels'),
            isolation_3d=LaunchConfiguration('isolation_3d'),
            isolation_3d_floor_margin_m=LaunchConfiguration(
                'isolation_3d_floor_margin_m'),
            isolation_3d_percentile=LaunchConfiguration('isolation_3d_percentile'),
            isolation_3d_ahead_m=LaunchConfiguration('isolation_3d_ahead_m'),
            isolation_3d_behind_m=LaunchConfiguration('isolation_3d_behind_m'),
            isolation_3d_bin_width_m=LaunchConfiguration('isolation_3d_bin_width_m'),
            isolation_3d_min_bin_fraction=LaunchConfiguration(
                'isolation_3d_min_bin_fraction'),
            mask_gate=LaunchConfiguration('mask_gate'),
            color_topic=camera_inputs.color_image_topic,
            depth_max_meters=LaunchConfiguration('mask_depth_max_meters'),
            depth_match_debug=LaunchConfiguration('depth_match_debug'),
        ))

    # The HUD lists only the rows this run actually launched.
    nodes.append(estimate_viz_node(
        namespace=namespace,
        use_sim_time=use_sim_time,
        estimators=','.join(selected_estimators),
        condition=launch.conditions.IfCondition(LaunchConfiguration('estimate_viz')),
    ))

    # The benchmark drops the per-detection label block: the runner's collage
    # carries the numbers, so the panels are there to be looked at, not read.
    nodes.append(overlay_node(
        namespace=namespace,
        use_sim_time=use_sim_time,
        estimators=','.join(selected_estimators),
        color_topic=camera_inputs.color_image_topic,
        depth_source=LaunchConfiguration('depth_source'),
        mask_gate=LaunchConfiguration('mask_gate'),
        rgb_panel_labels=False,
        condition=launch.conditions.IfCondition(LaunchConfiguration('overlay')),
    ))

    # The rows layout's widest line is "Euclidean Reconstruction" (24 columns)
    # + distance + signed error + the age column an aged row carries = 47
    # columns; 47 x 14.4 px = 677 plus insets. The truth header is longer still
    # (the trial id runs to ~63 columns) and is knowingly left to clip -- it is
    # prose, not a column anyone reads off.
    nodes.append(distance_hud_node(
        namespace=namespace,
        use_sim_time=use_sim_time,
        name='hud_node',
        overlay_width=720,
        condition=launch.conditions.IfCondition(LaunchConfiguration('estimate_viz')),
    ))

    runner = Node(
        package='ridgeback_autonomy',
        executable='target_distance_benchmark_runner',
        name='target_distance_benchmark_runner',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            'world': world,
            'scenario': LaunchConfiguration('scenario'),
            'repeats': repeats,
            'output_dir': output_dir,
            'run_dir_name': LaunchConfiguration('run_dir_name'),
            'settle_sec': settle_sec,
            'capture_sec': capture_sec,
            'replay_dataset_dir': LaunchConfiguration('replay_dataset_dir'),
            'capture_batches': LaunchConfiguration('capture_batches'),
            'capture_drain_sec': LaunchConfiguration('capture_drain_sec'),
            'capture_timeout_sec': LaunchConfiguration('capture_timeout_sec'),
            'raw_detections_topic': RAW_DETECTIONS_TOPIC,
            'aligned_depth_debug_topic': ALIGNED_DEPTH_DEBUG_TOPIC,
            'camera_info_topic': camera_inputs.color_camera_info_topic,
            'base_frame': base_frame,
            'estimators': ','.join(selected_estimators),
            'pointcloud_measurement_topic': POINTCLOUD_MEASUREMENTS_TOPIC,
            'mask_measurement_topic': MASK_MEASUREMENTS_TOPIC,
            'color_topic': camera_inputs.color_image_topic,
            'depth_source': LaunchConfiguration('depth_source'),
            'isolation_2d': LaunchConfiguration('isolation_2d'),
            'isolation_3d': LaunchConfiguration('isolation_3d'),
            'mask_gate': LaunchConfiguration('mask_gate'),
            # Recorded, not applied: this records the mask row's actual gate.
            'mask_depth_max_meters': LaunchConfiguration('mask_depth_max_meters'),
            # Recorded, not applied, for the same reason: run.json is where a
            # sweep's per-config parameter values are read back from, and these
            # are the axes a recipe sweep varies -- on either side, so the
            # euclidean row's numbers are recoverable from an archived run the
            # same way the projective row's already were.
            'isolation_2d_bin_width_m': LaunchConfiguration('isolation_2d_bin_width_m'),
            'isolation_2d_band_m': LaunchConfiguration('isolation_2d_band_m'),
            'isolation_2d_min_bin_fraction': LaunchConfiguration(
                'isolation_2d_min_bin_fraction'),
            'min_valid_pixels': LaunchConfiguration('min_valid_pixels'),
            'isolation_3d_floor_margin_m': LaunchConfiguration(
                'isolation_3d_floor_margin_m'),
            'isolation_3d_percentile': LaunchConfiguration('isolation_3d_percentile'),
            'isolation_3d_ahead_m': LaunchConfiguration('isolation_3d_ahead_m'),
            'isolation_3d_behind_m': LaunchConfiguration('isolation_3d_behind_m'),
            'isolation_3d_bin_width_m': LaunchConfiguration('isolation_3d_bin_width_m'),
            'isolation_3d_min_bin_fraction': LaunchConfiguration(
                'isolation_3d_min_bin_fraction'),
            # The recorder screen-grabs the RViz window, which competes with
            # the simulator's render path. Turn it off for timing runs and take
            # visual integrity from a separate smoke run instead.
            'record_video': LaunchConfiguration('record_video'),
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )
    nodes.append(runner)

    # The supervisor runs this layer in its own launch process and needs that
    # process to return when the runner does. The compatibility wrapper keeps
    # this false, preserving the inspect-after-run single-launch behaviour.
    if launch.conditions.IfCondition(
        LaunchConfiguration('shutdown_on_complete')
    ).evaluate(context):
        nodes.append(RegisterEventHandler(OnProcessExit(
            target_action=runner,
            on_exit=[EmitEvent(event=Shutdown(reason='benchmark runner exited'))],
        )))

    return nodes


def _resolved_topic(namespace: str, topic: str) -> str:
    parts = [namespace.strip('/'), topic.strip('/')]
    return '/' + '/'.join(part for part in parts if part)


def build_readiness_gate(context, *args, **kwargs):
    """Start the config stack only after the persistent layer is genuinely ready."""

    namespace = LaunchConfiguration('namespace').perform(context)
    camera_inputs = resolved_camera_inputs(
        context, 'color_topic', 'camera_info_topic',
        'depth_topic', 'pointcloud_topic')
    gate = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ridgeback_autonomy', 'launch_wait',
            '--topic', _resolved_topic(namespace, RAW_DETECTIONS_TOPIC),
            '--topic', _resolved_topic(namespace, camera_inputs.color_image_topic),
            '--timeout', str(ENV_READY_TIMEOUT_SEC),
        ],
        name='gate_benchmark_environment_ready',
        output='screen',
    )
    return [
        gate,
        RegisterEventHandler(OnProcessExit(
            target_action=gate,
            on_exit=[OpaqueFunction(function=build_benchmark_nodes)],
        )),
    ]


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    workspace_root = workspace_root_from_package_share(pkg_this)
    benchmark_output_dir = default_output_directory(workspace_root)
    namespace = LaunchConfiguration('namespace')

    arguments = [
        # Shared with target_benchmark_env.launch.py. Keep these defaults
        # byte-identical: the first declaration inherited by a wrapper wins.
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='target_distance_calibration'),
        DeclareLaunchArgument(
            'color_topic', default_value=SIMULATION_CAMERA_INPUTS.color_image_topic),
        DeclareLaunchArgument(
            'estimate_viz',
            default_value='true',
            description='Publish estimator rings and the HUD distance readout',
        ),
        DeclareLaunchArgument(
            'overlay',
            default_value='true',
            description='Launch the camera overlay with estimator distances',
        ),
        DeclareLaunchArgument('estimators', default_value='all'),
        DeclareLaunchArgument(
            'scenario',
            default_value='',
            description='Scenario YAML; empty uses benchmark_scenarios_full.yaml',
        ),
        DeclareLaunchArgument('repeats', default_value='5'),
        DeclareLaunchArgument('output_dir', default_value=benchmark_output_dir),
        DeclareLaunchArgument('run_dir_name', default_value=''),
        DeclareLaunchArgument('settle_sec', default_value='2.0'),
        DeclareLaunchArgument('capture_sec', default_value='10.0'),
        DeclareLaunchArgument(
            'replay_dataset_dir', default_value='',
            description='New directory for a V1 projective-ranging replay capture',
        ),
        DeclareLaunchArgument(
            'capture_batches', default_value='0',
            description='Raw detector batches per replay trial; empty batches count',
        ),
        DeclareLaunchArgument(
            'capture_drain_sec', default_value='2.0',
            description='Bounded depth-match drain after the replay batch quota',
        ),
        DeclareLaunchArgument(
            'capture_timeout_sec', default_value='30.0',
            description='Hard stall bound for one replay-capture trial',
        ),
        DeclareLaunchArgument(
            'depth_topic', default_value=SIMULATION_CAMERA_INPUTS.aligned_depth_topic),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=SIMULATION_CAMERA_INPUTS.color_camera_info_topic,
        ),
        DeclareLaunchArgument(
            'depth_source',
            default_value='stereoscopic',
            description='Aligned depth source: stereoscopic or monocular',
        ),
        DeclareLaunchArgument(
            'isolation_2d',
            default_value='nearest_mode_histogram',
            description='Projective-ranging box-gate foreground recipe',
        ),
        # The selected 2D recipe's own numbers. Defaulted to the constants the
        # recipes have always used, so an unset run is byte-identical to the
        # runs already in the archive; they are declared here only so a sweep
        # can measure how far each one moves the estimate.
        DeclareLaunchArgument(
            'isolation_2d_bin_width_m',
            default_value=str(NEAREST_MODE_BIN_WIDTH_M_DEFAULT),
            description='Depth-histogram bin width of the isolation_2d recipe',
        ),
        DeclareLaunchArgument(
            'isolation_2d_band_m',
            default_value=str(NEAR_SURFACE_BAND_M),
            description=(
                'Depth band kept around the near-surface anchor '
                '(nearest_mode_histogram only)'
            ),
        ),
        DeclareLaunchArgument(
            'isolation_2d_min_bin_fraction',
            default_value=str(NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT),
            description=(
                'Fraction of masked depths a bin must hold to anchor on it '
                '(nearest_mode_histogram only)'
            ),
        ),
        DeclareLaunchArgument(
            'min_valid_pixels',
            default_value=str(MIN_VALID_SAMPLES),
            description='Depth rows: foreground samples required for an estimate',
        ),
        DeclareLaunchArgument(
            'isolation_3d',
            default_value=ISOLATION_3D_DEFAULT,
            description='Euclidean-reconstruction box-gate foreground recipe',
        ),
        # The selected 3D recipe's own numbers, on the same terms as the 2D set
        # above. Separate arguments from their 2D namesakes: the 2D histogram
        # bins optical Z and the 3D one bins euclidean range, so a single
        # argument could not honestly set both.
        DeclareLaunchArgument(
            'isolation_3d_floor_margin_m',
            default_value=str(FLOOR_MARGIN_M_DEFAULT),
            description=(
                'Height above the floor a point must clear to survive the '
                'ground-plane crop (height_crop steps only)'
            ),
        ),
        DeclareLaunchArgument(
            'isolation_3d_percentile',
            default_value=str(FRONT_PERCENTILE),
            description=(
                'Range percentile the inlier window anchors on '
                '(range_band only)'
            ),
        ),
        DeclareLaunchArgument(
            'isolation_3d_ahead_m',
            default_value=str(INLIER_AHEAD_MARGIN_M),
            description='Inlier window kept in front of the anchor range',
        ),
        DeclareLaunchArgument(
            'isolation_3d_behind_m',
            default_value=str(INLIER_BEHIND_MARGIN_M),
            description='Inlier window kept behind the anchor range',
        ),
        DeclareLaunchArgument(
            'isolation_3d_bin_width_m',
            default_value=str(NEAREST_MODE_BIN_WIDTH_M_DEFAULT),
            description=(
                'Range-histogram bin width of the isolation_3d recipe '
                '(nearest_mode_band only)'
            ),
        ),
        DeclareLaunchArgument(
            'isolation_3d_min_bin_fraction',
            default_value=str(NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT),
            description=(
                'Fraction of ranges a bin must hold to anchor on it '
                '(nearest_mode_band only)'
            ),
        ),
        DeclareLaunchArgument(
            'mask_depth_max_meters',
            default_value=str(DEPTH_GATE_DISABLED),
            description='Mask-row working depth gate; 0 means unlimited',
        ),
        DeclareLaunchArgument(
            'mask_gate',
            default_value='box',
            description='Mask front-end: box or silhouette',
        ),
        DeclareLaunchArgument(
            'record_video',
            default_value='true',
            description=(
                'Record the RViz window to video/run.mp4. Turn off for timing '
                'runs: the screen grab competes with the simulator render path'
            ),
        ),
        DeclareLaunchArgument(
            'depth_match_debug',
            default_value='false',
            description='Log bounded mask depth-input and stage-timing evidence',
        ),
        DeclareLaunchArgument('scan_topic', default_value='sensors/lidar2d_0/scan'),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value=SIMULATION_CAMERA_INPUTS.organized_points_topic or '',
        ),
        # namespace must be declared before this substitution-backed default.
        DeclareLaunchArgument('base_frame', default_value=[namespace, '/robot/base_link']),
        DeclareLaunchArgument(
            'shutdown_on_complete',
            default_value='false',
            description='Shut this launch service down when the runner exits',
        ),
    ]

    declared_names = frozenset(argument.name for argument in arguments)
    if declared_names != CONFIG_LAUNCH_ARGUMENT_NAMES:
        missing = sorted(CONFIG_LAUNCH_ARGUMENT_NAMES - declared_names)
        extra = sorted(declared_names - CONFIG_LAUNCH_ARGUMENT_NAMES)
        raise RuntimeError(
            f'Config launch argument catalogue drifted: missing={missing}, extra={extra}'
        )

    return LaunchDescription([
        # A sweep starts this launch in a separate process from the environment,
        # so its children need their own copy of the venv environment actions.
        *cyclonedds_actions(pkg_this),
        *perception_venv_actions(pkg_this),
        *arguments,
        OpaqueFunction(function=build_readiness_gate),
    ])
