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

from ridgeback_autonomy.benchmarking.estimators import (
    parse_estimators,
    selected_mask_estimators,
    selected_pointcloud_estimators,
    uses_mask_estimators,
    uses_pointcloud_estimators,
)
from ridgeback_autonomy.benchmarking.launch_common import (
    CONFIG_LAUNCH_ARGUMENT_NAMES,
    RAW_DETECTIONS_TOPIC,
    perception_venv_actions,
    workspace_root_from_package_share,
)
from ridgeback_autonomy.perception.core.depth_common import DEPTH_GATE_DISABLED
from ridgeback_autonomy.perception.core.isolation_3d import ISOLATION_3D_DEFAULT


POINTCLOUD_MEASUREMENT_TOPIC = 'measurements/g1/pointcloud'
MASK_MEASUREMENT_TOPIC = 'measurements/g1/mask'

# Overlay columns for the RViz strip. Above any possible panel count, and
# pack_panels clamps to that count, so the effect is simply "one row".
OVERLAY_SINGLE_ROW = 99
# The mask node converts depth itself, at the detection stamp, and republishes
# the result for the overlay panel. Debug-only: nothing measures off this topic.
MASK_ALIGNED_DEPTH_DEBUG_TOPIC = 'debug/g1/mask/aligned_depth'
ENV_READY_TIMEOUT_SEC = 300.0


def build_benchmark_nodes(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    repeats = LaunchConfiguration('repeats')
    output_dir = LaunchConfiguration('output_dir')
    settle_sec = LaunchConfiguration('settle_sec')
    capture_sec = LaunchConfiguration('capture_sec')
    color_topic = LaunchConfiguration('color_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    base_frame = LaunchConfiguration('base_frame')

    selected_estimators = parse_estimators(
        LaunchConfiguration('estimators').perform(context).strip()
    )
    selected_pointcloud = selected_pointcloud_estimators(selected_estimators)
    selected_mask = selected_mask_estimators(selected_estimators)
    needs_pointcloud = uses_pointcloud_estimators(selected_estimators)
    needs_mask = uses_mask_estimators(selected_estimators)

    nodes = []

    if needs_pointcloud:
        nodes.append(
            Node(
                package='ridgeback_autonomy',
                executable='g1_pointcloud_measurement_node',
                name='g1_pointcloud_measurement',
                namespace=namespace,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'detections_topic': RAW_DETECTIONS_TOPIC,
                    'measurement_topic': POINTCLOUD_MEASUREMENT_TOPIC,
                    'color_topic': color_topic,
                    'pointcloud_topic': pointcloud_topic,
                    'base_frame': base_frame,
                    'enabled_estimators': ','.join(selected_pointcloud),
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                output='screen',
            )
        )

    if needs_mask:
        nodes.append(
            Node(
                package='ridgeback_autonomy',
                executable='g1_mask_measurement_node',
                name='g1_mask_measurement',
                namespace=namespace,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'detections_topic': RAW_DETECTIONS_TOPIC,
                    'measurement_topic': MASK_MEASUREMENT_TOPIC,
                    'enabled_estimators': ','.join(selected_mask),
                    'depth_source': LaunchConfiguration('depth_source'),
                    # On a real D435 this must be the driver's
                    # aligned_depth_to_color stream: the stereo source converts
                    # units, it does not align.
                    'depth_topic': depth_topic,
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'aligned_depth_debug_topic': MASK_ALIGNED_DEPTH_DEBUG_TOPIC,
                    'scan_topic': scan_topic,
                    'base_frame': base_frame,
                    'isolation_2d': LaunchConfiguration('isolation_2d'),
                    'isolation_3d': LaunchConfiguration('isolation_3d'),
                    'mask_gate': LaunchConfiguration('mask_gate'),
                    'color_topic': color_topic,
                    'depth_max_meters': LaunchConfiguration('mask_depth_max_meters'),
                    'depth_match_debug': LaunchConfiguration('depth_match_debug'),
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                output='screen',
            )
        )

    nodes.append(
        Node(
            package='ridgeback_autonomy',
            executable='g1_estimate_viz_node',
            name='g1_estimate_viz',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                # The HUD lists only the rows this run actually launched.
                'estimators': ','.join(selected_estimators),
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('estimate_viz')),
        )
    )

    nodes.append(
        Node(
            package='ridgeback_autonomy',
            executable='g1_overlay_node',
            name='g1_overlay',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'measurement_topic': POINTCLOUD_MEASUREMENT_TOPIC,
                'mask_measurement_topic': MASK_MEASUREMENT_TOPIC,
                'color_topic': color_topic,
                'aligned_depth_topic': MASK_ALIGNED_DEPTH_DEBUG_TOPIC,
                'estimators': ','.join(selected_estimators),
                'depth_source': LaunchConfiguration('depth_source'),
                'mask_gate': LaunchConfiguration('mask_gate'),
                'show_window': LaunchConfiguration('overlay_window'),
                'max_cols': OVERLAY_SINGLE_ROW,
                'rgb_panel_labels': False,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('overlay')),
        )
    )

    nodes.append(
        Node(
            package='ridgeback_autonomy',
            executable='hud_node',
            name='hud_node',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'panels': ['hud/g1_distances'],
                'text_size': 16.0,
                'overlay_width': 600,
                'horizontal_alignment': 'right',
                'vertical_alignment': 'top',
                'rich_text': True,
            }],
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('estimate_viz')),
        )
    )

    runner = Node(
        package='ridgeback_autonomy',
        executable='g1_distance_benchmark_runner',
        name='g1_distance_benchmark_runner',
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
            'estimators': ','.join(selected_estimators),
            'pointcloud_measurement_topic': POINTCLOUD_MEASUREMENT_TOPIC,
            'mask_measurement_topic': MASK_MEASUREMENT_TOPIC,
            'color_topic': color_topic,
            'depth_source': LaunchConfiguration('depth_source'),
            'isolation_2d': LaunchConfiguration('isolation_2d'),
            'isolation_3d': LaunchConfiguration('isolation_3d'),
            'mask_gate': LaunchConfiguration('mask_gate'),
            # Recorded, not applied: this records the mask row's actual gate.
            'mask_depth_max_meters': LaunchConfiguration('mask_depth_max_meters'),
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
    color_topic = LaunchConfiguration('color_topic').perform(context)
    gate = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ridgeback_autonomy', 'launch_wait',
            '--topic', _resolved_topic(namespace, RAW_DETECTIONS_TOPIC),
            '--topic', _resolved_topic(namespace, color_topic),
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
    benchmark_output_dir = os.path.join(workspace_root, 'benchmark-results')
    namespace = LaunchConfiguration('namespace')

    arguments = [
        # Shared with g1_benchmark_env.launch.py. Keep these defaults
        # byte-identical: the first declaration inherited by a wrapper wins.
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='g1_distance_calibration'),
        DeclareLaunchArgument('color_topic', default_value='sensors/camera_0/color/image'),
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
        DeclareLaunchArgument(
            'overlay_window',
            default_value='false',
            description='Also open the overlay in its own OpenCV window',
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
        DeclareLaunchArgument('depth_topic', default_value='sensors/camera_0/depth/image'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='sensors/camera_0/color/camera_info',
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
        DeclareLaunchArgument(
            'isolation_3d',
            default_value=ISOLATION_3D_DEFAULT,
            description='Euclidean-reconstruction box-gate foreground recipe',
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
            'depth_match_debug',
            default_value='false',
            description='Log mask depth-input lookup accounting',
        ),
        DeclareLaunchArgument('scan_topic', default_value='sensors/lidar2d_0/scan'),
        DeclareLaunchArgument('pointcloud_topic', default_value='sensors/camera_0/points'),
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
        *perception_venv_actions(pkg_this),
        *arguments,
        OpaqueFunction(function=build_readiness_gate),
    ])
