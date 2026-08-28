import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ridgeback_autonomy.benchmarking.estimators import (
    parse_estimators,
    selected_camera_estimators,
    selected_mask_estimators,
    uses_camera_estimators,
    uses_lidar_estimators,
    uses_mask_estimators,
)
from ridgeback_autonomy.perception.core.depth_common import (
    DEPTH_GATE_DISABLED,
    DEPTH_MAX_METERS_DEFAULT,
)
from ridgeback_autonomy.perception.core.isolation_3d import ISOLATION_3D_DEFAULT


CAMERA_MEASUREMENT_TOPIC = 'measurements/g1/camera'
LIDAR_MEASUREMENT_TOPIC = 'measurements/g1/lidar'
MASK_MEASUREMENT_TOPIC = 'measurements/g1/mask'

# Overlay columns for the RViz strip. Above any possible panel count, and
# pack_panels clamps to that count, so the effect is simply "one row".
OVERLAY_SINGLE_ROW = 99
# The mask node converts depth itself, at the detection stamp, and republishes
# the result for the overlay panel. Debug-only: nothing measures off this topic.
MASK_ALIGNED_DEPTH_DEBUG_TOPIC = 'debug/g1/mask/aligned_depth'


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
    selected_camera = selected_camera_estimators(selected_estimators)
    selected_mask = selected_mask_estimators(selected_estimators)
    needs_camera = uses_camera_estimators(selected_estimators)
    needs_lidar = uses_lidar_estimators(selected_estimators)
    needs_mask = uses_mask_estimators(selected_estimators)

    detector_parameters = {
        'use_sim_time': use_sim_time,
        'color_topic': color_topic,
        'detections_topic': 'detections/g1/raw',
    }

    nodes = [
        Node(
            package='ridgeback_autonomy',
            executable='g1_detector_node',
            name='g1_detector',
            namespace=namespace,
            parameters=[detector_parameters],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
    ]

    if needs_camera:
        nodes.append(
            Node(
                package='ridgeback_autonomy',
                executable='g1_camera_measurement_node',
                name='g1_camera_measurement',
                namespace=namespace,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'detections_topic': 'detections/g1/raw',
                    'measurement_topic': CAMERA_MEASUREMENT_TOPIC,
                    'color_topic': color_topic,
                    'depth_topic': depth_topic,
                    'pointcloud_topic': pointcloud_topic,
                    'base_frame': base_frame,
                    'enabled_estimators': ','.join(selected_camera),
                    'depth_anything_enabled': 'depth_anything' in selected_estimators,
                    'depth_max_meters': LaunchConfiguration('depth_max_meters'),
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                output='screen',
            )
        )

    if needs_lidar:
        nodes.append(
            Node(
                package='ridgeback_autonomy',
                executable='g1_lidar_measurement_node',
                name='g1_lidar_measurement',
                namespace=namespace,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'detections_topic': 'detections/g1/raw',
                    'measurement_topic': LIDAR_MEASUREMENT_TOPIC,
                    'scan_topic': scan_topic,
                    'base_frame': base_frame,
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
                    'detections_topic': 'detections/g1/raw',
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
                # The HUD lists the run's own estimators, not the registry:
                # rows for paths this run never launched can only ever read
                # "-- miss", which is the same thing the panel prints for an
                # estimator that ran and found nothing.
                'estimators': ','.join(selected_estimators),
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('estimate_viz')),
        )
    )

    # The overlay follows the run: its panels and labels are built from the
    # selected estimators, so it is useful for mask- and lidar-only runs too,
    # not just camera runs. Gated only on the ``overlay`` flag.
    nodes.append(
        Node(
            package='ridgeback_autonomy',
            executable='g1_overlay_node',
            name='g1_overlay',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'measurement_topic': CAMERA_MEASUREMENT_TOPIC,
                'lidar_measurement_topic': LIDAR_MEASUREMENT_TOPIC,
                'mask_measurement_topic': MASK_MEASUREMENT_TOPIC,
                'color_topic': color_topic,
                'depth_topic': depth_topic,
                'aligned_depth_topic': MASK_ALIGNED_DEPTH_DEBUG_TOPIC,
                'estimators': ','.join(selected_estimators),
                'depth_source': LaunchConfiguration('depth_source'),
                'mask_gate': LaunchConfiguration('mask_gate'),
                'show_window': LaunchConfiguration('overlay_window'),
                # Every panel on one row, so the composite fills the wide RViz
                # panel instead of letterboxing to a third of its width.
                # pack_panels clamps to the panel count, so this just means "one row".
                'max_cols': OVERLAY_SINGLE_ROW,
                # The HUD prints the distances as text RViz draws at full size,
                # so the label block would only cover the robot it annotates.
                'rgb_panel_labels': False,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('overlay')),
        )
    )

    # Renders the distance readout the estimate viz node publishes. Benchmark
    # RViz has no HUD otherwise; the exploration config has carried one all along.
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
                # The widest row is "Euclidean Reconstruction" (24) + distance +
                # signed error + the age column an aged row carries = 47
                # monospace columns, at the 12.6 px/column measured for this
                # font size. Too narrow and the overlay wraps the long labels
                # onto a second line and clips the age off the end -- which
                # would silently undo the fresh/aged distinction on exactly the
                # rows that need it; much wider and the panel is mostly empty.
                'overlay_width': 600,
                # Top-right, clear of the perception overlay docked below the
                # 3D view and of the robot, which sits left of centre.
                'horizontal_alignment': 'right',
                'vertical_alignment': 'top',
                # The distances panel colours each row to match its ring.
                'rich_text': True,
            }],
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('estimate_viz')),
        )
    )

    nodes.append(
        Node(
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
                'settle_sec': settle_sec,
                'capture_sec': capture_sec,
                'estimators': ','.join(selected_estimators),
                'camera_measurement_topic': CAMERA_MEASUREMENT_TOPIC,
                'lidar_measurement_topic': LIDAR_MEASUREMENT_TOPIC,
                'mask_measurement_topic': MASK_MEASUREMENT_TOPIC,
                'color_topic': color_topic,
                'depth_topic': depth_topic,
                'depth_source': LaunchConfiguration('depth_source'),
                'isolation_2d': LaunchConfiguration('isolation_2d'),
                'isolation_3d': LaunchConfiguration('isolation_3d'),
                'mask_gate': LaunchConfiguration('mask_gate'),
                'depth_max_meters': LaunchConfiguration('depth_max_meters'),
                # Recorded, not applied: the runner reads depth_max_meters for
                # the collage. This is here so run.json carries the gate the
                # mask rows actually ran under, which is the axis a gate
                # comparison varies.
                'mask_depth_max_meters': LaunchConfiguration('mask_depth_max_meters'),
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        )
    )

    return nodes


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    launch_dir = os.path.join(pkg_this, 'launch')
    includes_dir = os.path.join(launch_dir, 'includes')
    workspace_root = os.path.abspath(os.path.join(pkg_this, '..', '..', '..', '..'))
    perception_venv_path = os.path.join(workspace_root, 'perception_venv')
    perception_venv_bin = os.path.join(perception_venv_path, 'bin')
    benchmark_output_dir = os.path.join(workspace_root, 'benchmark-results')
    rviz_config = os.path.join(pkg_this, 'sim', 'rviz', 'benchmark.rviz')

    namespace = LaunchConfiguration('namespace')
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    exploration_rviz = LaunchConfiguration('exploration_rviz')

    return LaunchDescription([
        SetEnvironmentVariable('VIRTUAL_ENV', perception_venv_path),
        SetEnvironmentVariable(
            'PATH',
            os.pathsep.join([perception_venv_bin, os.environ.get('PATH', '')]),
        ),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('exploration_rviz', default_value='true',
                              description='Launch the exploration RViz2 config'),
        DeclareLaunchArgument('estimate_viz', default_value='true',
                              description='Publish the per-estimator position rings and the '
                                          'HUD distance readout, and run the HUD aggregator '
                                          'that renders it'),
        DeclareLaunchArgument('overlay', default_value='true',
                              description='Launch the g1_overlay_node camera view with estimator distances'),
        DeclareLaunchArgument('overlay_window', default_value='false',
                              description='Also open the overlay in its own OpenCV window. Off '
                                          'here because the overlay is published for the RViz '
                                          'panel, and a floating window would cover it'),
        DeclareLaunchArgument('setup_path',
                              default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument('world', default_value='g1_distance_calibration'),
        DeclareLaunchArgument(
            'estimators',
            default_value='rgb,sensor_depth,depth_anything,pointcloud,lidar',
        ),
        DeclareLaunchArgument(
            'scenario',
            default_value='',
            description='Path to a benchmark scenario YAML (robots + object '
                        'occluders per scene). Empty uses the packaged '
                        'config/benchmark_scenarios_full.yaml (88 scenes: '
                        'single, multi-robot, occlusion and clutter families).'),
        DeclareLaunchArgument('repeats', default_value='5'),
        DeclareLaunchArgument(
            'output_dir',
            default_value=benchmark_output_dir,
        ),
        DeclareLaunchArgument('settle_sec', default_value='2.0'),
        DeclareLaunchArgument('capture_sec', default_value='10.0'),
        DeclareLaunchArgument('color_topic', default_value='sensors/camera_0/color/image'),
        DeclareLaunchArgument('depth_topic', default_value='sensors/camera_0/depth/image'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='sensors/camera_0/color/camera_info'),
        DeclareLaunchArgument(
            'depth_source',
            default_value='stereoscopic',
            description='How the mask node obtains the aligned depth frame for '
                        'the projective_ranging / euclidean_reconstruction '
                        'rows: "stereoscopic" (convert the depth stream) or '
                        '"monocular" (Depth-Anything on the color stream; '
                        'needs perception_venv). Comparing sources = two runs.',
        ),
        DeclareLaunchArgument(
            'isolation_2d',
            default_value='nearest_mode_histogram',
            description='Projective-ranging rect-branch foreground recipe: '
                        '"nearest_mode_histogram" or "otsu".',
        ),
        DeclareLaunchArgument(
            'isolation_3d',
            default_value=ISOLATION_3D_DEFAULT,
            description='Euclidean-reconstruction rect-branch foreground recipe: '
                        '"height_crop_nearest_mode_band", "height_crop_range_band", '
                        '"height_crop", "range_band", or "nearest_mode_band". The '
                        'two chains differ only in how the background separator '
                        'anchors -- nearest mode vs. percentile. The mode anchor '
                        'is the default because it does not slide as background '
                        'grows; the percentile one is kept selectable because it '
                        'is what the legacy pointcloud estimator does, so a run '
                        'can measure the difference.',
        ),
        DeclareLaunchArgument(
            'depth_max_meters',
            default_value=str(DEPTH_MAX_METERS_DEFAULT),
            description='Working depth gate for the legacy camera rows, and the '
                        'range the depth collage normalizes its colours over. '
                        'Finite because neither has a source ceiling behind it: '
                        'the legacy rows would be left unbounded, and the '
                        'colorizer would render a uniform frame. The mask rows '
                        'have their own gate -- see mask_depth_max_meters.',
        ),
        DeclareLaunchArgument(
            'mask_depth_max_meters',
            default_value=str(DEPTH_GATE_DISABLED),
            description='Working depth gate for the mask rows. Not a validity '
                        'rule: how far a reading can be believed is the depth '
                        'source\'s own ceiling (monocular derives it from the '
                        'checkpoint, stereo declares none), and the tighter of '
                        'the two applies. 0 means no gate, which is the default '
                        '-- set a positive value only to deliberately admit less '
                        'scene. The percentile-anchored isolation_3d recipes are '
                        'sensitive to how much background gets through, so one of '
                        'those alongside a wide gate is the combination that '
                        'reads the wall instead of the robot.',
        ),
        DeclareLaunchArgument(
            'mask_gate',
            default_value='box',
            description='Mask front-end for the mask-based rows: "box" '
                        '(rasterized detection box, no model) or "silhouette" '
                        '(segmentation model prompted with the boxes; needs '
                        'perception_venv). Comparing gates = two runs.',
        ),
        DeclareLaunchArgument(
            'depth_match_debug', default_value='false',
            description='Log depth-input lookup hit/miss accounting from the mask '
                        'node, to separate reception loss from stamp mismatch.'),
        DeclareLaunchArgument('scan_topic', default_value='sensors/lidar2d_0/scan'),
        DeclareLaunchArgument('pointcloud_topic', default_value='sensors/camera_0/points'),
        DeclareLaunchArgument('base_frame', default_value=[namespace, '/robot/base_link']),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(includes_dir, 'camera_optical_tf.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),

        OpaqueFunction(function=build_benchmark_nodes),
    ])
