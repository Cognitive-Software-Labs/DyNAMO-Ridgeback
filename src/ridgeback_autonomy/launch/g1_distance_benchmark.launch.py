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
    uses_camera_estimators,
    uses_lidar_estimators,
    uses_mask_estimators,
)


CAMERA_MEASUREMENT_TOPIC = 'measurements/g1/camera'
LIDAR_MEASUREMENT_TOPIC = 'measurements/g1/lidar'
MASK_MEASUREMENT_TOPIC = 'measurements/g1/mask'
ALIGNED_DEPTH_TOPIC = 'perception/aligned_depth/image'
ALIGNED_CAMERA_INFO_TOPIC = 'perception/aligned_depth/camera_info'


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
                executable='aligned_depth_node',
                name='aligned_depth',
                namespace=namespace,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'depth_source': LaunchConfiguration('depth_source'),
                    'depth_topic': depth_topic,
                    'color_topic': color_topic,
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'aligned_depth_topic': ALIGNED_DEPTH_TOPIC,
                    'aligned_camera_info_topic': ALIGNED_CAMERA_INFO_TOPIC,
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                output='screen',
            )
        )
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
                    'aligned_depth_topic': ALIGNED_DEPTH_TOPIC,
                    'aligned_camera_info_topic': ALIGNED_CAMERA_INFO_TOPIC,
                    'scan_topic': scan_topic,
                    'isolation_2d': LaunchConfiguration('isolation_2d'),
                    'isolation_3d': LaunchConfiguration('isolation_3d'),
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
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('estimate_viz')),
        )
    )

    if needs_camera:
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
                    'color_topic': color_topic,
                    'depth_topic': depth_topic,
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                output='screen',
                condition=launch.conditions.IfCondition(LaunchConfiguration('overlay')),
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
        DeclareLaunchArgument('estimate_viz', default_value='false',
                              description='Launch the g1_estimate_viz_node RViz marker publisher'),
        DeclareLaunchArgument('overlay', default_value='true',
                              description='Launch the g1_overlay_node OpenCV camera view with estimator distances'),
        DeclareLaunchArgument('setup_path',
                              default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument('world', default_value='g1_distance_calibration'),
        DeclareLaunchArgument(
            'estimators',
            default_value='rgb,sensor_depth,depth_anything,pointcloud,lidar',
        ),
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
            description='Aligned-depth producer for the projective_ranging / '
                        'euclidean_reconstruction rows: "stereoscopic" or '
                        '"monocular". Comparing sources = two runs.',
        ),
        DeclareLaunchArgument(
            'isolation_2d',
            default_value='nearest_mode_histogram',
            description='Projective-ranging rect-branch foreground recipe: '
                        '"nearest_mode_histogram" or "otsu".',
        ),
        DeclareLaunchArgument(
            'isolation_3d',
            default_value='height_crop_range_band',
            description='Euclidean-reconstruction rect-branch foreground recipe: '
                        '"height_crop_range_band", "height_crop", or "range_band".',
        ),
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
