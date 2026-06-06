import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
)


CAMERA_MEASUREMENT_TOPIC = 'measurements/g1/camera'
LIDAR_MEASUREMENT_TOPIC = 'measurements/g1/lidar'


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
                'color_topic': color_topic,
                'depth_topic': depth_topic,
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

    namespace = LaunchConfiguration('namespace')
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')

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
        DeclareLaunchArgument('scan_topic', default_value='sensors/lidar2d_0/scan'),
        DeclareLaunchArgument('pointcloud_topic', default_value='sensors/camera_0/points'),
        DeclareLaunchArgument('base_frame', default_value=[namespace, '/robot/base_link']),

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
