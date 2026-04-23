import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BACKEND_CONFIGS = {
    'camera': {
        'measurement_executable': 'g1_camera_measurement_node',
        'measurement_name': 'g1_camera_measurement',
        'measurement_topic': 'measurements/g1/camera',
        'default_primary_metric': 'rgb',
        'default_output_csv': '/tmp/g1_distance_benchmark_camera.csv',
        'supported_primary_metrics': {'rgb', 'sensor_depth', 'mono_depth', 'pointcloud'},
    },
    'lidar': {
        'measurement_executable': 'g1_lidar_measurement_node',
        'measurement_name': 'g1_lidar_measurement',
        'measurement_topic': 'measurements/g1/lidar',
        'default_primary_metric': 'lidar',
        'default_output_csv': '/tmp/g1_distance_benchmark_lidar.csv',
        'supported_primary_metrics': {'lidar'},
    },
}


def build_benchmark_nodes(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    repeats = LaunchConfiguration('repeats')
    settle_sec = LaunchConfiguration('settle_sec')
    capture_sec = LaunchConfiguration('capture_sec')
    color_topic = LaunchConfiguration('color_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    base_frame = LaunchConfiguration('base_frame')
    depth_anything_enabled = LaunchConfiguration('depth_anything_enabled')
    failed_frame_dir = LaunchConfiguration('failed_frame_dir')
    save_failed_frames = LaunchConfiguration('save_failed_frames')

    backend = LaunchConfiguration('measurement_backend').perform(context).strip()
    primary_metric = LaunchConfiguration('primary_metric').perform(context).strip()
    output_csv = LaunchConfiguration('output_csv').perform(context).strip()

    if backend not in BACKEND_CONFIGS:
        supported = ', '.join(sorted(BACKEND_CONFIGS))
        raise RuntimeError(
            f'Unsupported measurement_backend "{backend}". Expected one of: {supported}'
        )

    config = BACKEND_CONFIGS[backend]
    resolved_primary_metric = primary_metric or 'auto'
    if resolved_primary_metric == 'auto':
        resolved_primary_metric = config['default_primary_metric']

    if resolved_primary_metric not in config['supported_primary_metrics']:
        supported = ', '.join(sorted(config['supported_primary_metrics']))
        raise RuntimeError(
            f'primary_metric "{resolved_primary_metric}" is not valid for measurement_backend '
            f'"{backend}". Supported values: {supported}'
        )

    resolved_output_csv = output_csv or config['default_output_csv']

    detector_parameters = {
        'use_sim_time': use_sim_time,
        'color_topic': color_topic,
        'detections_topic': 'detections/g1/raw',
    }
    measurement_parameters = {
        'use_sim_time': use_sim_time,
        'detections_topic': 'detections/g1/raw',
    }
    if backend == 'camera':
        measurement_parameters['color_topic'] = color_topic
        measurement_parameters['depth_topic'] = depth_topic
        measurement_parameters['pointcloud_topic'] = pointcloud_topic
        measurement_parameters['base_frame'] = base_frame
        measurement_parameters['depth_anything_enabled'] = depth_anything_enabled
    elif backend == 'lidar':
        measurement_parameters['scan_topic'] = scan_topic
        measurement_parameters['base_frame'] = base_frame

    return [
        Node(
            package='ridgeback_autonomy',
            executable='g1_detector_node',
            name='g1_detector',
            namespace=namespace,
            parameters=[detector_parameters],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
        Node(
            package='ridgeback_autonomy',
            executable=config['measurement_executable'],
            name=config['measurement_name'],
            namespace=namespace,
            parameters=[measurement_parameters],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
        Node(
            package='ridgeback_autonomy',
            executable='g1_distance_benchmark_runner',
            name='g1_distance_benchmark_runner',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'world': world,
                'repeats': repeats,
                'output_csv': resolved_output_csv,
                'settle_sec': settle_sec,
                'capture_sec': capture_sec,
                'measurement_topic': config['measurement_topic'],
                'primary_metric': resolved_primary_metric,
                'color_topic': color_topic,
                'failed_frame_dir': failed_frame_dir,
                'save_failed_frames': save_failed_frames,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    launch_dir = os.path.join(pkg_this, 'launch')
    includes_dir = os.path.join(launch_dir, 'includes')
    workspace_root = os.path.abspath(os.path.join(pkg_this, '..', '..', '..', '..'))
    perception_venv_path = os.path.join(workspace_root, 'perception_venv')
    perception_venv_bin = os.path.join(perception_venv_path, 'bin')

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
        DeclareLaunchArgument('measurement_backend', default_value='camera'),
        DeclareLaunchArgument('primary_metric', default_value='auto'),
        DeclareLaunchArgument('repeats', default_value='5'),
        DeclareLaunchArgument('output_csv', default_value=''),
        DeclareLaunchArgument('settle_sec', default_value='2.0'),
        DeclareLaunchArgument('capture_sec', default_value='3.0'),
        DeclareLaunchArgument('depth_anything_enabled', default_value='false'),
        DeclareLaunchArgument('color_topic', default_value='sensors/camera_0/color/image'),
        DeclareLaunchArgument('depth_topic', default_value='sensors/camera_0/depth/image'),
        DeclareLaunchArgument('scan_topic', default_value='sensors/lidar2d_0/scan'),
        DeclareLaunchArgument('pointcloud_topic', default_value='sensors/camera_0/points'),
        DeclareLaunchArgument('base_frame', default_value=[namespace, '/robot/base_link']),
        DeclareLaunchArgument('failed_frame_dir', default_value=''),
        DeclareLaunchArgument('save_failed_frames', default_value='true'),

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

        OpaqueFunction(function=build_benchmark_nodes),
    ])
