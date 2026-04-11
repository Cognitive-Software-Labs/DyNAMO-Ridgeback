import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_slam_exploration')
    launch_dir = os.path.join(pkg_this, 'launch')
    fastrtps_config = os.path.join(pkg_this, 'config', 'fastrtps_no_shm.xml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    repeats = LaunchConfiguration('repeats')
    output_csv = LaunchConfiguration('output_csv')
    settle_sec = LaunchConfiguration('settle_sec')
    capture_sec = LaunchConfiguration('capture_sec')
    base_frame = LaunchConfiguration('base_frame')
    color_topic = LaunchConfiguration('color_topic')
    failed_frame_dir = LaunchConfiguration('failed_frame_dir')
    save_failed_frames = LaunchConfiguration('save_failed_frames')

    return LaunchDescription([
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastrtps_config),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('setup_path', default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument('world', default_value='g1_distance_calibration'),
        DeclareLaunchArgument('repeats', default_value='5'),
        DeclareLaunchArgument('output_csv', default_value='/tmp/g1_pointcloud_distance_benchmark.csv'),
        DeclareLaunchArgument('settle_sec', default_value='2.0'),
        DeclareLaunchArgument('capture_sec', default_value='3.0'),
        DeclareLaunchArgument('base_frame', default_value=[namespace, '/robot/base_link']),
        DeclareLaunchArgument('color_topic', default_value='sensors/camera_0/color/image'),
        DeclareLaunchArgument('failed_frame_dir', default_value=''),
        DeclareLaunchArgument('save_failed_frames', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'simulation.launch.py')
            ),
            launch_arguments={
                'setup_path': setup_path,
                'world': world,
                'headless': 'true',
                'clearpath_rviz': 'false',
            }.items(),
        ),

        Node(
            package='ridgeback_slam_exploration',
            executable='g1_detection_pointcloud_node',
            name='g1_detection_pointcloud',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': base_frame,
                'color_topic': color_topic,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),

        Node(
            package='ridgeback_slam_exploration',
            executable='g1_distance_benchmark_runner',
            name='g1_pointcloud_distance_benchmark_runner',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'world': world,
                'repeats': repeats,
                'output_csv': output_csv,
                'settle_sec': settle_sec,
                'capture_sec': capture_sec,
                'detection_topic': 'vlm/g1_detection_pointcloud',
                'primary_metric': 'pointcloud',
                'color_topic': color_topic,
                'failed_frame_dir': failed_frame_dir,
                'save_failed_frames': save_failed_frames,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
    ])
