"""F1-only Gazebo world plus the existing Clearpath robot/sensor bringup."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer
from ridgeback_common.camera_profiles import resolve_camera_profile


def generate_launch_description():
    vendor = get_package_share_directory('clearpath_gz')
    resources = [os.path.join(p, 'share') for p in os.environ['AMENT_PREFIX_PATH'].split(os.pathsep)]
    profile = resolve_camera_profile('640x480')
    return LaunchDescription([
        DeclareLaunchArgument('world_file'), DeclareLaunchArgument('setup_path'),
        SetLaunchConfiguration('sim_camera_width', str(profile.width)),
        SetLaunchConfiguration('sim_camera_height', str(profile.height)),
        SetLaunchConfiguration('sim_camera_horizontal_fov', str(profile.horizontal_fov_rad)),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resources + [os.path.join(vendor, 'meshes')])),
        GzServer(world_sdf_file=LaunchConfiguration('world_file'), verbosity_level='2'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='f1_clock_bridge',
             arguments=['/world/leader_follow_scene/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             remappings=[('/world/leader_follow_scene/clock','/clock')], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(vendor, 'launch/robot_spawn.launch.py')),
            launch_arguments={'world': 'leader_follow_scene',
                              'setup_path': LaunchConfiguration('setup_path'),
                              'use_sim_time': 'true', 'rviz': 'false',
                              'x': '0', 'y': '0', 'z': '0.15', 'yaw': '0'}.items()),
    ])
