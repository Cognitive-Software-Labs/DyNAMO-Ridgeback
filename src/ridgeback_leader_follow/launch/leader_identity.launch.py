"""Run the leader_identity node: People in, LeaderState out."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='ridgeback_leader_follow',
            executable='leader_identity',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
    ])
