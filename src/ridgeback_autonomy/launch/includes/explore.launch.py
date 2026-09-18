import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    frontier_explorer_params_file = os.path.join(pkg_this, 'config', 'frontier_explorer_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *[DeclareLaunchArgument(k, default_value=v) for k,v in {
            'localization_required':'false', 'health_timeout':'3.0',
            'localization_progress_timeout':'15.0', 'cancellation_timeout':'5.0'}.items()],

        Node(
            package='ridgeback_autonomy',
            executable='frontier_explorer_node.py',
            name='frontier_explorer_node',
            namespace=namespace,
            output='screen',
            parameters=[
                frontier_explorer_params_file,
                {'use_sim_time': use_sim_time, **{k: LaunchConfiguration(k) for k in (
                    'localization_required', 'health_timeout', 'localization_progress_timeout', 'cancellation_timeout')}},
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
    ])
