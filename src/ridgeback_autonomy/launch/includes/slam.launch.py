import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, LifecycleTransition
from lifecycle_msgs.msg import Transition

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    pkg_this = get_package_share_directory('ridgeback_autonomy')

    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')

    # Read namespace from robot.yaml
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    clearpath_config = ClearpathConfig(config)
    namespace = clearpath_config.system.namespace
    scan_topic = f'/{namespace}/sensors/lidar2d_0/scan'

    slam_params_file = os.path.join(pkg_this, 'config', 'slam_toolbox_params.yaml')

    rewritten_params = RewrittenYaml(
        source_file=slam_params_file,
        root_key=namespace,
        param_rewrites={
            'scan_topic': scan_topic,
        },
        convert_types=True,
    )

    # Same TF remapping pattern used by Nav2 nodes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'),
                  ('/map', 'map'), ('/map_metadata', 'map_metadata')]

    return [
        LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace,
            output='screen',
            parameters=[
                rewritten_params,
                {'use_sim_time': use_sim_time, 'use_lifecycle_manager': False},
            ],
            remappings=remappings,
        ),
        TimerAction(
            period=2.0,
            actions=[
                LifecycleTransition(
                    lifecycle_node_names=[f'/{namespace}/slam_toolbox'],
                    transition_ids=[
                        Transition.TRANSITION_CONFIGURE,
                    ],
                ),
            ],
        ),
        TimerAction(
            period=8.0,
            actions=[
                LifecycleTransition(
                    lifecycle_node_names=[f'/{namespace}/slam_toolbox'],
                    transition_ids=[
                        Transition.TRANSITION_ACTIVATE,
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('setup_path',
                              default_value=os.path.expanduser('~/clearpath/')),
        OpaqueFunction(function=launch_setup),
    ])
