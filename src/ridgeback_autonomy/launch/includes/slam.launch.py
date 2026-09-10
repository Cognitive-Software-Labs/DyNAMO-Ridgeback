import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, LifecycleTransition, Node
from launch_ros.event_handlers import OnStateTransition
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
    raw_scan_topic = f'/{namespace}/sensors/lidar2d_0/scan'

    # slam_source: front_only (default, unchanged production behavior) or
    # merged (slam_toolbox reads scan_merger_node's SLAM-only front+rear
    # merge instead of the raw front scan). Nav2/collision_monitor/etc.
    # always keep consuming raw_scan_topic and the rear topic directly,
    # unaffected by this choice either way.
    slam_source = LaunchConfiguration('slam_source').perform(context)
    merged_scan_topic = f'/{namespace}/sensors/scan_slam_merged'
    scan_topic = merged_scan_topic if slam_source == 'merged' else raw_scan_topic

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

    slam_node = LifecycleNode(
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
    )

    # Event-driven lifecycle (replaces fixed 2s/8s timers): configure once the
    # node's change_state service is up, then activate as soon as it reports
    # 'inactive' (i.e. configured).
    gate_configure = ExecuteProcess(
        cmd=['ros2', 'run', 'ridgeback_autonomy', 'launch_wait',
             '--service', f'/{namespace}/slam_toolbox/change_state',
             '--timeout', '30'],
        name='gate_slam_configure', output='screen',
    )
    configure = LifecycleTransition(
        lifecycle_node_names=[f'/{namespace}/slam_toolbox'],
        transition_ids=[Transition.TRANSITION_CONFIGURE],
    )
    activate = LifecycleTransition(
        lifecycle_node_names=[f'/{namespace}/slam_toolbox'],
        transition_ids=[Transition.TRANSITION_ACTIVATE],
    )

    actions = [
        slam_node,
        gate_configure,
        RegisterEventHandler(OnProcessExit(
            target_action=gate_configure, on_exit=[configure],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=slam_node, goal_state='inactive',
            entities=[activate],
        )),
    ]

    if slam_source == 'merged':
        # Publishes ONLY to merged_scan_topic (slam_toolbox's input above);
        # raw front/rear topics keep publishing to every other consumer
        # unaffected by this node. See scan_merger_node.py.
        actions.append(Node(
            package='ridgeback_autonomy',
            executable='scan_merger_node',
            name='scan_merger_node',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            # tf2_ros.TransformListener subscribes to the ABSOLUTE '/tf' and
            # '/tf_static' -- the node's namespace does not move it. Without
            # this remap its buffer stays empty forever (the whole stack
            # publishes into '<ns>/tf'), every odom lookup fails, and each
            # rear scan is silently dropped instead of motion-compensated.
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('setup_path',
                              default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument(
            'slam_source', default_value='front_only',
            choices=['front_only', 'merged'],
            description='front_only (default, production-unchanged) or '
                        'merged (slam_toolbox reads scan_merger_node.py\'s '
                        'SLAM-only front+rear merge instead; Nav2/'
                        'collision_monitor still get raw front/rear topics '
                        'unchanged either way).'),
        OpaqueFunction(function=launch_setup),
    ])
