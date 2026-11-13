"""Closed-loop leader following in simulation: recognition, Nav2 and follow_controller.

Starts the F3 recognition pipeline (people_tracker + leader_identity), the Nav2
servers configured in config/follow_nav2_params.yaml (odom frame, no map),
people_predictor (predicted bystanders for the costmaps and a speed limit) and
follow_controller. The robot moves only while leader_identity reports LOCKED.
Start a simulator that consumes cmd_vel first, e.g. ``crowd_scene --drive``.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

LIFECYCLE_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                   'velocity_smoother', 'collision_monitor', 'bt_navigator']


def generate_launch_description():
    share = get_package_share_directory('ridgeback_leader_follow')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')
    # Jazzy costmap layers resolve relative observation topics inside the
    # costmap's own namespace (…/local_costmap/sensors/…), where nothing
    # publishes. Give every layer source its fully qualified topic.
    layer_topics = {
        'local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic': 'sensors/lidar2d_0/scan',
        'local_costmap.local_costmap.ros__parameters.voxel_layer.scan_rear.topic': 'sensors/lidar2d_1/scan',
        'local_costmap.local_costmap.ros__parameters.people_layer.people.topic': 'leader_follow/people_obstacles',
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': 'sensors/lidar2d_0/scan',
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan_rear.topic': 'sensors/lidar2d_1/scan',
        'global_costmap.global_costmap.ros__parameters.people_layer.people.topic': 'leader_follow/people_obstacles',
    }
    params = ParameterFile(RewrittenYaml(
        source_file=os.path.join(share, 'config', 'follow_nav2_params.yaml'), root_key=namespace,
        param_rewrites={'autostart': 'true',
                        'bt_navigator.ros__parameters.odom_topic': odom_topic,
                        'controller_server.ros__parameters.odom_topic': odom_topic,
                        'velocity_smoother.ros__parameters.odom_topic': odom_topic,
                        **{key: ['/', namespace, '/', topic] for key, topic in layer_topics.items()}},
        convert_types=True), allow_substs=True)
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def nav2(package, executable, extra_remaps=()):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=[params], remappings=tf + list(extra_remaps))

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='platform/odom',
                              description='Odometry topic: platform/odom (Isaac crowd runner) or '
                                          'platform/odom/filtered (Clearpath Gazebo)'),
        DeclareLaunchArgument('follow_distance', default_value='1.2'),
        DeclareLaunchArgument('hold_tolerance', default_value='0.15'),
        DeclareLaunchArgument('lead_time', default_value='1.5'),
        DeclareLaunchArgument('pace_matching', default_value='true',
                              description="Cap the robot's speed at the leader's pace plus the gap error"),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz with the follow view (people, leader, costmaps, plans)'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'leader_recognition.launch.py')),
            launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items()),
        GroupAction([
            PushRosNamespace(namespace),
            SetParameter('use_sim_time', use_sim_time),
            nav2('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_mppi')]),
            nav2('nav2_smoother', 'smoother_server'),
            nav2('nav2_planner', 'planner_server'),
            nav2('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
            nav2('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
            nav2('nav2_collision_monitor', 'collision_monitor'),
            nav2('nav2_bt_navigator', 'bt_navigator'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_follow', output='screen',
                 parameters=[{'autostart': True, 'node_names': LIFECYCLE_NODES}]),
            Node(package='ridgeback_leader_follow', executable='people_predictor', output='screen',
                 parameters=[{'odom_topic': odom_topic}], remappings=tf),
            Node(package='ridgeback_leader_follow', executable='follow_controller', output='screen',
                 parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                              for name in ('follow_distance', 'hold_tolerance', 'lead_time')},
                             {'pace_matching': ParameterValue(LaunchConfiguration('pace_matching'), value_type=bool)}],
                 remappings=tf),
            Node(package='ridgeback_leader_follow', executable='follow_markers', output='log',
                 condition=IfCondition(LaunchConfiguration('rviz'))),
            Node(package='rviz2', executable='rviz2', output='log',
                 arguments=['-d', os.path.join(share, 'rviz', 'follow.rviz')],
                 condition=IfCondition(LaunchConfiguration('rviz')), remappings=tf),
        ]),
    ])
