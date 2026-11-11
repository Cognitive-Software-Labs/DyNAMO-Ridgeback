"""Stationary F3 recognition pipeline; never starts a motion controller."""
import os
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    profile=json.loads((Path(get_package_share_directory('ridgeback_leader_follow'))/'config/recognition.json').read_text())
    return LaunchDescription([
        DeclareLaunchArgument('appearance_candidate', default_value=profile['appearance_candidate'],description='Selected F3 simulation appearance model'),
        DeclareLaunchArgument('appearance_association_weight',default_value=str(profile['appearance_association_weight'])),
        DeclareLaunchArgument('acquire_distance_tolerance',default_value=str(profile['acquire_distance_tolerance'])),
        DeclareLaunchArgument('acquire_lateral_tolerance',default_value=str(profile['acquire_lateral_tolerance'])),
        DeclareLaunchArgument('continuity_min_confidence',default_value=str(profile['continuity_min_confidence'])),
        DeclareLaunchArgument('acquire_interval',default_value=str(profile['acquire_interval'])),
        DeclareLaunchArgument('track_maximum_age',default_value=str(profile['track_maximum_age'])),
        DeclareLaunchArgument('observation_timeout',default_value=str(profile['observation_timeout'])),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('match_threshold', default_value=str(profile['match_threshold'])),
        DeclareLaunchArgument('recovery_margin', default_value=str(profile['recovery_margin'])),
        DeclareLaunchArgument('acquire_range', default_value=str(profile['acquire_range'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ridgeback_leader_follow'),'launch','people_tracker.launch.py')),
            launch_arguments={'namespace':LaunchConfiguration('namespace'),
                              'use_sim_time':LaunchConfiguration('use_sim_time'),
                              'appearance_candidate':LaunchConfiguration('appearance_candidate'),
                              'track_maximum_age':LaunchConfiguration('track_maximum_age'),
                              'observation_timeout':LaunchConfiguration('observation_timeout'),
                              'appearance_association_weight':LaunchConfiguration('appearance_association_weight')}.items()),
        Node(package='ridgeback_leader_follow', executable='leader_identity',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
                          'acquire_distance_tolerance':ParameterValue(LaunchConfiguration('acquire_distance_tolerance'),value_type=float),
                          'acquire_lateral_tolerance':ParameterValue(LaunchConfiguration('acquire_lateral_tolerance'),value_type=float),
                          'acquire_interval':ParameterValue(LaunchConfiguration('acquire_interval'),value_type=int),
                          'continuity_min_confidence':ParameterValue(LaunchConfiguration('continuity_min_confidence'),value_type=float),
                          'match_threshold':ParameterValue(LaunchConfiguration('match_threshold'),value_type=float),
                          'recovery_margin':ParameterValue(LaunchConfiguration('recovery_margin'),value_type=float),
                          'acquire_range':ParameterValue(LaunchConfiguration('acquire_range'),value_type=float)}],
             remappings=[('/tf','tf'),('/tf_static','tf_static')])])
