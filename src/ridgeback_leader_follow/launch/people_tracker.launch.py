"""Launch the simulation people tracker in the configured perception environment."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    python=str(Path(os.environ.get('RIDGEBACK_PERCEPTION_VENV',str(Path.cwd()/'perception_venv')))/'bin/python3')
    return LaunchDescription([
        DeclareLaunchArgument('namespace',default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('camera_candidate',default_value='owlv2'),
        DeclareLaunchArgument('lidar_candidate',default_value='lfe-ppn'),
        DeclareLaunchArgument('appearance_association_weight',default_value='.75'),
        DeclareLaunchArgument('appearance_candidate',default_value='none'),
        DeclareLaunchArgument('appearance_model_dir',default_value=str(Path.cwd()/'artifacts/leader-following/f3-models')),
        DeclareLaunchArgument('track_maximum_age',default_value='.75'),
        DeclareLaunchArgument('observation_timeout',default_value='.15'),
        DeclareLaunchArgument('device',default_value='cuda'),
        DeclareLaunchArgument('model_dir',default_value=str(Path.cwd()/'artifacts/leader-following/f2-models')),
        Node(package='ridgeback_leader_follow',executable='people_tracker',prefix=[python],
             namespace=LaunchConfiguration('namespace'),output='screen',
             parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
                          'camera_candidate':LaunchConfiguration('camera_candidate'),
                          'track_maximum_age':ParameterValue(LaunchConfiguration('track_maximum_age'),value_type=float),
                          'observation_timeout':ParameterValue(LaunchConfiguration('observation_timeout'),value_type=float),
                          'appearance_candidate':LaunchConfiguration('appearance_candidate'),
                          'appearance_association_weight':ParameterValue(LaunchConfiguration('appearance_association_weight'),value_type=float),
                          'appearance_model_dir':LaunchConfiguration('appearance_model_dir'),
                          'device':LaunchConfiguration('device'),'lidar_candidate':LaunchConfiguration('lidar_candidate'),'model_dir':LaunchConfiguration('model_dir')}],
             remappings=[('/tf','tf'),('/tf_static','tf_static')])])
