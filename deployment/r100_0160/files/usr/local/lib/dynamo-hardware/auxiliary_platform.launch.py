from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('PYTHONPATH', '/usr/local/lib/dynamo-hardware/aux-python' + ':' + os.environ.get('PYTHONPATH','')),
        SetLaunchConfiguration('imu_filter', '/usr/local/lib/dynamo-hardware/imu_filter_corrected.yaml'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource('/usr/local/lib/dynamo-hardware/commissioning_platform.launch.py')),
    ])
