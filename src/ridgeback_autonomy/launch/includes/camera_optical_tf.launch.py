import launch.conditions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument(
            'use_sim_time',
            description='Must be set to "true" in simulation, "false" on real hardware. '
                        'Controls whether the sim-only camera optical TF publisher starts.',
        ),

        # Simulation-only: publish the camera optical-frame TF that the RealSense driver
        # provides on real hardware. See docs/ISSUES.md "Camera Optical Frame TF in Simulation".
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_0_color_optical_tf',
            namespace=namespace,
            arguments=[
                '--x', '0.0', '--y', '0.015', '--z', '0.0',
                '--roll', '-1.5707963267948966',
                '--pitch', '0.0',
                '--yaw', '-1.5707963267948966',
                '--frame-id', 'camera_0_link',
                '--child-frame-id', 'camera_0_color_optical_frame',
            ],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            condition=launch.conditions.IfCondition(use_sim_time),
        ),
    ])
