from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_prefix = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='')

    prefix = LaunchConfiguration('prefix')

    # Nodes
    node_camera_0_gz_bridge = Node(
        name='camera_0_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='r100_0001/sensors/',
        output='screen',
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                    'config_file': '/home/czika-stefania/Documents/DyNAMO/Code/DyNAMO-Ridgeback/clearpath/sensors/config/camera_0.yaml'
                    ,
                }
                ,
            ]
        ,
    )

    node_camera_0_static_tf = Node(
        name='camera_0_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        namespace='r100_0001',
        output='screen',
        arguments=
            [
                '--frame-id'
                ,
                'camera_0_link'
                ,
                '--child-frame-id'
                ,
                'r100_0001/robot/base_link/camera_0'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/tf'
                    ,
                    'tf'
                )
                ,
                (
                    '/tf_static'
                    ,
                    'tf_static'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    node_camera_0_gz_image_bridge = Node(
        name='camera_0_gz_image_bridge',
        executable='image_bridge',
        package='ros_gz_image',
        namespace='r100_0001/sensors/',
        output='screen',
        arguments=
            [
                '/r100_0001/sensors/camera_0/image'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/r100_0001/sensors/camera_0/image'
                    ,
                    '/r100_0001/sensors/camera_0/color/image'
                )
                ,
                (
                    '/r100_0001/sensors/camera_0/image/compressed'
                    ,
                    '/r100_0001/sensors/camera_0/color/compressed'
                )
                ,
                (
                    '/r100_0001/sensors/camera_0/image/compressedDepth'
                    ,
                    '/r100_0001/sensors/camera_0/color/compressedDepth'
                )
                ,
                (
                    '/r100_0001/sensors/camera_0/image/theora'
                    ,
                    '/r100_0001/sensors/camera_0/color/theora'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    node_camera_0_gz_depth_bridge = Node(
        name='camera_0_gz_depth_bridge',
        executable='image_bridge',
        package='ros_gz_image',
        namespace='r100_0001/sensors/',
        output='screen',
        arguments=
            [
                '/r100_0001/sensors/camera_0/depth_image'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/r100_0001/sensors/camera_0/depth_image'
                    ,
                    '/r100_0001/sensors/camera_0/depth/image'
                )
                ,
                (
                    '/r100_0001/sensors/camera_0/depth_image/compressed'
                    ,
                    '/r100_0001/sensors/camera_0/depth/compressed'
                )
                ,
                (
                    '/r100_0001/sensors/camera_0/depth_image/compressedDepth'
                    ,
                    '/r100_0001/sensors/camera_0/depth/compressedDepth'
                )
                ,
                (
                    '/r100_0001/sensors/camera_0/depth_image/theora'
                    ,
                    '/r100_0001/sensors/camera_0/depth/theora'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_prefix)
    ld.add_action(node_camera_0_gz_bridge)
    ld.add_action(node_camera_0_static_tf)
    ld.add_action(node_camera_0_gz_image_bridge)
    ld.add_action(node_camera_0_gz_depth_bridge)
    return ld
