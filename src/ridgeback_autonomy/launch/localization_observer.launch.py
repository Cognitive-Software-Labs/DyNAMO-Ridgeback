"""Stationary Intel result viewer: no sensor bringup, navigation, or inference."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ridgeback_localization.launch_helpers import estimate_viz_node, overlay_node
from ridgeback_localization.estimator_registry import parse_estimators


def build_nodes(context):
    value=lambda key:LaunchConfiguration(key).perform(context)
    base=value('base_frame').strip()
    if not base: raise ValueError('base_frame must be the observed TF frame ID')
    namespace=value('namespace')
    common=dict(namespace=namespace,use_sim_time=value('use_sim_time')=='true')
    selected=','.join(parse_estimators(value('estimators')))
    nodes=[Node(package='ridgeback_autonomy',executable='localization_status',namespace=namespace,
        parameters=[dict(use_sim_time=common['use_sim_time'],health_timeout=float(value('health_timeout')))],output='screen')]
    if value('displays')=='true':
        nodes.extend([estimate_viz_node(**common,estimators=selected,base_frame=base,hud_layout='wide'),
            overlay_node(**common,estimators=selected,color_topic=value('color_topic'),
                depth_source=value('depth_source'),mask_gate=value('mask_gate'))])
    return nodes


def generate_launch_description():
    defaults=dict(namespace='r100_0001',use_sim_time='false',displays='true',
        estimators='projective_ranging,euclidean_reconstruction',
        color_topic='sensors/camera_0/color/image_raw',depth_source='stereoscopic',mask_gate='box',health_timeout='3.0')
    return LaunchDescription([DeclareLaunchArgument('base_frame',description='Observed base TF frame ID'),
        *[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],OpaqueFunction(function=build_nodes)])
