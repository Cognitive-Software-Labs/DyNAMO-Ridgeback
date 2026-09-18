"""Standalone, headless-by-default localization on explicit sensor/TF inputs."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ridgeback_localization.environment import compute_environment
from ridgeback_localization.estimator_registry import (
    parse_estimators, uses_mask_estimators, uses_pointcloud_estimators,
    selected_mask_estimators, selected_pointcloud_estimators,
)
from ridgeback_localization.launch_helpers import mask_measurement_node, pointcloud_measurement_node, overlay_node, estimate_viz_node


def build_nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    namespace = value('namespace')
    clock = value('use_sim_time').lower() == 'true'
    base = value('base_frame').strip()
    if not base: raise ValueError('base_frame must be the observed TF frame ID')
    estimators = parse_estimators(value('estimators'))
    labels = value('target_labels')
    if not any(x.strip() for x in labels.split(',')): raise ValueError('target_labels must not be empty')
    common = dict(namespace=namespace, use_sim_time=clock)
    nodes = [Node(package='ridgeback_localization', executable='target_detector_node',
        namespace=namespace, additional_env=compute_environment(), parameters=[dict(
            use_sim_time=clock, color_topic=value('color_topic'), target_labels=labels,
            detector_fps=float(value('detector_fps')))], output='screen')]
    if uses_mask_estimators(estimators):
        nodes.append(mask_measurement_node(**common, base_frame=base,
            enabled_estimators=','.join(selected_mask_estimators(estimators)),
            color_topic=value('color_topic'), depth_topic=value('depth_topic'),
            camera_info_topic=value('camera_info_topic'), scan_topic=value('scan_topic'),
            depth_source=value('depth_source'), mask_gate=value('mask_gate')))
    if uses_pointcloud_estimators(estimators):
        if not value('pointcloud_topic'): raise ValueError('pointcloud requires an observed organized pointcloud_topic')
        nodes.append(pointcloud_measurement_node(**common, base_frame=base,
            enabled_estimators=','.join(selected_pointcloud_estimators(estimators)),
            color_topic=value('color_topic'), pointcloud_topic=value('pointcloud_topic')))
    health_params = {name:value(name) for name in ('color_topic','depth_topic','camera_info_topic',
        'scan_topic','pointcloud_topic','depth_source','mask_gate','estimators','target_labels')}
    health_params.update({name:float(value(name)) for name in ('startup_timeout','input_timeout','progress_timeout','source_age')})
    nodes.append(Node(package='ridgeback_localization', executable='localization_health', namespace=namespace,
        parameters=[dict(use_sim_time=clock, mode='local', **health_params)], output='screen'))
    if value('displays').lower() == 'true':
        nodes.extend([estimate_viz_node(**common, estimators=','.join(estimators), base_frame=base),
            overlay_node(**common, estimators=','.join(estimators), color_topic=value('color_topic'),
                depth_source=value('depth_source'), mask_gate=value('mask_gate'))])
    return nodes


def generate_launch_description():
    defaults = dict(namespace='r100_0001', use_sim_time='false',
        color_topic='sensors/camera_0/color/image_raw', camera_info_topic='sensors/camera_0/color/camera_info',
        depth_topic='sensors/camera_0/aligned_depth_to_color/image_raw', scan_topic='sensors/lidar2d_0/scan',
        pointcloud_topic='', estimators='projective_ranging,euclidean_reconstruction',
        depth_source='stereoscopic', mask_gate='box', target_labels='humanoid robot', detector_fps='10.0',
        displays='false', startup_timeout='120.0', input_timeout='3.0', progress_timeout='15.0', source_age='15.0')
    return LaunchDescription([DeclareLaunchArgument('base_frame', description='Observed base TF frame ID'),
        *[DeclareLaunchArgument(key, default_value=value) for key,value in defaults.items()],
        OpaqueFunction(function=build_nodes)])
