"""Reusable localization launch wiring; no autonomy or simulator imports."""
from ridgeback_common.camera_inputs import SIMULATION_BACKEND, resolve_camera_inputs
from ridgeback_localization.contracts import (
    ALIGNED_DEPTH_DEBUG_TOPIC, MASK_MEASUREMENTS_TOPIC, POINTCLOUD_MEASUREMENTS_TOPIC,
    RAW_DETECTIONS_TOPIC,
)
from ridgeback_localization.environment import compute_environment
OVERLAY_SINGLE_ROW = 99

def _measurement_parameters(wiring: dict, tuning: dict) -> list:
    """Fixed topic wiring plus whichever tuning values the caller supplied.

    ``None`` means "leave the node's own default", which is the difference
    between the two entrypoints: the benchmark passes every axis it sweeps, and
    exploration passes only the ones it declares an argument for. Setting a
    parameter to the value the node already defaults to would be a second copy
    of that default, free to drift from it.
    """

    return [{**wiring, **{key: value for key, value in tuning.items() if value is not None}}]


def pointcloud_measurement_node(
    *,
    namespace,
    use_sim_time,
    enabled_estimators,
    base_frame,
    color_topic=None,
    pointcloud_topic=None,
):
    """The ``pointcloud`` row's producer, wired to the shared measurement topic.

    Shared by the benchmark and exploration entrypoints so the two cannot drift
    into measuring different things: the topic names here are the contract the
    viz node and the overlay subscribe against.
    """

    from launch_ros.actions import Node

    return Node(
        package='ridgeback_localization',
        executable='target_pointcloud_measurement_node',
        additional_env=compute_environment(),
        name='target_pointcloud_measurement',
        namespace=namespace,
        parameters=_measurement_parameters(
            {
                'use_sim_time': use_sim_time,
                'detections_topic': RAW_DETECTIONS_TOPIC,
                'measurement_topic': POINTCLOUD_MEASUREMENTS_TOPIC,
                'base_frame': base_frame,
                'enabled_estimators': enabled_estimators,
            },
            {
                'color_topic': color_topic,
                'pointcloud_topic': pointcloud_topic,
            },
        ),
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )


def mask_measurement_node(
    *,
    namespace,
    use_sim_time,
    enabled_estimators,
    base_frame,
    depth_source=None,
    depth_topic=None,
    camera_info_topic=None,
    scan_topic=None,
    isolation_2d=None,
    isolation_2d_bin_width_m=None,
    isolation_2d_band_m=None,
    isolation_2d_min_bin_fraction=None,
    min_valid_pixels=None,
    isolation_3d=None,
    isolation_3d_floor_margin_m=None,
    isolation_3d_percentile=None,
    isolation_3d_ahead_m=None,
    isolation_3d_behind_m=None,
    isolation_3d_bin_width_m=None,
    isolation_3d_min_bin_fraction=None,
    polar_range_jump_m=None,
    polar_range_band_m=None,
    polar_min_valid_rays=None,
    mask_gate=None,
    color_topic=None,
    depth_max_meters=None,
    depth_match_debug=None,
):
    """The three mask rows' producer, wired to the shared measurement topic.

    ``base_frame`` is required rather than optional because the node's own
    default is the bare string ``base_link`` -- unlike the pointcloud and viz
    nodes, which derive a namespaced frame from ``get_namespace()``. Under a
    namespace that default resolves to nothing and polar profiling's scan->base
    lookup silently fails, so no caller may fall back to it.
    """

    from launch_ros.actions import Node

    return Node(
        package='ridgeback_localization',
        executable='target_mask_measurement_node',
        additional_env=compute_environment(),
        name='target_mask_measurement',
        namespace=namespace,
        parameters=_measurement_parameters(
            {
                'use_sim_time': use_sim_time,
                'detections_topic': RAW_DETECTIONS_TOPIC,
                'measurement_topic': MASK_MEASUREMENTS_TOPIC,
                'aligned_depth_debug_topic': ALIGNED_DEPTH_DEBUG_TOPIC,
                'base_frame': base_frame,
                'enabled_estimators': enabled_estimators,
            },
            {
                'depth_source': depth_source,
                # On hardware this must be the RealSense driver's
                # aligned_depth_to_color stream: the stereo source converts
                # units, it does not align.
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'scan_topic': scan_topic,
                'isolation_2d': isolation_2d,
                # Each recipe's own numbers, plus the depth rows' shared
                # sufficiency floor. Swept by the benchmark; exploration leaves
                # every one of them unset and gets the node's defaults.
                'isolation_2d_bin_width_m': isolation_2d_bin_width_m,
                'isolation_2d_band_m': isolation_2d_band_m,
                'isolation_2d_min_bin_fraction': isolation_2d_min_bin_fraction,
                'min_valid_pixels': min_valid_pixels,
                'isolation_3d': isolation_3d,
                'isolation_3d_floor_margin_m': isolation_3d_floor_margin_m,
                'isolation_3d_percentile': isolation_3d_percentile,
                'isolation_3d_ahead_m': isolation_3d_ahead_m,
                'isolation_3d_behind_m': isolation_3d_behind_m,
                'isolation_3d_bin_width_m': isolation_3d_bin_width_m,
                'isolation_3d_min_bin_fraction': isolation_3d_min_bin_fraction,
                # Polar profiling's foreground isolation, the LiDAR analogue of
                # the isolation_2d / isolation_3d recipe numbers above. Neither
                # jump nor band is grounded in a measurement yet, which is what
                # exposing them here is for.
                'polar_range_jump_m': polar_range_jump_m,
                'polar_range_band_m': polar_range_band_m,
                'polar_min_valid_rays': polar_min_valid_rays,
                'mask_gate': mask_gate,
                'color_topic': color_topic,
                'depth_max_meters': depth_max_meters,
                'depth_match_debug': depth_match_debug,
            },
        ),
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )


def resolved_camera_inputs(
    context,
    *argument_names: str,
    backend: str = SIMULATION_BACKEND,
):
    """The camera topic set, with whichever launch arguments the caller declares.

    Each entrypoint declares a different subset -- the environment layer only
    has ``color_topic``, the config layers have all four -- so the names come
    from the caller and the resolution rule stays in one place.
    """

    from launch.substitutions import LaunchConfiguration

    return resolve_camera_inputs(backend, {
        name: LaunchConfiguration(name).perform(context)
        for name in argument_names
    })


def estimate_viz_node(
    *,
    namespace,
    use_sim_time,
    estimators,
    hud_layout=None,
    base_frame=None,
    condition=None,
):
    """The estimator rings and the distance panel that names them.

    ``estimators`` is the run's resolved set, so both surfaces are filtered from
    one list: an unselected row draws no ring, and a ring never appears without
    a column beside it to name it.
    """

    from launch_ros.actions import Node

    return Node(
        package='ridgeback_localization',
        executable='target_visualization_node',
        name='target_visualization',
        namespace=namespace,
        parameters=_measurement_parameters(
            {'use_sim_time': use_sim_time, 'estimators': estimators},
            {'hud_layout': hud_layout, 'base_frame': base_frame},
        ),
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
        condition=condition,
    )


def overlay_node(
    *,
    namespace,
    use_sim_time,
    estimators,
    color_topic,
    depth_source=None,
    mask_gate=None,
    rgb_panel_labels=None,
    condition=None,
):
    """The camera overlay, one panel per selected path, packed into one row.

    ``max_cols`` is fixed rather than optional: both entrypoints dock this as a
    wide, short strip, and the node's own 3-column default wraps a four-panel
    run onto a second row and halves the height every panel gets.
    """

    from launch_ros.actions import Node

    return Node(
        package='ridgeback_localization',
        executable='target_overlay_node',
        name='target_overlay',
        namespace=namespace,
        parameters=_measurement_parameters(
            {
                'use_sim_time': use_sim_time,
                'estimators': estimators,
                'measurement_topic': POINTCLOUD_MEASUREMENTS_TOPIC,
                'mask_measurement_topic': MASK_MEASUREMENTS_TOPIC,
                'aligned_depth_topic': ALIGNED_DEPTH_DEBUG_TOPIC,
                'color_topic': color_topic,
                'max_cols': OVERLAY_SINGLE_ROW,
            },
            {
                'depth_source': depth_source,
                'mask_gate': mask_gate,
                'rgb_panel_labels': rgb_panel_labels,
            },
        ),
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
        condition=condition,
    )

