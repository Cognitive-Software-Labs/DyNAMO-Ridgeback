"""Launch-layer constants and node specs shared by the public entrypoints.

Both `ridgeback_exploration.launch.py` and the benchmark layers build the same
target-localization stack, so the node specs live here rather than in either: the topic
names below are the contract the measurement nodes publish and the display
nodes subscribe against, and two copies of that contract are free to drift into
measuring off different topics.

This module deliberately stays importable without ROS launch installed: the
sweep spec loader imports the argument-name catalogue for validation, while
the launch actions are imported lazily only when a launch file asks for them.
"""

from __future__ import annotations

import os

from ridgeback_autonomy.common.camera_inputs import (
    SIMULATION_BACKEND,
    resolve_camera_inputs,
)
from ridgeback_autonomy.perception.target_localization.contracts import (
    ALIGNED_DEPTH_DEBUG_TOPIC,
    HUD_DISTANCES_PANEL_TOPIC,
    MASK_MEASUREMENTS_TOPIC,
    POINTCLOUD_MEASUREMENTS_TOPIC,
    RAW_DETECTIONS_TOPIC,
)

# Overlay columns for the RViz strip. Above any possible panel count, and
# pack_panels clamps to that count, so the effect is simply "one row". Both
# entrypoints dock the overlay as a wide, short strip, where the node's own
# 3-column default wraps a four-panel run onto a second row and halves the
# height each panel gets.
OVERLAY_SINGLE_ROW = 99

HUD_DISTANCES_TEXT_SIZE = 16.0

# Every simulation launch resolves the camera topic set from the same backend,
# so the default each of them declares comes from one place.
SIMULATION_CAMERA_INPUTS = resolve_camera_inputs(SIMULATION_BACKEND)

SHARED_BENCHMARK_ARGUMENT_DEFAULTS = {
    'namespace': 'r100_0001',
    'use_sim_time': 'true',
    'world': 'target_distance_calibration',
    'color_topic': 'sensors/camera_0/color/image',
}

# The source launch writes every declaration out explicitly so ``--show-args``
# and the source-layout guards remain useful. This catalogue is the validator's
# matching public contract; test_launch_layout verifies that it does not drift.
CONFIG_LAUNCH_ARGUMENT_NAMES = frozenset({
    'namespace',
    'use_sim_time',
    'world',
    'color_topic',
    'estimate_viz',
    'overlay',
    'estimators',
    'scenario',
    'repeats',
    'output_dir',
    'settle_sec',
    'capture_sec',
    'depth_topic',
    'camera_info_topic',
    'depth_source',
    'isolation_2d',
    'isolation_3d',
    'mask_depth_max_meters',
    'mask_gate',
    'depth_match_debug',
    'scan_topic',
    'pointcloud_topic',
    'base_frame',
    'run_dir_name',
    'shutdown_on_complete',
})

ENV_LAYER_CONFIG_KEYS = frozenset({
    'world',
    'setup_path',
    'namespace',
    'use_sim_time',
    'color_topic',
})


def workspace_root_from_package_share(package_share: str) -> str:
    return os.path.abspath(os.path.join(package_share, '..', '..', '..', '..'))


def perception_venv_actions(package_share: str):
    """Launch actions that expose the repository's perception virtualenv."""

    from launch.actions import SetEnvironmentVariable

    workspace_root = workspace_root_from_package_share(package_share)
    perception_venv_path = os.path.join(workspace_root, 'perception_venv')
    perception_venv_bin = os.path.join(perception_venv_path, 'bin')
    return [
        SetEnvironmentVariable('VIRTUAL_ENV', perception_venv_path),
        SetEnvironmentVariable(
            'PATH',
            os.pathsep.join([perception_venv_bin, os.environ.get('PATH', '')]),
        ),
    ]


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
        package='ridgeback_autonomy',
        executable='target_pointcloud_measurement_node',
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
    isolation_3d=None,
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
        package='ridgeback_autonomy',
        executable='target_mask_measurement_node',
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
                # On a real D435 this must be the driver's
                # aligned_depth_to_color stream: the stereo source converts
                # units, it does not align.
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'scan_topic': scan_topic,
                'isolation_2d': isolation_2d,
                'isolation_3d': isolation_3d,
                'mask_gate': mask_gate,
                'color_topic': color_topic,
                'depth_max_meters': depth_max_meters,
                'depth_match_debug': depth_match_debug,
            },
        ),
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )


def resolved_camera_inputs(context, *argument_names: str):
    """The camera topic set, with whichever launch arguments the caller declares.

    Each entrypoint declares a different subset -- the environment layer only
    has ``color_topic``, the config layers have all four -- so the names come
    from the caller and the resolution rule stays in one place.
    """

    from launch.substitutions import LaunchConfiguration

    return resolve_camera_inputs(SIMULATION_BACKEND, {
        name: LaunchConfiguration(name).perform(context)
        for name in argument_names
    })


def estimate_viz_node(
    *,
    namespace,
    use_sim_time,
    estimators,
    hud_layout=None,
    condition=None,
):
    """The estimator rings and the distance panel that names them.

    ``estimators`` is the run's resolved set, so both surfaces are filtered from
    one list: an unselected row draws no ring, and a ring never appears without
    a column beside it to name it.
    """

    from launch_ros.actions import Node

    return Node(
        package='ridgeback_autonomy',
        executable='target_visualization_node',
        name='target_visualization',
        namespace=namespace,
        parameters=_measurement_parameters(
            {'use_sim_time': use_sim_time, 'estimators': estimators},
            {'hud_layout': hud_layout},
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
        package='ridgeback_autonomy',
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


def distance_hud_node(
    *,
    namespace,
    use_sim_time,
    name,
    overlay_width,
    marker_topic=None,
    condition=None,
):
    """A hud_node aggregator dedicated to the distance panel.

    Dedicated because ``hud_node`` renders through ``QStaticText``, which
    switches the whole overlay to rich text as soon as any tag appears: this
    panel is unconditionally rich, and the velocity and coverage panels line
    their columns up with runs of spaces, which rich text collapses.

    ``overlay_width`` has no default because the two layouts need different
    ones and the overlay CLIPS rather than wraps -- a box that is too narrow
    silently deletes the last column, which reads as that estimator never
    reporting rather than as a layout fault. ``text_size`` is in POINTS, so
    columns-to-pixels follows the display scaling: measured at 12.6 px/column on
    one monitor and 14.4 on another, and each caller sizes for the wider.
    """

    from launch_ros.actions import Node

    return Node(
        package='ridgeback_autonomy',
        executable='hud_node',
        name=name,
        namespace=namespace,
        parameters=_measurement_parameters(
            {
                'use_sim_time': use_sim_time,
                'panels': [HUD_DISTANCES_PANEL_TOPIC],
                'text_size': HUD_DISTANCES_TEXT_SIZE,
                'overlay_width': overlay_width,
                'horizontal_alignment': 'right',
                'vertical_alignment': 'top',
                'rich_text': True,
            },
            {'marker_topic': marker_topic},
        ),
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
        condition=condition,
    )
