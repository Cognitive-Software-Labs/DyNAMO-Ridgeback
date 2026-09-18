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

from ridgeback_common.camera_inputs import (
    REALSENSE_BACKEND,
    SIMULATION_BACKEND,
    resolve_camera_inputs,
)
from ridgeback_common.camera_profiles import DEFAULT_CAMERA_PROFILE
from ridgeback_localization.contracts import (
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
REALSENSE_CAMERA_INPUTS = resolve_camera_inputs(REALSENSE_BACKEND)

SHARED_BENCHMARK_ARGUMENT_DEFAULTS = {
    'namespace': 'r100_0001',
    'use_sim_time': 'true',
    'world': 'target_distance_calibration',
    'camera_profile': DEFAULT_CAMERA_PROFILE,
    'color_topic': 'sensors/camera_0/color/image',
}

# The source launch writes every declaration out explicitly so ``--show-args``
# and the source-layout guards remain useful. This catalogue is the validator's
# matching public launch contract; replay profile stage ownership lives in the
# ROS-free benchmarking registry.
CONFIG_LAUNCH_ARGUMENT_NAMES = frozenset({
    'target_labels',
    'namespace',
    'use_sim_time',
    'world',
    'camera_profile',
    'color_topic',
    'estimate_viz',
    'overlay',
    'estimators',
    'scenario',
    'repeats',
    'output_dir',
    'settle_sec',
    'capture_sec',
    'replay_dataset_dir',
    'sensor_capture_dir',
    'capture_batches',
    'capture_drain_sec',
    'capture_timeout_sec',
    'depth_topic',
    'camera_info_topic',
    'depth_source',
    'isolation_2d',
    'isolation_2d_bin_width_m',
    'isolation_2d_band_m',
    'isolation_2d_min_bin_fraction',
    'min_valid_pixels',
    'isolation_3d',
    'isolation_3d_floor_margin_m',
    'isolation_3d_percentile',
    'isolation_3d_ahead_m',
    'isolation_3d_behind_m',
    'isolation_3d_bin_width_m',
    'isolation_3d_min_bin_fraction',
    'polar_range_jump_m',
    'polar_range_band_m',
    'polar_min_valid_rays',
    'mask_depth_max_meters',
    'mask_gate',
    'depth_match_debug',
    'record_video',
    'scan_topic',
    'pointcloud_topic',
    'base_frame',
    'run_dir_name',
    'shutdown_on_complete',
})

ENV_LAYER_CONFIG_KEYS = frozenset({
    'world',
    'camera_profile',
    'setup_path',
    'namespace',
    'use_sim_time',
    'color_topic',
    'headless_rendering',
    # The detector lives in the persistent layer, so its rate and diagnostic
    # are properties of a whole sweep. A per-config value would silently apply
    # to every other config too.
    'detector_fps',
    'detector_debug',
    'target_labels',
})

def workspace_root_from_package_share(package_share: str) -> str:
    from ridgeback_common.paths import workspace_root
    return str(workspace_root(package_share))


def cyclonedds_actions(package_share: str):
    """Point CycloneDDS at this workspace's config, unless one is already set.

    Cyclone's default participant-index ceiling is below what exploration
    needs: it starts 51 processes, and on the default setting slam_toolbox and
    the entire Nav2 stack abort with "failed to find a free participant index".
    The benchmark stays under the ceiling, which is why the default survived
    the switch to CycloneDDS unnoticed.

    An operator who has already chosen a configuration keeps it -- this only
    fills in a value where none exists, so it cannot silently discard tuning
    that someone set deliberately.
    """

    from launch.actions import SetEnvironmentVariable

    if os.environ.get('CYCLONEDDS_URI'):
        return []
    config = os.path.join(package_share, 'config', 'cyclonedds.xml')
    if not os.path.isfile(config):
        return []
    return [SetEnvironmentVariable('CYCLONEDDS_URI', config)]














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

from ridgeback_localization.launch_helpers import (
    _measurement_parameters, pointcloud_measurement_node, mask_measurement_node,
    resolved_camera_inputs, estimate_viz_node, overlay_node,
)
