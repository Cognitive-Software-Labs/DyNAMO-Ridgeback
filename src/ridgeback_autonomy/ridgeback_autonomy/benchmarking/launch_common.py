"""Launch-layer constants and helpers shared by benchmark entrypoints.

This module deliberately stays importable without ROS launch installed: the
sweep spec loader imports the argument-name catalogue for validation, while
the launch actions are imported lazily only when a launch file asks for them.
"""

from __future__ import annotations

import os


RAW_DETECTIONS_TOPIC = 'detections/g1/raw'

SHARED_BENCHMARK_ARGUMENT_DEFAULTS = {
    'namespace': 'r100_0001',
    'use_sim_time': 'true',
    'world': 'g1_distance_calibration',
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
    'overlay_window',
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
    'depth_max_meters',
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
