"""Filesystem policy shared by benchmark launch, standalone runner, and sweep."""

from __future__ import annotations

import os


def default_output_directory(workspace_root: str) -> str:
    return os.path.join(workspace_root, 'artifacts', 'benchmarks')


def subprocess_log_environment(output_directory: str) -> dict[str, str]:
    """Keep child ROS logs with their output, honoring an explicit override."""
    environment = os.environ.copy()
    if not environment.get('ROS_LOG_DIR'):
        environment['ROS_LOG_DIR'] = os.path.join(output_directory, 'logs', 'ros')
    os.makedirs(environment['ROS_LOG_DIR'], exist_ok=True)
    return environment
