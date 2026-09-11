"""Filesystem policy shared by benchmark launch, runner, sweep, and configurator."""

from __future__ import annotations

import json
import os
from pathlib import Path


BENCHMARK_ROOT_RELATIVE = Path('artifacts/benchmarks')
JOB_ROOT_RELATIVE = Path('artifacts/benchmark-jobs')
RUN_ROOT_RELATIVE = Path('artifacts/configurator/runs')


def default_output_directory(workspace_root: str) -> str:
    return os.path.join(workspace_root, *BENCHMARK_ROOT_RELATIVE.parts)


def anchored_path(workspace_root: str, value: str) -> str:
    """Resolve an operator-supplied path against the workspace root.

    A job file and the process reading it rarely share a parent directory, so a
    relative path otherwise means one directory to the runner, which resolves
    against the job file, and a different one to anything resolving against its
    own working directory. Anchoring both on the workspace root leaves one
    answer, which is what lets an inspected path and an executed path agree.
    """

    text = value.strip()
    if not text:
        return text
    expanded = os.path.expanduser(text)
    if not os.path.isabs(expanded):
        expanded = os.path.join(workspace_root, expanded)
    return os.path.realpath(expanded)


def write_json_atomic(path: str | Path, payload: object) -> None:
    """Replace a JSON file in one step, so no reader sees a half-written one."""

    destination = Path(path)
    temporary = destination.with_name(f'{destination.name}.tmp')
    temporary.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
    os.replace(temporary, destination)


def subprocess_log_environment(output_directory: str) -> dict[str, str]:
    """Keep child ROS logs with their output, honoring an explicit override."""
    environment = os.environ.copy()
    if not environment.get('ROS_LOG_DIR'):
        environment['ROS_LOG_DIR'] = os.path.join(output_directory, 'logs', 'ros')
    os.makedirs(environment['ROS_LOG_DIR'], exist_ok=True)
    return environment
