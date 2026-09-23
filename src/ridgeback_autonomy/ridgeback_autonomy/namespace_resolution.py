"""Resolve the ROS namespace shared by an exploration launch."""

from __future__ import annotations

from pathlib import Path
import re

import yaml


SIMULATION_NAMESPACE = 'r100_0001'
_NAMESPACE_PATTERN = re.compile(
    r'^[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$'
)


def normalize_namespace(value: object, *, source: str) -> str:
    """Return the slash-free namespace form expected by this repository."""

    namespace = str(value or '').strip().strip('/')
    if not namespace or not _NAMESPACE_PATTERN.fullmatch(namespace):
        raise ValueError(
            f'Invalid ROS namespace {value!r} from {source}; expected one or '
            'more slash-separated ROS name tokens')
    return namespace


def namespace_from_robot_yaml(setup_path: str) -> str:
    """Read the Clearpath namespace, failing with an operator-facing message."""

    robot_yaml = Path(setup_path).expanduser() / 'robot.yaml'
    try:
        document = yaml.safe_load(robot_yaml.read_text(encoding='utf-8'))
        value = document['system']['ros2']['namespace']
    except (OSError, TypeError, KeyError, yaml.YAMLError) as exc:
        raise RuntimeError(
            f'Cannot resolve the hardware namespace from {robot_yaml}: {exc}. '
            'Restore the robot-local Clearpath configuration or pass '
            'namespace:=<observed_namespace> explicitly.') from exc
    return normalize_namespace(value, source=str(robot_yaml))


def resolve_namespace(*, backend: str, requested: str, setup_path: str) -> str:
    """Resolve an explicit override, hardware config, or simulator default."""

    if requested.strip():
        return normalize_namespace(requested, source='namespace launch argument')
    if backend == 'hardware':
        return namespace_from_robot_yaml(setup_path)
    return SIMULATION_NAMESPACE
