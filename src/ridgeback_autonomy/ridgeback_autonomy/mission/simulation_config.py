"""Simulation launch inputs; no robot geometry or application policy lives here."""

from pathlib import Path

import yaml

from ridgeback_common.paths import workspace_root
from ridgeback_autonomy.namespace_resolution import namespace_from_robot_yaml


def simulation_setup_path(package_share, requested, *, backend, namespace, demo=False):
    canonical = workspace_root(package_share) / 'clearpath'
    selected = Path(requested).expanduser().resolve() if requested else canonical
    if namespace_from_robot_yaml(str(selected)) != namespace.strip('/'):
        raise ValueError(f'{selected}/robot.yaml namespace differs from namespace:={namespace}')
    if backend == 'isaac' or demo:
        # Isaac loads a committed USD, not a fresh import of an arbitrary YAML.
        # Refuse a different sensor rig instead of silently publishing false TF.
        reference = yaml.safe_load((canonical / 'robot.yaml').read_text())
        actual = yaml.safe_load((selected / 'robot.yaml').read_text())
        if any(actual.get(key) != reference.get(key) for key in ('sensors', 'mounts', 'links')):
            raise ValueError(
                f'{selected}/robot.yaml does not match the repository simulator sensor rig; '
                f'use setup_path:={canonical}/ (the Isaac USD is a committed import)')
    return str(selected) + '/'


def demo_world(backend):
    # The Isaac layer disables the world-authored distant G1 only. Gazebo's
    # depot has no authored humanoid; its native Gazebo name is warehouse.
    return 'parking_demo' if backend == 'isaac' else 'depot'
