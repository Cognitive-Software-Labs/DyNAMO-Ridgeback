"""Expand the Clearpath robot description and resolve frames from it.

The description, not a copied constant, owns where the sensors sit. This module
turns ``clearpath/robot.yaml`` into the expanded URDF exactly the way the
simulators receive it (``DescriptionGenerator`` then ``xacro`` with the shared
camera profile mappings) and resolves full rigid transforms from ``base_link``
to any frame in that URDF.

The Clearpath generator and ``xacro`` are imported lazily, so importing this
module never requires a sourced ROS workspace; only the expansion does.
"""

from __future__ import annotations

from collections.abc import Mapping
import math
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from ridgeback_common.camera_profiles import (
    DEFAULT_CAMERA_PROFILE,
    resolve_camera_profile,
)


DEFAULT_NAMESPACE = 'r100_0001'
ROOT_FRAME = 'base_link'


def xacro_mappings(
    *,
    is_sim: bool,
    profile_name: str = DEFAULT_CAMERA_PROFILE,
    namespace: str = DEFAULT_NAMESPACE,
) -> dict[str, str]:
    """The xacro arguments the launch files pass for one camera profile."""

    profile = resolve_camera_profile(profile_name)
    return {
        'is_sim': 'true' if is_sim else 'false',
        'namespace': namespace,
        'gazebo_controllers': '/dev/null',
        'sim_camera_width': str(profile.width),
        'sim_camera_height': str(profile.height),
        'sim_camera_horizontal_fov': str(profile.horizontal_fov_rad),
    }


def generate_setup(robot_yaml: str | Path, setup_path: str | Path) -> Path:
    """Copy ``robot_yaml`` into ``setup_path`` and run the Clearpath generator there.

    ``setup_path`` must be a directory the caller owns: the generator writes
    ``robot.urdf.xacro`` and friends next to the copied YAML.
    """

    from clearpath_generator_common.description.generator import DescriptionGenerator

    path = Path(setup_path)
    path.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(robot_yaml, path / 'robot.yaml')
    DescriptionGenerator(str(path)).generate()
    return path


def expand_description_xml(
    setup_path: str | Path,
    *,
    is_sim: bool,
    profile_name: str = DEFAULT_CAMERA_PROFILE,
    namespace: str = DEFAULT_NAMESPACE,
) -> str:
    """Run xacro on a generated setup directory; return the URDF text."""

    import xacro

    document = xacro.process_file(
        str(Path(setup_path) / 'robot.urdf.xacro'),
        mappings=xacro_mappings(
            is_sim=is_sim, profile_name=profile_name, namespace=namespace),
    )
    return document.toxml()


def expand_description(
    setup_path: str | Path,
    *,
    is_sim: bool,
    profile_name: str = DEFAULT_CAMERA_PROFILE,
    namespace: str = DEFAULT_NAMESPACE,
) -> ET.Element:
    """Like :func:`expand_description_xml`, parsed into an element tree."""

    return ET.fromstring(expand_description_xml(
        setup_path, is_sim=is_sim, profile_name=profile_name, namespace=namespace))


def expand_robot_yaml(
    robot_yaml: str | Path,
    *,
    is_sim: bool = True,
    profile_name: str = DEFAULT_CAMERA_PROFILE,
    namespace: str = DEFAULT_NAMESPACE,
) -> ET.Element:
    """Generate and expand ``robot_yaml`` in a private temporary directory."""

    with tempfile.TemporaryDirectory(prefix='clearpath_setup_') as directory:
        setup = generate_setup(robot_yaml, directory)
        return expand_description(
            setup, is_sim=is_sim, profile_name=profile_name, namespace=namespace)


def joint(urdf: ET.Element, name: str) -> ET.Element:
    """The single joint named ``name``; anything else is an error."""

    joints = [item for item in urdf.findall('joint') if item.get('name') == name]
    if len(joints) != 1:
        raise LookupError(f'expected exactly one joint named {name}, found {len(joints)}')
    return joints[0]


def origin(element: ET.Element) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """``(xyz, rpy)`` of a joint's or visual's ``<origin>``, URDF defaults applied."""

    node = element.find('origin')
    if node is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = tuple(float(value) for value in (node.get('xyz') or '0 0 0').split())
    rpy = tuple(float(value) for value in (node.get('rpy') or '0 0 0').split())
    return xyz, rpy


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF/SDF fixed-axis roll-pitch-yaw: ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def pose_matrix(xyz, rpy) -> np.ndarray:
    """4x4 homogeneous transform of a translation and URDF roll-pitch-yaw."""

    matrix = np.eye(4)
    matrix[:3, :3] = rpy_matrix(*rpy)
    matrix[:3, 3] = xyz
    return matrix


def matrix_rpy(matrix: np.ndarray) -> tuple[float, float, float]:
    """Inverse of :func:`rpy_matrix` (away from the pitch = +-pi/2 singularity)."""

    rotation = np.asarray(matrix)[:3, :3]
    pitch = math.asin(-max(-1.0, min(1.0, rotation[2, 0])))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return roll, pitch, yaw


def parent_joints(urdf: ET.Element) -> Mapping[str, ET.Element]:
    """Child link name to the joint that attaches it."""

    return {item.find('child').get('link'): item for item in urdf.findall('joint')}


def chain_to(urdf: ET.Element, child: str, root: str = ROOT_FRAME) -> list[ET.Element]:
    """Joints from ``root`` down to ``child``, root first."""

    parents = parent_joints(urdf)
    chain = []
    link = child
    while link != root:
        if link not in parents:
            raise LookupError(f'chain from {child} ended at {link}, not {root}')
        item = parents[link]
        chain.append(item)
        link = item.find('parent').get('link')
    return chain[::-1]


def transform_to(urdf: ET.Element, child: str, root: str = ROOT_FRAME) -> np.ndarray:
    """4x4 pose of ``child`` in ``root``, composing every joint's full rotation.

    Every joint on the chain is evaluated at its zero position, which for the
    fixed joints between ``base_link`` and the sensors is the only position.
    """

    matrix = np.eye(4)
    for item in chain_to(urdf, child, root):
        matrix = matrix @ pose_matrix(*origin(item))
    return matrix


def translation_to(urdf: ET.Element, child: str, root: str = ROOT_FRAME) -> tuple[float, ...]:
    """Position of ``child`` in ``root``."""

    return tuple(float(value) for value in transform_to(urdf, child, root)[:3, 3])


def gazebo_sensor(urdf: ET.Element, sensor_type: str, reference: str | None = None) -> ET.Element:
    """The single ``<gazebo><sensor type=...>`` block (optionally by reference frame)."""

    sensors = [
        sensor
        for block in urdf.findall('gazebo')
        if reference is None or block.get('reference') == reference
        for sensor in block.findall('sensor')
        if sensor.get('type') == sensor_type
    ]
    if len(sensors) != 1:
        raise LookupError(
            f'expected exactly one {sensor_type} sensor'
            f'{"" if reference is None else f" on {reference}"}, found {len(sensors)}')
    return sensors[0]
