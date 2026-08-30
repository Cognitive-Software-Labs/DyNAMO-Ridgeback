"""Pure Gazebo command, pose, and benchmark ground-truth helpers."""

from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Any

from ridgeback_autonomy.perception.target_localization.core.vehicle_frame import (
    planar_measurement_from_vehicle_front,
    yaw_from_quaternion,
)


TARGET_SPAWN_HEIGHT_M = 0.0


@dataclass(frozen=True)
class GroundTruthInstance:
    """Simulator truth for one spawned target, used only for scoring."""

    index: int
    model_name: str
    world_x: float
    world_y: float
    forward_m: float
    lateral_m: float
    distance_m: float


def normalize_pose(pose: dict[str, Any]) -> dict[str, Any]:
    """Normalize optional Gazebo pose components to concrete floats."""

    position = pose.get('position') or {}
    orientation = pose.get('orientation') or {}
    return {
        **pose,
        'position': {
            'x': float(position.get('x', 0.0)),
            'y': float(position.get('y', 0.0)),
            'z': float(position.get('z', 0.0)),
        },
        'orientation': {
            'x': float(orientation.get('x', 0.0)),
            'y': float(orientation.get('y', 0.0)),
            'z': float(orientation.get('z', 0.0)),
            'w': float(orientation.get('w', 1.0)),
        },
    }


def pose_snapshot_from_payload(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Index one Gazebo pose-info JSON payload by entity name."""

    return {
        pose['name']: normalize_pose(pose)
        for pose in payload.get('pose', [])
        if isinstance(pose, dict) and pose.get('name')
    }


def compute_ground_truth_instances(
    snapshot: dict[str, dict[str, Any]],
    robot_model_name: str,
    target_models: list[tuple[int, str, Any]],
) -> list[GroundTruthInstance]:
    """Compute front-referenced planar truth from one simulator pose snapshot."""

    robot_pose = snapshot.get(robot_model_name)
    if robot_pose is None:
        raise RuntimeError(f'Robot pose "{robot_model_name}" not present')
    orientation = robot_pose['orientation']
    robot_yaw = yaw_from_quaternion(
        orientation['x'], orientation['y'], orientation['z'], orientation['w'])

    instances = []
    for index, model_name, _target in target_models:
        target_pose = snapshot.get(model_name)
        if target_pose is None:
            raise RuntimeError(f'Target pose "{model_name}" not present')
        delta_x = target_pose['position']['x'] - robot_pose['position']['x']
        delta_y = target_pose['position']['y'] - robot_pose['position']['y']
        forward_m, lateral_m, distance_m = planar_measurement_from_vehicle_front(
            delta_x, delta_y, robot_yaw)
        instances.append(GroundTruthInstance(
            index=index,
            model_name=model_name,
            world_x=target_pose['position']['x'],
            world_y=target_pose['position']['y'],
            forward_m=forward_m,
            lateral_m=lateral_m,
            distance_m=distance_m,
        ))
    return instances


def model_sdf_path(models_dir: str, model: str) -> str:
    path = os.path.join(models_dir, model, 'model.sdf')
    if not os.path.isfile(path):
        raise RuntimeError(f'Object model "{model}" has no model.sdf at {path}')
    return path


def spawn_model_command(
    world: str,
    model_ref: str,
    model_name: str,
    x_m: float,
    y_m: float,
    z_m: float,
    yaw_rad: float,
) -> list[str]:
    return [
        '/opt/ros/jazzy/lib/ros_gz_sim/create',
        '-world', world,
        '-file', model_ref,
        '-name', model_name,
        '-x', f'{x_m:.6f}',
        '-y', f'{y_m:.6f}',
        '-z', f'{z_m:.6f}',
        '-Y', f'{yaw_rad:.6f}',
    ]


def despawn_model_command(world: str, model_name: str) -> list[str]:
    return [
        'gz',
        'service',
        '-s', f'/world/{world}/remove/blocking',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000',
        '--req', f'name: "{model_name}" type: MODEL',
    ]


def pose_snapshot_command(pose_info_topic: str) -> list[str]:
    return [
        'gz',
        'topic',
        '-e',
        '-t', pose_info_topic,
        '--json-output',
        '-n', '1',
    ]
