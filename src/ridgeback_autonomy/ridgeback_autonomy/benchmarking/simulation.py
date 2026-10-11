"""Simulator-neutral pose normalization and benchmark ground-truth helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ridgeback_localization.core.vehicle_frame import (
    planar_measurement_from_vehicle_front,
    yaw_from_quaternion,
)


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
    """Normalize optional pose components to concrete floats."""

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
