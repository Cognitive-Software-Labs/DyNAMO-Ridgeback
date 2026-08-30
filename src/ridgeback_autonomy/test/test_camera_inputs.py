from __future__ import annotations

import dataclasses

import pytest

from ridgeback_autonomy.common.camera_inputs import (
    REALSENSE_BACKEND,
    SIMULATION_BACKEND,
    resolve_camera_inputs,
)


def test_simulation_mapping_matches_existing_topics() -> None:
    inputs = resolve_camera_inputs(SIMULATION_BACKEND)

    assert dataclasses.is_dataclass(inputs)
    assert inputs.color_image_topic == 'sensors/camera_0/color/image'
    assert inputs.color_camera_info_topic == 'sensors/camera_0/color/camera_info'
    assert inputs.aligned_depth_topic == 'sensors/camera_0/depth/image'
    assert inputs.organized_points_topic == 'sensors/camera_0/points'
    assert inputs.backend == SIMULATION_BACKEND


def test_realsense_mapping_uses_driver_owned_alignment() -> None:
    inputs = resolve_camera_inputs(REALSENSE_BACKEND)

    assert inputs.color_image_topic == 'sensors/camera_0/color/image_raw'
    assert inputs.color_camera_info_topic == 'sensors/camera_0/color/camera_info'
    assert inputs.aligned_depth_topic == 'sensors/camera_0/aligned_depth_to_color/image_raw'
    assert inputs.organized_points_topic is None


def test_unknown_backend_is_rejected() -> None:
    with pytest.raises(ValueError, match='unknown camera backend'):
        resolve_camera_inputs('thermal')


@pytest.mark.parametrize('override', [
    {'color_topic': ''}, {'camera_info_topic': '  '}, {'depth_topic': None},
])
def test_required_topics_reject_empty_values(override) -> None:
    with pytest.raises(ValueError, match='required|must not be empty'):
        resolve_camera_inputs(SIMULATION_BACKEND, override)


def test_optional_pointcloud_can_be_disabled_or_overridden() -> None:
    disabled = resolve_camera_inputs(SIMULATION_BACKEND, {'pointcloud_topic': ''})
    overridden = resolve_camera_inputs(
        REALSENSE_BACKEND, {'pointcloud_topic': 'sensors/camera_0/depth/color/points'})

    assert disabled.organized_points_topic is None
    assert overridden.organized_points_topic == 'sensors/camera_0/depth/color/points'


def test_compatibility_overrides_win_after_backend_defaults() -> None:
    inputs = resolve_camera_inputs(SIMULATION_BACKEND, {
        'color_topic': 'custom/color',
        'camera_info_topic': 'custom/info',
        'depth_topic': 'custom/depth',
        'pointcloud_topic': 'custom/points',
    })

    assert inputs.color_image_topic == 'custom/color'
    assert inputs.color_camera_info_topic == 'custom/info'
    assert inputs.aligned_depth_topic == 'custom/depth'
    assert inputs.organized_points_topic == 'custom/points'


def test_topics_must_remain_relative_to_the_robot_namespace() -> None:
    with pytest.raises(ValueError, match='relative to the robot namespace'):
        resolve_camera_inputs(SIMULATION_BACKEND, {'color_topic': '/camera/color'})
