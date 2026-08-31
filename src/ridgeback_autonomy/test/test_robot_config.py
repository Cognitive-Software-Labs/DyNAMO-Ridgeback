from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
ROBOT_YAML = REPO_ROOT / 'clearpath' / 'robot.yaml'

# The physical robot carries an Intel RealSense D455. `device_type` is a
# device-name filter for the driver and the model selector for the generated
# description, so it is not a generic D400-family token.
EXPECTED_DEVICE_TYPE = 'd455'
EXPECTED_MOUNT_XYZ = [0.3, 0.0, 0.85]
EXPECTED_MOUNT_RPY = [0.0, 0.0, 0.0]


def _camera_source() -> dict:
    config = yaml.safe_load(ROBOT_YAML.read_text(encoding='utf-8'))
    return config['sensors']['camera'][0]


def _parsed_camera():
    """The camera as the checked-out Clearpath parser actually builds it."""
    clearpath_config = pytest.importorskip('clearpath_config.clearpath_config')
    yaml_utils = pytest.importorskip('clearpath_config.common.utils.yaml')
    cameras = pytest.importorskip('clearpath_config.sensors.types.cameras')

    config = clearpath_config.ClearpathConfig(yaml_utils.read_yaml(str(ROBOT_YAML)))
    parsed = [
        sensor
        for sensor in config.sensors.get_all_sensors()
        if isinstance(sensor, cameras.IntelRealsense)
    ]
    assert len(parsed) == 1
    return parsed[0]


def test_realsense_stream_profiles_are_left_to_clearpath_defaults() -> None:
    parameters = _camera_source()['ros_parameters']['intel_realsense']

    assert parameters['enable_color'] is True
    assert parameters['enable_depth'] is True
    assert parameters['align_depth.enable'] is True
    assert parameters['enable_sync'] is True
    assert not any(key.endswith('profile') for key in parameters)


def test_realsense_device_type_is_declared_explicitly() -> None:
    camera = _camera_source()

    assert camera['model'] == 'intel_realsense'
    assert camera['ros_parameters']['intel_realsense']['device_type'] == EXPECTED_DEVICE_TYPE


def test_camera_mount_pose_is_unchanged() -> None:
    camera = _camera_source()

    assert camera['parent'] == 'default_mount'
    assert camera['xyz'] == EXPECTED_MOUNT_XYZ
    assert camera['rpy'] == EXPECTED_MOUNT_RPY


def test_clearpath_parser_retains_alignment_and_sync_parameters() -> None:
    clearpath_config = pytest.importorskip('clearpath_config.sensors.types.cameras')
    source = _camera_source()['ros_parameters']

    generated = clearpath_config.IntelRealsense(ros_parameters=source).ros_parameters
    parameters = generated['intel_realsense']
    assert parameters['align_depth.enable'] is True
    assert parameters['enable_sync'] is True


def test_clearpath_parser_resolves_the_configured_device_type() -> None:
    camera = _parsed_camera()

    # Both sides matter: the property drives the description model, the emitted
    # ROS parameter drives the driver's device filter.
    assert camera.device_type == EXPECTED_DEVICE_TYPE
    assert camera.ros_parameters['intel_realsense']['device_type'] == EXPECTED_DEVICE_TYPE


def test_parsed_camera_preserves_streams_mount_and_clearpath_profiles() -> None:
    camera = _parsed_camera()
    parameters = camera.ros_parameters['intel_realsense']

    assert camera.parent == 'default_mount'
    assert camera.xyz == EXPECTED_MOUNT_XYZ
    assert camera.rpy == EXPECTED_MOUNT_RPY
    assert parameters['camera_name'] == 'camera_0'
    assert parameters['enable_color'] is True
    assert parameters['enable_depth'] is True
    assert parameters['align_depth.enable'] is True
    assert parameters['enable_sync'] is True
    # Unspecified profiles still resolve to Clearpath's current 640x480 @ 30
    # defaults rather than to a model-specific higher-resolution mode.
    assert parameters['rgb_camera.color_profile'] == '640,480,30'
    assert parameters['depth_module.depth_profile'] == '640,480,30'


def test_description_generator_selects_the_configured_model() -> None:
    sensors = pytest.importorskip('clearpath_generator_common.description.sensors')
    camera = _parsed_camera()

    parameters = sensors.SensorDescription(camera).parameters
    assert parameters['model'] == EXPECTED_DEVICE_TYPE
    assert parameters['name'] == 'camera_0'
    assert parameters['parent_link'] == 'default_mount'
    assert parameters['image_width'] == 640
    assert parameters['image_height'] == 480
    assert parameters['update_rate'] == 30
