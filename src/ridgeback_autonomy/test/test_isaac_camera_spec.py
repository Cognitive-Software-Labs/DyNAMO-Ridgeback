"""Generator-level camera contract tests; Isaac/pxr is not required."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
IMPORTER = REPO_ROOT / 'tools' / 'isaac' / 'import_ridgeback_urdf.py'
ROBOT_USDA = (
    REPO_ROOT / 'src/ridgeback_autonomy_isaac/sim/isaac/usd/robots/'
    'ridgeback_r100/ridgeback_r100.usda'
)
ISAAC_LAUNCH = (
    REPO_ROOT / 'src/ridgeback_autonomy_isaac/launch/backend.launch.py'
)
ISAAC_SENSORS = (
    REPO_ROOT / 'src/ridgeback_autonomy_isaac/sim/isaac/sensors.py'
)

module_spec = importlib.util.spec_from_file_location('import_ridgeback_urdf', IMPORTER)
importer = importlib.util.module_from_spec(module_spec)
sys.modules['import_ridgeback_urdf'] = importer
module_spec.loader.exec_module(importer)


def test_isaac_default_camera_spec_pins_d455_render_contract() -> None:
    spec = importer.load_camera_spec()
    resolution = spec['resolution']
    intrinsics = spec['intrinsics_px']

    assert spec['profile'] == '640x480'
    assert resolution == {'width': 640, 'height': 480}
    assert intrinsics == {'fx': 384.0, 'fy': 384.0, 'cx': 320.0, 'cy': 240.0}
    assert spec['tick_rate_hz'] == 30.0
    assert spec['clipping_range_m'] == [0.1, 100.0]

    horizontal_fov = math.degrees(
        2 * math.atan(resolution['width'] / (2 * intrinsics['fx']))
    )
    vertical_fov = math.degrees(
        2 * math.atan(resolution['height'] / (2 * intrinsics['fy']))
    )
    assert horizontal_fov == pytest.approx(79.611, abs=0.001)
    assert vertical_fov == pytest.approx(64.011, abs=0.001)


def test_isaac_hd_camera_spec_uses_the_same_nominal_lens() -> None:
    spec = importer.load_camera_spec('1280x720')

    assert spec['resolution'] == {'width': 1280, 'height': 720}
    assert spec['intrinsics_px'] == {
        'fx': 640.0, 'fy': 640.0, 'cx': 640.0, 'cy': 360.0}


def test_committed_usd_camera_uses_generated_d455_frames_and_rate() -> None:
    text = ROBOT_USDA.read_text(encoding='utf-8')

    assert 'over "camera_0_color_frame"' in text
    assert 'def Camera "d455_color"' in text
    assert 'prepend apiSchemas = ["OmniSensorAPI"]' in text
    assert 'float omni:sensor:tickRate = 30' in text
    assert 'custom int dynamo:resolutionWidth = 640' in text
    assert 'custom int dynamo:resolutionHeight = 480' in text
    assert 'double3 xformOp:translate = (0, 0.015, 0)' not in text


def test_isaac_robot_state_publisher_expands_the_simulation_frames() -> None:
    text = ISAAC_LAUNCH.read_text(encoding='utf-8')

    assert 'robot.urdf.xacro is_sim:=true' in text
    assert 'camera_optical_tf' not in text


def test_isaac_runtime_updates_camera_metadata_and_aperture_as_one_contract() -> None:
    text = ISAAC_SENSORS.read_text(encoding='utf-8')

    assert 'width_attr.Set(width)' in text
    assert 'height_attr.Set(height)' in text
    assert 'tick_attr.Set(tick_rate)' in text
    assert 'width * focal / profile.focal_length_px' in text
    assert 'height * focal / profile.focal_length_px' in text


def test_isaac_d455_depth_mode_uses_native_processed_depth_aov() -> None:
    text = ISAAC_SENSORS.read_text(encoding='utf-8')

    assert 'D455_BASELINE_MM = 95.0' in text
    assert 'D455_MAX_DISPARITY_PX = 123.0' in text
    assert '"DepthSensorDistance"' in text
    assert 'SingleViewDepthCameraSensor' in text
    assert 'profile.depth_focal_length_px' in text
    assert 'profile.minimum_depth_m' in text
    assert 'camera_products = [("cam_rgb", "rgb"' in text
    assert 'if depth_fidelity == "ideal"' in text
    assert 'elif depth_fidelity == "d455"' in text
