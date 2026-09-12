"""Generator-level camera contract tests; Isaac/pxr is not required."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
IMPORTER = REPO_ROOT / 'tools' / 'isaac' / 'import_ridgeback_urdf.py'
ROBOT_USDA = (
    REPO_ROOT / 'src/ridgeback_autonomy/sim/isaac/usd/robots/'
    'ridgeback_r100/ridgeback_r100.usda'
)
ISAAC_LAUNCH = (
    REPO_ROOT / 'src/ridgeback_autonomy/launch/includes/simulation_isaac.launch.py'
)

module_spec = importlib.util.spec_from_file_location('import_ridgeback_urdf', IMPORTER)
importer = importlib.util.module_from_spec(module_spec)
sys.modules['import_ridgeback_urdf'] = importer
module_spec.loader.exec_module(importer)


def test_isaac_camera_spec_pins_d455_render_contract() -> None:
    spec = importer.load_camera_spec()
    resolution = spec['resolution']
    intrinsics = spec['intrinsics_px']

    assert resolution == {'width': 1280, 'height': 720}
    assert intrinsics == {'fx': 631.0, 'fy': 631.0, 'cx': 640.0, 'cy': 360.0}
    assert spec['tick_rate_hz'] == 30.0
    assert spec['clipping_range_m'] == [0.1, 100.0]

    horizontal_fov = math.degrees(
        2 * math.atan(resolution['width'] / (2 * intrinsics['fx']))
    )
    vertical_fov = math.degrees(
        2 * math.atan(resolution['height'] / (2 * intrinsics['fy']))
    )
    assert horizontal_fov == pytest.approx(90.811, abs=0.001)
    assert vertical_fov == pytest.approx(59.412, abs=0.001)


def test_committed_usd_camera_uses_generated_d455_frames_and_rate() -> None:
    text = ROBOT_USDA.read_text(encoding='utf-8')

    assert 'over "camera_0_color_frame"' in text
    assert 'def Camera "d455_color"' in text
    assert 'prepend apiSchemas = ["OmniSensorAPI"]' in text
    assert 'float omni:sensor:tickRate = 30' in text
    assert 'custom int dynamo:resolutionWidth = 1280' in text
    assert 'custom int dynamo:resolutionHeight = 720' in text
    assert 'double3 xformOp:translate = (0, 0.015, 0)' not in text


def test_isaac_robot_state_publisher_expands_the_simulation_frames() -> None:
    text = ISAAC_LAUNCH.read_text(encoding='utf-8')

    assert 'robot.urdf.xacro is_sim:=true' in text
    assert 'camera_optical_tf' not in text
