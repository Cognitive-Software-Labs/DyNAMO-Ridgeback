"""Generator-level camera contract tests; Isaac/pxr is not required."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest
import numpy as np

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

sys.path.insert(0, str(REPO_ROOT / 'src' / 'ridgeback_autonomy'))
sys.path.insert(0, str(REPO_ROOT / 'src' / 'ridgeback_common'))
sys.path.insert(0, str(REPO_ROOT / 'src' / 'ridgeback_localization'))
sensors_spec = importlib.util.spec_from_file_location(
    'ridgeback_isaac_sensors', ISAAC_SENSORS)
sensors = importlib.util.module_from_spec(sensors_spec)
sensors_spec.loader.exec_module(sensors)
from ridgeback_common.camera_profiles import resolve_camera_profile


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


def test_isaac_d455_depth_mode_uses_geometric_left_imager_fallback() -> None:
    text = ISAAC_SENSORS.read_text(encoding='utf-8')

    assert 'D455_BASELINE_MM = 95.0' in text
    assert 'D455_MAX_DISPARITY_PX = 123.0' in text
    assert 'D455_DEPTH_TO_COLOR_X_M = -0.059' in text
    assert '"distance_to_image_plane"' in text
    assert 'from isaacsim.sensors.experimental.rtx' not in text
    assert 'profile.depth_focal_length_px' in text
    assert 'profile.minimum_depth_m' in text
    assert 'np.lexsort((z, target))' in text
    assert 'camera_products = [("cam_rgb", "rgb"' in text
    assert 'if depth_fidelity == "ideal"' in text
    assert 'elif depth_fidelity == "d455"' in text


def test_d455_fallback_quantizes_disparity_and_aligns_to_color() -> None:
    profile = resolve_camera_profile('640x480')
    geometric = np.full((profile.height, profile.width), 3.0, dtype=np.float32)
    aligned, points = sensors._align_d455_depth(
        geometric, profile, np.zeros_like(geometric))

    valid = aligned > 0.0
    # The depth imager's 75-degree FoV is narrower than the nominal color
    # imager, so aligned edge pixels are intentionally empty.
    assert valid.mean() > 0.80
    assert valid[48:-48, 64:-64].mean() > 0.99
    assert np.median(aligned[valid]) == pytest.approx(3.0, rel=0.002)
    assert points.shape == (profile.height, profile.width, 3)
    assert np.allclose(points[..., 2][valid], aligned[valid])
    assert np.isnan(points[~valid]).all()


def test_d455_fallback_noise_is_bounded_and_nonconstant() -> None:
    profile = resolve_camera_profile('640x480')
    geometric = np.full((profile.height, profile.width), 3.0, dtype=np.float32)
    rng = np.random.default_rng(0)
    noise = rng.normal(0.25, 0.25, geometric.shape).astype(np.float32)
    aligned, _ = sensors._align_d455_depth(geometric, profile, noise)

    valid = aligned > 0.0
    residual = aligned[valid] - 3.0
    assert residual.std() > 0.01
    assert np.percentile(np.abs(residual), 99) < 0.25


def test_d455_alignment_keeps_nearest_surface_on_projection_collision() -> None:
    profile = resolve_camera_profile('640x480')
    geometric = np.zeros((profile.height, profile.width), dtype=np.float32)
    noise = np.zeros_like(geometric)

    # These two left-imager samples reproject onto the same colour pixel.
    # The 1 m sample must win the z-buffer over the 2 m sample.
    row = int(profile.cy)
    geometric[row, 320] = 1.0
    geometric[row, 307] = 2.0
    aligned, points = sensors._align_d455_depth(geometric, profile, noise)

    targets = np.flatnonzero(aligned[row] > 0.0)
    assert targets.tolist() == [297]
    assert aligned[row, 297] == pytest.approx(1.0, rel=0.002)
    assert points[row, 297, 2] == pytest.approx(aligned[row, 297])
