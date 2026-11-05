"""2D LiDAR description: Gazebo simulates the same UST-10LX scan as Isaac.

The patched Clearpath generator gives every ``hokuyo_ust`` the UST-10LX scan
geometry, range and Gaussian range noise, and the macro counts both end angles
as rays. Gazebo, the Isaac sensor spec and the shared contract must agree, and
the hardware description's frames must stay untouched.
"""

import json
import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from ridgeback_autonomy import robot_description
from ridgeback_common.camera_profiles import DEFAULT_CAMERA_PROFILE
from ridgeback_common.lidar_contract import (
    GAZEBO_RANGE_NOISE_STDDEV,
    RAW_ANGLE_INCREMENT,
    RAW_ANGLE_MIN,
    RAW_RANGE_MAX,
    RAW_RANGE_MIN,
    RAW_SCAN_BINS,
    RAW_SCAN_PERIOD,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
ROBOT_YAML = REPO_ROOT / 'clearpath' / 'robot.yaml'
ISAAC_LIDAR_SPEC = (
    REPO_ROOT / 'src' / 'ridgeback_autonomy_isaac' / 'sim' / 'isaac' / 'ust10lx_2d.json')

LIDARS = ('lidar2d_0', 'lidar2d_1')
TOLERANCE = 1e-9
# robot.yaml spells the 270-degree window as +-2.35619449 rad.
ANGLE_TOLERANCE = 1e-8


def _expand(setup_path: Path, *, is_sim: bool) -> str:
    pytest.importorskip('xacro')
    return robot_description.expand_description_xml(
        setup_path, is_sim=is_sim, profile_name=DEFAULT_CAMERA_PROFILE)


@pytest.fixture(scope='module')
def setup_path(tmp_path_factory) -> Path:
    """A generated setup directory of our own, never the user's."""
    pytest.importorskip('clearpath_generator_common.description.generator')
    return robot_description.generate_setup(
        ROBOT_YAML, tmp_path_factory.mktemp('clearpath_setup'))


@pytest.fixture(scope='module')
def sim_urdf(setup_path: Path) -> ET.Element:
    return ET.fromstring(_expand(setup_path, is_sim=True))


@pytest.fixture(scope='module')
def hardware_urdf(setup_path: Path) -> ET.Element:
    return ET.fromstring(_expand(setup_path, is_sim=False))


@pytest.fixture(scope='module')
def gazebo_sdf(setup_path: Path) -> ET.Element:
    """Gazebo's fixed-joint-reduced view of the generated sim description."""
    if shutil.which('gz') is None:
        pytest.skip('gz is required to convert the description to SDF')

    urdf_path = setup_path / 'robot_sim.urdf'
    urdf_path.write_text(_expand(setup_path, is_sim=True), encoding='utf-8')
    converted = subprocess.run(
        ['gz', 'sdf', '-p', str(urdf_path)],
        capture_output=True,
        text=True,
        check=True,
    )
    # gz emits `gz:`-prefixed attributes without declaring the prefix.
    return ET.fromstring(
        converted.stdout.replace(
            '<sdf ', "<sdf xmlns:gz='http://gazebosim.org/schema' ", 1
        )
    )


def _sensor(gazebo_sdf: ET.Element, name: str) -> ET.Element:
    sensors = [sensor for sensor in gazebo_sdf.iter('sensor') if sensor.get('name') == name]
    assert len(sensors) == 1, f'expected exactly one sensor named {name}, found {len(sensors)}'
    return sensors[0]


_joint = robot_description.joint
_origin = robot_description.origin


@pytest.mark.parametrize('name', LIDARS)
def test_gazebo_lidar_emits_the_raw_scan_contract(gazebo_sdf: ET.Element, name: str) -> None:
    sensor = _sensor(gazebo_sdf, name)
    assert sensor.get('type') == 'gpu_lidar'
    assert float(sensor.find('update_rate').text) == pytest.approx(1.0 / RAW_SCAN_PERIOD)

    horizontal = sensor.find('lidar/scan/horizontal')
    samples = int(horizontal.find('samples').text)
    angle_min = float(horizontal.find('min_angle').text)
    angle_max = float(horizontal.find('max_angle').text)
    assert samples == RAW_SCAN_BINS
    assert angle_min == pytest.approx(RAW_ANGLE_MIN, abs=ANGLE_TOLERANCE)
    assert angle_max == pytest.approx(
        RAW_ANGLE_MIN + (RAW_SCAN_BINS - 1) * RAW_ANGLE_INCREMENT, abs=ANGLE_TOLERANCE)
    # Gazebo spaces its rays end to end; both ends are rays.
    assert (angle_max - angle_min) / (samples - 1) == pytest.approx(
        RAW_ANGLE_INCREMENT, abs=TOLERANCE)

    scan_range = sensor.find('lidar/range')
    assert float(scan_range.find('min').text) == pytest.approx(RAW_RANGE_MIN)
    assert float(scan_range.find('max').text) == pytest.approx(RAW_RANGE_MAX)


@pytest.mark.parametrize('name', LIDARS)
def test_gazebo_lidar_applies_the_chosen_gaussian_range_noise(
    gazebo_sdf: ET.Element,
    name: str,
) -> None:
    noise = _sensor(gazebo_sdf, name).find('lidar/noise')
    assert noise is not None
    assert noise.find('type').text == 'gaussian'
    assert float(noise.find('mean').text) == 0.0
    assert float(noise.find('stddev').text) == pytest.approx(GAZEBO_RANGE_NOISE_STDDEV)


def test_isaac_sensor_spec_matches_the_raw_scan_contract(gazebo_sdf: ET.Element) -> None:
    """Gazebo, the Isaac spec and the shared contract cannot drift apart."""
    attributes = json.loads(ISAAC_LIDAR_SPEC.read_text(encoding='utf-8'))['attributes']

    def core(key: str):
        return attributes[f'omni:sensor:Core:{key}']

    assert core('nearRangeM') == pytest.approx(RAW_RANGE_MIN)
    assert core('farRangeM') == pytest.approx(RAW_RANGE_MAX)
    assert core('scanRateBaseHz') == pytest.approx(1.0 / RAW_SCAN_PERIOD)
    assert attributes['omni:sensor:tickRate'] == pytest.approx(1.0 / RAW_SCAN_PERIOD)
    # One rotary emitter: firings per revolution set the angular step.
    step = 2.0 * math.pi * core('scanRateBaseHz') / core('patternFiringRateHz')
    assert step == pytest.approx(RAW_ANGLE_INCREMENT, abs=TOLERANCE)

    for name in LIDARS:
        resolution = _sensor(gazebo_sdf, name).find('lidar/range/resolution')
        assert float(resolution.text) == pytest.approx(core('rangeResolutionM'))


def test_only_the_hokuyo_ust_gets_the_ust10lx_simulation() -> None:
    sensors = pytest.importorskip('clearpath_generator_common.description.sensors')
    lidars = pytest.importorskip('clearpath_config.sensors.types.lidars_2d')
    description = sensors.SensorDescription

    assert description.MODEL[lidars.HokuyoUST.SENSOR_MODEL] is description.HokuyoUSTDescription
    assert description.MODEL[lidars.SickLMS1XX.SENSOR_MODEL] is description.Lidar2dDescription


def test_hardware_description_keeps_the_lidar_frames(hardware_urdf: ET.Element) -> None:
    """Scan geometry and noise live only in the ``<gazebo>`` block, which
    hardware ignores; the frames the driver publishes into must not move."""
    expected_mounts = {
        'lidar2d_0': ((0.3922, 0.0, 0.179), (0.0, 0.0, 0.0)),
        'lidar2d_1': ((-0.3922, 0.0, 0.179), (0.0, 0.0, 3.14159)),
    }
    links = {link.get('name') for link in hardware_urdf.findall('link')}
    for name, (xyz, rpy) in expected_mounts.items():
        assert {f'{name}_link', f'{name}_laser'} <= links

        mount = _joint(hardware_urdf, f'{name}_joint')
        assert mount.get('type') == 'fixed'
        assert mount.find('parent').get('link') == 'chassis_link'
        assert mount.find('child').get('link') == f'{name}_link'
        assert _origin(mount) == (pytest.approx(xyz, abs=TOLERANCE),
                                  pytest.approx(rpy, abs=TOLERANCE))

        laser = _joint(hardware_urdf, f'{name}_laser_joint')
        assert laser.get('type') == 'fixed'
        assert laser.find('parent').get('link') == f'{name}_link'
        assert laser.find('child').get('link') == f'{name}_laser'
        assert _origin(laser) == (pytest.approx((0.0, 0.0, 0.0474), abs=TOLERANCE),
                                  pytest.approx((0.0, 0.0, 0.0), abs=TOLERANCE))


def test_accurate_lidar_mounts_survive_gazebo_fixed_joint_reduction(
    sim_urdf: ET.Element,
    gazebo_sdf: ET.Element,
) -> None:
    expected = {
        'lidar2d_0': ((0.3922, 0.0, 0.2264), 0.0),
        'lidar2d_1': ((-0.3922, 0.0, 0.2264), 3.14159),
    }

    for name, (xyz, yaw) in expected.items():
        mount = _joint(sim_urdf, f'{name}_joint')
        assert mount.find('parent').get('link') == 'chassis_link'

        pose = tuple(float(value) for value in _sensor(gazebo_sdf, name).find('pose').text.split())
        assert pose[:3] == pytest.approx(xyz, abs=TOLERANCE)
        assert pose[3:] == pytest.approx((0.0, 0.0, yaw), abs=TOLERANCE)

    # This proves description compatibility. The 2026-09-11 live stationary
    # and moving scan gate additionally proved zero self-returns below 0.8 m;
    # collision_monitor remains a separate downstream integration gate.
