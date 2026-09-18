"""Camera geometry: the selected model owns the frames, in exactly one place.

Simulation has no RealSense driver, so the description supplies the nominal
internal camera frames and the renderer sits on the colour frame those products
are labelled with. Hardware leaves that whole chain to the driver, which reads
the factory calibration off the device.
"""

import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from ridgeback_common.camera_profiles import (
    DEFAULT_CAMERA_PROFILE,
    resolve_camera_profile,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
ROBOT_YAML = REPO_ROOT / 'clearpath' / 'robot.yaml'
LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'

CAMERA = 'camera_0'
COLOR_FRAME = f'{CAMERA}_color_frame'
COLOR_OPTICAL_FRAME = f'{CAMERA}_color_optical_frame'

# d455.urdf.xacro's nominal depth-to-colour offset. Nothing else in the repo may
# carry a second copy of this number.
D455_DEPTH_TO_COLOR_Y = -0.059
# The mount is a bracket pose, independent of which camera hangs off it.
MOUNT_XYZ = (0.2692, 0.0, 0.725)

TOLERANCE = 1e-9


def _expand(
    setup_path: Path,
    *,
    is_sim: bool,
    profile_name: str = DEFAULT_CAMERA_PROFILE,
) -> ET.Element:
    xacro = pytest.importorskip('xacro')
    profile = resolve_camera_profile(profile_name)
    document = xacro.process_file(
        str(setup_path / 'robot.urdf.xacro'),
        mappings={
            'is_sim': 'true' if is_sim else 'false',
            'namespace': 'r100_0001',
            'gazebo_controllers': '/dev/null',
            'sim_camera_width': str(profile.width),
            'sim_camera_height': str(profile.height),
            'sim_camera_horizontal_fov': str(profile.horizontal_fov_rad),
        },
    )
    return ET.fromstring(document.toxml())


@pytest.fixture(scope='module')
def setup_path(tmp_path_factory) -> Path:
    """A generated setup directory of our own, never the user's."""
    generator_module = pytest.importorskip('clearpath_generator_common.description.generator')

    path = tmp_path_factory.mktemp('clearpath_setup')
    shutil.copyfile(ROBOT_YAML, path / 'robot.yaml')
    generator_module.DescriptionGenerator(str(path)).generate()
    return path


@pytest.fixture(scope='module')
def sim_urdf(setup_path: Path) -> ET.Element:
    return _expand(setup_path, is_sim=True)


@pytest.fixture(scope='module')
def hardware_urdf(setup_path: Path) -> ET.Element:
    return _expand(setup_path, is_sim=False)


@pytest.fixture(scope='module')
def gazebo_sdf(setup_path: Path) -> ET.Element:
    """Gazebo's fixed-joint-reduced view of the generated sim description."""
    if shutil.which('gz') is None:
        pytest.skip('gz is required to convert the description to SDF')

    urdf_path = setup_path / 'robot_sim.urdf'
    xacro_module = pytest.importorskip('xacro')
    profile = resolve_camera_profile(DEFAULT_CAMERA_PROFILE)
    urdf_path.write_text(
        xacro_module.process_file(
            str(setup_path / 'robot.urdf.xacro'),
            mappings={
                'is_sim': 'true',
                'namespace': 'r100_0001',
                'gazebo_controllers': '/dev/null',
                'sim_camera_width': str(profile.width),
                'sim_camera_height': str(profile.height),
                'sim_camera_horizontal_fov': str(profile.horizontal_fov_rad),
            },
        ).toxml(),
        encoding='utf-8',
    )
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


def _joint(urdf: ET.Element, name: str) -> ET.Element:
    joints = [joint for joint in urdf.findall('joint') if joint.get('name') == name]
    assert len(joints) == 1, f'expected exactly one joint named {name}, found {len(joints)}'
    return joints[0]


def _origin(joint: ET.Element) -> tuple[tuple[float, ...], tuple[float, ...]]:
    origin = joint.find('origin')
    xyz = tuple(float(value) for value in (origin.get('xyz') or '0 0 0').split())
    rpy = tuple(float(value) for value in (origin.get('rpy') or '0 0 0').split())
    return xyz, rpy


def _translation_to(urdf: ET.Element, child: str) -> tuple[float, float, float]:
    """Sum the chain up to ``child``. Every joint involved is fixed and unrotated
    except the optical hop, so a plain sum is the whole transform."""
    parents = {
        joint.find('child').get('link'): joint
        for joint in urdf.findall('joint')
    }
    total = [0.0, 0.0, 0.0]
    link = child
    while link in parents:
        joint = parents[link]
        xyz, rpy = _origin(joint)
        assert all(abs(angle) < TOLERANCE for angle in rpy), (
            f'joint {joint.get("name")} rotates; the plain sum below is invalid'
        )
        total = [component + offset for component, offset in zip(total, xyz)]
        link = joint.find('parent').get('link')
    assert link == 'base_link', f'chain ended at {link}, not base_link'
    return tuple(total)


def test_simulation_expansion_supplies_the_models_nominal_camera_frames(
    sim_urdf: ET.Element,
) -> None:
    links = {link.get('name') for link in sim_urdf.findall('link')}
    assert COLOR_FRAME in links
    assert COLOR_OPTICAL_FRAME in links

    colour = _joint(sim_urdf, f'{CAMERA}_color_joint')
    assert colour.find('parent').get('link') == f'{CAMERA}_link'
    xyz, rpy = _origin(colour)
    # The D455's own offset, not the D435's +0.015 the retired static publisher
    # used to hardcode.
    assert xyz == pytest.approx((0.0, D455_DEPTH_TO_COLOR_Y, 0.0), abs=TOLERANCE)
    assert rpy == pytest.approx((0.0, 0.0, 0.0), abs=TOLERANCE)

    optical = _joint(sim_urdf, f'{CAMERA}_color_optical_joint')
    assert optical.find('parent').get('link') == COLOR_FRAME
    xyz, rpy = _origin(optical)
    assert xyz == pytest.approx((0.0, 0.0, 0.0), abs=TOLERANCE)
    assert rpy == pytest.approx((-math.pi / 2, 0.0, -math.pi / 2), abs=1e-12)


def test_hardware_expansion_leaves_internal_camera_frames_to_the_driver(
    hardware_urdf: ET.Element,
) -> None:
    links = {link.get('name') for link in hardware_urdf.findall('link')}
    # Mount-to-camera attachment stays URDF-owned either way.
    assert f'{CAMERA}_link' in links
    assert f'{CAMERA}_bottom_screw_frame' in links
    # Everything past the camera body is the driver's calibrated data.
    assert not any(
        name.endswith(('_color_frame', '_color_optical_frame', '_depth_optical_frame'))
        for name in links
    )
    assert not any(
        joint.get('name', '').startswith(f'{CAMERA}_color')
        for joint in hardware_urdf.findall('joint')
    )


def test_only_simulation_carries_the_rendered_camera(
    sim_urdf: ET.Element,
    hardware_urdf: ET.Element,
) -> None:
    def camera_sensors(urdf: ET.Element) -> list[ET.Element]:
        return [
            block
            for block in urdf.findall('gazebo')
            for sensor in block.findall('sensor')
            if sensor.get('type') == 'rgbd_camera'
        ]

    assert camera_sensors(hardware_urdf) == []

    blocks = camera_sensors(sim_urdf)
    assert len(blocks) == 1
    # Referencing the colour frame is what keeps the rendered viewpoint and the
    # frame its images are labelled with from drifting apart.
    assert blocks[0].get('reference') == COLOR_FRAME

    sensor = blocks[0].find('sensor')
    assert sensor.find('camera/optical_frame_id').text == COLOR_OPTICAL_FRAME
    assert sensor.find('camera/gz_frame_id').text == COLOR_FRAME
    # Simplified optics are deliberate: one nominal RGB pinhole, no stereo
    # baseline or noise. The default is the shared low-resolution profile.
    profile = resolve_camera_profile(DEFAULT_CAMERA_PROFILE)
    assert float(sensor.find('camera/horizontal_fov').text) == pytest.approx(
        profile.horizontal_fov_rad)
    assert int(sensor.find('camera/image/width').text) == profile.width
    assert int(sensor.find('camera/image/height').text) == profile.height
    assert sensor.find('update_rate').text == '30'


@pytest.mark.parametrize('profile_name', ['640x480', '1280x720'])
def test_simulation_profile_controls_gazebo_optics(
    setup_path: Path,
    profile_name: str,
) -> None:
    urdf = _expand(setup_path, is_sim=True, profile_name=profile_name)
    profile = resolve_camera_profile(profile_name)
    sensor = next(
        sensor
        for block in urdf.findall('gazebo')
        for sensor in block.findall('sensor')
        if sensor.get('type') == 'rgbd_camera'
    )

    assert int(sensor.find('camera/image/width').text) == profile.width
    assert int(sensor.find('camera/image/height').text) == profile.height
    assert float(sensor.find('camera/horizontal_fov').text) == pytest.approx(
        profile.horizontal_fov_rad)


def test_colour_frame_sits_at_the_configured_mount_plus_the_models_offsets(
    sim_urdf: ET.Element,
) -> None:
    camera_link = _translation_to(sim_urdf, f'{CAMERA}_link')
    colour = _translation_to(sim_urdf, COLOR_FRAME)

    mount = _origin(_joint(sim_urdf, f'{CAMERA}_joint'))[0]
    assert mount == pytest.approx(MOUNT_XYZ, abs=TOLERANCE)

    offset = tuple(c - link for c, link in zip(colour, camera_link))
    assert offset == pytest.approx((0.0, D455_DEPTH_TO_COLOR_Y, 0.0), abs=TOLERANCE)


def test_simulation_renders_from_the_colour_frame_pose(
    sim_urdf: ET.Element,
    gazebo_sdf: ET.Element,
) -> None:
    """Fixed-joint reduction moves the sensor onto ``base_link``; its reduced
    pose must still land on the colour frame."""
    sensors = [
        sensor
        for sensor in gazebo_sdf.iter('sensor')
        if sensor.get('type') == 'rgbd_camera'
    ]
    assert len(sensors) == 1
    pose = tuple(float(value) for value in sensors[0].find('pose').text.split())

    expected = _translation_to(sim_urdf, COLOR_FRAME)
    assert pose[:3] == pytest.approx(expected, abs=1e-9)
    # The render is unrotated relative to the colour frame; the optical rotation
    # lives in the frame chain, not in the sensor pose.
    assert pose[3:] == pytest.approx((0.0, 0.0, 0.0), abs=TOLERANCE)


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

        sensors = [
            sensor for sensor in gazebo_sdf.iter('sensor')
            if sensor.get('name') == name
        ]
        assert len(sensors) == 1
        pose = tuple(float(value) for value in sensors[0].find('pose').text.split())
        assert pose[:3] == pytest.approx(xyz, abs=TOLERANCE)
        assert pose[3:] == pytest.approx((0.0, 0.0, yaw), abs=TOLERANCE)
        assert sensors[0].find('update_rate').text == '40'

        horizontal = sensors[0].find('lidar/scan/horizontal')
        assert horizontal.find('samples').text == '540'
        assert float(horizontal.find('min_angle').text) == pytest.approx(
            -3 * math.pi / 4, abs=1e-8
        )
        assert float(horizontal.find('max_angle').text) == pytest.approx(
            3 * math.pi / 4, abs=1e-8
        )

    # This proves description compatibility. The 2026-09-11 live stationary
    # and moving scan gate additionally proved zero self-returns below 0.8 m;
    # collision_monitor remains a separate downstream integration gate.


def test_no_launch_file_publishes_a_competing_camera_transform() -> None:
    """The description is the only publisher of the internal camera chain."""
    for path in sorted(LAUNCH_DIR.rglob('*.launch.py')):
        text = path.read_text(encoding='utf-8')
        if 'static_transform_publisher' not in text:
            continue
        assert COLOR_OPTICAL_FRAME not in text, (
            f'{path.name} still publishes {COLOR_OPTICAL_FRAME}; '
            'robot_state_publisher owns it in simulation and the driver on hardware'
        )
