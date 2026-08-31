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
MOUNT_XYZ = (0.3, 0.0, 0.85)

TOLERANCE = 1e-9


def _expand(setup_path: Path, *, is_sim: bool) -> ET.Element:
    xacro = pytest.importorskip('xacro')
    document = xacro.process_file(
        str(setup_path / 'robot.urdf.xacro'),
        mappings={
            'is_sim': 'true' if is_sim else 'false',
            'namespace': 'r100_0001',
            'gazebo_controllers': '/dev/null',
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
    # Simplified optics are deliberate: one FoV, no stereo baseline, no noise.
    assert sensor.find('camera/horizontal_fov').text == '1.25'
    assert sensor.find('camera/image/width').text == '640'
    assert sensor.find('camera/image/height').text == '480'
    assert sensor.find('update_rate').text == '30'


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
    setup_path: Path,
    sim_urdf: ET.Element,
) -> None:
    """Fixed-joint reduction moves the sensor onto ``base_link``; its reduced
    pose must still land on the colour frame."""
    if shutil.which('gz') is None:
        pytest.skip('gz is required to convert the description to SDF')

    urdf_path = setup_path / 'robot_sim.urdf'
    xacro_module = pytest.importorskip('xacro')
    urdf_path.write_text(
        xacro_module.process_file(
            str(setup_path / 'robot.urdf.xacro'),
            mappings={
                'is_sim': 'true',
                'namespace': 'r100_0001',
                'gazebo_controllers': '/dev/null',
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

    # gz emits `gz:`-prefixed attributes without ever declaring the prefix, so
    # a plain parse trips over them long before it reaches the sensor.
    sdf = ET.fromstring(
        converted.stdout.replace(
            '<sdf ', "<sdf xmlns:gz='http://gazebosim.org/schema' ", 1
        )
    )
    sensors = [
        sensor
        for sensor in sdf.iter('sensor')
        if sensor.get('type') == 'rgbd_camera'
    ]
    assert len(sensors) == 1
    pose = tuple(float(value) for value in sensors[0].find('pose').text.split())

    expected = _translation_to(sim_urdf, COLOR_FRAME)
    assert pose[:3] == pytest.approx(expected, abs=1e-9)
    # The render is unrotated relative to the colour frame; the optical rotation
    # lives in the frame chain, not in the sensor pose.
    assert pose[3:] == pytest.approx((0.0, 0.0, 0.0), abs=TOLERANCE)


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
