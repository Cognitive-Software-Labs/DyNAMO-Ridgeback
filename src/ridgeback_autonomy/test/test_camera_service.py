"""Exercise the D455 boot-service installer against an isolated host tree."""
from importlib.machinery import SourceFileLoader
import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools/intel_thor/camera_service'
CAMERA_SERIAL = '338522302134'
# The USB descriptor carries a different number than the camera serial.
USB_SERIAL = '346543063318'
VENDOR = ('realsense-camera.service', 'depth-to-mono8.service',
          'ridgeback-camera-mjpeg.service', 'camera-web-ready.service')

FAKE = '''#!/usr/bin/python3
import json, os, pathlib, sys
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
root = pathlib.Path(os.environ['TEST_HOST'])
units_path = root / 'units.json'
units = json.loads(units_path.read_text())
with (root / 'calls.jsonl').open('a') as log:
    log.write(json.dumps([name, *args]) + '\\n')
if name == 'git':
    print('0123456789abcdef0123456789abcdef01234567')
elif name == 'rs-enumerate-devices':
    if units.get('realsense-camera.service', {}).get('active') == 'active':
        sys.exit('device busy')
    if (root / 'no-camera').exists():
        sys.exit('No device detected')
    print('Device Name                   Serial Number       Firmware Version')
    print('Intel RealSense D455          %s        5.17.3.10' % os.environ['TEST_CAMERA_SERIAL'])
elif name == 'systemctl':
    command, target = args[0], args[-1]
    state = units.get(target)
    if command == 'is-enabled':
        if state is None:
            sys.exit(1)
        print(state['enabled'])
        sys.exit(0 if state['enabled'] == 'enabled' else 1)
    if command == 'is-active':
        active = state['active'] if state else 'inactive'
        print(active)
        sys.exit(0 if active == 'active' else 3)
    if command == 'daemon-reload':
        sys.exit(0)
    for unit in [a for a in args[1:] if a.endswith('.service')]:
        state = units.setdefault(unit, {'enabled': 'disabled', 'active': 'inactive'})
        if command == 'disable':
            state['enabled'] = 'disabled'
            if '--now' in args:
                state['active'] = 'inactive'
        elif command == 'enable':
            state['enabled'] = 'enabled'
        elif command in ('start', 'restart'):
            if unit == 'dynamo-camera.service' and (root / 'fail-start').exists():
                sys.exit('start failed')
            state['active'] = 'active'
    units_path.write_text(json.dumps(units))
'''


def load_tool():
    loader = SourceFileLoader('camera_service', str(SCRIPT))
    spec = importlib.util.spec_from_loader('camera_service', loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


@pytest.fixture
def host(tmp_path):
    clearpath = tmp_path / 'etc/clearpath'
    clearpath.mkdir(parents=True)
    (tmp_path / 'etc/systemd/system').mkdir(parents=True)
    robot = {'serial_number': 'r100-0001',
             'system': {'ros2': {'namespace': 'r100_0001'}},
             'sensors': {'lidar2d': [{'model': 'hokuyo_ust', 'parent': 'chassis_link',
                                      'xyz': [-0.3922, 0.0, 0.1856], 'rpy': [0.0, 0.0, 3.14159]}]}}
    (clearpath / 'robot.yaml').write_text(yaml.safe_dump(robot, sort_keys=False))
    (clearpath / 'setup.bash').write_text(
        'export CYCLONEDDS_URI="file:///etc/clearpath/cyclonedds.xml"\n'
        'export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"\n')
    (clearpath / 'cyclonedds.xml').write_text(
        '<CycloneDDS><Domain><General><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>\n')
    device = tmp_path / 'sys/bus/usb/devices/2-5.1'
    device.mkdir(parents=True)
    (device / 'product').write_text('Intel(R) RealSense(TM) Depth Camera 455 \n')
    (device / 'serial').write_text(USB_SERIAL + '\n')
    units = {unit: {'enabled': 'enabled', 'active': 'active'} for unit in VENDOR}
    units['camera-web-ready.service']['active'] = 'failed'
    units['clearpath-robot.service'] = {'enabled': 'enabled', 'active': 'active'}
    (tmp_path / 'units.json').write_text(json.dumps(units))
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    fake = bin_dir / 'fake'
    fake.write_text(FAKE)
    fake.chmod(0o755)
    for name in ('git', 'systemctl', 'rs-enumerate-devices'):
        (bin_dir / name).symlink_to(fake)
    env = dict(os.environ, PATH=f'{bin_dir}:/usr/bin:/bin', TEST_HOST=str(tmp_path),
               TEST_CAMERA_SERIAL=CAMERA_SERIAL)

    def run(*args):
        return subprocess.run(['python3', str(SCRIPT), *args, '--root', str(tmp_path)],
                              env=env, capture_output=True, text=True, timeout=30)

    def units_now():
        return json.loads((tmp_path / 'units.json').read_text())

    def calls():
        return [json.loads(line) for line in (tmp_path / 'calls.jsonl').read_text().splitlines()]

    return tmp_path, run, units_now, calls


def test_apply_installs_the_contract_and_retires_the_vendor_camera(host):
    root, run, units_now, calls = host
    original = (root / 'etc/clearpath/robot.yaml').read_text()

    result = run('apply', '--yes')

    assert result.returncode == 0, result.stdout + result.stderr
    robot = yaml.safe_load((root / 'etc/clearpath/robot.yaml').read_text())
    camera, = robot['sensors']['camera']
    assert camera['urdf_enabled'] is True and camera['launch_enabled'] is False
    assert (camera['parent'], camera['xyz'], camera['rpy']) == (
        'default_mount', [0.2692, 0.0, 0.725], [0.0, 0.0, 0.0])
    assert robot['sensors']['lidar2d'][0]['xyz'] == [-0.3922, 0.0, 0.1856]
    params = yaml.safe_load((root / 'etc/clearpath/dynamo-camera/d455.yaml').read_text())
    parameters = params['/**']['ros__parameters']
    # The librealsense serial, as a string, never the USB descriptor serial.
    assert parameters['serial_no'] == CAMERA_SERIAL
    assert USB_SERIAL not in (root / 'etc/clearpath/dynamo-camera/d455.yaml').read_text()
    assert parameters['enable_infra1'] is False and parameters['enable_infra2'] is False
    assert parameters['align_depth.enable'] is True and parameters['pointcloud.enable'] is False
    assert parameters['rgb_camera.color_profile'] == parameters['depth_module.depth_profile'] == '640x480x30'
    states = units_now()
    for unit in VENDOR:
        assert states[unit] == {'enabled': 'disabled', 'active': 'inactive'}
    assert states['dynamo-camera.service'] == {'enabled': 'enabled', 'active': 'active'}
    log = calls()
    restart = log.index(['systemctl', 'restart', 'clearpath-robot.service'])
    assert restart < log.index(['systemctl', 'restart', 'dynamo-camera.service'])
    # The camera is enumerated only after the vendor driver released it.
    assert log.index(['systemctl', 'disable', '--now', 'realsense-camera.service']) < log.index(
        ['rs-enumerate-devices', '-s'])
    assert not any('mbs-webserver.service' in call for call in log)
    assert (root / 'etc/clearpath/dynamo-camera-backup/robot.yaml').read_text() == original


def test_unit_publishes_the_simulator_names_on_the_robot_tree():
    unit = load_tool().render_unit('r100_0160')

    assert '-r __ns:=/r100_0160/sensors -r __node:=camera_0' in unit
    assert '-r /tf:=/r100_0160/tf -r /tf_static:=/r100_0160/tf_static' in unit
    assert ('/r100_0160/sensors/camera_0/color/image_raw:='
            '/r100_0160/sensors/camera_0/color/image') in unit
    assert ('/r100_0160/sensors/camera_0/aligned_depth_to_color/image_raw:='
            '/r100_0160/sensors/camera_0/depth/image') in unit
    assert '--params-file /etc/clearpath/dynamo-camera/d455.yaml' in unit
    assert 'source /etc/clearpath/setup.bash' in unit
    assert 'Conflicts=realsense-camera.service' in unit
    assert '@' not in unit


def test_reapply_changes_profile_and_keeps_the_first_snapshot(host):
    root, run, units_now, calls = host
    original = (root / 'etc/clearpath/robot.yaml').read_text()
    assert run('apply', '--yes').returncode == 0

    result = run('apply', '--yes', '--profile', '1280x720')

    assert result.returncode == 0, result.stdout + result.stderr
    params = yaml.safe_load((root / 'etc/clearpath/dynamo-camera/d455.yaml').read_text())
    assert params['/**']['ros__parameters']['rgb_camera.color_profile'] == '1280x720x30'
    assert (root / 'etc/clearpath/dynamo-camera-backup/robot.yaml').read_text() == original
    # The existing entry supplies the serial; the device is not enumerated again.
    assert sum(call[0] == 'rs-enumerate-devices' for call in calls()) == 1


def test_rollback_restores_files_and_vendor_state(host):
    root, run, units_now, calls = host
    original = (root / 'etc/clearpath/robot.yaml').read_bytes()
    before = units_now()
    assert run('apply', '--yes').returncode == 0

    result = run('rollback', '--yes')

    assert result.returncode == 0, result.stdout + result.stderr
    assert (root / 'etc/clearpath/robot.yaml').read_bytes() == original
    assert not (root / 'etc/systemd/system/dynamo-camera.service').exists()
    assert not (root / 'etc/clearpath/dynamo-camera').exists()
    assert not (root / 'etc/clearpath/dynamo-camera-backup').exists()
    after = units_now()
    for unit in VENDOR:
        assert after[unit]['enabled'] == before[unit]['enabled']
        if before[unit]['active'] == 'active':
            assert after[unit]['active'] == 'active'


def test_failed_apply_restores_the_snapshot(host):
    root, run, units_now, calls = host
    original = (root / 'etc/clearpath/robot.yaml').read_bytes()
    (root / 'fail-start').touch()

    result = run('apply', '--yes')

    assert result.returncode != 0
    assert (root / 'etc/clearpath/robot.yaml').read_bytes() == original
    assert units_now()['realsense-camera.service'] == {'enabled': 'enabled', 'active': 'active'}
    assert not (root / 'etc/systemd/system/dynamo-camera.service').exists()


@pytest.mark.parametrize('break_host, reason', [
    (lambda root: (root / 'etc/clearpath/cyclonedds.xml').write_text('<CycloneDDS/>\n'), 'spdp'),
    (lambda root: (root / 'etc/clearpath/setup.bash').write_text('export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"\n'),
     'intel_services_rmw'),
    (lambda root: (root / 'sys/bus/usb/devices/2-5.1/product').write_text('Hub\n'), 'D455'),
    (lambda root: (root / 'etc/clearpath/robot.yaml').write_text(yaml.safe_dump({
        'system': {'ros2': {'namespace': 'r100_0001'}},
        'sensors': {'camera': [{'model': 'intel_realsense', 'ros_parameters': {
            'intel_realsense': {'camera_name': 'camera'}}}]}})), 'does not own'),
])
def test_apply_refuses_before_changing_anything(host, break_host, reason):
    root, run, units_now, calls = host
    break_host(root)
    before = units_now()

    result = run('apply', '--yes')

    assert result.returncode == 2
    assert reason in result.stderr
    assert units_now() == before
    assert not (root / 'etc/clearpath/dynamo-camera-backup').exists()


def test_status_reports_a_reenabled_vendor_camera(host):
    root, run, units_now, calls = host
    assert run('apply', '--yes').returncode == 0
    assert run('status').returncode == 0

    states = units_now()
    states['realsense-camera.service'] = {'enabled': 'enabled', 'active': 'inactive'}
    (root / 'units.json').write_text(json.dumps(states))
    result = run('status')

    assert result.returncode == 1
    assert 'DIFF  realsense-camera.service' in result.stdout
