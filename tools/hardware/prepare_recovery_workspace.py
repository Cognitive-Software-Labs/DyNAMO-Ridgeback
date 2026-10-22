#!/usr/bin/env python3
"""Prepare a fresh ROS hardware recovery workspace and host-file candidates.

Reads pinned public source and captured setup inputs; never installs files,
starts services, contacts ROS, or changes the running vendor workspace.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import tarfile

from verify_recovery import DEFAULT_ROOT, verify

REPO = Path(__file__).resolve().parents[2]
MERGER = Path('src/third_party/tools/ros2_laser_scan_merger')

# Recovery scope: ROS devices and independent desktop; no vendor web dashboard,
# navigation, reset/enable helper or boot script that rewrites vendor configs.
CAPTURED = [
    '/etc/clearpath/robot.yaml', '/etc/clearpath/cyclonedds.xml',
    '/etc/sysctl.d/60-dynamo-dds-buffers.conf', '/etc/sysctl.d/99-ridgeback-dual-homing.conf',
    '/etc/udev/rules.d/99-realsense-libusb.rules', '/etc/udev/rules.d/99-ridgeback-ps4.rules',
    '/etc/modprobe.d/bluetooth.conf', '/etc/modules-load.d/test_power.conf',
    '/etc/systemd/logind.conf.d/10-ridgeback-no-suspend.conf', '/etc/ds4drv.conf',
    '/etc/battery-guard.conf', '/usr/local/share/battery-bridge/udp_only.xml',
    '/usr/local/bin/battery_guard.py', '/usr/local/bin/battery_sysfs_bridge.py',
    '/usr/local/bin/power_blackbox.py', '/usr/local/bin/power_report.py',
    '/usr/local/bin/controller-autoconnect.sh', '/home/robot/controllers.txt',
    '/opt/mybotshop/scripts/joy_x_combiner.py',
    '/home/robot/ridgeback-remote-screen/index.html',
    '/etc/systemd/system/realsense-camera.service',
    '/etc/systemd/system/clearpath-scan-merger.service',
    '/etc/systemd/system/clearpath-joy-x-combiner.service',
    '/etc/systemd/system/controller-autoconnect.service',
    '/etc/systemd/system/battery-guard.service',
    '/etc/systemd/system/battery-sysfs-bridge.service',
    '/etc/systemd/system/power-blackbox.service',
    '/etc/systemd/system/ridgeback-remote-screen.service',
    '/etc/systemd/system/clearpath-platform.service.d/override.conf',
    '/etc/systemd/system/realsense-camera.service.d/60-dynamo-cyclonedds.conf',
    '/etc/systemd/system/clearpath-scan-merger.service.d/60-dynamo-cyclonedds.conf',
    '/etc/systemd/system/clearpath-joy-x-combiner.service.d/60-dynamo-cyclonedds.conf',
]


def git(checkout: Path, *args: str) -> bytes:
    return subprocess.check_output(['git', '-C', str(checkout), *args], stderr=subprocess.DEVNULL)


def unpack_public_source(archive: bytes, destination: Path) -> None:
    """Extract only regular files/directories with canonical relative names."""
    with tarfile.open(fileobj=io.BytesIO(archive)) as stream:
        members = stream.getmembers()
        for member in members:
            path = PurePosixPath(member.name)
            if (path.is_absolute() or '..' in path.parts or not path.parts
                    or not (member.isfile() or member.isdir())):
                raise ValueError('Unsafe entry in public Git archive')
        for member in members:
            target = destination / member.name
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(stream.extractfile(member).read())
                target.chmod(member.mode & 0o777)


def desktop_candidates() -> dict:
    # Reuse the maintained stationary desktop/Bluetooth renderer. No installer
    # execution or vendor camera/web readiness files are selected.
    from install_stationary_setup import desired_files
    return desired_files()


def prepare(upstream: Path, output: Path, deployment: Path = DEFAULT_ROOT) -> dict:
    deployment = deployment.resolve(strict=True)
    report = verify(deployment)
    if not report.ok:
        raise ValueError('Captured inputs failed verification: ' + '; '.join(report.errors))
    output = output.absolute()
    if output.exists() or output.is_symlink():
        raise ValueError('Output already exists; choose a fresh directory')
    resolved = output.resolve()
    permitted = [REPO / 'artifacts', Path('/tmp')]
    if not any(resolved.is_relative_to(p.resolve()) and resolved != p.resolve() for p in permitted):
        raise ValueError('Output must be a new directory below repository artifacts/ or /tmp/')
    upstream = upstream.resolve(strict=True)
    lock = json.loads((deployment / 'reconstruction/scan_merger.lock.json').read_text())
    revision = lock['revision']
    if not re.fullmatch(r'[0-9a-f]{40}', revision):
        raise ValueError('Invalid public source revision')
    if git(upstream, 'rev-parse', 'HEAD').decode().strip() != revision:
        raise ValueError('Public checkout is not at the recorded revision')
    if git(upstream, 'status', '--porcelain', '--untracked-files=all').strip():
        raise ValueError('Public checkout has local changes or untracked files')
    for entry in lock['files']:
        path = PurePosixPath(entry['path'])
        if path.is_absolute() or '..' in path.parts:
            raise ValueError('Unsafe lock-file path')
        if hashlib.sha256(git(upstream, 'show', revision + ':' + str(path))).hexdigest() != entry['sha256']:
            raise ValueError('Public source differs from recorded build-source fingerprint')
    archive = git(upstream, 'archive', '--format=tar', revision)
    manifest = json.loads((deployment / 'manifest.json').read_text())
    entries = {e['source']: e for e in manifest['files']}
    candidates = {}
    camera_config = deployment / 'camera.json'
    selected = CAPTURED
    if camera_config.exists():
        selected = [p for p in CAPTURED if p not in (
            '/etc/systemd/system/realsense-camera.service',
            '/etc/systemd/system/realsense-camera.service.d/60-dynamo-cyclonedds.conf')]
    for origin in selected:
        entry = entries[origin]
        candidates[origin] = {
            'content': (deployment / entry['repo_path']).read_text(),
            'mode': int(entry['mode'], 8), 'owner': entry['owner'], 'group': entry['group'],
            'provenance': entry['repo_path'],
        }
    if camera_config.exists():
        import importlib.util
        from importlib.machinery import SourceFileLoader
        loader = SourceFileLoader('recovery_camera_service', str(REPO / 'tools/intel_thor/camera_service'))
        spec = importlib.util.spec_from_loader(loader.name, loader)
        camera_tool = importlib.util.module_from_spec(spec)
        loader.exec_module(camera_tool)
        import yaml
        settings = json.loads(camera_config.read_text())
        robot = yaml.safe_load(candidates['/etc/clearpath/robot.yaml']['content'])
        robot.setdefault('sensors', {})['camera'] = [camera_tool.camera_entry(settings['serial'])]
        candidates['/etc/clearpath/robot.yaml']['content'] = yaml.safe_dump(robot, sort_keys=False)
        candidates['/etc/clearpath/robot.yaml']['provenance'] += '; camera.json + maintained description-only camera_entry'
        ns = camera_tool.namespace(robot)
        for origin, content in {
            '/etc/systemd/system/dynamo-camera.service': camera_tool.render_unit(ns, settings['dds_profile']),
            '/etc/clearpath/dynamo-camera/d455.yaml': camera_tool.render_params(settings['serial'], settings['profile'], settings['fps']),
        }.items():
            candidates[origin] = dict(content=content, mode=0o644, owner='root', group='root',
                provenance='camera.json + tools/intel_thor/camera_service renderer; no service activation')
    else:
        camera_entry = entries['/usr/local/sbin/realsense-camera-start']
        camera = (deployment / camera_entry['repo_path']).read_text()
        old_source = 'source /opt/mybotshop/install/setup.bash 2>/dev/null || true'
        if camera.count(old_source) != 1:
            raise ValueError('Unexpected captured camera wrapper; review recovery adaptation')
        candidates['/usr/local/sbin/realsense-camera-start'] = {
            'content': camera.replace(old_source, 'source /etc/clearpath/setup.bash'),
            'mode': 0o755, 'owner': 'root', 'group': 'root',
            'provenance': 'captured camera wrapper; optional vendor source replaced by Clearpath setup',
        }
    candidates['/etc/systemd/system/clearpath-scan-merger.service.d/30-recovery-workspace.conf'] = {
        'content': """[Service]
ExecStart=
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source /etc/clearpath/setup.bash && source /opt/dynamo-hardware/install/local_setup.bash && exec ros2 launch ros2_laser_scan_merger merge_ridgeback.launch.py'
""", 'mode': 0o644, 'owner': 'root', 'group': 'root',
        'provenance': 'recovery adaptation: public hardware overlay instead of vendor workspace',
    }
    for origin, data in desktop_candidates().items():
        candidates[origin] = dict(data, owner='root', group='root',
                                  provenance='tools/hardware/install_stationary_setup.py desired_files; desktop and Bluetooth only')
    from install_commissioning_caps import desired_files as commissioning_files
    import yaml
    robot_candidate = yaml.safe_load(candidates['/etc/clearpath/robot.yaml']['content'])
    for origin, content in commissioning_files(robot_candidate['system']['ros2']['namespace']).items():
        candidates[str(origin)] = dict(content=content.decode(), mode=0o644,
            owner='root', group='root',
            provenance='tools/hardware/install_commissioning_caps.py desired_files; fixed commissioning caps')
    # New output only. Partial preparation is left for inspection on failure.
    resolved.mkdir(parents=True)
    unpack_public_source(archive, resolved / MERGER)
    for relative in ('launch/merge_ridgeback.launch.py', 'config/params_ridgeback.yaml'):
        origin = '/opt/mybotshop/' + str(MERGER / relative)
        entry = entries[origin]
        target = resolved / MERGER / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((deployment / entry['repo_path']).read_bytes())
    installed = []
    for origin, data in sorted(candidates.items()):
        destination = resolved / 'host-files' / origin.lstrip('/')
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(data['content'])
        destination.chmod(0o644)  # Inert candidates; original target modes in manifest.
        installed.append({k: v for k, v in data.items() if k != 'content'} | {
            'destination': origin, 'candidate': str(destination.relative_to(resolved)),
            'sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
        })
    record = {
        'status': 'prepared-not-installed', 'vendor_dashboard_required': False,
        'public_source': lock['repository'], 'public_revision': revision,
        'input_manifest_sha256': hashlib.sha256((deployment / 'manifest.json').read_bytes()).hexdigest(),
        'camera_input_sha256': hashlib.sha256(camera_config.read_bytes()).hexdigest() if camera_config.exists() else None,
        'warnings': report.warnings, 'host_files': installed,
        'fresh_local_steps': ['Wi-Fi/SSH/Tailscale credentials', 'vncpasswd as robot',
                              'symlink noVNC1.3.0 to /usr/share/novnc',
                              'build at final /opt/dynamo-hardware prefix',
                              'operator-supervised service activation and cold boot'],
    }
    (resolved / 'RECOVERY_PREPARATION.json').write_text(json.dumps(record, indent=2) + '\n')
    return record


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, required=True, help='clean public merger checkout at recorded pin')
    parser.add_argument('--output', type=Path, required=True, help='new directory under artifacts/ or /tmp/')
    parser.add_argument('--deployment', type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args(argv)
    try:
        result = prepare(args.upstream, args.output, args.deployment)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError):
        print('Preparation failed; verify inputs, public revision and fresh output path. No host files installed.', file=sys.stderr)
        return 1
    print(f"Prepared public source and {len(result['host_files'])} host-file candidates; no services or installed files changed.")
    for warning in result['warnings']:
        print('WARN: ' + warning)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
