#!/usr/bin/env python3
"""Repair Intel desktop ownership and camera readiness with exact-file rollback."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import pwd
import signal
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

from install_independent_vnc import Installer, FILES as LEGACY_FILES, ROOT, WEB, VNC, PROXY
from desktop_ready import listener_identity

REMOTE = 'ridgeback-remote-screen.service'
CAMERA_READY = 'camera-web-ready.service'
LIB = Path('/usr/local/lib/dynamo-hardware')
SYSTEM = Path('/etc/systemd/system')
UNITS = (WEB, VNC, PROXY, REMOTE, CAMERA_READY)
PRESERVED = ('/etc/clearpath/cyclonedds.xml', '/etc/clearpath/robot.yaml',
             '/usr/local/sbin/realsense-camera-start', '/home/robot/.vnc/xstartup',
             '/home/robot/ridgeback-remote-screen/start-remote-screen.sh',
             '/opt/mybotshop/install/ros2_laser_scan_merger/share/ros2_laser_scan_merger/launch/merge_ridgeback.launch.py')


def sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def dump(path, value):
    """Persist write-ahead intent before changing a system file."""
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, 'w') as handle:
            json.dump(value, handle, indent=2)
            handle.write('\n')
            handle.flush()
            os.fchmod(handle.fileno(), 0o644)
            os.fsync(handle.fileno())
        temporary.replace(path)
        sync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def desired_files():
    vnc = '''[Unit]
Description=Independent Ridgeback VNC desktop
Wants=network.target
Requires=user@1000.service
After=network.target user@1000.service
Before=mbs-webserver.service
StartLimitIntervalSec=0

[Service]
Type=exec
User=robot
Group=robot
Environment=HOME=/home/robot
Environment=XDG_RUNTIME_DIR=/run/user/1000
Environment=DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
Environment=__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
Environment=LIBGL_ALWAYS_SOFTWARE=1
WorkingDirectory=/home/robot
ExecStartPre=/usr/bin/vncserver -list :1 -cleanstale
ExecStart=/usr/bin/vncserver :1 -fg -geometry 1920x1080 -depth 24 -localhost no -xstartup /usr/local/lib/dynamo-hardware/vnc-xstartup
ExecStop=/usr/bin/vncserver -kill :1
Restart=always
RestartSec=3
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
'''
    # Proxies keep listening while the backend recovers; Requires/PartOf would
    # leave them stopped after a failed backend startup. Vendor readiness below
    # prevents its port-guard fallback from claiming either listener.
    proxy = '''[Unit]
Description=Independent Ridgeback VNC browser proxy
Wants=ridgeback-vnc.service
After=ridgeback-vnc.service
Before=mbs-webserver.service
StartLimitIntervalSec=0

[Service]
Type=exec
User=robot
Group=robot
ExecStart=/usr/bin/websockify 9005 127.0.0.1:5901
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
'''
    xstartup = '''#!/bin/sh
unset SESSION_MANAGER DBUS_SESSION_BUS_ADDRESS
export XDG_SESSION_TYPE=x11
# Graphics selection applies to the virtual X server only. Applications keep
# their own GPU selection, including ROS processes launched from this desktop.
unset __EGL_VENDOR_LIBRARY_FILENAMES LIBGL_ALWAYS_SOFTWARE
# Isolate XFCE's session bus from any physical-console desktop. Keep the valid
# per-user runtime directory without overwriting the user manager's DISPLAY.
if command -v startxfce4 >/dev/null 2>&1; then
    exec dbus-run-session -- startxfce4
elif command -v openbox-session >/dev/null 2>&1; then
    exec dbus-run-session -- openbox-session
else
    xsetroot -solid grey
    exec xterm
fi
'''
    camera_wrapper = '''#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/mybotshop/install/setup.bash
exec /usr/bin/python3 /usr/local/lib/dynamo-hardware/camera_web_ready.py --domain 0 --profile /etc/clearpath/cyclonedds.xml --robot-config /etc/clearpath/robot.yaml --timeout 90
'''
    files = {
        SYSTEM / VNC: vnc,
        SYSTEM / PROXY: proxy,
        SYSTEM / 'mbs-webserver.service.d/20-independent-vnc.conf': '''[Unit]
Wants=ridgeback-vnc.service ridgeback-vnc-websockify.service
After=ridgeback-vnc.service ridgeback-vnc-websockify.service

[Service]
ExecStartPre=/usr/bin/python3 /usr/local/lib/dynamo-hardware/desktop_ready.py --timeout 60
''',
        SYSTEM / 'ridgeback-remote-screen.service.d/20-independent-vnc.conf': '''[Unit]
Wants=ridgeback-vnc.service
After=ridgeback-vnc.service
StartLimitIntervalSec=0

[Service]
ExecStart=
ExecStart=/usr/bin/websockify --web=/home/robot/ridgeback-remote-screen 0.0.0.0:9100 127.0.0.1:5901
Restart=always
''',
        SYSTEM / 'camera-web-ready.service.d/30-direct-readiness.conf': '''[Service]
User=robot
Group=robot
Environment=HOME=/home/robot
ExecStart=
ExecStart=/usr/local/lib/dynamo-hardware/camera-web-ready
TimeoutStartSec=100
''',
        LIB / 'vnc-xstartup': xstartup,
        LIB / 'camera-web-ready': camera_wrapper,
    }
    for helper in ('desktop_ready.py', 'camera_web_ready.py'):
        files[LIB / helper] = Path(__file__).with_name(helper).read_text()
    return {path: {'content': text, 'mode': 0o755 if path.parent == LIB else 0o644}
            for path, text in files.items()}


def snapshot_file(path):
    if path.is_symlink():
        raise RuntimeError('Refusing symlink target: ' + str(path))
    if not path.exists():
        return None
    stat = path.stat()
    return {'content_hex': path.read_bytes().hex(), 'mode': stat.st_mode & 0o7777,
            'uid': stat.st_uid, 'gid': stat.st_gid}


def write_file(path, data, mode, uid=0, gid=0):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '.dynamo-', dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, 'wb') as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        temporary.chmod(mode)
        os.chown(temporary, uid, gid)
        with temporary.open('rb') as handle:
            os.fsync(handle.fileno())
        temporary.replace(path)
        sync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


class Repair(Installer):
    def __init__(self, directory):
        super().__init__(directory)
        self.files = desired_files()
        self.before = {}
        self.activity = {}
        self.applied = []

    def state(self):
        dump(self.directory / 'state.json', {'stage': self.stage, 'changed': self.changed,
             'before': self.before, 'activity': self.activity, 'applied': self.applied,
             'desired': {str(p): entry for p, entry in self.files.items()}})

    def prepare(self):
        if socket.gethostname() != 'cpr-r100-0160' or pwd.getpwnam('robot').pw_uid != 1000:
            raise RuntimeError('Repair is restricted to Intel cpr-r100-0160 and robot UID 1000')
        self.check_prerequisites()
        for path, legacy in LEGACY_FILES.items():
            if path.is_symlink() or path.read_text() not in (legacy, self.files[path]['content']):
                raise RuntimeError('Independent desktop file differs from known deployment: ' + str(path))
        for path, entry in self.files.items():
            if path not in LEGACY_FILES and path.exists() and path.read_text() != entry['content']:
                raise RuntimeError('New managed path already has different content: ' + str(path))
            self.before[str(path)] = snapshot_file(path)
        self.activity = {unit: self.command(['systemctl', 'is-active', unit], check=False).strip()
                         for unit in UNITS}
        if self.activity[WEB] != 'active' or self.activity[REMOTE] != 'active':
            raise RuntimeError('Both existing browser services must be active for this repair')
        for unit in UNITS:
            (self.directory / (unit + '.before.txt')).write_text(self.command(['systemctl', 'cat', unit]))
        preserved = {path: hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in PRESERVED}
        dump(self.directory / 'preserved-hashes.json', preserved)
        for path, entry in self.files.items():
            candidate_name = 'candidate-' + str(path).lstrip('/').replace('/', '_')
            (self.directory / candidate_name).write_text(entry['content'])
        candidates = []
        for unit in (VNC, PROXY):
            candidate = self.directory / unit
            candidate.write_text(self.files[SYSTEM / unit]['content'])
            candidates.append(str(candidate))
        self.command(['systemd-analyze', 'verify', *candidates])
        # The old service may be failed because of its discovery mechanism, but
        # the actual camera/web graph must be healthy before a desktop transition.
        output = self.command(['runuser', '-u', 'robot', '--', 'env',
            'HOME=/home/robot', 'USER=robot', 'LOGNAME=robot', '/bin/bash', '-c',
            'set -eo pipefail; source /opt/ros/jazzy/setup.bash; source /opt/mybotshop/install/setup.bash; exec "$@"',
            'camera-ready-preflight', '/usr/bin/python3', str(Path(__file__).with_name('camera_web_ready.py')),
            '--domain', '0', '--profile', '/etc/clearpath/cyclonedds.xml', '--robot-config',
            '/etc/clearpath/robot.yaml', '--timeout', '15'])
        (self.directory / 'camera-ready-before.jsonl').write_text(output)
        self.state()

    def check_prerequisites(self):
        from camera_web_ready import read_namespace
        for value in ('/opt/ros/jazzy/setup.bash', '/opt/mybotshop/install/setup.bash',
                      '/usr/bin/dbus-run-session', '/usr/bin/vncserver', '/usr/bin/websockify',
                      '/usr/share/glvnd/egl_vendor.d/50_mesa.json'):
            if not Path(value).is_file():
                raise RuntimeError('Required desktop/camera prerequisite missing: ' + value)
        password = Path('/home/robot/.vnc/passwd').stat()
        if password.st_uid != 1000 or password.st_size < 8 or password.st_mode & 0o077:
            raise RuntimeError('Existing VNC credentials have invalid ownership, size or permissions')
        read_namespace('/etc/clearpath/robot.yaml')
        ET.fromstring(Path('/etc/clearpath/cyclonedds.xml').read_bytes())
        launch = Path('/opt/mybotshop/install/robot_webserver/share/robot_webserver/launch/webserver.launch.py').read_text()
        if not all(value in launch for value in (':5901 ', ':9005 ', 'already listening, skipping', 'already running, skipping')):
            raise RuntimeError('Vendor launch no longer has expected desktop/proxy guards')

    def check_preserved(self):
        hashes = json.loads((self.directory / 'preserved-hashes.json').read_text())
        for path, expected in hashes.items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
                raise RuntimeError('Preserved deployment changed: ' + path)

    def wait_all(self):
        deadline = time.monotonic() + 45
        error = 'Desktop not ready'
        while time.monotonic() < deadline:
            try:
                listeners = {'vnc': listener_identity(5901, VNC),
                             'proxy': listener_identity(9005, PROXY),
                             'remote': listener_identity(9100, REMOTE)}
                runtime = Path('/run/user/1000')
                if runtime.stat().st_uid != 1000 or runtime.stat().st_mode & 0o777 != 0o700:
                    raise RuntimeError('Desktop runtime ownership/permissions incorrect')
                if not (runtime / 'bus').is_socket():
                    raise RuntimeError('Desktop user D-Bus is missing')
                return listeners
            except (OSError, RuntimeError) as failure:
                error = str(failure)
                time.sleep(.25)
        raise RuntimeError(error)

    def install(self):
        self.prepare()
        self.changed = True
        self.stage = 'installing'
        self.state()
        # A detached systemd worker survives this single desktop transition.
        self.command(['systemctl', 'stop', CAMERA_READY, WEB, REMOTE, PROXY, VNC])
        for path, entry in self.files.items():
            # Record intent before writing so an interrupted write can recover.
            self.applied.append(str(path))
            self.state()
            write_file(path, entry['content'].encode(), entry['mode'])
        self.command(['systemctl', 'daemon-reload'])
        self.command(['systemctl', 'reset-failed', VNC, PROXY, REMOTE, CAMERA_READY])
        self.command(['systemctl', 'start', VNC, PROXY, REMOTE])
        before = self.wait_all()
        dump(self.directory / 'ownership-before.json', before)
        since = time.time()
        self.command(['systemctl', 'start', WEB])
        self.wait_vendor_skips(since)
        if self.wait_all() != before:
            raise RuntimeError('Starting vendor webserver changed desktop listener identities')
        self.command(['systemctl', 'start', CAMERA_READY])
        self.stage = 'verifying'
        self.state()
        since = time.time()
        self.command(['systemctl', 'restart', WEB])
        self.wait_vendor_skips(since)
        after = self.wait_all()
        dump(self.directory / 'ownership-after.json', after)
        if before != after:
            raise RuntimeError('Vendor restart changed desktop listener identities')
        # Check readiness again after the deliberate webserver restart.
        self.command(['systemctl', 'restart', CAMERA_READY])
        self.check_preserved()
        self.stage = 'repaired'
        self.state()
        dump(self.directory / 'verification.json', {'passed': True, 'before': before, 'after': after,
             'cold_boot_tested': False, 'camera_readiness': 'passed', 'dds_unchanged': True})

    def rollback(self):
        for value in self.applied:
            path = Path(value)
            expected = self.files[path]['content'].encode()
            original = self.before[value]
            allowed = [expected] + ([bytes.fromhex(original['content_hex'])] if original else [])
            if path.is_symlink() or (path.exists() and path.read_bytes() not in allowed):
                raise RuntimeError('External edit prevents rollback: ' + value)
        self.command(['systemctl', 'stop', CAMERA_READY, WEB, REMOTE, PROXY, VNC], check=False)
        for value in reversed(self.applied):
            path, original = Path(value), self.before[value]
            if original is None:
                path.unlink(missing_ok=True)
                sync_directory(path.parent)
            else:
                write_file(path, bytes.fromhex(original['content_hex']), original['mode'], original['uid'], original['gid'])
        self.command(['systemctl', 'daemon-reload'])
        # The original remote-screen script also starts display :1. Finish the
        # VNC job before letting that original launcher run during rollback.
        errors = []
        for unit in (VNC, PROXY, REMOTE, WEB, CAMERA_READY):
            if self.activity[unit] == 'active':
                try:
                    self.command(['systemctl', 'start', unit])
                except RuntimeError as error:
                    errors.append(str(error))
        if errors:
            raise RuntimeError('Original service recovery: ' + '; '.join(errors))
        self.check_preserved()
        self.changed = False
        self.stage = 'rolled-back'
        self.state()

    def load_recovery(self, saved):
        """Evidence paths never authorize arbitrary privileged file changes."""
        expected = {str(path): entry for path, entry in self.files.items()}
        if saved.get('desired') != expected or set(saved.get('before', {})) != set(expected):
            raise RuntimeError('Recovery state differs from the managed file allowlist/content')
        applied = saved.get('applied', [])
        if not isinstance(applied, list) or len(applied) != len(set(applied)) or not set(applied) <= set(expected):
            raise RuntimeError('Invalid recovery applied-file list')
        activity = saved.get('activity', {})
        if set(activity) != set(UNITS) or any(value not in ('active', 'inactive', 'failed') for value in activity.values()):
            raise RuntimeError('Invalid recovery unit activity')
        for value, original in saved['before'].items():
            if original is None:
                if Path(value) in LEGACY_FILES:
                    raise RuntimeError('Required original desktop unit missing from recovery')
                continue
            if not isinstance(original, dict) or set(original) != {'content_hex', 'mode', 'uid', 'gid'}:
                raise RuntimeError('Invalid original file metadata')
            content = bytes.fromhex(original['content_hex'])
            allowed = [expected[value]['content'].encode()]
            if Path(value) in LEGACY_FILES:
                allowed.append(LEGACY_FILES[Path(value)].encode())
            if content not in allowed or any(type(original[key]) is not int or original[key] < 0
                                            for key in ('mode', 'uid', 'gid')) or original['mode'] > 0o7777:
                raise RuntimeError('Invalid original managed file content or ownership')
        if type(saved.get('changed')) is not bool or not isinstance(saved.get('stage'), str):
            raise RuntimeError('Invalid recovery stage')
        self.stage, self.changed = saved['stage'], saved['changed']
        self.before, self.activity, self.applied = saved['before'], activity, applied


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-confirmed', action='store_true', required=True)
    parser.add_argument('--detach', action='store_true')
    parser.add_argument('--recover', action='store_true')
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--then-verify-current', type=Path)
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error('Root is needed to repair system services; use sudo locally')
    directory = args.run_dir.resolve()
    runs = [directory] + ([args.then_verify_current.resolve()] if args.then_verify_current else [])
    if any(not p.is_relative_to(ROOT / 'artifacts/hardware') or p == ROOT / 'artifacts/hardware' for p in runs):
        parser.error('Evidence directories must be below artifacts/hardware')
    if args.recover and args.then_verify_current:
        parser.error('Recovery cannot launch new trials')
    if not args.recover and any(p.exists() and any(p.iterdir()) for p in runs):
        parser.error('Choose fresh evidence directories')
    if args.detach:
        unit = 'dynamo-hardware-repair-' + hashlib.sha256(str(directory).encode()).hexdigest()[:12]
        command = ['systemd-run', '--unit=' + unit, '--collect', '--property=KillMode=mixed',
                   '--property=TimeoutStopSec=600', '--', '/usr/bin/python3', str(Path(__file__).resolve()),
                   '--operator-confirmed', '--run-dir', str(directory)]
        if args.recover:
            command.append('--recover')
        if args.then_verify_current:
            command += ['--then-verify-current', str(args.then_verify_current.resolve())]
        result = subprocess.run(command)
        if result.returncode == 0:
            print(f'Independent repair worker: {unit}.service\nEvidence: {directory}', flush=True)
        return result.returncode
    if any(unit in Path('/proc/self/cgroup').read_text() for unit in UNITS):
        parser.error('Use --detach; the repair must survive its desktop transition')
    directory.mkdir(parents=True, exist_ok=True)
    repair = Repair(directory)
    with open('/run/lock/dynamo-local-dds-test.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('An experiment or desktop transition is already running')
        def interrupted(signum, frame):
            raise KeyboardInterrupt(signal.Signals(signum).name)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, interrupted)
        try:
            if args.recover:
                saved = json.loads((directory / 'state.json').read_text())
                repair.load_recovery(saved)
                if repair.changed:
                    repair.rollback()
            else:
                repair.install()
        except (Exception, KeyboardInterrupt) as error:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                signal.signal(sig, signal.SIG_IGN)
            (directory / 'exception.txt').write_text(repr(error) + '\n')
            if repair.changed:
                try:
                    repair.rollback()
                except Exception as recovery:
                    repair.stage = 'recovery-required'
                    repair.state()
                    (directory / 'recovery-error.txt').write_text(repr(recovery) + '\n')
            print('Hardware workflow repair failed: ' + str(error), flush=True)
            return 2
    if args.then_verify_current and not args.recover:
        return subprocess.run(['/usr/bin/python3', str(ROOT / 'tools/intel_thor/run_local_dds_test.py'),
             '--operator-confirmed', '--detach', '--verify-current', '--run-dir', str(args.then_verify_current.resolve())]).returncode
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
