#!/usr/bin/env python3
"""Separate Intel's VNC desktop from vendor DDS services, with exact-file rollback."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
WEB = 'mbs-webserver.service'
VNC = 'ridgeback-vnc.service'
PROXY = 'ridgeback-vnc-websockify.service'
FILES = {
    Path('/etc/systemd/system') / VNC: '''[Unit]
Description=Independent Ridgeback VNC desktop
After=network.target
Before=mbs-webserver.service

[Service]
Type=forking
User=robot
Group=robot
Environment=HOME=/home/robot
WorkingDirectory=/home/robot
PIDFile=/home/robot/.vnc/%H:1.pid
ExecStartPre=/usr/bin/vncserver -list :1 -cleanstale
ExecStart=/usr/bin/vncserver :1 -geometry 1920x1080 -depth 24 -localhost no
ExecStop=/usr/bin/vncserver -kill :1
Restart=on-failure
RestartSec=3
TimeoutStartSec=45
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
''',
    Path('/etc/systemd/system') / PROXY: '''[Unit]
Description=Independent Ridgeback VNC browser proxy
Requires=ridgeback-vnc.service
After=ridgeback-vnc.service
PartOf=ridgeback-vnc.service
Before=mbs-webserver.service

[Service]
Type=simple
User=robot
Group=robot
ExecStart=/usr/bin/websockify 9005 127.0.0.1:5901
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
''',
    Path('/etc/systemd/system/mbs-webserver.service.d/20-independent-vnc.conf'): '''[Unit]
Wants=ridgeback-vnc.service ridgeback-vnc-websockify.service
After=ridgeback-vnc.service ridgeback-vnc-websockify.service
''',
}


def dump(path, value):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2) + '\n')
    tmp.replace(path)


def file_bytes():
    return {str(path): text.encode() for path, text in FILES.items()}


class Installer:
    def __init__(self, directory):
        self.directory = directory
        self.changed = False
        self.original_active = False
        self.stage = 'preparation'

    def command(self, args, check=True):
        with (self.directory / 'commands.jsonl').open('a') as log:
            log.write(json.dumps({'time': time.time(), 'argv': args}) + '\n')
        result = subprocess.run(args, text=True, capture_output=True, timeout=120)
        if check and result.returncode:
            raise RuntimeError(f'{args}: {result.stderr[-1000:]}')
        return result.stdout

    def state(self):
        dump(self.directory / 'state.json', {'stage': self.stage, 'changed': self.changed,
              'original_webserver_active': self.original_active,
              'files': list(file_bytes())})

    def listener(self, port, expected_unit, executable):
        output = self.command(['ss', '-H', '-ltnp', f'( sport = :{port} )'])
        pids = set(re.findall(r'pid=(\d+)', output))
        if not pids:
            raise RuntimeError(f'No listener on {port}')
        identities = []
        for pid in sorted(pids):
            proc = Path('/proc') / pid
            if proc.joinpath('exe').resolve().name != executable:
                raise RuntimeError(f'Unexpected listener on {port}: pid {pid}')
            groups = proc.joinpath('cgroup').read_text().splitlines()
            if not any(expected_unit in line.split(':', 2)[-1].split('/') for line in groups):
                raise RuntimeError(f'Listener {pid} belongs to another service')
            stat = proc.joinpath('stat').read_text().rsplit(')', 1)[1].split()
            identities.append({'pid': int(pid), 'start_ticks': int(stat[19])})
        if port in (9005, 9100):
            master = int(self.command(['systemctl', 'show', expected_unit, '-p', 'MainPID', '--value']).strip())
            identities = [identity for identity in identities if identity['pid'] == master]
            if not identities:
                raise RuntimeError(f'Proxy main process does not own port {port}')
        return identities

    def wait_listeners(self):
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            try:
                return {'vnc': self.listener(5901, VNC, 'Xtigervnc'),
                        'proxy': self.listener(9005, PROXY, 'python3.12')}
            except (OSError, RuntimeError):
                time.sleep(.5)
        raise RuntimeError('Independent VNC/proxy did not become ready in their own services')

    def wait_vendor_skips(self, since):
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            journal = self.command(['journalctl', '-u', WEB, '--since', '@' + str(since), '--no-pager'])
            if all(message in journal for message in ('VNC server :1 already listening, skipping',
                                                       'Websockify :9005 already running, skipping')):
                return
            time.sleep(.5)
        raise RuntimeError('Vendor launch did not report leaving independent VNC/proxy alone')

    def validate(self):
        if socket.gethostname() != 'cpr-r100-0160':
            raise RuntimeError('Installer is restricted to Intel cpr-r100-0160')
        if any(path.exists() or path.is_symlink() for path in FILES):
            raise RuntimeError('A managed VNC file already exists; inspect before replacing it')
        for unit in (VNC, PROXY):
            if self.command(['systemctl', 'show', unit, '-p', 'LoadState', '--value']).strip() != 'not-found':
                raise RuntimeError('A VNC unit with this name already exists')
        launch = Path('/opt/mybotshop/install/robot_webserver/share/robot_webserver/launch/webserver.launch.py').read_text()
        if not all(text in launch for text in (':5901 ', ':9005 ', 'already listening, skipping', 'already running, skipping')):
            raise RuntimeError('Vendor launch lacks expected VNC/proxy port guards')
        if not Path('/home/robot/.vnc/passwd').is_file():
            raise RuntimeError('Existing robot VNC credentials are missing')
        self.original_active = self.command(['systemctl', 'is-active', WEB], check=False).strip() == 'active'
        if not self.original_active:
            raise RuntimeError('Vendor webserver must be active before this controlled transition')
        for unit in (WEB,):
            (self.directory / (unit + '.before.txt')).write_text(self.command(['systemctl', 'cat', unit]))
        (self.directory / 'listeners-before.txt').write_text(self.command(['ss', '-ltnp']))
        preserved = ['/etc/clearpath/cyclonedds.xml', '/etc/clearpath/robot.yaml', '/usr/local/sbin/realsense-camera-start']
        dump(self.directory / 'preserved-hashes.json', {
            path: hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in preserved})
        for path, text in FILES.items():
            (self.directory / path.name).write_text(text)
        self.command(['systemd-analyze', 'verify', *[str(self.directory / unit) for unit in (VNC, PROXY)]])

    def rollback(self):
        # Never remove a file someone changed after this installation.
        for path, text in FILES.items():
            if path.is_symlink() or (path.exists() and path.read_text() != text):
                raise RuntimeError('Managed file was externally changed: ' + str(path))
        self.command(['systemctl', 'stop', WEB], check=False)
        self.command(['systemctl', 'disable', '--now', PROXY, VNC], check=False)
        for path in FILES:
            path.unlink(missing_ok=True)
        self.command(['systemctl', 'daemon-reload'])
        if self.original_active:
            self.command(['systemctl', 'start', WEB])
        self.changed = False
        self.stage = 'rolled-back'
        self.state()

    def install(self):
        self.validate()
        self.changed = True
        self.stage = 'installing'
        self.state()
        # This one transition closes the old desktop; systemd owns this installer.
        self.command(['systemctl', 'stop', WEB])
        for path, text in FILES.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
            path.chmod(0o644)
        self.command(['systemctl', 'daemon-reload'])
        self.command(['systemctl', 'enable', VNC, PROXY])
        self.command(['systemctl', 'start', VNC, PROXY])
        before = self.wait_listeners()
        since = time.time()
        self.command(['systemctl', 'start', WEB])
        self.wait_vendor_skips(since)
        if self.wait_listeners() != before:
            raise RuntimeError('Starting webserver changed desktop listener identity')
        self.stage = 'verifying'
        self.state()
        since = time.time()
        self.command(['systemctl', 'restart', WEB])
        self.wait_vendor_skips(since)
        after = self.wait_listeners()
        if after != before:
            raise RuntimeError('Restarting webserver changed desktop listener identity')
        preserved = json.loads((self.directory / 'preserved-hashes.json').read_text())
        if any(hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest for path, digest in preserved.items()):
            raise RuntimeError('Preserved DDS/camera/robot configuration changed during VNC installation')
        self.stage = 'installed'
        self.state()
        dump(self.directory / 'verification.json', {'passed': True, 'before': before, 'after': after})
        print('Independent VNC verified: desktop listener identities survived webserver restart.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-confirmed', action='store_true', required=True)
    parser.add_argument('--detach', action='store_true')
    parser.add_argument('--recover', action='store_true', help='Remove this installation and restore vendor ownership')
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--then-focused-run', type=Path)
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error('Run with sudo; service installation requires root')
    directory = args.run_dir.resolve()
    evidence = ROOT / 'artifacts/hardware'
    paths = [directory] + ([args.then_focused_run.resolve()] if args.then_focused_run else [])
    if any(not p.is_relative_to(evidence) or p == evidence for p in paths):
        parser.error('Evidence directories must be below artifacts/hardware/')
    if not args.recover and directory.exists() and any(directory.iterdir()):
        parser.error('Choose a new installation evidence directory')
    if args.then_focused_run and args.then_focused_run.exists() and any(args.then_focused_run.iterdir()):
        parser.error('Choose a new focused-run directory')
    if args.detach:
        token = hashlib.sha256(str(directory).encode()).hexdigest()[:12]
        command = ['systemd-run', '--unit=dynamo-vnc-install-' + token, '--collect',
                   '--property=KillMode=mixed', '--property=TimeoutStopSec=600',
                   '--', '/usr/bin/python3', str(Path(__file__).resolve()),
                   '--operator-confirmed', '--run-dir', str(directory)]
        if args.recover:
            command.append('--recover')
        if args.then_focused_run:
            command += ['--then-focused-run', str(args.then_focused_run.resolve())]
        return subprocess.run(command).returncode
    if any(unit in Path('/proc/self/cgroup').read_text() for unit in (WEB, VNC, PROXY)):
        parser.error('Use --detach from the desktop; this installer must survive its service transition')
    directory.mkdir(parents=True, exist_ok=True)
    installer = Installer(directory)
    lock = open('/run/lock/dynamo-local-dds-test.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('A DDS experiment is running; finish it before changing desktop ownership')
    def interrupted(signum, frame):
        raise KeyboardInterrupt(signal.Signals(signum).name)
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupted)
    try:
        if args.recover:
            saved = json.loads((directory / 'state.json').read_text())
            installer.original_active = saved['original_webserver_active']
            installer.changed = saved['changed']
            if installer.changed:
                installer.rollback()
        else:
            installer.install()
    except (Exception, KeyboardInterrupt) as error:
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(signum, signal.SIG_IGN)
        (directory / 'exception.txt').write_text(repr(error) + '\n')
        if installer.changed:
            try:
                installer.rollback()
            except Exception as recovery_error:
                installer.stage = 'recovery-required'
                installer.state()
                (directory / 'recovery-error.txt').write_text(repr(recovery_error) + '\n')
        print('VNC installation failed: ' + str(error), flush=True)
        return 2
    finally:
        lock.close()
    if args.then_focused_run and not args.recover:
        command = ['/usr/bin/python3', str(ROOT / 'tools/intel_thor/run_local_dds_test.py'),
                   '--operator-confirmed', '--detach', '--focused-native',
                   '--run-dir', str(args.then_focused_run.resolve())]
        return subprocess.run(command).returncode
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
