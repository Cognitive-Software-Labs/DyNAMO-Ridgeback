#!/usr/bin/env python3
"""Install/restore a pinned Puma feedback plugin under the stationary test lock."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import install_commissioning_caps as caps
from install_puma_repair import check_caps
from stage_puma_plugin_prefix import stage as stage_plugin, digest as file_digest

ROOT = Path(__file__).resolve().parents[2]
BASE = Path('/etc/systemd/system/clearpath-platform.service.d/90-dynamo-auxiliary.conf')
DROP = Path('/etc/systemd/system/clearpath-platform.service.d/95-dynamo-puma-feedback.conf')
PINNED = Path('/opt/dynamo-puma-feedback')
INDEPENDENT = ('clearpath-sensors.service', 'dynamo-camera.service', 'ridgeback-vnc.service',
               'ridgeback-vnc-websockify.service', 'ridgeback-remote-screen.service')


def save(run, record):
    path = run / 'result.json';temp = run / 'result.tmp'
    temp.write_text(json.dumps(record, indent=2) + '\n');temp.replace(path)


def load(run):
    run = run.resolve()
    if not run.is_relative_to(ROOT / 'artifacts/hardware') or run == ROOT / 'artifacts/hardware':
        raise ValueError('run directory must be below artifacts/hardware')
    return run, json.loads((run / 'result.json').read_text())


def running_plugin():
    found = []
    for process in Path('/proc').iterdir():
        if not process.name.isdecimal(): continue
        try:
            cmd = (process / 'cmdline').read_bytes().split(b'\0')[0].decode()
            if not cmd.endswith('/ros2_control_node'): continue
            mapped = sorted(set(line.split()[-1] for line in (process / 'maps').read_text().splitlines()
                                if '/libpuma_hardware.so' in line))
            found.append({'pid': int(process.name), 'loaded': mapped})
        except (OSError, UnicodeError): continue
    if len(found) != 1 or len(found[0]['loaded']) != 1:
        raise RuntimeError('expected one controller manager and one loaded Puma library: ' + str(found))
    return found[0]


def snapshot_services():
    return {name: caps.state(name) for name in INDEPENDENT}


def manifest(prefix):
    files = {}
    for p in prefix.rglob('*'):
        relative = str(p.relative_to(prefix))
        if p.is_symlink():
            target = p.readlink()
            if target.parent not in (Path('/opt/ros/jazzy/lib'),
                                    Path('/opt/ros/jazzy/lib/clearpath_hardware_interfaces')) or not p.is_file():
                raise RuntimeError('unexpected plugin symlink: ' + relative)
            files[relative] = {'target': str(target), 'sha256': file_digest(p)}
        elif p.is_file(): files[relative] = {'sha256': file_digest(p)}
        elif not p.is_dir(): raise RuntimeError('unexpected plugin package entry')
    return files


def dropin(base, prefix):
    old = "exec ros2 launch /usr/local/lib/dynamo-hardware/auxiliary_platform.launch.py"
    if base.count(old) != 1 or '/opt/dynamo-puma/' not in base:
        raise RuntimeError('installed auxiliary/Puma launch chain changed')
    replacement = (f'export AMENT_PREFIX_PATH={prefix}:$AMENT_PREFIX_PATH; '
                   f'export LD_LIBRARY_PATH={prefix}/lib:$LD_LIBRARY_PATH; ' + old)
    starts = [line for line in base.splitlines() if line.startswith('ExecStart=/bin/bash -c ')]
    if len(starts) != 1 or old not in starts[0]:
        raise RuntimeError('expected one auxiliary platform ExecStart')
    # Leave the earlier drop-in's clock gate and unit dependencies in place.
    return '[Service]\nExecStart=\n' + starts[0].replace(old, replacement) + '\n'


def validate(run, record):
    prefix = Path(record['prefix'])
    package_hash = caps.digest(json.dumps(record['package_manifest'], sort_keys=True).encode())
    if prefix != PINNED / package_hash or record['package_hash'] != package_hash:
        raise RuntimeError('unexpected root prefix')
    if file_digest(BASE) != record['baseline_hash']:
        raise RuntimeError('platform auxiliary configuration changed since staging')
    if manifest(run / 'plugin/prefix') != record['package_manifest']:
        raise RuntimeError('staged plugin package changed')
    if (run / 'dropin.conf').read_text() != record['dropin']:
        raise RuntimeError('staged drop-in changed')
    if caps.digest(Path('/opt/ros/jazzy/lib/libpuma_hardware.so').read_bytes()) != record['apt_puma_hash']:
        raise RuntimeError('apt Puma library changed since staging')
    if caps.identity() != record['namespace']:
        raise RuntimeError('robot identity changed')
    if check_caps(record['namespace']) != record['baseline_caps']:
        raise RuntimeError('commissioning cap/interlock files changed since staging')
    return prefix


def stage(candidate, run):
    run = run.resolve()
    if not run.is_relative_to(ROOT / 'artifacts/hardware') or run.exists():
        raise ValueError('fresh artifacts/hardware directory required')
    if DROP.exists() or DROP.is_symlink(): raise RuntimeError('feedback drop-in already exists')
    if running_plugin()['loaded'] != ['/opt/ros/jazzy/lib/libpuma_hardware.so']:
        raise RuntimeError('baseline controller manager does not load the packaged plugin')
    if caps.state(caps.PLATFORM)['ActiveState'] != 'active' or caps.state(caps.GUARD)['ActiveState'] != 'active':
        raise RuntimeError('capped platform/guard must be active')
    ns = caps.identity()
    baseline_caps = {str(path): caps.digest(data) for path, data in caps.desired_files(ns, True).items()}
    if check_caps(ns) != baseline_caps:
        raise RuntimeError('the temporary commissioning test interlock must be installed')
    run.mkdir(parents=True)
    stage_plugin(candidate, run / 'plugin')
    package_manifest = manifest(run / 'plugin/prefix')
    package_hash = caps.digest(json.dumps(package_manifest, sort_keys=True).encode())
    prefix = PINNED / package_hash
    base = BASE.read_text()
    staged_drop = dropin(base, prefix)
    (run / 'dropin.conf').write_text(staged_drop)
    record = {'status': 'staged-not-installed', 'namespace': ns, 'prefix': str(prefix),
              'baseline_hash': file_digest(BASE), 'apt_puma_hash': file_digest(Path('/opt/ros/jazzy/lib/libpuma_hardware.so')),
              'baseline_caps': baseline_caps,
              'package_manifest': package_manifest, 'package_hash': package_hash,
              'dropin': staged_drop,
              'independent_before': snapshot_services(), 'intent': []}
    save(run, record)
    return record


def secure_root(path):
    for p in (path, *path.parents):
        stat = p.lstat()
        if p.is_symlink() or stat.st_uid != 0 or stat.st_mode & 0o022:
            raise RuntimeError('prefix parent is not root controlled: ' + str(p))


def apply(run, record):
    if record['status'] != 'staged-not-installed': raise RuntimeError('fresh stage required')
    prefix = validate(run, record)
    if DROP.exists() or DROP.is_symlink(): raise RuntimeError('managed drop-in already exists')
    if prefix.exists() and (manifest(prefix) != record['package_manifest'] or
                            prefix.stat().st_uid != 0):
        raise RuntimeError('existing pinned plugin prefix differs from stage')
    if snapshot_services() != record['independent_before']: raise RuntimeError('independent services changed')
    if caps.state(caps.PLATFORM)['ActiveState'] != 'active' or caps.state(caps.GUARD)['ActiveState'] != 'active':
        raise RuntimeError('capped platform/guard not active')
    if running_plugin()['loaded'] != ['/opt/ros/jazzy/lib/libpuma_hardware.so']:
        raise RuntimeError('baseline plugin changed')
    clock = subprocess.run(['/usr/bin/python3', '/usr/local/lib/dynamo-hardware/check_clock_ready.py',
                            '--timeout', '5'], capture_output=True, text=True, timeout=10)
    (run / 'preflight-clock.json').write_text(clock.stdout)
    if clock.returncode:
        raise RuntimeError('platform clock startup gate is not ready; no services changed')
    caps.run_probe(record['namespace'], run, 'before')
    PINNED.mkdir(parents=True, exist_ok=True);secure_root(PINNED)
    record['status'] = 'applying';save(run, record)
    try:
        record['intent'].append('prefix');save(run,record)
        if not prefix.exists():
            shutil.copytree(run / 'plugin/prefix', prefix, symlinks=True)
        for p in (prefix, *prefix.rglob('*')):
            if not p.is_symlink():
                os.chown(p,0,0)
                p.chmod(0o755 if p.is_dir() else 0o644)
        if manifest(prefix) != record['package_manifest']:raise RuntimeError('root-pinned plugin differs from stage')
        caps.systemctl('stop', caps.PLATFORM)
        record['intent'].append('dropin');save(run,record)
        caps.write_file(DROP,record['dropin'].encode())
        caps.systemctl('daemon-reload');caps.systemctl('start',caps.PLATFORM)
        caps.run_probe(record['namespace'],run,'after')
        mapped = running_plugin()
        if mapped['loaded'] != [str(prefix/'lib/libpuma_hardware.so')]:
            raise RuntimeError('running controller manager loaded wrong plugin: '+str(mapped))
        if snapshot_services() != record['independent_before']:
            raise RuntimeError('independent service changed during install')
        record.update(status='installed-verified-stationary',loaded=mapped);save(run,record)
        return record
    except Exception as error:
        caps.systemctl('stop',caps.PLATFORM,check=False)
        if DROP.exists() and DROP.is_file() and DROP.read_text()==record['dropin']:
            DROP.unlink();caps.systemctl('daemon-reload')
        try:
            caps.systemctl('start',caps.PLATFORM)
            caps.run_probe(record['namespace'],run,'restored')
            record.update(status='failed-restored-original-platform',error=str(error))
        except Exception as recovery_error:
            record.update(status='failed-platform-left-stopped',error=str(error),recovery_error=str(recovery_error))
        save(run,record)
        raise


def restore(run, record):
    prefix = validate(run,record)
    if record['status'] not in ('installed-verified-stationary','failed-platform-left-stopped','applying'):
        raise RuntimeError('no installed plugin to restore')
    if DROP.is_symlink() or (DROP.exists() and (not DROP.is_file() or DROP.read_text()!=record['dropin'])):
        raise RuntimeError('managed drop-in changed; refuse overwrite')
    if record['status']=='installed-verified-stationary' and (not DROP.exists() or manifest(prefix)!=record['package_manifest']):
        raise RuntimeError('pinned plugin changed')
    if caps.state(caps.PLATFORM)['ActiveState']=='active':caps.run_probe(record['namespace'],run,'restore-before')
    caps.systemctl('stop',caps.PLATFORM)
    DROP.unlink(missing_ok=True);caps.systemctl('daemon-reload')
    caps.systemctl('start',caps.PLATFORM)
    caps.run_probe(record['namespace'],run,'restore-after')
    if running_plugin()['loaded'] != ['/opt/ros/jazzy/lib/libpuma_hardware.so']:
        raise RuntimeError('original plugin not restored')
    if snapshot_services() != record['independent_before']:
        raise RuntimeError('independent service changed during restore')
    record['status']='restored-original-plugin';save(run,record)
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    action=parser.add_mutually_exclusive_group(required=True)
    for name in ('stage','apply','restore'): action.add_argument('--'+name,action='store_true')
    parser.add_argument('--candidate',type=Path)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--operator-confirmed',action='store_true')
    args=parser.parse_args()
    try:
        if args.stage:
            if not args.candidate:parser.error('stage needs --candidate')
            result=stage(args.candidate,args.run_dir)
        else:
            if args.candidate:parser.error('apply/restore use saved stage')
            if os.geteuid()!=0 or not args.operator_confirmed:parser.error('sudo and stationary operator with pressed e-stop required')
            run,record=load(args.run_dir)
            fd=os.open('/run/lock/dynamo-puma-feedback.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
            try:
                fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
                result=apply(run,record) if args.apply else restore(run,record)
            finally:os.close(fd)
        print(json.dumps({'status':result['status'],'result':str(args.run_dir/'result.json')},indent=2))
    except Exception as error:
        print('Feedback plugin install stopped: '+str(error),file=sys.stderr)
        return 1
    return 0

if __name__=='__main__':raise SystemExit(main())
