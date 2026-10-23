#!/usr/bin/env python3
"""Install/restore the pinned per-wheel field-age Puma driver, physically stopped.

  python3 install_puma_field_driver.py --stage --candidate artifacts/hardware/<build> --run-dir <fresh>
  sudo python3 install_puma_field_driver.py --apply --operator-confirmed --run-dir <same>
  sudo python3 install_puma_field_driver.py --restore --operator-confirmed --run-dir <same>

The candidate comes from stage_puma_field_candidate.py. Staging verifies its
input hashes against the maintained patches/policy header, its dependency
versions and binary hash, and renders one later platform drop-in that copies
the installed feedback-plugin ExecStart and swaps only the Puma driver prefix.
Apply and restore restart clearpath-platform.service only, with the physical
stop pressed and verified before and after; the guard, camera, sensors and
desktop services are not touched. A failed apply removes its drop-in, restarts
the original driver and verifies it. Restore does the same on request. The
root-owned prefix stays on disk for evidence; no command is ever published.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys

import install_commissioning_caps as caps
from build_puma_repair import versions
from install_puma_feedback_plugin import running_plugin
from install_puma_repair import file_manifest

ROOT = Path(__file__).resolve().parents[2]
PINNED = Path('/opt/dynamo-puma-field')
BASE = Path('/etc/systemd/system/clearpath-platform.service.d/95-dynamo-puma-feedback.conf')
DROP = Path('/etc/systemd/system/clearpath-platform.service.d/97-dynamo-puma-field-driver.conf')
REPAIR = re.compile(r'/opt/dynamo-puma/[0-9a-f]{64}(?=/share/puma_motor_driver/local_setup\.bash)')
INDEPENDENT = ('clearpath-sensors.service', 'dynamo-camera.service', caps.GUARD, 'ridgeback-vnc.service',
               'ridgeback-vnc-websockify.service', 'ridgeback-remote-screen.service')
BUILT = 'offline-field-age-driver-built-not-installed'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(run, record):
    temporary = run / 'result.tmp'
    temporary.write_text(json.dumps(record, indent=2) + '\n')
    temporary.replace(run / 'result.json')


def run_directory(path):
    path = path.resolve()
    if not path.is_relative_to(ROOT / 'artifacts/hardware') or path == ROOT / 'artifacts/hardware':
        raise ValueError('run directory must be below artifacts/hardware')
    return path


def cap_mode(ns):
    """Name of the verified commissioning cap installation; any other state refuses."""
    for name, options in (('normal', {}), ('test-interlock', {'test_interlock': True}),
                          ('original-manual-speeds', {'original_manual_speeds': True})):
        files = caps.desired_files(ns, **options)
        if all(not p.is_symlink() and p.is_file() and p.read_bytes() == data for p, data in files.items()):
            return name, {str(p): caps.digest(data) for p, data in files.items()}
    raise RuntimeError('a verified commissioning cap installation is required')


def running_driver():
    found = []
    for process in Path('/proc').iterdir():
        if not process.name.isdecimal():
            continue
        try:
            argv0 = (process / 'cmdline').read_bytes().split(b'\0')[0].decode()
        except (OSError, UnicodeError):
            continue
        if argv0.endswith('/multi_puma_node'):
            found.append({'pid': int(process.name), 'executable': argv0})
    if len(found) != 1:
        raise RuntimeError('expected exactly one Puma driver process: ' + json.dumps(found))
    return found[0]


def services():
    return {name: caps.state(name).get('ActiveState') for name in INDEPENDENT}


def dropin(base, prefix):
    """The installed feedback ExecStart with only the driver prefix replaced."""
    starts = [line for line in base.splitlines() if line.startswith('ExecStart=/bin/bash -c ')]
    if len(starts) != 1 or len(REPAIR.findall(starts[0])) != 1:
        raise RuntimeError('expected one feedback-plugin ExecStart sourcing one pinned Puma repair')
    return ('[Service]\n# Per-wheel field-age Puma driver; remove with install_puma_field_driver.py --restore.\n'
            'ExecStart=\n' + REPAIR.sub(str(prefix), starts[0]) + '\n')


def check_candidate(candidate):
    candidate = candidate.resolve()
    if not candidate.is_relative_to(ROOT / 'artifacts/hardware'):
        raise ValueError('candidate must be below artifacts/hardware')
    record = json.loads((candidate / 'candidate.json').read_text())
    if record.get('status') != BUILT or record.get('motor_node_started') is not False:
        raise ValueError('candidate did not complete the offline field-age build and checks')
    changed = [name for name, sha in record['input_sha256'].items() if digest(ROOT / name) != sha]
    if changed:
        raise ValueError('maintained inputs changed since the candidate build: ' + ', '.join(changed))
    if record['dependency_versions'] != versions():
        raise ValueError('runtime dependencies differ from the candidate build')
    binary = candidate / 'install/puma_motor_driver/lib/puma_motor_driver/multi_puma_node'
    if digest(binary) != record['motor_binary_sha256']:
        raise ValueError('built driver differs from the recorded candidate')
    return candidate, record


def stage(candidate, run):
    run = run_directory(run)
    if run.exists():
        raise ValueError('choose a fresh run directory')
    candidate, provenance = check_candidate(candidate)
    if DROP.exists() or DROP.is_symlink():
        raise RuntimeError('field-driver drop-in already installed; restore it first')
    if caps.state(caps.PLATFORM).get('ActiveState') != 'active':
        raise RuntimeError('platform must be active on the original driver')
    driver = running_driver()
    base = BASE.read_text()
    repair = REPAIR.findall(base)
    if len(repair) != 1 or driver['executable'] != repair[0] + '/lib/puma_motor_driver/multi_puma_node':
        raise RuntimeError('baseline is not the pinned Puma repair named by ' + str(BASE))
    ns = caps.identity()
    mode, cap_hashes = cap_mode(ns)
    run.mkdir(parents=True)
    shutil.copytree(candidate / 'install/puma_motor_driver', run / 'package')
    shutil.copy2(candidate / 'candidate.json', run / 'candidate.json')
    manifest = file_manifest(run / 'package')
    package_hash = caps.digest(json.dumps(manifest, sort_keys=True).encode())
    prefix = PINNED / package_hash
    text = dropin(base, prefix)
    (run / 'dropin.conf').write_text(text)
    record = {'status': 'staged-not-installed', 'namespace': ns, 'cap_mode': mode, 'baseline_caps': cap_hashes,
              'candidate': str(candidate), 'candidate_binary_sha256': provenance['motor_binary_sha256'],
              'age_bounds_ms': provenance.get('candidate_age_bounds_ms'),
              'prefix': str(prefix), 'package_hash': package_hash, 'package_manifest': manifest,
              'baseline_hash': digest(BASE), 'baseline_driver': driver['executable'],
              'baseline_plugin': running_plugin()['loaded'], 'dropin': text,
              'services_before': services(), 'dependency_versions': versions(),
              'source': 'tools/hardware/install_puma_field_driver.py'}
    save(run, record)
    return record


def validate(run, record):
    prefix = Path(record['prefix'])
    if (prefix != PINNED / record['package_hash'] or
            caps.digest(json.dumps(record['package_manifest'], sort_keys=True).encode()) != record['package_hash']):
        raise RuntimeError('unexpected pinned prefix')
    if file_manifest(run / 'package') != record['package_manifest']:
        raise RuntimeError('staged package changed')
    if (run / 'dropin.conf').read_text() != record['dropin'] or digest(BASE) != record['baseline_hash']:
        raise RuntimeError('platform configuration or staged drop-in changed since staging')
    if caps.identity() != record['namespace'] or cap_mode(record['namespace'])[1] != record['baseline_caps']:
        raise RuntimeError('robot identity or commissioning caps changed since staging')
    if versions() != record['dependency_versions']:
        raise RuntimeError('runtime dependencies changed since staging')
    return prefix


def secure_root(path):
    for p in (path, *path.parents):
        info = p.lstat()
        if p.is_symlink() or info.st_uid != 0 or info.st_mode & 0o022:
            raise RuntimeError('prefix parent is not root controlled: ' + str(p))


def verify_running(record, executable, run, phase):
    """Physically stopped, controller active, expected driver and unchanged plugin."""
    # The post-start probe waits for the controller, guard routing and mux stop.
    (run / phase).mkdir(exist_ok=True)
    caps.run_probe(record['namespace'], run / phase, 'after')
    driver = running_driver()
    if driver['executable'] != executable:
        raise RuntimeError(f'{phase}: unexpected Puma driver {driver["executable"]}')
    if running_plugin()['loaded'] != record['baseline_plugin']:
        raise RuntimeError(f'{phase}: hardware plugin changed')
    if services() != record['services_before']:
        raise RuntimeError(f'{phase}: an independent service changed state')
    return driver


def restart_original(record, run, phase):
    caps.systemctl('stop', caps.PLATFORM, check=False)
    if DROP.is_file() and not DROP.is_symlink() and DROP.read_text() == record['dropin']:
        DROP.unlink()
    caps.systemctl('daemon-reload')
    caps.systemctl('start', caps.PLATFORM)
    return verify_running(record, record['baseline_driver'], run, phase)


def apply(run, record):
    if record['status'] != 'staged-not-installed':
        raise RuntimeError('a fresh stage is required')
    prefix = validate(run, record)
    if DROP.exists() or DROP.is_symlink():
        raise RuntimeError('field-driver drop-in already exists')
    if running_driver()['executable'] != record['baseline_driver']:
        raise RuntimeError('baseline driver changed since staging')
    caps.require_clock_start_margin()
    caps.run_probe(record['namespace'], run, 'before')
    PINNED.mkdir(parents=True, exist_ok=True)
    secure_root(PINNED)
    if prefix.exists():
        if file_manifest(prefix) != record['package_manifest']:
            raise RuntimeError('existing pinned prefix differs from stage')
    else:
        shutil.copytree(run / 'package', prefix)
    for p in (prefix, *prefix.rglob('*')):
        os.chown(p, 0, 0)
        p.chmod(0o755 if p.is_dir() else p.stat().st_mode & 0o755)
    if file_manifest(prefix) != record['package_manifest']:
        raise RuntimeError('root-owned prefix differs from stage')
    record['status'] = 'applying'
    save(run, record)
    try:
        caps.systemctl('stop', caps.PLATFORM)
        caps.write_file(DROP, record['dropin'].encode())
        caps.systemctl('daemon-reload')
        caps.systemctl('start', caps.PLATFORM)
        expected = str(prefix / 'lib/puma_motor_driver/multi_puma_node')
        record.update(status='installed-verified-stationary', driver=verify_running(record, expected, run, 'after'))
    except Exception as error:
        record.update(error=str(error))
        try:
            restart_original(record, run, 'rollback')
            record['status'] = 'failed-restored-original-driver'
        except Exception as recovery:
            record.update(status='failed-platform-needs-attention', recovery_error=str(recovery))
        save(run, record)
        raise
    save(run, record)
    return record


def restore(run, record):
    validate(run, record)
    if record['status'] not in ('installed-verified-stationary', 'applying', 'failed-platform-needs-attention'):
        raise RuntimeError('no installed field driver recorded in this run')
    if DROP.is_symlink() or (DROP.exists() and DROP.read_text() != record['dropin']):
        raise RuntimeError('field-driver drop-in changed; refuse to remove it')
    if caps.state(caps.PLATFORM).get('ActiveState') == 'active':
        caps.run_probe(record['namespace'], run, 'restore-before')
    record.update(status='restored-original-driver', restored=restart_original(record, run, 'restore-after'),
                  retained_prefix=record['prefix'])
    save(run, record)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    action = parser.add_mutually_exclusive_group(required=True)
    for name in ('stage', 'apply', 'restore'):
        action.add_argument('--' + name, action='store_true')
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--operator-confirmed', action='store_true')
    args = parser.parse_args()
    try:
        if args.stage:
            if not args.candidate:
                parser.error('--stage needs --candidate')
            result = stage(args.candidate, args.run_dir)
        else:
            if args.candidate:
                parser.error('apply/restore use the saved stage; omit --candidate')
            if os.geteuid() != 0 or not args.operator_confirmed:
                parser.error('sudo and --operator-confirmed (physical stop pressed, operator present) required')
            run = run_directory(args.run_dir)
            record = json.loads((run / 'result.json').read_text())
            fd = os.open('/run/lock/dynamo-puma-field-driver.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                result = apply(run, record) if args.apply else restore(run, record)
            finally:
                os.close(fd)
        print(json.dumps({'status': result['status'], 'result': str(args.run_dir / 'result.json')}, indent=2))
        return 0
    except Exception as error:
        print('Field-age driver install stopped: ' + str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
