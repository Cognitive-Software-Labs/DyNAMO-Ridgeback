#!/usr/bin/env python3
"""Check the installed feedback plugin and clock gate across one supervised reboot.

Preparation and verification are read-only for robot services. Results live in
one fresh artifacts/hardware directory; verification requires root only to read
the controller manager's mapped libraries.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

from install_puma_feedback_plugin import DROP, manifest, running_plugin
from verify_auxiliary_boot import verify as verify_auxiliary

ROOT = Path(__file__).resolve().parents[2]
CHRONY = Path('/etc/chrony/conf.d/dynamo-time.conf')
AUX_RECEIPT = ROOT / 'artifacts/hardware/20261013-auxiliary-repair-install'
PLUGIN_RECEIPT = ROOT / 'artifacts/hardware/20261013-puma-feedback-install-stage-5'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def current_boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def load_installed():
    record = json.loads((PLUGIN_RECEIPT / 'result.json').read_text())
    if record['status'] != 'installed-verified-stationary':
        raise RuntimeError('feedback plugin receipt is not installed-verified-stationary')
    prefix = Path(record['prefix'])
    if not DROP.is_file() or DROP.read_text() != record['dropin']:
        raise RuntimeError('installed platform feedback drop-in differs from receipt')
    if manifest(prefix) != record['package_manifest']:
        raise RuntimeError('installed plugin package differs from receipt')
    if 'maxpoll 7' not in CHRONY.read_text():
        raise RuntimeError('installed chrony poll bound is absent')
    return record


def prepare(run):
    if run.exists():
        raise FileExistsError('fresh run directory required: ' + str(run))
    record = load_installed()
    run.mkdir(parents=True)
    baseline = {
        'boot_id': current_boot_id(),
        'plugin_prefix': record['prefix'],
        'package_hash': record['package_hash'],
        'dropin_sha256': digest(DROP),
        'chrony_sha256': digest(CHRONY),
    }
    (run / 'before.json').write_text(json.dumps(baseline, indent=2) + '\n')
    return {'status': 'prepared-for-supervised-reboot', **baseline}


def verify(run):
    before = json.loads((run / 'before.json').read_text())
    record = load_installed()
    checks = {
        'different_boot': current_boot_id() != before['boot_id'],
        'same_plugin_prefix': record['prefix'] == before['plugin_prefix'],
        'same_package_hash': record['package_hash'] == before['package_hash'],
        'same_dropin': digest(DROP) == before['dropin_sha256'],
        'same_chrony_profile': digest(CHRONY) == before['chrony_sha256'],
    }
    if not checks['different_boot']:
        raise RuntimeError('robot has not rebooted since preparation')
    mapped = running_plugin()
    checks['plugin_loaded_after_boot'] = mapped['loaded'] == [record['prefix'] + '/lib/libpuma_hardware.so']
    auxiliary = verify_auxiliary(AUX_RECEIPT, run / 'auxiliary')
    checks['auxiliary_post_boot'] = auxiliary['status'] == 'post-reboot-check-passed'
    result = {
        'status': 'feedback-clock-post-boot-passed' if all(checks.values()) else 'feedback-clock-post-boot-failed',
        'checks': checks,
        'loaded': mapped,
        'auxiliary_report': str(run / 'auxiliary/result.json'),
        'limits': 'One reboot and stationary startup; no feedback-outage or moving-stop qualification.',
    }
    (run / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument('--prepare', action='store_true')
    actions.add_argument('--verify', action='store_true')
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    run = args.run_dir.resolve()
    if not run.is_relative_to(ROOT / 'artifacts/hardware') or run == ROOT / 'artifacts/hardware':
        parser.error('run directory must be under artifacts/hardware')
    if args.verify and os.geteuid() != 0:
        parser.error('post-boot verification needs sudo to inspect controller-manager mappings')
    if args.verify and importlib.util.find_spec('rclpy') is None:
        if os.environ.get('DYNAMO_ROS_SETUP_ATTEMPTED'):
            raise RuntimeError('rclpy unavailable after sourcing ROS Jazzy')
        os.environ['DYNAMO_ROS_SETUP_ATTEMPTED'] = '1'
        os.execv('/bin/bash', ['/bin/bash', '-c',
                               'source /opt/ros/jazzy/setup.bash && exec python3 "$@"',
                               'verify-feedback-boot', str(Path(__file__).resolve()), *sys.argv[1:]])
    result = prepare(run) if args.prepare else verify(run)
    print(json.dumps(result, indent=2))
    return 0 if result['status'].endswith('passed') or args.prepare else 1


if __name__ == '__main__':
    raise SystemExit(main())
