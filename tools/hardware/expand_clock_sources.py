#!/usr/bin/env python3
"""Expand Intel's existing Ubuntu NTP source without restarting or stepping clocks."""
import argparse
import csv
import fcntl
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile

PROFILE = Path('/etc/chrony/conf.d/dynamo-time.conf')
OLD = 'server ntp.ubuntu.com iburst'
NEW = 'pool ntp.ubuntu.com iburst maxsources 4'


def run(*args):
    return subprocess.run(args, check=True, capture_output=True, text=True, timeout=15).stdout


def candidate(text):
    if text.splitlines().count(OLD) != 1:
        raise RuntimeError('Expected exactly one unchanged Ubuntu server declaration')
    if 'maxslewrate 500' not in text.splitlines():
        raise RuntimeError('Expected installed slew policy')
    return text.replace(OLD + '\n', NEW + '\n', 1)


def sources():
    return {row[2] for row in csv.reader(run('chronyc', '-c', '-n', 'sources').splitlines()) if len(row) > 2}


def write_profile(text, metadata):
    fd, name = tempfile.mkstemp(dir=PROFILE.parent, prefix='.dynamo-time-')
    try:
        with os.fdopen(fd, 'w') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.chown(name, metadata['uid'], metadata['gid'])
        os.chmod(name, metadata['mode'])
        os.replace(name, PROFILE)
    finally:
        if os.path.exists(name): os.unlink(name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--restore', action='store_true')
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.apply and args.restore: parser.error('Choose apply or restore')
    if (args.apply or args.restore) and (os.geteuid() != 0 or not args.operator_confirmed):
        parser.error('sudo and stationary operator confirmation required')
    root = args.run_dir.resolve()
    if args.restore:
        record = json.loads((root / 'result.json').read_text())
        if PROFILE.read_text() not in (record['before'], record['after']): raise RuntimeError('Profile changed since this experiment; refusing overwrite')
        for address in record['added']:
            if address in sources(): run('chronyc', 'delete', address)
        write_profile(record['before'], record['metadata'])
        record['status'] = 'restored'
    else:
        root.mkdir(parents=True, exist_ok=False)
        text = PROFILE.read_text()
        after = candidate(text)
        (root / 'candidate.conf').write_text(after)
        stat = PROFILE.stat()
        record = {'status': 'staged', 'before': text, 'after': after,
                  'metadata': {'uid': stat.st_uid, 'gid': stat.st_gid, 'mode': stat.st_mode & 0o777},
                  'added': [], 'tracking_before': run('chronyc', 'tracking'),
                  'sources_before': run('chronyc', '-n', 'sources'),
                  'limits': 'Source expansion only; convergence, reboot and source-loss qualification remain separate.'}
        def save(): (root / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
        save()
        if args.apply:
            with open('/run/lock/dynamo-clock-sources.lock', 'w') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                if PROFILE.read_text() != text: raise RuntimeError('Profile changed while staging')
                # Ubuntu's chronyd confinement permits configs under /etc/chrony.
                fd, validation = tempfile.mkstemp(dir=PROFILE.parent, prefix='.dynamo-validate-')
                try:
                    with os.fdopen(fd, 'w') as stream: stream.write(after)
                    os.chmod(validation, 0o644)
                    run('chronyd', '-p', '-f', validation)
                finally:
                    os.unlink(validation)
                existing = sources()
                addresses = sorted({item[4][0] for item in socket.getaddrinfo('ntp.ubuntu.com', 123, socket.AF_INET, socket.SOCK_DGRAM)})
                if len(addresses) < 2: raise RuntimeError('DNS supplied fewer than two Ubuntu sources')
                record['status'] = 'applying'; save()
                try:
                    for address in addresses:
                        if address not in existing:
                            # Save intent before mutation so interrupted runs retain recovery evidence.
                            record['added'].append(address); save()
                            run('chronyc', 'add', 'server', address, 'iburst')
                    write_profile(after, record['metadata'])
                    record['status'] = 'installed-awaiting-clock-qualification'
                except Exception:
                    for address in record['added']:
                        if address in sources(): run('chronyc', 'delete', address)
                    write_profile(text, record['metadata'])
                    record['status'] = 'failed-restored'; save()
                    raise
                record['tracking_after'] = run('chronyc', 'tracking')
                record['sources_after'] = run('chronyc', '-n', 'sources')
    (root / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({'status': record['status'], 'result': str(root / 'result.json')}))


if __name__ == '__main__': main()
