#!/usr/bin/env python3
"""Reconnect configured, paired/trusted controllers without restarting ROS/base services."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import time

MAC = re.compile(r'(?:[0-9A-F]{2}:){5}[0-9A-F]{2}')


def controllers(path):
    devices = []
    for line in path.read_text().splitlines():
        value = line.split('#', 1)[0].strip().upper()
        if not value: continue
        if not MAC.fullmatch(value): raise ValueError('invalid controller address in configuration')
        if value not in devices: devices.append(value)
    return devices


def query(arguments):
    result = subprocess.run(['/usr/bin/bluetoothctl', '--timeout', '3', *arguments],
                            capture_output=True, text=True, stdin=subprocess.DEVNULL, timeout=5)
    return result.returncode, result.stdout


def poll(devices, run=query):
    results = []
    for device in devices:
        try:
            code, info = run(['info', device])
            fields = dict(re.findall(r'^\s*(Paired|Trusted|Connected): (yes|no)\s*$', info, re.M))
            if code or fields.get('Paired') != 'yes' or fields.get('Trusted') != 'yes':
                status = 'manual-pair-and-trust-required'
            elif fields.get('Connected') == 'yes': status = 'connected-link-only'
            else:
                code, _ = run(['connect', device])
                status = 'connect-request-sent' if not code else 'connect-request-failed'
            results.append({'device': device, 'status': status})
        except (OSError, subprocess.SubprocessError) as error:
            results.append({'device': device, 'status': 'query-failed', 'error': type(error).__name__})
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('/home/robot/controllers.txt'))
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args(); previous = None
    while True:
        try: result = poll(controllers(args.config))
        except (OSError, ValueError) as error: result = [{'status': 'configuration-error', 'error': type(error).__name__}]
        if result != previous:
            print(json.dumps(result), flush=True); previous = result
        if args.once: return 0
        time.sleep(5)


if __name__ == '__main__': raise SystemExit(main())
