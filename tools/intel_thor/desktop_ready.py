#!/usr/bin/env python3
"""Wait for desktop listeners owned by independent services, never start them."""
import argparse
from pathlib import Path
import re
import subprocess
import time

LISTENERS = {5901: 'ridgeback-vnc.service', 9005: 'ridgeback-vnc-websockify.service'}


def listener_identity(port, unit, run=subprocess.run, proc_root=Path('/proc'), timeout=5):
    result = run(['ss', '-H', '-ltnp', f'( sport = :{port} )'],
                 capture_output=True, text=True, check=True, timeout=timeout)
    pids = set(re.findall(r'pid=(\d+)', result.stdout))
    if not pids:
        raise RuntimeError(f'No visible listener on {port}')
    identities = []
    expected = 'Xtigervnc' if port == 5901 else Path('/usr/bin/python3').resolve().name
    for pid in sorted(pids):
        process = proc_root / pid
        if process.joinpath('exe').resolve().name != expected:
            raise RuntimeError(f'Unexpected executable listening on {port}')
        groups = process.joinpath('cgroup').read_text().splitlines()
        if not any(unit in line.split(':', 2)[-1].split('/') for line in groups):
            raise RuntimeError(f'Port {port} is owned outside {unit}')
        fields = process.joinpath('stat').read_text().rsplit(')', 1)[1].split()
        identities.append({'pid': int(pid), 'start_ticks': int(fields[19])})
    if port in (9005, 9100):
        # Websockify forks a worker for each browser connection. Those workers
        # inherit the listening FD and appear in ss; connecting a browser must
        # not be interpreted as restarting the proxy master.
        result = run(['systemctl', 'show', unit, '-p', 'MainPID', '--value'],
                     capture_output=True, text=True, check=True, timeout=timeout)
        master = int(result.stdout.strip())
        identities = [identity for identity in identities if identity['pid'] == master]
        if not identities:
            raise RuntimeError(f'Proxy main process does not own port {port}')
    return identities


def wait_ready(timeout=60):
    deadline = time.monotonic() + timeout
    error = 'Desktop not ready'
    while time.monotonic() < deadline:
        try:
            identities = {}
            for port, unit in LISTENERS.items():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError('Desktop readiness deadline expired')
                identities[str(port)] = listener_identity(port, unit, timeout=min(5, remaining))
            if time.monotonic() >= deadline:
                raise RuntimeError('Desktop readiness deadline expired')
            return identities
        except (OSError, RuntimeError, subprocess.SubprocessError) as failure:
            error = str(failure)
            time.sleep(min(.25, max(0, deadline - time.monotonic())))
    raise RuntimeError(error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=60)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('Timeout must be positive')
    try:
        print(wait_ready(args.timeout), flush=True)
    except Exception as error:
        print(f'Desktop ownership check failed: {error}', flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
