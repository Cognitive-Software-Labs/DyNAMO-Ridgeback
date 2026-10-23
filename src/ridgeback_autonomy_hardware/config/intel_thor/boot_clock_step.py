#!/usr/bin/env python3
"""Step Intel's wall clock once at boot, before anything stamps with it.

The RTC keeps whole seconds, so Intel boots up to a second off, and chrony's
`makestep 1 3` slews anything below a second at `maxslewrate 500`: about eleven
minutes for 350 ms, while sensors, PTP and the platform clock gate wait or stamp
through it. dynamo-boot-clock.service runs this before those consumers start.
It waits for chrony to select an external source, then steps once if the
remaining offset is above the limit. Without a source it exits 0 and leaves the
clock to holdover, so a time-source loss never blocks boot; the platform's own
clock gate still decides whether the base may start. It refuses to step while
any consumer is already active.
"""
import argparse
import subprocess
import sys
import time

CONSUMERS = ('clearpath-robot.service', 'clearpath-sensors.service', 'clearpath-platform.service',
             'dynamo-camera.service', 'dynamo-ptp4l.service', 'dynamo-phc2sys.service')
LOCAL_REFERENCES = ('7F7F0101', '00000000')  # chrony's local stratum, or no reference


def parse_tracking(csv):
    """Reference ID, stratum, system-time offset (s) and leap status from `chronyc -c tracking`."""

    fields = csv.strip().split(',')
    return {'reference': fields[0], 'name': fields[1], 'stratum': int(fields[2]),
            'offset_s': float(fields[4]), 'leap': fields[-1]}


def external_source(tracking):
    return (tracking['reference'] not in LOCAL_REFERENCES and tracking['leap'] != 'Not synchronised'
            and tracking['stratum'] < 10)


def decide(tracking, active_consumers, max_offset_s):
    """'step', 'none' or 'refuse' for one tracking sample whose source is external."""

    if abs(tracking['offset_s']) <= max_offset_s:
        return 'none'
    return 'refuse' if active_consumers else 'step'


def tracking():
    result = subprocess.run(['chronyc', '-c', 'tracking'], capture_output=True, text=True)
    return parse_tracking(result.stdout) if result.returncode == 0 and result.stdout.strip() else None


def active(units):
    return [unit for unit in units
            if subprocess.run(['systemctl', 'is-active', '--quiet', unit]).returncode == 0]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--wait', type=float, default=60.0, help='seconds to wait for an external source')
    parser.add_argument('--max-offset', type=float, default=0.005, help='step above this many seconds')
    args = parser.parse_args(argv)
    deadline = time.monotonic() + args.wait
    sample = None
    while time.monotonic() < deadline:
        sample = tracking()
        if sample and external_source(sample):
            break
        time.sleep(1.0)
    else:
        print(f'No external time source within {args.wait:.0f} s; continuing in holdover '
              f'({sample}).', flush=True)
        return 0
    consumers = active(CONSUMERS)
    action = decide(sample, consumers, args.max_offset)
    print(f"Source {sample['name']} stratum {sample['stratum']}, offset {sample['offset_s'] * 1000:+.3f} ms: "
          f'{action}' + (f' (active: {", ".join(consumers)})' if action == 'refuse' else ''), flush=True)
    if action == 'step':
        subprocess.run(['chronyc', 'makestep'], check=True)
        time.sleep(1.0)
        after = tracking()
        print(f"After step: offset {after['offset_s'] * 1000:+.3f} ms" if after else 'After step: no tracking',
              flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
