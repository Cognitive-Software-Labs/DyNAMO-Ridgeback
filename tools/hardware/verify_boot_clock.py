#!/usr/bin/env python3
"""Check the boot clock step on the current boot (S6). Read-only.

Usage:
  python3 tools/hardware/verify_boot_clock.py --record-before RUN_DIR   # before rebooting
  python3 tools/hardware/verify_boot_clock.py --verify RUN_DIR          # after the boot

--verify passes when the boot ID changed since --record-before;
dynamo-boot-clock.service succeeded and logged its decision; every consumer
started after it finished; chrony now has an external source within 5 ms; and
clearpath-platform is active. It reports how long the platform's own clock gate
took, which was up to 15 minutes before the boot step.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

UNIT = 'dynamo-boot-clock.service'
CONSUMERS = ('clearpath-robot.service', 'clearpath-sensors.service', 'clearpath-platform.service',
             'dynamo-camera.service', 'dynamo-ptp4l.service', 'dynamo-phc2sys.service')
MAX_OFFSET_S = 0.005
BOOT_ID = Path('/proc/sys/kernel/random/boot_id')


def show(unit, *props):
    out = subprocess.run(['systemctl', 'show', unit, *[f'-p{p}' for p in props]],
                         capture_output=True, text=True).stdout
    return dict(line.split('=', 1) for line in out.splitlines() if '=' in line)


def micros(value):
    return int(value) if value and value.isdigit() else None


def evaluate(before_boot, boot, unit, consumers, decision, tracking, platform_active):
    """Rows of (criterion, passed, detail); all inputs already read from the host."""

    rows = [('boot changed', before_boot is not None and before_boot != boot, f'{before_boot} -> {boot}')]
    done = micros(unit.get('ActiveEnterTimestampMonotonic'))
    rows.append(('boot step succeeded', unit.get('Result') == 'success' and done not in (None, 0),
                 f"result {unit.get('Result')}, finished {done / 1e6 if done else '-'} s after boot"))
    rows.append(('boot step logged its decision', bool(decision), decision or 'no decision line'))
    for name, props in consumers.items():
        started = micros(props.get('InactiveExitTimestampMonotonic'))
        rows.append((f'{name} after the step', started not in (None, 0) and done not in (None, 0)
                     and started >= done, f"started {started / 1e6 if started else '-'} s"))
    external = tracking is not None and tracking['reference'] not in ('7F7F0101', '00000000')
    rows.append(('chrony external source within 5 ms', external and abs(tracking['offset_s']) <= MAX_OFFSET_S,
                 f"{tracking['name']} offset {tracking['offset_s'] * 1000:+.3f} ms" if tracking else 'no tracking'))
    rows.append(('clearpath-platform active', platform_active, ''))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--record-before', type=Path)
    mode.add_argument('--verify', type=Path)
    args = parser.parse_args()
    boot = BOOT_ID.read_text().strip()
    if args.record_before:
        args.record_before.mkdir(parents=True, exist_ok=True)
        (args.record_before / 'before.json').write_text(json.dumps({'boot_id': boot}))
        print(f'Recorded boot {boot}; reboot with the robot e-stopped, then run --verify.')
        return 0
    run = args.verify
    before = json.loads((run / 'before.json').read_text())['boot_id'] if (run / 'before.json').exists() else None
    unit = show(UNIT, 'Result', 'ActiveEnterTimestampMonotonic', 'InactiveExitTimestampMonotonic')
    consumers = {name: show(name, 'InactiveExitTimestampMonotonic', 'ActiveEnterTimestampMonotonic')
                 for name in CONSUMERS}
    journal = subprocess.run(['journalctl', '-b', '-u', UNIT, '--no-pager', '-o', 'short-monotonic'],
                             capture_output=True, text=True).stdout
    decision = next((line.split(': ', 1)[-1] for line in journal.splitlines()
                     if 'offset' in line or 'No external time source' in line), None)
    sys.path.insert(0, '/usr/local/lib/dynamo-time')  # the installed copy
    from boot_clock_step import parse_tracking  # noqa: E402
    csv = subprocess.run(['chronyc', '-c', 'tracking'], capture_output=True, text=True).stdout
    tracking = parse_tracking(csv) if csv.strip() else None
    platform_active = subprocess.run(['systemctl', 'is-active', '--quiet', 'clearpath-platform']).returncode == 0
    gate = show('clearpath-platform.service', 'ExecMainStartTimestampMonotonic', 'InactiveExitTimestampMonotonic')
    rows = evaluate(before, boot, unit, consumers, decision, tracking, platform_active)
    for name, passed, detail in rows:
        print(f"{'PASS' if passed else 'FAIL'}  {name:42} {detail}")
    gate_s = None
    if micros(gate.get('ExecMainStartTimestampMonotonic')) and micros(gate.get('InactiveExitTimestampMonotonic')):
        gate_s = (int(gate['ExecMainStartTimestampMonotonic']) - int(gate['InactiveExitTimestampMonotonic'])) / 1e6
        print(f'INFO  platform clock gate (pre-start) took {gate_s:.1f} s')
    passed = all(ok for _, ok, _ in rows)
    print('BOOT CLOCK PASS' if passed else 'BOOT CLOCK FAIL')
    (run / f'verify-{boot}.json').write_text(json.dumps({
        'boot_id': boot, 'before': before, 'decision': decision, 'tracking': tracking,
        'platform_gate_s': gate_s, 'unit': unit, 'consumers': consumers,
        'rows': [{'criterion': n, 'passed': p, 'detail': d} for n, p, d in rows], 'passed': passed}, indent=1))
    (run / f'boot-clock-{boot}.journal').write_text(journal)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
