#!/usr/bin/env python3
"""Capture passive motor CAN during one existing bounded forward trial.

The selected observer owns the physical-stop and motion gate. This wrapper
starts candump before that gate and always stops it afterward. It never sends
CAN frames or publishes ROS messages itself.
"""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
OBSERVER = Path(__file__).with_name('observe_driving_stop.py')
PULSE = Path(__file__).with_name('commissioning_forward_pulse.py')
HANDOVER = Path(__file__).with_name('check_live_driving_handover.py')
REPLAY = Path(__file__).with_name('replay_puma_freshness.py')


def command_for_observer(run_dir, source='joystick'):
    if source == 'pulse':
        return [sys.executable, str(PULSE), '--execute', '--operator-confirmed',
                '--test-interlock', '--axis', 'forward', '--stop-mode', 'timeout',
                '--ramp', '--run-dir', str(run_dir)]
    if source == 'manual-pulse':
        return [sys.executable, str(PULSE), '--execute', '--operator-confirmed',
                '--original-manual-speeds', '--axis', 'forward', '--stop-mode', 'zero',
                '--ramp', '--run-dir', str(run_dir)]
    if source == 'handover':
        return [sys.executable, str(HANDOVER), '--operator-confirmed',
                '--bounded-motion-confirmed', '--run-dir', str(run_dir)]
    if source != 'joystick':
        raise ValueError('unknown bounded trial source')
    return [sys.executable, str(OBSERVER), '--test', 'interlock-permit',
            '--direction', 'forward', '--test-interlock', '--operator-confirmed',
            '--permit-motion-confirmed', '--run-dir', str(run_dir)]


def stop_process(child):
    if child is None or child.poll() is not None:
        return
    child.send_signal(signal.SIGINT)
    try:
        child.wait(timeout=5)
    except subprocess.TimeoutExpired:
        child.terminate()
        try:
            child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=3)


def wait_for_capture(child, capture, timeout=3.):
    """Do not arm the observer on an empty or failed CAN capture."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if child.poll() is not None:
            raise RuntimeError('passive CAN capture exited; no motion trial launched')
        if capture.stat().st_size:
            return
        time.sleep(.05)
    raise RuntimeError('no CAN frames recorded; no motion trial launched')


def capture_observer(root, source='joystick'):
    capture = root/'passive-can.log'
    candump = None
    observer = None
    observer_rc = None
    capture_died_early = False
    try:
        with capture.open('w') as output, (root/'candump.stderr.log').open('w') as errors:
            candump = subprocess.Popen(['candump', '-L', '-x', 'vcan0'], stdout=output,
                                       stderr=errors, start_new_session=True)
            wait_for_capture(candump, capture)
            print('CAN RECORDING READY. Run the one bounded forward trial using the prompts below.', flush=True)
            observer = subprocess.Popen(command_for_observer(root/'observer', source),
                                        start_new_session=True)
            while observer.poll() is None:
                if candump.poll() is not None:
                    capture_died_early = True
                    print('CAN capture exited. PRESS PHYSICAL E-STOP NOW; cancelling the observer.', flush=True)
                    stop_process(observer)
                    break
                time.sleep(.05)
            observer_rc = observer.wait()
    except KeyboardInterrupt:
        print('Interrupted. Press and leave the physical e-stop engaged.', flush=True)
        observer_rc = 130
    finally:
        stop_process(observer)
        stop_process(candump)
    return observer_rc, capture_died_early


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--bounded-motion-confirmed', action='store_true')
    parser.add_argument('--source', choices=('joystick', 'pulse', 'manual-pulse', 'handover'), default='joystick',
                        help='handover uses two low-speed forward sources through the installed manual-mode mux and guard')
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    if not args.operator_confirmed or not args.bounded_motion_confirmed:
        parser.error('operator presence and one bounded motion trial must be confirmed')
    root = args.run_dir.resolve()
    evidence = ROOT/'artifacts/hardware'
    if root == evidence or not root.is_relative_to(evidence) or root.exists():
        parser.error('use a fresh run directory below artifacts/hardware')
    root.mkdir(parents=True)
    capture = root/'passive-can.log'
    observer_dir = root/'observer'
    started = time.time()
    observer_rc, capture_died_early = capture_observer(root, args.source)

    observer_result = observer_dir/'result.json'
    record = {'status': 'captured-not-automatically-qualified',
              'observer_exit_code': observer_rc,
              'observer_result_exists': observer_result.exists(),
              'capture_died_early': capture_died_early,
              'capture_duration_wall_s': time.time()-started,
              'capture_bytes': capture.stat().st_size,
              'capture_sha256': hashlib.sha256(capture.read_bytes()).hexdigest(),
              'trial_source': args.source,
              'observer_command': command_for_observer(observer_dir, args.source),
              'limits': 'One bounded floor trial; operator must confirm motion and stopping. '
                        'CAN timestamps are host receive times, not physical wheel synchronization.'}
    (root/'capture-context.json').write_text(json.dumps(record, indent=2)+'\n')
    if capture.stat().st_size:
        replay = subprocess.run([sys.executable, str(REPLAY), '--capture', str(capture),
                                 '--output', str(root/'replay.json')], check=False)
        record['replay_exit_code'] = replay.returncode
        (root/'capture-context.json').write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record, indent=2), flush=True)
    return 0 if observer_rc == 0 and not capture_died_early and record.get('replay_exit_code') == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
