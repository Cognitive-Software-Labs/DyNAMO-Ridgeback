#!/usr/bin/env python3
"""D3: record what a held controller command does when its radio fades. Read-only.

Usage, with the physical red e-stop PRESSED for the whole run:
  python3 tools/hardware/observe_silent_controller_loss.py --operator-confirmed --run-dir DIR

One operator stays beside the robot with its physical stop pressed. A second
operator holds L1 (deadman) with the left stick pushed forward, then walks
away with the controller until its Bluetooth link degrades or drops, keeping
both held. Clearpath's ds4drv zeroes the input once the report rate falls below
40 %; joy_linux otherwise repeats the last state at 20 Hz. The recorder logs,
at every stage, when the held forward command was last seen: the joystick
(joy_teleop/joy), the teleop output, the combined output, the mux output
(platform/cmd_vel) and the guard output (platform/cmd_vel_limited), together
with the Bluetooth link state. It never publishes, connects or disconnects.

Pass: after the held command starts, fresh zero/neutral reports arrive within
FADE_LIMIT_S of the last held joystick report, the link drop is observed, and
the physical stop stays pressed. Mux and guard outputs must remain zero.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

CONTROLLERS = Path('/home/robot/controllers.txt')
ROOT = Path(__file__).resolve().parents[2]
FADE_LIMIT_S = 1.5          # teleop deadman + 0.5 s mux timeout, with margin
LINK_GRACE_S = 0.5          # one ds4drv signal-check interval
STOP_FRESH_S = 0.5
SETTLE_TAIL_S = 0.25
STAGES = ('joy_teleop/cmd_vel', 'joy_teleop/cmd_vel_combined', 'platform/cmd_vel', 'platform/cmd_vel_limited')


def held_forward(axes, buttons):
    return (len(axes) > 1 and len(buttons) > 4 and buttons[4] == 1
            and math.isfinite(axes[1]) and axes[1] > 0.3)


def nonzero(twist):
    return any(abs(v) > 1e-4 for v in (twist.linear.x, twist.linear.y, twist.angular.z))


def parse_connected(completed):
    """Return True/False only for a successful, explicit bluetoothctl report."""
    if completed.returncode != 0:
        return None
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip().startswith('Connected:')]
    if lines == ['Connected: yes']:
        return True
    if lines == ['Connected: no']:
        return False
    return None


def evaluate(events, end):
    """Summary and pass/fail from (t, stage, active) events; t is seconds since start.

    Pass: a held forward command was seen; the joystick stopped reporting it no
    later than LINK_GRACE_S after the link dropped (ds4drv zeroes faded input);
    every command stage was zero within FADE_LIMIT_S of the last held sample;
    the physical stop stayed pressed.
    """

    def times(stage, active=True):
        return [t for t, s, a in events if s == stage and a == active]

    held = times('joy_held')
    neutral = times('joy_held', False)
    connected = times('link')
    drop = times('link', False)
    report = {'held_start_s': min(held) if held else None, 'held_last_s': max(held) if held else None,
              'link_drop_s': min(drop) if drop else None, 'stop_released': bool(times('stop', False))}
    loss_at = max(held) if held else None
    drop_after_held = next((t for t in drop if held and t > min(held)), None)
    report['link_drop_s'] = drop_after_held
    report['neutral_after_loss_s'] = next((t for t in neutral if loss_at is not None and t > loss_at), None)
    stages = {}
    for stage in STAGES:
        active = times(stage)
        last = max(active) if active else None
        zeros = [t for t in times(stage, False) if loss_at is not None and t > loss_at]
        first_zero = next((t for t in zeros if last is None or t > last), None)
        stages[stage] = {'last_active_s': last,
                         'after_last_held_s': None if last is None or not held else round(last - loss_at, 3),
                         'first_zero_after_loss_s': first_zero,
                         'zero_within_limit': first_zero is not None and first_zero - loss_at <= FADE_LIMIT_S}
    report['stages'] = stages
    report['source_command_seen'] = all(
        any(min(held) <= t <= loss_at + .1 for t in times(stage))
        for stage in STAGES[:2]) if held else False
    report['stop_suppressed_mux'] = all(
        stages[stage]['last_active_s'] is None for stage in STAGES[2:])
    stop_samples = sorted((t, active) for t, stage, active in events if stage == 'stop')
    report['stop_fresh_throughout'] = bool(
        stop_samples and all(active for _, active in stop_samples)
        and stop_samples[-1][0] >= end - STOP_FRESH_S
        and (not held or stop_samples[0][0] <= min(held)))
    final_evidence = [report['neutral_after_loss_s']]
    final_evidence.extend(value['first_zero_after_loss_s'] for value in stages.values())
    report['settled_tail_observed'] = bool(
        all(value is not None for value in final_evidence)
        and end - max(final_evidence) >= SETTLE_TAIL_S)
    report['passed'] = bool(
        held and connected and drop_after_held is not None and not report['stop_released']
        and min(connected) < min(held)
        and -LINK_GRACE_S <= loss_at - drop_after_held <= FADE_LIMIT_S
        and report['source_command_seen'] and report['stop_suppressed_mux']
        and report['stop_fresh_throughout'] and report['settled_tail_observed']
        and report['neutral_after_loss_s'] is not None
        and report['neutral_after_loss_s'] - loss_at <= FADE_LIMIT_S
        and all(v['zero_within_limit'] and
                (v['after_last_held_s'] is None or v['after_last_held_s'] <= FADE_LIMIT_S)
                for v in stages.values()))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--namespace', default='r100_0160')
    parser.add_argument('--duration', type=float, default=120.0)
    args = parser.parse_args()
    if not args.operator_confirmed:
        parser.error('operator presence with the physical stop pressed must be confirmed')
    args.run_dir = args.run_dir.resolve()
    if (args.run_dir.exists() or args.run_dir == ROOT / 'artifacts/hardware'
            or not args.run_dir.is_relative_to(ROOT / 'artifacts/hardware')):
        parser.error('use a fresh run directory below artifacts/hardware')
    args.run_dir.mkdir(parents=True)
    mac = CONTROLLERS.read_text().split()[0]

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool

    rclpy.init()
    node = rclpy.create_node('d3_silent_loss_observer', namespace=args.namespace)
    t0 = time.monotonic()
    events, lock = [], threading.Lock()
    state = {'stop': None, 'last_joy': None, 'held': False, 'link': None}
    out = open(args.run_dir / 'trace.jsonl', 'w')

    def log(stage, active, **extra):
        t = round(time.monotonic() - t0, 3)
        with lock:
            events.append((t, stage, active))
            out.write(json.dumps({'t': t, 'stage': stage, 'active': active, **extra}) + '\n')

    def on_stop(msg):
        log('stop', bool(msg.data))
        state['stop'] = msg.data

    def on_joy(msg):
        key = (tuple(round(a, 3) for a in msg.axes), tuple(msg.buttons))
        if key != state['last_joy']:
            log('joy_change', True, axes=list(key[0]), buttons=list(key[1]))
            state['last_joy'] = key
        held = held_forward(msg.axes, msg.buttons)
        log('joy_held', held)
        if held != state['held']:
            print(f"{time.monotonic() - t0:7.2f} s  joystick {'HELD FORWARD' if held else 'neutral/released'}", flush=True)
            state['held'] = held

    node.create_subscription(Bool, 'platform/emergency_stop', on_stop, qos_profile_sensor_data)
    node.create_subscription(Joy, 'joy_teleop/joy', on_joy, qos_profile_sensor_data)
    for stage in STAGES:
        node.create_subscription(TwistStamped, stage,
                                 lambda m, stage=stage: log(stage, nonzero(m.twist)), qos_profile_sensor_data)

    def poll_link():
        while time.monotonic() - t0 < args.duration:
            try:
                completed = subprocess.run(['/usr/bin/bluetoothctl', '--timeout', '2', 'info', mac],
                                           capture_output=True, text=True,
                                           stdin=subprocess.DEVNULL, timeout=5)
            except subprocess.TimeoutExpired:
                log('link_poll_ok', False, error='timeout')
                time.sleep(0.5)
                continue
            connected = parse_connected(completed)
            if connected is None:
                log('link_poll_ok', False, returncode=completed.returncode)
                time.sleep(0.5)
                continue
            log('link_poll_ok', True)
            if connected != state['link']:
                log('link', connected)
                print(f"{time.monotonic() - t0:7.2f} s  controller link {'CONNECTED' if connected else 'DROPPED'}", flush=True)
                state['link'] = connected
            time.sleep(0.5)

    threading.Thread(target=poll_link, daemon=True).start()
    print('Checking the physical stop is pressed...', flush=True)
    deadline = time.monotonic() + 5
    while state['stop'] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if state['stop'] is not True:
        print('ABORT: the physical red e-stop must be pressed for this whole test.', flush=True)
        out.close()
        node.destroy_node()
        rclpy.shutdown()
        report = {'status': 'aborted', 'passed': False,
                  'reason': 'fresh pressed physical-stop evidence unavailable'}
        (args.run_dir / 'result.json').write_text(json.dumps(report, indent=1) + '\n')
        return 2
    print('READY. One operator stays at the robot with the red e-stop PRESSED. The second holds\n'
          'L1 and pushes the left stick FORWARD, then walks away with both held until the link\n'
          'drops (watch this log). Then come back,\n'
          'release everything and wait for the end.', flush=True)
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if state['stop'] is False:
            print('ABORT: physical stop released.', flush=True)
            break
    end = time.monotonic() - t0
    node.destroy_node()
    rclpy.shutdown()
    out.close()
    report = evaluate(events, end)
    report['status'] = 'passed' if report['passed'] else 'inconclusive'
    (args.run_dir / 'result.json').write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps(report, indent=1))
    print('D3 SILENT LOSS PASS' if report['passed'] else 'D3 SILENT LOSS NOT PASSED')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
