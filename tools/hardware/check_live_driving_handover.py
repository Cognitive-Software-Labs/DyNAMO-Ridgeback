#!/usr/bin/env python3
"""One supervised, low-speed live mux handover between two bounded sources.

Uses the installed original-manual guard, but publishes only 0.025/0.045 m/s
forward commands. The physical stop and 100 ms guard watchdog remain active.
Terminal completion alone does not qualify physical motion or stopping.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time

from install_commissioning_caps import desired_files, identity
from motor_status_gate import summarize as summarize_motor_status, ready as motor_status_ready, stable_after_release

ROOT = Path(__file__).resolve().parents[2]
MUX_CONFIG = Path('/etc/clearpath/platform/config/twist_mux.yaml')


def policy(namespace):
    import yaml
    config = yaml.safe_load(MUX_CONFIG.read_text())[namespace.lstrip('/')]['twist_mux']['ros__parameters']
    topics = config['topics']
    if not (topics['external']['priority'] < topics['rc']['priority']
            and topics['external']['timeout'] == topics['rc']['timeout'] == .5
            and config['use_stamped']):
        raise RuntimeError('deployed mux priority, timeout or stamped policy differs')
    return topics['external']['topic'], topics['rc']['topic']


def decide(phase, trace):
    limited = [r for r in trace if r['kind'] == 'limited' and r['phase'] == phase]
    return [r['velocity'][0] for r in limited if abs(r['velocity'][0]) > 1e-5]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preflight-only', action='store_true', help='stationary read-only checks; no publisher or motion')
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--bounded-motion-confirmed', action='store_true')
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    if not args.operator_confirmed or (not args.preflight_only and not args.bounded_motion_confirmed):
        parser.error('operator, pressed/reachable physical stop, and bounded motion confirmation required')
    root = args.run_dir.resolve()
    if root == ROOT/'artifacts/hardware' or not root.is_relative_to(ROOT/'artifacts/hardware') or root.exists():
        parser.error('use a fresh evidence directory below artifacts/hardware')
    ns = identity()
    low_topic, high_topic = policy(ns)
    for path, content in desired_files(ns, original_manual_speeds=True).items():
        if path.read_bytes() != content:
            raise RuntimeError('original-manual installed guard differs: ' + str(path))
    root.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool
    from clearpath_motor_msgs.msg import PumaMultiStatus
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=[])
    node = rclpy.create_node('d4_live_handover', namespace=ns, use_global_arguments=False,
                             enable_rosout=False, start_parameter_services=False)
    low = high = None
    started = time.monotonic()
    trace = []
    sent_stamps = set()
    state = {'phase': 'preflight', 'stop': None, 'stop_at': 0., 'odom_at': 0.,
             'odom': (0., 0., 0.), 'joy_at': 0., 'joy_neutral': False,
             'limited_at': 0., 'abort': None, 'motor_status': None,
             'motor_status_at': 0., 'motor_clear_started': None,
             'motor_clear_samples': 0}
    report = {'status': 'not-started', 'namespace': ns, 'sources': {'low': low_topic,
              'high': high_topic}, 'requested_forward_m_s': {'low': .025, 'high': .045},
              'physical_observation': 'awaiting operator',
              'limits': 'One bounded floor handover. ROS commands and raw odometry are not physical stopping distance or certified safety.'}

    def record(kind, **values):
        trace.append({'t': time.monotonic()-started, 'phase': state['phase'],
                      'kind': kind, **values})

    def stop_cb(msg):
        state['stop'], state['stop_at'] = bool(msg.data), time.monotonic()
        record('stop', pressed=state['stop'])

    def odom_cb(msg):
        v = msg.twist.twist
        state['odom'] = (v.linear.x, v.linear.y, v.angular.z)
        state['odom_at'] = time.monotonic()
        record('odom', velocity=state['odom'])
        if any(not math.isfinite(x) for x in state['odom']):
            state['abort'] = 'nonfinite raw odometry'
        elif state['phase'] in ('preflight', 'await-release') and any(abs(x) > .001 for x in state['odom']):
            state['abort'] = 'robot not stationary before trial'
        elif abs(state['odom'][0]) > .075 or abs(state['odom'][1]) > .02 or abs(state['odom'][2]) > .05:
            state['abort'] = 'unexpected odometry during handover'

    def joy_cb(msg):
        state['joy_at'] = time.monotonic()
        state['joy_neutral'] = (len(msg.axes) > 4 and len(msg.buttons) > 5
                                and not msg.buttons[4] and not msg.buttons[5]
                                and all(math.isfinite(msg.axes[i]) and abs(msg.axes[i]) <= .05
                                        for i in (0, 1, 3, 4)))
        if not state['joy_neutral']:
            state['abort'] = 'controller moved or lost neutral state'

    def motor_status_cb(msg):
        now = time.monotonic()
        summary = summarize_motor_status(msg)
        state['motor_status'], state['motor_status_at'] = summary, now
        if motor_status_ready(summary):
            if state['motor_clear_started'] is None:
                state['motor_clear_started'] = now
            state['motor_clear_samples'] += 1
        else:
            state['motor_clear_started'], state['motor_clear_samples'] = None, 0
            if state['phase'] in ('low', 'high_takeover', 'high_expiry_return', 'all_silent'):
                state['abort'] = 'motor status fault or speed mode lost during handover'
        record('motor_status', summary=summary)

    def cmd_cb(msg, kind):
        v = msg.twist
        speed = (v.linear.x, v.linear.y, v.angular.z)
        stamp_ns = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        record(kind, velocity=speed,
               stamp_ns=stamp_ns)
        if kind == 'limited':
            state['limited_at'] = time.monotonic()
        if any(abs(x) > 1e-5 for x in speed) and stamp_ns not in sent_stamps:
            state['abort'] = 'nonzero command from another source'
        if any(not math.isfinite(x) for x in speed) or speed[0] < -1e-5 or speed[0] > .05001 \
                or abs(speed[1]) > 1e-5 or abs(speed[2]) > 1e-5:
            state['abort'] = 'unexpected mux/guard command'
        if state['phase'] in ('preflight', 'await-release') and any(abs(x) > 1e-5 for x in speed):
            state['abort'] = 'nonzero command before trial'

    subscriptions = [
        node.create_subscription(Bool, 'platform/emergency_stop', stop_cb, qos_profile_sensor_data),
        node.create_subscription(Odometry, 'platform/odom', odom_cb, qos_profile_sensor_data),
        node.create_subscription(Joy, 'joy_teleop/joy', joy_cb, qos_profile_sensor_data),
        node.create_subscription(PumaMultiStatus, 'platform/motors/status', motor_status_cb,
                                 qos_profile_sensor_data),
        node.create_subscription(TwistStamped, 'platform/cmd_vel',
                                 lambda m: cmd_cb(m, 'mux'), qos_profile_sensor_data),
        node.create_subscription(TwistStamped, 'platform/cmd_vel_limited',
                                 lambda m: cmd_cb(m, 'limited'), qos_profile_sensor_data),
    ]

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.01, max(0., end-time.monotonic())))
            if state['abort']:
                raise RuntimeError(state['abort'])

    def fresh():
        now = time.monotonic()
        if state['stop'] is not False or now-state['stop_at'] > 2.5:
            raise RuntimeError('physical stop engaged or stale during trial')
        if now-state['odom_at'] > .25 or now-state['limited_at'] > .25:
            raise RuntimeError('wheel or guard observation stale')
        if not state['joy_neutral'] or now-state['joy_at'] > .5:
            raise RuntimeError('controller not fresh and neutral')

    def publish(publisher, speed):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(speed)
        sent_stamps.add(msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec)
        publisher.publish(msg)
        record('sent', speed=speed, source='high' if publisher is high else 'low')

    def phase(name, duration, use_low, use_high):
        state['phase'] = name
        record('phase_start')
        end = time.monotonic()+duration
        while time.monotonic() < end:
            fresh()
            if use_low: publish(low, .025)
            if use_high: publish(high, .045)
            spin(.04)

    def interrupted(signum, frame):
        raise KeyboardInterrupt('interrupted or deadline expired')

    previous = {sig: signal.signal(sig, interrupted)
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGALRM)}
    signal.alarm(120)
    try:
        print('Checking pressed stop, stationary odometry, neutral controller and live routing.', flush=True)
        spin(5)
        if state['stop'] is not True or time.monotonic()-state['stop_at'] > 2.5:
            raise RuntimeError('fresh pressed physical stop required')
        if not state['joy_neutral'] or time.monotonic()-state['joy_at'] > .5:
            raise RuntimeError('fresh neutral controller required')
        if time.monotonic()-state['odom_at'] > .25 or time.monotonic()-state['limited_at'] > .25:
            raise RuntimeError('stationary wheel or guard output stale')
        if state['motor_status'] is None or time.monotonic()-state['motor_status_at'] > 1.5 \
                or not state['motor_status']['complete']:
            raise RuntimeError('fresh four-wheel motor status required')
        if any(abs(x) > .001 for x in state['odom']):
            raise RuntimeError('raw odometry not stationary')
        for topic in (low_topic, high_topic):
            if node.get_publishers_info_by_topic(ns+'/'+topic):
                raise RuntimeError('competing source publisher on '+topic)
            if not any(x.node_name == 'twist_mux' for x in
                       node.get_subscriptions_info_by_topic(ns+'/'+topic)):
                raise RuntimeError('mux missing source subscription: '+topic)
        owners = node.get_publishers_info_by_topic(ns+'/platform/cmd_vel_limited')
        if len(owners) != 1 or owners[0].node_name != 'commissioning_speed_guard':
            raise RuntimeError('limited output not exclusively owned by guard')
        if args.preflight_only:
            report['status'] = 'stationary-preflight-pass'
            print(json.dumps(report, indent=2), flush=True)
            return 0
        low = node.create_publisher(TwistStamped, low_topic, 10)
        high = node.create_publisher(TwistStamped, high_topic, 10)
        spin(.3)
        if not low.get_subscription_count() or not high.get_subscription_count():
            raise RuntimeError('source publisher discovery incomplete')
        state['phase'] = 'await-release'
        print('RECORDING READY: release ONLY the robot red e-stop; leave controller untouched.\n'
              'Wait for PHYSICAL RELEASE RECORDED. Press stop immediately if anything moves.', flush=True)
        deadline = time.monotonic()+60
        while state['stop'] is not False:
            spin(.05)
            if time.monotonic() >= deadline or time.monotonic()-state['stop_at'] > 2.5:
                raise RuntimeError('physical stop release not recorded')
            if not state['joy_neutral'] or time.monotonic()-state['joy_at'] > .5:
                raise RuntimeError('controller changed before release')
        print('PHYSICAL RELEASE RECORDED. Keep the controller untouched and red stop reachable.\n'
              'Waiting for all four motor fault flags to clear before commanding.', flush=True)
        release_at = time.monotonic()
        state['motor_clear_started'], state['motor_clear_samples'] = None, 0
        state['phase'] = 'wait-motor-ready'
        ready_deadline = release_at+20
        while not stable_after_release(state['motor_clear_started'],
                state['motor_clear_samples'], time.monotonic(),
                state['motor_status_at'], release_at):
            spin(.05)
            now = time.monotonic()
            if state['stop'] is not False or now-state['stop_at']>2.5 \
                    or now-state['odom_at']>.25 or any(abs(v)>.005 for v in state['odom']) \
                    or not state['joy_neutral'] or now-state['joy_at']>.5:
                raise RuntimeError('stop, wheel or controller state changed while waiting for motor readiness')
            if now >= ready_deadline:
                raise RuntimeError('motor faults did not clear within 20 seconds; no command sent')
        report['motor_ready_after_release_s'] = time.monotonic()-release_at
        print('MOTORS READY: all four in speed mode with zero faults. '
              'Commanding two short forward sources and observing their handover.', flush=True)
        phase('low', .4, True, False)
        phase('high_takeover', .4, True, True)
        phase('high_expiry_return', .9, True, False)
        phase('all_silent', .5, False, False)
        state['phase'] = 'cleanup'
        for _ in range(10):
            publish(low, 0.); publish(high, 0.); spin(.05)
        if any(abs(x) > .005 for x in state['odom']):
            raise RuntimeError('raw odometry not stopped after handover')
        low_values = decide('low', trace)
        high_values = [r['velocity'][0] for r in trace if r['kind'] == 'limited'
                       and r['phase'] == 'high_takeover'
                       and r['t'] >= next(x['t'] for x in trace if x['phase'] == 'high_takeover') + .15
                       and abs(r['velocity'][0]) > 1e-5]
        expiry_zeros = [r for r in trace if r['phase'] == 'high_expiry_return'
                        and r['kind'] == 'limited' and abs(r['velocity'][0]) <= 1e-5]
        last_high_sent = max(r['t'] for r in trace if r['kind'] == 'sent' and r.get('source') == 'high'
                             and r['speed'] > 0)
        returned = [r for r in trace if r['phase'] == 'high_expiry_return'
                    and r['kind'] == 'limited' and abs(r['velocity'][0] - .025) <= .002]
        returned_at = returned[0]['t'] if returned else None
        return_values = [r['velocity'][0] for r in trace if r['phase'] == 'high_expiry_return'
                         and r['kind'] == 'limited' and returned_at is not None
                         and r['t'] >= returned_at and abs(r['velocity'][0]) > 1e-5]
        report['handover_after_last_high_s'] = returned_at-last_high_sent if returned_at else None
        silent_nonzero = [r for r in trace if r['phase'] == 'all_silent'
                          and r['kind'] == 'limited' and r['t'] >=
                          next(x['t'] for x in trace if x['phase'] == 'all_silent')+.15
                          and abs(r['velocity'][0]) > 1e-5]
        passed = (len(low_values) >= 2 and len(high_values) >= 2 and len(return_values) >= 2
                  and all(abs(v-.025) <= .002 for v in low_values)
                  and all(abs(v-.045) <= .002 for v in high_values)
                  and all(abs(v-.025) <= .002 for v in return_values)
                  and returned_at is not None
                  and .45 <= returned_at-last_high_sent <= .75
                  and any(r['t'] < returned_at for r in expiry_zeros)
                  and not silent_nonzero)
        report['software_handover_observed'] = passed
        report['raw_odom_peak_m_s'] = max((abs(r['velocity'][0]) for r in trace
                                            if r['kind']=='odom'), default=None)
        report['status'] = 'live-handover-observed-awaiting-operator' if passed else 'live-handover-incomplete'
        print('HANDOVER COMPLETE: press and leave the robot red e-stop engaged now.', flush=True)
        deadline = time.monotonic()+30
        while state['stop'] is not True and time.monotonic() < deadline:
            spin(.05)
        if state['stop'] is not True:
            raise RuntimeError('physical stop not re-engaged')
        report['final_physical_stop_pressed'] = True
    except (Exception, KeyboardInterrupt) as error:
        report.update(status='aborted', error=str(error))
        print('ABORT: '+str(error)+'. Press and leave the physical red e-stop engaged.', flush=True)
    finally:
        signal.alarm(0)
        state['phase'] = 'cleanup'
        if low is not None and high is not None:
            for _ in range(20):
                publish(low, 0.); publish(high, 0.)
                rclpy.spin_once(node, timeout_sec=.05)
        report['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (root/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in trace))
        (root/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        del subscriptions
        node.destroy_node(); rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['status'] == 'live-handover-observed-awaiting-operator' else 1


if __name__ == '__main__':
    raise SystemExit(main())
