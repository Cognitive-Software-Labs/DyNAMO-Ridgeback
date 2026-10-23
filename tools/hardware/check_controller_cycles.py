#!/usr/bin/env python3
"""Supervised repeated controller disconnect/reconnect with the physical stop held.

Publishes no ROS messages or permits. Disconnects only the configured Bluetooth
controller; the operator reconnects with PS. Never restarts robot services.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

from controller_autoconnect import controllers
from install_commissioning_caps import ROOT, desired_files, identity, run_probe
from observe_driving_stop import held_forward_input, release_sample_safe

WATCHED = {'platform/emergency_stop': 2.5, 'platform/emergency_stop_mux': .25,
           'platform/odom': .25, 'platform/motors/cmd': .25,
           'platform/cmd_vel_limited': .25}


def neutral(message):
    axes, buttons = message.get('axes', []), message.get('buttons', [])
    return (len(axes) > 4 and len(buttons) > 5
            and all(math.isfinite(v) for v in axes)
            and not any(buttons[b] for b in (4, 5))
            and all(abs(axes[a]) <= .05 for a in (0, 1, 3, 4)))


def stationary_problem(latest, now):
    for topic, age in WATCHED.items():
        row = latest.get(topic)
        if row is None or not 0 <= now-row['elapsed'] <= age:
            return 'missing/stale ' + topic
        message = row['message']
        if topic in ('platform/emergency_stop', 'platform/emergency_stop_mux'):
            if message.get('data') is not True:
                return 'physical stop or software lock was released: ' + topic
        elif topic == 'platform/motors/cmd' and len(message.get('velocity', [])) != 4:
            return 'expected four wheel command entries'
        elif not release_sample_safe(topic, message):
            return 'unexpected wheel command or motion: ' + topic
    return None


def bluetooth_state(result):
    if result['returncode'] != 0:
        return None
    fields = {}
    for line in result['stdout'].splitlines():
        key, sep, value = line.strip().partition(': ')
        if sep:
            fields[key] = value.strip()
    return {'yes': True, 'no': False}.get(fields.get('Connected'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--cycles', type=int, choices=range(1, 4), default=3)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    if not args.operator_confirmed:
        parser.error('operator must keep the physical e-stop pressed throughout')
    devices = controllers(Path('/home/robot/controllers.txt'))
    if len(devices) != 1:
        parser.error('exactly one configured controller required')
    device = devices[0]
    root = args.run_dir.resolve()
    evidence_root = ROOT/'artifacts/hardware'
    if root == evidence_root or not root.is_relative_to(evidence_root):
        parser.error('use a fresh directory under artifacts/hardware')
    ns = identity()
    expected = desired_files(ns, True)
    for path, data in expected.items():
        if path.read_bytes() != data:
            parser.error('verified installed test interlock required: ' + str(path))
    root.mkdir(parents=True, exist_ok=False)
    report = {'status': 'preflight', 'namespace': ns, 'device': device,
              'requested_cycles': args.cycles, 'cycles': [],
              'ros_publishers_created': 0, 'service_restarts': 0,
              'installed_sha256': {str(path): hashlib.sha256(data).hexdigest()
                                   for path, data in expected.items()},
              'limits': 'Stationary deliberate Bluetooth disconnect/reconnect only; no moving stop, silent radio loss or physical braking qualification.'}
    for name in ('check_controller_cycles.py', 'observe_driving_stop.py',
                 'commissioning_speed_guard.py', 'controller_autoconnect.py',
                 'install_commissioning_caps.py'):
        source = Path(__file__).with_name(name)
        (root/name).write_bytes(source.read_bytes())
    report['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    node = None
    pool = ThreadPoolExecutor(max_workers=1)
    log = (root/'samples.jsonl').open('w')
    latest = {}
    started = time.monotonic()
    phase = 'preflight'
    cycle = 0
    bt_index = 0
    try:
        print('Checking installed lock and pressed physical stop. KEEP THE STOP PRESSED throughout.', flush=True)
        report['before'] = run_probe(ns, root, 'before')
        if not all(report['before'].get(key) for key in
                   ('routing_ok', 'mux_stop_routing_ok', 'mux_stop_engaged')):
            raise RuntimeError('initial guard/mux routing or lock check failed')
        os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                          CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict
        from std_msgs.msg import Bool
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState, Joy
        from geometry_msgs.msg import TwistStamped
        rclpy.init(args=[])
        node = rclpy.create_node('stationary_controller_cycle_observer', namespace=ns,
                                 enable_rosout=False, start_parameter_services=False)

        def callback(topic, msg):
            row = {'elapsed': time.monotonic()-started, 'topic': topic,
                   'phase': phase, 'cycle': cycle, 'message': message_to_ordereddict(msg)}
            latest[topic] = row
            log.write(json.dumps(row)+'\n'); log.flush()
            if topic in ('platform/emergency_stop', 'platform/emergency_stop_mux'):
                if row['message']['data'] is not True:
                    raise RuntimeError('stop/lock released; PRESS PHYSICAL E-STOP NOW')
            elif topic in WATCHED or topic == 'platform/cmd_vel':
                if topic == 'platform/motors/cmd' and len(row['message'].get('velocity', [])) != 4:
                    raise RuntimeError('expected four wheel command entries')
                if not release_sample_safe(topic, row['message']):
                    raise RuntimeError('unexpected command/motion; PRESS PHYSICAL E-STOP NOW')
            if phase in ('initial-neutral-connect', 'neutral-reconnect') and topic == 'joy_teleop/joy' and not neutral(row['message']):
                raise RuntimeError('non-neutral controls during reconnect; keep physical stop pressed')

        for topic, kind in [('platform/emergency_stop', Bool), ('platform/emergency_stop_mux', Bool),
                            ('platform/odom', Odometry), ('platform/motors/cmd', JointState),
                            ('platform/cmd_vel_limited', TwistStamped), ('platform/cmd_vel', TwistStamped),
                            ('joy_teleop/joy', Joy), ('joy_teleop/cmd_vel_combined', TwistStamped)]:
            node.create_subscription(kind, topic, lambda msg, t=topic: callback(t, msg), qos_profile_sensor_data)

        def elapsed():
            return time.monotonic()-started

        def spin(check=True):
            rclpy.spin_once(node, timeout_sec=.02)
            if check:
                problem = stationary_problem(latest, elapsed())
                if problem:
                    raise RuntimeError(problem + '; keep physical stop pressed')
                if node.get_publishers_info_by_topic(ns+'/platform/commissioning_test_permit'):
                    raise RuntimeError('unexpected test-permit publisher; keep physical stop pressed')

        def wait_until(condition, seconds, failure):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                spin()
                if condition():
                    return
            raise RuntimeError(failure)

        def bt(action):
            nonlocal bt_index
            bt_index += 1
            def invoke():
                try:
                    result = subprocess.run(['/usr/bin/bluetoothctl', '--timeout', '2', action, device],
                                            stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=4)
                    return {'action': action, 'returncode': result.returncode,
                            'stdout': result.stdout, 'stderr': result.stderr}
                except (OSError, subprocess.SubprocessError) as error:
                    return {'action': action, 'returncode': None, 'stdout': '', 'error': str(error)}
            future = pool.submit(invoke)
            while not future.done():
                spin()
            result = future.result()
            result.update(elapsed=elapsed(), cycle=cycle, phase=phase)
            (root/f'bluetooth-{bt_index:03d}.json').write_text(json.dumps(result, indent=2)+'\n')
            return result

        def wait_link(connected, seconds):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                if bluetooth_state(bt('info')) is connected:
                    return
                until = time.monotonic()+.5
                while time.monotonic() < until:
                    spin()
            raise RuntimeError('Bluetooth state did not become ' + ('connected' if connected else 'disconnected'))

        def fresh_joy(predicate):
            row = latest.get('joy_teleop/joy')
            return (row is not None and 0 <= elapsed()-row['elapsed'] <= .5
                    and predicate(row['message']))

        def held_and_source_since(since):
            source = latest.get('joy_teleop/cmd_vel_combined')
            return (fresh_joy(held_forward_input) and source is not None
                    and source['elapsed'] >= since and elapsed()-source['elapsed'] <= .3
                    and source['message']['twist']['linear']['x'] > 0)

        deadline = time.monotonic()+10
        while time.monotonic() < deadline and stationary_problem(latest, elapsed()):
            spin(check=False)
        spin()
        phase = 'initial-neutral-connect'
        print('KEEP PHYSICAL E-STOP PRESSED. Release both enable buttons and centre sticks.\n'
              'If the controller is disconnected, press ONLY PS now. Waiting up to 60 seconds.', flush=True)
        wait_link(True, 60)
        wait_until(lambda: fresh_joy(neutral), 30, 'fresh neutral controller not observed')
        for cycle in range(1, args.cycles+1):
            item = {'cycle': cycle}
            report['active_cycle'] = item
            phase = 'held-before-disconnect'
            mark = elapsed()
            print(f'CYCLE {cycle}/{args.cycles}: STOP STAYS PRESSED. Hold L1 and a small forward input.\n'
                  'The recorder will disconnect the controller; release controls once disconnected.', flush=True)
            wait_until(lambda: held_and_source_since(mark), 60, 'no held input and matching source command')
            item['held_input_s'] = elapsed()
            phase = 'disconnecting'
            item['disconnect_client'] = bt('disconnect')
            wait_link(False, 8)
            item['disconnect_confirmed_s'] = elapsed()
            phase = 'disconnected'
            print('DISCONNECTED CONFIRMED. Keep stop pressed; release and centre controller controls.', flush=True)
            until = time.monotonic()+3
            while time.monotonic() < until:
                spin()
            if bluetooth_state(bt('info')) is not False:
                raise RuntimeError('controller reconnected before the neutral reconnect prompt')
            phase = 'neutral-reconnect'
            print('RECONNECT NOW: press ONLY PS with sticks centred and both enable buttons released.\n'
                  'Keep the ROBOT PHYSICAL E-STOP PRESSED. Do not touch sticks yet.', flush=True)
            wait_link(True, 60)
            item['reconnect_confirmed_s'] = elapsed()
            wait_until(lambda: fresh_joy(neutral) and latest['joy_teleop/joy']['elapsed'] >= item['reconnect_confirmed_s'],
                       8, 'fresh neutral joystick not observed after reconnect')
            phase = 'restored-input'
            mark = elapsed()
            print('NEUTRAL RECONNECT RECORDED. With STOP STILL PRESSED, hold L1 and small forward input once.', flush=True)
            wait_until(lambda: held_and_source_since(mark), 60, 'no restored input after reconnect')
            item['restored_input_s'] = elapsed()
            print('RESTORED INPUT RECORDED. Release L1 and centre both sticks now. Stop stays pressed.', flush=True)
            neutral_since = None
            def neutral_tail():
                nonlocal neutral_since
                if fresh_joy(neutral):
                    if neutral_since is None:
                        neutral_since = time.monotonic()
                    source = latest.get('joy_teleop/cmd_vel_combined')
                    source_quiet = source is None or elapsed()-source['elapsed'] >= .5
                    if source is not None and not source_quiet:
                        twist = source['message']['twist']
                        source_quiet = all(math.isfinite(twist[p][a]) and abs(twist[p][a]) <= 1e-8
                                           for p, a in [('linear', 'x'), ('linear', 'y'), ('angular', 'z')])
                    return time.monotonic()-neutral_since >= 2 and source_quiet
                neutral_since = None
                return False
            wait_until(neutral_tail, 30, 'neutral return not sustained for two seconds')
            if bluetooth_state(bt('info')) is not True:
                raise RuntimeError('controller link disappeared after restored input')
            item['completed_s'] = elapsed()
            report['cycles'].append(item)
            report.pop('active_cycle', None)
            print(f'CYCLE {cycle} COMPLETE: disconnect, neutral reconnect, real input and neutral return recorded.', flush=True)
        phase = 'final'
        report['after'] = run_probe(ns, root, 'after')
        report['status'] = 'stationary-controller-cycles-observed'
        return 0
    except (Exception, KeyboardInterrupt) as error:
        report.update(status='incomplete-stationary-controller-cycles', error=str(error))
        print('STOPPED: '+str(error)+'. Leave physical e-stop pressed; centre/release controls.', flush=True)
        return 1
    finally:
        report['elapsed_s'] = time.monotonic()-started
        report['last_phase'] = phase
        (root/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        log.close()
        pool.shutdown(wait=False)
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        print(json.dumps({'status': report['status'], 'cycles_completed': len(report['cycles']),
                          'result': str(root/'result.json')}, indent=2), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
