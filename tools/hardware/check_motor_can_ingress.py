#!/usr/bin/env python3
"""Observe a short incoming motor-CAN/UDP outage with the physical stop pressed.

This drops only packets from the MCU to local UDP port 11412. It sends no
movement or CAN commands. A separate systemd timer removes the rule even if
this process or the operator terminal exits. This is not a moving stop test.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from check_live_feedback_outage import PINNED_DRIVERS, driver_process, wait_hardware_processes
from install_commissioning_caps import identity

ROOT = Path(__file__).resolve().parents[2]
IPTABLES = '/usr/sbin/iptables'
RULE = ['INPUT', '-s', '192.168.131.2', '-d', '192.168.131.1', '-p', 'udp',
        '--dport', '11412', '-m', 'comment', '--comment']


def run_command(argv, timeout=5, check=True):
    return subprocess.run(argv, capture_output=True, text=True, timeout=timeout,
                          check=check)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    if os.geteuid() != 0 or not args.operator_confirmed:
        parser.error('sudo, operator present and physical e-stop pressed required')
    if importlib.util.find_spec('rclpy') is None:
        if os.environ.get('DYNAMO_ROS_SETUP_ATTEMPTED'):
            raise RuntimeError('rclpy unavailable after sourcing ROS Jazzy')
        os.environ['DYNAMO_ROS_SETUP_ATTEMPTED'] = '1'
        os.execv('/bin/bash', ['/bin/bash', '-c',
            'source /opt/ros/jazzy/setup.bash && exec python3 "$@"',
            'motor-can-ingress', str(Path(__file__).resolve()), *sys.argv[1:]])
    out = args.run_dir.resolve()
    if not out.is_relative_to(ROOT / 'artifacts/hardware') or out.exists():
        parser.error('use a fresh directory below artifacts/hardware')
    ns = identity()
    plugin, (driver_pid, driver_exe) = wait_hardware_processes()
    if not str(driver_exe).startswith(PINNED_DRIVERS) or len(plugin['loaded']) != 1:
        raise RuntimeError('pinned Puma driver and feedback plugin required')
    out.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
    import rclpy
    from std_msgs.msg import Bool
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from clearpath_motor_msgs.msg import PumaMultiFeedback
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=[])
    node = rclpy.create_node('motor_can_ingress_observer', namespace=ns,
                             use_global_arguments=False, enable_rosout=False,
                             start_parameter_services=False)
    started = time.monotonic()
    rows = []
    phase = 'baseline'
    raw = (out / 'samples.jsonl').open('w')
    token = 'dynamo-can-ingress-' + uuid.uuid4().hex[:12]
    rule = RULE + [token, '-j', 'DROP']
    timer = None
    installed = False
    report = {'status': 'not-started', 'namespace': ns, 'driver_pid': driver_pid,
              'driver_exe': str(driver_exe), 'plugin': plugin,
              'rule': rule, 'limits': 'Stationary MCU-to-PC UDP/CAN ingress only; no moving stop, motor watchdog or outgoing-link qualification.'}

    def sample(topic, msg):
        row = {'t': time.monotonic() - started, 'phase': phase, 'topic': topic}
        if topic == 'platform/emergency_stop':
            row['data'] = bool(msg.data)
        elif topic == 'platform/odom':
            v = msg.twist.twist
            row['values'] = [v.linear.x, v.linear.y, v.angular.z]
        elif topic == 'platform/motors/cmd':
            row['values'] = list(msg.velocity)
        else:
            row['stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            row['wheels'] = {w.device_name: {'id': w.device_number,
                'speed': w.speed, 'travel': w.travel} for w in msg.drivers_feedback}
        rows.append(row)
        raw.write(json.dumps(row) + '\n'); raw.flush()

    for topic, kind in [('platform/emergency_stop', Bool), ('platform/odom', Odometry),
                        ('platform/motors/cmd', JointState),
                        ('platform/motors/feedback', PumaMultiFeedback)]:
        node.create_subscription(kind, topic, lambda msg, name=topic: sample(name, msg),
                                 qos_profile_sensor_data)

    def observe(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.02, end - time.monotonic()))
            now = time.monotonic() - started
            for topic, age in [('platform/emergency_stop', 2.5)]:
                latest = next((r for r in reversed(rows) if r['topic'] == topic), None)
                if latest is None or now - latest['t'] > age:
                    raise RuntimeError(topic + ' observation stale; keep physical stop pressed')
            if not next(r for r in reversed(rows) if r['topic'] == 'platform/emergency_stop')['data']:
                raise RuntimeError('physical stop released; press it immediately')
            for row in rows[-20:]:
                if row['topic'] in ('platform/odom', 'platform/motors/cmd'):
                    limit = .001 if row['topic'] == 'platform/odom' else 1e-8
                    if any(not math.isfinite(v) or abs(v) > limit for v in row['values']):
                        raise RuntimeError('unexpected wheel motion or motor command; press physical stop')

    try:
        print('Checking pressed stop, stationary wheels and real feedback for three seconds.', flush=True)
        deadline = time.monotonic() + 5
        required = {'platform/emergency_stop', 'platform/odom',
                    'platform/motors/cmd', 'platform/motors/feedback'}
        while required - {r['topic'] for r in rows}:
            if time.monotonic() >= deadline:
                raise RuntimeError('required ROS topics unavailable before preflight')
            rclpy.spin_once(node, timeout_sec=.05)
            if any(r['topic'] == 'platform/emergency_stop' and not r['data'] for r in rows):
                raise RuntimeError('physical stop released before preflight')
        observe(3)
        for topic, minimum in [('platform/emergency_stop', 2), ('platform/odom', 20),
                               ('platform/motors/cmd', 20), ('platform/motors/feedback', 20)]:
            if len([r for r in rows if r['topic'] == topic]) < minimum:
                raise RuntimeError('insufficient baseline ' + topic)
        last_odom = next(r for r in reversed(rows) if r['topic'] == 'platform/odom')
        if time.monotonic() - started - last_odom['t'] > .25:
            raise RuntimeError('baseline wheel odometry is stale')
        if driver_process()[0] != driver_pid:
            raise RuntimeError('driver changed before test')
        if run_command([IPTABLES, '-w', '-C', *rule], check=False).returncode == 0:
            raise RuntimeError('test firewall rule unexpectedly exists')
        timer = token
        run_command(['/usr/bin/systemd-run', '--system', '--on-active=3s',
                     '--unit=' + timer, IPTABLES, '-w', '-D', *rule])
        if run_command(['/usr/bin/systemctl', 'is-active', timer + '.timer'], check=False).stdout.strip() != 'active':
            raise RuntimeError('independent firewall rollback timer did not arm')
        report['rollback_timer'] = timer
        phase = 'blocked-ingress'
        report['blocked_at_s'] = time.monotonic() - started
        run_command([IPTABLES, '-w', '-I', 'INPUT', '1', *rule[1:]])
        installed = True
        observe(.8)
        run_command([IPTABLES, '-w', '-D', *rule]); installed = False
        phase = 'after-restore'
        observe(2)
        report['driver_same_process'] = driver_process()[0] == driver_pid
        report['firewall_rule_removed'] = run_command([IPTABLES, '-w', '-C', *rule], check=False).returncode != 0
        report['feedback_by_phase'] = {name: len([r for r in rows if r['phase'] == name
            and r['topic'] == 'platform/motors/feedback'])
            for name in ('baseline', 'blocked-ingress', 'after-restore')}
        report['status'] = ('stationary-can-ingress-observed' if
            (report['driver_same_process'] and report['firewall_rule_removed']
             and report['feedback_by_phase']['after-restore'] >= 10) else
            'stationary-can-ingress-not-recovered')
    except Exception as exc:
        report.update(status='stationary-can-ingress-failed', error=str(exc))
    finally:
        if installed:
            removed = run_command([IPTABLES, '-w', '-D', *rule], check=False)
            report['manual_rollback_rc'] = removed.returncode
        report['firewall_rule_removed_final'] = run_command([IPTABLES, '-w', '-C', *rule], check=False).returncode != 0
        report['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        raw.close()
        node.destroy_node(); rclpy.shutdown()
        (out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['status'] == 'stationary-can-ingress-observed' and report['firewall_rule_removed_final'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
