#!/usr/bin/env python3
"""Stationary, e-stop-held check of the installed plugin on real feedback loss.

Temporarily pauses only the Puma driver process. An independent watchdog resumes
it within one second even if this observer exits. No movement or permit is sent.
This tests ROS feedback loss, not electrical CAN loss or physical braking.
"""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from install_commissioning_caps import identity
from install_puma_feedback_plugin import running_plugin, load as load_plugin_receipt

ROOT = Path(__file__).resolve().parents[2]
PLUGIN_RECEIPT = ROOT / 'artifacts/hardware/20261013-puma-feedback-install-stage-5'
# The pinned repair, or the per-wheel field-age driver installed over it.
PINNED_DRIVERS = ('/opt/dynamo-puma/', '/opt/dynamo-puma-field/')


def driver_process():
    matches = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdecimal():
            continue
        try:
            exe = (directory / 'exe').resolve()
            group = (directory / 'cgroup').read_text()
            if exe.name == 'multi_puma_node' and 'clearpath-platform.service' in group:
                matches.append((int(directory.name), exe))
        except (OSError, PermissionError):
            continue
    if len(matches) != 1 or not str(matches[0][1]).startswith(PINNED_DRIVERS):
        raise RuntimeError('expected one pinned Puma driver inside clearpath-platform: ' + str(matches))
    return matches[0]


def wait_hardware_processes(timeout=15):
    deadline = time.monotonic() + timeout
    while True:
        try:
            return running_plugin(), driver_process()
        except RuntimeError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(.2)


def guard_resume(pid):
    # A system timer survives terminal/VNC loss and resumes the process in 1 s.
    unit = 'dynamo-feedback-resume-' + uuid.uuid4().hex[:12]
    subprocess.run(['/usr/bin/systemd-run', '--system', '--on-active=1s',
                    '--unit=' + unit, '/bin/kill', '-CONT', str(pid)],
                   check=True, capture_output=True, text=True, timeout=5)
    check = subprocess.run(['/usr/bin/systemctl', 'is-active', unit + '.timer'],
                           capture_output=True, text=True, timeout=2)
    if check.returncode or check.stdout.strip() != 'active':
        raise RuntimeError('independent driver-resume timer is not active')
    return unit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--verify-recovery', action='store_true',
                        help='read-only pressed-stop/active-controller check after a supervised platform restart')
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    if os.geteuid() != 0 or not args.operator_confirmed:
        parser.error('sudo and stationary operator with physical e-stop pressed required')
    if importlib.util.find_spec('rclpy') is None:
        if os.environ.get('DYNAMO_ROS_SETUP_ATTEMPTED'):
            raise RuntimeError('rclpy unavailable after sourcing ROS Jazzy')
        os.environ['DYNAMO_ROS_SETUP_ATTEMPTED'] = '1'
        os.execv('/bin/bash', ['/bin/bash', '-c',
                               'source /opt/ros/jazzy/setup.bash && exec python3 "$@"',
                               'live-feedback-outage', str(Path(__file__).resolve()), *sys.argv[1:]])
    run = args.run_dir.resolve()
    if not run.is_relative_to(ROOT / 'artifacts/hardware') or run.exists():
        parser.error('fresh artifacts/hardware directory required')
    ns = identity()
    _, receipt = load_plugin_receipt(PLUGIN_RECEIPT)
    loaded, (pid, exe) = wait_hardware_processes()
    if receipt['status'] != 'installed-verified-stationary' or loaded['loaded'] != [receipt['prefix'] + '/lib/libpuma_hardware.so']:
        raise RuntimeError('installed feedback plugin is not loaded')
    run.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from sensor_msgs.msg import JointState
    from clearpath_motor_msgs.msg import PumaMultiFeedback
    from geometry_msgs.msg import TwistStamped
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict

    rclpy.init(args=[])
    node = rclpy.create_node('stationary_feedback_outage_observer', namespace=ns,
                             use_global_arguments=False, enable_rosout=False,
                             start_parameter_services=False)
    start = time.monotonic()
    rows = []
    phase = 'preflight'
    report = {'status': 'not-started', 'driver_pid': pid, 'driver_exe': str(exe),
              'plugin': loaded, 'namespace': ns, 'outage_s': 0.0 if args.verify_recovery else 0.8,
              'limits': 'Real ROS feedback interruption while physically stopped; no physical braking or CAN-wire-loss claim.'}
    output = (run / 'samples.jsonl').open('w')

    def sample(topic, message):
        row = {'t': time.monotonic() - start, 'phase': phase, 'topic': topic,
               'message': message_to_ordereddict(message)}
        rows.append(row)
        output.write(json.dumps(row, default=list) + '\n'); output.flush()

    topics = [('platform/emergency_stop', Bool), ('platform/odom', Odometry),
              ('platform/motors/feedback', PumaMultiFeedback),
              ('platform/motors/cmd', JointState),
              ('platform/cmd_vel', TwistStamped),
              ('platform/cmd_vel_limited', TwistStamped)]
    for topic, kind in topics:
        node.create_subscription(kind, topic,
                                 lambda msg, name=topic: sample(name, msg),
                                 qos_profile_sensor_data)
    controller_client = node.create_client(ListControllers, ns + '/controller_manager/list_controllers')

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.02, end-time.monotonic()))
            latest_stop = [r for r in rows if r['topic'] == 'platform/emergency_stop']
            if latest_stop and latest_stop[-1]['message']['data'] is False:
                raise RuntimeError('physical e-stop released; press it immediately')
            if latest_stop and time.monotonic() - start - latest_stop[-1]['t'] > 2.5:
                raise RuntimeError('physical e-stop observation stale; press it immediately')
            for row in rows[-20:]:
                if row['topic'] in ('platform/cmd_vel', 'platform/cmd_vel_limited'):
                    twist = row['message']['twist']
                    values = [twist['linear'][axis] for axis in ('x', 'y')] + [twist['angular']['z']]
                elif row['topic'] == 'platform/motors/cmd':
                    values = row['message']['velocity']
                elif row['topic'] == 'platform/odom':
                    twist = row['message']['twist']['twist']
                    values = [twist['linear'][axis] for axis in ('x', 'y')] + [twist['angular']['z']]
                    if all(math.isfinite(v) and abs(v) <= .001 for v in values):
                        continue
                else:
                    continue
                if any(not math.isfinite(v) or abs(v) > 1e-8 for v in values):
                    raise RuntimeError('nonzero command or wheel motion; press physical e-stop')

    def controller_state():
        if not controller_client.wait_for_service(timeout_sec=1):
            return 'unavailable'
        request = controller_client.call_async(ListControllers.Request())
        deadline = time.monotonic() + 2
        while not request.done() and time.monotonic() < deadline:
            spin(.02)
        if not request.done() or request.result() is None:
            return 'unavailable'
        states = [c.state for c in request.result().controller
                  if c.name == 'platform_velocity_controller']
        return states[0] if len(states) == 1 else 'missing-or-duplicate'

    try:
        print('Checking pressed e-stop, stationary wheels and real feedback for three seconds.', flush=True)
        spin(3)
        stops = [r for r in rows if r['topic'] == 'platform/emergency_stop']
        odom = [r for r in rows if r['topic'] == 'platform/odom']
        feedback = [r for r in rows if r['topic'] == 'platform/motors/feedback']
        commands = [r for r in rows if r['topic'] == 'platform/motors/cmd']
        if (len(stops) < 2 or not all(r['message']['data'] for r in stops[-2:])
                or len(odom) < 20 or len(feedback) < 20 or len(commands) < 20
                or time.monotonic() - start - feedback[-1]['t'] > .15):
            raise RuntimeError('pressed stop, stationary odometry, zero commands or fresh feedback unverified')
        report['controller_before'] = controller_state()
        if report['controller_before'] != 'active':
            raise RuntimeError('platform controller was not active before feedback interruption')
        if args.verify_recovery:
            report.update(status='recovered-stationary',
                          limits='Active controller, real feedback, zero commands, stationary odometry and pressed e-stop after restart; no movement qualification.')
            print(json.dumps(report, indent=2))
            return 0
        report['status'] = 'outage-in-progress'
        (run / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        if driver_process()[0] != pid:
            raise RuntimeError('Puma driver PID changed before trial')
        report['resume_timer'] = guard_resume(pid)
        paused_at = time.monotonic() - start
        phase = 'outage'
        os.kill(pid, signal.SIGSTOP)
        try:
            spin(0.8)
        finally:
            os.kill(pid, signal.SIGCONT)
        phase = 'after-resume'
        spin(1.0)
        outage_commands = [r for r in rows if r['topic'] == 'platform/motors/cmd'
                           and r['t'] >= paused_at + .25 and r['t'] <= paused_at + .5]
        outage_feedback = [r for r in rows if r['topic'] == 'platform/motors/feedback'
                           and r['t'] >= paused_at + .2 and r['t'] <= paused_at + .8]
        report['controller_after'] = controller_state()
        report['driver_same_process'] = driver_process()[0] == pid
        report['controller_manager_same_process'] = running_plugin()['pid'] == loaded['pid']
        report.update(status='stationary-feedback-outage-passed' if
                      (outage_commands and not outage_feedback and report['controller_after'] != 'active'
                       and report['controller_after'] not in ('unavailable', 'missing-or-duplicate')
                       and report['driver_same_process'] and report['controller_manager_same_process']) else
                      'stationary-feedback-outage-inconclusive',
                      paused_at_s=paused_at, zero_commands_after_timeout=len(outage_commands),
                      feedback_during_outage=len(outage_feedback))
    except Exception as error:
        report.update(status='stationary-feedback-outage-failed', error=str(error))
    finally:
        try:
            os.kill(pid, signal.SIGCONT)
        except ProcessLookupError:
            pass
        output.close()
        node.destroy_node(); rclpy.shutdown()
        (run / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'stationary-feedback-outage-passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
