#!/usr/bin/env python3
"""D6/D7: bounded motor-link fault during one raised-wheel forward pulse.

  sudo python3 tools/hardware/check_raised_wheel_link_fault.py --fault feedback \\
      --source manual-pulse --operator-confirmed --wheels-raised-confirmed --run-dir artifacts/hardware/<fresh>
  sudo python3 tools/hardware/check_raised_wheel_link_fault.py --recover --operator-confirmed \\
      --run-dir artifacts/hardware/<fresh>

All four wheels must be clear of the floor on rated stands, with a qualified
operator at the physical stop. The unchanged guarded pulse tool is the only
motion source (one ramped 1 s request at the installed 0.05 m/s cap); this tool
never publishes. Once commanded forward motion is seen on the motor command
and every wheel's feedback, it inserts ONE firewall rule on the MCU CAN-over-UDP
bridge (port 11412) for at most --block-s seconds:

  feedback      (D6) drops MCU->PC packets only. The Puma driver keeps running
                and sending commands; the hardware plugin must zero the motor
                command and latch its controller fault from missing feedback.
  command-link  (D7) drops PC->MCU packets only. Commands can no longer reach
                the motors while feedback still arrives; the motor controllers
                must stop the wheels on their own command-loss watchdog.

Before motion, a target-less counting rule must see bridge traffic in the chosen
direction, or the test refuses. A systemd unit started before the rule exists
deletes it after the bounded block, when this process disappears, or after a
hard runtime cap; it does not depend on this terminal. A fault leaves the
platform latched: --recover requires the stop pressed, restarts only
clearpath-platform.service and runs the existing read-only recovery check.
"""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
IPTABLES = '/usr/sbin/iptables'
MCU, PC, PORT = '192.168.131.2', '192.168.131.1', '11412'
PATHS = {'feedback': ['INPUT', '-s', MCU, '-d', PC, '-p', 'udp', '--dport', PORT],
         'command-link': ['OUTPUT', '-s', PC, '-d', MCU, '-p', 'udp', '--dport', PORT]}
MAX_BLOCK_S = 1.5
MOVING_RAD_S, STOPPED_RAD_S = 0.3, 0.05   # 0.05 m/s is about 0.66 rad/s at the wheel
RAMP_S = 0.35                              # pulse ramp is 250 ms
D6_ZERO_LIMIT_S = 0.5                      # 250 ms consumer timeout plus control cycles
D7_STOP_LIMIT_S = 0.5                      # provisional; the motor watchdog is what is measured
SUSTAINED_STOP_S = 0.25
FEEDBACK_GAP_LIMIT_S = 0.15


def motion_started(commands, feedback, now):
    """Commanded forward motion held for RAMP_S and seen on every wheel's feedback."""
    nonzero = [t for t, values in commands if any(abs(v) > 1e-3 for v in values)]
    if not nonzero or now - nonzero[0] < RAMP_S or commands[-1][0] != nonzero[-1]:
        return False
    recent = [speeds for t, speeds in feedback if now - t <= 0.1]
    return bool(recent) and valid_wheels(recent[-1]) and all(abs(s) >= MOVING_RAD_S for s in recent[-1])


def valid_wheels(values):
    return len(values) == 4 and all(math.isfinite(value) for value in values)


def first_sustained_stop(samples, start, duration=SUSTAINED_STOP_S):
    """Return the first stopped sample backed by fresh stopped data for duration."""
    samples = [(t, values) for t, values in samples if t >= start and valid_wheels(values)]
    for index, (candidate, values) in enumerate(samples):
        if not all(abs(value) <= STOPPED_RAD_S for value in values):
            continue
        # Include the first sample at or past the duration: a 20 Hz stream rarely lands
        # exactly on candidate + duration, and requiring that made a pass impossible.
        end = next((j for j in range(index, len(samples)) if samples[j][0] >= candidate + duration), None)
        if end is None:
            continue
        tail = samples[index:end + 1]
        if (all(all(abs(value) <= STOPPED_RAD_S for value in speeds) for _, speeds in tail)
                and all(b[0] - a[0] <= FEEDBACK_GAP_LIMIT_S for a, b in zip(tail, tail[1:]))):
            return candidate
    return None


def evaluate(fault, fault_at, cleared_at, commands, feedback, controller_states, stop_pressed_end,
             source_commands=None, dropped_packets=1):
    """Pass/fail from (t, values) command/feedback series and (t, state) controller samples."""
    after = lambda series, start, end=math.inf: [(t, v) for t, v in series if start <= t < end]
    source_commands = commands if source_commands is None else source_commands
    report = {'fault': fault, 'fault_at_s': fault_at, 'cleared_at_s': cleared_at,
              'stop_pressed_at_end': stop_pressed_end, 'dropped_packets': dropped_packets}
    if fault_at is None or cleared_at is None:
        report.update(passed=False, reason='fault was never injected')
        return report
    zero = [t for t, values in after(commands, fault_at) if all(abs(v) <= 1e-6 for v in values)]
    report['first_zero_command_after_fault_s'] = round(zero[0] - fault_at, 3) if zero else None
    late_nonzero = [t for t, values in after(commands, fault_at + D6_ZERO_LIMIT_S)
                    if any(abs(v) > 1e-6 for v in values)]
    report['nonzero_commands_after_limit'] = len(late_nonzero)
    states = after(controller_states, fault_at)
    report['controller_left_active'] = any(state != 'active' for _, state in states)
    report['controller_reactivated_after_fault'] = any(
        state == 'active' for t, state in states if t > min((t for t, s in states if s != 'active'), default=math.inf))
    malformed = [(t, values) for t, values in feedback if not valid_wheels(values)]
    report['malformed_feedback_samples'] = len(malformed)
    during = after(feedback, fault_at + .1, cleared_at)
    report['feedback_samples_during_fault'] = len(during)
    stopped_at = first_sustained_stop(during, fault_at)
    report['wheels_stopped_after_fault_s'] = round(stopped_at - fault_at, 3) if stopped_at else None
    resumed = after(feedback, cleared_at)
    report['first_feedback_after_clear'] = resumed[0][1] if resumed else None
    post_clear_stop = first_sustained_stop(resumed, cleared_at)
    report['post_clear_stopped_sustained'] = post_clear_stop is not None
    active_source = [(t, values) for t, values in source_commands
                     if any(abs(v) > 1e-4 for v in values)]
    first_zero = zero[0] if zero else None
    report['source_active_after_response'] = bool(first_zero is not None and any(
        t >= first_zero + .1 for t, _ in active_source))
    report['source_active_through_watchdog_stop'] = bool(stopped_at is not None and any(
        t >= stopped_at + .1 for t, _ in active_source))
    feedback_times = [t for t, _ in during]
    report['max_feedback_gap_during_fault_s'] = max(
        (b - a for a, b in zip(feedback_times, feedback_times[1:])), default=None)
    if fault == 'feedback':
        report['automatic_checks_passed'] = bool(
            zero and zero[0] - fault_at <= D6_ZERO_LIMIT_S and not late_nonzero
            and report['controller_left_active'] and not report['controller_reactivated_after_fault']
            and not during and report['source_active_after_response']
            and post_clear_stop is not None and not malformed and dropped_packets > 0 and stop_pressed_end)
    else:
        feedback_fresh = (len(during) >= 3 and report['max_feedback_gap_during_fault_s'] is not None
                          and report['max_feedback_gap_during_fault_s'] <= FEEDBACK_GAP_LIMIT_S)
        no_resume = bool(post_clear_stop is not None and not any(
            any(abs(value) > STOPPED_RAD_S for value in speeds)
            for _, speeds in resumed if valid_wheels(speeds)))
        report['automatic_checks_passed'] = bool(
            stopped_at is not None and stopped_at - fault_at <= D7_STOP_LIMIT_S
            and feedback_fresh and report['source_active_through_watchdog_stop']
            and no_resume and not malformed and dropped_packets > 0 and stop_pressed_end)
    report['passed'] = report['automatic_checks_passed']
    return report


def rule_packet_count(chain, token):
    listing = run([IPTABLES, '-w', '-L', chain, '-v', '-x', '-n']).stdout
    lines = [line for line in listing.splitlines() if token in line]
    return int(lines[0].split()[0]) if len(lines) == 1 else 0


def watchdog_program(rule, owner_pid, block_s):
    """Independent cleanup loop, run by systemd as root."""
    return (
        'import os,subprocess,time\n'
        f'rule={rule!r};owner={owner_pid};block={block_s}\n'
        'present=lambda:subprocess.run(["' + IPTABLES + '","-w","-C",*rule],capture_output=True).returncode==0\n'
        'delete=lambda:subprocess.run(["' + IPTABLES + '","-w","-D",*rule],capture_output=True)\n'
        'seen=None\n'
        'while True:\n'
        '  alive=os.path.exists(f"/proc/{owner}")\n'
        '  if present():\n'
        '    seen=seen or time.monotonic()\n'
        '    if not alive or time.monotonic()-seen>block+0.25:\n'
        '      delete();print("watchdog removed rule",flush=True)\n'
        '  elif not alive:\n'
        '    break\n'
        '  time.sleep(0.02)\n')


def run(argv, check=True, timeout=10):
    return subprocess.run(argv, capture_output=True, text=True, check=check, timeout=timeout)


def count_path(path, token, seconds=1.0):
    """Packets matched by a counting-only rule (no target, so packets are unaffected)."""
    rule = PATHS[path] + ['-m', 'comment', '--comment', token + '-count']
    run([IPTABLES, '-w', '-I', rule[0], '1', *rule[1:]])
    try:
        time.sleep(seconds)
        listing = run([IPTABLES, '-w', '-L', rule[0], '-v', '-x', '-n']).stdout
    finally:
        run([IPTABLES, '-w', '-D', *rule], check=False)
    lines = [line for line in listing.splitlines() if token + '-count' in line]
    return int(lines[0].split()[0]) if len(lines) == 1 else 0


def reexec_with_ros():
    if importlib.util.find_spec('rclpy') is None:
        if os.environ.get('DYNAMO_ROS_SETUP_ATTEMPTED'):
            raise RuntimeError('rclpy unavailable after sourcing ROS Jazzy')
        os.environ['DYNAMO_ROS_SETUP_ATTEMPTED'] = '1'
        os.execv('/bin/bash', ['/bin/bash', '-c', 'source /opt/ros/jazzy/setup.bash && exec python3 "$@"',
                               'link-fault', str(Path(__file__).resolve()), *sys.argv[1:]])


def recover(run_dir):
    """Supervised recovery: stop pressed, restart only the platform, read-only check."""
    from install_commissioning_caps import identity, systemctl
    ns = identity()
    stop = observe_stop(ns)
    if stop is not True:
        raise RuntimeError('press the physical stop before recovery')
    for path in PATHS.values():
        if any(token in line for line in run([IPTABLES, '-w', '-S', path[0]]).stdout.splitlines()
               for token in ('dynamo-link-fault-',)):
            raise RuntimeError('a link-fault rule is still installed; inspect before recovery')
    run_dir.mkdir(parents=True)
    systemctl('restart', 'clearpath-platform.service', timeout=60)
    check = subprocess.run([sys.executable, str(Path(__file__).with_name('check_live_feedback_outage.py')),
                            '--verify-recovery', '--operator-confirmed', '--run-dir', str(run_dir / 'recovery')],
                           capture_output=True, text=True, timeout=180)
    (run_dir / 'recovery.log').write_text(check.stdout + check.stderr)
    print(check.stdout[-2000:])
    return check.returncode


def observe_stop(ns, seconds=2.0):
    os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
    import rclpy
    from std_msgs.msg import Bool
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(args=[])
    node = rclpy.create_node('link_fault_recovery_stop', namespace=ns, use_global_arguments=False,
                             enable_rosout=False, start_parameter_services=False)
    values = []
    node.create_subscription(Bool, 'platform/emergency_stop', lambda m: values.append(bool(m.data)),
                             qos_profile_sensor_data)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    return (len(values) >= 3 and all(values)) or None


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--fault', choices=sorted(PATHS))
    parser.add_argument('--source', choices=('pulse', 'manual-pulse'), default='manual-pulse',
                        help='pulse: installed test interlock; manual-pulse: installed original-manual guard')
    parser.add_argument('--block-s', type=float, default=1.0)
    parser.add_argument('--recover', action='store_true')
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--wheels-raised-confirmed', action='store_true')
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    if os.geteuid() != 0 or not args.operator_confirmed:
        parser.error('sudo and a qualified operator at the physical stop are required')
    if args.recover == bool(args.fault):
        parser.error('choose exactly one of --fault or --recover')
    if args.fault and not args.wheels_raised_confirmed:
        parser.error('all four wheels must be raised on rated stands (--wheels-raised-confirmed)')
    if not 0.3 <= args.block_s <= MAX_BLOCK_S:
        parser.error(f'--block-s must be 0.3..{MAX_BLOCK_S}')
    run_dir = args.run_dir.resolve()
    if not run_dir.is_relative_to(ROOT / 'artifacts/hardware') or run_dir.exists():
        parser.error('use a fresh directory below artifacts/hardware')
    reexec_with_ros()
    if args.recover:
        return recover(run_dir)

    from install_commissioning_caps import identity
    from check_live_feedback_outage import wait_hardware_processes
    from capture_wheel_field_trial import command_for_observer
    ns = identity()
    plugin, (driver_pid, driver_exe) = wait_hardware_processes()
    if not str(driver_exe).startswith('/opt/dynamo-puma-field/'):
        raise RuntimeError('install and accept the per-wheel field-age driver (D5) first')
    token = 'dynamo-link-fault-' + uuid.uuid4().hex[:10]
    rule = PATHS[args.fault] + ['-m', 'comment', '--comment', token, '-j', 'DROP']
    traffic = count_path(args.fault, token)
    if traffic < 20:
        raise RuntimeError(f'only {traffic} bridge packets matched {args.fault} in 1 s; path not verified')
    run_dir.mkdir(parents=True)
    report = {'status': 'not-started', 'fault': args.fault, 'rule': rule, 'path_packets_per_s': traffic,
              'driver_exe': str(driver_exe), 'driver_pid': driver_pid, 'plugin': plugin,
              'block_s': args.block_s, 'source': args.source, 'namespace': ns}
    unit = token + '-watchdog'
    run(['/usr/bin/systemd-run', '--system', '--unit=' + unit, '--collect',
         '--property=RuntimeMaxSec=600', '--property=ExecStopPost=-' + IPTABLES + ' -w -D ' + ' '.join(rule),
         '/usr/bin/python3', '-c', watchdog_program(rule, os.getpid(), args.block_s)])
    if run(['/usr/bin/systemctl', 'is-active', unit + '.service'], check=False).stdout.strip() != 'active':
        raise RuntimeError('independent cleanup watchdog did not start; no fault armed')
    report['watchdog_unit'] = unit

    os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
    import rclpy
    from std_msgs.msg import Bool
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import TwistStamped
    from clearpath_motor_msgs.msg import PumaMultiFeedback
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(args=[])
    node = rclpy.create_node('raised_wheel_link_fault', namespace=ns, use_global_arguments=False,
                             enable_rosout=False, start_parameter_services=False)
    started = time.monotonic()
    clock = lambda: time.monotonic() - started
    commands, source_commands, feedback, controller, stops = [], [], [], [], []
    trace = (run_dir / 'trace.jsonl').open('w')
    lock = threading.Lock()

    def log(kind, **values):
        with lock:
            trace.write(json.dumps({'t': round(clock(), 4), 'kind': kind, **values}) + '\n')

    def on_command(m):
        commands.append((clock(), list(m.velocity)))
        log('cmd', v=list(m.velocity))

    def on_feedback(m):
        speeds = [w.speed for w in m.drivers_feedback]
        feedback.append((clock(), speeds))
        log('feedback', speed=speeds)

    def on_source(m):
        values = [m.twist.linear.x, m.twist.linear.y, m.twist.angular.z]
        source_commands.append((clock(), values))
        log('limited-source', v=values)

    def on_stop(m):
        stops.append((clock(), bool(m.data)))
        log('stop', pressed=bool(m.data))

    node.create_subscription(JointState, 'platform/motors/cmd', on_command, qos_profile_sensor_data)
    node.create_subscription(TwistStamped, 'platform/cmd_vel_limited', on_source, qos_profile_sensor_data)
    node.create_subscription(PumaMultiFeedback, 'platform/motors/feedback', on_feedback, qos_profile_sensor_data)
    node.create_subscription(Bool, 'platform/emergency_stop', on_stop, qos_profile_sensor_data)
    client = node.create_client(ListControllers, ns + '/controller_manager/list_controllers')
    pending, next_poll = None, 0.0

    def poll_controller():
        nonlocal pending, next_poll
        if pending is not None and pending.done():
            state = next((c.state for c in pending.result().controller
                          if c.name == 'platform_velocity_controller'), 'missing')
            controller.append((clock(), state))
            log('controller', state=state)
            pending = None
        if pending is None and clock() >= next_poll and client.service_is_ready():
            pending, next_poll = client.call_async(ListControllers.Request()), clock() + 0.1

    source_dir = run_dir / 'source'
    source_dir.mkdir()
    uid, gid = int(os.environ.get('SUDO_UID', '1000')), int(os.environ.get('SUDO_GID', '1000'))
    os.chown(run_dir, uid, gid)
    os.chown(source_dir, uid, gid)
    command = command_for_observer(source_dir / 'pulse', args.source)
    report['source_command'] = command
    fault_at = cleared_at = None
    dropped_packets = 0
    inserted = False
    source = None
    source_reader = None
    source_log = None
    try:
        print('Wheels raised, operator at the physical stop. Starting the guarded pulse tool;\n'
              'follow its prompts. The fault is armed only after measured forward motion.', flush=True)
        source_log = (run_dir / 'source.log').open('w')
        source = subprocess.Popen(command, user=uid, group=gid, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True, bufsize=1,
                                  env={**os.environ, 'HOME': f'/home/{os.environ.get("SUDO_USER", "robot")}'})

        def relay_source():
            for line in source.stdout:
                source_log.write(line)
                source_log.flush()
                print(line, end='', flush=True)

        source_reader = threading.Thread(target=relay_source, name='bounded-source-log')
        source_reader.start()
        while source.poll() is None or (inserted and cleared_at is None):
            rclpy.spin_once(node, timeout_sec=0.01)
            poll_controller()
            now = clock()
            if fault_at is None and motion_started(commands, feedback, now):
                run([IPTABLES, '-w', '-I', rule[0], '1', *rule[1:]])
                inserted, fault_at = True, clock()
                log('fault-inserted')
                print(f'{fault_at:7.2f} s  FAULT INSERTED ({args.fault})', flush=True)
            if inserted and cleared_at is None and now - fault_at >= args.block_s:
                dropped_packets = rule_packet_count(rule[0], token)
                run([IPTABLES, '-w', '-D', *rule], check=False)
                inserted, cleared_at = False, clock()
                log('fault-cleared')
                print(f'{cleared_at:7.2f} s  fault cleared; PRESS THE PHYSICAL STOP once the wheels stop', flush=True)
            if now > 300:
                raise RuntimeError('test exceeded five minutes; press the physical stop')
        end = time.monotonic() + 3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            poll_controller()
        report['source_returncode'] = source.returncode
        report.update(evaluate(args.fault, fault_at, cleared_at, commands, feedback, controller,
                               bool(stops) and stops[-1][1], source_commands, dropped_packets))
        report['status'] = ('automatic-criteria-passed-awaiting-video-and-recovery'
                            if report['passed'] else 'not-passed')
    except Exception as error:
        report.update(status='aborted', error=str(error), passed=False)
        print('ABORTED: ' + str(error) + ' -- PRESS THE PHYSICAL STOP', flush=True)
    finally:
        if inserted:
            run([IPTABLES, '-w', '-D', *rule], check=False)
        if source is not None and source.poll() is None:
            source.terminate()
            try:
                source.wait(timeout=10)
            except subprocess.TimeoutExpired:
                source.kill()
                source.wait(timeout=5)
        if source_reader is not None:
            source_reader.join(timeout=5)
        if source_log is not None:
            source_log.close()
        report['rule_absent_at_end'] = run([IPTABLES, '-w', '-C', *rule], check=False).returncode != 0
        trace.close()
        node.destroy_node()
        rclpy.shutdown()
        (run_dir / 'result.json').write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('plugin', 'source_command')}, indent=1))
    print(('D6' if args.fault == 'feedback' else 'D7')
          + (' AUTOMATIC CRITERIA PASSED; VIDEO AND RECOVERY STILL REQUIRED'
             if report.get('passed') else ' NOT PASSED')
          + ' -- the platform is latched or unverified: run --recover with the stop pressed')
    return 0 if report.get('passed') and report['rule_absent_at_end'] else 1


if __name__ == '__main__':
    sys.exit(main())
