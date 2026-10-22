#!/usr/bin/env python3
"""One supervised axis pulse through the existing mux and installed guard.

Default is passive. --execute --operator-confirmed authorizes ONE fixed 1 s,
0.05 m/s or 0.1 rad/s peak pulse, followed by zeros. Optional ramp softens
start/end without increasing peak or duration. Never resets/enables services.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time

from install_commissioning_caps import identity, desired_files
from motor_status_gate import summarize as summarize_motor_status, ready as motor_status_ready, stable_after_release

AXES = {'forward': (0, 1), 'backward': (0, -1), 'left': (1, 1),
        'right': (1, -1), 'turn-left': (2, 1), 'turn-right': (2, -1)}


def requested_speed(elapsed, ramp=False, stop_mode='zero', duration=1.0):
    if not 0 <= elapsed < duration:
        return 0.0
    if ramp:
        if stop_mode in ('timeout', 'physical-e-stop'):
            return .05 * min(1.0, elapsed/.25)
        return .05 * min(1.0, elapsed/.25, (duration-elapsed)/.25)
    return .05


def populate_command(message, stamp, speed, axis='forward'):
    """ROS float64 fields require actual Python floats at serialization time."""
    message.header.stamp = stamp
    message.header.frame_id = 'base_link'
    index, sign = AXES[axis]
    values = [0.0, 0.0, 0.0]
    values[index] = float(speed) * sign * (2 if index == 2 else 1)
    message.twist.linear.x, message.twist.linear.y, message.twist.angular.z = values
    return message


def feedback_problem(stop, stop_age, velocity, odom_age, limited_age, axis='forward', rebound_after_peak=None):
    if stop is not False or stop_age > 2.5:
        return 'stop engaged, missing or stale'
    if odom_age > .25 or limited_age > .25:
        return 'odometry or guard output missing/stale'
    if not all(math.isfinite(v) for v in velocity):
        return 'nonfinite odometry'
    # Conservative test abort thresholds; these are not certified stop limits.
    index, sign = AXES[axis]
    reverse_limit = -.02 if index == 2 else -.01
    if rebound_after_peak is not None and index != 2 and rebound_after_peak >= .04:
        # Physical e-stop trial only: an abrupt brake from a measured forward run
        # (run 11 reached 0.05 m/s) can rebound a little before the slow stop report.
        reverse_limit = -.03
    if velocity[index]*sign < reverse_limit:
        return 'unexpected reverse odometry'
    if any(abs(v) > (.05 if i == 2 else .02) for i, v in enumerate(velocity) if i != index):
        return 'unexpected direction/lateral/yaw odometry'
    if velocity[index]*sign > (.15 if index == 2 else .075):
        return 'odometry exceeds test threshold'
    return None


def source_silence_qualifies(motor_nonzero_time, first_zero_time, permit_start,
                            nonzero_resumed):
    return (motor_nonzero_time is not None and first_zero_time is not None
            and permit_start <= motor_nonzero_time < first_zero_time
            and first_zero_time - permit_start < .95 and not nonzero_resumed)


def physical_stop_qualifies(start, stop_at, motor_before, zero_motor_after,
                            zero_limited_after, nonzero_motor_after,
                            nonzero_limited_after, source_after, odom_motion_before):
    """Require the red stop to cut motor references while source input continues."""
    return (stop_at is not None and start < stop_at < start + 2.
            and motor_before is not None and start <= motor_before < stop_at
            and stop_at - motor_before <= .25
            and zero_motor_after is not None and 0 <= zero_motor_after - stop_at <= .5
            and zero_limited_after is not None and 0 <= zero_limited_after - stop_at <= .5
            and not nonzero_motor_after and not nonzero_limited_after
            and source_after >= 2 and odom_motion_before)


def motor_fault_problem(stop_mode, phase, stop_action_time, first_fault_time,
                        now, report_grace_s=1.2):
    """Allow the expected motor-fault/report ordering during a red-stop test.

    The motor status stream can expose the hardware stop fault before the slower
    firmware emergency-stop topic reports the same physical action.  Only the
    dedicated physical-stop trial gets this short grace period; every other
    pulse still aborts immediately on a motor fault.
    """
    if first_fault_time is None or phase != 'pulse':
        return None
    if stop_mode != 'physical-e-stop':
        return 'motor status fault or speed mode lost during pulse'
    if stop_action_time is not None:
        return None
    if now - first_fault_time > report_grace_s:
        return 'motor status fault was not followed by physical-stop report'
    return None


def motor_status_is_estop_fault_onset(summary):
    """Some wheels report the e-stop fault (8) while the rest still report none.

    The fault reaches the four drivers a moment apart, so one status sample can
    show it on only part of the set. Every wheel must still be in speed mode
    with fault 0 or 8; any other fault or mode is a genuine motor problem.
    """
    drivers = summary.get('drivers', [])
    faults = [driver.get('fault') for driver in drivers]
    return (summary.get('complete') is True and len(drivers) == 4
            and all(driver.get('mode') == 2 for driver in drivers)
            and all(fault in (0, 8) for fault in faults) and 8 in faults)


def stop_confirmation_pending(stop_mode, stop_action_time, first_fault_time, now, report_grace_s=1.2):
    """True while the pulse window is over but a seen e-stop fault awaits its slow report.

    The firmware stop topic publishes about once per second, so the report can land
    just after the bounded source window. The source is already silent then
    (requested_speed is zero past the duration); only the observation continues.
    """
    return (stop_mode == 'physical-e-stop' and stop_action_time is None
            and first_fault_time is not None and now - first_fault_time <= report_grace_s)


def motor_status_is_estop_fault(summary):
    drivers = summary.get('drivers', [])
    return (summary.get('complete') is True and len(drivers) == 4
            and all(driver.get('mode') == 2 and driver.get('fault') == 8
                    for driver in drivers))


def physical_stop_tail_deadline(start, duration, stop_at, requested_tail_s=.35):
    """Never extend the nonzero source beyond its original bounded window."""
    return min(start + duration, stop_at + requested_tail_s)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--axis', choices=AXES, default='forward')
    parser.add_argument('--ramp', action='store_true', help='250 ms rise/fall within the fixed one-second pulse')
    parser.add_argument('--stop-mode', choices=['zero', 'timeout', 'physical-e-stop'], default='zero',
                        help='timeout tests source silence; physical-e-stop keeps a bounded source command until the red stop is pressed')
    parser.add_argument('--physical-stop-confirmed', action='store_true',
                        help='confirm one operator will press the physical red stop during the bounded forward pulse')
    parser.add_argument('--test-interlock', action='store_true',
                        help='verify installed one-shot test lock; execute only a half-second forward pulse followed by source silence')
    parser.add_argument('--original-manual-speeds', action='store_true',
                        help='verify installed original-manual guard; require physical stop release for one 1-second 0.05 m/s forward pulse')
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.execute and not args.operator_confirmed:
        parser.error('execution requires an operator, clear floor and reachable physical stop')
    if args.original_manual_speeds and args.test_interlock:
        parser.error('choose one installed guard mode')
    if args.execute and args.test_interlock and (args.axis != 'forward' or args.stop_mode != 'timeout'):
        parser.error('interlocked execution supports only forward publishing silence')
    if args.execute and args.original_manual_speeds and args.axis != 'forward':
        parser.error('original-manual diagnostic execution supports only forward')
    if args.stop_mode == 'physical-e-stop' and not (args.execute and args.operator_confirmed
            and args.original_manual_speeds and args.physical_stop_confirmed
            and args.axis == 'forward' and args.ramp):
        parser.error('physical-e-stop requires an executed, ramped forward original-manual trial and explicit stop confirmation')
    if args.physical_stop_confirmed and args.stop_mode != 'physical-e-stop':
        parser.error('--physical-stop-confirmed requires --stop-mode physical-e-stop')
    root = args.run_dir.resolve()
    evidence = Path(__file__).resolve().parents[2] / 'artifacts/hardware'
    if not root.is_relative_to(evidence) or root == evidence:
        parser.error('run directory must be a fresh directory under artifacts/hardware')
    root.mkdir(parents=True, exist_ok=False)
    ns = identity()
    for path, content in desired_files(ns, args.test_interlock,
                                       original_manual_speeds=args.original_manual_speeds).items():
        if path.read_bytes() != content:
            raise RuntimeError('installed commissioning file differs: ' + str(path))
    os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from controller_manager_msgs.srv import ListControllers
    from clearpath_motor_msgs.msg import PumaMultiFeedback, PumaMultiStatus
    from control_msgs.msg import MecanumDriveControllerState
    from sensor_msgs.msg import JointState, Joy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rclpy.qos import qos_profile_sensor_data

    report = {'namespace': ns, 'mode': 'execute' if args.execute else 'passive',
              'axis': args.axis, 'request_m_s': .05 if AXES[args.axis][0] != 2 else 0,
              'request_yaw_rad_s': .1 if AXES[args.axis][0] == 2 else 0,
              'ramp': args.ramp,
              'stop_mode': args.stop_mode,
              'duration_s': .5 if args.test_interlock else 2 if args.stop_mode == 'physical-e-stop' else 1,
              'test_interlock': args.test_interlock,
              'original_manual_speeds': args.original_manual_speeds,
              'physical_observation': 'awaiting operator',
              'status': 'not-started', 'nonzero_commands': 0}
    rclpy.init(args=[])
    node = rclpy.create_node('commissioning_forward_pulse', namespace=ns,
                             use_global_arguments=False, enable_rosout=False,
                             start_parameter_services=False)
    pub = None
    permit_pub = None
    log = (root / 'samples.jsonl').open('w')
    state = {'stop': None, 'stop_time': 0., 'odom_time': 0., 'limited_time': 0.,
             'velocity': (0., 0., 0.), 'abort': None, 'phase': 'preflight',
             'odom_count': 0, 'stop_count': 0,
             'motor_nonzero_time': None, 'motor_zero_during_silence': None,
             'motor_nonzero_after_silence_zero': False,
             'controller_neutral': False, 'controller_time': 0.,
             'motor_status': None, 'motor_status_time': 0.,
             'motor_clear_started': None, 'motor_clear_samples': 0,
             'motor_fault_during_pulse_time': None}
    state.update(stop_action_time=None, motor_before_stop=None,
                 zero_motor_after_stop=None, zero_limited_after_stop=None,
                 nonzero_motor_after_stop=False, nonzero_limited_after_stop=False,
                 source_after_stop=0, odom_motion_before_stop=False,
                 movement_alert_printed=False)
    sent_stamps = set()
    origin = time.monotonic()

    def record(kind, values):
        log.write(json.dumps({'elapsed': time.monotonic()-origin, 'kind': kind,
                              'phase': state['phase'], 'values': values})+'\n')
        log.flush()

    def stop_callback(msg):
        now = time.monotonic()
        state.update(stop=bool(msg.data), stop_time=now)
        if args.stop_mode == 'physical-e-stop' and state['phase'] == 'pulse' \
                and msg.data and state['stop_action_time'] is None:
            state['stop_action_time'] = now
            state['stop_report_time'] = now
            state['phase'] = 'physical-stop'
            record('physical_stop_action', {
                'report_time': now,
                'motor_fault_time': state['motor_fault_during_pulse_time']})
        state['stop_count'] += 1
        record('emergency_stop', msg.data)

    def odom_callback(msg):
        velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                    msg.twist.twist.angular.z)
        state.update(velocity=velocity, odom_time=time.monotonic())
        if state['phase'] == 'pulse' and state['stop_action_time'] is None \
                and math.isfinite(velocity[0]) and velocity[0] > .005:
            state['odom_motion_before_stop'] = True
            if args.stop_mode == 'physical-e-stop' and not state['movement_alert_printed']:
                state['movement_alert_printed'] = True
                print('\aMOVEMENT DETECTED: PRESS THE RED PHYSICAL E-STOP NOW.', flush=True)
        state['odom_count'] += 1
        record('raw_odom', {'velocity': velocity, 'position': [msg.pose.pose.position.x,
                                                           msg.pose.pose.position.y]})
        if state['phase'] == 'preflight' and any(not math.isfinite(v) or abs(v) > .001 for v in velocity):
            state['abort'] = 'robot not stationary during preflight'

    def command_callback(msg, kind):
        v = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
             msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z)
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        record(kind, {'velocity': v, 'stamp_ns': stamp})
        if kind == 'limited':
            state['limited_time'] = time.monotonic()
            action = state['motor_fault_during_pulse_time'] or state['stop_action_time']
            if action is not None and not any(abs(x) > 1e-8 for x in v):
                state['zero_limited_after_stop'] = state['zero_limited_after_stop'] or time.monotonic()
            elif action is not None:
                state['nonzero_limited_after_stop'] = True
        if any(v):
            if state['phase'] == 'preflight' or stamp not in sent_stamps:
                state['abort'] = 'unexpected competing nonzero command'
            index, sign = AXES[args.axis]
            planar = (v[0], v[1], v[5])
            if (not all(math.isfinite(x) for x in v) or any(v[i] for i in [2, 3, 4])
                    or planar[index]*sign < 0
                    or planar[index]*sign > (.100000001 if index == 2 else .050000001)
                    or any(value for i, value in enumerate(planar) if i != index)):
                state['abort'] = 'unexpected command direction or magnitude'

    def motor_callback(msg):
        record('platform/motors/cmd', message_to_ordereddict(msg))
        values = msg.velocity
        if values and all(math.isfinite(v) for v in values):
            action = state['motor_fault_during_pulse_time'] or state['stop_action_time']
            if any(abs(v) > 1e-8 for v in values):
                state['motor_nonzero_time'] = time.monotonic()
                if action is None:
                    state['motor_before_stop'] = time.monotonic()
                else:
                    state['nonzero_motor_after_stop'] = True
                if state['phase'] == 'timeout' and state['motor_zero_during_silence'] is not None:
                    state['motor_nonzero_after_silence_zero'] = True
            elif action is not None and state['zero_motor_after_stop'] is None:
                state['zero_motor_after_stop'] = time.monotonic()
            elif state['phase'] == 'timeout' and state['motor_nonzero_time'] is not None \
                    and state['motor_zero_during_silence'] is None:
                state['motor_zero_during_silence'] = time.monotonic()

    def joy_callback(msg):
        axes, buttons = msg.axes, msg.buttons
        state['controller_time'] = time.monotonic()
        state['controller_neutral'] = (len(axes) > 4 and len(buttons) > 5
            and all(math.isfinite(v) for v in axes)
            and not buttons[4] and not buttons[5]
            and all(abs(axes[i]) <= .05 for i in (0,1,3,4)))
        record('joy_neutral', state['controller_neutral'])

    def motor_status_callback(msg):
        now = time.monotonic()
        summary = summarize_motor_status(msg)
        state['motor_status'], state['motor_status_time'] = summary, now
        if motor_status_ready(summary):
            if state['motor_clear_started'] is None:
                state['motor_clear_started'] = now
            state['motor_clear_samples'] += 1
        else:
            state['motor_clear_started'], state['motor_clear_samples'] = None, 0
            if state['phase'] == 'pulse':
                if args.stop_mode == 'physical-e-stop' and (
                        motor_status_is_estop_fault(summary) or motor_status_is_estop_fault_onset(summary)):
                    state['motor_fault_during_pulse_time'] = (
                        state['motor_fault_during_pulse_time'] or now)
                else:
                    state['abort'] = 'motor status fault or speed mode lost during pulse'
        record('motor_status', summary)

    node.create_subscription(Bool, 'platform/emergency_stop', stop_callback, qos_profile_sensor_data)
    node.create_subscription(Odometry, 'platform/odom', odom_callback, qos_profile_sensor_data)
    node.create_subscription(JointState, 'platform/motors/cmd', motor_callback, qos_profile_sensor_data)
    node.create_subscription(PumaMultiStatus, 'platform/motors/status', motor_status_callback,
                             qos_profile_sensor_data)
    if args.test_interlock or args.original_manual_speeds:
        node.create_subscription(Joy, 'joy_teleop/joy', joy_callback, qos_profile_sensor_data)
    for topic, message_type in [('platform/motors/feedback', PumaMultiFeedback),
            ('platform_velocity_controller/controller_state', MecanumDriveControllerState),
            ('platform/odom/filtered', Odometry)]:
        node.create_subscription(message_type, topic,
            lambda msg, topic=topic: record(topic, message_to_ordereddict(msg)), qos_profile_sensor_data)
    for topic, kind in [('platform/cmd_vel', 'mux'), ('platform/cmd_vel_limited', 'limited')]:
        node.create_subscription(TwistStamped, topic,
                                 lambda msg, kind=kind: command_callback(msg, kind), qos_profile_sensor_data)
    client = node.create_client(ListControllers, ns+'/controller_manager/list_controllers')

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.01, max(0, end-time.monotonic())))

    def routing():
        raw = node.get_subscriptions_info_by_topic(ns+'/platform/cmd_vel')
        limited = node.get_subscriptions_info_by_topic(ns+'/platform/cmd_vel_limited')
        owners = node.get_publishers_info_by_topic(ns+'/platform/cmd_vel_limited')
        if any(e.node_name == 'platform_velocity_controller' for e in raw):
            raise RuntimeError('controller still subscribed to raw command topic')
        if not any(e.node_name == 'platform_velocity_controller' for e in limited):
            raise RuntimeError('controller missing limited-topic subscription')
        if len(owners) != 1 or owners[0].node_name != 'commissioning_speed_guard':
            raise RuntimeError('guard output ownership differs')
        if node.count_publishers(ns+'/cmd_vel'):
            raise RuntimeError('another external command publisher is present')
        if not any(e.node_name == 'twist_mux' for e in node.get_subscriptions_info_by_topic(ns+'/cmd_vel')):
            raise RuntimeError('external input is not subscribed by the mux')

    def send(speed):
        msg = populate_command(TwistStamped(), node.get_clock().now().to_msg(), speed, args.axis)
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        sent_stamps.add(stamp)
        pub.publish(msg)
        record('sent', {'speed': speed, 'stamp_ns': stamp})
        report['nonzero_commands'] += int(speed > 0)
        if speed > 0 and (state['motor_fault_during_pulse_time'] is not None
                          or state['stop_action_time'] is not None):
            state['source_after_stop'] += 1

    def interrupted(signum, frame):
        raise KeyboardInterrupt('interrupted or deadline expired')

    previous = {s: signal.signal(s, interrupted) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGALRM)}
    signal.alarm(120 if args.execute and (args.test_interlock or args.original_manual_speeds) else 20)
    try:
        print('Observing stop, stationary odometry and capped routing for five seconds...', flush=True)
        spin(5)
        routing()
        if state['abort']:
            raise RuntimeError(state['abort'])
        if state['odom_count'] < 20 or state['stop_count'] < 2:
            raise RuntimeError('insufficient odometry/stop evidence')
        if state['motor_status'] is None or time.monotonic()-state['motor_status_time'] > 1.5 \
                or not state['motor_status']['complete']:
            raise RuntimeError('fresh four-wheel motor status required')
        if not client.service_is_ready():
            raise RuntimeError('controller manager is unavailable')
        future = client.call_async(ListControllers.Request())
        deadline = time.monotonic()+2
        while not future.done() and time.monotonic() < deadline:
            spin(.02)
        if not future.done() or not any(c.name == 'platform_velocity_controller' and c.state == 'active'
                                      and c.type == 'mecanum_drive_controller/MecanumDriveController'
                                      for c in future.result().controller):
            raise RuntimeError('expected active controller missing')
        if not args.execute:
            report['status'] = 'passive-routing-stationary-verified'
            return 0
        if args.test_interlock or args.original_manual_speeds:
            if state['stop'] is not True or time.monotonic()-state['stop_time']>2.5:
                raise RuntimeError('bounded diagnostic must start with fresh pressed physical stop')
            if not state['controller_neutral'] or time.monotonic()-state['controller_time']>.5:
                raise RuntimeError('fresh neutral controller required before physical stop release')
            instruction = ('As soon as forward movement begins, PRESS the red physical e-stop; '
                           'keep the controller untouched. The source keeps requesting forward briefly '
                           'after your press to isolate the stop action.'
                           if args.stop_mode == 'physical-e-stop' else
                           'Do not touch the controller. The recorder will send one bounded forward pulse after release.')
            print('RECORDING READY: release ONLY the physical e-stop with controller centred.\n'
                  + instruction + '\nPress physical stop immediately for unexpected motion.',flush=True)
            release_deadline=time.monotonic()+60
            while state['stop'] is not False:
                spin(.05)
                now=time.monotonic()
                if state['abort'] or now-state['stop_time']>2.5 or now-state['odom_time']>.25:
                    raise RuntimeError(state['abort'] or 'stop or wheel observation stale before release')
                if not state['controller_neutral'] or now-state['controller_time']>.5:
                    raise RuntimeError('controller changed or became stale before release')
                if now>=release_deadline:
                    raise RuntimeError('physical stop was not released; no command sent')
            print('PHYSICAL RELEASE RECORDED. Keep the controller centred and physical stop ready.',flush=True)
            if args.original_manual_speeds or args.test_interlock:
                release_at = time.monotonic()
                state['motor_clear_started'], state['motor_clear_samples'] = None, 0
                state['phase'] = 'wait-motor-ready'
                print('Waiting for all four motor fault flags to clear and remain clear for one second. '
                      'Do not touch the controller.', flush=True)
                ready_deadline = release_at+20
                while not stable_after_release(state['motor_clear_started'],
                        state['motor_clear_samples'], time.monotonic(),
                        state['motor_status_time'], release_at):
                    spin(.05)
                    now = time.monotonic()
                    if state['abort'] or state['stop'] is not False or now-state['stop_time']>2.5 \
                            or now-state['odom_time']>.25 or any(abs(v)>.005 for v in state['velocity']) \
                            or not state['controller_neutral'] or now-state['controller_time']>.5:
                        raise RuntimeError(state['abort'] or 'stop, wheel or controller state changed while waiting for motor readiness')
                    if now >= ready_deadline:
                        raise RuntimeError('motor faults did not clear within 20 seconds; no command sent')
                report['motor_ready_after_release_s'] = time.monotonic()-release_at
                print('MOTORS READY: all four in speed mode with zero faults. Starting one bounded pulse.',
                      flush=True)
        now = time.monotonic()
        problem = feedback_problem(state['stop'], now-state['stop_time'], state['velocity'],
                                   now-state['odom_time'], now-state['limited_time'], args.axis)
        if problem:
            raise RuntimeError(problem)
        pub = node.create_publisher(TwistStamped, 'cmd_vel', 1)
        if args.test_interlock:
            existing = node.get_publishers_info_by_topic(ns+'/platform/commissioning_test_permit')
            if existing:
                raise RuntimeError('another commissioning permit publisher is present')
            permit_pub = node.create_publisher(Bool, 'platform/commissioning_test_permit', 1)
        spin(.3)
        if not pub.get_subscription_count():
            raise RuntimeError('publisher has no mux subscriber')
        if permit_pub is not None and not permit_pub.get_subscription_count():
            raise RuntimeError('test permit has no guard subscriber')
        print(('Starting ONE bounded forward pulse now. PRESS the red physical e-stop as soon as movement begins.'
               if args.stop_mode == 'physical-e-stop' else
               'Starting ONE '+args.axis+' pulse now. Press the physical stop if movement is unexpected.'), flush=True)
        state['phase'] = 'pulse'
        start = time.monotonic()
        duration = .5 if args.test_interlock else 2. if args.stop_mode == 'physical-e-stop' else 1.
        if permit_pub is not None:
            permit_pub.publish(Bool(data=True))
            record('test_permit', True)
        while (time.monotonic()-start < duration or stop_confirmation_pending(
                args.stop_mode, state['stop_action_time'], state['motor_fault_during_pulse_time'],
                time.monotonic())):
            now = time.monotonic()
            if args.stop_mode == 'physical-e-stop' and state['stop_action_time'] is not None:
                break
            state['peak_forward'] = max(state.get('peak_forward') or 0.0, state['velocity'][AXES[args.axis][0]])
            problem = state['abort'] or motor_fault_problem(
                args.stop_mode, state['phase'], state['stop_action_time'],
                state['motor_fault_during_pulse_time'], now) or feedback_problem(state['stop'], now-state['stop_time'],
                state['velocity'], now-state['odom_time'], now-state['limited_time'], args.axis,
                rebound_after_peak=state.get('peak_forward') if args.stop_mode == 'physical-e-stop' else None)
            if (args.test_interlock or args.original_manual_speeds) and (not state['controller_neutral'] or now-state['controller_time']>.5):
                problem = 'controller changed or became stale during direct command-loss pulse'
            if problem:
                raise RuntimeError(problem)
            if permit_pub is not None:
                permit_pub.publish(Bool(data=True))
                record('test_permit', True)
            send(requested_speed(time.monotonic()-start, args.ramp, args.stop_mode, duration))
            spin(.05)
        if args.stop_mode == 'physical-e-stop':
            if state['stop_action_time'] is None:
                raise RuntimeError('red physical stop was not recorded during the two-second bound; PRESS IT NOW')
            print('PHYSICAL STOP RECORDED. Leave it pressed and keep the controller untouched.', flush=True)
            action_time = state['motor_fault_during_pulse_time'] or state['stop_action_time']
            source_tail_deadline = physical_stop_tail_deadline(
                start, duration, action_time)
            while time.monotonic() < source_tail_deadline:
                if (state['stop'] is not True or state['abort'] or state['nonzero_motor_after_stop']
                        or state['nonzero_limited_after_stop']):
                    raise RuntimeError('physical stop released or nonzero command persisted; source halted')
                send(.05)
                spin(.05)
            report['physical_stop_command_chain'] = {
                'action_after_pulse_start_s': action_time - start,
                'action_basis': ('motor e-stop fault (first status sample showing it)'
                                 if state['motor_fault_during_pulse_time'] is not None
                                 else 'firmware physical-stop report'),
                'stop_report_after_action_s': state['stop_action_time'] - action_time,
                'recent_nonzero_motor_before_action': state['motor_before_stop'] is not None
                    and action_time - state['motor_before_stop'] <= .25,
                'first_zero_motor_after_action_s': None if state['zero_motor_after_stop'] is None
                    else state['zero_motor_after_stop'] - action_time,
                'first_zero_limited_after_action_s': None if state['zero_limited_after_stop'] is None
                    else state['zero_limited_after_stop'] - action_time,
                'nonzero_motor_after_action': state['nonzero_motor_after_stop'],
                'nonzero_limited_after_action': state['nonzero_limited_after_stop'],
                'source_commands_after_action': state['source_after_stop'],
                'raw_odom_motion_before_action': state['odom_motion_before_stop'],
                'qualified': physical_stop_qualifies(start, action_time,
                    state['motor_before_stop'], state['zero_motor_after_stop'],
                    state['zero_limited_after_stop'], state['nonzero_motor_after_stop'],
                    state['nonzero_limited_after_stop'],
                    state['source_after_stop'], state['odom_motion_before_stop'])}
            if not report['physical_stop_command_chain']['qualified']:
                raise RuntimeError('physical-stop command chain not isolated; preserve trace and keep stop pressed')
        state['phase'] = 'stopping'
        if args.stop_mode == 'timeout':
            print('Testing publishing silence for one second; no commands sent during this interval.', flush=True)
            state['phase'] = 'timeout'
            silence_start = time.monotonic()
            record('publishing_silence_started', {})
            while time.monotonic()-silence_start < 1.:
                if permit_pub is not None:
                    permit_pub.publish(Bool(data=True))
                    record('test_permit', True)
                spin(.05)
                if state['abort']:
                    raise RuntimeError(state['abort'])
                if args.test_interlock or args.original_manual_speeds:
                    now=time.monotonic()
                    problem=feedback_problem(state['stop'],now-state['stop_time'],state['velocity'],
                                             now-state['odom_time'],now-state['limited_time'],args.axis)
                    if not state['controller_neutral'] or now-state['controller_time']>.5:
                        problem='controller changed or became stale during publishing silence'
                    if problem:
                        raise RuntimeError(problem)
            if permit_pub is not None:
                zero_time=state['motor_zero_during_silence']
                report['source_silence_stop']={
                    'motor_motion_observed':state['motor_nonzero_time'] is not None,
                    'zero_motor_before_budget':zero_time is not None and zero_time-start < .95,
                    'nonzero_motor_resumed_after_zero':state['motor_nonzero_after_silence_zero'],
                    'first_zero_motor_after_silence_s':zero_time-silence_start if zero_time is not None else None,
                    'limit':'A zero motor command before the fixed permit budget expires is supporting evidence; operator must confirm actual stopping.'}
                if not source_silence_qualifies(
                        state['motor_nonzero_time'], zero_time, start,
                        state['motor_nonzero_after_silence_zero']):
                    raise RuntimeError('source-loss stop was not isolated before the independent guard budget')
            report['stationary_during_silence'] = (
                time.monotonic()-state['odom_time'] <= .25
                and all(math.isfinite(v) and abs(v) <= .005 for v in state['velocity']))
            if not report['stationary_during_silence']:
                raise RuntimeError('odometry did not stop during publishing silence')
            state['phase'] = 'stopping'
        print('Pulse ended; sending zeros and recording the stop...', flush=True)
        for _ in range(20):
            send(0)
            spin(.05)
        state['phase'] = 'after'
        spin(2)
        if state['abort']:
            raise RuntimeError(state['abort'])
        if args.stop_mode == 'physical-e-stop' and (state['nonzero_motor_after_stop']
                or state['nonzero_limited_after_stop'] or state['stop'] is not True):
            raise RuntimeError('physical stop released or nonzero command observed after action')
        if time.monotonic()-state['odom_time'] > .25 or any(not math.isfinite(v) or abs(v) > .005 for v in state['velocity']):
            raise RuntimeError('final odometry not stationary/fresh; operator must press stop')
        report['status'] = 'pulse-completed-final-odometry-stationary'
        if args.test_interlock or args.original_manual_speeds:
            if permit_pub is not None:
                permit_pub.publish(Bool(data=False))
                record('test_permit', False)
            print('PULSE COMPLETE: press and leave the physical e-stop engaged now.',flush=True)
            stop_deadline=time.monotonic()+30
            while state['stop'] is not True:
                send(0)
                spin(.05)
                if time.monotonic()>=stop_deadline:
                    raise RuntimeError('physical stop not re-engaged after command-loss trial')
            report['final_physical_stop_pressed'] = True
        else:
            print('Finished. Re-engage the physical e-stop and report what you saw.', flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        report['status'] = 'aborted'
        report['error'] = str(error)
        print('ABORT: '+str(error)+'. Keep/re-engage the physical stop.', flush=True)
        return 1
    finally:
        signal.alarm(0)
        if permit_pub is not None:
            permit_pub.publish(Bool(data=False))
            record('test_permit', False)
            spin(.05)
        if pub is not None:
            state['phase'] = 'cleanup'
            for _ in range(20 if report['status'] == 'aborted' else 6):
                send(0)
                spin(.05)
            if report['status'] == 'aborted':
                spin(1)
                report['stationary_after_abort'] = (
                    time.monotonic()-state['odom_time'] <= .25
                    and all(math.isfinite(v) and abs(v) <= .005 for v in state['velocity']))
        report['elapsed_s'] = time.monotonic()-origin
        report['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (root/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
