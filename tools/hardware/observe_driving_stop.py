#!/usr/bin/env python3
"""Record a supervised physical-control stop test; never publish drive commands.

Run in the operator's local terminal to coordinate button actions with recording.
Keep existing commissioning caps; no service, parameter, reset or enable writes.
"""
import argparse
import hashlib
import json
import math
import os
import re
import subprocess
from pathlib import Path
import time

from install_commissioning_caps import identity, desired_files
from controller_autoconnect import controllers

DIRECTIONS = {'forward': (0, 1), 'backward': (0, -1),
              'left': (1, 1), 'right': (1, -1),
              'turn-left': (2, 1), 'turn-right': (2, -1)}


def connected_controller(config=Path('/home/robot/controllers.txt'), run=subprocess.run):
    """Require the one configured physical controller, not just neutral ROS Joy."""
    try:
        devices = controllers(config)
        if len(devices) != 1:
            return False
        info = run(['/usr/bin/bluetoothctl', '--timeout', '3', 'info', devices[0]],
                   capture_output=True, text=True, stdin=subprocess.DEVNULL, timeout=5)
        return info.returncode == 0 and 'Connected: yes' in info.stdout
    except (OSError, ValueError, subprocess.SubprocessError):
        return False


def changed_motion_axes(message):
    axes = message.get('axes', [])
    if len(axes) <= 4 or not all(math.isfinite(v) for v in axes):
        raise ValueError('missing or nonfinite joystick axes')
    return [i for i in (0,1,3,4) if abs(axes[i]) > .1]


def mapping_input_ready(message):
    buttons = message.get('buttons', [])
    return len(buttons) > 5 and not any(buttons[i] for i in (4,5))


def await_pressed_stop(records, spin, clock, timeout=60):
    """Keep a physical test unarmed until two recent stop reports assert it."""
    deadline = clock() + timeout
    while True:
        stops = [r for r in records if r['topic'] == 'platform/emergency_stop']
        if (len(stops) >= 2 and all(r['message']['data'] for r in stops[-2:])
                and 0 <= clock() - stops[-1]['elapsed'] <= 2.5):
            return
        if clock() >= deadline:
            raise RuntimeError('timed out waiting for fresh asserted physical e-stop reports; test never armed')
        spin(.1)


def controller_neutral(records, now):
    joy = [r for r in records if r['topic'] == 'joy_teleop/joy']
    if not joy or not 0 <= now - joy[-1]['elapsed'] <= .5:
        return False
    message = joy[-1]['message']
    return (len(message['buttons']) > 5 and len(message['axes']) > 4 and
            not any(message['buttons'][b] for b in (4, 5)) and
            all(math.isfinite(v) for v in message['axes']) and
            all(abs(message['axes'][i]) <= .05 for i in (0,1,3,4)))


def release_sample_safe(topic, message):
    if topic == 'platform/motors/cmd':
        values = message['velocity']; tolerance = 1e-9
    elif topic in ('platform/cmd_vel', 'platform/cmd_vel_limited', 'platform/odom'):
        twist = message['twist']['twist'] if topic == 'platform/odom' else message['twist']
        values = [twist[part][axis] for part, axis in
                  [('linear', 'x'), ('linear', 'y'), ('angular', 'z')]]
        tolerance = .001 if topic == 'platform/odom' else 1e-9
    else:
        return True
    return bool(values) and all(math.isfinite(v) and abs(v) <= tolerance for v in values)


def disconnect_due(first_command, now, already_started):
    return first_command is not None and not already_started and now-first_command >= .5


def held_forward_input(message, direction='forward'):
    if direction not in ('forward', 'backward'):
        return False
    return held_direction_input(message, direction)


def held_direction_input(message, direction='forward'):
    if direction not in DIRECTIONS:
        return False
    axes, buttons = message['axes'], message['buttons']
    if (len(buttons) <= 5 or len(axes) <= 4 or not any(buttons[b] for b in (4,5))
            or not all(math.isfinite(v) for v in axes)):
        return False
    index, sign = DIRECTIONS[direction]
    # Installed joy_x_combiner: vertical sticks sum to x, right horizontal
    # commands y, left horizontal commands yaw. Do not permit diagonal input.
    values = (axes[1] + axes[4], axes[3], axes[0])
    other_axes = (0,3) if index == 0 else ((0,1,4) if index == 1 else (1,3,4))
    return sign*values[index] > .1 and all(abs(axes[i]) <= .05 for i in other_axes)


def bounded_permit_window(first_input, now, cancelled):
    return (first_input is not None and not cancelled
            and 0 <= now-first_input < 1.25)


def cancel_permit_on_input_loss(test, stop_action='budget'):
    # During disconnect qualification, keep permits alive briefly so input-loss
    # stopping can be distinguished from permit cancellation/budget expiration.
    # Stop-action trials keep the one-shot permit alive after the action. The
    # physical stop still locks the guard directly; cancelling the permit at
    # the same instant would obscure which input caused the commanded stop.
    return test != 'interlock-disconnect' and stop_action == 'budget'


def stop_action_evidence(records, action, action_elapsed):
    """Classify a stop only if it precedes the independent one-second budget.

    Host receive times support a command-chain observation, not physical
    braking latency. Keeping true permits after deadman release is essential:
    otherwise permit cancellation would explain the stop.
    """
    true_permits = [r['elapsed'] for r in records
                    if r['topic'] == 'observer/test_permit_sent'
                    and r['message']['data'] is True]
    evidence = {'action': action, 'action_elapsed_s': action_elapsed,
                'qualified_command_chain': False, 'reasons': []}
    if not true_permits or action_elapsed is None:
        evidence['reasons'].append('permit or stop action was not observed')
        return evidence
    budget_end = true_permits[0] + 1.
    evidence['first_permit_s'] = true_permits[0]
    evidence['guard_budget_end_s'] = budget_end
    if not true_permits[0] < action_elapsed < budget_end - .05:
        evidence['reasons'].append('stop action did not precede the guard budget with margin')
    if not any(action_elapsed < t < budget_end for t in true_permits):
        evidence['reasons'].append('true permits did not continue after stop action')
    motors = [(r['elapsed'], r['message'].get('velocity', [])) for r in records
              if r['topic'] == 'platform/motors/cmd']
    def is_zero(values):
        return len(values) == 4 and all(math.isfinite(v) and abs(v) <= 1e-8 for v in values)
    def is_nonzero(values):
        return len(values) == 4 and all(math.isfinite(v) for v in values) and not is_zero(values)
    if not any(max(true_permits[0], action_elapsed - .15) <= t < action_elapsed
               and is_nonzero(values)
               for t, values in motors):
        evidence['reasons'].append('no recent nonzero motor command before stop action')
    zeros = [t for t, values in motors if action_elapsed <= t < budget_end - .01
             and is_zero(values)]
    if not zeros:
        evidence['reasons'].append('no zero motor command before the guard budget')
    else:
        evidence['first_zero_motor_after_action_s'] = zeros[0]
        if any(t > zeros[0] and t < budget_end and not is_zero(values)
               for t, values in motors):
            evidence['reasons'].append('nonzero or invalid motor command followed the first zero')
    limited = [(r['elapsed'], r['message']['twist']) for r in records
               if r['topic'] == 'platform/cmd_vel_limited']
    def limited_zero(message):
        values = (message['linear']['x'], message['linear']['y'], message['angular']['z'])
        return all(math.isfinite(v) and abs(v) <= 1e-8 for v in values)
    if not any(action_elapsed <= t < budget_end - .01 and limited_zero(v)
               for t, v in limited):
        evidence['reasons'].append('no zero limited command before the guard budget')
    lock_state = action == 'e-stop'
    if not any(action_elapsed <= r['elapsed'] < budget_end and
               r['topic'] == 'platform/emergency_stop_mux' and
               r['message']['data'] is lock_state for r in records):
        evidence['reasons'].append('expected software lock state was not observed before the guard budget')
    evidence['qualified_command_chain'] = not evidence['reasons']
    return evidence


def deadman_released_with_forward(message):
    """Recognize enable-button release before the forward stick is centered."""
    axes, buttons = message.get('axes', []), message.get('buttons', [])
    return (len(axes) > 4 and len(buttons) > 5
            and all(math.isfinite(v) for v in axes)
            and not buttons[4] and not buttons[5]
            and axes[1] + axes[4] > .1
            and abs(axes[0]) <= .05 and abs(axes[3]) <= .05)


def disconnected_stationary_tail(records, now):
    """End a disconnect recording only after a pressed stop and fresh zeros."""
    stops=[r for r in records if r['topic']=='platform/emergency_stop']
    if len(stops)<2 or not all(r['message']['data'] for r in stops[-2:]) \
            or now-stops[-1]['elapsed']>2.5:
        return False
    for topic, count, age in [('platform/odom',20,.25),
                              ('platform/motors/cmd',10,.25),
                              ('platform/cmd_vel_limited',10,.25)]:
        samples=[r for r in records if r['topic']==topic]
        if len(samples)<count or now-samples[-1]['elapsed']>age:
            return False
        if not all(release_sample_safe(topic,r['message']) for r in samples[-count:]):
            return False
    return True


def forward_trial_command_safe(twist, direction='forward', yaw_tolerance=.01):
    if direction not in ('forward', 'backward'):
        return False
    return directional_trial_command_safe(twist, direction, yaw_tolerance)


DEADMAN_YAW_TOLERANCE = .02  # rad/s, 20% of the yaw cap; D2 deadman trial only (PS4 thumb drift reads 0.0109-0.011)


def directional_trial_command_safe(twist, direction='forward', yaw_tolerance=.01):
    # Manual stick drift may request tiny yaw. A captured forward trial had
    # 0.00616 rad/s incidental yaw without visible turning; allow at most
    # 10% of the installed yaw cap. Other command and odometry bounds stay.
    if direction not in DIRECTIONS:
        return False
    index, sign = DIRECTIONS[direction]
    values=[twist['linear']['x'],twist['linear']['y'],twist['angular']['z']]
    limit = .100000001 if index == 2 else .050000001
    return (all(math.isfinite(v) for v in values) and 0<=sign*values[index]<=limit
            and all(abs(v) <= (yaw_tolerance if i == 2 else 1e-9)
                    for i, v in enumerate(values) if i != index)
            and all(twist[p][a]==0 for p,a in [('linear','z'),('angular','x'),('angular','y')]))


def permit_odometry_safe(values, direction='forward'):
    if direction not in DIRECTIONS:
        return False
    index, sign = DIRECTIONS[direction]
    return (all(math.isfinite(v) for v in values)
            and (-.02 if index == 2 else -.01) <= sign*values[index] <= (.15 if index == 2 else .075)
            and all(abs(v) <= (.05 if i == 2 else .02)
                    for i, v in enumerate(values) if i != index))


def feedback_fault_due(records, first_input, started, now, already_sent):
    """Inject only after fresh, bounded forward wheel evidence inside the permit."""
    if first_input is None or already_sent or not 0 <= now-first_input < .7:
        return False
    recent_command = any(r['topic'] == 'platform/motors/cmd'
        and r['elapsed'] >= first_input-started
        and now-started-r['elapsed'] <= .15
        and r['message']['velocity']
        and all(math.isfinite(v) for v in r['message']['velocity'])
        and any(abs(v) > 1e-8 for v in r['message']['velocity'])
        for r in records)
    return recent_command and any(r['topic'] == 'platform/odom'
        and r['elapsed'] >= first_input-started
        and now-started-r['elapsed'] <= .15
        and all(math.isfinite(v) for v in (
            r['message']['twist']['twist']['linear']['x'],
            r['message']['twist']['twist']['linear']['y'],
            r['message']['twist']['twist']['angular']['z']))
        and .005 < r['message']['twist']['twist']['linear']['x'] <= .075
        and abs(r['message']['twist']['twist']['linear']['y']) <= .02
        and abs(r['message']['twist']['twist']['angular']['z']) <= .05
        for r in records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test', choices=['stationary', 'deadman', 'disconnect',
                                         'disconnect-stationary', 'e-stop', 'e-stop-release', 'interlock-held', 'interlock-permit', 'interlock-disconnect', 'interlock-reconnect', 'interlock-feedback-fault'], default='stationary')
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--mapping-axis', type=int, choices=(0, 3),
                        help='stationary yaw or lateral stick check: wait for input and neutral return')
    parser.add_argument('--disconnect-device', help='disconnect one Bluetooth MAC during a supervised disconnect test')
    parser.add_argument('--reconnect-device', help='observe operator reconnection of this initially disconnected Bluetooth MAC')
    parser.add_argument('--moving-disconnect-confirmed', action='store_true')
    parser.add_argument('--test-interlock', action='store_true', help='inspect a temporarily interlocked installation; stationary-only for now')
    parser.add_argument('--permit-motion-confirmed', action='store_true', help='authorize one supervised bounded permit trial')
    parser.add_argument('--direction', choices=DIRECTIONS, default='forward',
                        help='axis selection applies only to the plain interlock-permit trial')
    parser.add_argument('--stop-action', choices=('budget','deadman','e-stop'), default='budget',
                        help='interlock-permit only: operator action during a bounded forward trial')
    parser.add_argument('--feedback-fault-confirmed', action='store_true', help='authorize one invalid ROS feedback batch during a bounded, wheels-raised trial')
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.mapping_axis is not None and (args.test != 'stationary' or not args.test_interlock
                                         or not args.operator_confirmed):
        parser.error('--mapping-axis requires stationary, test-interlock and operator-confirmed')
    if args.mapping_axis is not None and not connected_controller():
        parser.error('controller disconnected; reconnect with physical stop pressed first')
    if args.direction != 'forward' and args.test != 'interlock-permit':
        parser.error('--direction '+args.direction+' applies only to interlock-permit; fault/disconnect trials stay forward-only')
    if args.stop_action != 'budget' and (args.test != 'interlock-permit' or args.direction != 'forward'):
        parser.error('--stop-action requires a forward interlock-permit trial')
    permit_trial = args.test in ('interlock-permit','interlock-disconnect','interlock-feedback-fault')
    if args.test in ('interlock-held','interlock-permit','interlock-disconnect','interlock-reconnect','interlock-feedback-fault') and not args.test_interlock:
        parser.error('held-command test requires the installed test interlock; keep physical e-stop pressed')
    if permit_trial and not (args.operator_confirmed and args.permit_motion_confirmed):
        parser.error('permit trial requires explicit motion and operator confirmation')
    if args.test == 'interlock-disconnect' and not (args.disconnect_device and args.moving_disconnect_confirmed):
        parser.error('interlocked disconnection requires a device and explicit moving-disconnect confirmation')
    if args.test == 'interlock-feedback-fault' and not args.feedback_fault_confirmed:
        parser.error('wheels-raised feedback fault requires explicit confirmation')
    if args.feedback_fault_confirmed and args.test != 'interlock-feedback-fault':
        parser.error('--feedback-fault-confirmed applies only to interlock-feedback-fault')
    if args.test_interlock and args.test not in ('stationary','disconnect-stationary','e-stop-release','interlock-held','interlock-permit','interlock-disconnect','interlock-reconnect','interlock-feedback-fault'):
        parser.error('interlocked observer currently supports only stationary checks; keep physical stop pressed')
    if args.disconnect_device and args.test == 'disconnect':
        parser.error('automatic moving disconnect is disabled pending a preflight motion interlock; keep physical e-stop pressed')
    if args.disconnect_device:
        if (args.test not in ('disconnect-stationary','disconnect','interlock-disconnect') or not args.operator_confirmed
                or (args.test in ('disconnect','interlock-disconnect') and not args.moving_disconnect_confirmed)
                or not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', args.disconnect_device)):
            parser.error('automatic Bluetooth disconnect requires a supported test, operator confirmation, one MAC and explicit moving confirmation for motion')
        args.disconnect_device = args.disconnect_device.upper()
        info = subprocess.run(['/usr/bin/bluetoothctl', 'info', args.disconnect_device],
                              capture_output=True, text=True, timeout=8, check=True)
        if 'Connected: yes' not in info.stdout:
            parser.error('selected controller is not connected; reconnect with stop pressed and controls neutral first')
    if args.test != 'stationary' and not args.operator_confirmed:
        parser.error('physical actions require an operator, clear floor and reachable stop')
    if permit_trial and not connected_controller():
        parser.error('configured Bluetooth controller is disconnected; keep physical e-stop pressed')
    reconnect_info=None
    if args.test == 'interlock-reconnect':
        if (args.disconnect_device or not args.reconnect_device
                or not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}',args.reconnect_device)):
            parser.error('reconnect observation requires one --reconnect-device MAC and no disconnect action')
        args.reconnect_device=args.reconnect_device.upper()
        reconnect_info=subprocess.run(['/usr/bin/bluetoothctl','--timeout','1','info',args.reconnect_device],
            stdin=subprocess.DEVNULL,capture_output=True,text=True,timeout=3)
        if reconnect_info.returncode or 'Connected: no' not in reconnect_info.stdout:
            parser.error('controller must initially be disconnected; keep physical e-stop pressed')
    elif args.reconnect_device:
        parser.error('--reconnect-device applies only to interlock-reconnect')
    root = args.run_dir.resolve()
    evidence = Path(__file__).resolve().parents[2]/'artifacts/hardware'
    if not root.is_relative_to(evidence) or root == evidence:
        parser.error('use a fresh directory below artifacts/hardware')
    root.mkdir(parents=True, exist_ok=False)
    if reconnect_info is not None:
        (root/'bluetooth-before.txt').write_text(reconnect_info.stdout+reconnect_info.stderr)
    ns = identity()
    for path, content in desired_files(ns,args.test_interlock).items():
        if path.read_bytes() != content:
            raise RuntimeError('commissioning installation differs: '+str(path))
    os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from sensor_msgs.msg import JointState, Joy
    from clearpath_motor_msgs.msg import PumaMultiStatus, PumaMultiFeedback
    from control_msgs.msg import MecanumDriveControllerState
    from diagnostic_msgs.msg import DiagnosticArray
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=[])
    node = rclpy.create_node('supervised_driving_stop_observer', namespace=ns,
                            use_global_arguments=False, enable_rosout=False,
                            start_parameter_services=False)
    log = (root/'samples.jsonl').open('w')
    started = time.monotonic()
    phase = 'preflight'
    records = []; first_command = None; first_joystick_input = None; first_release = None
    stop_action_at = None; action_prompted = False
    disconnect_worker = None; disconnect_log = None; disconnect_started = None
    permit_pub = None; permit_cancelled = False; permit_prompted = False
    feedback_fault_pub = None; feedback_fault_at = None
    if args.test == 'interlock-feedback-fault':
        # Create early so DDS discovery completes before the one-second permit.
        feedback_fault_pub = node.create_publisher(
            PumaMultiFeedback, 'platform/motors/feedback', qos_profile_sensor_data)

    def begin_disconnect():
        nonlocal disconnect_worker, disconnect_log, disconnect_started
        if disconnect_worker is not None:
            return
        disconnect_log = (root/'bluetooth-disconnect.log').open('w')
        disconnect_started = time.monotonic()
        disconnect_worker = subprocess.Popen(
            ['/usr/bin/bluetoothctl', '--timeout', '1', 'disconnect', args.disconnect_device],
            stdin=subprocess.DEVNULL, stdout=disconnect_log, stderr=subprocess.STDOUT)
        event={'elapsed':disconnect_started-started,'phase':phase,
               'topic':'observer/bluetooth_disconnect_requested','message':{'device':args.disconnect_device}}
        records.append(event);log.write(json.dumps(event)+'\n');log.flush()

    def callback(topic, msg):
        nonlocal first_command, first_joystick_input, first_release
        nonlocal stop_action_at
        nonlocal disconnect_worker, disconnect_log, disconnect_started
        row = {'elapsed': time.monotonic()-started, 'phase': phase,
               'topic': topic, 'message': message_to_ordereddict(msg)}
        records.append(row)
        log.write(json.dumps(row, default=list)+'\n');log.flush()
        if phase == 'preflight' and not release_sample_safe(topic, row['message']):
            raise RuntimeError('command or wheel movement appeared BEFORE RECORDING READY; release driving-enable and PRESS THE PHYSICAL E-STOP NOW')
        if phase == 'operator' and args.test == 'stationary':
            if topic == 'platform/emergency_stop' and not msg.data:
                raise RuntimeError('physical e-stop released during stationary recording; PRESS PHYSICAL E-STOP NOW')
            if not release_sample_safe(topic,row['message']):
                raise RuntimeError('unexpected command or wheel movement during stationary recording; PRESS PHYSICAL E-STOP NOW')
        if args.test_interlock and topic == 'platform/emergency_stop_mux' and not msg.data:
            if not (permit_trial and first_joystick_input is not None
                    and time.monotonic()-first_joystick_input < 1.6):
                raise RuntimeError('test interlock unexpectedly reported unlocked without a test permit; press physical e-stop')
        if phase == 'operator' and args.test == 'interlock-reconnect':
            if topic == 'platform/emergency_stop' and not msg.data:
                raise RuntimeError('physical stop released during stationary reconnect; PRESS PHYSICAL STOP')
            if not release_sample_safe(topic,row['message']):
                raise RuntimeError('unexpected commands or wheel motion during reconnect; PRESS PHYSICAL STOP')
            if topic == 'joy_teleop/joy' and not controller_neutral(records,time.monotonic()-started):
                raise RuntimeError('non-neutral driving controls during reconnect; keep physical stop pressed')
        if phase == 'operator' and permit_trial:
            if topic == 'platform/emergency_stop' and not msg.data and first_release is None:
                first_release = time.monotonic()
                instruction = ('Gently hold driving-enable and FORWARD until automatic disconnection, then release controls.'
                               if args.test == 'interlock-disconnect' else
                               f'Gently hold driving-enable and {args.direction.upper()} until the stop prompt (about 1.25 seconds).')
                if args.stop_action == 'e-stop':
                    instruction = ('Gently hold driving-enable and FORWARD. As soon as you SEE or FEEL forward movement,'
                                   ' press the ROBOT RED E-STOP while still holding the controls, then release them.'
                                   ' Do not wait for another terminal prompt.')
                elif args.stop_action == 'deadman':
                    instruction = ('Gently hold driving-enable and FORWARD. As soon as you SEE or FEEL'
                                   ' forward movement, release L1 while keeping the stick forward briefly,'
                                   ' then center it. Do not wait for another terminal prompt.')
                print('PHYSICAL RELEASE RECORDED. Controller input is allowed NOW.\n'
                      + instruction+'\n'
                      'The guard must block movement after one second. Press physical stop immediately for unexpected movement.',flush=True)
            if topic == 'joy_teleop/joy' and held_direction_input(row['message'], args.direction) and first_joystick_input is None:
                stops=[r for r in records if r['topic']=='platform/emergency_stop']
                if first_release is None or not stops or stops[-1]['message']['data']:
                    raise RuntimeError(args.direction+' began before physical release was recorded; press physical stop')
                first_joystick_input=time.monotonic()
                print('ONE-SHOT PERMIT STARTED. Keep the physical stop ready; do not make another movement.',flush=True)
            if (args.stop_action == 'deadman' and topic == 'joy_teleop/joy'
                    and first_joystick_input is not None and stop_action_at is None
                    and deadman_released_with_forward(row['message'])):
                stop_action_at = time.monotonic()
                print('DEADMAN RELEASE RECORDED. Center the stick; keep the physical stop ready.', flush=True)
            if (args.stop_action == 'e-stop' and topic == 'platform/emergency_stop'
                    and first_joystick_input is not None and stop_action_at is None and msg.data):
                stop_action_at = time.monotonic()
                print('PHYSICAL STOP RECORDED. Release the controller controls; leave the stop pressed.', flush=True)
            if topic == 'platform/odom' and first_joystick_input is not None:
                v=msg.twist.twist
                if not permit_odometry_safe([v.linear.x,v.linear.y,v.angular.z], args.direction):
                    raise RuntimeError('wheel feedback exceeds '+args.direction+' trial bounds; PRESS PHYSICAL STOP NOW')
            if topic == 'platform/cmd_vel_limited':
                if not directional_trial_command_safe(row['message']['twist'], args.direction,
                        DEADMAN_YAW_TOLERANCE if args.stop_action == 'deadman' else .01):
                    raise RuntimeError('limited command outside '+args.direction+' trial bounds; PRESS PHYSICAL STOP NOW')
        if phase == 'operator' and args.test == 'interlock-held':
            if not release_sample_safe(topic,row['message']):
                raise RuntimeError('unexpected command or wheel motion during held-input lock check; PRESS PHYSICAL E-STOP NOW')
            if topic == 'platform/emergency_stop' and not msg.data and first_release is None:
                first_release = time.monotonic()
                print('PHYSICAL RELEASE RECORDED; SOFTWARE LOCK STILL ACTIVE.\n'
                      'Gently hold driving-enable and FORWARD for at most one second.\n'
                      'Release controller controls, then press the physical stop.',flush=True)
            if topic == 'joy_teleop/joy' and held_forward_input(row['message']) and first_joystick_input is None:
                stops=[r for r in records if r['topic']=='platform/emergency_stop']
                if first_release is None or not stops or stops[-1]['message']['data']:
                    raise RuntimeError('input began before a released physical stop was recorded; press physical stop')
                first_joystick_input=time.monotonic()
                print('FORWARD INPUT RECORDED. Release controller controls now and press the physical stop.\n'
                      'Recording continues for five seconds. No test permits are sent.',flush=True)
        if phase == 'operator' and args.test == 'e-stop-release':
            if not release_sample_safe(topic, row['message']):
                raise RuntimeError('unexpected command or wheel movement during neutral stop-release check')
            if topic == 'platform/emergency_stop' and not msg.data and first_release is None:
                first_release = time.monotonic()
                print('STOP RELEASE RECORDED: keep controller untouched. Observing for 10 seconds.', flush=True)
            elif (topic == 'platform/emergency_stop' and msg.data and first_release is not None
                  and time.monotonic()-first_release < 10):
                raise RuntimeError('physical stop re-engaged before the ten-second release observation completed')
        if phase == 'operator' and args.test == 'disconnect-stationary':
            if topic == 'platform/emergency_stop' and not msg.data:
                raise RuntimeError('e-stop became released during stationary disconnection check')
            if (topic == 'joy_teleop/joy' and first_joystick_input is None
                    and len(msg.buttons) > 5 and any(msg.buttons[b] for b in (4, 5))
                    and any(abs(value) > .1 for value in msg.axes)):
                first_joystick_input = time.monotonic()
                if args.disconnect_device:
                    begin_disconnect()
                    print('HELD INPUT RECORDED: disconnecting the selected controller now.\n'
                          'KEEP THE PHYSICAL E-STOP PRESSED. Release controls after disconnection.', flush=True)
                    return
                print('JOYSTICK INPUT RECORDED: keep the physical E-STOP PRESSED.\n'
                      'Switch off/disconnect the controller while holding that input, '
                      'then release its controls. Recording continues for 20 seconds.', flush=True)
        if phase == 'operator' and topic == 'platform/cmd_vel' and first_command is None:
            v = msg.twist
            if v.linear.x or v.linear.y or v.angular.z:
                if args.test == 'disconnect-stationary':
                    raise RuntimeError('nonzero mux command appeared during stationary disconnection check')
                first_command = time.monotonic()
                action = {'deadman': 'Release the DEADMAN BUTTON now, then center the stick.',
                          'disconnect': 'Disconnect/power off the controller now.',
                          'e-stop': 'Press the physical E-STOP now.'}
                if args.test == 'disconnect' and args.disconnect_device:
                    action['disconnect'] = 'Automatic disconnect in half a second. Keep forward held, at most one second; press physical stop if anything is unexpected.'
                if args.test == 'interlock-disconnect':
                    print('FORWARD COMMAND RECORDED: automatic controller disconnect in half a second.\n'
                          'Release controls after it disconnects; press physical stop at the stop prompt.',flush=True)
                elif args.test != 'interlock-permit':
                    print('Movement command detected. '+action.get(args.test, 'Observe only.')+
                          ' Recording continues for 10 seconds; leave the e-stop engaged afterward.', flush=True)

    topics = [('platform/emergency_stop', Bool), ('platform/emergency_stop_mux', Bool), ('platform/odom', Odometry),
              ('platform/odom/filtered', Odometry), ('platform/cmd_vel', TwistStamped),
              ('platform/cmd_vel_limited', TwistStamped), ('joy_teleop/cmd_vel_combined', TwistStamped),
              ('joy_teleop/joy', Joy), ('platform/motors/cmd', JointState),
              ('platform/motors/feedback', PumaMultiFeedback), ('platform/motors/status', PumaMultiStatus),
              ('platform_velocity_controller/controller_state', MecanumDriveControllerState),
              ('diagnostics', DiagnosticArray)]
    for topic, message_type in topics:
        node.create_subscription(message_type, topic,
            lambda msg, topic=topic: callback(topic, msg), qos_profile_sensor_data)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.05, max(0., end-time.monotonic())))

    report = {'test': args.test, 'direction': args.direction, 'stop_action': args.stop_action,
              'incidental_yaw_tolerance_rad_s': DEADMAN_YAW_TOLERANCE if args.stop_action == 'deadman' else .01,
              'namespace': ns, 'command_publishers_created': 0,
              'status': 'incomplete', 'operator_observation': 'required separately'}
    try:
        print('Checking capped routing and stationary stop. Keep the e-stop engaged.', flush=True)
        spin(4)
        stop = [r for r in records if r['topic'] == 'platform/emergency_stop']
        if args.test != 'stationary' and (len(stop) < 2 or
                not all(r['message']['data'] for r in stop[-2:])):
            print('NOT READY: press the ROBOT PHYSICAL E-STOP and leave it pressed.\n'
                  'Do not drive. Waiting up to 60 seconds for the stop to be reported pressed.', flush=True)
            await_pressed_stop(records, spin, lambda: time.monotonic()-started)
            stop = [r for r in records if r['topic'] == 'platform/emergency_stop']
        odom = [r for r in records if r['topic'] == 'platform/odom']
        if len(odom) < 20:
            print('Waiting for sufficient wheel feedback; keep the physical e-stop pressed.', flush=True)
            feedback_deadline = time.monotonic()+6
            while len(odom) < 20 and time.monotonic() < feedback_deadline:
                spin(.1)
                odom = [r for r in records if r['topic'] == 'platform/odom']
            stop = [r for r in records if r['topic'] == 'platform/emergency_stop']
        report['preflight'] = {'stop_samples': len(stop), 'raw_odom_samples': len(odom),
                               'last_stop_states': [r['message']['data'] for r in stop[-2:]]}
        if len(stop) < 2:
            raise RuntimeError('insufficient firmware e-stop messages; stop state is unverified')
        if not all(r['message']['data'] for r in stop[-2:]):
            raise RuntimeError('firmware reports e-stop RELEASED; press it BEFORE running, '
                               'and release only AFTER RECORDING READY')
        if len(odom) < 20:
            raise RuntimeError('insufficient raw wheel odometry; stationary state is unverified')
        for r in odom:
            v = r['message']['twist']['twist']
            if any(not math.isfinite(v[part][axis]) or abs(v[part][axis]) > .001
                   for part, axis in [('linear','x'),('linear','y'),('angular','z')]):
                raise RuntimeError('robot not stationary during preflight')
        owners = node.get_publishers_info_by_topic(ns+'/platform/cmd_vel_limited')
        raw = node.get_subscriptions_info_by_topic(ns+'/platform/cmd_vel')
        limited = node.get_subscriptions_info_by_topic(ns+'/platform/cmd_vel_limited')
        if (len(owners) != 1 or owners[0].node_name != 'commissioning_speed_guard'
                or any(e.node_name == 'platform_velocity_controller' for e in raw)
                or not any(e.node_name == 'platform_velocity_controller' for e in limited)):
            raise RuntimeError('capped routing/ownership differs')
        if args.test_interlock:
            locks=[r for r in records if r['topic']=='platform/emergency_stop_mux']
            if (len(locks)<2 or not all(r['message']['data'] for r in locks[-2:])
                    or time.monotonic()-started-locks[-1]['elapsed']>.25):
                raise RuntimeError('fresh asserted software-lock evidence required before physical actions')
            if node.get_publishers_info_by_topic(ns+'/platform/commissioning_test_permit'):
                raise RuntimeError('unexpected test-permit publisher; keep physical stop pressed')
        if permit_trial and not connected_controller():
            raise RuntimeError('configured Bluetooth controller disconnected before arming; keep physical stop pressed')
        if args.test == 'interlock-feedback-fault':
            from install_puma_feedback_plugin import load as load_plugin_receipt
            _, receipt = load_plugin_receipt(Path(__file__).resolve().parents[2] /
                'artifacts/hardware/20261013-puma-feedback-install-stage-5')
            selected = Path('/etc/systemd/system/clearpath-platform.service.d/95-dynamo-puma-feedback.conf')
            if receipt['status'] != 'installed-verified-stationary' or not selected.is_file():
                raise RuntimeError('installed feedback-protection receipt/drop-in missing')
            if receipt['prefix'] not in selected.read_text():
                raise RuntimeError('feedback-protection drop-in changed')
            sources = node.get_publishers_info_by_topic(ns+'/platform/motors/feedback')
            if sorted(e.node_name for e in sources) != ['puma_control', 'supervised_driving_stop_observer']:
                raise RuntimeError('wheel-feedback publisher ownership differs; keep physical stop pressed')
            report['plugin_prefix'] = receipt['prefix']
        if args.test != 'stationary' and not controller_neutral(records, time.monotonic()-started):
            print('NOT READY: release both driving-enable buttons and center the sticks.\n'
                  'Keep the ROBOT PHYSICAL E-STOP PRESSED while waiting.', flush=True)
            deadline = time.monotonic() + 60
            while not controller_neutral(records, time.monotonic()-started):
                if time.monotonic() >= deadline:
                    raise RuntimeError('controller did not become fresh and neutral; test never armed')
                spin(.1)
            # Revalidate stop freshness after waiting for the operator's controls.
            await_pressed_stop(records, spin, lambda: time.monotonic()-started)
        if args.test != 'stationary':
            now = time.monotonic()-started
            latest_stop = [r for r in records if r['topic']=='platform/emergency_stop']
            latest_odom = [r for r in records if r['topic']=='platform/odom'][-20:]
            if (len(latest_stop)<2 or not all(r['message']['data'] for r in latest_stop[-2:])
                    or not 0 <= now-latest_stop[-1]['elapsed'] <= 2.5):
                raise RuntimeError('physical stop changed or became stale before arming')
            if (len(latest_odom)<20 or not 0 <= now-latest_odom[-1]['elapsed'] <= .25
                    or any(not math.isfinite(r['message']['twist']['twist'][part][axis])
                           or abs(r['message']['twist']['twist'][part][axis])>.001
                           for r in latest_odom
                           for part,axis in [('linear','x'),('linear','y'),('angular','z')])):
                raise RuntimeError('fresh stationary wheel evidence lost before arming')
        phase = 'operator'
        if args.test == 'stationary':
            if args.mapping_axis is None:
                print('Recording stationary diagnostics for 15 seconds. Leave stop engaged.', flush=True)
            else:
                stick_instruction = ('Move ONLY the left-hand thumbstick RIGHT for one second, then let go.'
                                     if args.mapping_axis == 0 else
                                     'Move ONLY the right-hand thumbstick LEFT for one second, then let go.')
                print('MAPPING READY: KEEP PHYSICAL E-STOP PRESSED and driving-enable RELEASED.\n'
                      + stick_instruction + '\n'
                      'Waiting up to 60 seconds for input; the robot must stay still.',flush=True)
        elif args.test == 'interlock-reconnect':
            print('RECORDING READY: KEEP THE PHYSICAL E-STOP PRESSED; leave the controller disconnected.\n'
                  'Observe ten seconds of disconnected state. Wait for RECONNECT NOW before pressing PS.\n'
                  'Keep both driving-enable buttons released and sticks centred throughout.',flush=True)
        elif permit_trial:
            print(f'RECORDING READY: one bounded {args.direction}-permit trial.\n'
                  'STEP 1 ONLY: physically release the ROBOT RED E-STOP using its normal procedure.\n'
                  'Keep both hands OFF the controller and wait for PHYSICAL RELEASE RECORDED.\n'
                  'If that line does not appear, do not drive; press the red e-stop again and stop the recorder.\n'
                  'Press the red e-stop immediately if anything moves.',flush=True)
            if args.test == 'interlock-disconnect':
                print('The recorder disconnects the selected controller half a second after the first forward command.\n'
                      'Keep controls held until it disconnects; release them afterward. Do not reconnect.',flush=True)
            elif args.test == 'interlock-feedback-fault':
                print('WHEELS MUST BE SECURELY RAISED. After forward wheel motion is measured, the recorder\n'
                      'publishes an invalid ROS feedback batch while the Puma driver stays running.\n'
                      'Watch whether all wheels stop by themselves before the stop prompt.\n'
                      'Press the physical stop immediately if anything is unexpected.',flush=True)
        elif args.test == 'interlock-held':
            print('RECORDING READY: software test lock is asserted; no permits are sent.\n'
                  'Release only the PHYSICAL E-STOP with the controller centred.\n'
                  'Wait for PHYSICAL RELEASE RECORDED before touching controller input.\n'
                  'If anything moves, press the physical stop immediately.',flush=True)
        elif args.test == 'e-stop-release':
            print('RECORDING READY: keep both driving-enable buttons released and sticks centred.\n'
                  'Release the physical e-stop using its normal procedure. DO NOT command movement.\n'
                  'Press the physical e-stop immediately if anything moves.\n'
                  'Wait for the prompt to re-engage it after 10 seconds of observation.', flush=True)
        elif args.test == 'disconnect-stationary':
            if args.disconnect_device:
                print('RECORDING READY: KEEP THE PHYSICAL E-STOP PRESSED throughout this check.\n'
                      'Hold driving-enable and a small forward input. The recorder will disconnect\n'
                      'the selected Bluetooth controller immediately when held input is received.\n'
                      'Keep input held until disconnection, then release controls. Do not reconnect.', flush=True)
            else:
                print('RECORDING READY: KEEP THE PHYSICAL E-STOP PRESSED throughout this check.\n'
                  'Hold the deadman and a small forward stick input while the stop is engaged.\n'
                  'Switch off/disconnect the controller while holding that input, then release its controls.\n'
                  'Do not release the e-stop. Reconnect only afterward, with controls centered and stop engaged.\n'
                      'Waiting up to 60 seconds for joystick input, then recording 20 more seconds.', flush=True)
        else:
            instructions = {
                'deadman': 'Release the DEADMAN BUTTON FIRST, then center the stick. Centering the stick alone does not test deadman release.',
                'disconnect': 'Disconnect/power off the controller, then release its controls. Do not reconnect until the physical stop is engaged.',
                'e-stop': 'Press the physical e-stop while the forward command is held; then release the joystick controls. Leave the stop engaged.'}
            print('RECORDING READY: release the physical stop using its normal procedure.\n'
                  'Hold the deadman and gently command FORWARD for at most one second.\n'
                  +instructions[args.test]+'\n'
                  'Press the physical stop immediately for unexpected movement.\n'
                  'Re-engage the physical stop after observing the stop.\n'
                  'Waiting up to 60 seconds for the first movement command, then recording 10 more seconds.', flush=True)
        if args.test == 'stationary':
            if args.mapping_axis is None:
                spin(15)
            else:
                deadline=time.monotonic()+60
                next_notice=time.monotonic()+5
                neutral_since=None
                observed_axes=set()
                expected_axis_values=[]
                while time.monotonic()<deadline:
                    spin(.05)
                    now=time.monotonic()-started
                    for topic,age in [('platform/emergency_stop',2.5),('platform/emergency_stop_mux',.25),
                                      ('platform/odom',.25),('platform/cmd_vel_limited',.25),('joy_teleop/joy',.5)]:
                        rr=[r for r in records if r['topic']==topic]
                        if not rr or now-rr[-1]['elapsed']>age:
                            raise RuntimeError('mapping observation stale: '+topic+'; keep physical stop pressed')
                    joy=[r for r in records if r['topic']=='joy_teleop/joy']
                    if not mapping_input_ready(joy[-1]['message']):
                        neutral_since=None
                        if time.monotonic()>=next_notice:
                            print('WAITING: release both driving-enable buttons. KEEP PHYSICAL E-STOP PRESSED.',flush=True)
                            next_notice=time.monotonic()+5
                        continue
                    changed=changed_motion_axes(joy[-1]['message'])
                    observed_axes.update(changed)
                    if args.mapping_axis in changed:
                        expected_axis_values.append(float(joy[-1]['message']['axes'][args.mapping_axis]))
                    if changed:
                        neutral_since=None
                        if first_joystick_input is None:
                            first_joystick_input=time.monotonic()
                            print('STICK INPUT RECEIVED on axes '+str(changed)+'. Let go of the stick now; keep e-stop pressed.',flush=True)
                    elif first_joystick_input is not None:
                        if neutral_since is None:neutral_since=time.monotonic()
                        if time.monotonic()-neutral_since>=1:
                            break
                    if time.monotonic()>=next_notice:
                        print('Still waiting for stick input/neutral return. Keep e-stop pressed; driving-enable released.',flush=True)
                        next_notice=time.monotonic()+5
                else:
                    raise RuntimeError('mapping input/neutral return not completed; keep physical stop pressed')
                expected_sign = -1 if args.mapping_axis == 0 else 1
                correct_sign = bool(expected_axis_values) and any(v * expected_sign > .1 for v in expected_axis_values)
                report['mapping']={'expected_axis':args.mapping_axis,'observed_axes':sorted(observed_axes),
                                   'neutral_return':True,'expected_axis_only':observed_axes=={args.mapping_axis},
                                   'axis_min':min(expected_axis_values,default=None),
                                   'axis_max':max(expected_axis_values,default=None),
                                   'expected_sign_observed':correct_sign}
                print('MAPPING RECORDED: '+json.dumps(report['mapping'])+'. No motion or permits sent.',flush=True)
        elif args.test == 'interlock-reconnect':
            stage_start=time.monotonic(); reconnect_prompt=False
            while time.monotonic()-stage_start<40:
                spin(.05)
                now=time.monotonic()-started
                for topic,age in [('platform/emergency_stop',2.5),('platform/emergency_stop_mux',.25),
                                  ('platform/odom',.25),('platform/cmd_vel_limited',.25)]:
                    samples=[r for r in records if r['topic']==topic]
                    if not samples or now-samples[-1]['elapsed']>age:
                        last_age=now-samples[-1]['elapsed'] if samples else None
                        raise RuntimeError(f'reconnect observation stale: {topic}, age={last_age}, limit={age}s; keep physical stop pressed')
                if node.get_publishers_info_by_topic(ns+'/platform/commissioning_test_permit'):
                    raise RuntimeError('unexpected permit publisher during reconnect; keep physical stop pressed')
                if time.monotonic()-stage_start>=10 and not reconnect_prompt:
                    print('RECONNECT NOW: briefly press controller PS with sticks centred and driving-enable released.\n'
                          'KEEP ROBOT PHYSICAL E-STOP PRESSED. Do not request movement. Recording continues for 30 seconds.',flush=True)
                    reconnect_prompt=True
            check=subprocess.run(['/usr/bin/bluetoothctl','--timeout','1','info',args.reconnect_device],
                stdin=subprocess.DEVNULL,capture_output=True,text=True,timeout=3)
            (root/'bluetooth-after.txt').write_text(check.stdout+check.stderr)
            if check.returncode or 'Connected: yes' not in check.stdout:
                raise RuntimeError('controller reconnection not confirmed; keep physical stop pressed')
            spin(2)
            now=time.monotonic()-started
            stops=[r for r in records if r['topic']=='platform/emergency_stop']
            if (not controller_neutral(records,now) or len(stops)<2
                    or not all(r['message']['data'] for r in stops[-2:])
                    or now-stops[-1]['elapsed']>2.5):
                raise RuntimeError('fresh neutral controller and asserted stop required after reconnect')
            report['controller_reconnected']=True
        elif permit_trial:
            permit_pub=node.create_publisher(Bool,'platform/commissioning_test_permit',1)
            deadline=time.monotonic()+60
            while True:
                spin(.025)
                now=time.monotonic()-started
                watched = [('platform/emergency_stop',2.5),('platform/emergency_stop_mux',.25),
                           ('platform/cmd_vel_limited',.25)]
                if args.test != 'interlock-disconnect' or disconnect_worker is None:
                    watched.append(('joy_teleop/joy',.5))
                if not (args.test == 'interlock-feedback-fault' and feedback_fault_at is not None):
                    watched.append(('platform/odom',.25))
                for topic,age in watched:
                    samples=[r for r in records if r['topic']==topic]
                    if not samples or now-samples[-1]['elapsed']>age:
                        raise RuntimeError('permit trial observation stale; press physical stop')
                publishers=node.get_publishers_info_by_topic(ns+'/platform/commissioning_test_permit')
                if len(publishers)>1 or any(e.node_name!='supervised_driving_stop_observer' for e in publishers):
                    raise RuntimeError('unexpected permit publisher; press physical stop')
                stops=[r for r in records if r['topic']=='platform/emergency_stop']
                joy=[r for r in records if r['topic']=='joy_teleop/joy']
                if first_joystick_input is not None and ((stops[-1]['message']['data'] and args.stop_action != 'e-stop') or
                        (cancel_permit_on_input_loss(args.test,args.stop_action) and not held_direction_input(joy[-1]['message'], args.direction))):
                    permit_cancelled=True
                if (args.stop_action != 'budget' and first_joystick_input is not None
                        and not action_prompted and time.monotonic()-first_joystick_input>=.4):
                    print(('RELEASE DEADMAN NOW: keep the forward stick held briefly, then center it.'
                           if args.stop_action == 'deadman' else
                           'PRESS PHYSICAL STOP NOW if not already pressed; keep forward held until pressing, then release controls.'),
                          flush=True)
                    action_prompted=True
                if args.test == 'interlock-disconnect':
                    if disconnect_due(first_command,time.monotonic(),disconnect_worker is not None):
                        begin_disconnect()
                        print('CONTROLLER DISCONNECT REQUEST SENT. Release controls after disconnection.\n'
                              'If still connected/moving at the stop prompt, release controls and press physical stop.',flush=True)
                    # bluetoothctl can return an error after the radio has
                    # already disconnected. Preserve the full stop trace and
                    # verify actual link state only once the physical stop is
                    # asserted; its process return code is supporting evidence.
                if args.test == 'interlock-feedback-fault' and not stops[-1]['message']['data']:
                    if (feedback_fault_due(records, first_joystick_input, started,
                                           time.monotonic(), feedback_fault_at is not None)
                            and bounded_permit_window(first_joystick_input, time.monotonic(), permit_cancelled)):
                        bad = PumaMultiFeedback()
                        bad.header.stamp = node.get_clock().now().to_msg()
                        feedback_fault_pub.publish(bad)
                        feedback_fault_at = time.monotonic()
                        event = {'elapsed': feedback_fault_at-started, 'phase': phase,
                                 'topic': 'observer/invalid_feedback_published',
                                 'message': {'drivers_feedback': 0}}
                        records.append(event); log.write(json.dumps(event)+'\n'); log.flush()
                        print('INVALID FEEDBACK SENT. Watch for wheel stop; keep physical stop ready.\n'
                              'At the permit-window prompt, release controls and press physical stop.', flush=True)
                allow=bounded_permit_window(first_joystick_input,time.monotonic(),permit_cancelled)
                permit_pub.publish(Bool(data=allow))
                records.append({'elapsed':now,'phase':phase,'topic':'observer/test_permit_sent','message':{'data':allow}})
                log.write(json.dumps(records[-1])+'\n');log.flush()
                if first_joystick_input is not None and (not allow) and not permit_prompted:
                    print('PERMIT WINDOW COMPLETE: release controls and PRESS PHYSICAL E-STOP NOW.\n'
                          'If still moving, stop it immediately. Do not make another movement.',flush=True)
                    permit_prompted=True
                if first_joystick_input is not None and time.monotonic()-first_joystick_input>=6:
                    if (disconnected_stationary_tail(records,now) if args.test == 'interlock-disconnect'
                            else all(r['message']['data'] for r in stops[-2:]) and controller_neutral(records,now)):
                        break
                if first_release is not None and first_joystick_input is None and time.monotonic()-first_release>=15:
                    raise RuntimeError('no valid '+args.direction+' input within 15 seconds of physical release; PRESS PHYSICAL STOP NOW')
                if time.monotonic()>=deadline:
                    raise RuntimeError('bounded permit sequence incomplete; press physical stop')
        elif args.test == 'interlock-held':
            deadline=time.monotonic()+60
            while True:
                spin(.05)
                now=time.monotonic()-started
                for topic,age in [('platform/emergency_stop',2.5),('platform/emergency_stop_mux',.25),
                                  ('platform/odom',.25),('platform/cmd_vel_limited',.25),('joy_teleop/joy',.5)]:
                    samples=[r for r in records if r['topic']==topic]
                    if not samples or now-samples[-1]['elapsed']>age:
                        raise RuntimeError('held-lock observation missing or stale; press physical stop')
                if node.get_publishers_info_by_topic(ns+'/platform/commissioning_test_permit'):
                    raise RuntimeError('test-permit publisher appeared; press physical stop')
                if first_joystick_input is not None and time.monotonic()-first_joystick_input>=5:
                    stops=[r for r in records if r['topic']=='platform/emergency_stop']
                    if all(r['message']['data'] for r in stops[-2:]) and controller_neutral(records,now):
                        break
                if time.monotonic()>=deadline:
                    raise RuntimeError('held-input/re-engaged-stop sequence incomplete; keep physical stop pressed')
        elif args.test == 'e-stop-release':
            deadline = time.monotonic()+60
            prompted = False
            while True:
                spin(.05)
                now = time.monotonic()-started
                if not controller_neutral(records, now):
                    raise RuntimeError('controller input became held, non-neutral or stale during release check')
                stops = [r for r in records if r['topic'] == 'platform/emergency_stop']
                odom = [r for r in records if r['topic'] == 'platform/odom']
                if now-stops[-1]['elapsed'] > 2.5 or now-odom[-1]['elapsed'] > .25:
                    raise RuntimeError('stop or wheel observation became stale during release check')
                if args.test_interlock:
                    locks=[r for r in records if r['topic']=='platform/emergency_stop_mux']
                    if not locks or now-locks[-1]['elapsed'] > .25:
                        raise RuntimeError('test-lock observation missing/stale during neutral release; press physical e-stop')
                if first_release is not None and time.monotonic()-first_release >= 10 and not prompted:
                    print('OBSERVATION COMPLETE: press and leave the physical e-stop engaged now.', flush=True)
                    prompted = True
                    deadline = time.monotonic()+60
                if prompted and len(stops)>=2 and all(r['message']['data'] for r in stops[-2:]):
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError('timed out waiting for physical stop release or re-engagement')
        else:
            waiting_deadline = time.monotonic()+60
            while True:
                event = first_joystick_input if args.test == 'disconnect-stationary' else first_command
                tail = 20 if args.test == 'disconnect-stationary' else 10
                if time.monotonic() >= (waiting_deadline if event is None else event+tail):
                    break
                rclpy.spin_once(node, timeout_sec=.05)
                if args.test == 'disconnect' and args.disconnect_device:
                    if disconnect_due(first_command,time.monotonic(),disconnect_worker is not None):
                        begin_disconnect()
                        print('DISCONNECT REQUEST SENT. Release controller controls after disconnection.\n'
                              'If still connected at one second, release driving-enable and press the physical stop.', flush=True)
                    if first_command is not None:
                        now = time.monotonic()-started
                        for topic, max_age in [('platform/emergency_stop',2.5),('platform/odom',.25),('platform/cmd_vel_limited',.25)]:
                            samples = [r for r in records if r['topic'] == topic]
                            if not samples or now-samples[-1]['elapsed'] > max_age:
                                raise RuntimeError('observation stale during moving disconnect; press the physical stop now')
                if disconnect_worker is not None and disconnect_worker.poll() is None and time.monotonic()-disconnect_started > 8:
                    disconnect_worker.kill()
                    raise RuntimeError('Bluetooth disconnect command timed out; keep physical stop pressed')
                if disconnect_worker is not None and disconnect_worker.poll() not in (None, 0):
                    raise RuntimeError('Bluetooth disconnect command failed; keep physical stop pressed')
        report['status'] = ('recorded-awaiting-analysis-and-operator-observation'
                            if args.test == 'stationary' or report.get('controller_reconnected') or first_release is not None or first_command is not None
                            or first_joystick_input is not None else 'no-operator-input-recorded')
        report['movement_command_detected'] = first_command is not None
        report['joystick_input_detected'] = first_joystick_input is not None
        report['stop_release_detected'] = first_release is not None
        if args.test == 'interlock-disconnect' and disconnect_worker is not None:
            if disconnect_worker.poll() is None:
                disconnect_worker.kill();disconnect_worker.wait(timeout=2)
            link=subprocess.run(['/usr/bin/bluetoothctl','--timeout','1','info',args.disconnect_device],
                stdin=subprocess.DEVNULL,capture_output=True,text=True,timeout=3)
            (root/'bluetooth-after.txt').write_text(link.stdout+link.stderr)
            report['bluetooth_disconnect_confirmed']=(link.returncode==0 and 'Connected: no' in link.stdout)
            if not report['bluetooth_disconnect_confirmed']:
                report['status']='disconnect-unconfirmed'
                return 1
        if args.stop_action != 'budget':
            report['stop_action_observed'] = stop_action_at is not None
            report['stop_action_elapsed_s'] = stop_action_at-started if stop_action_at is not None else None
            report['stop_action_within_guard_budget'] = (stop_action_at is not None
                and first_joystick_input is not None
                and 0 <= stop_action_at-first_joystick_input < 1.)
            report['stop_action_evidence'] = stop_action_evidence(
                records, args.stop_action,
                stop_action_at-started if stop_action_at is not None else None)
            if not report['stop_action_evidence']['qualified_command_chain']:
                report['status'] = 'stop-action-command-chain-inconclusive'
                print('Stop action was not isolated before the guard budget; preserve the recording and keep the stop pressed.', flush=True)
                return 1
        if args.test == 'interlock-feedback-fault':
            report['invalid_feedback_published'] = feedback_fault_at is not None
            report['invalid_feedback_elapsed_s'] = feedback_fault_at-started if feedback_fault_at is not None else None
            if feedback_fault_at is not None:
                motor_after = [r for r in records if r['topic'] == 'platform/motors/cmd'
                               and r['elapsed'] >= feedback_fault_at-started]
                zeros = [r for r in motor_after if r['message']['velocity']
                         and all(math.isfinite(v) and abs(v) <= 1e-8
                                 for v in r['message']['velocity'])]
                report['first_zero_motor_command_after_fault_s'] = (
                    zeros[0]['elapsed']-(feedback_fault_at-started) if zeros else None)
                report['motor_commands_after_fault'] = len(motor_after)
            if feedback_fault_at is None:
                report['status'] = 'no-feedback-fault-injected'
                print('No bounded forward wheel motion was observed in time; no feedback fault was sent.', flush=True)
                return 1
        if args.disconnect_device:
            report['bluetooth_disconnect'] = {'device':args.disconnect_device,
                'started':disconnect_worker is not None,
                'request_elapsed_s':disconnect_started-started if disconnect_started is not None else None,
                'returncode':disconnect_worker.poll() if disconnect_worker is not None else None}
        print('Recording finished. Leave physical e-stop engaged. This is not an automatic pass.', flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        report['status'] = 'aborted-observer';report['error'] = str(error)
        print('Observer stopped: '+str(error)+'. Engage the physical e-stop.', flush=True)
        return 1
    finally:
        if permit_pub is not None:
            permit_pub.publish(Bool(data=False))
        if disconnect_worker is not None and disconnect_worker.poll() is None:
            disconnect_worker.kill(); disconnect_worker.wait(timeout=2)
        if disconnect_log is not None:
            disconnect_log.close()
        if args.disconnect_device:
            report['bluetooth_disconnect']={'device':args.disconnect_device,
                'started':disconnect_worker is not None,
                'request_elapsed_s':disconnect_started-started if disconnect_started is not None else None,
                'returncode':disconnect_worker.poll() if disconnect_worker is not None else None}
        report['elapsed_s'] = time.monotonic()-started
        report['samples'] = len(records)
        report['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (root/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        log.close();node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
