"""No controller-cycle result may hide stale/released stops or wheel commands."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parent))
from check_controller_cycles import bluetooth_state, neutral, stationary_problem
import check_controller_cycles as recorder


def safe_rows():
    return {topic: {'elapsed': 10., 'message': message} for topic, message in {
        'platform/emergency_stop': {'data': True},
        'platform/emergency_stop_mux': {'data': True},
        'platform/odom': {'twist': {'twist': {'linear': {'x': 0., 'y': 0.}, 'angular': {'z': 0.}}}},
        'platform/motors/cmd': {'velocity': [0.]*4},
        'platform/cmd_vel_limited': {'twist': {'linear': {'x': 0., 'y': 0.}, 'angular': {'z': 0.}}},
    }.items()}


class StationaryCycleTests(unittest.TestCase):
    def test_neutral_uses_motion_axes_and_both_enables(self):
        joy = {'axes': [0., 0., 1., 0., 0., 1.], 'buttons': [0]*15}
        self.assertTrue(neutral(joy))
        for button in (4, 5):
            changed = copy.deepcopy(joy); changed['buttons'][button] = 1
            self.assertFalse(neutral(changed))
        for axis in (0, 1, 3, 4):
            changed = copy.deepcopy(joy); changed['axes'][axis] = .1
            self.assertFalse(neutral(changed))
        joy['axes'][2] = float('nan')
        self.assertFalse(neutral(joy))

    def test_released_or_stale_stop_and_motion_block_cycle(self):
        rows = safe_rows()
        self.assertIsNone(stationary_problem(rows, 10.1))
        for topic in rows:
            changed = copy.deepcopy(rows)
            del changed[topic]
            self.assertIsNotNone(stationary_problem(changed, 10.1))
            changed = copy.deepcopy(rows); changed[topic]['elapsed'] = 0.
            self.assertIsNotNone(stationary_problem(changed, 10.1))
        for topic in ('platform/emergency_stop', 'platform/emergency_stop_mux'):
            changed = copy.deepcopy(rows); changed[topic]['message']['data'] = False
            self.assertIsNotNone(stationary_problem(changed, 10.1))
        changed = copy.deepcopy(rows)
        changed['platform/motors/cmd']['message']['velocity'][0] = .01
        self.assertIsNotNone(stationary_problem(changed, 10.1))
        changed['platform/motors/cmd']['message']['velocity'][0] = float('nan')
        self.assertIsNotNone(stationary_problem(changed, 10.1))

    def test_bluetooth_command_success_alone_does_not_establish_link_state(self):
        self.assertIs(bluetooth_state({'returncode': 0, 'stdout': '  Connected: no\n'}), False)
        self.assertIs(bluetooth_state({'returncode': 0, 'stdout': '  Connected: yes\n'}), True)
        self.assertIsNone(bluetooth_state({'returncode': 0, 'stdout': 'Successful disconnected\n'}))
        self.assertIsNone(bluetooth_state({'returncode': 1, 'stdout': 'Connected: no\n'}))

    def test_cli_requires_operator_before_accessing_hardware(self):
        result = subprocess.run([sys.executable, str(Path(__file__).with_name('check_controller_cycles.py')),
                                 '--run-dir', '/tmp/unused-cycle-test'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('operator must keep the physical e-stop pressed', result.stderr)

    def run_simulated_cycle(self, bad_reconnect=False, cycles=1):
        """Exercise main's real control flow with no ROS or Bluetooth access."""
        state = {'held': False, 'connected': True, 'disconnects': 0}
        class Node:
            def __init__(self):
                self.callbacks = {}
            def create_subscription(self, kind, topic, callback, qos):
                self.callbacks[topic] = callback
            def get_publishers_info_by_topic(self, topic):
                return []
            def destroy_node(self):
                pass
        node = Node()
        def spin_once(node, timeout_sec):
            time.sleep(.005)
            messages = {topic: row['message'] for topic, row in safe_rows().items()}
            messages['joy_teleop/joy'] = {'axes': [0., .3 if state['held'] else 0., 1., 0., 0., 1.],
                                         'buttons': [0]*4+[int(state['held']), 0]+[0]*9}
            if state['held']:
                messages['joy_teleop/cmd_vel_combined'] = {'twist': {
                    'linear': {'x': .12, 'y': 0.}, 'angular': {'z': 0.}}}
            for topic, callback in node.callbacks.items():
                if topic in messages:
                    callback(messages[topic])
        def output(message, **kwargs):
            if (message.startswith('CYCLE ') and ': STOP STAYS PRESSED.' in message) or message.startswith('NEUTRAL RECONNECT RECORDED'):
                state['held'] = True
            if message.startswith('DISCONNECTED CONFIRMED') or message.startswith('RESTORED INPUT RECORDED'):
                state['held'] = False
            if message.startswith('RECONNECT NOW'):
                state.update(connected=True, held=bad_reconnect)
        def bluetooth(argv, **kwargs):
            self.assertEqual(argv[:3], ['/usr/bin/bluetoothctl', '--timeout', '2'])
            if argv[3] == 'disconnect':
                state['connected'] = False
                state['disconnects'] += 1
            return types.SimpleNamespace(returncode=0, stderr='',
                                         stdout='Connected: '+('yes' if state['connected'] else 'no'))
        modules = {
            'rclpy': types.SimpleNamespace(init=lambda **kwargs: None, create_node=lambda *a, **k: node,
                                          spin_once=spin_once, ok=lambda: True, shutdown=lambda: None),
            'rclpy.qos': types.SimpleNamespace(qos_profile_sensor_data=object()),
            'rosidl_runtime_py.convert': types.SimpleNamespace(message_to_ordereddict=lambda msg: msg),
            'std_msgs.msg': types.SimpleNamespace(Bool=object),
            'nav_msgs.msg': types.SimpleNamespace(Odometry=object),
            'sensor_msgs.msg': types.SimpleNamespace(JointState=object, Joy=object),
            'geometry_msgs.msg': types.SimpleNamespace(TwistStamped=object),
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); run = root/'artifacts/hardware/cycles'
            with (patch.object(recorder, 'ROOT', root),
                  patch.object(recorder, 'identity', return_value='/test_robot'),
                  patch.object(recorder, 'desired_files', return_value={}),
                  patch.object(recorder, 'controllers', return_value=['AC:36:1B:89:A1:0A']),
                  patch.object(recorder, 'run_probe', return_value={'ok': True, 'routing_ok': True,
                               'mux_stop_routing_ok': True, 'mux_stop_engaged': True}),
                  patch.object(recorder.subprocess, 'run', side_effect=bluetooth),
                  patch('builtins.print', side_effect=output),
                  patch.dict(sys.modules, modules),
                  patch.dict(recorder.os.environ),
                  patch.object(sys, 'argv', ['cycles', '--operator-confirmed', '--cycles', str(cycles), '--run-dir', str(run)])):
                code = recorder.main()
            return code, json.loads((run/'result.json').read_text()), state

    def test_full_stationary_cycle_has_no_publishers_and_confirms_restored_input(self):
        code, report, state = self.run_simulated_cycle(cycles=2)
        self.assertEqual(code, 0)
        self.assertEqual(report['status'], 'stationary-controller-cycles-observed')
        self.assertEqual(len(report['cycles']), 2)
        self.assertEqual(state['disconnects'], 2)
        self.assertEqual(report['ros_publishers_created'], 0)
        cycle = report['cycles'][0]
        self.assertLess(cycle['held_input_s'], cycle['disconnect_confirmed_s'])
        self.assertLess(cycle['reconnect_confirmed_s'], cycle['restored_input_s'])

    def test_non_neutral_reconnect_stops_before_accepting_restored_input(self):
        code, report, _ = self.run_simulated_cycle(bad_reconnect=True)
        self.assertEqual(code, 1)
        self.assertIn('non-neutral controls during reconnect', report['error'])
        self.assertEqual(report['cycles'], [])


if __name__ == '__main__':
    unittest.main()
