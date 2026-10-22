"""Test pulse deadline and conservative feedback abort boundaries offline."""
import sys
import subprocess
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).parent))
from commissioning_forward_pulse import requested_speed, feedback_problem, populate_command, source_silence_qualifies, physical_stop_qualifies, motor_fault_problem, motor_status_is_estop_fault, motor_status_is_estop_fault_onset, physical_stop_tail_deadline, stop_confirmation_pending, AXES


class PulseTests(unittest.TestCase):
    def test_source_silence_requires_stop_before_independent_budget(self):
        self.assertTrue(source_silence_qualifies(.45, .62, 0., False))
        for motion, zero, resumed in [(None,.62,False),(.45,None,False),
                                       (.45,.99,False),(.45,.62,True),
                                       (.7,.62,False)]:
            self.assertFalse(source_silence_qualifies(motion, zero, 0., resumed))

    def test_deadline_cannot_extend_nonzero_request(self):
        for elapsed in [-1, 1, 1.001, 10, float('inf'), float('nan')]:
            self.assertEqual(requested_speed(elapsed), 0)
        self.assertEqual(requested_speed(0), .05)
        self.assertEqual(requested_speed(.999), .05)

    def test_ramp_bounds_slope_and_duration(self):
        samples = [requested_speed(i/1000, True) for i in range(1001)]
        self.assertEqual(samples[0], 0.)
        self.assertEqual(samples[-1], 0.)
        self.assertEqual(max(samples), .05)
        self.assertLessEqual(max(abs(b-a) for a,b in zip(samples,samples[1:])), .000200000001)
        self.assertAlmostEqual(sum(samples)*.001, .0375)
        for elapsed in [-1, 1.001, float('nan'), float('inf')]:
            self.assertEqual(requested_speed(elapsed, True), 0.)

    def test_timeout_profile_loses_publishing_at_peak_but_never_extends_duration(self):
        self.assertEqual(requested_speed(.999, True, 'timeout'), .05)
        self.assertEqual(requested_speed(.1, True, 'timeout'), .020000000000000004)
        for elapsed in [1., 1.01, 20, float('nan')]:
            self.assertEqual(requested_speed(elapsed, True, 'timeout'), 0.)

    def test_interlocked_command_loss_stops_source_at_half_second(self):
        self.assertEqual(requested_speed(.25, True, 'timeout', .5), .05)
        self.assertEqual(requested_speed(.499, True, 'timeout', .5), .05)
        for elapsed in [.5, .501, 1., float('nan')]:
            self.assertEqual(requested_speed(elapsed, True, 'timeout', .5), 0.)

    def test_physical_stop_profile_is_bounded_and_has_no_end_ramp(self):
        self.assertEqual(requested_speed(.25, True, 'physical-e-stop', 2.), .05)
        self.assertEqual(requested_speed(1.999, True, 'physical-e-stop', 2.), .05)
        self.assertEqual(requested_speed(2., True, 'physical-e-stop', 2.), 0.)

    def test_physical_stop_requires_recent_motor_motion_then_zero_under_continued_source(self):
        good = (10., 10.7, 10.6, 10.82, 10.81, False, False, 3, True)
        self.assertTrue(physical_stop_qualifies(*good))
        for changed in [
            (10., None, 10.6, 10.82, 10.81, False, False, 3, True),
            (10., 12.1, 12., 12.2, 12.2, False, False, 3, True),
            (10., 10.7, None, 10.82, 10.81, False, False, 3, True),
            (10., 10.7, 10.3, 10.82, 10.81, False, False, 3, True),
            (10., 10.7, 10.6, None, 10.81, False, False, 3, True),
            (10., 10.7, 10.6, 10.82, 11.3, False, False, 3, True),
            (10., 10.7, 10.6, 10.6, 10.81, False, False, 3, True),
            (10., 10.7, 10.6, 10.82, 10.6, False, False, 3, True),
            (10., 10.7, 10.6, 10.82, 10.81, True, False, 3, True),
            (10., 10.7, 10.6, 10.82, 10.81, False, True, 3, True),
            (10., 10.7, 10.6, 10.82, 10.81, False, False, 1, True),
            (10., 10.7, 10.6, 10.82, 10.81, False, False, 3, False)]:
            self.assertFalse(physical_stop_qualifies(*changed))

    def test_physical_stop_allows_motor_fault_to_precede_stop_report_briefly(self):
        self.assertIsNone(motor_fault_problem(
            'physical-e-stop', 'pulse', None, 10., 10.10))
        self.assertIsNone(motor_fault_problem(
            'physical-e-stop', 'pulse', 10.12, 10., 10.20))
        self.assertEqual(motor_fault_problem(
            'physical-e-stop', 'pulse', None, 10., 11.21),
            'motor status fault was not followed by physical-stop report')
        self.assertEqual(motor_fault_problem(
            'zero', 'pulse', None, 10., 10.),
            'motor status fault or speed mode lost during pulse')
        self.assertIsNone(motor_fault_problem(
            'physical-e-stop', 'cleanup', None, 10., 11.))

    def test_physical_stop_tail_never_extends_original_source_window(self):
        self.assertAlmostEqual(physical_stop_tail_deadline(10., 2., 10.7), 11.05)
        self.assertEqual(physical_stop_tail_deadline(10., 2., 11.9), 12.)

    def test_only_exact_four_wheel_estop_fault_gets_report_grace(self):
        good = {'complete': True, 'drivers': [
            {'mode': 2, 'fault': 8} for _ in range(4)]}
        self.assertTrue(motor_status_is_estop_fault(good))
        for changed in [
                {'complete': False, 'drivers': good['drivers']},
                {'complete': True, 'drivers': good['drivers'][:3]},
                {'complete': True, 'drivers': [*good['drivers'][:3], {'mode': 1, 'fault': 8}]},
                {'complete': True, 'drivers': [*good['drivers'][:3], {'mode': 2, 'fault': 4}]}]:
            self.assertFalse(motor_status_is_estop_fault(changed))

    def test_physical_stop_mode_rejects_missing_operator_or_mode_flags(self):
        script=str(Path(__file__).with_name('commissioning_forward_pulse.py'))
        base=[sys.executable,script,'--execute','--stop-mode','physical-e-stop',
              '--run-dir','/tmp/not-used']
        for flags in ([],['--operator-confirmed'],
                      ['--operator-confirmed','--original-manual-speeds']):
            result=subprocess.run(base+flags,capture_output=True,text=True)
            self.assertEqual(result.returncode,2)

    def test_interlocked_execution_rejects_other_axes_and_stop_modes_before_robot_access(self):
        script=str(Path(__file__).with_name('commissioning_forward_pulse.py'))
        for extra in (['--axis','left','--stop-mode','timeout'],
                      ['--axis','forward','--stop-mode','zero']):
            result=subprocess.run([sys.executable,script,'--execute','--operator-confirmed',
                '--test-interlock',*extra,'--run-dir','/tmp/not-used'],capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('interlocked execution supports only forward publishing silence',result.stderr)

    def test_manual_diagnostic_rejects_other_axes_and_combined_modes_before_robot_access(self):
        script=str(Path(__file__).with_name('commissioning_forward_pulse.py'))
        for extra, message in [
            (['--original-manual-speeds', '--axis', 'left'],
             'original-manual diagnostic execution supports only forward'),
            (['--original-manual-speeds', '--test-interlock'], 'choose one installed guard mode')]:
            result=subprocess.run([sys.executable,script,'--execute','--operator-confirmed',
                *extra,'--run-dir','/tmp/not-used'],capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn(message,result.stderr)

    def test_stop_and_missing_feedback_abort(self):
        self.assertIsNone(feedback_problem(False, .1, (.05, 0, 0), .1, .1))
        for stop, stop_age, odom_age, limited_age in [(None, 0, 0, 0), (True, 0, 0, 0),
                (False, 2.501, 0, 0), (False, 0, .251, 0), (False, 0, 0, .251)]:
            self.assertIsNotNone(feedback_problem(stop, stop_age, (0, 0, 0), odom_age, limited_age))

    def test_wrong_direction_and_excess_feedback_abort(self):
        for v in [(-.02, 0, 0), (0, .021, 0), (0, 0, .051), (.076, 0, 0), (float('nan'), 0, 0)]:
            self.assertIsNotNone(feedback_problem(False, 0, v, 0, 0))

    def test_stop_and_motion_commands_serialize_with_actual_ros_types(self):
        from geometry_msgs.msg import TwistStamped
        from builtin_interfaces.msg import Time
        from rclpy.serialization import serialize_message, deserialize_message
        for speed in [0, 0.0, .05]:
            msg = populate_command(TwistStamped(), Time(sec=123), speed)
            self.assertIsInstance(msg.twist.linear.x, float)
            decoded = deserialize_message(serialize_message(msg), TwistStamped)
            self.assertEqual(decoded.twist.linear.x, float(speed))
            self.assertEqual(decoded.header.stamp.sec, 123)
            self.assertEqual(decoded.header.frame_id, 'base_link')

    def test_each_axis_has_exactly_one_bounded_component_and_feedback_gate(self):
        from geometry_msgs.msg import TwistStamped
        from builtin_interfaces.msg import Time
        from rclpy.serialization import serialize_message, deserialize_message
        for axis, (index, sign) in AXES.items():
            with self.subTest(axis=axis):
                msg = deserialize_message(serialize_message(populate_command(TwistStamped(), Time(sec=123), .05, axis)), TwistStamped)
                values = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]
                expected = [0., 0., 0.]; expected[index] = sign*(.1 if index == 2 else .05)
                self.assertEqual(values, expected)
                self.assertIsNone(feedback_problem(False, 0, values, 0, 0, axis))
                self.assertIsNotNone(feedback_problem(False, 0, [-v for v in values], 0, 0, axis))
                zero = populate_command(TwistStamped(), Time(sec=123), 0, axis)
                self.assertEqual([zero.twist.linear.x, zero.twist.linear.y, zero.twist.angular.z], [0., 0., 0.])



class ReboundTests(unittest.TestCase):
    def test_small_rebound_after_a_measured_forward_run_is_tolerated_only_for_the_estop_trial(self):
        rebound = (-.0119, 0, 0)   # run 11, 0.2 s after braking from 0.05 m/s
        self.assertIsNotNone(feedback_problem(False, 0, rebound, 0, 0))
        self.assertIsNone(feedback_problem(False, 0, rebound, 0, 0, rebound_after_peak=.0506))
        self.assertIsNotNone(feedback_problem(False, 0, rebound, 0, 0, rebound_after_peak=.01))   # never moved forward
        self.assertIsNotNone(feedback_problem(False, 0, (-.031, 0, 0), 0, 0, rebound_after_peak=.0506))  # real reversal


class EstopFaultOnsetTests(unittest.TestCase):
    @staticmethod
    def status(*faults, mode=2):
        return {'complete': True, 'drivers': [{'name': f'w{i}', 'mode': mode, 'fault': f} for i, f in enumerate(faults)]}

    def test_run_10_partial_sample_is_onset_not_a_motor_problem(self):
        self.assertTrue(motor_status_is_estop_fault_onset(self.status(8, 8, 0, 0)))   # run 10, 1.16 s
        self.assertTrue(motor_status_is_estop_fault_onset(self.status(0, 0, 0, 8)))
        self.assertTrue(motor_status_is_estop_fault_onset(self.status(8, 8, 8, 8)))

    def test_other_faults_modes_or_incomplete_status_still_abort(self):
        self.assertFalse(motor_status_is_estop_fault_onset(self.status(0, 0, 0, 0)))
        self.assertFalse(motor_status_is_estop_fault_onset(self.status(8, 8, 0, 4)))
        self.assertFalse(motor_status_is_estop_fault_onset(self.status(8, 8, 0, 0, mode=1)))
        self.assertFalse(motor_status_is_estop_fault_onset(self.status(8, 8, 8)))
        self.assertFalse(motor_status_is_estop_fault_onset({'complete': False, 'drivers': self.status(8, 8, 8, 8)['drivers']}))


class StopConfirmationPendingTests(unittest.TestCase):
    def test_waits_for_slow_report_only_after_a_seen_estop_fault(self):
        # Run 6: fault seen at 1.68 s, 1 Hz report landed at 2.02 s, after the 2.0 s window.
        self.assertTrue(stop_confirmation_pending('physical-e-stop', None, 1.68, 2.0))
        self.assertTrue(stop_confirmation_pending('physical-e-stop', None, 1.68, 2.88))
        self.assertFalse(stop_confirmation_pending('physical-e-stop', None, 1.68, 2.89))  # grace spent
        self.assertFalse(stop_confirmation_pending('physical-e-stop', 2.02, 1.68, 2.1))   # report arrived
        self.assertFalse(stop_confirmation_pending('physical-e-stop', None, None, 2.1))   # no fault: no wait
        self.assertFalse(stop_confirmation_pending('timeout', None, 1.68, 2.0))           # other modes unchanged


if __name__ == '__main__':
    unittest.main()
