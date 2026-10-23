"""The capture wrapper must never start motion observation without CAN capture."""
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).parent))
import capture_wheel_field_trial as trial


class CaptureTrialTests(unittest.TestCase):
    def test_observer_command_retains_one_shot_interlock(self):
        command = trial.command_for_observer(Path('/tmp/trial/observer'))
        self.assertIn('interlock-permit', command)
        self.assertIn('--test-interlock', command)
        self.assertIn('--permit-motion-confirmed', command)
        self.assertEqual(command[command.index('--direction')+1], 'forward')

    def test_pulse_command_retains_physical_stop_gate_and_short_interlock(self):
        command = trial.command_for_observer(Path('/tmp/trial/observer'), 'pulse')
        self.assertIn('commissioning_forward_pulse.py', command[1])
        for option in ('--execute', '--operator-confirmed', '--test-interlock', '--ramp'):
            self.assertIn(option, command)
        self.assertEqual(command[command.index('--stop-mode')+1], 'timeout')
        with self.assertRaises(ValueError):
            trial.command_for_observer(Path('/tmp/trial/observer'), 'unknown')

    def test_handover_command_retains_live_stop_and_bounded_source_gate(self):
        command = trial.command_for_observer(Path('/tmp/trial/observer'), 'handover')
        self.assertIn('check_live_driving_handover.py', command[1])
        self.assertIn('--operator-confirmed', command)
        self.assertIn('--bounded-motion-confirmed', command)

    def test_manual_pulse_requires_installed_mode_and_one_ramped_forward_command(self):
        command = trial.command_for_observer(Path('/tmp/trial/observer'), 'manual-pulse')
        self.assertIn('commissioning_forward_pulse.py', command[1])
        for option in ('--execute', '--operator-confirmed', '--original-manual-speeds',
                       '--ramp'):
            self.assertIn(option, command)
        self.assertEqual(command[command.index('--axis')+1], 'forward')
        self.assertEqual(command[command.index('--stop-mode')+1], 'zero')

    def test_candump_start_failure_never_launches_observer(self):
        with TemporaryDirectory() as directory:
            failed_capture = Mock()
            failed_capture.poll.return_value = 1
            with patch.object(trial.subprocess, 'Popen', return_value=failed_capture) as launch, \
                 patch.object(trial.time, 'sleep'):
                with self.assertRaisesRegex(RuntimeError, 'no motion trial launched'):
                    trial.capture_observer(Path(directory))
            self.assertEqual(launch.call_count, 1)

    def test_observer_exit_stops_capture(self):
        with TemporaryDirectory() as directory:
            capture = Mock()
            capture.poll.return_value = None
            observer = Mock()
            observer.wait.return_value = 0
            observer.poll.return_value = 0
            capture.wait.return_value = -2
            with patch.object(trial.subprocess, 'Popen', side_effect=[capture, observer]) as launch, \
                 patch.object(trial, 'wait_for_capture'):
                self.assertEqual(trial.capture_observer(Path(directory)), (0, False))
            self.assertEqual(launch.call_count, 2)
            capture.send_signal.assert_called_once()

    def test_empty_capture_never_launches_observer(self):
        with TemporaryDirectory() as directory:
            capture = Mock()
            capture.poll.return_value = None
            with patch.object(trial.subprocess, 'Popen', return_value=capture) as launch, \
                 patch.object(trial.time, 'sleep'), \
                 patch.object(trial.time, 'monotonic', side_effect=[0., 0., 4.]):
                with self.assertRaisesRegex(RuntimeError, 'no CAN frames recorded'):
                    trial.capture_observer(Path(directory))
            self.assertEqual(launch.call_count, 1)
            capture.send_signal.assert_called_once()

    def test_capture_failure_during_observation_cancels_observer(self):
        with TemporaryDirectory() as directory:
            capture = Mock()
            capture.poll.return_value = 1
            observer = Mock()
            observer.poll.side_effect = [None, None, 1]
            observer.wait.return_value = 1
            with patch.object(trial.subprocess, 'Popen', side_effect=[capture, observer]), \
                 patch.object(trial, 'wait_for_capture'):
                self.assertEqual(trial.capture_observer(Path(directory)), (1, True))
            observer.send_signal.assert_called_once()


if __name__ == '__main__':
    unittest.main()
