import subprocess
import sys
import unittest

import check_raised_wheel_link_fault as fault

MOVING = [0.66, 0.66, 0.66, 0.66]
STOPPED = [0.0, 0.0, 0.0, 0.0]
CMD = [0.66] * 4


def series(start, end, value, step=0.05):
    return [(round(start + i * step, 3), value) for i in range(int(round((end - start) / step)))]


class MotionStartTests(unittest.TestCase):
    def test_requires_held_command_and_moving_wheels(self):
        commands = series(0, 0.5, CMD)
        self.assertTrue(fault.motion_started(commands, [(0.44, MOVING)], 0.45))
        self.assertFalse(fault.motion_started(commands[:4], [(0.14, MOVING)], 0.15))   # still ramping
        self.assertFalse(fault.motion_started(commands, [(0.44, [0.66, 0.66, 0.1, 0.66])], 0.45))
        self.assertFalse(fault.motion_started(commands, [(0.2, MOVING)], 0.45))       # stale feedback
        self.assertFalse(fault.motion_started(commands + [(0.5, [0.0] * 4)], [(0.49, MOVING)], 0.5))


class EvaluateTests(unittest.TestCase):
    def d6(self, **change):
        values = dict(fault='feedback', fault_at=1.0, cleared_at=2.0,
                      commands=series(0, 1.3, CMD) + series(1.3, 3, [0.0] * 4),
                      source_commands=series(0, 2.2, [0.05, 0.0, 0.0]),
                      feedback=series(0, 1.0, MOVING) + series(2.0, 3, STOPPED),
                      controller_states=series(0, 1.3, 'active') + series(1.3, 3, 'inactive'),
                      stop_pressed_end=True)
        values.update(change)
        return fault.evaluate(**values)

    def test_d6_passes_when_plugin_zeroes_and_latches(self):
        report = self.d6()
        self.assertTrue(report['passed'], report)
        self.assertEqual(report['first_zero_command_after_fault_s'], 0.3)

    def test_d6_fails_on_late_zero_restart_moving_or_released_stop(self):
        self.assertFalse(self.d6(commands=series(0, 1.6, CMD) + series(1.6, 3, [0.0] * 4))['passed'])
        self.assertFalse(self.d6(controller_states=series(0, 1.3, 'active') + series(1.3, 2.2, 'inactive')
                                 + series(2.2, 3, 'active'))['passed'])
        self.assertFalse(self.d6(controller_states=series(0, 3, 'active'))['passed'])
        self.assertFalse(self.d6(feedback=series(0, 1.0, MOVING) + series(2.0, 3, MOVING))['passed'])
        self.assertFalse(self.d6(stop_pressed_end=False)['passed'])
        self.assertFalse(self.d6(fault_at=None)['passed'])
        self.assertFalse(self.d6(source_commands=series(0, 1.3, [0.05, 0.0, 0.0]))['passed'])
        self.assertFalse(self.d6(dropped_packets=0)['passed'])
        self.assertFalse(self.d6(feedback=series(0, 1.0, MOVING) + [(1.3, MOVING)]
                                 + series(2.0, 3, STOPPED))['passed'])

    def test_d7_needs_wheels_stopped_by_motor_watchdog_while_blocked(self):
        base = dict(fault='command-link', fault_at=1.0, cleared_at=2.0, commands=series(0, 3, CMD),
                    controller_states=series(0, 3, 'active'), stop_pressed_end=True)
        report = fault.evaluate(feedback=series(0, 1.3, MOVING) + series(1.3, 3, STOPPED), **base)
        self.assertTrue(report['passed'], report)
        self.assertEqual(report['wheels_stopped_after_fault_s'], 0.3)
        self.assertFalse(fault.evaluate(feedback=series(0, 2.0, MOVING) + series(2.0, 3, STOPPED), **base)['passed'])
        ended = series(0, 1.3, CMD) + series(1.3, 3, [0.0] * 4)
        self.assertFalse(fault.evaluate(feedback=series(0, 1.3, MOVING) + series(1.3, 3, STOPPED),
                                        source_commands=ended, **base)['passed'])

    def test_malformed_or_momentary_stop_never_passes(self):
        base = dict(fault='command-link', fault_at=1.0, cleared_at=2.0, commands=series(0, 3, CMD),
                    controller_states=series(0, 3, 'active'), stop_pressed_end=True)
        momentary = series(0, 1.3, MOVING) + [(1.3, STOPPED)] + series(1.35, 3, MOVING)
        self.assertFalse(fault.evaluate(feedback=momentary, **base)['passed'])
        malformed = series(0, 1.3, MOVING) + [(1.3, [0.0, 0.0])] + series(1.35, 3, STOPPED)
        self.assertFalse(fault.evaluate(feedback=malformed, **base)['passed'])


class SustainedStopTests(unittest.TestCase):
    def test_real_20hz_jitter_does_not_defeat_the_sustained_stop(self):
        # D6 trial 1: 50 ms stopped samples after the block cleared, never exactly 0.25 s apart.
        samples = [(19.495 + i * .0499, STOPPED) for i in range(20)]
        self.assertEqual(fault.first_sustained_stop(samples, 19.44), 19.495)

    def test_motion_inside_the_window_or_a_feedback_gap_still_fails(self):
        self.assertIsNone(fault.first_sustained_stop([(1.0, STOPPED), (1.1, MOVING), (1.35, STOPPED)], 1.0))   # motion inside the window
        self.assertIsNone(fault.first_sustained_stop([(1.0, STOPPED), (1.5, STOPPED)], 1.0))   # 0.5 s gap
        self.assertIsNone(fault.first_sustained_stop([(1.0, STOPPED), (1.1, STOPPED)], 1.0))   # too short


class WatchdogTests(unittest.TestCase):
    def test_program_compiles_and_names_the_exact_rule(self):
        rule = fault.PATHS['feedback'] + ['-m', 'comment', '--comment', 'dynamo-link-fault-x', '-j', 'DROP']
        program = fault.watchdog_program(rule, 4242, 1.0)
        compile(program, 'watchdog', 'exec')
        self.assertIn(repr(rule), program)
        self.assertIn('owner=4242', program)

    def test_paths_isolate_one_direction_of_the_bridge(self):
        self.assertEqual(fault.PATHS['feedback'][:5], ['INPUT', '-s', fault.MCU, '-d', fault.PC])
        self.assertEqual(fault.PATHS['command-link'][:5], ['OUTPUT', '-s', fault.PC, '-d', fault.MCU])


class WatchdogBehaviourTests(unittest.TestCase):
    """Run the real cleanup loop against a fake iptables that keeps rule state in a file."""

    def run_watchdog(self, owner_alive, block_s):
        import os
        import tempfile
        import time
        from unittest import mock
        with tempfile.TemporaryDirectory() as directory:
            state = os.path.join(directory, 'rule')
            fake = os.path.join(directory, 'iptables')
            with open(fake, 'w') as stream:
                stream.write('#!/bin/sh\ncase "$2" in -C) test -e %s;; -D) rm -f %s;; esac\n' % (state, state))
            os.chmod(fake, 0o755)
            open(state, 'w').close()
            owner = subprocess.Popen(['sleep', '30'])
            try:
                with mock.patch.object(fault, 'IPTABLES', fake):
                    program = fault.watchdog_program(['INPUT', '-j', 'DROP'], owner.pid, block_s)
                watchdog = subprocess.Popen([sys.executable, '-c', program], stdout=subprocess.DEVNULL)
                if not owner_alive:
                    owner.kill(); owner.wait()
                started = time.monotonic()
                while os.path.exists(state) and time.monotonic() - started < 5:
                    time.sleep(0.01)
                return time.monotonic() - started, os.path.exists(state)
            finally:
                owner.kill(); owner.wait()
                watchdog.wait(timeout=5)

    def test_removes_rule_after_block_bound(self):
        elapsed, present = self.run_watchdog(True, 0.3)
        self.assertFalse(present)
        self.assertGreaterEqual(elapsed, 0.3)
        self.assertLess(elapsed, 1.0)

    def test_removes_rule_at_once_when_owner_dies(self):
        elapsed, present = self.run_watchdog(False, 1.5)
        self.assertFalse(present)
        self.assertLess(elapsed, 0.5)


class CliTests(unittest.TestCase):
    def test_refuses_without_root(self):
        result = subprocess.run([sys.executable, fault.__file__, '--fault', 'feedback', '--run-dir', '/tmp/x'],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('sudo', result.stderr)


if __name__ == '__main__':
    unittest.main()
