"""Offline boundary tests for the physical commissioning guard."""
import importlib.util
import argparse
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('guard', Path(__file__).with_name('commissioning_speed_guard.py'))
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


class GuardTests(unittest.TestCase):
    def test_original_manual_limits_preserve_l1_r1_and_diagonals(self):
        for values in [( .4, -.4, 0., 0., 0., .5),
                       (-1.2, 1.2, 0., 0., 0., -1.5)]:
            result = guard.clamp_command(values, 1_000_000_000, 1_000_000_000,
                                         'base_link', original_manual_speeds=True)
            self.assertEqual(result, (values[0], values[1], values[5]))
        result = guard.clamp_command((1.7e308,-1.7e308,0,0,0,1.7e308),
                                    1_000_000_000,1_000_000_000,'base_link',True)
        self.assertEqual(result,(1.2,-1.2,1.5))

    def test_original_manual_mode_keeps_stop_and_source_silence_protection(self):
        gate = guard.Gate(original_manual_speeds=True)
        values = (.4,0,0,0,0,.5)
        stamp = 1_000_000_000
        self.assertIsNone(gate.command(values,stamp,stamp,10,'base_link'))
        gate.update_stop(False,10)
        self.assertEqual(gate.command(values,stamp,stamp,10.01,'base_link'),(.4,0,.5))
        self.assertTrue(gate.needs_zero(stamp+50_000_000,10.12))
        self.assertIsNone(gate.command(values,stamp,stamp+100_000_001,10.13,'base_link'))
        self.assertIsNone(gate.command((float('nan'),0,0,0,0,0),stamp,stamp,10.14,'base_link'))
        self.assertIsNone(gate.command(values,stamp,stamp,12.51,'base_link'))
        gate.update_stop(True,13)
        self.assertIsNone(gate.command(values,stamp,stamp,13,'base_link'))
        gate.update_stop(False,13.1)
        self.assertTrue(gate.needs_zero(stamp,13.11))

    def test_manual_speeds_cannot_be_combined_with_test_permits(self):
        with self.assertRaises(ValueError):
            guard.Gate(require_test_permit=True,original_manual_speeds=True)

    def test_test_mode_blocks_preflight_and_missing_or_stale_permit(self):
        gate=guard.Gate(require_test_permit=True);gate.update_stop(False,10)
        self.assertTrue(gate.stop_blocks(10))
        gate.update_permit(True,10)
        self.assertFalse(gate.stop_blocks(10.1))
        self.assertTrue(gate.stop_blocks(10.201))
        gate.update_permit(False,10.21)
        self.assertTrue(gate.needs_zero(1_000_000_000,10.21))

    def test_heartbeat_cannot_extend_motion_budget_and_stop_cycle_required(self):
        gate=guard.Gate(require_test_permit=True);gate.update_stop(False,10)
        for at in [10,10.1,10.5,10.9,11]:gate.update_permit(True,at)
        self.assertTrue(gate.stop_blocks(11))
        gate.update_permit(False,11.1);gate.update_permit(True,11.2)
        self.assertTrue(gate.stop_blocks(11.2))
        gate.update_stop(True,12);gate.update_stop(False,12.1)
        self.assertTrue(gate.stop_blocks(12.1))
        gate.update_permit(True,12.2)
        self.assertFalse(gate.stop_blocks(12.2))
        self.assertTrue(gate.needs_zero(1_000_000_000,12.2))

    def test_missing_source_commands_zero_before_one_second_permit_expires(self):
        gate=guard.Gate(require_test_permit=True)
        gate.update_stop(False,10.)
        gate.update_permit(True,10.)
        values=(.05,0.,0.,0.,0.,0.)
        self.assertIsNotNone(gate.command(values,1_000_000_000,1_000_000_000,10.05,'base_link'))
        gate.update_permit(True,10.4)
        self.assertIsNotNone(gate.command(values,1_400_000_000,1_400_000_000,10.45,'base_link'))
        gate.update_permit(True,10.56)
        self.assertFalse(gate.stop_blocks(10.56))
        self.assertTrue(gate.needs_zero(1_490_000_000,10.56))

    def clamp(self, values, stamp=1_000_000_000, now=1_000_000_000, frame='base_link'):
        return guard.clamp_command(values, stamp, now, frame)

    def test_boost_and_diagonal_norm_are_bounded(self):
        for x, y, yaw in [(1.2, 1.2, 1.5), (-1.2, 1.2, -1.5), (.04, .04, .1), (1.7e308, 1.7e308, 1.7e308)]:
            with self.subTest(x=x):
                result = self.clamp((x, y, 0, 0, 0, yaw))
                self.assertLessEqual(math.hypot(*result[:2]), .05 + 1e-15)
                self.assertLessEqual(abs(result[2]), .1)
                self.assertGreater(result[0] * (1 if x>0 else -1), 0)

    def test_small_commands_preserve_components(self):
        self.assertEqual(self.clamp((.01, -.02, 0, 0, 0, .03)), (.01, -.02, .03))

    def test_nonfinite_and_nonplanar_inputs_fail_closed(self):
        for axis in range(6):
            for invalid in [float('nan'), float('inf'), -float('inf')]:
                values=[0.0]*6; values[axis]=invalid
                self.assertIsNone(self.clamp(values))
        for axis in [2,3,4]:
            values=[0.0]*6; values[axis]=.001
            self.assertIsNone(self.clamp(values))

    def test_frame_and_age_validation(self):
        for kwargs in [dict(frame='map'),dict(stamp=0),dict(stamp=899_999_999),dict(stamp=1_005_000_001)]:
            self.assertIsNone(self.clamp((.01,0,0,0,0,0),**kwargs))
        self.assertIsNotNone(self.clamp((.01,0,0,0,0,0),frame=''))

    def test_stop_startup_true_and_stale_block(self):
        gate=guard.Gate()
        self.assertIsNone(gate.command((.01,0,0,0,0,0),1_000_000_000,1_000_000_000,10,'base_link'))
        gate.update_stop(False,10)
        self.assertEqual(gate.command((.01,0,0,0,0,0),1_000_000_000,1_000_000_000,10.01,'base_link'),(.01,0,0))
        self.assertTrue(gate.needs_zero(1_010_000_000,12.51))
        gate.update_stop(True,13)
        self.assertIsNone(gate.command((.01,0,0,0,0,0),1_000_000_000,1_000_000_000,13,'base_link'))

    def test_physical_stop_overrides_continued_test_permits(self):
        gate = guard.Gate(require_test_permit=True)
        gate.update_stop(False, 10.)
        gate.update_permit(True, 10.01)
        self.assertFalse(gate.stop_blocks(10.02))
        gate.update_stop(True, 10.40)
        for now in (10.41, 10.50, 10.75):
            gate.update_permit(True, now)
            self.assertTrue(gate.stop_blocks(now))
            self.assertTrue(gate.needs_zero(1_000_000_000, now))

    def test_release_does_not_replay_and_two_clocks_expire(self):
        gate=guard.Gate();gate.update_stop(False,10)
        gate.command((.01,0,0,0,0,0),1_000_000_000,1_000_000_000,10,'base_link')
        self.assertFalse(gate.needs_zero(1_050_000_000,10.05))
        self.assertTrue(gate.needs_zero(1_050_000_000,10.101))
        self.assertTrue(gate.needs_zero(1_100_000_001,10.05))
        gate.update_stop(True,10.1);gate.update_stop(False,10.2)
        self.assertTrue(gate.needs_zero(1_050_000_000,10.2))

    def test_clock_rewind_invalidates_previous_command(self):
        gate=guard.Gate();gate.update_stop(False,10)
        gate.command((.01,0,0,0,0,0),1_000_000_000,1_000_000_000,10,'base_link')
        self.assertTrue(gate.needs_zero(999_999_999,10.01))

    def test_namespace_rejects_root_and_remapping(self):
        self.assertEqual(guard.namespace('r100_0160'),'/r100_0160')
        for bad in ['', '/', 'robot;bad','robot//bad','1bad']:
            with self.assertRaises(argparse.ArgumentTypeError): guard.namespace(bad)


if __name__=='__main__': unittest.main()
