"""A pressed-stop bridge fault must never qualify a bounded motion trial."""
from types import SimpleNamespace
import unittest

from tools.hardware.motor_status_gate import ready, stable_after_release, summarize


WHEELS = ('front_left_wheel_joint', 'front_right_wheel_joint',
          'rear_left_wheel_joint', 'rear_right_wheel_joint')


def status(fault=0, mode=2, voltage=23.0, names=WHEELS):
    return SimpleNamespace(drivers=[SimpleNamespace(device_name=name, fault=fault,
                          mode=mode, bus_voltage=voltage) for name in names])


class MotorStatusGateTests(unittest.TestCase):
    def test_all_four_clear_speed_mode_required(self):
        self.assertTrue(ready(summarize(status())))
        for candidate in [status(fault=8), status(mode=0), status(voltage=0),
                          status(names=WHEELS[:3]), status(names=WHEELS[:3]+WHEELS[:1])]:
            self.assertFalse(ready(summarize(candidate)))

    def test_clear_status_must_follow_release_and_persist(self):
        self.assertFalse(stable_after_release(None, 0, 3, 3, 1))
        self.assertFalse(stable_after_release(.5, 3, 3, 3, 1))
        self.assertFalse(stable_after_release(1.1, 1, 2.2, 2.2, 1))
        self.assertFalse(stable_after_release(1.1, 3, 2.2, 1.1, 1))
        self.assertTrue(stable_after_release(1.1, 3, 2.2, 2.1, 1))


if __name__ == '__main__':
    unittest.main()
