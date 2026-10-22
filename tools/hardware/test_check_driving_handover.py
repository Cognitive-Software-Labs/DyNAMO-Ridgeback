"""The D4 integration harness must never retain deployed ROS endpoints."""
from pathlib import Path
import sys
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).parent))
from check_driving_handover import isolated_policy


class HandoverIsolationTests(unittest.TestCase):
    def source(self):
        return {'/r100_0160/twist_mux': {'ros__parameters': {
            'use_stamped': True,
            'topics': {
                'external': {'topic': '/r100_0160/cmd_vel', 'priority': 1, 'timeout': .5},
                'joy': {'topic': '/r100_0160/joy', 'priority': 10, 'timeout': .5}},
            'locks': {'e_stop': {'topic': '/r100_0160/platform/emergency_stop',
                                  'priority': 255, 'timeout': 0.},
                      'safety_stop': {'topic': '/r100_0160/platform/safety_stop',
                                      'priority': 254, 'timeout': 0.}}}}}

    def test_absolute_live_endpoints_are_replaced_and_guard_relay_is_relative(self):
        original = self.source()
        policy = isolated_policy(yaml.safe_dump(original))
        self.assertEqual(policy['locks']['e_stop']['topic'], 'platform/emergency_stop_mux')
        for category in ('topics', 'locks'):
            for key, entry in policy[category].items():
                self.assertFalse(entry['topic'].startswith('/'))
                self.assertNotIn('r100_0160', entry['topic'])
                before = original['/r100_0160/twist_mux']['ros__parameters'][category][key]
                self.assertEqual(entry['priority'], before['priority'])
                self.assertEqual(entry['timeout'], before['timeout'])

    def test_unidentified_stop_lock_is_rejected_before_ros_start(self):
        source = self.source()
        locks = source['/r100_0160/twist_mux']['ros__parameters']['locks']
        locks['other_stop'] = locks.pop('e_stop')
        with self.assertRaisesRegex(ValueError, 'e_stop'):
            isolated_policy(yaml.safe_dump(source))


if __name__ == '__main__':
    unittest.main()
