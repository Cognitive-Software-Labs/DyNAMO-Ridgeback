"""Deployed mux values may be tested only after every endpoint is isolated."""
import copy
from pathlib import Path
import sys
import unittest
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from check_driving_mux import sandbox_policy


class SandboxPolicyTests(unittest.TestCase):
    def policy(self):
        return {'robot': {'twist_mux': {'ros__parameters': {
            'use_stamped': True,
            'topics': {'external': {'topic': '/robot/cmd_vel', 'timeout': .5, 'priority': 1},
                       'joy': {'topic': '/robot/joy', 'timeout': .5, 'priority': 10}},
            'locks': {'stop': {'topic': '/robot/stop', 'timeout': 0., 'priority': 255}},
        }}}}

    def test_all_absolute_endpoints_are_replaced_but_priorities_and_timeouts_remain(self):
        original = self.policy()
        policy = sandbox_policy(yaml.safe_dump(original))
        for group in ('topics', 'locks'):
            for name, entry in policy[group].items():
                before = original['robot']['twist_mux']['ros__parameters'][group][name]
                self.assertEqual(entry['priority'], before['priority'])
                self.assertEqual(entry['timeout'], before['timeout'])
                self.assertTrue(entry['topic'].startswith('isolated_'))
                self.assertNotIn('/', entry['topic'])

    def test_ambiguous_or_unsupported_policy_fails_before_starting_ros(self):
        original = self.policy()
        ambiguous = copy.deepcopy(original)
        ambiguous['other'] = copy.deepcopy(ambiguous['robot'])
        with self.assertRaises(ValueError):
            sandbox_policy(yaml.safe_dump(ambiguous))
        for field, value in [('priority', True), ('priority', 256), ('timeout', float('nan')),
                             ('timeout', -.1), ('timeout', 10.)]:
            changed = copy.deepcopy(original)
            changed['robot']['twist_mux']['ros__parameters']['topics']['joy'][field] = value
            with self.assertRaises(ValueError):
                sandbox_policy(yaml.safe_dump(changed))
        original['robot']['twist_mux']['ros__parameters']['topics']['joy']['priority'] = 1
        with self.assertRaises(ValueError):
            sandbox_policy(yaml.safe_dump(original))


if __name__ == '__main__':
    unittest.main()
