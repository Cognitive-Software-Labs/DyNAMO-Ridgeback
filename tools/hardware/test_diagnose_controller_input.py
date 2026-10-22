"""The stationary button check must distinguish L1 release from centring."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).parent))
from diagnose_controller_input import separate_release_evidence


class SeparateReleaseTests(unittest.TestCase):
    def test_release_must_happen_while_forward_remains_held_on_both_devices(self):
        def row(kind, t, l1, forward):
            return {'kind': kind, 'elapsed_s': t,
                    'buttons': [310] if l1 else [],
                    'axes': {'1': 60 if forward else 126}}
        rows = [row(kind, t, l1, forward)
                for kind in ('physical', 'virtual')
                for t, l1, forward in ((1., True, True), (2., False, True),
                                       (3., False, False))]
        self.assertTrue(separate_release_evidence(rows)['physical_and_virtual_release_observed'])
        centred_first = [row(kind, t, l1, forward)
                         for kind in ('physical', 'virtual')
                         for t, l1, forward in ((1., True, True), (2., True, False),
                                                (3., False, False))]
        self.assertFalse(separate_release_evidence(centred_first)['physical_and_virtual_release_observed'])
        rows = [x for x in rows if not (x['kind'] == 'virtual' and x['elapsed_s'] == 2.)]
        self.assertFalse(separate_release_evidence(rows)['physical_and_virtual_release_observed'])


if __name__ == '__main__':
    unittest.main()
