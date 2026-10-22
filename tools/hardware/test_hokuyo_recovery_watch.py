import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from hokuyo_recovery_watch import TOPICS, SIGNATURES, recovery_due, stationary_ok, scans_restored


class RecoveryDecisionTests(unittest.TestCase):
    def setUp(self):
        self.journal = '\n'.join(SIGNATURES)

    def test_requires_prior_scans_and_full_signature(self):
        self.assertIsNone(recovery_due(dict.fromkeys(TOPICS), 100, self.journal))
        self.assertIsNone(recovery_due({TOPICS[0]: 80, TOPICS[1]: 99}, 100,
                                        SIGNATURES[0]))

    def test_one_stale_stream_with_reconnect_errors(self):
        self.assertEqual(recovery_due({TOPICS[0]: 80, TOPICS[1]: 99}, 100,
                                      self.journal), [TOPICS[0]])

    def test_short_gap_never_restarts(self):
        self.assertIsNone(recovery_due({TOPICS[0]: 89, TOPICS[1]: 99}, 100,
                                        self.journal))

    def test_one_unobserved_lidar_blocks_recovery(self):
        self.assertIsNone(recovery_due({TOPICS[0]: 80, TOPICS[1]: None}, 100,
                                        self.journal))

    def test_stationary_gate_rejects_released_stale_and_nonfinite_evidence(self):
        stops = [(98, True), (99, True)]
        odom = [(99 + n / 100, (0, 0, 0)) for n in range(100)]
        self.assertTrue(stationary_ok(stops, odom, 100))
        self.assertFalse(stationary_ok(stops + [(100, False)], odom, 100))
        self.assertFalse(stationary_ok(stops, odom, 104))
        self.assertFalse(stationary_ok(stops, odom + [(100, (float('nan'), 0, 0))], 100))
        self.assertFalse(stationary_ok(stops, odom + [(100, (.01, 0, 0))], 100))

    def test_recovery_requires_sustained_both_scans_and_increasing_stamps(self):
        good = {topic: [(1 + n / 40, n) for n in range(161)] for topic in TOPICS}
        self.assertTrue(scans_restored(good, 0, 5))
        self.assertFalse(scans_restored({topic: rows[:1] for topic, rows in good.items()}, 0, 5))
        self.assertFalse(scans_restored(good, 0, 6))
        bad = {topic: list(rows) for topic, rows in good.items()}
        bad[TOPICS[1]][40] = (bad[TOPICS[1]][40][0], 39)
        self.assertFalse(scans_restored(bad, 0, 5))
        bad[TOPICS[1]] = [r for r in good[TOPICS[1]] if not 2 < r[0] < 2.4]
        self.assertFalse(scans_restored(bad, 0, 5))


if __name__ == '__main__':
    unittest.main()
