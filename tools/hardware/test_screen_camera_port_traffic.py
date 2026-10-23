from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
import subprocess

sys.path.insert(0, str(Path(__file__).parent))
import screen_camera_port_traffic as screen


class TrafficScreenTests(unittest.TestCase):
    def test_byte_deltas_to_megabits_and_zero_interval_rejected(self):
        before = {p: {'rx': 20, 'tx': 100} for p in screen.PORTS}
        after = {p: {'rx': 1250020, 'tx': 2500100} for p in screen.PORTS}
        result = screen.rates(before, after, 5)
        self.assertEqual(result['eno1']['tx_mbit_s'], 4)
        self.assertEqual(result['eno1']['rx_mbit_s'], 2)
        with self.assertRaises(ValueError):
            screen.rates(before, after, 0)

    def test_capture_cleanup_kills_only_owned_stuck_worker(self):
        worker = Mock()
        worker.poll.return_value = None
        worker.wait.side_effect = [subprocess.TimeoutExpired('tcpdump', 3), 0]
        screen.stop_captures([worker])
        worker.send_signal.assert_called_once()
        worker.kill.assert_called_once()
        self.assertEqual(worker.wait.call_count, 2)

    def test_finished_capture_is_not_signalled(self):
        worker = Mock()
        worker.poll.return_value = 0
        screen.stop_captures([worker])
        worker.send_signal.assert_not_called()

    def test_high_baseline_does_not_masquerade_as_reader_increase(self):
        baseline = {name: {'tx_mbit_s': 98} for name in screen.PORTS}
        current = {name: {'tx_mbit_s': 101} for name in screen.PORTS}
        self.assertEqual(screen.excess_wired_tx(current, baseline), [])
        current['eno1']['tx_mbit_s'] = 119
        self.assertEqual(screen.excess_wired_tx(current, baseline), ['eno1'])


if __name__ == '__main__':
    unittest.main()
