"""Pure sensor observer checks; no ROS graph or robot processes are touched."""
import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('local_sensor_probe', Path(__file__).with_name('local_sensor_probe.py'))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def image_cdr(endian='<', empty=False):
    body = bytearray()

    def put(fmt, value, alignment=4):
        body.extend(b'\0' * ((-len(body)) % alignment))
        body.extend(struct.pack(endian + fmt, value))

    def string(value):
        encoded = value.encode() + b'\0'
        put('I', len(encoded))
        body.extend(encoded)

    put('i', 123)
    put('I', 456)
    string('camera_color_frame')
    put('I', 2)
    put('I', 3)
    string('rgb8')
    put('B', 0, 1)
    put('I', 9)
    put('I', 0 if empty else 18)
    body.extend(b'\1' * (0 if empty else 18))
    return (b'\0\1\0\0' if endian == '<' else b'\0\0\0\0') + body


class CdrTest(unittest.TestCase):
    def test_both_endians_and_alignment(self):
        for endian in ('<', '>'):
            metadata = probe.image_metadata(image_cdr(endian))
            self.assertEqual(metadata['stamp_ns'], 123000000456)
            self.assertEqual(metadata['data_bytes'], 18)
            self.assertEqual(metadata['height'], 2)
            self.assertEqual(metadata['width'], 3)
            self.assertEqual(metadata['encoding'], 'rgb8')
            self.assertFalse(metadata['empty'])

    def test_empty_is_distinct_from_valid(self):
        self.assertTrue(probe.image_metadata(image_cdr(empty=True))['empty'])

    def test_corrupt_and_unknown_representations_fail(self):
        for data in (b'', b'\0\7\0\0' + image_cdr()[4:], image_cdr()[:-1],
                     image_cdr()[:12]):
            with self.assertRaises(ValueError):
                probe.image_metadata(data)

    def test_invalid_nanosecond_rejected(self):
        data = bytearray(image_cdr())
        struct.pack_into('<I', data, 8, 1000000000)
        with self.assertRaises(ValueError):
            probe.image_metadata(data)


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'source ROS to test actual Jazzy serialization/QoS')
class RosOfflineTest(unittest.TestCase):
    def test_real_ros_serialization_and_qos_without_a_node(self):
        from rclpy.serialization import serialize_message
        from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image
        image = Image(height=480, width=640, encoding='16UC1', step=1280)
        image.header.frame_id = 'camera_depth_optical_frame'
        image.header.stamp.sec = 123
        image.header.stamp.nanosec = 456
        image.data = bytes(480 * 1280)
        metadata = probe.image_metadata(serialize_message(image))
        self.assertEqual(metadata['stamp_ns'], 123000000456)
        self.assertEqual(metadata['data_bytes'], 480 * 1280)
        self.assertFalse(metadata['empty'])
        qos = probe.sensor_qos()
        self.assertEqual(qos.reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(qos.durability, DurabilityPolicy.VOLATILE)
        self.assertEqual(qos.history, HistoryPolicy.KEEP_LAST)
        self.assertEqual(qos.depth, 5)


class MetricTest(unittest.TestCase):
    def test_empty_window_and_pair_never_pass(self):
        summary = probe.StreamMetrics(10).summary(11, 29, .15)
        self.assertFalse(summary['passed'])
        self.assertEqual(summary['max_receive_gap_s'], 1)
        self.assertFalse(probe.exact_matches(set(), set())['passed'])

    def test_warmup_excluded_and_boundaries_included(self):
        metric = probe.StreamMetrics(10)
        metric.record(1, 9.9)
        for i in range(30):
            metric.record(10000000000 + i * 33333333, 10 + i / 30)
        summary = metric.summary(11, 29, .15)
        self.assertTrue(summary['passed'])
        self.assertEqual(summary['unique'], 30)
        self.assertAlmostEqual(summary['max_receive_gap_s'], 1 / 30)
        self.assertFalse(metric.summary(11.2, 29, .15)['passed'])

    def test_initial_gap_fails_despite_good_total_rate(self):
        metric = probe.StreamMetrics(0)
        for i in range(40):
            metric.record(i * 1000000, .2 + i / 100)
        summary = metric.summary(1, 29, .15)
        self.assertFalse(summary['passed'])
        self.assertGreaterEqual(summary['max_receive_gap_s'], .2)

    def test_duplicate_regression_empty_and_stamp_gap(self):
        metric = probe.StreamMetrics(0)
        for stamp, receive in ((0, 0), (0, .03), (1000000000, .06), (1, .09)):
            metric.record(stamp, receive)
        metric.record(2, .12, empty=True)
        summary = metric.summary(.15, 0, .15)
        self.assertEqual(summary['duplicates'], 1)
        self.assertEqual(summary['regressions'], 1)
        self.assertEqual(summary['empty'], 1)
        self.assertEqual(summary['max_stamp_gap_s'], 1)
        self.assertFalse(summary['passed'])

    def test_match_ratio_both_directions(self):
        matches = probe.exact_matches({1, 2, 3}, {1, 2})
        self.assertEqual(matches['aligned_ratio'], 1)
        self.assertAlmostEqual(matches['color_ratio'], 2 / 3)
        self.assertFalse(matches['passed'])


class GuardTest(unittest.TestCase):
    def test_two_consecutive_samples_per_port_and_reset(self):
        guard = probe.TrafficGuard({'eno1': 2, 'enp2s0': 3})
        self.assertFalse(guard.sample({'eno1': {'tx_mbps': 23}}))
        self.assertFalse(guard.sample({'eno1': {'tx_mbps': 22}}))
        self.assertFalse(guard.sample({'eno1': {'tx_mbps': 23}}))
        self.assertEqual(len(guard.sample({'eno1': {'tx_mbps': 23}})), 1)
        self.assertFalse(guard.sample({'enp2s0': {'tx_mbps': 24}}))

    def test_stop_marker_preserves_first_reason(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'stop'
            self.assertTrue(probe.write_stop(path, {'reason': 'first'}))
            self.assertFalse(probe.write_stop(path, {'reason': 'second'}))
            self.assertIn('first', path.read_text())
            self.assertNotIn('second', path.read_text())

    def test_rate_elapsed_and_counter_reset(self):
        previous = {'eno1': {'rx': 1, 'tx': 2}}
        current = {'eno1': {'rx': 1000001, 'tx': 2000002}}
        self.assertEqual(probe.counter_rates(previous, current, 2)['eno1']['tx_mbps'], 8)
        with self.assertRaises(ValueError):
            probe.counter_rates(current, previous, 1)


if __name__ == '__main__':
    unittest.main()
