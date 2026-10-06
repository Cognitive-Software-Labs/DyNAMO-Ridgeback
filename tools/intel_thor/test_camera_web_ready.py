"""Offline readiness checks; no live ROS graph, network, or services are touched."""
import importlib.util
import json
from pathlib import Path
import tempfile
from threading import Event
import types
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location(
    'camera_web_ready', Path(__file__).with_name('camera_web_ready.py'))
ready = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ready)


class ContractTest(unittest.TestCase):
    def setUp(self):
        self.topics = ready.required_topics('r100_0160')
        self.publishers = {topic: [ready.IMAGE_TYPE] for topic in self.topics}

    def test_all_three_actual_publishers_and_exact_web_entries_required(self):
        self.assertTrue(ready.evaluate_readiness(
            self.topics, self.publishers, set(self.topics))['ready'])
        for topic in self.topics:
            incomplete = {key: value for key, value in self.publishers.items() if key != topic}
            result = ready.evaluate_readiness(self.topics, incomplete, set(self.topics))
            self.assertFalse(result['ready'])
            self.assertEqual(result['missing_publishers'], [topic])
            wrong_type = {**self.publishers, topic: ['std_msgs/msg/String']}
            self.assertFalse(ready.evaluate_readiness(
                self.topics, wrong_type, set(self.topics))['ready'])
            result = ready.evaluate_readiness(
                self.topics, self.publishers, set(self.topics) - {topic})
            self.assertFalse(result['ready'])
            self.assertEqual(result['missing_web_topics'], [topic])

    def test_advertised_or_subscriber_only_topics_cannot_pass(self):
        # A stale CLI daemon listing or a web topic subscription supplies names,
        # but neither supplies direct Image publisher endpoint evidence.
        result = ready.evaluate_readiness(self.topics, {}, set(self.topics))
        self.assertFalse(result['ready'])
        self.assertEqual(result['missing_publishers'], list(self.topics))

    def test_web_contract_not_substring_or_nested_json(self):
        payload = json.dumps([topic + '/compressed' for topic in self.topics])
        result = ready.evaluate_readiness(
            self.topics, self.publishers, ready.parse_web_topics(payload))
        self.assertFalse(result['ready'])
        for payload in ('{"topics": []}', 'null', '"topic"', '[1]', 'garbage'):
            with self.assertRaises(ValueError):
                ready.parse_web_topics(payload)
        self.assertEqual(ready.parse_web_topics(json.dumps(list(self.topics))), set(self.topics))

    def test_http_failure_cannot_pass_even_with_other_evidence(self):
        result = ready.evaluate_readiness(
            self.topics, self.publishers, set(self.topics), 'connection refused')
        self.assertFalse(result['ready'])
        self.assertEqual(result['web_error'], 'connection refused')

    def test_robot_namespace_is_read_from_safe_yaml_without_default(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot.yaml'
            for namespace in ('r100_0160', '/fleet/unit_2/'):
                path.write_text(f'system:\n  ros2:\n    namespace: {namespace}\n')
                self.assertEqual(ready.read_namespace(path), namespace.strip('/'))
            for contents in ('{}', 'system: null', 'system: {ros2: {namespace: ""}}',
                             'system: {ros2: {namespace: 4}}',
                             'system: {ros2: {namespace: bad-name}}'):
                path.write_text(contents)
                with self.assertRaises(ValueError):
                    ready.read_namespace(path)
            import yaml
            path.write_text('!!python/object/apply:os.system ["false"]')
            with self.assertRaises(yaml.YAMLError):
                ready.read_namespace(path)


class FakeClock:
    def __init__(self):
        self.now = 100.0
        self.waits = []

    def __call__(self):
        return self.now

    def wait(self, seconds):
        self.waits.append(seconds)
        self.now += seconds


def state(publishers=False, web=False):
    return {'ready': publishers and web,
            'missing_publishers': [] if publishers else ['image'],
            'missing_web_topics': [] if web else ['image'], 'web_error': None}


class DeadlineTest(unittest.TestCase):
    def test_immediate_readiness_has_no_stability_delay(self):
        clock, events = FakeClock(), []
        result = ready.wait_for_readiness(
            lambda remaining: state(True, True), clock.wait, 90, .5, events.append, clock)
        self.assertTrue(result['ready'])
        self.assertEqual(clock.waits, [])
        self.assertEqual(events[0]['status'], 'ready')

    def test_conditions_rechecked_together_on_events(self):
        clock, events = FakeClock(), []
        observations = iter((state(True, False), state(False, True), state(True, True)))
        result = ready.wait_for_readiness(
            lambda remaining: next(observations), clock.wait, 90, .5, events.append, clock)
        self.assertTrue(result['ready'])
        self.assertEqual(clock.waits, [.5, .5])
        self.assertEqual([event['status'] for event in events], ['waiting', 'waiting', 'ready'])

    def test_timeout_bounds_last_wait_and_preserves_missing_conditions(self):
        clock, events, budgets = FakeClock(), [], []

        def sample(remaining):
            budgets.append(remaining)
            return state(True, False)

        result = ready.wait_for_readiness(
            sample, clock.wait, 1.2, .5, events.append, clock)
        self.assertFalse(result['ready'])
        self.assertEqual(result['status'], 'timeout')
        self.assertEqual(result['missing_web_topics'], ['image'])
        self.assertAlmostEqual(sum(clock.waits), 1.2)
        self.assertAlmostEqual(clock.waits[-1], .2)
        self.assertTrue(all(0 < budget <= 1.2 + 1e-9 for budget in budgets))
        self.assertEqual([event['status'] for event in events], ['waiting', 'timeout'])

    def test_http_work_uses_deadline_budget_no_success_after_expiry(self):
        clock, events = FakeClock(), []

        def sample(remaining):
            self.assertAlmostEqual(remaining, 1)
            clock.now += remaining
            return state(True, True)

        result = ready.wait_for_readiness(sample, clock.wait, 1, .5, events.append, clock)
        self.assertFalse(result['ready'])
        self.assertEqual(result['status'], 'timeout')
        self.assertEqual(clock.waits, [])

    def test_invalid_budgets_fail_without_observing(self):
        for duration in (0, -1, float('inf'), float('nan')):
            with self.assertRaises(ValueError):
                ready.wait_for_readiness(None, None, duration, .5, None)
            with self.assertRaises(ValueError):
                ready.wait_for_readiness(None, None, 90, duration, None)


class HttpBudgetTest(unittest.TestCase):
    def test_incomplete_response_cannot_hold_the_main_loop(self):
        release, finished = Event(), Event()

        def incomplete_response(url, timeout):
            try:
                release.wait()
                return {'late_topic'}
            finally:
                finished.set()

        try:
            with patch.object(ready, '_fetch_web_topics', side_effect=incomplete_response):
                with self.assertRaises(TimeoutError):
                    ready.fetch_web_topics('http://127.0.0.1:9003/topics', .02)
                self.assertFalse(finished.is_set())
        finally:
            release.set()
            self.assertTrue(finished.wait(1))

    def test_response_and_parse_errors_are_returned_from_worker(self):
        with patch.object(ready, '_fetch_web_topics', return_value={'topic'}):
            self.assertEqual(ready.fetch_web_topics('unused', 1), {'topic'})
        with patch.object(ready, '_fetch_web_topics', side_effect=ValueError('invalid JSON')):
            with self.assertRaisesRegex(ValueError, 'invalid JSON'):
                ready.fetch_web_topics('unused', 1)


class DirectDiscoveryTest(unittest.TestCase):
    def test_runtime_selects_domain_profile_and_queries_only_publisher_endpoints(self):
        calls = []
        context = types.SimpleNamespace(ok=lambda: True, shutdown=lambda: calls.append('shutdown'))
        node = types.SimpleNamespace(
            create_timer=lambda interval, callback: calls.append(('timer', interval)),
            get_publishers_info_by_topic=lambda topic: (
                calls.append(('publisher', topic)) or
                [types.SimpleNamespace(topic_type=ready.IMAGE_TYPE)]),
            destroy_node=lambda: calls.append('destroy'))
        executor = types.SimpleNamespace(
            add_node=lambda actual: self.assertIs(actual, node),
            spin_once=lambda **kwargs: self.fail('ready graph must not wait'),
            shutdown=lambda **kwargs: calls.append('executor_shutdown'))

        def init(**kwargs):
            self.assertEqual(kwargs['domain_id'], 0)
            self.assertEqual(kwargs['args'], [])
            self.assertEqual(ready.os.environ['ROS_DOMAIN_ID'], '0')
            self.assertEqual(ready.os.environ['RMW_IMPLEMENTATION'], 'rmw_cyclonedds_cpp')
            self.assertEqual(ready.os.environ['CYCLONEDDS_URI'], profile.as_uri())

        def make_node(name, **kwargs):
            self.assertFalse(kwargs['enable_rosout'])
            self.assertFalse(kwargs['start_parameter_services'])
            self.assertFalse(kwargs['use_global_arguments'])
            return node

        modules = {
            'rclpy': types.SimpleNamespace(init=init),
            'rclpy.context': types.SimpleNamespace(Context=lambda: context),
            'rclpy.node': types.SimpleNamespace(Node=make_node),
            'rclpy.executors': types.SimpleNamespace(SingleThreadedExecutor=lambda **kw: executor),
        }
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / 'dds.xml'
            profile.write_text('<CycloneDDS/>')
            robot = Path(directory) / 'robot.yaml'
            robot.write_text('system: {ros2: {namespace: deployed_robot}}')
            args = ready.parser().parse_args([
                '--domain', '0', '--profile', str(profile), '--robot-config', str(robot)])
            topics = ready.required_topics('deployed_robot')
            with patch.dict('sys.modules', modules), patch.dict(ready.os.environ), \
                    patch.object(ready, 'fetch_web_topics', return_value=set(topics)), \
                    patch('builtins.print'):
                self.assertEqual(ready.run(args), 0)
        self.assertEqual([call[1] for call in calls
                          if isinstance(call, tuple) and call[0] == 'publisher'], list(topics))
        self.assertEqual(calls[-3:], ['executor_shutdown', 'destroy', 'shutdown'])


if __name__ == '__main__':
    unittest.main()
