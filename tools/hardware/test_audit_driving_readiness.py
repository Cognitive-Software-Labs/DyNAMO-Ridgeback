"""Offline tests for stationary graph evidence; never import or invoke ROS."""

import ast
from array import array
from collections import namedtuple
import contextlib
from enum import IntEnum
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    'audit_driving_readiness', Path(__file__).with_name('audit_driving_readiness.py'))
audit = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = audit
SPEC.loader.exec_module(audit)


class ListParameters:
    class Request:
        pass


class GetParameters:
    class Request:
        pass


class Clock:
    def __init__(self):
        self.now = 100.0
        self.executor = object()

    def monotonic(self):
        return self.now

    def spin_once(self, node, timeout_sec, executor):
        if executor is not self.executor:
            raise AssertionError('spin must use the context-bound executor')
        if not 0 <= timeout_sec <= 0.1:
            raise AssertionError('unbounded wait')
        self.now += timeout_sec


class Future:
    def __init__(self, response, done=True):
        self.response = response
        self.ready = done
        self.cancelled = False

    def done(self):
        return self.ready

    def result(self):
        return self.response

    def cancel(self):
        self.cancelled = True


class Client:
    def __init__(self, owner, service, name):
        self.owner, self.service, self.name = owner, service, name

    def service_is_ready(self):
        return self.name not in self.owner.unavailable

    def call_async(self, request):
        self.owner.calls.append((self.service, self.name, request))
        if self.service is ListParameters:
            response = SimpleNamespace(result=SimpleNamespace(names=self.owner.names))
        elif self.service is GetParameters:
            response = SimpleNamespace(values=self.owner.values)
        else:
            raise AssertionError('a non-read-only service was called')
        future = Future(response, self.name not in self.owner.stalled)
        self.owner.futures.append(future)
        return future


class Node:
    """Only graph and read-only client methods exist on this fake."""
    def __init__(self):
        self.calls, self.clients, self.destroyed, self.futures = [], [], [], []
        self.unavailable, self.stalled = set(), set()
        self.names = ['topics.joystick.topic', 'cmd_vel_timeout', 'use_stamped_vel',
                      'base_frame_id', 'locks.estop.timeout', 'wheel_names',
                      'deadman_button', 'cmd_password', 'irrelevant']
        self.values = [SimpleNamespace(type=4, string_value='live-' + name)
                       for name in audit.select_parameter_names(self.names)]

    def create_client(self, service, name):
        if service not in (ListParameters, GetParameters):
            raise AssertionError('non-read-only client')
        client = Client(self, service, name)
        self.clients.append(client)
        return client

    def destroy_client(self, client):
        self.destroyed.append(client)

    def get_node_names_and_namespaces(self):
        return [('twist_mux', '/r100_0160'), ('joy_x_combiner', '/r100_0160'),
                ('twist_mux', '/other_robot'), ('velocity_smoother', '/r100_0160/nav2')]

    def get_topic_names_and_types(self):
        return [('/r100_0160/platform/cmd_vel', ['geometry_msgs/msg/TwistStamped']),
                ('/r100_0160/lidar2d_0/scan', ['sensor_msgs/msg/LaserScan']),
                ('/r100_0160/camera_0/image', ['sensor_msgs/msg/Image']),
                ('/other_robot/safety_stop', ['std_msgs/msg/Bool'])]

    def get_publishers_info_by_topic(self, topic):
        return [SimpleNamespace(node_name='twist_mux', node_namespace='/r100_0160',
                                topic_type='geometry_msgs/msg/TwistStamped',
                                qos_profile=SimpleNamespace(depth=10, reliability=1))]

    def get_subscriptions_info_by_topic(self, topic):
        return []


def runtime(node, clock):
    return SimpleNamespace(node=node, rclpy=clock, executor=clock.executor,
                           ListParameters=ListParameters, GetParameters=GetParameters)


class DrivingAuditTests(unittest.TestCase):
    def test_known_qos_delivery_blockers_are_explicit(self):
        def endpoint(name,reliability,durability='VOLATILE'):
            return {'node_namespace':'/robot','node_name':name,
                    'qos':{'reliability':{'name':reliability},
                           'durability':{'name':durability}}}
        topic={'name':'/robot/platform/emergency_stop',
               'publishers':[endpoint('firmware','BEST_EFFORT')],
               'subscribers':[endpoint('mux','RELIABLE'),endpoint('guard','BEST_EFFORT')]}
        failures=audit.known_qos_incompatibilities([topic])
        self.assertEqual(len(failures),1)
        self.assertEqual(failures[0]['subscriber'],'/robot/mux')
        self.assertEqual(failures[0]['policy'],'reliability')
        topic['publishers']=[endpoint('relay','RELIABLE')]
        self.assertEqual(audit.known_qos_incompatibilities([topic]),[])
        topic['subscribers']=[endpoint('late_reader','RELIABLE','TRANSIENT_LOCAL')]
        self.assertEqual(audit.known_qos_incompatibilities([topic])[0]['policy'],'durability')
        topic['publishers']=[{'qos':None}]
        self.assertEqual(audit.known_qos_incompatibilities([topic]),[])

    def test_namespace_validation_and_boundary(self):
        self.assertEqual(audit.normalize_namespace('/fleet/r100_0160/'), '/fleet/r100_0160')
        for value in ('', '/', '1bad', 'robot//child', 'robot name', None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                audit.normalize_namespace(value)
        self.assertTrue(audit.in_namespace('/robot/nav2/cmd_vel', '/robot'))
        self.assertFalse(audit.in_namespace('/robot2/cmd_vel', '/robot'))

    def test_settings_read_deployment_identity_and_force_domain_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / 'robot.yaml'
            config.write_text('system:\n  ros2:\n    namespace: r100_0160\n    domain_id: 42\n')
            cyclone = root / 'cyclonedds.xml'
            cyclone.write_text('<CycloneDDS/>')
            with patch.object(audit, 'CYCLONE_FILE', cyclone):
                settings = audit.read_settings(config)
            self.assertEqual(settings['namespace'], '/r100_0160')
            self.assertEqual(settings['ros_domain_id'], 0)
            with patch.dict(audit.os.environ, {'ROS_DOMAIN_ID': '42',
                                             'RMW_IMPLEMENTATION': 'other'}):
                audit.configure_environment(settings)
                self.assertEqual(audit.os.environ['ROS_DOMAIN_ID'], '0')
                self.assertEqual(audit.os.environ['RMW_IMPLEMENTATION'], 'rmw_cyclonedds_cpp')
                self.assertEqual(audit.os.environ['CYCLONEDDS_URI'], cyclone.as_uri())
            self.assertNotIn('domain_id: 42', json.dumps(settings))
            with patch.object(audit, 'CYCLONE_FILE', root / 'missing'), self.assertRaises(ValueError):
                audit.read_settings(config)

    def test_missing_namespace_does_not_fall_back_to_repository(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'robot.yaml'
            config.write_text('system: {}\npassword: hidden\n')
            with self.assertRaisesRegex(ValueError, 'cannot read robot namespace') as error:
                audit.read_settings(config)
            self.assertNotIn('hidden', str(error.exception))

    def test_topic_matching_handles_live_variants_without_sensor_topics(self):
        expected = {'/robot/cmd_vel_smoothed': ['cmd_vel'],
                    '/robot/platform_velocity_controller/reference': ['reference'],
                    '/robot/joy_teleop/cmd_vel_combined': ['cmd_vel', 'joy'],
                    '/robot/platform/emergency_stop': ['emergency_stop'],
                    '/robot/safety_stop': ['safety_stop'],
                    '/robot/bt_quality_stop': ['bt_quality_stop'],
                    '/robot/odometry/filtered': ['odom'],
                    '/robot/camera_0/image': [], '/robot/lidar2d_0/scan': [],
                    '/robot/controller_manager/activity': ['controller_activity'],
                    '/robot/diagnostics': [], '/robot/not_cmd_vel': []}
        for name, groups in expected.items():
            with self.subTest(name=name):
                self.assertEqual(audit.topic_groups(name), groups)

    def test_endpoint_qos_namedtuple_enum_duration_and_plain_values(self):
        class Reliability(IntEnum):
            RELIABLE = 1
        Named = namedtuple('Named', 'version value')
        info = SimpleNamespace(node_name='controller', node_namespace='/robot',
                               topic_type='geometry_msgs/msg/TwistStamped',
                               endpoint_type=Reliability.RELIABLE, endpoint_gid=b'\x01\x02',
                               topic_type_hash=Named(1, [2, 3]),
                               qos_profile=SimpleNamespace(reliability=Reliability.RELIABLE,
                                                           depth=1, history=1,
                                                           deadline=SimpleNamespace(nanoseconds=500)))
        record = audit.endpoint_record(info)
        self.assertEqual(record['endpoint_gid'], [1, 2])
        self.assertEqual(record['topic_type_hash'], {'version': 1, 'value': [2, 3]})
        self.assertEqual(record['qos']['reliability'], {'name': 'RELIABLE', 'value': 1})
        self.assertEqual(record['qos']['deadline'], {'nanoseconds': 500})
        json.dumps(record, allow_nan=False)
        self.assertEqual(audit.plain_value(array('q', [1, 2])), [1, 2])
        self.assertEqual(audit.parameter_record(SimpleNamespace(
            type=7, integer_array_value=array('q', [1, 2])))['value'], [1, 2])
        self.assertEqual(audit.plain_value(float('inf')), 'inf')
        self.assertEqual(audit.plain_value(object()), {'unavailable': 'object'})

    def test_only_live_discovered_selected_parameters_are_read(self):
        node, clock = Node(), Clock()
        graph = audit.graph_snapshot(node, '/r100_0160')
        with patch.object(audit.time, 'monotonic', clock.monotonic):
            entries = audit.read_live_parameters(runtime(node, clock), graph, clock.now + 5)
        self.assertEqual(len(entries), 3)
        self.assertEqual(len(node.calls), 6)
        self.assertEqual(node.destroyed, node.clients)
        self.assertTrue(all(entry['status'] == 'observed' for entry in entries))
        for service, name, request in node.calls:
            self.assertTrue(name.startswith('/r100_0160/'))
            if service is ListParameters:
                self.assertEqual(request.prefixes, [])
                self.assertEqual(request.depth, 0)
            else:
                self.assertNotIn('cmd_password', request.names)
                self.assertNotIn('irrelevant', request.names)
                self.assertIn('use_stamped_vel', request.names)
        self.assertEqual(len(graph['topics']), 2)
        self.assertEqual(graph['topics'][0]['publishers_count'], 1)
        self.assertEqual(graph['topics'][0]['subscribers_count'], 0)
        self.assertIn('safety_stop', graph['missing_topic_groups'])
        self.assertTrue(all('scan' not in topic['name'] for topic in graph['topics']))

    def test_custom_live_node_name_is_found_from_driving_endpoints(self):
        node, clock = Node(), Clock()
        with patch.object(node, 'get_publishers_info_by_topic', return_value=[
                SimpleNamespace(node_name='input_mux', node_namespace='/r100_0160',
                                topic_type='geometry_msgs/msg/TwistStamped')]):
            graph = audit.graph_snapshot(node, '/r100_0160')
        custom = next(entry for entry in graph['nodes'] if entry['name'] == 'input_mux')
        self.assertEqual(custom['roles'], [])
        self.assertIn('/r100_0160/platform/cmd_vel', custom['driving_topics'])
        with patch.object(audit.time, 'monotonic', clock.monotonic):
            entries = audit.read_live_parameters(runtime(node, clock), graph, clock.now + 5)
        self.assertIn('/r100_0160/input_mux', [entry['node'] for entry in entries])
        self.assertTrue(all(entry['status'] == 'observed' for entry in entries))

    def test_missing_and_stalled_services_share_a_global_deadline(self):
        node, clock = Node(), Clock()
        graph = audit.graph_snapshot(node, '/r100_0160')
        node.unavailable.add('/r100_0160/twist_mux/list_parameters')
        node.stalled.add('/r100_0160/joy_x_combiner/list_parameters')
        node.unavailable.add('/r100_0160/nav2/velocity_smoother/get_parameters')
        start = clock.now
        with patch.object(audit.time, 'monotonic', clock.monotonic):
            entries = audit.read_live_parameters(runtime(node, clock), graph, start + 5)
        self.assertLessEqual(clock.now, start + 5.00001)
        self.assertEqual({entry['status'] for entry in entries},
                         {'list_service_unavailable', 'list_parameters_timeout',
                          'get_service_unavailable'})
        self.assertTrue(any(future.cancelled for future in node.futures))
        self.assertEqual(node.destroyed, node.clients)

    def test_not_set_and_missing_values_never_report_observed(self):
        self.assertEqual(audit.parameter_record(SimpleNamespace(type=0))['status'],
                         'unknown_or_not_set')
        node, clock = Node(), Clock()
        node.values = []
        with patch.object(audit.time, 'monotonic', clock.monotonic):
            entries = audit.read_live_parameters(runtime(node, clock),
                                                audit.graph_snapshot(node, '/r100_0160'),
                                                clock.now + 1)
        self.assertTrue(all(entry['status'] == 'incomplete' for entry in entries))
        self.assertTrue(all(value['status'] == 'missing_response_value'
                            for entry in entries for value in entry['parameters'].values()))

    def test_parameter_limit_is_explicit(self):
        node, clock = Node(), Clock()
        node.names = [f'controller_{index}.type' for index in range(300)]
        node.values = [SimpleNamespace(type=2, integer_value=1)] * audit.MAX_PARAMETERS
        with patch.object(audit.time, 'monotonic', clock.monotonic):
            entries = audit.read_live_parameters(runtime(node, clock),
                                                audit.graph_snapshot(node, '/r100_0160'),
                                                clock.now + 1)
        self.assertTrue(all(entry['truncated'] for entry in entries))
        self.assertTrue(all(entry['selected_parameter_count'] == 300 for entry in entries))
        self.assertTrue(all(len(entry['parameters']) == 256 for entry in entries))

    def test_observation_has_no_readiness_pass(self):
        node, clock = Node(), Clock()
        with patch.object(audit.time, 'monotonic', clock.monotonic):
            report = audit.observe(runtime(node, clock), {'namespace': '/r100_0160'}, 5, 1)
        self.assertEqual(report['readiness'], 'not_established')
        self.assertEqual(report['status'], 'observed')
        self.assertIn('controller_manager', report['missing_node_roles'])
        self.assertEqual(report['elapsed_seconds'], 1.0)

    def test_worker_binds_custom_context_executor_and_reserves_parent_deadline(self):
        node, clock = Node(), Clock()
        contexts, executors, cleaned = [], [], []

        class Context:
            def __init__(self):
                contexts.append(self)

            def ok(self):
                return True

        class Executor:
            def __init__(self, context):
                self.context = context
                executors.append(self)
                clock.executor = self

            def shutdown(self, timeout_sec):
                self.timeout = timeout_sec
                cleaned.append('executor')

        def initialize(args, context):
            self.assertEqual(args, [])
            self.assertIs(context, contexts[0])
            clock.now += 2  # Import/runtime startup must consume the same budget.

        def create_node(*args, **kwargs):
            self.assertIs(kwargs['context'], contexts[0])
            self.assertFalse(kwargs['enable_rosout'])
            self.assertFalse(kwargs['start_parameter_services'])
            return node

        node.destroy_node = lambda: cleaned.append('node')
        fake_rclpy = SimpleNamespace(context=SimpleNamespace(Context=Context), init=initialize,
                                     create_node=create_node, spin_once=clock.spin_once,
                                     shutdown=lambda context: cleaned.append('context'))
        modules = {'rclpy': fake_rclpy,
                   'rclpy.executors': SimpleNamespace(SingleThreadedExecutor=Executor),
                   'rcl_interfaces': SimpleNamespace(),
                   'rcl_interfaces.srv': SimpleNamespace(GetParameters=GetParameters,
                                                        ListParameters=ListParameters)}
        node.unavailable.add('/r100_0160/twist_mux/list_parameters')
        node.unavailable.add('/r100_0160/nav2/velocity_smoother/get_parameters')
        with patch.dict(sys.modules, modules), \
                patch.object(audit.time, 'monotonic', clock.monotonic), \
                patch.object(audit, 'configure_environment'):
            report = audit.worker({'namespace': '/r100_0160'}, 20, 3, deadline=120)
        self.assertIs(executors[0].context, contexts[0])
        self.assertLessEqual(clock.now, 119.00001)
        self.assertEqual(report['elapsed_seconds'], 17.0)
        self.assertEqual(cleaned, ['executor', 'node', 'context'])
        self.assertLessEqual(executors[0].timeout, 0.2)

    def test_output_restricted_and_symlink_escape_rejected(self):
        self.assertEqual(audit.validate_output('/tmp/evidence.json'), Path('/tmp/evidence.json'))
        for path in ('/etc/evidence.json', str(audit.REPO_ROOT / 'evidence.json')):
            with self.assertRaises(ValueError):
                audit.validate_output(path)
        with tempfile.TemporaryDirectory() as directory:
            escape = Path(directory) / 'outside'
            escape.symlink_to('/etc')
            with self.assertRaises(ValueError):
                audit.validate_output(escape / 'evidence.json')

    def test_parent_watchdog_returns_bounded_error_json_without_worker_output(self):
        settings = {'namespace': '/robot'}
        out, err = io.StringIO(), io.StringIO()
        with patch.object(audit, 'read_settings', return_value=settings), \
                patch.object(audit.subprocess, 'run', side_effect=subprocess.TimeoutExpired('worker', 1)) as run, \
                contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            result = audit.main(['--duration', '1', '--discovery-seconds', '0'])
        self.assertEqual(result, 1)
        report = json.loads(out.getvalue())
        self.assertEqual(report['status'], 'deadline_exceeded')
        self.assertEqual(report['readiness'], 'not_established')
        self.assertLessEqual(run.call_args.kwargs['timeout'], 1)
        command = run.call_args.args[0]
        self.assertEqual(command[:2], [sys.executable, str(Path(audit.__file__).resolve())])
        self.assertIn('--worker', command)
        self.assertEqual(err.getvalue(), '')

    def test_error_diagnostics_never_expose_exception_values(self):
        out, err = io.StringIO(), io.StringIO()
        with patch.object(audit, 'read_settings', side_effect=ValueError('PRIVATE_VALUE')), \
                contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            self.assertEqual(audit.main([]), 1)
        self.assertNotIn('PRIVATE_VALUE', out.getvalue() + err.getvalue())
        self.assertEqual(json.loads(out.getvalue())['error_type'], 'ValueError')

    def test_source_prohibits_control_operations_and_non_read_only_ros_services(self):
        source = Path(audit.__file__).read_text()
        tree = ast.parse(source)
        forbidden = {'create_publisher', 'create_subscription', 'publish',
                     'create_action_client', 'send_goal_async', 'set_parameters',
                     'set_parameters_atomically', 'call', 'system', 'popen'}
        for item in ast.walk(tree):
            if isinstance(item, ast.Call) and isinstance(item.func, ast.Attribute):
                self.assertNotIn(item.func.attr, forbidden)
            if isinstance(item, ast.ImportFrom) and item.module == 'rcl_interfaces.srv':
                self.assertEqual({name.name for name in item.names},
                                 {'GetParameters', 'ListParameters'})
        self.assertNotIn('systemctl', source)
        self.assertNotIn('sudo', source)
        self.assertNotIn('geometry_msgs', source)


if __name__ == '__main__':
    unittest.main()
