#!/usr/bin/env python3
"""Bounded, stationary driving graph/parameter evidence; never command the robot.

Run in a sourced ROS 2 environment. The parent watchdog bounds the ROS worker,
including imports and discovery. Only graph queries and ListParameters /
GetParameters clients are used; no data subscriptions or control requests.
Absence and parameter read failures are evidence gaps, never a readiness pass.
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from enum import Enum
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[2]
CYCLONE_FILE = Path('/etc/clearpath/cyclonedds.xml')
TOPIC_GROUPS = ('cmd_vel', 'reference', 'joy', 'emergency_stop',
                'safety_stop', 'bt_quality_stop', 'odom', 'controller_activity')
NODE_ROLES = ('twist_mux', 'platform_velocity_controller', 'controller_manager',
              'joy', 'teleop', 'velocity_smoother', 'collision_monitor')
PARAMETER_PATTERN = re.compile(
    r'cmd|command|reference|timeout|stamped|frame|chain|odom|wheel|controller|'
    r'lock|topic|deadman|enable|button|axis|axes|scale|deadzone|priority|'
    r'velocity|accel|decel|safety|stop|use_sim_time|update_rate|autorepeat|'
    r'sticky|coalesce|linear|angular|speed', re.IGNORECASE)
SECRET_PATTERN = re.compile(r'password|passwd|secret|token|credential|private_key',
                            re.IGNORECASE)
MAX_PARAMETERS = 256


def normalize_namespace(value):
    if not isinstance(value, str):
        raise ValueError('robot namespace must be a non-empty ROS namespace')
    result = value.strip('/')
    if not result or any(not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', part)
                         for part in result.split('/')):
        raise ValueError('robot namespace must be a non-empty ROS namespace')
    return '/' + result


def read_settings(robot_config):
    """Read identity only, never serialize the robot configuration or environment."""
    import yaml

    try:
        document = yaml.safe_load(Path(robot_config).read_text())
        namespace = normalize_namespace(document['system']['ros2']['namespace'])
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise ValueError('cannot read robot namespace from robot configuration') from error
    if not CYCLONE_FILE.is_file():
        raise ValueError('explicit CycloneDDS configuration is unavailable')
    return {'namespace': namespace, 'robot_config': str(Path(robot_config).resolve()),
            'ros_domain_id': 0, 'rmw_implementation': 'rmw_cyclonedds_cpp',
            'cyclonedds_uri': CYCLONE_FILE.as_uri()}


def configure_environment(settings):
    os.environ['ROS_DOMAIN_ID'] = '0'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['CYCLONEDDS_URI'] = settings['cyclonedds_uri']
    # A graph-only node needs no persistent ROS log directory.
    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0'


def plain_value(value):
    """Serialize rclpy enums, named tuples, durations and ordinary QoS values."""
    if isinstance(value, Enum):
        return {'name': value.name, 'value': plain_value(value.value)}
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if hasattr(value, '_asdict'):
        return {str(key): plain_value(item) for key, item in value._asdict().items()}
    if isinstance(value, Mapping):
        return {str(key): plain_value(item) for key, item in value.items()}
    if isinstance(value, (Sequence, bytes, bytearray, memoryview)):
        return [plain_value(item) for item in value]
    if hasattr(value, 'nanoseconds'):
        return {'nanoseconds': int(value.nanoseconds)}
    # Do not stringify arbitrary objects: their repr may contain private state.
    return {'unavailable': type(value).__name__}


def endpoint_record(info):
    fields = ('node_name', 'node_namespace', 'topic_type', 'topic_type_hash',
              'endpoint_type', 'endpoint_gid')
    record = {field: plain_value(getattr(info, field)) for field in fields
              if hasattr(info, field)}
    qos = getattr(info, 'qos_profile', None)
    qos_fields = ('history', 'depth', 'reliability', 'durability', 'lifespan',
                  'deadline', 'liveliness', 'liveliness_lease_duration',
                  'avoid_ros_namespace_conventions')
    record['qos'] = ({field: plain_value(getattr(qos, field)) for field in qos_fields
                      if hasattr(qos, field)} if qos is not None else None)
    return record


def known_qos_incompatibilities(topics):
    """Flag known delivery blockers; absence is not a full compatibility pass."""
    failures=[]
    for topic in topics:
        pubs,subs=topic.get('publishers'),topic.get('subscribers')
        if not isinstance(pubs,list) or not isinstance(subs,list):
            continue
        for pub in pubs:
            for sub in subs:
                for policy,offered,requested in [
                        ('reliability','BEST_EFFORT','RELIABLE'),
                        ('durability','VOLATILE','TRANSIENT_LOCAL')]:
                    p=(pub.get('qos') or {}).get(policy)
                    s=(sub.get('qos') or {}).get(policy)
                    if isinstance(p,dict) and isinstance(s,dict) and p.get('name')==offered and s.get('name')==requested:
                        failures.append({'topic':topic['name'],'policy':policy,
                            'publisher':pub.get('node_namespace','').rstrip('/')+'/'+pub.get('node_name',''),
                            'subscriber':sub.get('node_namespace','').rstrip('/')+'/'+sub.get('node_name',''),
                            'offered':offered,'requested':requested})
    return failures


def topic_groups(topic):
    """Match live names, including remapped cmd_vel and stamped/combined variants."""
    parts = topic.lower().strip('/').split('/')
    groups = []
    for group in TOPIC_GROUPS:
        if group == 'cmd_vel':
            match = any(part.startswith('cmd_vel') for part in parts)
        elif group == 'joy':
            match = any(part == 'joy' or part.startswith('joy_') for part in parts)
        elif group == 'odom':
            match = any(part == 'odom' or part.startswith('odom_')
                        or part == 'odometry' for part in parts)
        elif group == 'controller_activity':
            match = parts[-1] == 'activity' and 'controller_manager' in parts
        else:
            match = any(part == group or part.startswith(group + '_') for part in parts)
        if match:
            groups.append(group)
    return groups


def in_namespace(name, namespace):
    return name == namespace or name.startswith(namespace + '/')


def node_roles(name):
    lowered = name.lower()
    return [role for role in NODE_ROLES if role in lowered]


def select_parameter_names(names):
    return sorted({name for name in names if PARAMETER_PATTERN.search(name)
                   and not SECRET_PATTERN.search(name)})


def parameter_record(value):
    parameter_type = int(value.type)
    fields = {1: ('bool', 'bool_value'), 2: ('integer', 'integer_value'),
              3: ('double', 'double_value'), 4: ('string', 'string_value'),
              5: ('byte_array', 'byte_array_value'), 6: ('bool_array', 'bool_array_value'),
              7: ('integer_array', 'integer_array_value'),
              8: ('double_array', 'double_array_value'),
              9: ('string_array', 'string_array_value')}
    if parameter_type not in fields:
        return {'type': parameter_type, 'status': 'unknown_or_not_set', 'value': None}
    label, field = fields[parameter_type]
    return {'type': label, 'status': 'observed',
            'value': plain_value(getattr(value, field))}


def error_record(error):
    # Exception messages may contain config values; preserve only their class.
    return {'status': 'unavailable', 'error_type': type(error).__name__}


def spin_until(runtime, predicate, deadline):
    while time.monotonic() < deadline and not predicate():
        runtime.rclpy.spin_once(runtime.node, executor=runtime.executor,
                              timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))


def graph_snapshot(node, namespace):
    nodes = []
    for name, ns in sorted(set(node.get_node_names_and_namespaces())):
        full_name = ns.rstrip('/') + '/' + name
        nodes.append({'name': name, 'namespace': ns, 'full_name': full_name,
                      'in_robot_namespace': in_namespace(full_name, namespace),
                      'roles': node_roles(name), 'driving_topics': []})
    topics = []
    for topic, types in sorted(node.get_topic_names_and_types()):
        groups = topic_groups(topic)
        if not groups:
            continue
        record = {'name': topic, 'types': sorted(types), 'groups': groups,
                  'in_robot_namespace': in_namespace(topic, namespace)}
        for key, method in (('publishers', node.get_publishers_info_by_topic),
                            ('subscribers', node.get_subscriptions_info_by_topic)):
            try:
                record[key] = [endpoint_record(info) for info in method(topic)]
                record[key + '_count'] = len(record[key])
            except Exception as error:
                record[key] = error_record(error)
                record[key + '_count'] = None
        topics.append(record)
    # A remapped/custom node name can still participate in the driving chain.
    # Include its read-only parameters without assigning an inferred role.
    by_name = {entry['full_name']: entry for entry in nodes}
    for topic in topics:
        for kind in ('publishers', 'subscribers'):
            endpoints = topic[kind]
            if not isinstance(endpoints, list):
                continue
            for endpoint in endpoints:
                name, ns = endpoint.get('node_name'), endpoint.get('node_namespace')
                if not isinstance(name, str) or not isinstance(ns, str):
                    continue
                full_name = ns.rstrip('/') + '/' + name
                if full_name not in by_name:
                    by_name[full_name] = {
                        'name': name, 'namespace': ns, 'full_name': full_name,
                        'in_robot_namespace': in_namespace(full_name, namespace),
                        'roles': node_roles(name), 'driving_topics': [],
                        'discovered_via': 'topic_endpoint'}
                    nodes.append(by_name[full_name])
                by_name[full_name]['driving_topics'].append(topic['name'])
    for entry in nodes:
        entry['driving_topics'] = sorted(set(entry['driving_topics']))
    nodes.sort(key=lambda entry: entry['full_name'])
    return {'status': 'observed', 'nodes': nodes, 'topics': topics,
            'known_qos_incompatibilities': known_qos_incompatibilities(topics),
            'qos_compatibility': 'not_fully_evaluated',
            'missing_topic_groups': [group for group in TOPIC_GROUPS
                                     if not any(group in topic['groups']
                                                and topic['in_robot_namespace']
                                                for topic in topics)]}


def read_live_parameters(runtime, graph, deadline):
    """Discover names first; only issue the two explicitly read-only service types."""
    candidates = [node for node in graph.get('nodes', [])
                  if (node['roles'] or node.get('driving_topics'))
                  and node['in_robot_namespace']]
    entries = []
    clients = []
    pending = []
    try:
        for node in candidates:
            entry = {'node': node['full_name'], 'roles': node['roles'],
                     'driving_topics': node.get('driving_topics', []), 'status': 'unknown', 'parameters': {}}
            entries.append(entry)
            if time.monotonic() >= deadline:
                entry['status'] = 'deadline_exceeded'
                continue
            try:
                client = runtime.node.create_client(
                    runtime.ListParameters, node['full_name'] + '/list_parameters')
                clients.append(client)
                pending.append({'entry': entry, 'client': client, 'future': None})
            except Exception as error:
                entry.update(error_record(error))

        # Reserve time for GetParameters, so one missing service cannot consume
        # the entire audit. Discovery runs concurrently across live candidates.
        list_deadline = time.monotonic() + max(0.0, deadline - time.monotonic()) / 2

        def lists_done():
            for item in pending:
                if item['future'] is None and item['client'].service_is_ready():
                    request = runtime.ListParameters.Request()
                    request.prefixes = []
                    request.depth = 0  # DEPTH_RECURSIVE
                    try:
                        item['future'] = item['client'].call_async(request)
                    except Exception as error:
                        item['entry'].update(error_record(error))
                        item['future'] = False
            return all(item['future'] is False or
                       (item['future'] is not None and item['future'].done())
                       for item in pending)

        spin_until(runtime, lists_done, list_deadline)
        gets = []
        for item in pending:
            entry, future = item['entry'], item['future']
            if future is False:
                continue
            if future is None or not future.done():
                entry['status'] = ('list_service_unavailable' if future is None
                                   else 'list_parameters_timeout')
                if future is not None:
                    future.cancel()
                continue
            try:
                response = future.result()
                if response is None:
                    raise ValueError('empty parameter response')
                names = select_parameter_names(response.result.names)
                entry['selected_parameter_count'] = len(names)
                entry['parameter_limit'] = MAX_PARAMETERS
                entry['truncated'] = len(names) > MAX_PARAMETERS
                names = names[:MAX_PARAMETERS]
                entry['selected_names'] = names
                if not names:
                    entry['status'] = 'no_matching_live_parameters'
                    continue
                client = runtime.node.create_client(
                    runtime.GetParameters, entry['node'] + '/get_parameters')
                clients.append(client)
                gets.append({'entry': entry, 'client': client, 'names': names, 'future': None})
            except Exception as error:
                entry.update(error_record(error))

        def gets_done():
            for item in gets:
                if item['future'] is None and item['client'].service_is_ready():
                    request = runtime.GetParameters.Request()
                    request.names = item['names']
                    try:
                        item['future'] = item['client'].call_async(request)
                    except Exception as error:
                        item['entry'].update(error_record(error))
                        item['future'] = False
            return all(item['future'] is False or
                       (item['future'] is not None and item['future'].done())
                       for item in gets)

        spin_until(runtime, gets_done, deadline)
        for item in gets:
            entry, future = item['entry'], item['future']
            if future is False:
                continue
            if future is None or not future.done():
                entry['status'] = ('get_service_unavailable' if future is None
                                   else 'get_parameters_timeout')
                if future is not None:
                    future.cancel()
                continue
            try:
                values = future.result().values
                entry['parameters'] = {
                    name: parameter_record(values[index]) if index < len(values)
                    else {'status': 'missing_response_value', 'value': None}
                    for index, name in enumerate(item['names'])}
                entry['status'] = 'observed' if len(values) == len(item['names']) else 'incomplete'
            except Exception as error:
                entry.update(error_record(error))
    finally:
        for client in clients:
            runtime.node.destroy_client(client)
    return entries


def observe(runtime, settings, duration, discovery_seconds):
    start = time.monotonic()
    deadline = start + duration
    spin_until(runtime, lambda: False, min(deadline, start + discovery_seconds))
    report = {'status': 'observed', 'readiness': 'not_established',
              'limitations': ['Graph presence and parameter values do not establish '
                              'fresh command/odometry delivery or stop-state behavior.',
                              'No motion commands, safety commands or sensor subscriptions '
                              'were created; controller state was not changed.',
                              'Controller lifecycle/activity state is unknown: only graph and '
                              'read-only parameter services were inspected.']}
    try:
        report['graph'] = graph_snapshot(runtime.node, settings['namespace'])
    except Exception as error:
        report['graph'] = error_record(error)
    report['live_parameters'] = read_live_parameters(runtime, report['graph'], deadline)
    report['missing_node_roles'] = [role for role in NODE_ROLES
                                   if not any(role in node['roles'] and node['in_robot_namespace']
                                              for node in report['graph'].get('nodes', []))]
    report['elapsed_seconds'] = round(time.monotonic() - start, 3)
    return report


def worker(settings, duration, discovery_seconds, deadline=None):
    deadline = deadline if deadline is not None else time.monotonic() + duration
    configure_environment(settings)
    # ROS is deliberately imported only in the isolated worker.
    import rclpy
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.executors import SingleThreadedExecutor

    node = None
    executor = None
    context = rclpy.context.Context()
    try:
        rclpy.init(args=[], context=context)
        node = rclpy.create_node(
            'stationary_driving_readiness_audit', namespace=settings['namespace'],
            context=context, enable_rosout=False, start_parameter_services=False,
            use_global_arguments=False)
        executor = SingleThreadedExecutor(context=context)
        runtime = SimpleNamespace(node=node, rclpy=rclpy, executor=executor,
                                  GetParameters=GetParameters, ListParameters=ListParameters)
        # The absolute parent deadline includes child startup/imports. Leave a
        # margin for executor/node/context teardown and JSON serialization.
        cleanup_margin = min(1.0, duration * 0.2)
        observation_budget = max(0.0, deadline - time.monotonic() - cleanup_margin)
        return observe(runtime, settings, observation_budget,
                       min(discovery_seconds, observation_budget))
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=min(0.2, duration * 0.1))
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def validate_output(path):
    result = Path(path).resolve()
    allowed = (REPO_ROOT / 'artifacts' / 'hardware', Path('/tmp'))
    if not any(result.is_relative_to(root.resolve()) for root in allowed):
        raise ValueError('output must be under repository artifacts/hardware or /tmp')
    if result.is_dir():
        raise ValueError('output must name a JSON file')
    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-config', type=Path, default=Path('/etc/clearpath/robot.yaml'))
    parser.add_argument('--duration', type=float, default=20.0,
                        help='global observation/watchdog limit in seconds (0.5 to 60)')
    parser.add_argument('--discovery-seconds', type=float, default=3.0)
    parser.add_argument('--output', type=Path, help='JSON file under artifacts/hardware or /tmp')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--deadline-monotonic', type=float, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 0.5 <= args.duration <= 60:
        parser.error('--duration must be between 0.5 and 60 seconds')
    if not math.isfinite(args.discovery_seconds) or not 0 <= args.discovery_seconds < args.duration:
        parser.error('--discovery-seconds must be nonnegative and smaller than --duration')
    return args


def main(argv=None):
    args = parse_args(argv)
    started = time.monotonic()
    report = {'schema_version': 1, 'captured_utc': datetime.now(timezone.utc).isoformat(),
              'mode': 'stationary_read_only', 'readiness': 'not_established',
              'duration_limit_seconds': args.duration}
    output = None
    try:
        output = validate_output(args.output) if args.output else None
        settings = read_settings(args.robot_config)
        report['settings'] = settings
        if args.worker:
            report.update(worker(settings, args.duration, args.discovery_seconds,
                                 args.deadline_monotonic or started + args.duration))
        else:
            remaining = args.duration - (time.monotonic() - started)
            if remaining <= 0:
                raise subprocess.TimeoutExpired('audit worker', args.duration)
            command = [sys.executable, str(Path(__file__).resolve()), '--worker',
                       '--robot-config', str(args.robot_config), '--duration', str(args.duration),
                       '--discovery-seconds', str(args.discovery_seconds),
                       '--deadline-monotonic', str(started + args.duration)]
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=remaining, check=False)
            try:
                child_report = json.loads(result.stdout)
            except (ValueError, TypeError):
                raise RuntimeError('ROS worker did not return JSON evidence') from None
            report.update(child_report)
            report['worker_exit_code'] = result.returncode
    except subprocess.TimeoutExpired:
        report.update(status='deadline_exceeded', error_type='TimeoutExpired')
    except Exception as error:
        report.update(error_record(error))
        print('driving audit unavailable: ' + type(error).__name__, file=sys.stderr)
    report['elapsed_total_seconds'] = round(time.monotonic() - started, 3)
    text = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n'
    if output is not None:
        try:
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(text)
        except OSError as error:
            print('driving audit output unavailable: ' + type(error).__name__, file=sys.stderr)
            print(text, end='')
            return 1
    print(text, end='')
    return 0 if report.get('status') == 'observed' else 1


if __name__ == '__main__':
    sys.exit(main())
