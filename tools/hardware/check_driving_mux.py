#!/usr/bin/env python3
"""Exercise the installed mux in a loopback-only sandbox, never the robot mux.

Uses fixed domain 213, a private namespace, and a newly generated configuration
with synthetic inputs. An optional deployed YAML supplies only policy values;
every topic name is replaced before starting the isolated mux.
Requires a sourced ROS environment and the installed twist_mux package.
"""

from pathlib import Path
import argparse
import copy
import hashlib
import json
import math
import os
import signal
import subprocess
import tempfile
import time


def sandbox_policy(source):
    """Preserve policy values while replacing every configured ROS endpoint."""
    import yaml
    document = yaml.safe_load(source)
    found = []
    def walk(value):
        if not isinstance(value, dict):
            return
        params = value.get('ros__parameters')
        if isinstance(params, dict) and 'topics' in params and 'locks' in params:
            found.append(params)
        for key, child in value.items():
            if key != 'ros__parameters':
                walk(child)
    walk(document)
    if len(found) != 1:
        raise ValueError('expected exactly one mux parameter block')
    policy = copy.deepcopy(found[0])
    if policy.get('use_stamped') is not True:
        raise ValueError('only stamped mux policy is supported')
    if not isinstance(policy['topics'], dict) or not 2 <= len(policy['topics']) <= 10:
        raise ValueError('expected two to ten input sources')
    if not isinstance(policy['locks'], dict) or not 1 <= len(policy['locks']) <= 10:
        raise ValueError('expected one to ten locks')
    for category in ('topics', 'locks'):
        for index, (name, entry) in enumerate(policy[category].items()):
            if not isinstance(entry, dict) or not isinstance(entry.get('topic'), str):
                raise ValueError('topic declarations must be dictionaries with a topic name')
            priority, timeout = entry.get('priority'), entry.get('timeout')
            if type(priority) is not int or not 0 <= priority <= 255:
                raise ValueError('priority must be an integer from 0 through 255')
            if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
                    or not math.isfinite(timeout) or not 0 <= timeout <= 2):
                raise ValueError('test supports finite timeouts from zero through two seconds')
            if category == 'topics' and timeout <= .15:
                raise ValueError('input timeout too short for this isolated test')
            entry['topic'] = f'isolated_{category}_{index}'
    priorities = [entry['priority'] for entry in policy['topics'].values()]
    if len(set(priorities)) != len(priorities):
        raise ValueError('equal input priorities need a separate arbitration test')
    if any(entry['priority'] <= max(priorities) for entry in policy['locks'].values()):
        raise ValueError('this test requires each lock to outrank all sources')
    return {'use_stamped': True, 'topics': policy['topics'], 'locks': policy['locks']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        help='read a deployed YAML policy, replace all topic names, test only on domain 213/loopback')
    args = parser.parse_args()
    source = args.config.read_text() if args.config else '''/**:
  ros__parameters:
    use_stamped: true
    topics:
      external: {topic: external, timeout: 0.5, priority: 1}
      joy: {topic: joy, timeout: 0.5, priority: 10}
      rc: {topic: rc, timeout: 0.5, priority: 12}
    locks:
      stop: {topic: stop, timeout: 0.0, priority: 255}
'''
    policy = sandbox_policy(source)
    import yaml
    with tempfile.TemporaryDirectory(prefix='driving-mux-sandbox-') as directory:
        root = Path(directory)
        profile = root / 'cyclone.xml'
        profile.write_text('<CycloneDDS><Domain id="any"><General>'
                           '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
                           '<AllowMulticast>false</AllowMulticast>'
                           '</General><Discovery><Peers><Peer Address="127.0.0.1"/>'
                           '</Peers></Discovery></Domain></CycloneDDS>')
        environment = dict(os.environ, ROS_DOMAIN_ID='213',
                           RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                           CYCLONEDDS_URI=profile.as_uri(), ROS_LOG_DIR=str(root / 'ros'))
        # Set before ROS imports; shell selections cannot point this test at the base.
        os.environ.update({key: environment[key] for key in
                           ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI',
                            'ROS_LOG_DIR')})
        namespace = '/driving_mux_sandbox_' + str(os.getpid())
        config = root / 'mux.yaml'
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': policy}}))
        command = ['ros2', 'run', 'twist_mux', 'twist_mux', '--ros-args',
                   '-r', '__ns:=' + namespace, '--params-file', str(config)]
        child = None
        node = None
        checks = []
        try:
            import rclpy
            from geometry_msgs.msg import TwistStamped
            from std_msgs.msg import Bool

            with (root / 'mux.log').open('w') as log:
                child = subprocess.Popen(command, env=environment, stdout=log,
                                         stderr=log, start_new_session=True)
                rclpy.init(args=[])
                node = rclpy.create_node('mux_test_observer', namespace=namespace,
                                        enable_rosout=False, start_parameter_services=False)
                received = []
                subscription = node.create_subscription(
                    TwistStamped, 'cmd_vel_out', lambda msg: received.append(msg), 10)
                publishers = {key: node.create_publisher(TwistStamped, entry['topic'], 10)
                              for key, entry in policy['topics'].items()}
                locks = {key: node.create_publisher(Bool, entry['topic'], 10)
                         for key, entry in policy['locks'].items()}

                def spin(duration):
                    end = time.monotonic() + duration
                    while time.monotonic() < end:
                        if child.poll() is not None:
                            raise RuntimeError('sandbox mux exited')
                        rclpy.spin_once(node, timeout_sec=min(.02, max(0, end-time.monotonic())))

                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    if (all(pub.get_subscription_count() for pub in publishers.values())
                            and all(lock.get_subscription_count() for lock in locks.values())
                            and node.count_publishers(namespace + '/cmd_vel_out')):
                        break
                    spin(.05)
                else:
                    raise RuntimeError('sandbox discovery timed out')

                def send(key, speed):
                    msg = TwistStamped()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.header.frame_id = 'sandbox_base'
                    msg.twist.linear.x = speed
                    publishers[key].publish(msg)
                    spin(.12)
                    return msg

                def check(name, passed):
                    checks.append({'name': name, 'passed': bool(passed)})

                for lock in locks.values():
                    lock.publish(Bool(data=False))
                spin(.15)
                order = sorted(policy['topics'], key=lambda key: policy['topics'][key]['priority'])
                for index, key in enumerate(order):
                    original = send(key, .011+index*.01)
                    check(key+' takes priority with identical stamp and frame',
                          bool(received) and received[-1] == original)
                    masked = True
                    for lower in order[:index]:
                        send(key, .011+index*.01)
                        before = len(received)
                        send(lower, .001)
                        masked = masked and len(received) == before
                    check(key+' masks lower priority sources', masked)
                highest, lowest = order[-1], order[0]
                for key, lock in locks.items():
                    lock.publish(Bool(data=True))
                    spin(.05)
                    before = len(received)
                    send(highest, .051)
                    check(key+' asserted lock suppresses commands', len(received) == before)
                    lock.publish(Bool(data=False))
                    spin(.05)
                    original = send(highest, .053)
                    check(key+' cleared lock permits fresh command', received[-1] == original)
                before = len(received)
                spin(max(entry['timeout'] for entry in policy['topics'].values())+.15)
                check('input expiry emits no zero command', len(received) == before)
                send(lowest, .033)
                check('lowest priority resumes after higher priority input expiry',
                      received[-1].twist.linear.x == .033)
                print(json.dumps({'mode': 'isolated_installed_mux', 'domain': 213,
                                  'interface': 'lo', 'namespace': namespace,
                                  'checks': checks,
                                  'policy_file': str(args.config) if args.config else None,
                                  'policy_source_sha256': hashlib.sha256(source.encode()).hexdigest(),
                                  'isolated_policy': policy,
                                  'limitations': ['No base controller or hardware tested.',
                                                 'Synthetic inputs, not deployed joystick.']},
                                 indent=2))
                return 0 if all(item['passed'] for item in checks) else 1
        finally:
            if node is not None:
                node.destroy_node()
                rclpy.shutdown()
            if child is not None and child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=3)


if __name__ == '__main__':
    raise SystemExit(main())
