#!/usr/bin/env python3
"""D4 synthetic priority handover through an installed mux and manual-speed guard.

Always uses domain 214, loopback DDS and a private namespace. No controller,
hardware plugin, deployed process or service is addressed. A deployed-policy
snapshot is required; all of its topic names are replaced before ROS starts.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from check_driving_mux import sandbox_policy


def isolated_policy(source):
    """Keep deployed policy values and connect only its e-stop to the fake guard."""
    policy = sandbox_policy(source)
    if 'e_stop' not in policy['locks']:
        raise ValueError('the deployed policy must name its firmware lock e_stop')
    policy['locks']['e_stop']['topic'] = 'platform/emergency_stop_mux'
    return policy


def vector(message):
    return [message.twist.linear.x, message.twist.linear.y, message.twist.angular.z]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    source = args.config.read_text()
    policy = isolated_policy(source)
    root = args.output_dir.resolve()
    root.mkdir(parents=True, exist_ok=False)
    import yaml
    (root / 'deployed-policy.yaml').write_text(source)
    (root / 'isolated-policy.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': policy}}))
    profile = root / 'cyclone.xml'
    profile.write_text('<CycloneDDS><Domain id="any"><General><Interfaces>'
                       '<NetworkInterface name="lo"/></Interfaces><AllowMulticast>false'
                       '</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/>'
                       '</Peers></Discovery></Domain></CycloneDDS>')
    os.environ.update(ROS_DOMAIN_ID='214', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI=profile.as_uri(), ROS_LOG_DIR=str(root / 'ros'),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    # No ROS imports occur before the mandatory isolation above.
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Bool
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    ns = '/driving_handover_sandbox_' + str(os.getpid())
    guard = Path(__file__).with_name('commissioning_speed_guard.py')
    children, handles, checks, trace, transitions, relay = [], [], [], [], [], []
    node = None
    phase = 'discovery'
    started = time.monotonic()
    report = {'mode': 'isolated_mux_guard_handover', 'domain': 214, 'interface': 'lo',
              'namespace': ns, 'original_manual_speeds': True, 'checks': checks,
              'transitions': transitions, 'isolated_policy': policy,
              'limitations': ['Synthetic sources; no physical joystick, controller or wheels.',
                              'No live takeover, displacement or braking qualification.',
                              'Policy is the supplied deployed snapshot, not a fresh live audit.']}
    for path, name in [(Path(__file__), 'tested-check_driving_handover.py'),
                       (guard, 'tested-commissioning_speed_guard.py'),
                       (Path(__file__).with_name('check_driving_mux.py'), 'tested-check_driving_mux.py')]:
        data = path.read_bytes()
        (root / name).write_bytes(data)
        report.setdefault('source_sha256', {})[name] = hashlib.sha256(data).hexdigest()
    report['policy_sha256'] = hashlib.sha256(source.encode()).hexdigest()
    try:
        from ament_index_python.packages import get_package_prefix
        mux_prefix = Path(get_package_prefix('twist_mux'))
        mux_binary = mux_prefix/'lib/twist_mux/twist_mux'
        report['mux_executable'] = str(mux_binary)
        report['mux_executable_sha256'] = hashlib.sha256(mux_binary.read_bytes()).hexdigest()
        (root/'installed-twist_mux-package.xml').write_bytes(
            (mux_prefix/'share/twist_mux/package.xml').read_bytes())
        rclpy.init(args=[])
        node = rclpy.create_node('handover_observer', namespace=ns, enable_rosout=False,
                                start_parameter_services=False)

        def record(kind, message, source_name=None):
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            row = {'t': time.monotonic() - started, 'phase': phase, 'kind': kind,
                   'stamp_ns': stamp, 'frame': message.header.frame_id, 'velocity': vector(message)}
            if source_name is not None:
                row['source'] = source_name
            trace.append(row)

        node.create_subscription(TwistStamped, 'platform/cmd_vel', lambda m: record('mux', m), 100)
        node.create_subscription(TwistStamped, 'platform/cmd_vel_limited', lambda m: record('guard', m), 100)
        node.create_subscription(Bool, 'platform/emergency_stop_mux',
                                 lambda m: relay.append((time.monotonic()-started, m.data)), 100)
        publishers = {key: node.create_publisher(TwistStamped, entry['topic'], 10)
                      for key, entry in policy['topics'].items()}
        locks = {key: node.create_publisher(Bool, entry['topic'], 10)
                 for key, entry in policy['locks'].items() if key != 'e_stop'}
        stop = node.create_publisher(Bool, 'platform/emergency_stop',
                                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        for command, name in [
            (['ros2', 'run', 'twist_mux', 'twist_mux', '--ros-args', '-r', '__ns:='+ns,
              '-r', 'cmd_vel_out:=platform/cmd_vel', '--params-file', str(root/'isolated-policy.yaml')], 'mux.log'),
            ([sys.executable, str(guard), '--namespace', ns, '--original-manual-speeds'], 'guard.log')]:
            handle = (root/name).open('w')
            handles.append(handle)
            children.append(subprocess.Popen(command, stdout=handle, stderr=handle, start_new_session=True))

        def spin(duration):
            end = time.monotonic()+duration
            while time.monotonic() < end:
                if any(child.poll() is not None for child in children):
                    raise RuntimeError('isolated mux or guard exited')
                rclpy.spin_once(node, timeout_sec=min(.005, max(0., end-time.monotonic())))

        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            if (all(pub.get_subscription_count() for pub in publishers.values())
                    and all(pub.get_subscription_count() for pub in locks.values())
                    and stop.get_subscription_count() and node.count_publishers(ns+'/platform/cmd_vel_limited')
                    and node.count_subscribers(ns+'/platform/emergency_stop_mux') >= 2):
                break
            spin(.05)
        else:
            raise RuntimeError('private mux/guard discovery timed out')

        def check(name, passed):
            checks.append({'name': name, 'passed': bool(passed)})

        order = sorted(policy['topics'], key=lambda key: policy['topics'][key]['priority'])
        speeds = {key: [.2 + .1*i, .01*i, .02*i] for i, key in enumerate(order)}

        def run(name, duration, active=(), stopped=False, overrides=None):
            nonlocal phase
            phase = name
            mark, beginning = len(trace), time.monotonic()-started
            end = time.monotonic()+duration
            while time.monotonic() < end:
                stop.publish(Bool(data=stopped))
                for publisher in locks.values():
                    publisher.publish(Bool(data=False))
                for key in active:
                    message = TwistStamped()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.header.frame_id = 'base_link'
                    values = (overrides or {}).get(key, speeds[key])
                    message.twist.linear.x, message.twist.linear.y, message.twist.angular.z = values
                    publishers[key].publish(message)
                    record('input', message, key)
                spin(.04)
            return trace[mark:], beginning

        def nonzero(rows, kind='guard', after=0.):
            return [row for row in rows if row['kind'] == kind and row['t'] >= after
                    and any(row['velocity'])]

        run('released_idle', .2)
        for index, key in enumerate(order):
            rows, beginning = run('takeover_'+key, .4, order[:index+1])
            steady = nonzero(rows, after=beginning+.15)
            check(key+' takes priority through guard over continuously active lower sources',
                  steady and all(row['velocity'] == speeds[key] for row in steady))

        for index in range(len(order)-1, 0, -1):
            removed, resumed = order[index], order[index-1]
            last_source_input = next(row['t'] for row in reversed(trace)
                                     if row['kind']=='input' and row['source']==removed)
            # The source just removed was publishing until the preceding phase ended.
            rows, beginning = run('return_'+removed+'_to_'+resumed,
                                  policy['topics'][removed]['timeout']+.35, order[:index])
            returning = [row for row in nonzero(rows) if row['velocity'] == speeds[resumed]]
            zeros = [row for row in rows if row['kind'] == 'guard' and not any(row['velocity'])]
            delay = returning[0]['t']-last_source_input if returning else None
            zero_delay = zeros[0]['t']-last_source_input if zeros else None
            transitions.append({'from': removed, 'to': resumed,
                                'return_after_last_source_input_seconds': delay,
                                'guard_zero_after_last_source_input_seconds': zero_delay})
            check(removed+' expiry returns only fresh '+resumed+' commands',
                  returning and delay >= policy['topics'][removed]['timeout']-.03
                  and delay <= policy['topics'][removed]['timeout']+.2
                  and all(row['velocity'] == speeds[resumed] for row in nonzero(rows)))
            check(removed+' silence produces guard zero before mux handover',
                  zeros and zero_delay <= .2 and returning
                  and zeros[0]['t'] < returning[0]['t'])

        rows, beginning = run('all_sources_silent', .7)
        check('all-source silence produces sustained zero without replay',
              not nonzero(rows) and any(row['kind']=='guard' and row['t']>beginning+.2 for row in rows))
        run('before_stop', .25, order)
        rows, beginning = run('asserted_stop_with_all_sources', .5, order, stopped=True)
        check('firmware stop relay suppresses all ongoing priorities',
              not nonzero(rows, after=beginning+.15)
              and not nonzero(rows, 'mux', after=beginning+.15)
              and relay and all(value for t, value in relay if t>beginning+.15))
        rows, beginning = run('stop_released_no_input', .7)
        check('stop release does not replay a prior mux or guard command',
              not nonzero(rows) and not nonzero(rows, 'mux'))
        rows, beginning = run('fresh_low_priority_after_release', .3, [order[0]])
        check('fresh lowest-priority request resumes after stop release',
              nonzero(rows, after=beginning+.1)
              and all(row['velocity']==speeds[order[0]] for row in nonzero(rows)))
        boost = [1.2, 1.2, 1.5]
        rows, beginning = run('original_manual_boost_envelope', .35, [order[0]],
                               overrides={order[0]: boost})
        check('original manual diagonal boost and yaw reach guard output unchanged',
              nonzero(rows, after=beginning+.1)
              and all(row['velocity']==boost for row in nonzero(rows, after=beginning+.1)))
        inputs = {row['stamp_ns']: row for row in trace if row['kind']=='input'}
        outputs = nonzero(trace)
        check('every nonzero guarded output preserves the exact input stamp frame and velocity',
              outputs and all(row['stamp_ns'] in inputs and row['frame']==inputs[row['stamp_ns']]['frame']
                              and row['velocity']==inputs[row['stamp_ns']]['velocity'] for row in outputs))
        check('every nonzero guarded output corresponds to an input received within 100 ms',
              outputs and all(0 <= row['t']-inputs[row['stamp_ns']]['t'] <= .1 for row in outputs))
        run('final_stop', .2, stopped=True)
        report['passed'] = all(check['passed'] for check in checks)
    except Exception as error:
        report['passed'] = False
        report['error'] = repr(error)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=3)
        for handle in handles:
            handle.close()
        (root/'trace.json').write_text(json.dumps(trace, indent=2)+'\n')
        (root/'stop-relay.json').write_text(json.dumps(relay, indent=2)+'\n')
        (root/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
