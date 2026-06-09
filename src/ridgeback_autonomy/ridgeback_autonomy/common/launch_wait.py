#!/usr/bin/env python3
"""Block until ROS readiness conditions are met, then exit 0.

Used as a launch "gate": an ``ExecuteProcess`` runs this, and a
``RegisterEventHandler(OnProcessExit(...))`` fires the next bringup stage when it
exits. This replaces fixed ``TimerAction`` delays so each stage starts when its
prerequisites actually exist, with a ``--timeout`` safety fallback so the chain
never deadlocks (worst case ~= the old fixed wait).

Conditions (all ANDed; each flag repeatable):
  --topic NAME        wait until count_publishers(NAME) > 0  (QoS-agnostic)
  --tf PARENT CHILD   wait until a TF PARENT->CHILD lookup succeeds (listens on /tf)
  --service NAME      wait until the service NAME appears on the graph
  --timeout SEC       give up after SEC and exit 0 anyway (default 60)

Topic checks use ``count_publishers`` (a publisher existing), not message
receipt, so they don't care about the topic's QoS (scan is best-effort, map is
transient-local, ...). For namespaced TF (published on /<ns>/tf, not /tf) prefer
a topic proxy over --tf, e.g. ``--topic /<ns>/map`` already implies the
map->odom->base_link chain is alive.
"""

import argparse
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    ap = argparse.ArgumentParser(description='Wait for ROS readiness conditions, then exit 0.')
    ap.add_argument('--topic', action='append', default=[], metavar='NAME')
    ap.add_argument('--tf', action='append', nargs=2, default=[], metavar=('PARENT', 'CHILD'))
    ap.add_argument('--service', action='append', default=[], metavar='NAME')
    ap.add_argument('--timeout', type=float, default=60.0)
    # argparse chokes on ROS args appended by `ros2 run`; ignore them.
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Node('launch_wait')
    log = node.get_logger()

    tf_buffer = None
    if args.tf:
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)

    pending_topics = set(args.topic)
    pending_tf = {tuple(p) for p in args.tf}
    pending_srv = set(args.service)

    summary = []
    if pending_topics:
        summary.append(f'topics={sorted(pending_topics)}')
    if pending_tf:
        summary.append(f'tf={sorted(pending_tf)}')
    if pending_srv:
        summary.append(f'services={sorted(pending_srv)}')
    log.info(f"waiting for {', '.join(summary) or '(nothing)'} (timeout {args.timeout:.0f}s)")

    start = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.monotonic() - start

        for t in list(pending_topics):
            if node.count_publishers(t) > 0:
                pending_topics.discard(t)
                log.info(f'[{elapsed:5.1f}s] topic ready: {t}')

        if pending_tf:
            services_names = None
            for (parent, child) in list(pending_tf):
                if tf_buffer.can_transform(parent, child, Time(), Duration(seconds=0.0)):
                    pending_tf.discard((parent, child))
                    log.info(f'[{elapsed:5.1f}s] tf ready: {parent}->{child}')

        if pending_srv:
            available = {name for name, _ in node.get_service_names_and_types()}
            for s in list(pending_srv):
                if s in available:
                    pending_srv.discard(s)
                    log.info(f'[{elapsed:5.1f}s] service ready: {s}')

        if not (pending_topics or pending_tf or pending_srv):
            log.info(f'all conditions met in {elapsed:.1f}s')
            break
        if elapsed >= args.timeout:
            log.warn(
                f'timeout after {elapsed:.1f}s; proceeding anyway. unmet: '
                f'topics={sorted(pending_topics)} tf={sorted(pending_tf)} '
                f'services={sorted(pending_srv)}'
            )
            break

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
