#!/usr/bin/env python3
"""Record what the follow stack decided, for follow_metrics.

Writes one JSON object per line to ``--output``, timestamped with the node's
clock (simulation time in simulation):

* ``leader``: LeaderState stamp, state, track id and odometry-frame position;
* ``status``: follow_controller's ``mode:reason``;
* ``monitor``: collision-monitor action changes (STOP counts as a safety stop);
* ``odom``: the robot's odometry pose, so metrics can map odometry-frame
  targets through simulator truth even when odometry drifts.

Stops on SIGINT or SIGTERM and flushes on exit.
"""
import argparse
import json
import math
import signal


def main(argv=None):
    import rclpy
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    from leader_follow_interfaces.msg import LeaderState

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--output', required=True)
    parser.add_argument('--namespace', default='r100_0001')
    parser.add_argument('--odom-topic', default='platform/odom')
    parser.add_argument('--wall-time', action='store_true', help='use wall time instead of /clock')
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('follow_recorder', namespace=args.namespace)
    if not args.wall_time:
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    out = open(args.output, 'w')

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def write(kind, **fields):
        out.write(json.dumps({'t': round(now(), 4), 'kind': kind, **fields}) + '\n')

    def on_leader(m):
        write('leader', stamp=stamp(m.header), state=int(m.state), reason=m.reason, has_leader=bool(m.has_leader),
              track_id=int(m.track_id), x=m.position.x, y=m.position.y, frame=m.header.frame_id)

    def on_odom(m):
        q = m.pose.pose.orientation
        write('odom', stamp=stamp(m.header), x=m.pose.pose.position.x, y=m.pose.pose.position.y,
              yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    last_action = [None]

    def on_monitor(m):
        if m.action_type != last_action[0]:
            last_action[0] = m.action_type
            write('monitor', action=int(m.action_type), polygon=m.polygon_name)

    node.create_subscription(LeaderState, 'leader_follow/leader', on_leader, 50)
    node.create_subscription(String, 'leader_follow/follow_status', lambda m: write('status', status=m.data), 50)
    node.create_subscription(CollisionMonitorState, 'collision_monitor_state', on_monitor, 50)
    node.create_subscription(Odometry, args.odom_topic, on_odom, 50)

    def stop(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        out.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
