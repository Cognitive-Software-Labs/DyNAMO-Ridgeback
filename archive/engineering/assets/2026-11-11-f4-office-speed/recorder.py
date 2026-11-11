"""Record LeaderState and follow status (sim time) to jsonl until killed."""
import json, sys, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from leader_follow_interfaces.msg import LeaderState
out = open(sys.argv[1], 'w')
rclpy.init(); node = Node('f4_recorder', namespace='r100_0001')
node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
status = ['']
def now(): return node.get_clock().now().nanoseconds * 1e-9
def on_status(m): status[0] = m.data; out.write(json.dumps({'t': now(), 'status': m.data}) + '\n')
def on_leader(m):
    out.write(json.dumps({'t': now(), 'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 'state': m.state,
        'reason': m.reason, 'has_leader': m.has_leader, 'track_id': m.track_id,
        'x': m.position.x, 'y': m.position.y, 'frame': m.header.frame_id}) + '\n')
node.create_subscription(String, 'leader_follow/follow_status', on_status, 10)
from geometry_msgs.msg import TwistStamped
for topic in ('cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel'):
    node.create_subscription(TwistStamped, topic, lambda m, k=topic: out.write(json.dumps({'t': now(), 'cmd': k, 'vx': m.twist.linear.x, 'vy': m.twist.linear.y, 'wz': m.twist.angular.z}) + chr(10)), 10)
node.create_subscription(LeaderState, 'leader_follow/leader', on_leader, 10)
try: rclpy.spin(node)
except (KeyboardInterrupt, Exception): pass
finally: out.flush(); out.close()
