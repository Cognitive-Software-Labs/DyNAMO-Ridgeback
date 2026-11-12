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
from leader_follow_interfaces.msg import People
from nav_msgs.msg import Odometry
def on_people(m):
    out.write(json.dumps({'t': now(), 'people_stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 'people': [
        [p.id, round(p.position.x, 3), round(p.position.y, 3), round(p.velocity.x, 3), round(p.velocity.y, 3),
         round(p.confidence, 3), len(p.appearance) > 0] for p in m.people]}) + chr(10))
def on_odom(m):
    q = m.pose.pose.orientation
    import math
    out.write(json.dumps({'t': now(), 'odom_stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
        'ox': m.pose.pose.position.x, 'oy': m.pose.pose.position.y,
        'oyaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}) + chr(10))
node.create_subscription(People, 'leader_follow/people', on_people, 10)
node.create_subscription(People, 'leader_follow/moving_people', lambda m: out.write(json.dumps({'t': now(), 'moving_stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 'moving': [
    [p.id, round(p.position.x, 3), round(p.position.y, 3), round(p.velocity.x, 3), round(p.velocity.y, 3)] for p in m.people]}) + chr(10)), 10)
from nav2_msgs.msg import SpeedLimit, CollisionMonitorState
from nav_msgs.msg import Path as NavPath, OccupancyGrid
node.create_subscription(NavPath, 'plan', lambda m: out.write(json.dumps({'t': now(), 'plan_frame': m.header.frame_id, 'plan': [[round(q.pose.position.x,2), round(q.pose.position.y,2)] for q in m.poses[::4]]}) + chr(10)), 10)
last_grid = [0.]
def on_grid(m):
    if now() - last_grid[0] < .5: return
    last_grid[0] = now()
    import base64
    out.write(json.dumps({'t': now(), 'grid_frame': m.header.frame_id, 'res': m.info.resolution, 'w': m.info.width, 'h': m.info.height,
        'ox': m.info.origin.position.x, 'oy': m.info.origin.position.y, 'data': base64.b64encode(bytes((v % 256) for v in m.data)).decode()}) + chr(10))
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
node.create_subscription(OccupancyGrid, 'local_costmap/costmap', on_grid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
node.create_subscription(SpeedLimit, 'speed_limit', lambda m: out.write(json.dumps({'t': now(), 'speed_limit': m.speed_limit}) + chr(10)), 10)
node.create_subscription(CollisionMonitorState, 'collision_monitor_state', lambda m: out.write(json.dumps({'t': now(), 'monitor_action': m.action_type, 'polygon': m.polygon_name}) + chr(10)), 10)
import os
node.create_subscription(Odometry, os.environ.get('REC_ODOM', 'platform/odom/filtered'), on_odom, 10)
try: rclpy.spin(node)
except (KeyboardInterrupt, Exception): pass
finally: out.flush(); out.close()
