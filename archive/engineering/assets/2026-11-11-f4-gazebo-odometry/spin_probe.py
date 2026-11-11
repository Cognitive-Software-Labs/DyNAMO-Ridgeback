"""Fake follow stack: publish leader_follow/leader + cmd_vel (spin, then drive), log odom/imu yaw."""
import json, math, sys, rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from leader_follow_interfaces.msg import LeaderState
out = open(sys.argv[1], 'w')
rclpy.init(); node = Node('spin_probe', namespace='r100_0001')
node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
cmd = node.create_publisher(TwistStamped, 'cmd_vel', 10); leader = node.create_publisher(LeaderState, 'leader_follow/leader', 10)
def now(): return node.get_clock().now().nanoseconds * 1e-9
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
imu = {'yaw': 0., 't': None}
def on_imu(m):
    t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
    if imu['t'] is not None and 0 < t - imu['t'] < .5: imu['yaw'] += m.angular_velocity.z * (t - imu['t'])
    imu['t'] = t
    out.write(json.dumps({'t': t, 'imu_yaw': imu['yaw'], 'imu_wz': m.angular_velocity.z}) + '\n')
for topic, key in (('platform/odom', 'raw'), ('platform/odom/filtered', 'filtered')):
    node.create_subscription(Odometry, topic, lambda m, k=key: out.write(json.dumps({'t': m.header.stamp.sec + m.header.stamp.nanosec*1e-9, k+'_yaw': yaw(m.pose.pose.orientation), k+'_x': m.pose.pose.position.x, k+'_y': m.pose.pose.position.y, k+'_wz': m.twist.twist.angular.z}) + '\n'), 50)
node.create_subscription(Imu, 'sensors/imu_0/data', on_imu, 50)
start = [None]
def tick():
    t = now(); leader.publish(LeaderState())
    if start[0] is None and t > 0: start[0] = t
    el = 0 if start[0] is None else t - start[0]
    m = TwistStamped(); m.header.stamp = node.get_clock().now().to_msg(); m.header.frame_id = 'base_link'
    if 8 < el < 14: m.twist.linear.y = .4            # strafe
    elif 14 < el < 20: m.twist.linear.x = .6; m.twist.linear.y = .2; m.twist.angular.z = .3   # drive, strafe, turn
    cmd.publish(m)
node.create_timer(.05, tick)
try: rclpy.spin(node)
except (KeyboardInterrupt, Exception): pass
finally: out.flush(); out.close()
