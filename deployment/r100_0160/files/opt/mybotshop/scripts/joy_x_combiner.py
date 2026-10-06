#!/usr/bin/env python3
# Combines both F710 sticks' vertical axes into linear.x (summed, clamped),
# right stick horizontal into linear.y, left stick horizontal into angular.z.
# Replaces teleop_twist_joy_node's output on the twist_mux "joy" input because
# that node can only read one physical axis per twist field.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

AXIS_LEFT_X = 0     # yaw
AXIS_LEFT_Y = 1     # linear.x (left stick contribution)
AXIS_RIGHT_X = 3    # linear.y
AXIS_RIGHT_Y = 4     # linear.x (right stick contribution)
ENABLE_BUTTON = 4   # L1 - normal-speed deadman
BOOST_BUTTON = 5     # R1 - same controls, boosted speed (confirmed live)
BOOST_MULTIPLIER = 3.0

SCALE_LINEAR_X = 0.4
SCALE_LINEAR_Y = 0.4
SCALE_ANGULAR_YAW = 0.5


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class JoyXCombiner(Node):
    def __init__(self):
        super().__init__('joy_x_combiner')
        self.pub = self.create_publisher(TwistStamped, 'joy_teleop/cmd_vel_combined', 10)
        self.sub = self.create_subscription(Joy, 'joy_teleop/joy', self.on_joy, 10)

    def on_joy(self, msg: Joy):
        axes = msg.axes
        buttons = msg.buttons

        normal = len(buttons) > ENABLE_BUTTON and buttons[ENABLE_BUTTON]
        boosted = len(buttons) > BOOST_BUTTON and buttons[BOOST_BUTTON]
        enabled = normal or boosted

        # Don't publish at all while neither deadman is held. twist_mux treats
        # this input as "active" for as long as messages keep arriving
        # (0.5s timeout) - if we always publish (even zeros), we permanently
        # occupy priority #10 and mask out lower-priority sources like Nav2's
        # autonomous output, even when nobody is actually driving.
        if not enabled or len(axes) <= max(AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X, AXIS_RIGHT_Y):
            return

        multiplier = BOOST_MULTIPLIER if boosted else 1.0

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        combined_x = clamp(axes[AXIS_LEFT_Y] + axes[AXIS_RIGHT_Y], -1.0, 1.0)
        out.twist.linear.x = combined_x * SCALE_LINEAR_X * multiplier
        out.twist.linear.y = axes[AXIS_RIGHT_X] * SCALE_LINEAR_Y * multiplier
        out.twist.angular.z = axes[AXIS_LEFT_X] * SCALE_ANGULAR_YAW * multiplier

        self.pub.publish(out)


def main():
    rclpy.init()
    node = JoyXCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
