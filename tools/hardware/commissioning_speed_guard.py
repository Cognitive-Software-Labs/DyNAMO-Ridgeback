#!/usr/bin/env python3
"""Stop/freshness gate with commissioning or original manual speed limits."""
import argparse
import copy
import math
import re
import time

MAX_PLANAR = 0.05
MAX_YAW = 0.1
# Original installed MyBotShop joy_x_combiner: 0.4 m/s and 0.5 rad/s,
# multiplied by three with R1. Keep per-axis diagonal behavior in manual mode.
MANUAL_MAX_AXIS = 1.2
MANUAL_MAX_YAW = 1.5
COMMAND_AGE_NS = 100_000_000
FUTURE_TOLERANCE_NS = 5_000_000
STOP_FRESH_SECONDS = 2.5


def clamp_command(components, stamp_ns, now_ns, frame, original_manual_speeds=False):
    """Return planar components or None for malformed/unsupported input."""
    if (len(components) != 6 or not all(math.isfinite(v) for v in components)
            or any(components[i] != 0 for i in (2, 3, 4))
            or frame not in ('', 'base_link') or stamp_ns <= 0
            or not -FUTURE_TOLERANCE_NS <= now_ns - stamp_ns <= COMMAND_AGE_NS):
        return None
    x, y, _, _, _, yaw = components
    if original_manual_speeds:
        return (max(-MANUAL_MAX_AXIS, min(MANUAL_MAX_AXIS, x)),
                max(-MANUAL_MAX_AXIS, min(MANUAL_MAX_AXIS, y)),
                max(-MANUAL_MAX_YAW, min(MANUAL_MAX_YAW, yaw)))
    # Scaling before hypot also avoids overflow on finite extreme inputs.
    biggest = max(abs(x), abs(y))
    if biggest > MAX_PLANAR:
        norm = math.hypot(x / biggest, y / biggest)
        x, y = (x / biggest) * MAX_PLANAR / norm, (y / biggest) * MAX_PLANAR / norm
    else:
        norm = math.hypot(x, y)
        if norm > MAX_PLANAR:
            x, y = x * MAX_PLANAR / norm, y * MAX_PLANAR / norm
    return x, y, max(-MAX_YAW, min(MAX_YAW, yaw))


class Gate:
    """Only callbacks forward nonzero commands; timer never replays them."""
    def __init__(self, require_test_permit=False, original_manual_speeds=False):
        if require_test_permit and original_manual_speeds:
            raise ValueError('bounded test permits require commissioning speed limits')
        self.stop = True
        self.stop_time = None
        self.input_time = None
        self.input_stamp = None
        self.previous_ros = None
        self.require_test_permit = require_test_permit
        self.original_manual_speeds = original_manual_speeds
        self.permit_time = None
        self.permit_started = None
        self.permit = False

    def clock(self, now_ns):
        if self.previous_ros is not None and now_ns < self.previous_ros:
            self.input_time = self.input_stamp = None
        self.previous_ros = now_ns

    def update_stop(self, value, monotonic):
        self.stop = bool(value)
        self.stop_time = monotonic
        # Stop/release never resumes a buffered command.
        self.input_time = self.input_stamp = None
        self.permit = False
        self.permit_time = None
        if value:
            self.permit_started = None

    def update_permit(self, value, monotonic):
        # A false heartbeat cannot reset/extend the one-shot motion budget.
        self.permit = bool(value)
        self.permit_time = monotonic
        if value and not self.stop and self.permit_started is None:
            self.permit_started = monotonic
        if not value:
            self.input_time = self.input_stamp = None

    def permit_blocks(self, monotonic):
        return self.require_test_permit and (
            not self.permit or self.permit_time is None or self.permit_started is None
            or not 0 <= monotonic-self.permit_time <= .2
            or not 0 <= monotonic-self.permit_started < 1.)

    def stop_blocks(self, monotonic):
        return (self.stop or self.permit_blocks(monotonic) or self.stop_time is None
                or not 0 <= monotonic - self.stop_time <= STOP_FRESH_SECONDS)

    def command(self, components, stamp_ns, now_ns, monotonic, frame):
        self.clock(now_ns)
        result = clamp_command(components, stamp_ns, now_ns, frame,
                               self.original_manual_speeds)
        if self.stop_blocks(monotonic) or result is None:
            self.input_time = self.input_stamp = None
            return None
        self.input_time, self.input_stamp = monotonic, stamp_ns
        return result

    def needs_zero(self, now_ns, monotonic):
        self.clock(now_ns)
        return (self.stop_blocks(monotonic) or self.input_time is None
                or not 0 <= monotonic - self.input_time <= .1
                or not -FUTURE_TOLERANCE_NS <= now_ns - self.input_stamp <= COMMAND_AGE_NS)


def namespace(value):
    parts = value.strip('/').split('/')
    if not all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', p) for p in parts):
        raise argparse.ArgumentTypeError('a nonempty ROS namespace is required')
    return '/' + '/'.join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', type=namespace, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--require-test-permit', action='store_true',
                        help='temporary test mode: require fresh permits with a one-second budget per physical stop cycle')
    mode.add_argument('--original-manual-speeds', action='store_true',
                      help='preserve original L1/R1 commands; cap each planar axis at 1.2 m/s and yaw at 1.5 rad/s')
    args = parser.parse_args()
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Bool
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('commissioning_speed_guard', namespace=args.namespace,
                                 use_global_arguments=False, start_parameter_services=False)
        gate = Gate(require_test_permit=args.require_test_permit,
                    original_manual_speeds=args.original_manual_speeds)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub = node.create_publisher(TwistStamped, 'platform/cmd_vel_limited', 1)
        # Native firmware offers BEST_EFFORT; twist_mux locks request RELIABLE.
        # A separate topic avoids a loop or refreshing stale firmware reports.
        stop_pub = node.create_publisher(Bool, 'platform/emergency_stop_mux', 1)

        def publish_mux_stop():
            stop_pub.publish(Bool(data=gate.stop_blocks(time.monotonic())))

        def zero():
            message = TwistStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            pub.publish(message)

        def on_stop(message):
            # Repeated false state updates do not invalidate a fresh command.
            if gate.stop != bool(message.data):
                gate.update_stop(message.data, time.monotonic())
                zero()
            else:
                gate.stop_time = time.monotonic()
            publish_mux_stop()

        def on_command(message):
            values = (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
                      message.twist.angular.x, message.twist.angular.y, message.twist.angular.z)
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            limited = gate.command(values, stamp, node.get_clock().now().nanoseconds,
                                   time.monotonic(), message.header.frame_id)
            if limited is None:
                zero()
            else:
                output = copy.deepcopy(message)
                output.twist.linear.x, output.twist.linear.y, output.twist.angular.z = limited
                pub.publish(output)  # Preserve source stamp; never refresh a nonzero command.

        def on_permit(message):
            gate.update_permit(message.data, time.monotonic())
            publish_mux_stop()
            if gate.stop_blocks(time.monotonic()):
                zero()

        def watchdog():
            publish_mux_stop()  # Missing/stale firmware state asserts the mux lock.
            if gate.needs_zero(node.get_clock().now().nanoseconds, time.monotonic()):
                zero()

        node.create_subscription(TwistStamped, 'platform/cmd_vel', on_command, qos)
        node.create_subscription(Bool, 'platform/emergency_stop', on_stop, qos)
        if args.require_test_permit:
            node.create_subscription(Bool, 'platform/commissioning_test_permit', on_permit, 1)
        node.create_timer(.05, watchdog)
        node.get_logger().info(
            'Original manual limits: each planar axis 1.2 m/s; yaw 1.5 rad/s'
            if args.original_manual_speeds else
            'Commissioning caps: planar norm 0.05 m/s; yaw 0.1 rad/s')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
