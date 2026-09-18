#!/usr/bin/env python3
"""Intel-side health consumer for stationary deployments, with no compute imports."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from ridgeback_interfaces.msg import LocalizationHealth
from ridgeback_localization.contracts import HEALTH_TOPIC


class LocalizationStatusNode(Node):
    def __init__(self):
        super().__init__('localization_status')
        self.declare_parameter('health_timeout', 3.0)
        self.latest = None
        self.received = 0.0
        self.last_state = None
        self.create_subscription(LocalizationHealth, HEALTH_TOPIC, self.observe, 10)
        self.pub = self.create_publisher(String, 'localization/status', 10)
        self.create_timer(.5, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def observe(self, msg):
        self.latest, self.received = msg, time.monotonic()

    def report(self):
        missing = self.latest is None or time.monotonic()-self.received > float(self.get_parameter('health_timeout').value)
        state = 'missing' if missing else self.latest.state
        detail = 'No current remote health' if missing else self.latest.detail
        self.pub.publish(String(data=json.dumps(dict(mode='external', state=state, detail=detail))))
        if state != self.last_state:
            self.get_logger().info(f'External localization: {state}: {detail}')
            self.last_state = state


def main():
    rclpy.init(); node=LocalizationStatusNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
