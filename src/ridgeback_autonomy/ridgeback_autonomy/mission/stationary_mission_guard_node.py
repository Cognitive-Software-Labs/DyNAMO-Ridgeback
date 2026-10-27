#!/usr/bin/env python3
"""Idle ownership acknowledgment for non-driving mission integration tests.

This node is intentionally not a replacement for MissionGuard.  It owns no
navigation client and can only model the case where no explorer goal exists.
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


class StationaryMissionGuardNode(Node):
    def __init__(self) -> None:
        super().__init__('stationary_mission_guard')
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._paused = False
        self._status_publisher = self.create_publisher(String, 'mission/status', qos)
        self._pause_service = self.create_service(
            Trigger, 'mission/pause', self._request_pause)
        self._publish_status('Stationary test interface ready')
        self.get_logger().warning(
            'Using stationary mock handoff: no explorer or Nav2 goal is owned')

    def _request_pause(self, request, response):
        del request
        self._paused = True
        response.success = True
        response.message = (
            'Stationary test handoff acknowledged; no explorer goal exists')
        self._publish_status(response.message)
        return response

    def _publish_status(self, reason: str) -> None:
        payload = {
            'required': False,
            'paused': self._paused,
            'healthy': True,
            'reason': reason,
            'pause_requested': self._paused,
            'handoff_safe': self._paused,
            'cancellation': 'none',
            'cancellation_overdue': False,
            'goal_pending': False,
            'goal_active': False,
            'ownership_mode': 'stationary_mock',
        }
        self._status_publisher.publish(
            String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StationaryMissionGuardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
