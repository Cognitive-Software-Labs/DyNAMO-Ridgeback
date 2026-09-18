"""Latched producer epochs and explicit failures, without a heartbeat thread."""
import json
import uuid
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from ridgeback_localization.contracts import WORKER_STATE_TOPIC


class WorkerState:
    def __init__(self, node, component):
        self.component = component
        self.epoch = uuid.uuid4().hex
        self.previous = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = node.create_publisher(String, WORKER_STATE_TOPIC + '/' + component, qos)
        self.set('startup')

    def set(self, state, detail=''):
        value = (state, detail)
        if value != self.previous:
            self.pub.publish(String(data=json.dumps(dict(component=self.component,
                epoch=self.epoch, state=state, detail=detail))))
            self.previous = value
