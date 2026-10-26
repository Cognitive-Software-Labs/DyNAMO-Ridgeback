"""Latched producer epochs and explicit failures, without a heartbeat thread."""
import json
import threading
import time
import uuid
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from ridgeback_localization.contracts import WORKER_STATE_TOPIC


# Total time teardown waits for a node's workers. A worker inside a model call
# finishes that call first; launch escalates to SIGTERM five seconds after the
# SIGINT, so a longer wait would only be cut short.
WORKER_STOP_BUDGET_SEC = 5.0


class WorkerState:
    def __init__(self, node, component):
        self.component = component
        self.epoch = uuid.uuid4().hex
        self.previous = None
        self.closed = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = node.create_publisher(String, WORKER_STATE_TOPIC + '/' + component, qos)
        self.set('startup')

    def set(self, state, detail=''):
        if self.closed:
            return
        value = (state, detail)
        if value != self.previous:
            self.pub.publish(String(data=json.dumps(dict(component=self.component,
                epoch=self.epoch, state=state, detail=detail))))
            self.previous = value

    def close(self):
        """Stop publishing: teardown has begun, and a state now would be noise."""
        self.closed = True


def join_workers(threads, logger, budget_sec=WORKER_STOP_BUDGET_SEC):
    """Wait for ``threads`` to finish, sharing one budget; warn about any still alive.

    Called after the workers were told to stop and before the node's ROS
    resources are destroyed, so no worker is left publishing into them.
    """
    deadline = time.monotonic() + budget_sec
    for thread in threads:
        if thread is threading.current_thread():
            continue
        thread.join(timeout=max(0.0, deadline - time.monotonic()))
    alive = [thread.name for thread in threads if thread.is_alive()]
    if alive:
        logger.warning(
            f'Worker thread(s) {", ".join(alive)} still running after '
            f'{budget_sec:.1f} s; destroying the node anyway.')
