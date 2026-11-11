"""drain_callbacks runs queued callbacks and stops without waiting."""
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from ros_io import drain_callbacks  # noqa: E402


class Owner:
    pass


def test_drain_runs_ready_callbacks_and_returns_when_empty():
    from std_msgs.msg import Int32
    context = rclpy.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('drain_test', context=context)
        received = []
        node.create_subscription(Int32, 'drain_test', lambda m: received.append(m.data), 10)
        publisher = node.create_publisher(Int32, 'drain_test', 10)
        owner = Owner()
        assert drain_callbacks(owner, node, 8) == 0
        for value in range(3):
            publisher.publish(Int32(data=value))
        deadline = time.monotonic() + 5.
        while len(received) < 3 and time.monotonic() < deadline:
            drain_callbacks(owner, node, 8)
        assert received == [0, 1, 2]
        executor = owner._executor
        drain_callbacks(owner, node, 8)
        assert owner._executor is executor
        start = time.perf_counter()
        assert drain_callbacks(owner, node, 8) == 0
        assert time.perf_counter() - start < .5
        executor.shutdown()
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
