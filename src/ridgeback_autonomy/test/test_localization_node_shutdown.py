"""The detector and measurement nodes stop their workers before ROS goes away.

Each entry point runs in a child process and is signalled with ``os.kill``:
once, and twice 40 ms apart -- the double delivery a group signal plus a
launch relay produces. The detector is caught inside a (stubbed) inference
call, the case where the worker used to publish into a dead context and a
second signal broke the join.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time

import pytest


DETECTOR_CHILD = r'''
import time

from ridgeback_localization import detector_node
from ridgeback_localization.core import detection


class SlowDetector:
    def load(self):
        pass

    def detect(self, image, **kwargs):
        print('DETECT', flush=True)
        time.sleep(0.5)
        return []


# The production owlv2 backend builds its engine through this name.
detection.OwlV2Detector = lambda *args, **kwargs: SlowDetector()
detector_node.main()
print('EXIT', flush=True)
'''

IDLE_CHILD = r'''
import importlib
import sys

module = importlib.import_module(sys.argv[1])
spin = module.spin_until_interrupted


def announced_spin(node, **kwargs):
    print('SPINNING', flush=True)
    return spin(node, **kwargs)


module.spin_until_interrupted = announced_spin
module.main()
print('EXIT', flush=True)
'''

FEEDER = r'''
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node('color_feeder')
publisher = node.create_publisher(Image, 'sensors/camera_0/color/image', qos_profile_sensor_data)
frame = Image(height=6, width=8, encoding='rgb8', step=24)
frame.data = np.zeros(6 * 8 * 3, dtype=np.uint8).tobytes()
node.create_timer(1.0 / 30.0, lambda: publisher.publish(frame))
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
'''

FORBIDDEN = ('Traceback', 'context is invalid', 'terminate called')
# DDS discovery ports are 7400 + 250 * domain. Keep these test domains below
# Linux's ephemeral ports: domain 167 uses 49150, which another process can own.
TEST_DOMAIN_BASE = 90


class Child:
    def __init__(self, env: dict[str, str], script: str, *args: str) -> None:
        self.process = subprocess.Popen(
            [sys.executable, '-c', script, *args],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
        )
        self.lines: list[str] = []

    def expect(self, prefix: str, timeout_sec: float = 20.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            line = self.process.stdout.readline()
            if not line:
                break
            self.lines.append(line.rstrip('\n'))
            if line.startswith(prefix):
                return
        self.process.kill()
        pytest.fail(f'no {prefix!r} line; output:\n' + '\n'.join(self.lines))

    def interrupt(self, count: int) -> None:
        for index in range(count):
            if index:
                time.sleep(0.04)
            os.kill(self.process.pid, signal.SIGINT)

    def finish(self, timeout_sec: float = 15.0) -> tuple[int, str]:
        # Not communicate(): with a timeout it reads the raw pipe and drops
        # whatever readline() has already buffered.
        try:
            self.process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        rest = self.process.stdout.read()
        self.lines.extend(rest.splitlines())
        return self.process.returncode, '\n'.join(self.lines)


def _env(tmp_path, domain: int) -> dict[str, str]:
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain)
    env['ROS_LOG_DIR'] = str(tmp_path / 'ros_logs')
    return env


def _assert_clean_exit(child: Child) -> None:
    returncode, output = child.finish()
    assert returncode == 0, output
    assert 'EXIT' in output, output
    for marker in FORBIDDEN:
        assert marker not in output, output


@pytest.mark.parametrize('signals', [1, 2])
def test_detector_interrupted_during_inference_exits_cleanly(tmp_path, signals) -> None:
    env = _env(tmp_path, TEST_DOMAIN_BASE + signals)
    feeder = Child(env, FEEDER)
    try:
        child = Child(env, DETECTOR_CHILD)
        child.expect('DETECT')
        child.interrupt(signals)
        _assert_clean_exit(child)
    finally:
        feeder.process.send_signal(signal.SIGINT)
        feeder.finish()


@pytest.mark.parametrize('signals', [1, 2])
@pytest.mark.parametrize('module', [
    'ridgeback_localization.mask_measurement_node',
    'ridgeback_localization.pointcloud_measurement_node',
])
def test_measurement_node_interrupted_while_spinning_exits_cleanly(
    tmp_path, module, signals,
) -> None:
    domain = TEST_DOMAIN_BASE + 3 + 2 * ('pointcloud' in module) + signals
    child = Child(_env(tmp_path, domain), IDLE_CHILD, module)
    child.expect('SPINNING')
    time.sleep(0.3)
    child.interrupt(signals)
    _assert_clean_exit(child)


@pytest.fixture
def ros_context():
    rclpy = pytest.importorskip('rclpy')
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_pointcloud_teardown_waits_for_a_frame_and_publishes_nothing_after_stop(
    ros_context, monkeypatch,
) -> None:
    from rclpy.node import Node
    from std_msgs.msg import Header

    from ridgeback_common.messages import build_detections_message
    from ridgeback_common.models import DetectionBatch
    import ridgeback_localization.pointcloud_measurement_node as pointcloud_module

    entered = threading.Event()
    release = threading.Event()
    original_batch = pointcloud_module.batch_from_detections_message

    def blocking_batch(message):
        entered.set()
        assert release.wait(timeout=5.0)
        return original_batch(message)

    monkeypatch.setattr(pointcloud_module, 'batch_from_detections_message', blocking_batch)
    node = pointcloud_module.TargetPointcloudMeasurementNode()
    published: list = []
    node.measurement_pub.publish = published.append
    states: list = []
    node.worker_state.pub.publish = states.append
    alive_at_destroy: list[bool] = []
    original_destroy = Node.destroy_node

    def recording_destroy(self):
        alive_at_destroy.append(
            self.worker_thread.is_alive() or self.pointcloud_worker_thread.is_alive())
        return original_destroy(self)

    monkeypatch.setattr(Node, 'destroy_node', recording_destroy)

    batch = DetectionBatch(image_width=8, image_height=6, detections=[])
    node.detections_callback(build_detections_message(batch, Header()))
    assert entered.wait(timeout=2.0), 'worker never started the frame'
    teardown = threading.Thread(target=node.destroy_node)
    teardown.start()
    deadline = time.monotonic() + 2.0
    while not node.stop_event.is_set() and time.monotonic() < deadline:
        time.sleep(0.005)
    release.set()
    teardown.join(timeout=5.0)

    assert not teardown.is_alive()
    assert alive_at_destroy == [False]
    assert published == []
    assert states == []
    assert not node.worker_thread.daemon and not node.pointcloud_worker_thread.daemon
