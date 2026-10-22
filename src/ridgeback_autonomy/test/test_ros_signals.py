"""Owned signal handling keeps the ROS context usable after a real signal.

Every case runs a child process and delivers signals with ``os.kill``: the
behavior under test is the handler installation, which an in-process test
would share with pytest itself.
"""

from __future__ import annotations

import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import time

import pytest


CHILD = r'''
import sys
import time

import rclpy
from std_srvs.srv import Trigger

from ridgeback_common.ros_signals import (
    init_owned, shielded, signals_held, spin_step, spin_until_interrupted)

mode = sys.argv[1]


def say(*words):
    print(*words, flush=True)


def call(node, client, timeout_sec=5.0):
    with signals_held():
        future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        spin_step(node, timeout_sec=0.05)
    return future.done() and future.result().success


def respond(_request, response):
    response.success = True
    return response


if mode == 'default':
    rclpy.init()
else:
    init_owned()
node = rclpy.create_node('signal_probe')
node.create_service(Trigger, 'signal_probe_trigger', respond)
client = node.create_client(Trigger, 'signal_probe_trigger')
assert client.wait_for_service(timeout_sec=10.0)

if mode in ('interrupt', 'shield'):
    say('READY')
    try:
        spin_until_interrupted(node)
    except KeyboardInterrupt:
        say('INTERRUPTED', time.monotonic())
    if mode == 'shield':
        with shielded() as received:
            say('SHIELDED')
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                spin_step(node, timeout_sec=0.05)
        say('RECORDED', len(received))
    say('CALL', call(node, client))
    if mode == 'shield':
        say('ARMED')
        try:
            spin_until_interrupted(node)
        except KeyboardInterrupt:
            say('RESTORED')

elif mode == 'storm':
    calls = 0
    node.create_timer(0.001, lambda: None)
    pending = client.call_async(Trigger.Request())
    say('READY')
    interrupts = 0
    announced = 0
    target = int(sys.argv[2])
    # The announcement stays inside the try: the next signal may land while
    # it is still being written.
    while announced < target:
        try:
            if announced < interrupts:
                announced = interrupts
                say('INT', announced)
            if interrupts >= target:
                continue
            if pending.done():
                calls += 1
                with signals_held():
                    pending = client.call_async(Trigger.Request())
            spin_step(node, timeout_sec=0.01)
        except KeyboardInterrupt:
            interrupts += 1
    say('CALLS', calls)
    say('CALL', call(node, client))

elif mode == 'default':
    say('READY')
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        say('INTERRUPTED')
    try:
        say('CALL', call(node, client))
    except Exception as exc:
        say('ERROR', type(exc).__name__, exc)

node.destroy_node()
rclpy.try_shutdown()
say('EXIT')
'''


class Child:
    def __init__(self, tmp_path: Path, domain: int, *args: str) -> None:
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(domain)
        env['ROS_LOG_DIR'] = str(tmp_path / 'ros_logs')
        self.process = subprocess.Popen(
            [sys.executable, '-c', CHILD, *args],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
        )
        self.lines: list[str] = []

    def expect(self, prefix: str, timeout_sec: float = 15.0) -> list[str]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            line = self.process.stdout.readline()
            if not line:
                break
            self.lines.append(line.rstrip('\n'))
            if line.startswith(prefix):
                return line.split()
        self.kill()
        pytest.fail(f'no {prefix!r} line; output:\n' + '\n'.join(self.lines))

    def signal(self, signum: int) -> float:
        sent = time.monotonic()
        os.kill(self.process.pid, signum)
        return sent

    def finish(self, timeout_sec: float = 15.0) -> tuple[int, str]:
        try:
            rest, _ = self.process.communicate(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            self.kill()
            rest, _ = self.process.communicate()
        self.lines.extend(rest.splitlines())
        return self.process.returncode, '\n'.join(self.lines)

    def kill(self) -> None:
        if self.process.poll() is None:
            self.process.kill()


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM], ids=['sigint', 'sigterm'])
def test_a_signal_interrupts_spinning_and_leaves_the_context_usable(tmp_path, signum) -> None:
    child = Child(tmp_path, 171 + int(signum == signal.SIGTERM), 'interrupt')
    child.expect('READY')
    time.sleep(0.3)
    sent = child.signal(signum)
    interrupted = float(child.expect('INTERRUPTED')[1])
    assert interrupted - sent <= 0.2
    assert child.expect('CALL') == ['CALL', 'True']
    returncode, output = child.finish()
    assert returncode == 0, output
    assert 'Traceback' not in output, output


def test_a_signal_during_shielded_cleanup_is_recorded_not_raised(tmp_path) -> None:
    child = Child(tmp_path, 173, 'shield')
    child.expect('READY')
    time.sleep(0.3)
    child.signal(signal.SIGINT)
    child.expect('SHIELDED')
    time.sleep(0.04)
    child.signal(signal.SIGINT)
    assert child.expect('RECORDED') == ['RECORDED', '1']
    assert child.expect('CALL') == ['CALL', 'True']
    child.expect('ARMED')
    time.sleep(0.3)
    child.signal(signal.SIGINT)
    child.expect('RESTORED')
    returncode, output = child.finish()
    assert returncode == 0, output
    assert 'Traceback' not in output, output
    assert 'Received SIGINT during cleanup' in output


def test_repeated_signals_inside_executor_internals_leave_ros_working(tmp_path) -> None:
    interrupts = 30
    child = Child(tmp_path, 174, 'storm', str(interrupts))
    child.expect('READY')
    rng = random.Random(7)
    for count in range(1, interrupts + 1):
        time.sleep(rng.uniform(0.0, 0.03))
        child.signal(signal.SIGINT)
        assert child.expect('INT') == ['INT', str(count)]
    assert int(child.expect('CALLS')[1]) > 0
    assert child.expect('CALL') == ['CALL', 'True']
    returncode, output = child.finish()
    assert returncode == 0, output
    assert 'Traceback' not in output, output


def test_default_rclpy_handlers_invalidate_the_context_before_cleanup(tmp_path) -> None:
    # The control: what the owned handlers exist to prevent.
    child = Child(tmp_path, 175, 'default')
    child.expect('READY')
    time.sleep(0.3)
    child.signal(signal.SIGINT)
    child.expect('INTERRUPTED')
    error = child.expect('ERROR')
    assert error[1] == 'RCLError'
    assert 'context is not valid' in ' '.join(error)
    child.finish()
