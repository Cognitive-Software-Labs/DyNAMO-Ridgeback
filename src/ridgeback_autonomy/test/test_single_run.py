"""Exercise shutdown ordering with real, harmless child sessions."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from ridgeback_autonomy.benchmarking import single_run


CHILD = r'''
import os, pathlib, signal, sys, time
root = pathlib.Path(sys.argv[1])
mode = sys.argv[2]
def mark(name):
    (root / name).write_text(str(os.getpid()))
def until(name, budget=5):
    deadline = time.monotonic() + budget
    while not (root / name).exists():
        if time.monotonic() > deadline:
            raise RuntimeError('missing ' + name)
        time.sleep(.01)
def interrupt(*args):
    raise KeyboardInterrupt
signal.signal(signal.SIGINT, interrupt)
signal.signal(signal.SIGTERM, interrupt)
if mode == 'environment':
    mark('environment.ready')
    try:
        while True:
            if (root / 'environment.fail').exists():
                sys.exit(7)
            if (root / 'ping').exists():
                mark('pong')
            time.sleep(.01)
    except KeyboardInterrupt:
        mark('environment.stopped')
else:
    until('environment.ready')
    mark('configuration.ready')
    mark('runner.done')
    if mode == 'failure':
        sys.exit(9)
    try:
        if mode == 'inspect':
            while True:
                time.sleep(.01)
    except KeyboardInterrupt:
        pass
    finally:
        mark('cleanup.started')
        mark('ping')
        until('pong')
        time.sleep(.25)
        if (root / 'environment.stopped').exists():
            raise RuntimeError('environment stopped during cleanup')
        mark('cleanup.done')
'''


def _wait(path: Path, process: subprocess.Popen, timeout: float = 8.0) -> None:
    deadline = time.monotonic() + timeout
    while not path.exists():
        assert process.poll() is None, f'supervisor exited before {path.name}'
        assert time.monotonic() < deadline, f'timed out waiting for {path.name}'
        time.sleep(.01)


def _start(tmp_path, mode):
    command = lambda role: [sys.executable, '-c', CHILD, str(tmp_path), role]
    log = (tmp_path / 'supervisor.log').open('w')
    process = subprocess.Popen(
        [sys.executable, '-m', 'ridgeback_autonomy.benchmarking.single_run',
         '--environment-command', json.dumps(command('environment')),
         '--configuration-command', json.dumps(command(mode))],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    log.close()
    return process


def _finish(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        raise


@pytest.mark.parametrize('mode', ['complete', 'inspect'])
def test_configuration_cleanup_keeps_environment_responsive(tmp_path, mode):
    process = _start(tmp_path, mode)
    try:
        _wait(tmp_path / 'runner.done', process)
        if mode == 'inspect':
            # Runner completion alone keeps both layers available for inspection.
            time.sleep(.1)
            assert process.poll() is None
            assert not (tmp_path / 'cleanup.started').exists()
            process.send_signal(signal.SIGINT)
        assert process.wait(timeout=10) == 0, (tmp_path / 'supervisor.log').read_text()
        assert (tmp_path / 'cleanup.done').exists()
        assert (tmp_path / 'pong').exists()
        assert (tmp_path / 'environment.stopped').exists()
    finally:
        _finish(process)


def test_repeated_interrupts_do_not_cut_short_ordered_cleanup(tmp_path):
    process = _start(tmp_path, 'inspect')
    try:
        _wait(tmp_path / 'runner.done', process)
        process.send_signal(signal.SIGINT)
        _wait(tmp_path / 'cleanup.started', process)
        process.send_signal(signal.SIGINT)
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=10) == 0, (tmp_path / 'supervisor.log').read_text()
        assert (tmp_path / 'cleanup.done').exists()
        assert (tmp_path / 'environment.stopped').exists()
    finally:
        _finish(process)


def test_unexpected_environment_exit_stops_configuration_and_fails(tmp_path):
    process = _start(tmp_path, 'inspect')
    try:
        _wait(tmp_path / 'runner.done', process)
        (tmp_path / 'environment.fail').touch()
        # The child's bounded cleanup cannot get a reply from the dead provider.
        assert process.wait(timeout=10) == 1
        assert (tmp_path / 'cleanup.started').exists()
    finally:
        _finish(process)


def test_nonzero_configuration_exit_fails_and_stops_environment(tmp_path):
    process = _start(tmp_path, 'failure')
    try:
        assert process.wait(timeout=10) == 1
        assert (tmp_path / 'environment.stopped').exists()
    finally:
        _finish(process)


def test_library_restores_signal_handlers_and_stops_environment_after_cleanup_error(monkeypatch):
    before = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    children = []
    class Process:
        def __init__(self, command, **kwargs):
            self.command = command
            children.append(self)
            self.pid = 1000 + len(children)
        def poll(self):
            return None if self is children[0] else 0
    stopped = []
    def stop(process, label):
        stopped.append(process.command)
        if process.command == ['configuration']:
            raise RuntimeError('owned child remained after escalation')
        return {'returncode': 0, 'escalated': False, 'group_empty': True}
    monkeypatch.setattr(single_run.subprocess, 'Popen', Process)
    monkeypatch.setattr(single_run, 'stop_process_group', stop)
    assert single_run.run_commands(['environment'], ['configuration']) == 1
    assert stopped == [['configuration'], ['environment']]
    assert {sig: signal.getsignal(sig) for sig in before} == before


@pytest.mark.parametrize('returncode,escalated', [(0, True), (-6, False), (130, False)])
def test_ordinary_completion_rejects_escalation_crashes_and_cancel_status(
    monkeypatch, returncode, escalated,
):
    children = []
    class Process:
        def __init__(self, command, **kwargs):
            children.append(self)
            self.pid = 1000 + len(children)
        def poll(self):
            return None if self is children[0] else 0
    def stop(process, label):
        return {'returncode': 0 if process is children[0] else returncode,
                'escalated': False if process is children[0] else escalated,
                'group_empty': True}
    monkeypatch.setattr(single_run.subprocess, 'Popen', Process)
    monkeypatch.setattr(single_run, 'stop_process_group', stop)
    assert single_run.run_commands(['environment'], ['configuration']) == 1


@pytest.mark.parametrize('configuration_status,caller_status', [(0, 0), (7, 1)])
def test_public_ros2_launch_propagates_supervisor_status(tmp_path, configuration_status, caller_status):
    wrapper = Path(__file__).resolve().parents[1] / 'launch/target_distance_benchmark.launch.py'
    fixture = tmp_path / 'owned_benchmark.launch.py'
    fixture.write_text(f'''
import importlib.util
import sys
spec = importlib.util.spec_from_file_location('public_benchmark_wrapper', {str(wrapper)!r})
wrapper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wrapper)
wrapper.layer_arguments = lambda: {{wrapper.CONFIG_LAUNCH: [], wrapper.ENVIRONMENT_LAUNCH: []}}
def command(filename, arguments):
    if filename == wrapper.CONFIG_LAUNCH:
        return [sys.executable, '-c', 'raise SystemExit({configuration_status})']
    return [sys.executable, '-c', 'import time; time.sleep(30)']
wrapper._launch_command = command
generate_launch_description = wrapper.generate_launch_description
''')
    result = subprocess.run(['ros2', 'launch', str(fixture)],
                            env=dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros_log')),
                            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=15)
    assert result.returncode == caller_status, result.stdout
    if caller_status:
        assert 'Benchmark supervisor exited with code 1' in result.stdout
