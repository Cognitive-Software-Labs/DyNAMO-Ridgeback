"""A real Ctrl-C during a benchmark scene still removes what the scene spawned.

A provider process serves the five standard ``simulation_interfaces``
services from a dictionary; a client process enters a three-entity scene the
way the benchmark runner does and is signalled with ``os.kill``. When the
client has exited, the provider reports which entities are left.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from ridgeback_autonomy.benchmarking.scene_control import ROLLBACK_INTERRUPT_CALL_SEC


NAMESPACE = '/fake_sim'
ENTITIES = ['bench_sig_t0_target_0', 'bench_sig_t0_obj_0', 'bench_sig_t0_obj_1']

PROVIDER = r'''
import json
import re
import sys
import time

import rclpy
from simulation_interfaces.msg import EntityState, Result, SimulatorFeatures
from simulation_interfaces.srv import (
    DeleteEntity, GetEntities, GetEntitiesStates, GetSimulatorFeatures, SpawnEntity)

from ridgeback_common.ros_signals import init_owned, spin_until_interrupted

namespace = sys.argv[1]
delete_delay_sec = float(sys.argv[2])
hold = set(sys.argv[3:])
entities = {}


def say(*words):
    print(*words, flush=True)


def ok(response):
    response.result.result = Result.RESULT_OK
    return response


def features(_request, response):
    response.features.features = [
        SimulatorFeatures.SPAWNING, SimulatorFeatures.DELETING,
        SimulatorFeatures.ENTITY_STATE_GETTING]
    response.features.spawn_formats = ['sdf']
    return response


def spawn(request, response):
    entities[request.name] = request.initial_pose.pose
    say('SPAWN', request.name)
    if request.name in hold:
        say('HOLD', request.name)
        time.sleep(1.0)
    response.entity_name = request.name
    return ok(response)


def delete(request, response):
    say('DELETE', request.entity)
    time.sleep(delete_delay_sec)
    if entities.pop(request.entity, None) is None:
        response.result.result = Result.RESULT_OPERATION_FAILED
        return response
    return ok(response)


def list_entities(request, response):
    pattern = re.compile(request.filters.filter or '.*')
    response.entities = [name for name in entities if pattern.fullmatch(name)]
    return ok(response)


def states(_request, response):
    for name, pose in entities.items():
        state = EntityState()
        state.header.frame_id = 'world'
        state.header.stamp.sec = 7
        state.pose = pose
        response.entities.append(name)
        response.states.append(state)
    return ok(response)


init_owned()
node = rclpy.create_node('fake_scene_provider')
for suffix, kind, handler in [
        ('get_simulator_features', GetSimulatorFeatures, features),
        ('spawn_entity', SpawnEntity, spawn),
        ('delete_entity', DeleteEntity, delete),
        ('get_entities', GetEntities, list_entities),
        ('get_entities_states', GetEntitiesStates, states)]:
    node.create_service(kind, f'{namespace}/{suffix}', handler)
say('READY')
try:
    spin_until_interrupted(node)
except KeyboardInterrupt:
    pass
say('REMAINING', json.dumps(sorted(entities)))
node.destroy_node()
rclpy.try_shutdown()
'''

CLIENT = r'''
import sys

import rclpy

from ridgeback_autonomy.benchmarking.scene_control import SpawnSpec, create_scene_control_client
from ridgeback_common.ros_signals import init_owned, shielded, spin_step


def main():
    namespace, names = sys.argv[1], sys.argv[2:]
    init_owned()
    node = rclpy.create_node('fake_scene_client')
    try:
        scene_control = create_scene_control_client(node, namespace, observe_clock=False)
        scene_control.wait_for_services(10.0)
        specs = [
            SpawnSpec(name=name, uri='/assets/model.sdf', resource_format='sdf',
                      x=1.0 + index, y=0.0, z=0.0, yaw=0.0)
            for index, name in enumerate(names)]
        with scene_control.scene(specs):
            print('BODY', flush=True)
            while True:
                spin_step(node, timeout_sec=0.1)
    except KeyboardInterrupt as exc:
        for note in getattr(exc, '__notes__', []):
            print('NOTE', note, flush=True)
        return 130
    finally:
        with shielded(final=True):
            node.destroy_node()
            rclpy.try_shutdown()


raise SystemExit(main())
'''


class Process:
    def __init__(self, script: str, env: dict[str, str], *args: str) -> None:
        self.process = subprocess.Popen(
            [sys.executable, '-c', script, *args],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
        )
        self.lines: list[str] = []

    def expect(self, prefix: str, timeout_sec: float = 20.0) -> str:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            line = self.process.stdout.readline()
            if not line:
                break
            self.lines.append(line.rstrip('\n'))
            if line.startswith(prefix):
                return line.rstrip('\n')
        self.process.kill()
        pytest.fail(f'no {prefix!r} line; output:\n' + '\n'.join(self.lines))

    def signal(self, signum: int = signal.SIGINT) -> None:
        os.kill(self.process.pid, signum)

    def finish(self, timeout_sec: float = 20.0) -> tuple[int, str]:
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


@pytest.fixture
def ros_env(tmp_path, request) -> dict[str, str]:
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(190 + request.param)
    env['ROS_LOG_DIR'] = str(tmp_path / 'ros_logs')
    return env


def _remaining(provider: Process) -> list[str]:
    provider.signal()
    line = provider.expect('REMAINING')
    provider.finish()
    return json.loads(line.split(' ', 1)[1])


def _assert_clean_interrupt(client: Process, provider: Process) -> str:
    returncode, output = client.finish()
    assert returncode == 130, output
    assert 'Traceback' not in output, output
    assert _remaining(provider) == [], provider.lines
    return output


@pytest.mark.parametrize('ros_env', [0], indirect=True)
def test_an_interrupt_in_the_scene_body_removes_every_entity(ros_env) -> None:
    provider = Process(PROVIDER, ros_env, NAMESPACE, '0')
    provider.expect('READY')
    client = Process(CLIENT, ros_env, NAMESPACE, *ENTITIES)
    client.expect('BODY')
    client.signal()

    _assert_clean_interrupt(client, provider)
    assert [line.split()[1] for line in provider.lines if line.startswith('DELETE')] == (
        list(reversed(ENTITIES)))


@pytest.mark.parametrize('ros_env', [1], indirect=True)
def test_an_interrupt_while_awaiting_a_spawn_response_removes_that_entity_too(
        ros_env) -> None:
    provider = Process(PROVIDER, ros_env, NAMESPACE, '0', ENTITIES[-1])
    provider.expect('READY')
    client = Process(CLIENT, ros_env, NAMESPACE, *ENTITIES)
    provider.expect(f'HOLD {ENTITIES[-1]}')
    client.signal()

    _assert_clean_interrupt(client, provider)


@pytest.mark.parametrize('ros_env, gap_sec', [(2, 0.04), (4, 0.3)], indirect=['ros_env'])
def test_a_second_interrupt_does_not_abandon_the_rollback(ros_env, gap_sec) -> None:
    # Slow deletes keep the rollback running for about 0.6 s. A second signal
    # 40 ms after the first -- the double delivery of a group signal plus a
    # launch relay -- still lands in the executor step that holds the first,
    # and the two raise once; 300 ms later it lands inside the rollback.
    provider = Process(PROVIDER, ros_env, NAMESPACE, '0.2')
    provider.expect('READY')
    client = Process(CLIENT, ros_env, NAMESPACE, *ENTITIES)
    client.expect('BODY')
    client.signal()
    time.sleep(gap_sec)
    client.signal()

    output = _assert_clean_interrupt(client, provider)
    if gap_sec > 0.1:
        assert 'Received SIGINT during cleanup' in output


@pytest.mark.parametrize('ros_env', [3], indirect=True)
def test_an_interrupt_with_the_provider_gone_gives_up_after_one_call(ros_env) -> None:
    # Timed to the end of the rollback, not to process exit: destroying a
    # writer that still holds data for a vanished peer lingers for a time the
    # middleware sets (one second per writer under CycloneDDS).
    provider = Process(PROVIDER, ros_env, NAMESPACE, '0')
    provider.expect('READY')
    client = Process(CLIENT, ros_env, NAMESPACE, *ENTITIES)
    client.expect('BODY')
    provider.process.kill()
    started = time.monotonic()
    client.signal()
    client.expect('NOTE rollback:')
    rolled_back = time.monotonic() - started

    returncode, output = client.finish()
    provider.finish()
    assert returncode == 130, output
    assert 'Traceback' not in output, output
    assert rolled_back <= ROLLBACK_INTERRUPT_CALL_SEC + 0.5, output
    assert output.count('NOTE rollback:') == len(ENTITIES), output
