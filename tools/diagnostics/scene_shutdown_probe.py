#!/usr/bin/env python3
"""Compare owned scene rollback with simultaneous Gazebo shutdown.

Run from a sourced ROS workspace, outside a UDP-restricted sandbox. This
diagnostic starts only physics and scene services; it does not use a GPU.
Each trial uses an isolated ROS domain and Gazebo partition, with separate
owned process sessions. Evidence includes direct status, confirmed cleanup, logs, and loaded
library maps. A zero server status alone never proves successful cleanup.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback
import uuid


WORLD = '''<?xml version="1.0"?>
<sdf version="1.10"><world name="shutdown_probe">
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <model name="authored_floor"><static>true</static><link name="link">
    <collision name="floor"><geometry><box><size>20 20 0.1</size></box></geometry></collision>
  </link></model>
</world></sdf>
'''
ENTITY = '''<?xml version="1.0"?>
<sdf version="1.10"><model name="probe"><static>true</static><link name="link">
  <collision name="box"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision>
</link></model></sdf>
'''
SIGNATURES = {
    'broken_promise': r'broken promise',
    'future_error': r'(?:std::)?future_error',
    'native_abort': r'terminate called|Aborted|SIGABRT|core dumped',
    'invalid_ros_context': r'context is not valid|invalid context|rcl context.*invalid',
    'timeout': r'no response before the deadline|did not answer|timed out',
    'rollback_failure': r'rollback:',
}


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')
    temporary.replace(path)


def classify_log(contents: str) -> dict[str, list[str]]:
    return {
        name: [line for line in contents.splitlines() if re.search(pattern, line, re.I)]
        for name, pattern in SIGNATURES.items()
    }


def signal_owned(process: subprocess.Popen, signum: int) -> bool:
    """Signal only the session created by this probe, while its leader lives."""
    if process.poll() is not None:
        return False
    try:
        os.killpg(process.pid, signum)
        return True
    except ProcessLookupError:
        return False


def finish_owned(process: subprocess.Popen, timeout: float) -> dict:
    started = time.monotonic()
    escalated = []
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        for signum in (signal.SIGTERM, signal.SIGKILL):
            if signal_owned(process, signum):
                escalated.append(signal.Signals(signum).name)
            try:
                process.wait(timeout=2.0)
                break
            except subprocess.TimeoutExpired:
                continue
    return {'returncode': process.poll(), 'wait_elapsed_sec': time.monotonic() - started,
            'escalated': escalated}


def loaded_maps(pid: int, output: Path) -> list[dict]:
    contents = Path(f'/proc/{pid}/maps').read_text()
    output.write_text(contents)
    paths = sorted({line.split(maxsplit=5)[-1] for line in contents.splitlines()
                    if len(line.split(maxsplit=5)) == 6})
    selected = []
    for name in paths:
        if not re.search(r'ros_gz|libgz-(?:sim|transport)|UserCommands|simulation_interfaces', name):
            continue
        path = Path(name)
        selected.append({'path': name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()
                         if path.is_file() else None})
    return selected


def run_client(args: argparse.Namespace) -> int:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from simulation_interfaces.msg import Result
    from simulation_interfaces.srv import GetEntities
    from ridgeback_autonomy.benchmarking.scene_control import SpawnSpec, create_scene_control_client
    from ridgeback_common.ros_signals import init_owned, spin_step

    directory = args.child.resolve()
    state = {'ready': False, 'interrupted': False, 'cleanup_confirmed': False,
             'remaining_owned': None, 'query_error': None, 'exception': None}
    started = time.monotonic()
    init_owned(args=[])
    node = rclpy.create_node('scene_shutdown_probe_client')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    client = create_scene_control_client(
        node, args.namespace + '/gzserver', observe_clock=False,
        spin_once=lambda timeout: spin_step(node, executor=executor, timeout_sec=timeout),
        call_timeout_sec=2.0, ready_timeout_sec=5.0, delete_timeout_sec=3.0,
        retry_sleep_sec=0.05)
    raw = node.create_client(GetEntities, args.namespace + '/gzserver/get_entities')
    specs = [SpawnSpec(f'bench_shutdown_{i:03d}', (directory / 'entity.sdf').as_uri(),
                       'sdf', float(i % 10), float(i // 10), 0.5, 0.0)
             for i in range(args.entities)]
    try:
        client.wait_for_services(args.start_timeout)
        client.require_capabilities(['sdf'])
        state['services_ready_elapsed_sec'] = time.monotonic() - started
        with client.scene(specs) as names:
            state.update(ready=True, names=list(names),
                         scene_ready_elapsed_sec=time.monotonic() - started,
                         readiness_timeline=client.readiness_timeline())
            write_json(directory / 'client-state.json', state)
            # Parent has a bounded startup deadline and owns shutdown.
            deadline = time.monotonic() + args.start_timeout + args.stop_timeout
            while time.monotonic() < deadline:
                spin_step(node, executor=executor, timeout_sec=0.05)
            raise TimeoutError('parent did not interrupt ready scene before deadline')
    except KeyboardInterrupt as exc:
        state['interrupted'] = True
        state['rollback_notes'] = list(getattr(exc, '__notes__', []))
        for note in state['rollback_notes']:
            print(note, flush=True)
    except BaseException as exc:
        state['exception'] = f'{type(exc).__name__}: {exc}'
        traceback.print_exc()
    finally:
        # Independent unfiltered query: no prefix-filter semantics can hide
        # leftovers. This runs before ordered provider shutdown.
        try:
            future = raw.call_async(GetEntities.Request())
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                spin_step(node, executor=executor, timeout_sec=0.05)
            if not future.done():
                raise TimeoutError('unfiltered post-rollback query did not answer')
            response = future.result()
            if response.result.result != Result.RESULT_OK:
                raise RuntimeError(response.result.error_message)
            state['remaining_owned'] = [name for name in response.entities
                                        if name.startswith('bench_')]
            state['authored_floor_present'] = 'authored_floor' in response.entities
            state['cleanup_confirmed'] = not state['remaining_owned']
            state['cleanup_confirmed_monotonic'] = time.monotonic()
        except BaseException as exc:
            state['query_error'] = f'{type(exc).__name__}: {exc}'
        state['elapsed_sec'] = time.monotonic() - started
        write_json(directory / 'client-state.json', state)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if state['cleanup_confirmed'] and state['exception'] is None else 2


def run_trial(args: argparse.Namespace, executable: Path, order: str, repetition: int) -> dict:
    directory = args.output / f'{order}_{repetition:02d}'
    directory.mkdir(parents=True, exist_ok=False)
    (directory / 'world.sdf').write_text(WORLD)
    (directory / 'entity.sdf').write_text(ENTITY)
    token = uuid.uuid4().hex[:10]
    namespace = f'/shutdown_probe_{token}'
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_base + (repetition - 1) % 4),
               GZ_PARTITION=f'shutdown_probe_{token}', GZ_IP='127.0.0.1',
               ROS_LOG_DIR=str(directory / 'ros_log'))
    server_command = [str(executable), '--ros-args', '-r', f'__ns:={namespace}',
                      '-p', f'world_sdf_file:={directory / "world.sdf"}',
                      '-p', 'verbosity_level:=2']
    client_command = [sys.executable, str(Path(__file__).resolve()), '--child', str(directory),
                      '--namespace', namespace, '--entities', str(args.entities),
                      '--start-timeout', str(args.start_timeout),
                      '--stop-timeout', str(args.stop_timeout)]
    result = {'order': order, 'repetition': repetition, 'ros_domain_id': env['ROS_DOMAIN_ID'],
              'gz_partition': env['GZ_PARTITION'], 'namespace': namespace,
              'server_command': server_command, 'client_command': client_command,
              'ready': False, 'cleanup_confirmed_before_provider_stop': False,
              'startup_failure': None}
    if order == 'supervised':
        return run_supervised_trial(args, directory, env, server_command, client_command, result)
    processes = []
    started = time.monotonic()
    with (directory / 'gzserver.log').open('w') as server_log, (directory / 'client.log').open('w') as client_log:
        try:
            server = subprocess.Popen(server_command, env=env, stdout=server_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(server)
            client = subprocess.Popen(client_command, env=env, stdout=client_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(client)
            result.update(server_pid=server.pid, client_pid=client.pid)
            deadline = time.monotonic() + args.start_timeout
            state_path = directory / 'client-state.json'
            while time.monotonic() < deadline:
                if state_path.exists() and json.loads(state_path.read_text()).get('ready'):
                    result['ready'] = True
                    break
                if server.poll() is not None or client.poll() is not None:
                    break
                time.sleep(0.05)
            result['ready_elapsed_sec'] = time.monotonic() - started
            if not result['ready']:
                result['startup_failure'] = 'scene never reached ready state'
            else:
                result['loaded_libraries'] = loaded_maps(server.pid, directory / 'gzserver.maps')
                result['client_signal_monotonic'] = time.monotonic()
                signal_owned(client, signal.SIGINT)
                if order == 'simultaneous':
                    result['provider_signal_monotonic'] = time.monotonic()
                    signal_owned(server, signal.SIGINT)
                result['client'] = finish_owned(client, args.stop_timeout)
                if order == 'config_first':
                    state = json.loads(state_path.read_text())
                    result['provider_alive_after_client'] = server.poll() is None
                    result['cleanup_confirmed_before_provider_stop'] = bool(
                        state.get('cleanup_confirmed') and state.get('authored_floor_present')
                        and result['provider_alive_after_client'])
                    result['provider_signal_monotonic'] = time.monotonic()
                    signal_owned(server, signal.SIGINT)
                result['provider'] = finish_owned(server, args.stop_timeout)
                result['shutdown_elapsed_sec'] = time.monotonic() - result['client_signal_monotonic']
        except BaseException as exc:
            result['probe_exception'] = f'{type(exc).__name__}: {exc}'
            traceback.print_exc()
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    signal_owned(process, signal.SIGINT)
                final = finish_owned(process, args.stop_timeout)
                key = 'provider' if process is processes[0] else 'client'
                result.setdefault(key, final)
    for name in ('gzserver', 'client'):
        result[f'{name}_signatures'] = classify_log((directory / f'{name}.log').read_text())
    state_path = directory / 'client-state.json'
    result['client_state'] = json.loads(state_path.read_text()) if state_path.exists() else None
    result['elapsed_sec'] = time.monotonic() - started
    write_json(directory / 'result.json', result)
    print(json.dumps({'directory': str(directory), 'ready': result['ready'],
                      'provider': result.get('provider'), 'client': result.get('client'),
                      'cleanup_confirmed_before_provider_stop':
                      result['cleanup_confirmed_before_provider_stop'],
                      'gzserver_signatures': result['gzserver_signatures']}), flush=True)
    return result


def run_supervised_trial(args, directory, env, server_command, client_command, result):
    """Exercise the actual product owner, which alone signals both child groups."""
    from ridgeback_autonomy.benchmarking.target_benchmark_sweep import PROCESS_GROUP_STOP_BUDGET_SEC

    # exec preserves the direct child PID/status while keeping separate logs.
    logging_exec = ('import os,sys; f=open(sys.argv[1],"a"); '
                    'os.dup2(f.fileno(),1); os.dup2(f.fileno(),2); '
                    'os.execv(sys.argv[2],sys.argv[2:])')
    def logged(command, filename):
        return [sys.executable, '-c', logging_exec, str(directory / filename), *command]
    owner_report = directory / 'owner-report.json'
    owner_command = [sys.executable, '-m', 'ridgeback_autonomy.benchmarking.single_run',
                     '--environment-command', json.dumps(logged(server_command, 'gzserver.log')),
                     '--configuration-command', json.dumps(logged(client_command, 'client.log')),
                     '--report', str(owner_report)]
    result['owner_command'] = owner_command
    started = time.monotonic()
    with (directory / 'owner.log').open('w') as owner_log:
        owner = subprocess.Popen(owner_command, env=env, stdout=owner_log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        result['owner_pid'] = owner.pid
        try:
            state_path = directory / 'client-state.json'
            deadline = time.monotonic() + args.start_timeout
            while time.monotonic() < deadline:
                if state_path.exists() and json.loads(state_path.read_text()).get('ready'):
                    result['ready'] = True
                    break
                if owner.poll() is not None:
                    break
                time.sleep(0.05)
            result['ready_elapsed_sec'] = time.monotonic() - started
            if result['ready']:
                evidence = json.loads(owner_report.read_text())
                result.update(server_pid=evidence['environment_pid'],
                              client_pid=evidence['configuration_pid'])
                result['loaded_libraries'] = loaded_maps(
                    result['server_pid'], directory / 'gzserver.maps')
            else:
                result['startup_failure'] = 'scene never reached ready state'
        except BaseException as exc:
            result['probe_exception'] = f'{type(exc).__name__}: {exc}'
            traceback.print_exc()
        finally:
            result['owner_signal_monotonic'] = time.monotonic()
            signal_owned(owner, signal.SIGINT)
            result['owner'] = finish_owned(owner, 2 * PROCESS_GROUP_STOP_BUDGET_SEC + 1)
    evidence = json.loads(owner_report.read_text()) if owner_report.exists() else {}
    result['owner_lifecycle'] = evidence
    result['client_state'] = json.loads(state_path.read_text()) if state_path.exists() else None
    for name, label in (('provider', 'benchmark environment'), ('client', 'benchmark configuration')):
        result[name] = evidence.get('groups', {}).get(label, {}).get('report')
    provider_stop = evidence.get('groups', {}).get('benchmark environment', {}).get('stop_started_monotonic')
    state = result['client_state'] or {}
    result['cleanup_confirmed_before_provider_stop'] = bool(
        state.get('cleanup_confirmed') and state.get('authored_floor_present')
        and provider_stop is not None
        and state.get('cleanup_confirmed_monotonic', float('inf')) < provider_stop)
    for name in ('gzserver', 'client', 'owner'):
        log = directory / f'{name}.log'
        result[f'{name}_signatures'] = classify_log(log.read_text() if log.exists() else '')
    result['elapsed_sec'] = time.monotonic() - started
    write_json(directory / 'result.json', result)
    print(json.dumps({'directory': str(directory), 'ready': result['ready'],
                      'owner': result['owner'], 'provider': result['provider'], 'client': result['client'],
                      'cleanup_confirmed_before_provider_stop':
                      result['cleanup_confirmed_before_provider_stop']}), flush=True)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/benchmarks/scene_shutdown_probe'))
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--entities', type=int, default=16)
    parser.add_argument('--domain-base', type=int, default=183)
    parser.add_argument('--start-timeout', type=float, default=30.0)
    parser.add_argument('--stop-timeout', type=float, default=10.0)
    parser.add_argument('--supervised', action='store_true', help='Use the actual product single-run owner')
    parser.add_argument('--child', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--namespace', default='', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.entities < 1 or args.repetitions < 1 or min(args.start_timeout, args.stop_timeout) <= 0:
        parser.error('entities, repetitions, and timeouts must be positive')
    if args.child:
        return run_client(args)
    from ament_index_python.packages import get_package_prefix
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    executable = Path(get_package_prefix('ros_gz_sim')) / 'lib/ros_gz_sim/gzserver'
    root = Path(__file__).resolve().parents[2]
    provenance = {'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                  'server_executable': str(executable),
                  'server_sha256': hashlib.sha256(executable.read_bytes()).hexdigest(),
                  'python': sys.executable, 'arguments': vars(args) | {'output': str(args.output), 'child': None},
                  'packages': subprocess.run(['dpkg-query', '-W', '-f=${Package} ${Version}\n',
                                              'ros-jazzy-ros-gz-sim', 'ros-jazzy-gz-sim-vendor',
                                              'ros-jazzy-gz-transport-vendor',
                                              'ros-jazzy-simulation-interfaces'],
                                             capture_output=True, text=True).stdout,
                  'environment': {key: os.environ.get(key) for key in
                                  ('AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'RMW_IMPLEMENTATION')}}
    write_json(args.output / 'provenance.json', provenance)
    trials = [run_trial(args, executable, order, repetition)
              for repetition in range(1, args.repetitions + 1)
              for order in (('supervised',) if args.supervised else ('simultaneous', 'config_first'))]
    write_json(args.output / 'summary.json', {'trials': trials})
    # Failed simultaneous cleanup is a measured outcome, not probe failure.
    return int(any(
        not trial['ready'] or trial.get('probe_exception')
        or (args.supervised and (
            trial['owner']['returncode'] != 0
            or not trial['cleanup_confirmed_before_provider_stop']
            or any((trial.get(role) or {}).get('returncode') != 0
                   for role in ('provider', 'client'))))
        for trial in trials))


if __name__ == '__main__':
    raise SystemExit(main())
