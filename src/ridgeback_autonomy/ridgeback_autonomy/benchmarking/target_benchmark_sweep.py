#!/usr/bin/env python3
"""Run many benchmark configurations against one persistent simulation."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory

from ridgeback_autonomy.perception.target_localization.launch import (
    SHARED_BENCHMARK_ARGUMENT_DEFAULTS,
    workspace_root_from_package_share,
)
from ridgeback_autonomy.benchmarking.process_utils import (
    extract_json_payload,
    git_provenance,
    run_command,
    try_command,
)
from ridgeback_autonomy.benchmarking.report import markdown_table
from ridgeback_autonomy.benchmarking.paths import (
    default_output_directory,
    subprocess_log_environment,
)
from ridgeback_autonomy.benchmarking.scenarios import load_scenarios
from ridgeback_autonomy.benchmarking.sweep import SweepConfig, SweepSpec, load_sweep
from ridgeback_autonomy.benchmarking.sweep_report import load_json, write_sweep_report


TRIAL_WALL_TIME_SEC = 15.4
COMMAND_TIMEOUT_SEC = 10.0
DELETE_TIMEOUT_SEC = 15.0
ENV_GAZEBO_WAIT_TIMEOUT_SEC = 300.0
RTF_SAMPLE_WINDOW_SEC = 2.0
PROCESS_POLL_SEC = 1.0
PROCESS_INT_GRACE_SEC = 20.0
FFMPEG_INT_GRACE_SEC = 10.0
PROCESS_TERM_GRACE_SEC = 5.0
MANIFEST_VERSION = 1


def _log(message: str) -> None:
    print(f'[benchmark-sweep] {message}', flush=True)


def _timestamp() -> str:
    return time.strftime('%Y%m%d_%H%M%S')


def _wall_timestamp() -> str:
    return time.strftime('%Y-%m-%d %H:%M:%S %z')


def _write_manifest(sweep_dir: str, manifest: dict) -> None:
    path = os.path.join(sweep_dir, 'sweep.json')
    temporary = f'{path}.tmp'
    with open(temporary, 'w', encoding='utf-8') as handle:
        json.dump(manifest, handle, indent=2, sort_keys=False)
        handle.write('\n')
    os.replace(temporary, path)


def _file_sha256(path: str | None) -> str | None:
    """Hash one existing input file without making in-memory specs invalid."""

    if not path or not os.path.isfile(path):
        return None
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _selected_configs(spec: SweepSpec, only: str | None) -> tuple[SweepConfig, ...]:
    if not only:
        return spec.configs
    requested = [token.strip() for token in only.split(',') if token.strip()]
    if not requested:
        raise ValueError('--only must name at least one configuration.')
    by_name = {config.name: config for config in spec.configs}
    unknown = [name for name in requested if name not in by_name]
    if unknown:
        raise ValueError(
            f'--only names unknown config "{unknown[0]}"; available: '
            f'{", ".join(by_name)}'
        )
    if len(set(requested)) != len(requested):
        raise ValueError('--only contains a duplicate configuration name.')
    return tuple(by_name[name] for name in requested)


def _scenario_path(config: SweepConfig, default_scenario_path: str) -> str:
    return config.arguments.get('scenario', '').strip() or default_scenario_path


def estimate_trials(config: SweepConfig, default_scenario_path: str) -> int:
    repeats = int(config.arguments.get('repeats', '5'))
    return sum(
        scene.repeats_override if scene.repeats_override is not None else repeats
        for scene in load_scenarios(_scenario_path(config, default_scenario_path))
    )


def _format_hours(seconds: float) -> str:
    return f'{seconds / 3600.0:.2f} h'


def print_dry_run(configs: tuple[SweepConfig, ...], default_scenario_path: str) -> None:
    rows = []
    total_seconds = 0.0
    for config in configs:
        trials = estimate_trials(config, default_scenario_path)
        seconds = trials * TRIAL_WALL_TIME_SEC
        total_seconds += seconds
        rows.append([
            config.name,
            config.arguments['estimators'],
            config.arguments.get('repeats', '5'),
            str(trials),
            _format_hours(seconds),
        ])
    print(markdown_table(
        ['Config', 'Estimators', 'Repeats', 'Trials', 'Estimated wall'],
        rows,
    ))
    print(f'\n{len(configs)} configs resolved; estimated sweep wall time: '
          f'{_format_hours(total_seconds)} ({TRIAL_WALL_TIME_SEC:.1f} s/trial).')


def _shared_arguments(spec: SweepSpec) -> dict[str, str]:
    return {
        name: spec.defaults.get(name, default)
        for name, default in SHARED_BENCHMARK_ARGUMENT_DEFAULTS.items()
    }


def _resolved_arguments(
    spec: SweepSpec,
    config: SweepConfig,
    sweep_dir: str,
) -> dict[str, str]:
    arguments = dict(config.arguments)
    arguments.update(_shared_arguments(spec))
    arguments['output_dir'] = sweep_dir
    arguments['run_dir_name'] = config.name
    arguments['shutdown_on_complete'] = 'true'
    return arguments


def _launch_argument_tokens(arguments: dict[str, str]) -> list[str]:
    return [
        f'{key}:={value}'
        for key, value in arguments.items()
        if value != ''
    ]


def _environment_command(spec: SweepSpec) -> list[str]:
    arguments = _shared_arguments(spec)
    if 'setup_path' in spec.defaults:
        arguments['setup_path'] = spec.defaults['setup_path']
    return [
        'ros2', 'launch', 'ridgeback_autonomy', 'target_benchmark_env.launch.py',
        *_launch_argument_tokens(arguments),
    ]


def _config_command(arguments: dict[str, str]) -> list[str]:
    return [
        'ros2', 'launch', 'ridgeback_autonomy', 'target_benchmark_config.launch.py',
        *_launch_argument_tokens(arguments),
    ]


def _valid_run_document(path: str) -> dict | None:
    if not os.path.isfile(path):
        return None
    try:
        document = load_json(path)
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(document.get('run'), dict):
        return None
    if not isinstance(document.get('parameters'), dict):
        return None
    if not isinstance(document.get('estimators'), list):
        return None
    return document


def _resume_signature(spec: SweepSpec, configs: tuple[SweepConfig, ...]) -> dict:
    """Stable sweep inputs, excluding supervisor-owned output/lifecycle args."""

    ignored = {'output_dir', 'run_dir_name', 'shutdown_on_complete'}
    environment = _shared_arguments(spec)
    if 'setup_path' in spec.defaults:
        environment['setup_path'] = spec.defaults['setup_path']
    return {
        'sweep_source_sha256': _file_sha256(spec.source),
        'environment': environment,
        'configs': {
            config.name: {
                **{
                    key: value for key, value in config.arguments.items()
                    if key not in ignored
                },
                'scenario_sha256': _file_sha256(
                    config.arguments.get('scenario')),
            }
            for config in configs
        },
    }


def _find_resumable_sweep(
    output_root: str,
    spec: SweepSpec,
    configs: tuple[SweepConfig, ...],
) -> str | None:
    if not os.path.isdir(output_root):
        return None
    expected_names = [config.name for config in configs]
    suffix = f'_{spec.name}'
    candidates = []
    for entry in os.scandir(output_root):
        if not entry.is_dir() or not entry.name.endswith(suffix):
            continue
        manifest_path = os.path.join(entry.path, 'sweep.json')
        try:
            manifest = load_json(manifest_path)
        except (OSError, ValueError, json.JSONDecodeError):
            continue
        sweep = manifest.get('sweep') or {}
        if os.path.abspath(str(sweep.get('source', ''))) != os.path.abspath(spec.source):
            continue
        if manifest.get('selected_configs') != expected_names:
            continue
        if manifest.get('resume_signature') != _resume_signature(spec, configs):
            continue
        if sweep.get('status') == 'complete':
            continue
        candidates.append(entry.path)
    return max(candidates) if candidates else None


def _new_manifest(
    spec: SweepSpec,
    configs: tuple[SweepConfig, ...],
    sweep_dir: str,
    workspace_root: str,
) -> dict:
    return {
        'version': MANIFEST_VERSION,
        'sweep': {
            'name': spec.name,
            'description': spec.description,
            'source': os.path.abspath(spec.source),
            'source_sha256': _file_sha256(spec.source),
            'status': 'running',
            'started': _wall_timestamp(),
            'finished': None,
        },
        'provenance': git_provenance(workspace_root),
        'defaults': dict(spec.defaults),
        'selected_configs': [config.name for config in configs],
        'resume_signature': _resume_signature(spec, configs),
        'configs': [
            {
                'name': config.name,
                'arguments': _resolved_arguments(spec, config, sweep_dir),
                'scenario_sha256': _file_sha256(
                    config.arguments.get('scenario')),
                'status': 'pending',
                'wall_time_sec': None,
                'output_path': config.name,
                'real_time_factor': None,
                'error': None,
            }
            for config in configs
        ],
    }


def _resume_manifest(
    manifest: dict,
    spec: SweepSpec,
    configs: tuple[SweepConfig, ...],
    sweep_dir: str,
) -> dict:
    existing = {entry.get('name'): entry for entry in manifest.get('configs', [])}
    entries = []
    for config in configs:
        entry = existing.get(config.name) or {}
        entry.update({
            'name': config.name,
            'arguments': _resolved_arguments(spec, config, sweep_dir),
            'output_path': config.name,
        })
        entry.setdefault('status', 'pending')
        entry.setdefault('wall_time_sec', None)
        entry.setdefault('real_time_factor', None)
        entry.setdefault('error', None)
        entries.append(entry)
    manifest['configs'] = entries
    manifest['defaults'] = dict(spec.defaults)
    manifest['selected_configs'] = [config.name for config in configs]
    manifest['resume_signature'] = _resume_signature(spec, configs)
    manifest.setdefault('sweep', {})['status'] = 'running'
    manifest['sweep']['finished'] = None
    manifest['resume_count'] = int(manifest.get('resume_count', 0)) + 1
    return manifest


def _pose_snapshot(world: str) -> dict[str, dict[str, Any]]:
    topic = f'/world/{world}/pose/info'
    result = run_command(
        ['gz', 'topic', '-e', '-t', topic, '--json-output', '-n', '1'],
        timeout_sec=COMMAND_TIMEOUT_SEC,
        description=f'read {topic}',
    )
    payload = extract_json_payload(result.stdout)
    poses = payload.get('pose') or []
    if isinstance(poses, dict):
        poses = [poses]
    return {
        str(pose['name']): pose
        for pose in poses
        if isinstance(pose, dict) and pose.get('name')
    }


def _pose_snapshot_with_retry(
    world: str,
    timeout_sec: float = ENV_GAZEBO_WAIT_TIMEOUT_SEC,
) -> dict[str, dict[str, Any]]:
    """Wait for the Gazebo pose stream needed by the pre-config orphan check."""

    deadline = time.monotonic() + timeout_sec
    last_error = None
    while time.monotonic() < deadline:
        try:
            return _pose_snapshot(world)
        except RuntimeError as exc:
            last_error = exc
            time.sleep(PROCESS_POLL_SEC)
    raise RuntimeError(
        f'Gazebo pose stream was unavailable for {timeout_sec:.0f}s: {last_error}'
    )


def _delete_entity(world: str, name: str) -> None:
    deadline = time.monotonic() + DELETE_TIMEOUT_SEC
    command = [
        'gz', 'service',
        '-s', f'/world/{world}/remove/blocking',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000',
        '--req', f'name: "{name}" type: MODEL',
    ]
    last_error = None
    while time.monotonic() < deadline:
        try:
            result = try_command(command, timeout_sec=COMMAND_TIMEOUT_SEC)
            if result.returncode == 0:
                return
            last_error = f'exit {result.returncode}: {result.stderr.strip()}'
        except RuntimeError as exc:
            last_error = str(exc)
        time.sleep(0.5)
    raise RuntimeError(f'failed to remove orphan "{name}": {last_error}')


def reap_orphan_benchmark_entities(world: str) -> list[str]:
    snapshot = _pose_snapshot_with_retry(world)
    orphans = sorted(name for name in snapshot if name.startswith('bench_'))
    if not orphans:
        return []
    _log(f'WARNING: found orphan benchmark entities from an unclean prior config: '
         f'{", ".join(orphans)}')
    for name in orphans:
        _delete_entity(world, name)
    remaining = [name for name in orphans if name in _pose_snapshot(world)]
    if remaining:
        raise RuntimeError(f'orphan entities remain after removal: {", ".join(remaining)}')
    return orphans


def _clock_seconds() -> float:
    result = run_command(
        ['ros2', 'topic', 'echo', '/clock', '--once'],
        timeout_sec=COMMAND_TIMEOUT_SEC,
        description='sample /clock',
    )
    sec_match = re.search(r'\bsec:\s*(-?\d+)', result.stdout)
    nanosec_match = re.search(r'\bnanosec:\s*(\d+)', result.stdout)
    if sec_match is None or nanosec_match is None:
        raise RuntimeError(f'Could not parse /clock sample: {result.stdout.strip()}')
    return float(sec_match.group(1)) + float(nanosec_match.group(1)) / 1_000_000_000.0


def sample_real_time_factor() -> float | None:
    try:
        sim_start = _clock_seconds()
        wall_start = time.monotonic()
        time.sleep(RTF_SAMPLE_WINDOW_SEC)
        sim_end = _clock_seconds()
        wall_elapsed = time.monotonic() - wall_start
        if wall_elapsed <= 0.0:
            return None
        return max(0.0, (sim_end - sim_start) / wall_elapsed)
    except RuntimeError as exc:
        _log(f'WARNING: real-time-factor sample unavailable: {exc}')
        return None


def _process_group_ffmpeg_pids(process_group: int) -> list[int]:
    try:
        result = try_command(
            ['pgrep', '-g', str(process_group), '-f', 'ffmpeg.*x11grab'],
            timeout_sec=5.0,
        )
    except RuntimeError:
        return []
    if result.returncode not in {0, 1}:
        return []
    return [int(token) for token in result.stdout.split() if token.isdigit()]


def _signal_pids(pids: list[int], sig: signal.Signals) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass


def _wait_until(predicate, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.25)
    return predicate()


def stop_process_group(process: subprocess.Popen | None, label: str) -> None:
    """Gracefully stop a launch group, finalizing ffmpeg before escalation."""

    if process is None:
        return
    process_group = process.pid
    if process.poll() is None:
        _log(f'Stopping {label}...')
        try:
            os.killpg(process_group, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=PROCESS_INT_GRACE_SEC)
        except subprocess.TimeoutExpired:
            pass

    ffmpeg_pids = _process_group_ffmpeg_pids(process_group)
    if ffmpeg_pids:
        _log(f'Waiting for {len(ffmpeg_pids)} screen recorder(s) to finalize...')
        _signal_pids(ffmpeg_pids, signal.SIGINT)
        _wait_until(
            lambda: not _process_group_ffmpeg_pids(process_group),
            FFMPEG_INT_GRACE_SEC,
        )
    ffmpeg_pids = _process_group_ffmpeg_pids(process_group)
    if ffmpeg_pids:
        _signal_pids(ffmpeg_pids, signal.SIGTERM)
        _wait_until(
            lambda: not _process_group_ffmpeg_pids(process_group),
            PROCESS_TERM_GRACE_SEC,
        )
    ffmpeg_pids = _process_group_ffmpeg_pids(process_group)
    if ffmpeg_pids:
        _log('WARNING: screen recorder ignored graceful shutdown; forcing recorder exit.')
        _signal_pids(ffmpeg_pids, signal.SIGKILL)
        _wait_until(
            lambda: not _process_group_ffmpeg_pids(process_group),
            PROCESS_TERM_GRACE_SEC,
        )

    # Only escalate the whole process group after no x11grab recorder remains.
    if process.poll() is None:
        try:
            os.killpg(process_group, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=PROCESS_TERM_GRACE_SEC)
        except subprocess.TimeoutExpired:
            if _process_group_ffmpeg_pids(process_group):
                raise RuntimeError(f'Refusing to SIGKILL {label} while ffmpeg is active.')
            try:
                os.killpg(process_group, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=PROCESS_TERM_GRACE_SEC)


def _archive_incomplete_output(output_path: str) -> str | None:
    if not os.path.exists(output_path):
        return None
    run_json = os.path.join(output_path, 'run.json')
    if _valid_run_document(run_json) is not None:
        return None
    archived = f'{output_path}.incomplete_{_timestamp()}'
    counter = 1
    while os.path.exists(archived):
        archived = f'{output_path}.incomplete_{_timestamp()}_{counter}'
        counter += 1
    os.rename(output_path, archived)
    _log(f'Preserved incomplete output as {archived}')
    return archived


def _wait_for_config(
    process: subprocess.Popen,
    environment: subprocess.Popen,
    timeout_sec: float,
) -> tuple[str, str | None]:
    deadline = time.monotonic() + timeout_sec
    while process.poll() is None:
        if environment.poll() is not None:
            return 'failed', f'persistent environment exited with code {environment.returncode}'
        if time.monotonic() >= deadline:
            return 'timeout', f'configuration exceeded {_format_hours(timeout_sec)} timeout'
        time.sleep(PROCESS_POLL_SEC)
    return 'exited', None


def _entry_by_name(manifest: dict, name: str) -> dict:
    return next(entry for entry in manifest['configs'] if entry['name'] == name)


def run_sweep(
    spec: SweepSpec,
    configs: tuple[SweepConfig, ...],
    *,
    package_share: str,
) -> tuple[str, bool]:
    workspace_root = workspace_root_from_package_share(package_share)
    output_root = spec.defaults.get(
        'output_dir', default_output_directory(workspace_root))
    output_root = os.path.abspath(os.path.expanduser(output_root))
    os.makedirs(output_root, exist_ok=True)

    sweep_dir = _find_resumable_sweep(output_root, spec, configs)
    if sweep_dir is None:
        sweep_dir = os.path.join(output_root, f'{_timestamp()}_{spec.name}')
        os.makedirs(sweep_dir, exist_ok=False)
        manifest = _new_manifest(spec, configs, sweep_dir, workspace_root)
        _log(f'Created sweep directory {sweep_dir}')
    else:
        manifest = _resume_manifest(
            load_json(os.path.join(sweep_dir, 'sweep.json')),
            spec,
            configs,
            sweep_dir,
        )
        _log(f'Resuming incomplete sweep {sweep_dir}')
    _write_manifest(sweep_dir, manifest)
    write_sweep_report(sweep_dir, manifest)

    environment: subprocess.Popen | None = None
    current_config_process: subprocess.Popen | None = None
    interrupted = False
    try:
        environment = subprocess.Popen(
            _environment_command(spec),
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            env=subprocess_log_environment(os.path.join(sweep_dir, 'environment')),
        )
        time.sleep(1.0)
        if environment.poll() is not None:
            raise RuntimeError(
                f'persistent environment exited immediately with code {environment.returncode}'
            )

        shared = _shared_arguments(spec)
        world = shared['world']
        default_scenario_path = os.path.join(
            package_share, 'config', 'benchmark_scenarios_full.yaml')

        for config in configs:
            entry = _entry_by_name(manifest, config.name)
            output_path = os.path.join(sweep_dir, config.name)
            existing_run = _valid_run_document(os.path.join(output_path, 'run.json'))
            if existing_run is not None:
                entry.update(status='skipped', error=None)
                _log(f'{config.name}: existing run.json is valid; skipping.')
                _write_manifest(sweep_dir, manifest)
                write_sweep_report(sweep_dir, manifest)
                continue

            if environment.poll() is not None:
                entry.update(
                    status='failed',
                    error=f'persistent environment exited with code {environment.returncode}',
                )
                _write_manifest(sweep_dir, manifest)
                break

            _archive_incomplete_output(output_path)
            try:
                reaped = reap_orphan_benchmark_entities(world)
                entry['orphan_entities_reaped'] = reaped
            except RuntimeError as exc:
                entry.update(status='failed', error=str(exc))
                _log(f'{config.name}: cannot establish a clean scene: {exc}')
                _write_manifest(sweep_dir, manifest)
                write_sweep_report(sweep_dir, manifest)
                continue

            rtf = sample_real_time_factor()
            entry['real_time_factor'] = rtf
            if rtf is not None:
                _log(f'{config.name}: pre-run real-time factor {rtf:.3f}')

            arguments = _resolved_arguments(spec, config, sweep_dir)
            entry['arguments'] = arguments
            entry.update(status='running', error=None, started=_wall_timestamp())
            _write_manifest(sweep_dir, manifest)
            _log(f'{config.name}: starting {arguments["estimators"]}')

            started = time.monotonic()
            current_config_process = subprocess.Popen(
                _config_command(arguments),
                stdin=subprocess.DEVNULL,
                start_new_session=True,
                env=subprocess_log_environment(output_path),
            )
            trials = estimate_trials(config, default_scenario_path)
            timeout_sec = max(30.0 * 60.0, trials * TRIAL_WALL_TIME_SEC * 2.0 + 600.0)
            wait_status, wait_error = _wait_for_config(
                current_config_process,
                environment,
                timeout_sec,
            )
            if wait_status in {'timeout', 'failed'}:
                stop_process_group(current_config_process, f'config {config.name}')
            wall_time = time.monotonic() - started
            entry['wall_time_sec'] = wall_time
            entry['finished'] = _wall_timestamp()

            run_document = _valid_run_document(os.path.join(output_path, 'run.json'))
            if run_document is not None:
                entry.update(status='success', error=None)
                _log(f'{config.name}: completed in {_format_hours(wall_time)}')
            else:
                error = wait_error or (
                    f'launch exited with code {current_config_process.returncode}; '
                    'run.json is missing or invalid'
                )
                entry.update(
                    status='timeout' if wait_status == 'timeout' else 'failed',
                    error=error,
                )
                _log(f'{config.name}: {entry["status"]}: {error}')
            current_config_process = None
            _write_manifest(sweep_dir, manifest)
            write_sweep_report(sweep_dir, manifest)

    except KeyboardInterrupt:
        interrupted = True
        manifest['sweep']['status'] = 'interrupted'
        _log('Interrupted; preserving completed configuration outputs for resume.')
        stop_process_group(current_config_process, 'active benchmark config')
    except Exception as exc:  # keep the manifest useful for report-only/resume
        manifest['sweep']['status'] = 'failed'
        manifest['sweep']['error'] = str(exc)
        _log(f'ERROR: {exc}')
    finally:
        stop_process_group(current_config_process, 'active benchmark config')
        stop_process_group(environment, 'persistent benchmark environment')

    statuses = [entry.get('status') for entry in manifest['configs']]
    success = all(status in {'success', 'skipped'} for status in statuses)
    if interrupted:
        manifest['sweep']['status'] = 'interrupted'
    elif success:
        manifest['sweep']['status'] = 'complete'
    elif manifest['sweep'].get('status') == 'running':
        manifest['sweep']['status'] = 'complete_with_failures'
    manifest['sweep']['finished'] = _wall_timestamp()
    _write_manifest(sweep_dir, manifest)
    write_sweep_report(sweep_dir, manifest)
    return sweep_dir, success


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Run benchmark configurations against one Gazebo/RViz environment.')
    parser.add_argument('sweep_yaml', nargs='?', help='Sweep YAML to validate and run')
    parser.add_argument('--only', help='Comma-separated configuration names to run')
    parser.add_argument('--dry-run', action='store_true', help='Validate and estimate without launching')
    parser.add_argument(
        '--report-only',
        metavar='SWEEP_DIR',
        help='Regenerate summary.md for an existing sweep and exit',
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.report_only:
        try:
            report_path = write_sweep_report(args.report_only)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            print(f'target_benchmark_sweep: {exc}', file=sys.stderr)
            return 2
        print(f'Regenerated {report_path}')
        return 0
    if not args.sweep_yaml:
        print('target_benchmark_sweep: sweep_yaml is required unless --report-only is used.',
              file=sys.stderr)
        return 2

    package_share = get_package_share_directory('ridgeback_autonomy')
    default_scenario_path = os.path.join(
        package_share, 'config', 'benchmark_scenarios_full.yaml')
    try:
        spec = load_sweep(
            args.sweep_yaml,
            default_scenario_path=default_scenario_path,
        )
        configs = _selected_configs(spec, args.only)
        if args.dry_run:
            print_dry_run(configs, default_scenario_path)
            return 0
        sweep_dir, success = run_sweep(spec, configs, package_share=package_share)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'target_benchmark_sweep: {exc}', file=sys.stderr)
        return 2

    print(f'Sweep outputs: {sweep_dir}')
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
