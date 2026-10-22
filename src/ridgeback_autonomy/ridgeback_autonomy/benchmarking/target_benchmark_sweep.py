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
import rclpy

from ridgeback_autonomy.localization_launch import (
    SHARED_BENCHMARK_ARGUMENT_DEFAULTS,
    workspace_root_from_package_share,
)
from ridgeback_autonomy.benchmarking.asset_catalog import (
    DEFAULT_TARGET,
    scene_object_models,
)
from ridgeback_autonomy.benchmarking.backends import (
    BenchmarkEnvironment,
    resolve_benchmark_environment,
    simulator_provenance,
)
from ridgeback_autonomy.benchmarking.process_utils import (
    git_provenance,
    gl_renderer_provenance,
    run_command,
    software_gl_warning,
    try_command,
)
from ridgeback_autonomy.benchmarking.scene_control import (
    SceneControlClient,
    create_scene_control_client,
)
from ridgeback_autonomy.benchmarking.report import markdown_table
from ridgeback_autonomy.benchmarking.paths import (
    default_output_directory,
    subprocess_log_environment,
)
from ridgeback_autonomy.benchmarking.scenarios import load_scenarios
from ridgeback_autonomy.benchmarking.sweep import (
    ENVIRONMENT_ONLY_DEFAULT_NAMES,
    TRIAL_WALL_TIME_SEC,
    SweepConfig,
    SweepSpec,
    estimate_trials,
    load_sweep,
)
from ridgeback_autonomy.benchmarking.sweep_report import load_json, write_sweep_report


COMMAND_TIMEOUT_SEC = 10.0
ENV_SCENE_CONTROL_WAIT_TIMEOUT_SEC = 300.0
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


def _format_hours(seconds: float) -> str:
    return f'{seconds / 3600.0:.2f} h'


def print_dry_run(
    configs: tuple[SweepConfig, ...],
    default_scenario_path: str,
    environment: BenchmarkEnvironment,
) -> None:
    print(
        f'Backend {environment.backend.name}: world "{environment.public_world}" '
        f'runs as "{environment.native_world}"; asset catalog '
        f'{environment.catalog.path} (sha256 {environment.catalog.sha256}); '
        f'settings {_format_settings(environment.settings)}.\n')
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
    # Reported on the dry run too, so the renderer can be corrected before
    # committing hours to a sweep whose sensor rates would be invalid.
    warning = software_gl_warning(gl_renderer_provenance())
    if warning is not None:
        print(f'\nWARNING: {warning}')


def _format_settings(settings: dict[str, str]) -> str:
    return ', '.join(f'{name}={value}' for name, value in settings.items()) or 'none'


def resolve_sweep_environment(
    spec: SweepSpec,
    configs: tuple[SweepConfig, ...],
    default_scenario_path: str,
    *,
    package_share=None,
) -> BenchmarkEnvironment:
    """The sweep's one environment, checked against every selected scenario.

    Resolved before anything launches -- including for a dry run -- so a
    world or asset the selected simulator's catalog lacks is reported up front
    instead of as a failed configuration hours in.
    """

    environment = resolve_benchmark_environment(
        spec.backend,
        _shared_arguments(spec)['world'],
        spec.backend_settings,
        package_share=package_share,
        owner=f'Sweep file {spec.source} field "defaults"',
    )
    for config in configs:
        scenario = config.arguments.get('scenario', '').strip() or default_scenario_path
        try:
            environment.catalog.select(
                environment.public_world,
                DEFAULT_TARGET,
                scene_object_models(load_scenarios(scenario)),
            )
        except ValueError as exc:
            raise ValueError(f'Sweep config "{config.name}": {exc}') from exc
    return environment


def _shared_arguments(spec: SweepSpec) -> dict[str, str]:
    return {
        name: spec.defaults.get(name, default)
        for name, default in SHARED_BENCHMARK_ARGUMENT_DEFAULTS.items()
    }


def _environment_arguments(spec: SweepSpec) -> dict[str, str]:
    """Everything the persistent layer is started with, backend settings explicit."""

    arguments = _shared_arguments(spec)
    # Sorted so the recorded command is byte-identical between runs of the
    # same sweep; a frozenset's iteration order is not stable across processes.
    for name in sorted(ENVIRONMENT_ONLY_DEFAULT_NAMES | {'target_labels'}):
        if name in spec.defaults:
            arguments[name] = spec.defaults[name]
    arguments.update(spec.backend_settings)
    return arguments


def _resolved_arguments(
    spec: SweepSpec,
    config: SweepConfig,
    sweep_dir: str,
) -> dict[str, str]:
    arguments = dict(config.arguments)
    arguments.update(_shared_arguments(spec))
    # Recorded by the config runner; the environment applied them.
    arguments.update(spec.backend_settings)
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


def _launch_command(launch_file: str, arguments: dict[str, str]) -> list[str]:
    """``ros2 launch ridgeback_autonomy <launch_file>``, in terminal mode.

    stop_process_group() signals the whole group, as a terminal's Ctrl-C does,
    so launch must not forward its own SIGINT to every child as well; plain
    ``ros2 launch`` would, because the supervisor's stdin is not a terminal.
    """

    return [
        sys.executable, '-m', 'ridgeback_autonomy.benchmarking.interactive_launch',
        'ridgeback_autonomy', launch_file, *_launch_argument_tokens(arguments),
    ]


def _environment_command(spec: SweepSpec) -> list[str]:
    return _launch_command('target_benchmark_env.launch.py', _environment_arguments(spec))


def run_preflight_cleanup(workspace_root: str) -> None:
    """Clear stale ROS/simulator processes once before starting a sweep."""

    cleanup_script = os.path.join(workspace_root, 'cleanup.sh')
    if not os.path.isfile(cleanup_script):
        raise RuntimeError(f'Benchmark cleanup script is missing: {cleanup_script}')
    _log('Running cleanup.sh before the persistent benchmark environment.')
    subprocess.run(
        ['bash', cleanup_script],
        cwd=workspace_root,
        stdin=subprocess.DEVNULL,
        check=True,
    )


def _config_command(arguments: dict[str, str]) -> list[str]:
    return _launch_command('target_benchmark_config.launch.py', arguments)


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


def _resume_signature(
    spec: SweepSpec,
    configs: tuple[SweepConfig, ...],
    environment: BenchmarkEnvironment,
) -> dict:
    """Stable sweep inputs, excluding supervisor-owned output/lifecycle args.

    The simulation part pins the resolved native world and the exact catalog
    bytes as well as the normalized backend settings, so a sweep never resumes
    under another simulator, world, depth mode, or asset catalog.
    """

    ignored = {'output_dir', 'run_dir_name', 'shutdown_on_complete'}
    return {
        'sweep_source_sha256': _file_sha256(spec.source),
        'environment': _environment_arguments(spec),
        'simulation': environment.signature(),
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
    environment: BenchmarkEnvironment,
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
        if manifest.get('resume_signature') != _resume_signature(spec, configs, environment):
            continue
        if sweep.get('status') == 'complete':
            continue
        candidates.append(entry.path)
    return max(candidates) if candidates else None


def _simulation_manifest(environment: BenchmarkEnvironment) -> dict:
    return {
        **environment.provenance(),
        **simulator_provenance(environment.backend),
    }


def _new_manifest(
    spec: SweepSpec,
    configs: tuple[SweepConfig, ...],
    sweep_dir: str,
    workspace_root: str,
    environment: BenchmarkEnvironment,
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
        # Records whether the simulator rasterized on the CPU. Without it a
        # run that produced a fraction of its configured sensor rate is
        # indistinguishable, after the fact, from a healthy one.
        'gl': gl_renderer_provenance(),
        'simulation': _simulation_manifest(environment),
        'environment_readiness': None,
        'defaults': dict(spec.defaults),
        'selected_configs': [config.name for config in configs],
        'resume_signature': _resume_signature(spec, configs, environment),
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
    environment: BenchmarkEnvironment,
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
    manifest['resume_signature'] = _resume_signature(spec, configs, environment)
    manifest['simulation'] = _simulation_manifest(environment)
    manifest['environment_readiness'] = None
    manifest.setdefault('sweep', {})['status'] = 'running'
    manifest['sweep']['finished'] = None
    manifest['resume_count'] = int(manifest.get('resume_count', 0)) + 1
    return manifest


def reap_orphan_benchmark_entities(scene_control: SceneControlClient) -> list[str]:
    orphans = list(scene_control.find_benchmark_entities())
    if not orphans:
        return []
    _log(f'WARNING: found orphan benchmark entities from an unclean prior config: '
         f'{", ".join(orphans)}')
    for name in orphans:
        scene_control.delete_entity(name)
    remaining = list(scene_control.find_benchmark_entities())
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
        # One SIGINT to every process, as a terminal's Ctrl-C delivers it. The
        # launch runs in terminal mode (_launch_command), so it forwards none
        # of its own; each child receives exactly this one.
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
    simulation: BenchmarkEnvironment,
    preflight_cleanup: bool = True,
) -> tuple[str, bool]:
    workspace_root = workspace_root_from_package_share(package_share)
    output_root = spec.defaults.get(
        'output_dir', default_output_directory(workspace_root))
    output_root = os.path.abspath(os.path.expanduser(output_root))
    os.makedirs(output_root, exist_ok=True)

    sweep_dir = _find_resumable_sweep(output_root, spec, configs, simulation)
    if sweep_dir is None:
        sweep_dir = os.path.join(output_root, f'{_timestamp()}_{spec.name}')
        os.makedirs(sweep_dir, exist_ok=False)
        manifest = _new_manifest(spec, configs, sweep_dir, workspace_root, simulation)
        _log(f'Created sweep directory {sweep_dir}')
    else:
        manifest = _resume_manifest(
            load_json(os.path.join(sweep_dir, 'sweep.json')),
            spec,
            configs,
            sweep_dir,
            simulation,
        )
        _log(f'Resuming incomplete sweep {sweep_dir}')
    _write_manifest(sweep_dir, manifest)
    write_sweep_report(sweep_dir, manifest)

    _log(
        f'Backend {simulation.backend.name}: world "{simulation.public_world}" '
        f'runs as "{simulation.native_world}" '
        f'(settings {_format_settings(simulation.settings)}).')

    # Last point before the simulator starts. A software rasterizer does not fail the
    # run, it quietly reduces every rendered sensor, so say so loudly here as
    # well as recording it in the manifest.
    gl_warning = software_gl_warning(manifest.get('gl') or {})
    if gl_warning is not None:
        _log(f'WARNING: {gl_warning}')

    environment: subprocess.Popen | None = None
    current_config_process: subprocess.Popen | None = None
    scene_control_node = None
    interrupted = False
    try:
        if preflight_cleanup:
            run_preflight_cleanup(workspace_root)
        else:
            _log(
                'Skipping cleanup.sh by operator request; existing ROS/simulator '
                'processes may collide with this sweep.')
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

        rclpy.init()
        scene_control_node = rclpy.create_node('target_benchmark_sweep_scene_control')
        scene_control = create_scene_control_client(
            scene_control_node, simulation.backend.scene_control_service_namespace)
        scene_control.wait_for_services(ENV_SCENE_CONTROL_WAIT_TIMEOUT_SEC)
        scene_control.require_capabilities()
        # Orphan reaping is the first scene operation; it waits until the
        # simulator is stepping, not merely answering.
        _log('Scene-control services are available; waiting for the simulation clock.')
        scene_control.wait_for_advancing_clock(ENV_SCENE_CONTROL_WAIT_TIMEOUT_SEC)
        manifest['environment_readiness'] = scene_control.readiness_timeline()
        _log(
            'Simulation clock advancing '
            f'{manifest["environment_readiness"]["clock_advance_after_services_sec"]:.3f} s '
            'after the scene-control services appeared.')
        _write_manifest(sweep_dir, manifest)
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
                reaped = reap_orphan_benchmark_entities(scene_control)
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
        if scene_control_node is not None:
            scene_control_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
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
        description=(
            'Run benchmark configurations against one persistent simulator/RViz '
            'environment.'))
    parser.add_argument('sweep_yaml', nargs='?', help='Sweep YAML to validate and run')
    parser.add_argument('--only', help='Comma-separated configuration names to run')
    parser.add_argument('--dry-run', action='store_true', help='Validate and estimate without launching')
    parser.add_argument(
        '--skip-preflight-cleanup',
        action='store_true',
        help=(
            'Do not run the aggressive same-user ROS/simulator cleanup before the '
            'sweep; use only when another session must remain alive'
        ),
    )
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
        environment = resolve_sweep_environment(spec, configs, default_scenario_path)
        if args.dry_run:
            print_dry_run(configs, default_scenario_path, environment)
            return 0
        sweep_dir, success = run_sweep(
            spec,
            configs,
            package_share=package_share,
            simulation=environment,
            preflight_cleanup=not args.skip_preflight_cleanup,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'target_benchmark_sweep: {exc}', file=sys.stderr)
        return 2

    print(f'Sweep outputs: {sweep_dir}')
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
