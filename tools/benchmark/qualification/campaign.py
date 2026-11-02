#!/usr/bin/env python3
"""Prepare and supervise this workstation's fixed 16-run qualification campaign."""
import argparse
import ctypes
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import psutil
import yaml

from evidence import CONDITIONS, ESTIMATORS, csv_rows, compare_rows, read_json, repeatability

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ORDER = ('gz_r1', 'isaac_r1', 'gz_r2', 'isaac_r2')
TIMING_SPENT_SEC = 428
TOTAL_SEC = 12 * 3600 - TIMING_SPENT_SEC
LIVE_SEC = 9 * 3600
CLEANUP_SEC = 180
ADMISSION_SEC = 35 * 60
STOP_REQUESTED = False


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')
    temporary.replace(path)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def frozen_files(bundle):
    files = set()
    for package in ('ridgeback_autonomy', 'ridgeback_autonomy_gz', 'ridgeback_autonomy_isaac',
                    'ridgeback_common', 'ridgeback_localization', 'ridgeback_interfaces'):
        for base in (ROOT / 'src' / package, ROOT / 'install' / package / 'lib' / package):
            files.update(p for p in base.rglob('*') if p.is_file()
                         and '__pycache__' not in p.parts and p.suffix != '.pyc')
    files.add(ROOT / 'clearpath/robot.yaml')
    for model in ('google/owlv2-base-patch16-ensemble', 'Zigeng/SlimSAM-uniform-50',
                  'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf'):
        cache = Path(os.environ.get('HF_HUB_CACHE', str(Path.home()/'.cache/huggingface/hub')))
        reference = cache / ('models--'+model.replace('/', '--')) / 'refs/main'
        if reference.exists():
            files.add(reference)
    files.update(p for p in HERE.rglob('*') if p.is_file()
                 and '__pycache__' not in p.parts and p.suffix != '.pyc')
    files.update((Path(bundle) / 'inputs').glob('*.yaml'))
    return {str(p): sha(p) for p in sorted(files)}


def check_frozen(bundle):
    before = read_json(Path(bundle) / 'frozen.json')['files']
    after = frozen_files(bundle)
    changed = sorted(k for k in set(before) | set(after) if before.get(k) != after.get(k))
    if changed:
        raise RuntimeError('Experimental files changed: ' + ', '.join(changed[:8]))


def check_installed():
    pairs = (
        ('ridgeback_autonomy', 'benchmarking/target_distance_benchmark_runner_node.py', 'target_distance_benchmark_runner'),
        ('ridgeback_autonomy', 'benchmarking/target_benchmark_sweep.py', 'target_benchmark_sweep'),
        ('ridgeback_localization', 'mask_measurement_node.py', 'target_mask_measurement_node'),
        ('ridgeback_localization', 'detector_node.py', 'target_detector_node'),
    )
    for package, source, executable in pairs:
        a = ROOT / 'src' / package / package / source
        b = ROOT / 'install' / package / 'lib' / package / executable
        if not b.exists() or sha(a) != sha(b):
            raise RuntimeError(f'Installed executable differs from source: {executable}; rebuild before launch')
    assets = (
        ('ridgeback_autonomy', 'config/benchmark_scenarios_full.yaml'),
        ('ridgeback_autonomy', 'config/benchmark_background_lab.yaml'),
        ('ridgeback_autonomy', 'config/benchmark_backgrounds/lab.yaml'),
        ('ridgeback_autonomy_gz', 'sim/benchmark_assets.yaml'),
        ('ridgeback_autonomy_isaac', 'sim/benchmark_assets.yaml'),
        ('ridgeback_autonomy_isaac', 'sim/isaac/isaac_runner.py'),
    )
    for package, relative in assets:
        source = ROOT/'src'/package/relative
        installed = ROOT/'install'/package/'share'/package/relative
        if not installed.exists() or sha(source) != sha(installed):
            raise RuntimeError(f'Installed asset/config differs from source: {package}/{relative}')


def prepare(bundle):
    bundle = Path(bundle).resolve()
    bundle.mkdir(parents=True, exist_ok=False)
    (bundle / 'inputs').mkdir()
    overrides = yaml.safe_load((HERE / 'capture_arguments.yaml').read_text()) or {}
    if not isinstance(overrides, dict):
        raise ValueError('capture_arguments.yaml must contain a launch-argument mapping')
    scenario = ROOT / 'src/ridgeback_autonomy/config/benchmark_scenarios_full.yaml'
    for name in ORDER:
        document = yaml.safe_load((HERE / 'sweeps' / f'{name}.yaml').read_text())
        document['defaults'].update(scenario=str(scenario), output_dir=str(bundle / name))
        # Both repetitions use the same recording workload; only r1 is replayed.
        for config in document['configs']:
            config.update(overrides)
            config['sensor_capture_dir'] = str(bundle / 'captures' / name / config['name'])
        (bundle / 'inputs' / f'{name}.yaml').write_text(yaml.safe_dump(document, sort_keys=False))
    # Four short capture checks precede all full runs. Their unequal durations
    # check that capture follows time, rather than the old fixed batch quota.
    for backend in ('gz', 'isaac'):
        document = yaml.safe_load((bundle / 'inputs' / f'{backend}_r1.yaml').read_text())
        document['sweep']['name'] = f'capture_check_{backend}'
        document['defaults'].update(scene_ids='multi_04', output_dir=str(bundle / f'check_{backend}'))
        document['configs'] = [dict(document['configs'][index]) for index in (0, 2)]
        for config, duration in zip(document['configs'], (4.0, 6.0)):
            config['capture_sec'] = duration
            config['sensor_capture_dir'] = str(bundle / 'check_captures' / backend / config['name'])
        (bundle / 'inputs' / f'check_{backend}.yaml').write_text(yaml.safe_dump(document, sort_keys=False))
    atomic_json(bundle / 'campaign.json', {
        'schema': 'ridgeback-qualification-campaign/1', 'status': 'prepared-not-launched',
        'order': list(ORDER), 'operations': [], 'validated': {}, 'errors': [],
        'budget_sec': TOTAL_SEC, 'live_budget_sec': LIVE_SEC, 'timing_already_spent_sec': TIMING_SPENT_SEC,
        'qualification_verdict': 'pending; capture checks, scene audit and acceptance margins are unresolved',
    })
    atomic_json(bundle/'plan_inputs.json', {str(path): sha(path) for path in (
        scenario, ROOT/'src/ridgeback_autonomy/config/benchmark_background_lab.yaml',
        ROOT/'src/ridgeback_autonomy/config/benchmark_backgrounds/lab.yaml')})
    report(bundle)
    return bundle


def validate_matrix(bundle):
    from ridgeback_autonomy.benchmarking.sweep import load_sweep
    from ridgeback_autonomy.benchmarking.scenarios import load_scenarios, select_scenes, parse_scene_ids
    from ridgeback_autonomy.benchmarking.trial_results import build_trials
    from ridgeback_autonomy.benchmarking.backends import resolve_benchmark_environment
    counts = {}
    for path, checksum in read_json(Path(bundle)/'plan_inputs.json').items():
        if sha(path) != checksum:
            raise ValueError('Scenario/background changed since preparation: '+path)
    for path in sorted((Path(bundle) / 'inputs').glob('*.yaml')):
        spec = load_sweep(str(path))
        resolve_benchmark_environment(spec.backend, 'target_distance_calibration', spec.backend_settings,
            package_share=lambda package: str(ROOT / 'install' / package / 'share' / package))
        trials = []
        for config in spec.configs:
            args = config.arguments
            expected_gate = 'silhouette' if config.name.startswith('silhouette') else 'box'
            expected_depth = 'monocular' if config.name.endswith('monocular') else 'stereoscopic'
            if args['mask_gate'] != expected_gate or args['depth_source'] != expected_depth:
                raise ValueError('Mask/depth settings differ from the named condition')
            if args['repeats'] != '1':
                raise ValueError('Run replications must not change within-scene repeats')
            scenes = select_scenes(load_scenarios(args['scenario']), parse_scene_ids(args.get('scene_ids', '')))
            count = len(build_trials(scenes, int(args['repeats'])))
            if args['estimators'] != ','.join(ESTIMATORS):
                raise ValueError('Unexpected estimator matrix')
            for key in ('record_video', 'overlay', 'estimate_viz', 'depth_match_debug'):
                if args[key] != 'true':
                    raise ValueError(f'{key} must remain enabled')
            if spec.defaults['detector_debug'] != 'true':
                raise ValueError('Detector diagnostics must remain enabled')
            trials.append(count)
        expected = [1, 1] if path.stem.startswith('check_') else [109] * 4
        if trials != expected:
            raise ValueError(f'{path.name}: expected {expected}, got {trials}')
        if not path.stem.startswith('check_') and tuple(c.name for c in spec.configs) != CONDITIONS:
            raise ValueError('The four-condition matrix changed')
        counts[path.stem] = trials
    return counts


def host_available():
    own = {os.getpid(), *[p.pid for p in psutil.Process().parents()]}
    heavy = {'gzserver', 'isaac_runner.py', 'target_distance_benchmark_runner',
             'target_benchmark_sweep', 'target_mask_measurement_node', 'target_detector_node'}
    for process in psutil.process_iter(['pid', 'cmdline', 'status']):
        if process.pid in own or process.info['status'] == psutil.STATUS_ZOMBIE:
            continue
        if any(Path(token).name in heavy for token in (process.info['cmdline'] or [])):
            raise RuntimeError(f'Another simulator/perception job owns the workstation: PID {process.pid}')
    sampled = subprocess.run(['nvidia-smi', 'pmon', '-c', '1', '-s', 'um'],
                             capture_output=True, text=True, timeout=8, check=True)
    for line in sampled.stdout.splitlines():
        fields = line.split()
        if len(fields) > 4 and not line.startswith('#') and fields[2] == 'C':
            if fields[3].isdigit() and int(fields[3]) > 5:
                raise RuntimeError(f'Another CUDA process is active: PID {fields[1]}')
    if shutil.disk_usage(ROOT).free < 10 * 1024**3:
        raise RuntimeError('Less than 10 GiB free; stop before exhausting evidence storage')


def _alive(tracked):
    result = []
    for pid, created in tracked.items():
        try:
            process = psutil.Process(pid)
            if process.create_time() == created and process.status() != psutil.STATUS_ZOMBIE:
                result.append(process)
        except psutil.Error:
            pass
    return result


def acquire_idle_inhibitor():
    """Keep the fd in the owner; an inhibitor wrapper would intercept SIGINT."""
    import dbus
    manager = dbus.Interface(dbus.SystemBus().get_object(
        'org.freedesktop.login1', '/org/freedesktop/login1'), 'org.freedesktop.login1.Manager')
    return manager.Inhibit('idle', 'Ridgeback qualification',
                           'Unattended benchmark is using the workstation', 'block').take()


def owned_command(command, log_path, env, deadline):
    """Track descendants across setsid and adopt orphans; never signal by name."""
    # PR_SET_CHILD_SUBREAPER keeps SDK helpers attributable if a leader exits.
    if ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) != 0:
        raise RuntimeError('Cannot establish child-process ownership')
    tracked = {}
    started = time.monotonic()
    interrupt_at = term_at = kill_at = None
    cleanup_escalated = False
    with Path(log_path).open('wb') as log:
        child = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        while child.poll() is None:
            for process in psutil.Process().children(recursive=True):
                try:
                    tracked[process.pid] = process.create_time()
                except psutil.Error:
                    pass
            now = time.monotonic()
            if (STOP_REQUESTED or now >= deadline) and interrupt_at is None:
                child.send_signal(signal.SIGINT)
                interrupt_at = now
            if interrupt_at is not None and now - interrupt_at >= 100 and term_at is None:
                for process in _alive(tracked):
                    try:
                        process.terminate()
                    except psutil.Error:
                        pass
                term_at = now
            if term_at is not None and now - term_at >= 20 and kill_at is None:
                for process in _alive(tracked):
                    try:
                        process.kill()
                    except psutil.Error:
                        pass
                kill_at = now
            if kill_at is not None and now-kill_at >= 10:
                break
            time.sleep(.25)
        # Reparented descendants remain ours even after the root exited.
        for process in psutil.Process().children(recursive=True):
            try:
                tracked[process.pid] = process.create_time()
            except psutil.Error:
                pass
        leftovers = _alive(tracked)
        if leftovers:
            for signum, grace in ((signal.SIGINT, 30), (signal.SIGTERM, 10), (signal.SIGKILL, 3)):
                if signum != signal.SIGINT and leftovers:
                    cleanup_escalated = True
                for process in leftovers:
                    try:
                        process.send_signal(signum)
                    except psutil.Error:
                        pass
                psutil.wait_procs(leftovers, timeout=grace)
                leftovers = _alive(tracked)
                if not leftovers:
                    break
    return {'command': command, 'wall_sec': time.monotonic()-started,
            'returncode': child.poll(), 'deadline_or_stop': interrupt_at is not None,
            'outer_escalated': term_at is not None or kill_at is not None or cleanup_escalated,
            'leftover_pids': [p.pid for p in leftovers], 'owned_pids': sorted(tracked),
            'log': str(log_path)}


def environment(bundle, backend=None):
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='127', GZ_PARTITION='qualification_'+Path(bundle).name,
               RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', HF_HUB_OFFLINE='1',
               TRANSFORMERS_OFFLINE='1', PYTHONUNBUFFERED='1',
               CYCLONEDDS_URI=str(ROOT / 'install/ridgeback_autonomy/share/ridgeback_autonomy/config/cyclonedds.xml'),
               ROS_LOG_DIR=str(Path(bundle) / 'ros'),
               QUALIFICATION_CAPTURE_TIMINGS=str(Path(bundle) / 'capture_windows.jsonl'))
    env['PYTHONPATH'] = str(HERE / 'observer') + os.pathsep + env.get('PYTHONPATH', '')
    if backend == 'isaac':
        env['AMENT_PREFIX_PATH'] = os.pathsep.join(x for x in env.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
                                                 if not x.endswith('/ridgeback_autonomy_gz'))
    return env


def find_sweep(bundle, stage):
    paths = list((Path(bundle) / stage).glob('*/sweep.json'))
    if len(paths) != 1:
        raise ValueError(f'Expected exactly one sweep for {stage}; no automatic reuse across repetitions')
    return paths[0]


def run(bundle):
    global STOP_REQUESTED
    bundle = Path(bundle).resolve()
    state = read_json(bundle / 'campaign.json')
    if state['status'] != 'prepared-not-launched':
        raise RuntimeError('Campaign already launched. Preserve it; resume needs review, not automatic retry.')
    def stop(_signal, _frame):
        global STOP_REQUESTED
        STOP_REQUESTED = True
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    lock_path = ROOT / 'artifacts/benchmarks/.qualification-workstation.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        started = time.monotonic()
        end = started + TOTAL_SEC
        state.update(status='running-preflight', started_utc=datetime.now(timezone.utc).isoformat())
        atomic_json(bundle / 'campaign.json', state)
        idle_fd = None
        try:
            idle_fd = acquire_idle_inhibitor()
            host_available()
            check_installed()
            state['matrix'] = validate_matrix(bundle)
            atomic_json(bundle / 'frozen.json', {
                'files': frozen_files(bundle),
                'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'dirty_patch': subprocess.check_output(['git', 'diff', '--binary', '--', 'src', 'tools/benchmark/qualification'], cwd=ROOT, text=True),
            })
            live_end = None
            for stage in ('check_gz', 'check_isaac', *ORDER):
                check_frozen(bundle)
                host_available()
                proof = stage.startswith('check_')
                if not proof and live_end is None:
                    live_end = min(time.monotonic()+LIVE_SEC, end-CLEANUP_SEC)
                stage_end = min(time.monotonic()+(240 if proof else 8100),
                                end-CLEANUP_SEC, end if proof else live_end)
                env = environment(bundle, 'isaac' if 'isaac' in stage else 'gz')
                env.update(QUALIFICATION_BUNDLE=str(bundle), QUALIFICATION_STAGE=stage,
                           QUALIFICATION_STOP_AT=str(stage_end-90),
                           QUALIFICATION_ADMISSION_SEC=str(30 if proof else ADMISSION_SEC))
                state.update(status='checking-capture' if proof else 'running-live', current_stage=stage)
                atomic_json(bundle / 'campaign.json', state)
                operation = owned_command([sys.executable, str(HERE/'sweep_entry.py'), str(bundle/'inputs'/f'{stage}.yaml'),
                                           '--skip-preflight-cleanup'], bundle/f'{stage}.log', env, stage_end)
                operation['stage'] = stage
                state['operations'].append(operation)
                atomic_json(bundle/'campaign.json', state)
                if (operation['returncode'] or operation['deadline_or_stop']
                        or operation['leftover_pids'] or operation['outer_escalated']):
                    raise RuntimeError(f'{stage} stopped; inspect {stage}.log and guard.json')
                manifest = read_json(find_sweep(bundle, stage))
                if manifest['sweep']['status'] != 'complete' or not manifest['lifecycle']['clean']:
                    raise RuntimeError(f'{stage} has incomplete or unclean results')
                guard = read_json(bundle/f'{stage}_guard.json')
                expected = {'box_stereo', 'box_monocular'} if proof else set(CONDITIONS)
                if guard.get('error') or set(guard.get('checked', {})) != expected:
                    raise RuntimeError(f'{stage} is missing condition-level validation evidence')
            state['status'] = 'processing-offline'
            atomic_json(bundle/'campaign.json', state)
            offline(bundle, state, end-CLEANUP_SEC)
            check_frozen(bundle)
            state['status'] = 'execution-complete-review-required'
        except BaseException as error:
            state['status'] = 'stopped' if STOP_REQUESTED else 'failed'
            state['errors'].append(f'{type(error).__name__}: {error}')
        finally:
            state.update(elapsed_sec=time.monotonic()-started,
                         finished_utc=datetime.now(timezone.utc).isoformat())
            atomic_json(bundle/'campaign.json', state)
            report(bundle)
            if idle_fd is not None:
                os.close(idle_fd)
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    return 0 if state['status'] == 'execution-complete-review-required' else 1


def offline(bundle, state, deadline):
    for stage in ('gz_r1', 'isaac_r1'):
        sweep_dir = find_sweep(bundle, stage).parent
        for condition in CONDITIONS:
            check_frozen(bundle)
            if STOP_REQUESTED or time.monotonic()+120 >= deadline:
                raise RuntimeError('No remaining budget for another offline condition')
            sensor = bundle/'captures'/stage/condition
            cache = bundle/'masks'/stage/condition
            cache.parent.mkdir(parents=True, exist_ok=True)
            producer = 'slimsam' if condition.startswith('silhouette') else 'box'
            commands = [([str(ROOT/'perception_venv/bin/python3'), '-m',
                'ridgeback_autonomy.benchmarking.target_replay_materialize_masks', str(sensor),
                '--producer', producer, '--output-dir', str(cache)], 'masks')]
            job = {'job_version': 1, 'question': 'validate-live-replay', 'profile': 'mask-output',
                   'inputs': {'sensor_capture': str(sensor), 'mask_caches': [str(cache)]},
                   'sweep': {'sweep': {'name': f'{stage}_{condition}'}, 'configs': [
                       {'name': 'matching_defaults', 'estimators': ','.join(ESTIMATORS)}]},
                   'resources': {'measurement_workers': 1},
                   'output_dir': str(bundle/'replay'/stage/condition)}
            # Generated outputs are not part of the frozen live input set.
            job_path = bundle / f'offline_{stage}_{condition}.yaml'
            job_path.write_text(yaml.safe_dump(job, sort_keys=False))
            commands.append(([sys.executable, '-m', 'ridgeback_autonomy.benchmarking.target_replay_benchmark',
                              str(job_path)], 'numeric'))
            for command, suffix in commands:
                op = owned_command(command, bundle/f'{stage}_{condition}_{suffix}.log', environment(bundle), deadline)
                state['operations'].append(dict(op, stage=f'{stage}_{condition}_{suffix}'))
                atomic_json(bundle/'campaign.json', state)
                if op['returncode'] or op['deadline_or_stop'] or op['leftover_pids'] or op['outer_escalated']:
                    raise RuntimeError('Offline execution failed; retain inputs and diagnostics')
            replay = bundle/'replay'/stage/condition/'results/matching_defaults'
            parity = {key: compare_rows(csv_rows(sweep_dir/condition, key), csv_rows(replay, key))
                      for key in ESTIMATORS}
            state.setdefault('parity', {})[f'{stage}/{condition}'] = parity
            atomic_json(bundle/'campaign.json', state)
            if not all(result['passed'] for result in parity.values()):
                raise RuntimeError(f'Trial-level replay mismatch: {stage}/{condition}')
            repeat_output = bundle/'replay_repeat'/stage/condition
            job['output_dir'] = str(repeat_output)
            job_path = bundle/f'offline_repeat_{stage}_{condition}.yaml'
            job_path.write_text(yaml.safe_dump(job, sort_keys=False))
            op = owned_command([sys.executable, '-m', 'ridgeback_autonomy.benchmarking.target_replay_benchmark',
                                str(job_path)], bundle/f'{stage}_{condition}_determinism.log',
                               environment(bundle), deadline)
            state['operations'].append(dict(op, stage=f'{stage}_{condition}_determinism'))
            if op['returncode'] or op['deadline_or_stop'] or op['leftover_pids'] or op['outer_escalated']:
                raise RuntimeError('Repeated fixed-input evaluation failed')
            deterministic = {key: compare_rows(csv_rows(replay, key),
                csv_rows(repeat_output/'results/matching_defaults', key), tolerance=0) for key in ESTIMATORS}
            state.setdefault('fixed_input_checks', {})[f'{stage}/{condition}'] = deterministic
            atomic_json(bundle/'campaign.json', state)
            if not all(result['passed'] for result in deterministic.values()):
                raise RuntimeError('Identical frozen numeric evaluations differed')


def report(bundle):
    bundle = Path(bundle)
    state = read_json(bundle/'campaign.json')
    service_result = os.environ.get('SERVICE_RESULT')
    if service_result and service_result != 'success':
        state['status'] = 'service-failed'
        state['errors'].append('User service: '+service_result)
        atomic_json(bundle/'campaign.json', state)
    comparisons = {}
    for backend in ('gz', 'isaac'):
        try:
            first = find_sweep(bundle, backend+'_r1').parent
            second = find_sweep(bundle, backend+'_r2').parent
            for condition in CONDITIONS:
                for estimator in ESTIMATORS:
                    try:
                        comparisons[f'{backend}/{condition}/{estimator}'] = repeatability(
                            csv_rows(first/condition, estimator), csv_rows(second/condition, estimator))
                    except (OSError, ValueError, KeyError) as error:
                        comparisons[f'{backend}/{condition}/{estimator}'] = {'error': str(error)}
        except ValueError:
            continue
    atomic_json(bundle/'comparisons.json', comparisons)
    lines = ['# Overnight benchmark campaign', '', f"Execution status: **{state['status']}**", '',
             '16 full condition runs: Gazebo/Isaac × four conditions × two independent sessions.', '',
             '| Stage | Exit | Wall seconds | Owned leftovers |', '| --- | ---: | ---: | --- |']
    for operation in state['operations']:
        lines.append(f"| {operation['stage']} | {operation['returncode']} | {operation['wall_sec']:.1f} | {operation['leftover_pids']} |")
    lines += ['', '## Problems', ''] + (['- '+e for e in state['errors']] or ['No execution error recorded.'])
    lines += ['', '## Full live configurations', '',
              '| Session / condition | Checked | Captured events |', '| --- | --- | ---: |']
    for stage in ORDER:
        path = bundle/f'{stage}_guard.json'
        checked = read_json(path).get('checked', {}) if path.exists() else {}
        for condition in CONDITIONS:
            result = checked.get(condition)
            lines.append(f"| {stage}/{condition} | {'yes' if result else 'pending'} | {result['window_events'] if result else '—'} |")
    lines += ['', '## Repeatability', '',
              '| Backend / condition / estimator | Changed outcome/reason rows | Median paired estimate delta (m) |',
              '| --- | ---: | ---: |']
    for key, result in comparisons.items():
        lines.append(f"| {key} | {result.get('outcome_or_reason_changes', 'unavailable')} | {result.get('median_abs_estimate_delta_m', 'unavailable')} |")
    lines += ['', '## Frozen-input replay', '',
              '| Dataset | Trial/instance parity | Repeated numeric evaluation |', '| --- | --- | --- |']
    for key, values in state.get('parity', {}).items():
        repeated = state.get('fixed_input_checks', {}).get(key)
        lines.append(f"| {key} | {all(r['passed'] for r in values.values())} | {all(r['passed'] for r in repeated.values()) if repeated else 'pending'} |")
    for path in bundle.glob('*_guard.json'):
        guard = read_json(path)
        if guard.get('error'):
            lines.append(f"- {path.name}: {guard['error']}")
    lines += ['', '## Qualification scope', '',
              'Execution success is not a scientific qualification verdict. See comparisons.json for descriptive repeatability and campaign.json for trial/instance replay parity.',
              'Scientific repeatability margins, full per-event live-output parity, camera/LiDAR scene audit and deliberate cancellation/pending-request qualification still require explicit evidence. They are never silently marked passed.',
              'Missing required sensor inputs or a capture window that follows a batch quota stops the batch before full runs.',
              'Current capture integration is pending: correct RGB/intrinsics/scan retention AND timed monocular capture must pass the four short capture checks.', '',
              'No automatic experiment retry or replacement is performed. Failures retain their output. No hardware is operated.', '',
              'The timing study already consumed 428 seconds; the job budget is the remaining part of the 12-hour allocation, with nine hours maximum for the full live blocks and a final cleanup reserve.', '']
    (bundle/'REPORT.md').write_text('\n'.join(lines))


def start(bundle):
    bundle = Path(bundle).resolve()
    if read_json(bundle/'campaign.json')['status'] != 'prepared-not-launched':
        raise RuntimeError('Bundle has already been launched')
    validate_matrix(bundle)
    check_installed()
    host_available()
    unit = 'ridgeback-qualification-'+bundle.name.replace('_','-')
    launch = bundle/'launch.sh'
    import shlex
    launch.write_text('#!/bin/bash\nset -e\ncd '+shlex.quote(str(ROOT))+'\n'
                      'source /opt/ros/jazzy/setup.bash\nsource install/setup.bash\n'
                      'exec tools/gpu-run /usr/bin/python3 '+shlex.quote(str(HERE/'campaign.py'))+
                      ' run '+shlex.quote(str(bundle))+'\n')
    command = ['systemd-run', '--user', '--unit='+unit, '--property=Restart=no',
               '--property=KillMode=mixed', '--property=KillSignal=SIGINT',
               '--property=TimeoutStopSec=150', '--property=RuntimeMaxSec='+str(TOTAL_SEC),
               '--property=WorkingDirectory='+str(ROOT),
               '--property=ExecStopPost='+shlex.join(['/usr/bin/python3', str(HERE/'campaign.py'), 'report', str(bundle)])]
    for key in ('DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS'):
        if os.environ.get(key):
            command.append('--setenv='+key+'='+os.environ[key])
    command += ['/bin/bash', str(launch)]
    subprocess.run(command, check=True)
    atomic_json(bundle/'service.json', {'unit': unit, 'command': command})
    print(f'Started {unit}.service; automatic report: {bundle}/REPORT.md')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'check', 'start', 'run', 'report'))
    parser.add_argument('bundle', type=Path)
    args = parser.parse_args()
    if args.action == 'prepare':
        print(prepare(args.bundle))
    elif args.action == 'check':
        print(json.dumps({'matrix': validate_matrix(args.bundle),
                          'launch_status': 'prepared; timed stereo/monocular capture checks have NOT run'}, indent=2))
    elif args.action == 'run':
        return run(args.bundle)
    elif args.action == 'start':
        start(args.bundle)
    else:
        report(args.bundle)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f'qualification campaign: {error}', file=sys.stderr)
        raise SystemExit(2)
