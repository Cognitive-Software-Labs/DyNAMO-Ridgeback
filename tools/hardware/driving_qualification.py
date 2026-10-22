#!/usr/bin/env python3
"""Create and finalize uniform D-lane qualification evidence.

Recorders own raw traces and their automatic checks.  This tool adds the
human/physical observation that software cannot infer, preserves hashes and
conditions, and builds a matrix without upgrading incomplete evidence to a
pass.  It never publishes ROS messages or changes robot state.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone


ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / 'artifacts/hardware'
VERDICTS = ('passed', 'failed', 'inconclusive', 'aborted')
VIDEO_REQUIRED = frozenset(('D6', 'D7', 'D8'))
TRANSLATION_SPEEDS = (0.05, 0.10, 0.20, 0.40, 0.80, 1.20)
ROTATION_SPEEDS = (0.10, 0.25, 0.50, 1.00, 1.50)
BASELINE_FILES = (
    ROOT / 'tools/hardware/commissioning_speed_guard.py',
    ROOT / 'tools/hardware/observe_driving_stop.py',
    ROOT / 'tools/hardware/check_raised_wheel_link_fault.py',
    Path('/etc/systemd/system/dynamo-commissioning-speed-guard.service'),
    Path('/etc/systemd/system/clearpath-platform.service.d/70-dynamo-commissioning.conf'),
    Path('/etc/systemd/system/clearpath-platform.service.d/95-dynamo-puma-feedback.conf'),
    Path('/etc/systemd/system/clearpath-platform.service.d/97-dynamo-puma-field-driver.conf'),
    Path('/etc/clearpath/robot.yaml'),
    Path('/home/robot/controllers.txt'),
)
BASELINE_UNITS = ('clearpath-platform.service', 'dynamo-commissioning-speed-guard.service')


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path, value):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')
    os.replace(temporary, path)


def validate_run_dir(path, must_exist=True):
    resolved = path.resolve()
    if resolved == EVIDENCE or not resolved.is_relative_to(EVIDENCE):
        raise ValueError('run directory must be below artifacts/hardware')
    if must_exist and not resolved.is_dir():
        raise ValueError('run directory does not exist')
    return resolved


def automatic_result(run_dir):
    candidates = (run_dir / 'result.json', run_dir / 'observer/result.json',
                  run_dir / 'capture-context.json')
    for candidate in candidates:
        if candidate.is_file():
            try:
                return candidate, json.loads(candidate.read_text())
            except (OSError, json.JSONDecodeError) as error:
                raise ValueError(f'invalid automatic result: {candidate}: {error}') from error
    raise ValueError('no recorder result.json or capture-context.json exists')


def result_supports_pass(result):
    """Accept only an explicit successful automatic result, never absence."""
    if result.get('passed') is True:
        return True
    if result.get('automatic_checks_passed') is True:
        return True
    status = str(result.get('status', '')).lower()
    return status in {
        'passed', 'complete', 'recording-finished', 'pulse-completed-final-odometry-stationary',
        'feedback-clock-post-boot-passed', 'recovered-stationary',
        'live-handover-observed-awaiting-operator',
        'stationary-controller-cycles-observed',
    } or (status == 'captured-not-automatically-qualified'
          and result.get('observer_exit_code') == 0
          and result.get('capture_died_early') is False
          and result.get('replay_exit_code') == 0)


def finalize(run_dir, lane, mechanism, verdict, observation, video=None,
             recovery=None, final_mode=None, conditions=None, measurements=None, baseline=None):
    run_dir = validate_run_dir(run_dir)
    lane = lane.upper()
    if lane not in {f'D{i}' for i in range(1, 9)}:
        raise ValueError('lane must be D1..D8')
    if verdict not in VERDICTS:
        raise ValueError('invalid verdict')
    source, automatic = automatic_result(run_dir)
    reasons = []
    video_record = None
    if video:
        video = Path(video).resolve()
        if not video.is_file() or not video.stat().st_size:
            reasons.append('video evidence is missing or empty')
        else:
            video_record = {'path': str(video), 'bytes': video.stat().st_size,
                            'sha256': sha256(video)}
    if not observation.strip():
        reasons.append('physical observation is missing')
    automatic_support = result_supports_pass(automatic) or (
        lane == 'D1' and automatic.get('status') == 'recorded-awaiting-analysis-and-operator-observation'
        and automatic.get('movement_command_detected') is True
        and automatic.get('stop_release_detected') is True)
    if verdict == 'passed' and not automatic_support:
        reasons.append('automatic recorder result does not support a pass')
    if verdict == 'passed' and lane in VIDEO_REQUIRED and video_record is None:
        reasons.append(f'{lane} requires nonempty independent video evidence')
    if verdict == 'passed' and lane in ('D6', 'D7') and not recovery:
        reasons.append(f'{lane} requires a supervised recovery result')
    baseline_record = None
    if baseline:
        baseline = Path(baseline).resolve()
        try:
            payload = json.loads(baseline.read_text())
            if payload.get('schema') != 'dynamo-driving-baseline-v1':
                raise ValueError('wrong baseline schema')
            baseline_record = {'path': str(baseline), 'sha256': sha256(baseline)}
        except (OSError, json.JSONDecodeError, ValueError) as error:
            reasons.append(f'invalid baseline snapshot: {error}')
    if verdict == 'passed' and lane in VIDEO_REQUIRED and baseline_record is None:
        reasons.append(f'{lane} requires a hashed baseline snapshot')
    measurements = measurements or {}
    if verdict == 'passed' and lane == 'D8':
        required = ('target_speed', 'actual_speed', 'stopping_distance_m',
                    'distance_uncertainty_m', 'stop_duration_s')
        missing = [name for name in required if measurements.get(name) is None]
        if missing:
            reasons.append('D8 measurements missing: ' + ', '.join(missing))
        elif abs(measurements['actual_speed']) < .9 * abs(measurements['target_speed']):
            reasons.append('actual speed did not reach 90% of target speed')
        for key in ('test_series', 'motion_class', 'direction', 'floor', 'payload',
                    'battery_state', 'tire_condition', 'available_clearance_m'):
            if conditions is None or conditions.get(key) in (None, ''):
                reasons.append(f'D8 condition missing: {key}')
        if not reasons:
            speeds = TRANSLATION_SPEEDS if conditions['motion_class'] == 'translation' else ROTATION_SPEEDS
            target = abs(float(measurements['target_speed']))
            if not any(abs(target - candidate) < 1e-6 for candidate in speeds):
                reasons.append('target speed is not in the approved progression')
            if measurements['stopping_distance_m'] < 0 or measurements['distance_uncertainty_m'] < 0:
                reasons.append('distance and uncertainty must be nonnegative')
            if (measurements['stopping_distance_m'] + measurements['distance_uncertainty_m']
                    > float(conditions['available_clearance_m'])):
                reasons.append('stopping distance plus uncertainty exceeds available clearance')
            prior = speeds[:speeds.index(min(speeds, key=lambda value: abs(value - target)))]
            existing = []
            for path in EVIDENCE.rglob('qualification.json'):
                try:
                    item = json.loads(path.read_text())
                except (OSError, json.JSONDecodeError):
                    continue
                if (item.get('lane') == 'D8' and item.get('verdict') == 'passed'
                        and item.get('mechanism') == mechanism
                        and item.get('conditions', {}).get('test_series') == conditions['test_series']
                        and item.get('conditions', {}).get('direction') == conditions['direction']):
                    existing.append(abs(float(item.get('measurements', {}).get('target_speed', -1))))
            missing_prior = [value for value in prior
                             if not any(abs(value - found) < 1e-6 for found in existing)]
            if missing_prior:
                reasons.append('lower speed stages have not passed: ' + ', '.join(map(str, missing_prior)))
    if reasons and verdict == 'passed':
        verdict = 'inconclusive'
    record = {
        'schema': 'dynamo-driving-qualification-v1',
        'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
        'exact_finalize_command': sys.argv,
        'lane': lane,
        'mechanism': mechanism,
        'verdict': verdict,
        'verdict_reasons': reasons,
        'automatic_result': {'path': str(source), 'sha256': sha256(source),
                             'status': automatic.get('status'),
                             'passed': automatic.get('passed'),
                             'recorder_context': {key: automatic[key] for key in
                                                  ('source_sha256', 'observer_command', 'namespace', 'limits')
                                                  if key in automatic}},
        'baseline_snapshot': baseline_record,
        'physical_observation': observation.strip(),
        'video': video_record,
        'recovery_result': recovery,
        'final_installed_mode': final_mode,
        'conditions': conditions or {},
        'measurements': measurements,
    }
    atomic_json(run_dir / 'qualification.json', record)
    return record


def build_matrix(root):
    root = validate_run_dir(root) if root.resolve() != EVIDENCE else EVIDENCE
    records = []
    for path in sorted(root.rglob('qualification.json')):
        try:
            record = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        if record.get('schema') == 'dynamo-driving-qualification-v1':
            records.append({'path': str(path), **record})
    summary = {}
    for lane in (f'D{i}' for i in range(1, 9)):
        lane_records = [r for r in records if r.get('lane') == lane]
        summary[lane] = {
            'passed': sum(r.get('verdict') == 'passed' for r in lane_records),
            'failed': sum(r.get('verdict') == 'failed' for r in lane_records),
            'inconclusive': sum(r.get('verdict') == 'inconclusive' for r in lane_records),
            'aborted': sum(r.get('verdict') == 'aborted' for r in lane_records),
            'mechanisms_passed': sorted({r.get('mechanism') for r in lane_records
                                         if r.get('verdict') == 'passed'}),
        }
    passed = [record for record in records if record.get('verdict') == 'passed']
    count = lambda lane, mechanism: sum(
        record.get('lane') == lane and record.get('mechanism') == mechanism for record in passed)
    d1 = ('forward', 'backward', 'left', 'right', 'turn-left', 'turn-right')
    d2 = ('command-loss', 'deadman', 'physical-e-stop')
    reconnect_cycles = sum(int(record.get('measurements', {}).get('cycles', 0)) for record in passed
                           if record.get('lane') == 'D3' and record.get('mechanism') == 'reconnect-cycle')
    d8_mechanisms = ('command-loss', 'deadman', 'physical-e-stop')
    d8_directions = {
        'translation': (('forward', 'backward', 'left', 'right'), TRANSLATION_SPEEDS),
        'rotation': (('turn-left', 'turn-right'), ROTATION_SPEEDS),
    }
    d8_groups = []
    for mechanism in d8_mechanisms:
        for motion_class, (directions, speeds) in d8_directions.items():
            for direction in directions:
                candidates = {record.get('conditions', {}).get('test_series') for record in passed
                              if record.get('lane') == 'D8' and record.get('mechanism') == mechanism
                              and record.get('conditions', {}).get('motion_class') == motion_class
                              and record.get('conditions', {}).get('direction') == direction}
                complete_series = []
                for series in candidates - {None}:
                    if all(sum(record.get('lane') == 'D8' and record.get('mechanism') == mechanism
                               and record.get('conditions', {}).get('test_series') == series
                               and record.get('conditions', {}).get('direction') == direction
                               and abs(float(record.get('measurements', {}).get('target_speed', -1)) - speed) < 1e-6
                               for record in passed) >= 3 for speed in speeds):
                        complete_series.append(series)
                d8_groups.append({'mechanism': mechanism, 'direction': direction,
                                  'complete_series': sorted(complete_series)})
    acceptance = {
        'D1': {'complete': all(count('D1', mechanism) >= 1 for mechanism in d1),
               'requirement': 'one bounded sign trial in all six principal directions'},
        'D2': {'complete': all(count('D2', mechanism) >= 3 for mechanism in d2),
               'requirement': 'three isolated repetitions each: command loss, deadman and physical e-stop'},
        'D3': {'complete': count('D3', 'silent-radio-loss') >= 3 and reconnect_cycles >= 3,
               'requirement': 'three genuine silent radio losses and three neutral reconnect/input/return cycles',
               'reconnect_cycles': reconnect_cycles},
        'D4': {'complete': any(record.get('lane') == 'D4'
                               and record.get('conditions', {}).get('final_configuration') is True
                               for record in passed),
               'requirement': 'one handover regression on the final installed configuration'},
        'D5': {'complete': count('D5', 'stationary-field-freshness') >= 1,
               'requirement': 'installed stationary field-freshness acceptance; moving coverage remains D6-D8'},
        'D6': {'complete': count('D6', 'feedback-loss') >= 3,
               'requirement': 'three raised-wheel feedback-loss trials with video and recovery'},
        'D7': {'complete': count('D7', 'command-link-loss') >= 3,
               'requirement': 'three raised-wheel outgoing-link loss trials with video and recovery'},
        'D8': {'complete': all(group['complete_series'] for group in d8_groups),
               'requirement': 'three measurements at every staged speed for each stop mechanism and principal direction in one consistent series',
               'groups': d8_groups},
    }
    return {'schema': 'dynamo-driving-acceptance-matrix-v1', 'summary': summary,
            'acceptance': acceptance,
            'records': records}


def key_values(values):
    output = {}
    for item in values:
        if '=' not in item:
            raise ValueError(f'expected NAME=VALUE: {item}')
        key, value = item.split('=', 1)
        try:
            output[key] = float(value)
        except ValueError:
            output[key] = value
    return output


def command_output(command):
    completed = subprocess.run(command, capture_output=True, text=True, timeout=10)
    return {'command': command, 'returncode': completed.returncode,
            'stdout': completed.stdout.strip(), 'stderr': completed.stderr.strip()}


def baseline_snapshot(extra_files=()):
    files = {}
    for path in (*BASELINE_FILES, *extra_files):
        path = Path(path)
        try:
            files[str(path)] = {'sha256': sha256(path), 'bytes': path.stat().st_size}
        except OSError as error:
            files[str(path)] = {'unavailable': str(error)}
    units = {unit: command_output([
        'systemctl', 'show', unit, '-p', 'ActiveState', '-p', 'SubState', '-p', 'MainPID',
        '-p', 'ExecStart', '--no-pager']) for unit in BASELINE_UNITS}
    processes = command_output(['ps', '-eo', 'pid,lstart,args'])
    relevant = [line for line in processes['stdout'].splitlines()
                if any(token in line for token in ('multi_puma_node', 'controller_manager',
                                                    'commissioning_speed_guard', 'joy_node', 'ds4drv'))]
    return {
        'schema': 'dynamo-driving-baseline-v1',
        'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
        'exact_command': sys.argv,
        'repository_revision': command_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD']),
        'repository_status': command_output(['git', '-C', str(ROOT), 'status', '--short']),
        'files': files,
        'units': units,
        'relevant_processes': relevant,
        'link_fault_rules': command_output(['/usr/sbin/iptables', '-w', '-S']),
        'limits': 'Read-only identity snapshot. Run the stationary graph/clock checks separately; unavailable privileged firewall evidence is not treated as clean.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='action', required=True)
    finish = sub.add_parser('finalize')
    finish.add_argument('--run-dir', type=Path, required=True)
    finish.add_argument('--lane', required=True)
    finish.add_argument('--mechanism', required=True)
    finish.add_argument('--verdict', choices=VERDICTS, required=True)
    finish.add_argument('--observation', required=True)
    finish.add_argument('--video', type=Path)
    finish.add_argument('--recovery')
    finish.add_argument('--final-mode')
    finish.add_argument('--baseline', type=Path)
    finish.add_argument('--condition', action='append', default=[])
    finish.add_argument('--measurement', action='append', default=[])
    matrix = sub.add_parser('matrix')
    matrix.add_argument('--root', type=Path, default=EVIDENCE)
    matrix.add_argument('--output', type=Path, required=True)
    snapshot = sub.add_parser('snapshot')
    snapshot.add_argument('--output', type=Path, required=True)
    snapshot.add_argument('--config', type=Path, action='append', default=[])
    args = parser.parse_args()
    try:
        if args.action == 'finalize':
            record = finalize(args.run_dir, args.lane, args.mechanism, args.verdict,
                              args.observation, args.video, args.recovery, args.final_mode,
                              key_values(args.condition), key_values(args.measurement), args.baseline)
            print(json.dumps(record, indent=2))
            return 0 if record['verdict'] == args.verdict else 1
        output = args.output.resolve()
        if output.exists() or not output.is_relative_to(EVIDENCE):
            raise ValueError('matrix output must be a fresh path below artifacts/hardware')
        output.parent.mkdir(parents=True, exist_ok=True)
        record = (baseline_snapshot(args.config) if args.action == 'snapshot'
                  else build_matrix(args.root))
        atomic_json(output, record)
        print(json.dumps({'summary': record['summary'], 'acceptance': record['acceptance']}
                         if 'summary' in record else record, indent=2))
        return 0
    except ValueError as error:
        parser.error(str(error))


if __name__ == '__main__':
    sys.exit(main())
