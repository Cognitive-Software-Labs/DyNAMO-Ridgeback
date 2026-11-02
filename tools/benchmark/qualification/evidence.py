"""Fail-closed evidence checks and comparisons; no ROS or model initialization."""
import csv
import json
import math
from pathlib import Path
import re
import statistics
import subprocess

ESTIMATORS = ('projective_ranging', 'euclidean_reconstruction', 'polar_profiling')
CONDITIONS = ('box_stereo', 'silhouette_stereo', 'box_monocular', 'silhouette_monocular')
PARITY_ATOL_M = 0.0001


def read_json(path):
    return json.loads(Path(path).read_text())


def capture_windows(path, run_label, expected_trials, seconds):
    records = [json.loads(line) for line in Path(path).read_text().splitlines()]
    selected = [row for row in records if row['run_label'] == run_label]
    if len(selected) != expected_trials or len({r['trial_id'] for r in selected}) != expected_trials:
        raise ValueError('Missing or duplicated capture-window timing records')
    # Diagnostic execution tolerance, not a scientific equivalence margin.
    if any(abs(row['wall_sec'] - seconds) > .5 for row in selected):
        raise ValueError('Capture did not follow the requested wall-time window; quota capture is insufficient')
    if any(not row.get('total_events') for row in selected):
        raise ValueError('A capture window received no measurement batches, including empty batches')
    return selected


def sensor_completeness(path, depth_source, expected_trials):
    from ridgeback_autonomy.benchmarking.replay_artifacts import load_artifact, load_sensor_trial
    artifact = load_artifact(path, require_parent=False)
    if artifact.kind != 'sensor-capture' or len(artifact.trial_entries) != expected_trials:
        raise ValueError('Wrong sensor artifact kind or trial count')
    parameters = artifact.manifest['producer'].get('parameters', {})
    if str(parameters.get('depth_source')) != depth_source:
        raise ValueError('Sensor parent does not preserve the requested depth source')
    required = ('rgb', 'depth_m', 'intrinsics', 'camera_rotation', 'camera_translation',
                'scan', 'scan_rotation', 'scan_translation')
    total = detections = empty = 0
    explicit_missing = {key: 0 for key in required}
    for entry in artifact.trial_entries:
        _trial, events = load_sensor_trial(artifact, entry)
        if not events:
            raise ValueError(f"No events in {entry['trial_id']}")
        for event in events:
            missing = [key for key in required if event.get(key) is None]
            for key in missing:
                explicit_missing[key] += 1
            if missing and event['detections']:
                raise ValueError(f"{entry['trial_id']} stamp {event['stamp_ns']} missing {missing}")
            empty += not event['detections']
            if (event['image_width'], event['image_height']) != (640, 480):
                raise ValueError('Unexpected camera dimensions')
            total += 1
            detections += len(event['detections'])
    return {'artifact_id': artifact.id, 'events': total, 'detections': detections,
            'empty_detection_batches': empty, 'explicit_missing_inputs': explicit_missing,
            'manifest_sha256': artifact.manifest_sha256}


def processing_diagnostics(log_path, config):
    text = Path(log_path).read_text(errors='replace')
    start = text.find('[benchmark-sweep] '+config+': starting ')
    if start < 0:
        return {}
    end = text.find('[benchmark-sweep] ', start+20)
    section = text[start:end if end >= 0 else len(text)]
    stages = {}
    pattern = r'stage ([a-z_]+)_ms=count=(\d+) first=([\d.]+).*?warm_n=(\d+) warm_mean=([\d.]+|n/a).*?warm_p95=([\d.]+|n/a)'
    for match in re.finditer(pattern, section):
        name, count, first, warm, mean, p95 = match.groups()
        stages[name] = {'count': int(count), 'first_ms': float(first), 'warm_n': int(warm),
                        'warm_mean_ms': None if mean == 'n/a' else float(mean),
                        'warm_p95_ms': None if p95 == 'n/a' else float(p95)}
    return stages


def validate_run(run_dir, entry, expected_trials, timing_path):
    run_dir = Path(run_dir)
    document = read_json(run_dir / 'run.json')
    run, parameters = document['run'], document['parameters']
    if run['trials_included'] != expected_trials or run['trials_skipped'] != 0:
        raise ValueError(f"Trial accounting: included={run['trials_included']} skipped={run['trials_skipped']}")
    if {e['key'] for e in document['estimators']} != set(ESTIMATORS):
        raise ValueError('Estimator set differs from the frozen matrix')
    if run.get('detector_identity_verified') is not True:
        raise ValueError('Detector identity was not verified')
    shutdown = entry.get('shutdown') or {}
    if (entry.get('launch_returncode') != 0 or not shutdown.get('group_empty')
            or shutdown.get('escalated') or shutdown.get('returncode') != 0):
        raise ValueError('Configuration did not shut down cleanly')
    if not run.get('background', {}).get('props_spawned'):
        raise ValueError('Furnished background was not spawned')
    checks = run['background'].get('trials', [])
    if len(checks) != expected_trials or any(not r.get('state_check', {}).get('ok') for r in checks):
        raise ValueError('A scene state check failed')
    windows = capture_windows(timing_path, run['label'], expected_trials,
                              float(parameters['capture_sec']))
    for estimator in ESTIMATORS:
        rows = csv_rows(run_dir, estimator)
        if len(rows) != run['instances'] or {key[0] for key in rows} != {w['trial_id'] for w in windows}:
            raise ValueError('CSV trial/instance accounting differs from the completed run')
    video = run_dir / 'video/run.mp4'
    result = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration',
                             '-of', 'json', str(video)], capture_output=True, text=True, check=True)
    if float(json.loads(result.stdout)['format']['duration']) <= 0:
        raise ValueError('Video is empty')
    capture = None
    if parameters.get('sensor_capture_dir'):
        capture = sensor_completeness(parameters['sensor_capture_dir'],
                                      parameters['depth_source'], expected_trials)
    return {'run_json': str(run_dir / 'run.json'), 'capture': capture,
            'window_events': sum(row['total_events'] or 0 for row in windows),
            'estimators': document['estimators']}


def csv_rows(run_dir, estimator):
    matches = [p for p in Path(run_dir).glob('*.csv') if estimator in p.name]
    if len(matches) != 1:
        raise ValueError(f'Expected one {estimator} CSV in {run_dir}')
    with matches[0].open() as stream:
        rows = list(csv.DictReader(stream))
    keyed = {(row['trial_id'], row['instance_index']): row for row in rows}
    if len(keyed) != len(rows):
        raise ValueError('Duplicate trial/instance rows')
    return keyed


def compare_rows(live, replay, tolerance=PARITY_ATOL_M):
    if set(live) != set(replay):
        return {'passed': False, 'error': 'Trial/instance keys differ'}
    mismatches = []
    exact = ('repeat_index', 'scene_id', 'outcome', 'miss_reason',
             'usable_aligned_events', 'frames_captured')
    numeric = ('true_forward_m', 'true_lateral_m', 'true_distance_m',
               'trial_estimate_m', 'abs_error_m', 'rel_error')
    for key, row in live.items():
        other = replay[key]
        for column in exact:
            if row[column] != other[column]:
                mismatches.append([*key, column, row[column], other[column]])
        for column in numeric:
            a, b = row[column], other[column]
            if a == b:
                continue
            try:
                x, y = float(a), float(b)
                same = (math.isnan(x) and math.isnan(y)) or math.isclose(x, y, abs_tol=tolerance, rel_tol=0)
            except ValueError:
                same = False
            if not same:
                mismatches.append([*key, column, a, b])
    return {'passed': not mismatches, 'rows': len(live), 'mismatch_count': len(mismatches),
            'first_mismatches': mismatches[:20], 'absolute_tolerance': tolerance,
            'scope': 'trial/instance CSV parity; not per-event live-output parity'}


def repeatability(first, second):
    changes, differences = 0, []
    if set(first) != set(second):
        raise ValueError('Repetition trial/instance keys differ')
    for key, a in first.items():
        b = second[key]
        changes += (a['outcome'], a['miss_reason']) != (b['outcome'], b['miss_reason'])
        try:
            x, y = float(a['trial_estimate_m']), float(b['trial_estimate_m'])
            if math.isfinite(x) and math.isfinite(y):
                differences.append(abs(x-y))
        except ValueError:
            pass
    return {'rows': len(first), 'outcome_or_reason_changes': changes,
            'paired_numeric_rows': len(differences),
            'median_abs_estimate_delta_m': statistics.median(differences) if differences else None,
            'max_abs_estimate_delta_m': max(differences) if differences else None,
            'verdict': 'descriptive; scientific acceptance margins are not yet frozen'}
