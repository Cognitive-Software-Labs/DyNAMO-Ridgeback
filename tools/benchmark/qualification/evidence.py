"""Fail-closed evidence checks and comparisons; no ROS or model initialization."""
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import subprocess

ESTIMATORS = ('projective_ranging', 'euclidean_reconstruction', 'polar_profiling')
CONDITIONS = ('box_stereo', 'silhouette_stereo', 'box_monocular', 'silhouette_monocular')
# The short readiness checks: one stereo and both monocular mask paths.
CHECK_CONDITIONS = ('box_stereo', 'box_monocular', 'silhouette_monocular')
SAMPLE_BATCHES = 5
PARITY_ATOL_M = 0.0001


def read_json(path):
    return json.loads(Path(path).read_text())


def capture_windows(path, run_label, expected_trials, seconds, beyond_sample=False):
    records = [json.loads(line) for line in Path(path).read_text().splitlines()]
    selected = [row for row in records if row['run_label'] == run_label]
    if len(selected) != expected_trials or len({r['trial_id'] for r in selected}) != expected_trials:
        raise ValueError('Missing or duplicated capture-window timing records')
    if any(not row['trial_id'] for row in selected):
        raise ValueError('A capture-window record carries no trial identity')
    if any(row.get('measurement_sec') is None for row in selected):
        raise ValueError('No sampled-window evidence; quota capture is insufficient')
    # Diagnostic execution tolerance, not a scientific equivalence margin. The
    # measurement window alone is timed; the sample's drain is reported apart.
    if any(abs(row['measurement_sec'] - seconds) > .5 for row in selected):
        raise ValueError('Capture did not follow the requested wall-time window; quota capture is insufficient')
    if any(not row.get('total_events') for row in selected):
        raise ValueError('A capture window received no measurement batches, including empty batches')
    for row in selected:
        if row['requested_samples'] != SAMPLE_BATCHES or row['actual_samples'] > SAMPLE_BATCHES:
            raise ValueError(f"{row['trial_id']}: sample is not the bounded {SAMPLE_BATCHES}-batch sample")
        if row['sample_coverage'] != 'complete':
            raise ValueError(f"{row['trial_id']}: insufficient sample coverage "
                             f"({row['actual_samples']}/{row['requested_samples']})")
        # A check proves storage stops at the quota while measurement goes on.
        if beyond_sample and row['live_raw_batches'] <= row['actual_samples']:
            raise ValueError(f"{row['trial_id']}: live window did not continue beyond the sample")
    return selected


def depth_contract_error(event, depth_source):
    """Why an event's frozen depth lacks the live source's own range, if it does."""
    if event.get('depth_m') is None:
        return None
    declared = event.get('depth_source') or {}
    if declared.get('depth_source') != depth_source:
        return 'no matching depth-source metadata'
    ceiling = event.get('depth_usable_max_m')
    if depth_source == 'monocular':
        if ceiling is None or not math.isfinite(ceiling) or ceiling != declared.get('usable_max_m'):
            return 'no finite usable ceiling from the loaded checkpoint'
        if not declared.get('model_id') or not declared.get('declared_max_depth_m'):
            return 'no checkpoint identity behind the ceiling'
    elif ceiling is not None:
        return 'stereo depth carries a ceiling it never declared'
    return None


def sensor_completeness(path, depth_source, expected_trials, windows=None):
    from ridgeback_autonomy.benchmarking.replay_artifacts import load_artifact, load_sensor_trial
    from ridgeback_autonomy.benchmarking.sampled_capture import (
        live_record_expects_depth, live_record_expects_scan, live_record_matches_raw_batch,
    )
    artifact = load_artifact(path, require_parent=False)
    if artifact.kind != 'sensor-capture' or len(artifact.trial_entries) != expected_trials:
        raise ValueError('Wrong sensor artifact kind or trial count')
    parameters = artifact.manifest['producer'].get('parameters', {})
    if str(parameters.get('depth_source')) != depth_source:
        raise ValueError('Sensor parent does not preserve the requested depth source')
    required = ('rgb', 'depth_m', 'intrinsics', 'camera_rotation', 'camera_translation',
                'scan', 'scan_rotation', 'scan_translation')
    total = detections = empty = without_live = 0
    explicit_missing = {key: 0 for key in required}
    ceilings = set()
    for entry in artifact.trial_entries:
        _trial, events = load_sensor_trial(artifact, entry)
        if not events:
            raise ValueError(f"No events in {entry['trial_id']}")
        if windows is not None:
            window = windows.get(entry['trial_id'])
            if window is None or [e['stamp_ns'] for e in events] != window['sampled_stamps_ns']:
                raise ValueError(f"{entry['trial_id']}: stored events differ from the sampled stamps")
            if window.get('payload_sha256') != entry['sha256']:
                raise ValueError(f"{entry['trial_id']}: sampled reference names another payload")
        for event in events:
            missing = [key for key in required if event.get(key) is None]
            for key in missing:
                explicit_missing[key] += 1
            live = event.get('live_measurement')
            if windows is not None and live_record_expects_scan(live):
                chosen = event.get('live_scan_stamp_ns')
                if chosen is None or (event.get('scan') or {}).get('stamp_ns') != chosen:
                    raise ValueError(f"{entry['trial_id']} stamp {event['stamp_ns']}: "
                                     'saved scan does not match the live polar selection')
            # Depth exists only where the live node produced it: never on a batch
            # it skipped (it keeps only the newest pending batch, e.g. while a
            # model loads) nor where its result says no depth source ran. The
            # recorder then matches live by holding none, explicitly.
            if (windows is not None and 'depth_m' in missing
                    and (live is None or not live_record_expects_depth(live))):
                missing.remove('depth_m')
            if missing and event['detections']:
                raise ValueError(f"{entry['trial_id']} stamp {event['stamp_ns']} missing {missing}")
            error = depth_contract_error(event, depth_source)
            if error:
                raise ValueError(f"{entry['trial_id']} stamp {event['stamp_ns']}: {error}")
            if event.get('depth_m') is not None:
                ceilings.add(event.get('depth_usable_max_m'))
            if windows is not None:
                if live is None:
                    without_live += 1
                elif not live_record_matches_raw_batch(live, event):
                    raise ValueError(f"{entry['trial_id']} stamp {event['stamp_ns']}: "
                                     'live record measured another batch')
            empty += not event['detections']
            if (event['image_width'], event['image_height']) != (640, 480):
                raise ValueError('Unexpected camera dimensions')
            total += 1
            detections += len(event['detections'])
    return {'artifact_id': artifact.id, 'events': total, 'detections': detections,
            'empty_detection_batches': empty, 'explicit_missing_inputs': explicit_missing,
            'events_without_live_record': without_live,
            'depth_usable_max_m': sorted(ceilings, key=lambda value: (value is None, value or 0)),
            'manifest_sha256': artifact.manifest_sha256}


def sampled_reference(run_dir, trial_ids):
    """The run's sampled live reference: present, hashed, and covering its trials."""
    root = Path(run_dir) / 'sampled_reference'
    document = read_json(root / 'sampled_windows.json')
    if document.get('schema') != 'ridgeback-sampled-windows/1':
        raise ValueError('Unknown sampled-reference schema')
    windows = {window['trial_id']: window for window in document['windows']}
    if set(windows) != set(trial_ids) or len(windows) != len(document['windows']):
        raise ValueError('Sampled reference trials differ from the completed run')
    summary = document['summary']
    for name, checksum in summary['csv_sha256'].items():
        if sha256(root / name) != checksum:
            raise ValueError(f'Sampled reference CSV changed: {name}')
    for estimator in ESTIMATORS:
        if {key[0] for key in csv_rows(root, estimator)} != set(trial_ids):
            raise ValueError('Sampled reference rows differ from the completed run')
    return {'summary': summary, 'windows': windows}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


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


def validate_run(run_dir, entry, expected_trials, timing_path, beyond_sample=False):
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
                              float(parameters['capture_sec']), beyond_sample)
    for estimator in ESTIMATORS:
        rows = csv_rows(run_dir, estimator)
        if len(rows) != run['instances'] or {key[0] for key in rows} != {w['trial_id'] for w in windows}:
            raise ValueError('CSV trial/instance accounting differs from the completed run')
    video = run_dir / 'video/run.mp4'
    result = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration',
                             '-of', 'json', str(video)], capture_output=True, text=True, check=True)
    if float(json.loads(result.stdout)['format']['duration']) <= 0:
        raise ValueError('Video is empty')
    if parameters.get('sensor_capture_mode') != 'sampled_window' or not parameters.get('sensor_capture_dir'):
        raise ValueError('Every condition must store a sampled-window sensor capture')
    reference = sampled_reference(run_dir, {w['trial_id'] for w in windows})
    capture = sensor_completeness(parameters['sensor_capture_dir'], parameters['depth_source'],
                                  expected_trials, reference['windows'])
    return {'run_json': str(run_dir / 'run.json'), 'capture': capture,
            'sampled_reference': reference['summary'],
            'window_events': sum(row['total_events'] or 0 for row in windows),
            'live_raw_batches': sum(row['live_raw_batches'] for row in windows),
            'sampled_batches': sum(row['actual_samples'] for row in windows),
            'measurement_sec': [min(r['measurement_sec'] for r in windows),
                                max(r['measurement_sec'] for r in windows)],
            'drain_sec_max': max(row['drain_sec'] for row in windows),
            'estimators': document['estimators']}


def sampled_identity(sensor_path, run_dir):
    """Event identity of a stored sample against its live reference, before any numbers.

    A sampled batch the live node never measured (its worker keeps only the
    newest pending batch) has no live record: its trial cannot be compared and
    is named, never dropped silently or counted as a pass.
    """
    from ridgeback_autonomy.benchmarking.replay_artifacts import load_artifact, load_sensor_trial
    from ridgeback_autonomy.benchmarking.sampled_capture import live_record_matches_raw_batch
    artifact = load_artifact(sensor_path, require_parent=False)
    windows = sampled_reference(run_dir, [e['trial_id'] for e in artifact.trial_entries])['windows']
    problems, inconclusive = [], {}
    events_total = detections_total = 0
    for entry in artifact.trial_entries:
        trial_id = entry['trial_id']
        _trial, events = load_sensor_trial(artifact, entry)
        window = windows[trial_id]
        if [e['stamp_ns'] for e in events] != window['sampled_stamps_ns']:
            problems.append([trial_id, 'stored stamps differ from the sampled stamps'])
        if window.get('payload_sha256') != entry['sha256']:
            problems.append([trial_id, 'sampled reference names another payload'])
        for event in events:
            events_total += 1
            detections_total += len(event['detections'])
            live = event.get('live_measurement')
            if live is None:
                inconclusive.setdefault(trial_id, []).append(event['stamp_ns'])
            elif not live_record_matches_raw_batch(live, event):
                problems.append([trial_id, f"stamp {event['stamp_ns']}: live record measured another batch"])
    return {'passed': not problems, 'problems': problems[:20], 'problem_count': len(problems),
            'artifact_id': artifact.id, 'manifest_sha256': artifact.manifest_sha256,
            'trials': len(artifact.trial_entries), 'events': events_total,
            'detections': detections_total, 'inconclusive_trials': inconclusive}


def _finite(value):
    return None if value is None or not math.isfinite(float(value)) else float(value)


def event_parity(sensor_path, cache_path, replay_dir, tolerance=PARITY_ATOL_M):
    """Replay each live-measured sampled batch and compare it with its own live record.

    Finer than trial parity and unaffected by batches the live node skipped:
    those have no record and are counted, not compared. Uses the exact variant
    arguments the numeric replay recorded, so both views measure the same thing.
    """
    from ridgeback_autonomy.benchmarking.layered_replay import replay_sensor_event
    from ridgeback_autonomy.benchmarking.replay_artifacts import (
        load_artifact, load_mask_trial, load_sensor_trial, trial_entry_map,
    )
    from ridgeback_localization.estimator_registry import (
        ESTIMATOR_FIELD_KEYS, ESTIMATOR_POSITION_ATTRS, ESTIMATOR_STATUS_FIELD_KEYS,
    )
    arguments = read_json(Path(replay_dir)/'run.json')['parameters']
    selected = tuple(arguments['estimators'].split(','))
    sensor = load_artifact(sensor_path, require_parent=False)
    cache = load_artifact(cache_path, parent=sensor)
    cache_entries = trial_entry_map(cache)
    compared = unmeasured = detections = 0
    mismatches, worst = [], {estimator: 0.0 for estimator in selected}
    # Polar events whose live row read another scan than the frozen nearest one.
    other_scan = set()
    for entry in sensor.trial_entries:
        _trial, events = load_sensor_trial(sensor, entry)
        masks = load_mask_trial(cache, cache_entries[entry['trial_id']])
        for event, outcomes in zip(events, masks):
            live = event.get('live_measurement')
            if live is None:
                unmeasured += 1
                continue
            compared += 1
            replayed = replay_sensor_event(event, outcomes, arguments, selected)
            where = [entry['trial_id'], event['stamp_ns']]
            stored_scan = (event.get('scan') or {}).get('stamp_ns')
            live_scan = event.get('live_scan_stamp_ns')
            if live_scan is not None and live_scan != stored_scan:
                other_scan.add(tuple(where))
            if len(replayed.detections) != len(live['detections']):
                mismatches.append([*where, 'detection count'])
                continue
            for index, (expected, detection) in enumerate(zip(live['detections'], replayed.detections)):
                detections += 1
                for estimator in selected:
                    values = expected['estimators'][estimator]
                    status = getattr(detection, ESTIMATOR_STATUS_FIELD_KEYS[estimator])
                    if values['status'] != status:
                        mismatches.append([*where, index, estimator, 'status', values['status'], status])
                    forward, lateral = ESTIMATOR_POSITION_ATTRS[estimator]
                    for name, attribute in (('distance_m', ESTIMATOR_FIELD_KEYS[estimator]),
                                            ('forward_m', forward), ('lateral_m', lateral)):
                        a, b = _finite(values[name]), _finite(getattr(detection, attribute))
                        if (a is None) != (b is None):
                            mismatches.append([*where, index, estimator, name, a, b])
                        elif a is not None:
                            worst[estimator] = max(worst[estimator], abs(a - b))
                            if abs(a - b) > tolerance:
                                mismatches.append([*where, index, estimator, name, a, b])
    verdict = 'failed' if mismatches else 'passed' if compared else 'inconclusive'
    polar = {tuple(item[:2]) for item in mismatches if 'polar_profiling' in item}
    return {'verdict': verdict, 'compared_events': compared, 'compared_detections': detections,
            # Attribution, not exemption: these still fail the comparison.
            'polar_mismatch_events': len(polar),
            'polar_mismatch_events_on_another_live_scan': len(polar & other_scan),
            'events_on_another_live_scan': len(other_scan),
            'events_without_live_record': unmeasured, 'mismatch_count': len(mismatches),
            'first_mismatches': mismatches[:20], 'max_abs_delta_m': worst,
            'absolute_tolerance': tolerance,
            'scope': 'per-event live-output parity over the live-measured sampled batches'}


def without_trials(rows, trial_ids):
    return {key: row for key, row in rows.items() if key[0] not in trial_ids}


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
