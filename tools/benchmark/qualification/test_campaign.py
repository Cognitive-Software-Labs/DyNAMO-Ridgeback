"""No ROS graph, simulator, hardware or model execution in these tests."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import campaign
import evidence


def write_windows(path, seconds=10, events=4, count=1, **sampled):
    window = {'measurement_sec': seconds, 'drain_sec': .2, 'live_raw_batches': 90,
              'requested_samples': 5, 'actual_samples': 5, 'sample_coverage': 'complete',
              'complete': True}
    window.update(sampled)
    path.write_text('\n'.join(json.dumps({'run_label': 'r', 'trial_id': f't{i}',
        'wall_sec': seconds + window['drain_sec'], 'total_events': events, **window}) for i in range(count)))


def test_capture_quota_and_missing_heartbeat_rejected(tmp_path):
    path = tmp_path/'windows.jsonl'
    write_windows(path, seconds=2.4)
    with pytest.raises(ValueError, match='quota'):
        evidence.capture_windows(path, 'r', 1, 10)
    write_windows(path, events=0)
    with pytest.raises(ValueError, match='no measurement'):
        evidence.capture_windows(path, 'r', 1, 10)
    write_windows(path)
    assert len(evidence.capture_windows(path, 'r', 1, 10)) == 1


def test_measurement_window_is_timed_apart_from_the_sample_drain(tmp_path):
    path = tmp_path/'windows.jsonl'
    write_windows(path, drain_sec=2.0)
    assert evidence.capture_windows(path, 'r', 1, 10)[0]['wall_sec'] > 10.5
    write_windows(path, actual_samples=3, sample_coverage='insufficient')
    with pytest.raises(ValueError, match='insufficient sample coverage'):
        evidence.capture_windows(path, 'r', 1, 10)
    write_windows(path, requested_samples=8)
    with pytest.raises(ValueError, match='bounded'):
        evidence.capture_windows(path, 'r', 1, 10)
    write_windows(path, live_raw_batches=5)
    assert evidence.capture_windows(path, 'r', 1, 10)
    with pytest.raises(ValueError, match='beyond the sample'):
        evidence.capture_windows(path, 'r', 1, 10, beyond_sample=True)
    path.write_text(json.dumps({'run_label': 'r', 'trial_id': 't', 'wall_sec': 10,
                                'total_events': 4}))
    with pytest.raises(ValueError, match='sampled-window'):
        evidence.capture_windows(path, 'r', 1, 10)
    path.write_text(json.dumps({'run_label': 'r', 'trial_id': '', 'wall_sec': 10,
                                'total_events': 4}))
    with pytest.raises(ValueError, match='trial identity'):
        evidence.capture_windows(path, 'r', 1, 10)


def test_duplicate_or_missing_trial_window_rejected(tmp_path):
    path = tmp_path/'windows.jsonl'
    write_windows(path)
    with pytest.raises(ValueError, match='Missing or duplicated'):
        evidence.capture_windows(path, 'r', 109, 10)
    path.write_text(path.read_text()+'\n'+path.read_text())
    with pytest.raises(ValueError, match='Missing or duplicated'):
        evidence.capture_windows(path, 'r', 2, 10)


def test_capture_missing_context_is_not_silently_dropped(monkeypatch, tmp_path):
    from types import SimpleNamespace
    import ridgeback_autonomy.benchmarking.replay_artifacts as artifacts
    artifact = SimpleNamespace(kind='sensor-capture', trial_entries=[{'trial_id': 't'}],
        manifest={'producer': {'parameters': {'depth_source': 'stereoscopic'}}}, id='id', manifest_sha256='hash')
    event = dict.fromkeys(('rgb','depth_m','intrinsics','camera_rotation','camera_translation',
                          'scan','scan_rotation','scan_translation'), object())
    event.update(stamp_ns=1, image_width=640, image_height=480, detections=[{}],
                 depth_source={'depth_source': 'stereoscopic', 'usable_max_m': None},
                 depth_usable_max_m=None)
    monkeypatch.setattr(artifacts, 'load_artifact', lambda *a, **k: artifact)
    monkeypatch.setattr(artifacts, 'load_sensor_trial', lambda *a: ({}, [event]))
    assert evidence.sensor_completeness(tmp_path, 'stereoscopic', 1)['events'] == 1
    event['rgb'] = None
    with pytest.raises(ValueError, match='rgb'):
        evidence.sensor_completeness(tmp_path, 'stereoscopic', 1)
    event['detections'] = []
    result = evidence.sensor_completeness(tmp_path, 'stereoscopic', 1)
    assert result['empty_detection_batches'] == 1 and result['explicit_missing_inputs']['rgb'] == 1
    with pytest.raises(ValueError, match='depth source'):
        evidence.sensor_completeness(tmp_path, 'monocular', 1)


def test_depth_absent_where_live_never_produced_it_is_kept_explicit(monkeypatch, tmp_path):
    from types import SimpleNamespace
    import ridgeback_autonomy.benchmarking.replay_artifacts as artifacts
    artifact = SimpleNamespace(kind='sensor-capture', trial_entries=[{'trial_id': 't', 'sha256': 'p'}],
        manifest={'producer': {'parameters': {'depth_source': 'monocular'}}}, id='id', manifest_sha256='hash')
    detection = {'bbox_xyxy': [1, 1, 3, 3]}
    event = dict.fromkeys(('rgb', 'intrinsics', 'camera_rotation', 'camera_translation',
                           'scan', 'scan_rotation', 'scan_translation'), object())
    event.update(stamp_ns=1, image_width=640, image_height=480, detections=[detection],
                 depth_m=None, depth_source=None, depth_usable_max_m=None, live_measurement=None)
    windows = {'t': {'sampled_stamps_ns': [1], 'payload_sha256': 'p'}}
    monkeypatch.setattr(artifacts, 'load_artifact', lambda *a, **k: artifact)
    monkeypatch.setattr(artifacts, 'load_sensor_trial', lambda *a: ({}, [event]))
    # Never measured live (the node was loading its model): explicit, not a failure.
    result = evidence.sensor_completeness(tmp_path, 'monocular', 1, windows)
    assert result['events_without_live_record'] == 1
    assert result['explicit_missing_inputs']['depth_m'] == 1
    # Measured live without depth: the live result says why, and it matches.
    status = {'status': 20}
    event['live_measurement'] = {'stamp_ns': 1, 'detections': [dict(detection, estimators={
        'projective_ranging': status, 'euclidean_reconstruction': status})]}
    assert evidence.sensor_completeness(tmp_path, 'monocular', 1, windows)['events_without_live_record'] == 0
    # Measured live WITH depth, yet none stored: the capture lost it.
    status['status'] = 0
    with pytest.raises(ValueError, match='depth_m'):
        evidence.sensor_completeness(tmp_path, 'monocular', 1, windows)


def test_sampled_capture_rejects_a_different_or_unknown_live_polar_scan(monkeypatch, tmp_path):
    from types import SimpleNamespace
    import ridgeback_autonomy.benchmarking.replay_artifacts as artifacts
    artifact = SimpleNamespace(kind='sensor-capture', trial_entries=[{'trial_id': 't', 'sha256': 'p'}],
        manifest={'producer': {'parameters': {'depth_source': 'stereoscopic'}}}, id='id', manifest_sha256='hash')
    detection = {'bbox_xyxy': [1, 1, 3, 3]}
    event = dict.fromkeys(('rgb', 'intrinsics', 'camera_rotation', 'camera_translation',
                          'scan_rotation', 'scan_translation'), object())
    event.update(stamp_ns=1, image_width=640, image_height=480, detections=[detection],
        depth_m=None, depth_source=None, depth_usable_max_m=None, scan={'stamp_ns': 10},
        live_scan_stamp_ns=9, live_measurement={'stamp_ns': 1, 'detections': [
            dict(detection, estimators={'polar_profiling': {'status': 0}})]})
    windows = {'t': {'sampled_stamps_ns': [1], 'payload_sha256': 'p'}}
    monkeypatch.setattr(artifacts, 'load_artifact', lambda *a, **k: artifact)
    monkeypatch.setattr(artifacts, 'load_sensor_trial', lambda *a: ({}, [event]))
    for chosen in (9, None):
        event['live_scan_stamp_ns'] = chosen
        with pytest.raises(ValueError, match='live polar selection'):
            evidence.sensor_completeness(tmp_path, 'stereoscopic', 1, windows)
    event['live_scan_stamp_ns'] = 10
    assert evidence.sensor_completeness(tmp_path, 'stereoscopic', 1, windows)['events'] == 1


def test_frozen_depth_must_carry_its_live_source_range():
    stereo = {'depth_m': object(), 'depth_usable_max_m': None,
              'depth_source': {'depth_source': 'stereoscopic', 'usable_max_m': None}}
    assert evidence.depth_contract_error(stereo, 'stereoscopic') is None
    assert 'never declared' in evidence.depth_contract_error(
        dict(stereo, depth_usable_max_m=6.0), 'stereoscopic')
    assert 'metadata' in evidence.depth_contract_error(dict(stereo, depth_source=None), 'stereoscopic')
    mono = {'depth_m': object(), 'depth_usable_max_m': 18.0, 'depth_source': {
        'depth_source': 'monocular', 'usable_max_m': 18.0, 'model_id': 'm',
        'declared_max_depth_m': 20.0}}
    assert evidence.depth_contract_error(mono, 'monocular') is None
    assert 'finite' in evidence.depth_contract_error(dict(mono, depth_usable_max_m=None), 'monocular')
    assert 'metadata' in evidence.depth_contract_error(mono, 'stereoscopic')
    assert evidence.depth_contract_error(dict(mono, depth_m=None, depth_source=None), 'monocular') is None


def row(estimate='1.0', outcome='scored'):
    result = {k: '0' for k in ('true_forward_m','true_lateral_m','true_distance_m','abs_error_m','rel_error')}
    result.update(repeat_index='1', scene_id='s', outcome=outcome, miss_reason='',
                  usable_aligned_events='5', frames_captured='5', trial_estimate_m=estimate)
    return result


def test_replay_identity_outcomes_and_values_are_checked():
    original = {('t','0'): row()}
    assert evidence.compare_rows(original, original)['passed']
    assert not evidence.compare_rows(original, {('other','0'): row()})['passed']
    assert not evidence.compare_rows(original, {('t','0'): row(outcome='detector_miss')})['passed']
    assert not evidence.compare_rows(original, {('t','0'): row('1.01')})['passed']
    assert evidence.compare_rows(original, {('t','0'): row('1.00001')})['passed']
    assert not evidence.compare_rows(original, {('t','0'): row('1.00001')}, tolerance=0)['passed']


def test_altered_status_and_association_are_detected():
    original = {('t', '0'): row('1.0'), ('t', '1'): row('2.0')}
    assert not evidence.compare_rows(original, {
        ('t', '0'): dict(row('1.0'), miss_reason='NO_DEPTH_FRAME'), ('t', '1'): row('2.0')})['passed']
    # The same two estimates associated with the other targets.
    assert not evidence.compare_rows(original, {('t', '0'): row('2.0'), ('t', '1'): row('1.0')})['passed']


def test_expected_misses_are_descriptive_not_execution_failures():
    first = {('t','0'): row('nan', 'detector_miss')}
    second = {('t','0'): row('1.0')}
    assert evidence.repeatability(first, second)['outcome_or_reason_changes'] == 1
    assert evidence.repeatability(first, second)['paired_numeric_rows'] == 0


def test_frozen_input_edits_and_new_files_rejected(tmp_path, monkeypatch):
    original = {'a':'hash'}
    campaign.atomic_json(tmp_path/'frozen.json', {'files': original})
    monkeypatch.setattr(campaign, 'frozen_files', lambda _: original)
    campaign.check_frozen(tmp_path)
    monkeypatch.setattr(campaign, 'frozen_files', lambda _: {'a':'different', 'new':'hash'})
    with pytest.raises(RuntimeError, match='Experimental files changed'):
        campaign.check_frozen(tmp_path)


def test_prepare_has_separate_repetitions_with_equal_capture_workloads(tmp_path):
    bundle = campaign.prepare(tmp_path/'bundle')
    import yaml
    for name in campaign.ORDER:
        spec = yaml.safe_load((bundle/'inputs'/f'{name}.yaml').read_text())
        assert [c['name'] for c in spec['configs']] == list(evidence.CONDITIONS)
        assert spec['defaults']['repeats'] == 1
        assert spec['defaults']['capture_sec'] == 10.0
        assert spec['defaults']['output_dir'].endswith(name)
        assert all(c['sensor_capture_dir'].endswith(name+'/'+c['name']) for c in spec['configs'])
        assert all(c['sensor_capture_mode'] == 'sampled_window' and c['capture_batches'] == 5
                   for c in spec['configs'])
    for backend in ('gz', 'isaac'):
        spec = yaml.safe_load((bundle/'inputs'/f'check_{backend}.yaml').read_text())
        assert [(c['name'], c['capture_sec']) for c in spec['configs']] == [
            ('box_stereo', 4.0), ('box_monocular', 6.0), ('silhouette_monocular', 6.0)]
        assert all(c['sensor_capture_mode'] == 'sampled_window' for c in spec['configs'])
    assert campaign.read_json(bundle/'campaign.json')['status'] == 'prepared-not-launched'
    assert 'pending' in (bundle/'REPORT.md').read_text()


def test_stage_failure_prevents_all_full_benchmarks(tmp_path, monkeypatch):
    bundle = campaign.prepare(tmp_path/'bundle')
    monkeypatch.setattr(campaign, 'ROOT', tmp_path)
    monkeypatch.setattr(campaign, 'host_available', lambda *_args: None)
    monkeypatch.setattr(campaign, 'sample_host', lambda *_args: None)
    monkeypatch.setattr(campaign, 'check_installed', lambda: None)
    monkeypatch.setattr(campaign, 'acquire_idle_inhibitor', lambda: None)
    monkeypatch.setattr(campaign, 'validate_matrix', lambda _: {})
    monkeypatch.setattr(campaign, 'frozen_files', lambda _: {})
    monkeypatch.setattr(campaign.subprocess, 'check_output', lambda *a, **k: '')
    calls = []
    def fail(command, *args):
        calls.append(command)
        return {'returncode': 1, 'deadline_or_stop': False, 'outer_escalated': False,
                'leftover_pids': [], 'wall_sec': .1}
    monkeypatch.setattr(campaign, 'owned_command', fail)
    assert campaign.run(bundle) == 1
    assert len(calls) == 1 and 'check_gz.yaml' in str(calls[0])
    assert campaign.read_json(bundle/'campaign.json')['status'] == 'failed'


def test_sweep_adapter_rejects_failed_condition_and_budget_exhaustion(tmp_path, monkeypatch):
    import types
    import ridgeback_autonomy.benchmarking as package
    import sweep_entry
    fake = types.SimpleNamespace()
    fake._write_manifest = lambda directory, document: campaign.atomic_json(Path(directory)/'sweep.json', document)
    fake._config_command = lambda args: ['never-spawned']
    fake._wait_for_config = lambda *args: None
    fake.main = lambda: 0
    monkeypatch.setattr(package, 'target_benchmark_sweep', fake, raising=False)
    monkeypatch.setattr(sweep_entry, 'check_frozen', lambda _: None)
    for key, value in {'QUALIFICATION_BUNDLE': str(tmp_path), 'QUALIFICATION_STAGE':'gz_r1',
                       'QUALIFICATION_STOP_AT': str(time.monotonic()+2),
                       'QUALIFICATION_ADMISSION_SEC':'2100'}.items():
        monkeypatch.setenv(key, value)
    assert sweep_entry.main() == 0
    with pytest.raises(RuntimeError, match='Insufficient'):
        fake._config_command({})
    manifest = {'configs':[{'name':'box_stereo','status':'failed'}]}
    with pytest.raises(RuntimeError, match='stop rather than continue'):
        fake._write_manifest(tmp_path, manifest)
    assert campaign.read_json(tmp_path/'sweep.json') == manifest
    assert campaign.read_json(tmp_path/'gz_r1_guard.json')['error']


def test_owned_process_failure_is_retained(tmp_path):
    result = campaign.owned_command([sys.executable, '-c', 'raise SystemExit(7)'],
        tmp_path/'log', os.environ.copy(), time.monotonic()+3)
    assert result['returncode'] == 7 and not result['leftover_pids']


def test_deadline_stops_process_without_other_process_cleanup(tmp_path):
    result = campaign.owned_command([sys.executable, '-c', 'import time; time.sleep(20)'],
        tmp_path/'log', os.environ.copy(), time.monotonic()+.3)
    assert result['deadline_or_stop'] and not result['leftover_pids']
    assert result['wall_sec'] < 5


def test_orphan_in_new_session_remains_owned(tmp_path):
    code = ('import subprocess,sys,time; '
            'subprocess.Popen([sys.executable,"-c","import time;time.sleep(20)"],start_new_session=True); '
            'time.sleep(.4)')
    result = campaign.owned_command([sys.executable, '-c', code],
        tmp_path/'log', os.environ.copy(), time.monotonic()+3)
    assert not result['leftover_pids'] and len(result['owned_pids']) >= 2


def test_observer_does_not_change_capture_result(tmp_path):
    path = Path(__file__).parent/'observer/sitecustomize.py'
    # Like the runner, the window clears the trial identity when it closes.
    code = '''class TargetDistanceBenchmarkRunner:
 def __init__(self): self.run_label='r'; self.active_trial_id='t'
 def capture_measurement_window(self, seconds):
  self.active_trial_id=''
  return {'total_events':2,'usable_events':[],'sampled_window':{'measurement_sec':4.01,
   'drain_sec':.3,'live_raw_batches':40,'requested_samples':5,'actual_samples':5,
   'sample_coverage':'complete','complete':True}}
result=TargetDistanceBenchmarkRunner().capture_measurement_window(4)
assert result['total_events']==2 and result['usable_events']==[]
'''
    import subprocess
    env = dict(os.environ, PYTHONPATH=str(path.parent), QUALIFICATION_CAPTURE_TIMINGS=str(tmp_path/'timings'))
    subprocess.run([sys.executable, '-c', code], env=env, check=True)
    record = json.loads((tmp_path/'timings').read_text())
    assert record['requested_sec'] == 4 and record['trial_id'] == 't'
    assert record['measurement_sec'] == 4.01 and record['drain_sec'] == .3
    assert record['actual_samples'] == 5 and record['live_raw_batches'] == 40


def sampled_dataset(tmp_path, *, live=True, stamps=(11, 12)):
    """A sensor capture and its sampled reference, written by the runner's own code."""
    from ridgeback_autonomy.benchmarking import target_distance_benchmark_runner_node as module
    from ridgeback_autonomy.benchmarking.replay_artifacts import SensorCaptureWriter
    writer = SensorCaptureWriter(tmp_path/'sensor', producer={'parameters': {'depth_source': 'monocular'}})
    events = []
    for stamp in stamps:
        detection = {'bbox_xyxy': [1, 1, 3, 3], 'label': 'target', 'score': .9}
        events.append({'stamp_ns': stamp, 'frame_id': 'camera', 'detected': True, 'count': 1,
                       'image_width': 4, 'image_height': 3, 'detections': [detection],
                       'live_measurement': None if not live else {
                           'stamp_ns': stamp, 'detections': [dict(detection, estimators={})]}})
    writer.write_trial({'trial_id': 't'}, events)
    run_dir = tmp_path/'run'
    run_dir.mkdir()
    runner = object.__new__(module.TargetDistanceBenchmarkRunner)
    rows = {key: [dict(row(), trial_id='t', instance_index='0')] for key in evidence.ESTIMATORS}
    runner.__dict__.update(
        sensor_capture_mode='sampled_window', sensor_writer=writer, run_output_dir=str(run_dir),
        selected_estimators=evidence.ESTIMATORS, sampled_reference_rows=rows,
        estimator_output_names={key: f'{key}_box_monocular' for key in evidence.ESTIMATORS},
        sampled_window_records=[{'trial_id': 't', 'sampled_stamps_ns': list(stamps),
            'measurement_sec': 10.0, 'drain_sec': .1, 'actual_samples': len(stamps),
            'sample_coverage': 'complete', 'complete': True, 'missing': {'live_record': 0}}],
        sensor_capture_dir=str(tmp_path/'sensor'), capture_batches=5, run_label='r')
    summary = runner.write_sampled_reference()
    writer.finalize(sampled_reference=summary)
    return tmp_path/'sensor', run_dir


def test_sampled_identity_checks_stamps_before_numbers(tmp_path):
    sensor, run_dir = sampled_dataset(tmp_path)
    identity = evidence.sampled_identity(sensor, run_dir)
    assert identity['passed'] and identity['events'] == 2 and not identity['inconclusive_trials']
    windows = run_dir/'sampled_reference/sampled_windows.json'
    document = json.loads(windows.read_text())
    document['windows'][0]['sampled_stamps_ns'] = [11, 13]
    windows.write_text(json.dumps(document))
    assert not evidence.sampled_identity(sensor, run_dir)['passed']
    reference = run_dir/'sampled_reference'
    next(reference.glob('projective_ranging*.csv')).write_text('trial_id\n')
    with pytest.raises(ValueError, match='CSV changed'):
        evidence.sampled_reference(run_dir, ['t'])


def test_sampled_batch_without_a_live_record_is_named_inconclusive(tmp_path):
    sensor, run_dir = sampled_dataset(tmp_path, live=False)
    identity = evidence.sampled_identity(sensor, run_dir)
    assert identity['passed'] and identity['inconclusive_trials'] == {'t': [11, 12]}
    rows = {('t', '0'): row(), ('u', '0'): row()}
    assert set(evidence.without_trials(rows, {'t'})) == {('u', '0')}


def live_complete_bundle(tmp_path, monkeypatch, live_elapsed=3600.0):
    bundle = campaign.prepare(tmp_path/'bundle')
    state = campaign.read_json(bundle/'campaign.json')
    state.update(status=campaign.LIVE_COMPLETE, phases={'live': {'elapsed_sec': live_elapsed}})
    campaign.atomic_json(bundle/'campaign.json', state)
    monkeypatch.setattr(campaign, 'ROOT', tmp_path)
    for name in ('host_available', 'check_frozen', 'verify_inventory'):
        monkeypatch.setattr(campaign, name, lambda *_args: None)
    monkeypatch.setattr(campaign, 'acquire_idle_inhibitor', lambda: None)
    return bundle


def test_deferred_offline_uses_only_the_remaining_budget(tmp_path, monkeypatch):
    bundle = live_complete_bundle(tmp_path, monkeypatch)
    seen = {}
    def offline(path, state, deadline):
        seen.update(deadline=deadline-time.monotonic(), status=state['status'])
    monkeypatch.setattr(campaign, 'offline', offline)
    assert campaign.run_offline(bundle) == 0
    expected = campaign.TOTAL_SEC - 3600 - campaign.CLEANUP_SEC
    assert expected - 5 < seen['deadline'] <= expected
    state = campaign.read_json(bundle/'campaign.json')
    assert state['status'] == 'execution-complete-review-required'
    assert set(state['phases']) == {'live', 'offline'}
    assert state['budget_remaining_sec'] <= campaign.TOTAL_SEC - 3600
    # Neither phase can run again: no repeated live condition, no second offline.
    with pytest.raises(RuntimeError, match='already launched'):
        campaign.run(bundle)
    with pytest.raises(RuntimeError, match='Offline processing needs'):
        campaign.run_offline(bundle)


def test_offline_refuses_without_a_completed_live_phase_or_budget(tmp_path, monkeypatch):
    bundle = campaign.prepare(tmp_path/'bundle')
    with pytest.raises(RuntimeError, match='Offline processing needs'):
        campaign.run_offline(bundle)
    bundle = live_complete_bundle(tmp_path/'spent', monkeypatch, live_elapsed=campaign.TOTAL_SEC-60)
    with pytest.raises(RuntimeError, match='No workstation budget'):
        campaign.run_offline(bundle)


def test_offline_phase_consumes_inventoried_evidence_and_spawns_no_live_stage(tmp_path, monkeypatch):
    bundle = live_complete_bundle(tmp_path, monkeypatch)
    inputs = {f'{stage}/{condition}': {'sensor_capture': str(tmp_path/stage/condition/'sensor'),
                                        'run_dir': str(tmp_path/stage/condition/'run')}
              for stage in campaign.ORDER for condition in evidence.CONDITIONS}
    campaign.atomic_json(bundle/'offline_inputs.json', inputs)
    commands = []
    def owned(command, log, env, deadline):
        commands.append(command)
        return {'returncode': 0, 'deadline_or_stop': False, 'outer_escalated': False,
                'leftover_pids': [], 'wall_sec': .1}
    monkeypatch.setattr(campaign, 'owned_command', owned)
    identities = []
    def identity(sensor, run_dir):
        identities.append((str(sensor), str(run_dir)))
        return {'passed': True, 'inconclusive_trials': {}, 'trials': 1}
    monkeypatch.setattr(campaign, 'sampled_identity', identity)
    monkeypatch.setattr(campaign, 'event_parity', lambda *_args: {'verdict': 'passed'})
    compared = []
    def rows(directory, _estimator):
        compared.append(Path(directory))
        return {('t', '0'): row()}
    monkeypatch.setattr(campaign, 'csv_rows', rows)
    state = campaign.read_json(bundle/'campaign.json')
    campaign.offline(bundle, state, time.monotonic()+3600)
    flat = [' '.join(command) for command in commands]
    assert not any('sweep_entry' in command or 'launch' in command for command in flat)
    assert len(identities) == 8 and all('_r1/' in sensor for sensor, _run in identities)
    # Parity reads the sampled reference, never the full-window CSV directory.
    references = [p for p in compared if p.name == 'sampled_reference']
    assert len(references) == 8 * len(evidence.ESTIMATORS)
    assert not any(p.name == 'run' for p in compared)
    assert all(result['identity']['passed'] for result in state['parity'].values())


def test_event_parity_compares_each_live_measured_batch_with_its_own_record(tmp_path):
    import numpy as np
    from ridgeback_autonomy.benchmarking.layered_replay import replay_sensor_event
    from ridgeback_autonomy.benchmarking.replay_artifacts import SensorCaptureWriter
    from ridgeback_autonomy.benchmarking.replay_materialization import materialize_masks
    from ridgeback_autonomy.benchmarking.sampled_capture import live_measurement_record
    height, width = 60, 80
    depth = np.full((height, width), 4.0, dtype=np.float32)
    depth[20:40, 30:50] = 2.0
    arguments = {'estimators': 'projective_ranging,euclidean_reconstruction',
                 'isolation_2d': 'nearest_mode_histogram',
                 'isolation_3d': 'height_crop_nearest_mode_band'}
    def event(stamp, live=True):
        return {'stamp_ns': stamp, 'frame_id': 'camera', 'detected': True, 'count': 1,
                'image_width': width, 'image_height': height,
                'intrinsics': {'fx': 100.0, 'fy': 100.0, 'cx': 40.0, 'cy': 30.0,
                               'width': width, 'height': height},
                'camera_rotation': [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
                'camera_translation': [0.0, 0.0, 0.0], 'front_offset_m': 0.25,
                'base_above_floor_m': 0.026, 'depth_usable_max_m': None,
                'rgb': np.zeros((height, width, 3), dtype=np.uint8), 'depth_m': depth,
                'detections': [{'bbox_xyxy': [30, 20, 50, 40], 'label': 'target', 'score': .9}],
                'live_measurement': None}
    def build(root, events):
        writer = SensorCaptureWriter(root/'sensor', producer={'parameters': {}})
        writer.write_trial({'trial_id': 't'}, events)
        sensor = writer.finalize()
        cache = materialize_masks(sensor, root/'masks', producer='box',
                                  code_provenance={'commit': 'abc', 'dirty_count': 0})
        return sensor, cache
    # The live record is what the live kernel produced; replay must reproduce it.
    first, second = event(1), event(2)
    sensor, cache = build(tmp_path/'probe', [first, second])
    from ridgeback_autonomy.benchmarking.replay_artifacts import load_mask_trial
    outcomes = load_mask_trial(cache, cache.trial_entries[0])[0]
    live = replay_sensor_event(first, outcomes, arguments, tuple(arguments['estimators'].split(',')))
    record = live_measurement_record(
        SimpleNamespace(**vars(live), key=('camera',)), arguments['estimators'].split(','))
    replay_dir = tmp_path/'replay'
    replay_dir.mkdir()
    campaign.atomic_json(replay_dir/'run.json', {'parameters': arguments})

    first['live_measurement'] = record
    sensor, cache = build(tmp_path/'match', [first, second])
    result = evidence.event_parity(sensor.root, cache.root, replay_dir)
    assert result['verdict'] == 'passed'
    assert (result['compared_events'], result['events_without_live_record']) == (1, 1)

    altered = json.loads(json.dumps(record))
    altered['detections'][0]['estimators']['projective_ranging']['status'] = 20
    first['live_measurement'] = altered
    sensor, cache = build(tmp_path/'status', [first])
    assert evidence.event_parity(sensor.root, cache.root, replay_dir)['verdict'] == 'failed'

    altered = json.loads(json.dumps(record))
    altered['detections'][0]['estimators']['euclidean_reconstruction']['distance_m'] += .001
    first['live_measurement'] = altered
    sensor, cache = build(tmp_path/'value', [first])
    assert evidence.event_parity(sensor.root, cache.root, replay_dir)['mismatch_count'] == 1

    first['live_measurement'] = None
    sensor, cache = build(tmp_path/'none', [first])
    assert evidence.event_parity(sensor.root, cache.root, replay_dir)['verdict'] == 'inconclusive'


def completed_source(tmp_path, monkeypatch, conditions=('box_stereo', 'silhouette_stereo')):
    """An earlier bundle whose gz_r1 completed and validated the given conditions."""
    import ridgeback_autonomy.benchmarking.replay_artifacts as artifacts
    source = campaign.prepare(tmp_path/'source')
    sweep = source/'gz_r1'/'20261103_000000_qualification_gz_r1'
    configs = []
    for condition in conditions:
        run_dir = sweep/condition
        (run_dir/'sampled_reference').mkdir(parents=True)
        (run_dir/'sampled_reference/sampled_windows.json').write_text('{}')
        (run_dir/'projective.csv').write_text('trial_id\n')
        campaign.atomic_json(run_dir/'run.json', {
            'run': {'commit': '4e22a20'},
            'parameters': {'sensor_capture_dir': str(source/'captures/gz_r1'/condition)}})
        configs.append({'name': condition, 'status': 'success'})
    campaign.atomic_json(sweep/'sweep.json', {'configs': configs})
    campaign.atomic_json(source/'gz_r1_guard.json', {'checked': {c: {} for c in conditions}})
    validation = {'window_events': 10898, 'live_raw_batches': 10898, 'sampled_batches': 545,
                  'measurement_sec': [10.0, 10.0], 'drain_sec_max': 0.0}
    monkeypatch.setattr(campaign, 'validate_run', lambda *_args: validation)
    monkeypatch.setattr(campaign, 'processing_diagnostics',
                        lambda *_args: {'slimsam': {'warm_n': 100}, 'depth_anything': {'warm_n': 100}})
    monkeypatch.setattr(artifacts, 'load_artifact',
                        lambda *_args, **_kwargs: SimpleNamespace(id='artifact', manifest_sha256='manifest'))
    return source, sweep


def test_import_adopts_validated_conditions_and_the_stage_runs_only_the_rest(tmp_path, monkeypatch):
    import yaml
    bundle = campaign.prepare(tmp_path/'bundle')
    source, sweep = completed_source(tmp_path, monkeypatch)
    campaign.import_conditions(bundle, source, 'gz_r1', ['box_stereo', 'silhouette_stereo'])
    state = campaign.read_json(bundle/'campaign.json')
    spec = yaml.safe_load((bundle/'inputs/gz_r1.yaml').read_text())
    assert [c['name'] for c in spec['configs']] == ['box_monocular', 'silhouette_monocular']
    assert campaign.stage_conditions(state, 'gz_r1') == ('box_monocular', 'silhouette_monocular')
    assert campaign.stage_conditions(state, 'gz_r2') == evidence.CONDITIONS
    record = state['imported']['gz_r1/box_stereo']
    assert record['source_revision'] == '4e22a20' and record['validation']['sampled_batches'] == 545
    assert campaign.condition_paths(bundle, state, 'gz_r1', 'box_stereo') == (
        sweep/'box_stereo', source/'captures/gz_r1/box_stereo')
    assert '| gz_r1/box_stereo | imported | 10898 | 545 |' in (bundle/'REPORT.md').read_text()
    # Imported evidence stays frozen: any later edit stops inventory before offline use.
    (sweep/'box_stereo/projective.csv').write_text('trial_id\nchanged\n')
    with pytest.raises(RuntimeError, match='Imported evidence changed'):
        campaign.inventory(bundle, state)


def test_import_refuses_changed_settings_unvalidated_runs_and_launched_bundles(
        tmp_path, monkeypatch):
    import yaml
    bundle = campaign.prepare(tmp_path/'bundle')
    source, _sweep = completed_source(tmp_path, monkeypatch, conditions=('box_stereo',))
    with pytest.raises(ValueError, match='did not complete'):
        campaign.import_conditions(bundle, source, 'gz_r1', ['silhouette_stereo'])
    with pytest.raises(ValueError, match='Only full-run stages'):
        campaign.import_conditions(bundle, source, 'check_gz', ['box_stereo'])
    path = source/'inputs/gz_r1.yaml'
    original = yaml.safe_load(path.read_text())
    original['defaults']['capture_sec'] = 9.0
    path.write_text(yaml.safe_dump(original, sort_keys=False))
    with pytest.raises(ValueError, match='settings differ'):
        campaign.import_conditions(bundle, source, 'gz_r1', ['box_stereo'])
    state = campaign.read_json(bundle/'campaign.json')
    assert 'imported' not in state or not state['imported']
    state['status'] = 'running-live'
    campaign.atomic_json(bundle/'campaign.json', state)
    with pytest.raises(RuntimeError, match='before launch'):
        campaign.import_conditions(bundle, source, 'gz_r1', ['box_stereo'])


def test_external_cuda_work_is_recorded_only_under_the_record_policy(tmp_path, monkeypatch):
    pmon = ('# gpu pid type sm mem enc dec jpg ofa fb ccpm command\n'
            '    0  4242  C  37  1  -  -  -  -  8000  0  python\n')
    monkeypatch.setattr(campaign.psutil, 'process_iter', lambda *_args: [])
    monkeypatch.setattr(campaign.subprocess, 'run', lambda *_args, **_kwargs: SimpleNamespace(stdout=pmon))
    free = {'bytes': 100 * 1024**3}
    monkeypatch.setattr(campaign.shutil, 'disk_usage', lambda _path: SimpleNamespace(free=free['bytes']))
    with pytest.raises(RuntimeError, match='Another CUDA process is active: PID 4242'):
        campaign.host_available('stop')
    assert campaign.host_available('record')['active_cuda'] == [{'pid': 4242, 'sm_percent': 37}]
    free['bytes'] = 1024**3
    with pytest.raises(RuntimeError, match='Less than 10 GiB'):
        campaign.host_available('record')
    bundle = campaign.prepare(tmp_path/'bundle')
    campaign.allow_external_gpu(bundle)
    assert campaign.read_json(bundle/'campaign.json')['external_gpu_policy'] == 'record'
    assert '**record**' in (bundle/'REPORT.md').read_text()
    state = campaign.read_json(bundle/'campaign.json')
    state['status'] = 'running-live'
    campaign.atomic_json(bundle/'campaign.json', state)
    with pytest.raises(RuntimeError, match='before launch'):
        campaign.allow_external_gpu(bundle)


def test_host_load_is_attributed_to_the_stage_that_was_running(tmp_path):
    rows = [{'t': 5, 'external_busy': True, 'meminfo_kib': {'MemAvailable': 40 * 1024**2}},
            {'t': 15, 'external_busy': False, 'meminfo_kib': {'MemAvailable': 30 * 1024**2}},
            {'t': 25, 'external_busy': True, 'meminfo_kib': {'MemAvailable': 20 * 1024**2}}]
    (tmp_path/'host_samples.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
    state = {'operations': [{'stage': 'check_gz', 'epoch': [0, 10]},
                            {'stage': 'gz_r1', 'epoch': [10, 30]}]}
    assert campaign.stage_host_load(tmp_path, state) == {
        'check_gz': {'samples': 1, 'external_busy': 1, 'min_mem_available_gib': 40.0},
        'gz_r1': {'samples': 2, 'external_busy': 1, 'min_mem_available_gib': 20.0}}


def test_fully_imported_stage_is_skipped_and_its_input_kept_aside(tmp_path, monkeypatch):
    bundle = campaign.prepare(tmp_path/'bundle')
    source, _sweep = completed_source(tmp_path, monkeypatch, conditions=evidence.CONDITIONS)
    campaign.import_conditions(bundle, source, 'gz_r1', list(evidence.CONDITIONS))
    state = campaign.read_json(bundle/'campaign.json')
    assert campaign.stage_conditions(state, 'gz_r1') == ()
    assert not (bundle/'inputs/gz_r1.yaml').exists() and (bundle/'inputs_imported/gz_r1.yaml').exists()
    monkeypatch.setattr(campaign, 'check_frozen', lambda _bundle: None)
    monkeypatch.setattr(campaign, 'host_available', lambda *_args: {})
    ran = []
    def owned(command, *_args):
        ran.append(command)
        raise RuntimeError('stop after the first stage that runs')
    monkeypatch.setattr(campaign, 'owned_command', owned)
    with pytest.raises(RuntimeError, match='first stage that runs'):
        campaign.run_stages(bundle, state, ('gz_r1', 'isaac_r1'), time.monotonic()+3600)
    assert len(ran) == 1 and ran[0][2].endswith('isaac_r1.yaml')
