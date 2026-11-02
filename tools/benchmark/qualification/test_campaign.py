"""No ROS graph, simulator, hardware or model execution in these tests."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import campaign
import evidence


def write_windows(path, seconds=10, events=4, count=1):
    path.write_text('\n'.join(json.dumps({'run_label': 'r', 'trial_id': f't{i}',
        'wall_sec': seconds, 'total_events': events}) for i in range(count)))


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
    event.update(stamp_ns=1, image_width=640, image_height=480, detections=[{}])
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
        assert spec['defaults']['output_dir'].endswith(name)
        assert all(c['sensor_capture_dir'].endswith(name+'/'+c['name']) for c in spec['configs'])
    assert campaign.read_json(bundle/'campaign.json')['status'] == 'prepared-not-launched'
    assert 'pending' in (bundle/'REPORT.md').read_text()


def test_stage_failure_prevents_all_full_benchmarks(tmp_path, monkeypatch):
    bundle = campaign.prepare(tmp_path/'bundle')
    monkeypatch.setattr(campaign, 'ROOT', tmp_path)
    monkeypatch.setattr(campaign, 'host_available', lambda: None)
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
    code = '''class TargetDistanceBenchmarkRunner:
 def __init__(self): self.run_label='r'; self.active_trial_id='t'
 def capture_measurement_window(self, seconds): return {'total_events':2,'usable_events':[]}
result=TargetDistanceBenchmarkRunner().capture_measurement_window(4)
assert result=={'total_events':2,'usable_events':[]}
'''
    import subprocess
    env = dict(os.environ, PYTHONPATH=str(path.parent), QUALIFICATION_CAPTURE_TIMINGS=str(tmp_path/'timings'))
    subprocess.run([sys.executable, '-c', code], env=env, check=True)
    assert json.loads((tmp_path/'timings').read_text())['requested_sec'] == 4
