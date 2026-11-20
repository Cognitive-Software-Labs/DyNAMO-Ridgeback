"""Preparation, monitoring and comparisons without a simulator or model load."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parent))
import campaign
import evidence
import repeatability_watch


def test_isaac_preparation_has_no_gazebo_inputs_and_does_not_launch(tmp_path, monkeypatch):
    monkeypatch.setattr(campaign, 'frozen_files', lambda _: {})
    monkeypatch.setattr(campaign, 'owned_command', lambda *a, **k: pytest.fail('unexpected launch'))
    bundle = campaign.prepare(tmp_path/'bundle', isaac_only=True)
    state = campaign.read_json(bundle/'campaign.json')
    assert state['status'] == 'prepared-not-launched'
    assert campaign.campaign_order(state) == ('isaac_r1', 'isaac_r2')
    assert campaign.campaign_checks(state) == ('check_isaac',)
    assert sorted(p.stem for p in (bundle/'inputs').glob('*.yaml')) == [
        'check_isaac', 'isaac_r1', 'isaac_r2']
    for stage in campaign.campaign_order(state):
        document = yaml.safe_load((bundle/'inputs'/f'{stage}.yaml').read_text())
        assert [c['name'] for c in document['configs']] == list(evidence.CONDITIONS)
        assert document['defaults']['backend'] == 'isaac'
        assert document['defaults']['sim_mode'] == 'deterministic'
        assert document['defaults']['renderer'] == 'PathTracing'
        assert document['defaults']['anti_aliasing'] == 0
        assert document['defaults']['capture_sec'] == 10.0
        assert all(c['capture_batches'] == 5 for c in document['configs'])
    assert '8 full condition runs: isaac' in (bundle/'REPORT.md').read_text()
    assert not (bundle/'service.json').exists()
    with pytest.raises(RuntimeError, match='failure heartbeat'):
        campaign.require_monitoring(bundle, state)
    with pytest.raises(ValueError, match='two fresh sessions'):
        campaign.campaign_order(dict(state, order=['gz_r1', 'isaac_r1']))


def test_monitor_registration_must_include_verified_delivery(tmp_path):
    state = {'purpose': 'isaac-repeatability'}
    path = tmp_path/'monitor_registration.json'
    registration = {'enabled': True, 'automation_id': 'test', 'target_chat_id': 'chat'}
    campaign.atomic_json(path, registration)
    with pytest.raises(RuntimeError, match='verify'):
        campaign.require_monitoring(tmp_path, state)
    campaign.atomic_json(path, dict(registration, delivery_verified=True))
    campaign.require_monitoring(tmp_path, state)


def test_full_isaac_matrix_and_equal_repetition_settings(tmp_path, monkeypatch):
    monkeypatch.setattr(campaign, 'frozen_files', lambda _: {})
    bundle = campaign.prepare(tmp_path/'bundle', isaac_only=True)
    assert campaign.validate_matrix(bundle) == {
        'check_isaac': [1, 1, 1], 'isaac_r1': [109]*4, 'isaac_r2': [109]*4}
    path = bundle/'inputs/isaac_r2.yaml'
    document = yaml.safe_load(path.read_text())
    document['defaults']['detector_fps'] = 9.0
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='configurations differ'):
        campaign.validate_matrix(bundle)


def test_live_owner_dispatches_only_isaac_stages(tmp_path, monkeypatch):
    monkeypatch.setattr(campaign, 'frozen_files', lambda _: {})
    bundle = campaign.prepare(tmp_path/'bundle', isaac_only=True)
    called = []
    monkeypatch.setattr(campaign, 'preflight', lambda *_: None)
    monkeypatch.setattr(campaign, 'sample_host', lambda *_: None)
    monkeypatch.setattr(campaign, 'acquire_idle_inhibitor', lambda: None)
    monkeypatch.setattr(campaign, 'inventory', lambda *_: {})
    monkeypatch.setattr(campaign, 'check_frozen', lambda *_: None)
    monkeypatch.setattr(campaign, 'workstation_lock', lambda: open(tmp_path/'lock', 'w'))
    monkeypatch.setattr(campaign, 'run_stages', lambda _b, _s, stages, _end: called.extend(stages))
    assert campaign.run(bundle) == 0
    assert called == ['check_isaac', 'isaac_r1', 'isaac_r2']


def stage_budget(tmp_path, monkeypatch, stage):
    """The environment a stage would get, then its adapter on a simulated clock."""
    import types
    import ridgeback_autonomy.benchmarking as package
    import sweep_entry
    clock = types.SimpleNamespace(now=1000.0)
    monkeypatch.setattr(campaign.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(campaign, 'frozen_files', lambda _: {})
    monkeypatch.setattr(campaign, 'check_frozen', lambda *_: None)
    monkeypatch.setattr(sweep_entry, 'check_frozen', lambda *_: None)
    monkeypatch.setattr(campaign, 'host_available', lambda *_: {})
    bundle = campaign.prepare(tmp_path/'bundle', isaac_only=True)
    state = campaign.read_json(bundle/'campaign.json')
    environments = []

    def stop_after_capture(_command, _log, env, deadline):
        environments.append((env, deadline))
        raise InterruptedError

    monkeypatch.setattr(campaign, 'owned_command', stop_after_capture)
    with pytest.raises(InterruptedError):
        campaign.run_stages(bundle, state, (stage,), clock.now+campaign.remaining_budget(state))
    env, deadline = environments[0]
    for key, value in env.items():
        if key.startswith('QUALIFICATION_'):
            monkeypatch.setenv(key, value)
    waits = []
    fake = types.SimpleNamespace(
        _write_manifest=lambda *_: None, _config_command=lambda _args: ['admitted'],
        _wait_for_config=lambda _process, _env, timeout: waits.append(timeout), main=lambda: 0)
    monkeypatch.setattr(package, 'target_benchmark_sweep', fake, raising=False)
    sweep_entry.main()
    return clock, fake, waits, deadline


def test_readiness_admits_third_condition_at_the_failed_elapsed_time(tmp_path, monkeypatch):
    clock, fake, waits, deadline = stage_budget(tmp_path, monkeypatch, 'check_isaac')
    assert deadline == 1000+campaign.CHECK_STAGE_SEC
    clock.now = 1000+132  # silhouette_monocular reached admission here on 2026-11-05
    assert fake._config_command({}) == ['admitted']
    fake._wait_for_config(None, None, 1e6)
    assert waits == [campaign.CHECK_STAGE_SEC-campaign.STAGE_RESERVE_SEC-132]
    assert waits[0] > campaign.CHECK_ADMISSION_SEC
    last = campaign.CHECK_STAGE_SEC-campaign.STAGE_RESERVE_SEC-campaign.CHECK_ADMISSION_SEC
    clock.now = 1000+last-1
    assert fake._config_command({}) == ['admitted']
    clock.now = 1000+last
    with pytest.raises(RuntimeError, match='Insufficient'):
        fake._config_command({})


def test_full_block_admits_fourth_pessimistic_condition(tmp_path, monkeypatch):
    clock, fake, waits, deadline = stage_budget(tmp_path, monkeypatch, 'isaac_r1')
    assert deadline == 1000+campaign.LIVE_SEC/2
    condition = 2600  # 109 trials at the slowest measured 23.4 s, plus startup
    assert condition < campaign.ISAAC_ADMISSION_SEC < campaign.ISAAC_CONFIG_TIMEOUT_SEC
    clock.now = 1000+3*condition
    assert fake._config_command({}) == ['admitted']
    fake._wait_for_config(None, None, 1e6)
    assert waits == [campaign.ISAAC_CONFIG_TIMEOUT_SEC]
    clock.now = deadline-campaign.STAGE_RESERVE_SEC-campaign.ISAAC_ADMISSION_SEC
    with pytest.raises(RuntimeError, match='Insufficient'):
        fake._config_command({})


def test_blocks_share_the_live_budget_and_keep_the_four_block_limits():
    repeatability = {'purpose': 'isaac-repeatability', 'order': ['isaac_r1', 'isaac_r2']}
    cap, _admission, _timeout = campaign.stage_limits(repeatability, 'isaac_r1')
    assert 2*cap == campaign.LIVE_SEC
    assert campaign.stage_limits({'purpose': 'qualification'}, 'gz_r1') == (8100, 2100, 2400)


def test_repeatability_keeps_matching_scene_instance_values_and_misses():
    def row(value, outcome='scored'):
        return {'trial_estimate_m': value, 'outcome': outcome, 'miss_reason': ''}
    a = {('scene2','0'): row('2.0'), ('scene1','0'): row('4.0'),
         ('scene2','1'): row('', 'gate_miss')}
    b = {('scene1','0'): row('4.001'), ('scene2','1'): row('', 'gate_miss'),
         ('scene2','0'): row('2.01')}
    r = evidence.repeatability(a, b)
    assert r['paired_numeric_rows'] == 2 and r['outcome_or_reason_changes'] == 0
    assert r['max_abs_estimate_delta_m'] == pytest.approx(.01)
    assert [(p['trial_id'], p['instance_index']) for p in r['pairs']] == [
        ('scene1','0'), ('scene2','0'), ('scene2','1')]
    assert r['pairs'][-1]['abs_delta_m'] is None
    assert r['pairs'][0]['first_estimate_m'] == 4


def test_readiness_proof_rejects_missing_gate_or_unmeasured_samples(tmp_path):
    capture = {'events_without_live_record': 0}
    document = {'producer': {'parameters': {'depth_source': 'monocular'}}}
    campaign.atomic_json(tmp_path/'manifest.json', document)
    windows = {'scene': {'sampled_stamps_ns': [10, 11]}}
    with pytest.raises(ValueError, match='readiness gate'):
        evidence.validate_depth_readiness(tmp_path, windows, capture)
    document['metadata'] = {'depth_warmup': {'component': 'mask', 'state': 'processing',
        'readiness': {'depth_source': 'monocular', 'ready': True, 'stamp_ns': 9}}}
    campaign.atomic_json(tmp_path/'manifest.json', document)
    evidence.validate_depth_readiness(tmp_path, windows, capture)
    with pytest.raises(ValueError, match='unmeasured'):
        evidence.validate_depth_readiness(tmp_path, windows, {'events_without_live_record': 1})
    document['metadata']['depth_warmup']['readiness']['stamp_ns'] = 10
    campaign.atomic_json(tmp_path/'manifest.json', document)
    with pytest.raises(ValueError, match='precede'):
        evidence.validate_depth_readiness(tmp_path, windows, capture)


def running_bundle(tmp_path, now=1000):
    started = datetime.fromtimestamp(now-400, timezone.utc).isoformat()
    campaign.atomic_json(tmp_path/'campaign.json', {'status': 'running-live',
        'current_stage': 'isaac_r1', 'stage_started_utc': started, 'errors': []})
    campaign.atomic_json(tmp_path/'service.json', {'unit': 'test-owned-unit'})
    (tmp_path/'host_samples.jsonl').write_text('{}\n')
    os.utime(tmp_path/'host_samples.jsonl', (now, now))


def test_heartbeat_detects_stall_even_if_diagnostic_log_keeps_chattering(tmp_path):
    running_bundle(tmp_path)
    (tmp_path/'isaac_r1.log').write_text('diagnostic chatter')
    snapshot = repeatability_watch.inspect(tmp_path, now=1000, unit_state='active')
    assert snapshot['attention']
    assert any('No completed capture' in error for error in snapshot['issues'])
    (tmp_path/'capture_windows.jsonl').write_text('{}\n')
    os.utime(tmp_path/'capture_windows.jsonl', (990, 990))
    assert not repeatability_watch.inspect(tmp_path, now=1000, unit_state='active')['attention']
    assert repeatability_watch.inspect(tmp_path, now=1000, unit_state='inactive')['attention']


def test_prepared_heartbeat_stays_quiet_and_failed_guard_wakes_agent(tmp_path):
    campaign.atomic_json(tmp_path/'campaign.json', {'status': 'prepared-not-launched'})
    before = {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    assert not repeatability_watch.inspect(tmp_path)['attention']
    assert before == {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    campaign.atomic_json(tmp_path/'isaac_r1_guard.json', {'error': 'worker failed'})
    assert repeatability_watch.inspect(tmp_path)['attention']


def test_completion_and_numeric_review_triggers(tmp_path):
    campaign.atomic_json(tmp_path/'campaign.json', {'status': campaign.LIVE_COMPLETE})
    assert repeatability_watch.inspect(tmp_path)['attention']  # missing comparisons
    compared = {str(i): {'pairs': [], 'outcome_or_reason_changes': 0} for i in range(12)}
    campaign.atomic_json(tmp_path/'comparisons.json', compared)
    snapshot = repeatability_watch.inspect(tmp_path)
    assert snapshot['completed'] and not snapshot['attention']
    compared['0']['pairs'] = [{'abs_delta_m': .021, 'relative_delta': .005}]
    campaign.atomic_json(tmp_path/'comparisons.json', compared)
    assert repeatability_watch.inspect(tmp_path)['review']
