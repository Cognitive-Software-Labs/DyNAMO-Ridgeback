"""Focused contract and safety tests for the local benchmark configurator."""

from __future__ import annotations

import http.client
import json
import os
from pathlib import Path
import signal
import threading
import time

import pytest

from ridgeback_autonomy.benchmarking.configurator import (
    ConfiguratorError,
    _ui_sweep,
    capabilities,
    import_job,
    make_server,
    render_job,
    start_run,
    validate_job,
)
from ridgeback_autonomy.benchmarking import configurator_runs
from ridgeback_autonomy.benchmarking.configurator_results import (
    classify,
    discover_results,
    inspect_path,
    rename_result,
)
from ridgeback_autonomy.benchmarking.configurator_runs import RunSupervisor, is_alive
from ridgeback_autonomy.benchmarking.replay import ReplayDatasetWriter
from ridgeback_autonomy.benchmarking.replay_jobs import parse_job


# Long enough that a same-machine process cannot die inside it by accident,
# short enough to keep the suite fast.
GRACE_SEC = 1.0
QUICK_ESCALATION = (
    (signal.SIGINT, GRACE_SEC), (signal.SIGTERM, GRACE_SEC), (signal.SIGKILL, 0.0))


def _dataset(tmp_path: Path) -> Path:
    root = tmp_path / 'replay'
    writer = ReplayDatasetWriter(root, {'trials_included': 1})
    writer.write_trial(
        {'trial_id': 'trial', 'scene_id': 'scene', 'ground_truth': []},
        [{'stamp_ns': 1, 'detected': False, 'count': 0, 'image_width': 1,
          'image_height': 1, 'detections': []}],
    )
    writer.finalize(trials_included=1, trials_skipped=0)
    return root


def _job(dataset: Path) -> dict:
    return {
        'question': 'measurement', 'profile': 'measurement',
        'inputs': {'dataset': str(dataset)}, 'repeats': 1, 'workers': 1,
        'variants': [
            {'name': 'baseline', 'arguments': {}},
            {'name': 'wide_band', 'arguments': {'isolation_2d_band_m': 0.5}},
        ], 'baseline': 'baseline',
    }


def _workspace(tmp_path: Path) -> tuple[Path, dict]:
    """A workspace whose job refers to its evidence and output relatively."""

    _dataset(tmp_path)
    return tmp_path, {
        **_job(tmp_path / 'replay'),
        'inputs': {'dataset': 'replay'},
        'output_dir': 'artifacts/benchmarks/first',
    }


def _settled(supervisor: RunSupervisor, run_id: str, timeout: float = 20.0) -> dict:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        record = supervisor.detail(run_id)
        if record['state'] not in configurator_runs.ACTIVE_STATES:
            return record
        time.sleep(0.02)
    raise AssertionError(f'Run {run_id} never settled: {supervisor.detail(run_id)}')


@pytest.fixture
def fixed_command(monkeypatch):
    """Replace the rendered benchmark command with a controllable one."""

    def use(*argv: str):
        monkeypatch.setattr(configurator_runs, 'command_argv', lambda _path, _job: list(argv))

    return use


def test_capabilities_expose_all_four_canonical_profiles():
    data = capabilities()
    assert [item['id'] for item in data['questions']] == [
        'tune-measurement', 'compare-mask-outputs', 'try-mask-model', 'measure-live-system']
    assert data['profiles']['measurement']['available'] is True
    assert data['profiles']['mask-output']['available'] is True
    assert data['profiles']['mask-model']['available'] is True


def _variant(profile: str, arguments: dict, root: Path) -> dict:
    """The one variant a draft renders, after unreachable settings are dropped."""

    raw = {'variants': [{'name': 'baseline', 'arguments': arguments}]}
    return _ui_sweep(raw, profile, root)['configs'][0]


def test_selecting_both_estimators_keeps_both_parameter_sets(tmp_path):
    config = _variant('mask-output', {
        'estimators': 'projective_ranging,euclidean_reconstruction',
        'isolation_2d_band_m': 0.75,
        'isolation_3d_floor_margin_m': 0.12,
    }, tmp_path)

    # The value is an estimator list, so membership decides reachability: the
    # whole comma-joined string never equals one estimator's name.
    assert config['isolation_2d_band_m'] == 0.75
    assert config['isolation_3d_floor_margin_m'] == 0.12
    assert config['estimators'] == 'projective_ranging,euclidean_reconstruction'


@pytest.mark.parametrize('estimator,kept,dropped', [
    ('projective_ranging', 'isolation_2d_band_m', 'isolation_3d_floor_margin_m'),
    ('euclidean_reconstruction', 'isolation_3d_floor_margin_m', 'isolation_2d_band_m'),
])
def test_one_estimator_carries_only_its_own_parameters(tmp_path, estimator, kept, dropped):
    config = _variant('mask-output', {'estimators': estimator}, tmp_path)

    assert kept in config
    assert dropped not in config


def test_a_recipe_only_carries_the_settings_it_binds(tmp_path):
    nearest = _variant('measurement', {'isolation_2d': 'nearest_mode_histogram'}, tmp_path)
    otsu = _variant('measurement', {
        'isolation_2d': 'otsu', 'isolation_2d_band_m': 0.75}, tmp_path)

    assert {'isolation_2d_band_m', 'isolation_2d_min_bin_fraction'} <= set(nearest)
    # otsu_foreground accepts a bin width only, so the other two are inert.
    assert 'isolation_2d_band_m' not in otsu
    assert 'isolation_2d_min_bin_fraction' not in otsu
    assert 'isolation_2d_bin_width_m' in otsu


def test_a_job_whose_recipe_drops_fields_still_validates(tmp_path):
    draft = {**_job(_dataset(tmp_path)), 'variants': [
        {'name': 'baseline', 'arguments': {'isolation_2d': 'otsu'}}]}

    rendered = render_job(draft, tmp_path)

    assert rendered['valid'] is True
    assert 'isolation_2d_band_m' not in rendered['job']['sweep']['configs'][0]


@pytest.mark.parametrize('recipe,settings', [
    ('height_crop', {'floor_margin_m'}),
    ('range_band', {'percentile', 'ahead_m', 'behind_m'}),
    ('nearest_mode_band', {'ahead_m', 'behind_m', 'bin_width_m', 'min_bin_fraction'}),
    ('height_crop_range_band', {'floor_margin_m', 'percentile', 'ahead_m', 'behind_m'}),
    ('height_crop_nearest_mode_band',
     {'floor_margin_m', 'ahead_m', 'behind_m', 'bin_width_m', 'min_bin_fraction'}),
])
def test_each_euclidean_recipe_carries_exactly_the_settings_it_binds(
        tmp_path, recipe, settings):
    config = _variant('mask-output', {
        'estimators': 'euclidean_reconstruction', 'isolation_3d': recipe}, tmp_path)

    # build_isolation_3d binds a value only to the steps that declare it: a
    # chain without a floor crop never reads a margin, and a percentile anchor
    # never reads a bin width.
    assert {key.removeprefix('isolation_3d_') for key in config
            if key.startswith('isolation_3d_')} == settings


def test_capabilities_scope_each_field_to_what_can_reach_it():
    fields = {field['key']: field
              for field in capabilities()['profiles']['mask-output']['variant_fields']}

    assert fields['isolation_2d_band_m']['recipes'] == ['nearest_mode_histogram']
    assert fields['isolation_2d_min_bin_fraction']['recipes'] == ['nearest_mode_histogram']
    assert fields['isolation_2d_bin_width_m']['recipes'] == []
    assert fields['isolation_3d_percentile']['recipes'] == [
        'height_crop_range_band', 'range_band']
    assert fields['isolation_3d_floor_margin_m']['recipes'] == [
        'height_crop', 'height_crop_nearest_mode_band', 'height_crop_range_band']
    # An empty scope reads as "every recipe reaches this", so a euclidean
    # setting that reached none would be offered everywhere, not nowhere.
    assert all(field['recipes'] for key, field in fields.items()
               if key.startswith('isolation_3d_'))
    # Unscoped, and therefore reachable on every profile: the estimator choice
    # itself cannot be hidden behind an estimator.
    assert fields['estimators']['estimators'] == []
    assert fields['estimators']['recipes'] == []


def test_every_profile_offers_both_depth_estimators(tmp_path):
    offered = {
        name: next(field['options'] for field in profile['variant_fields']
                   if field['key'] == 'estimators')
        for name, profile in capabilities()['profiles'].items()
    }

    # Both estimators read the same frozen evidence, so no profile may offer one
    # and then refuse it. The choices are still scoped to what the profile can
    # evaluate: nothing here offers polar profiling, which needs a LiDAR scan.
    assert set(offered) == {'measurement', 'mask-output', 'mask-model', 'live-system'}
    for options in offered.values():
        assert options == ['projective_ranging', 'euclidean_reconstruction']


def test_the_measurement_profile_accepts_euclidean_against_legacy_evidence(tmp_path):
    draft = {**_job(_dataset(tmp_path)), 'variants': [
        {'name': 'baseline', 'arguments': {'estimators': 'euclidean_reconstruction'}}]}

    outcome = validate_job(draft, tmp_path)

    assert outcome['valid'] is True
    assert outcome['job']['sweep']['configs'][0]['estimators'] == 'euclidean_reconstruction'
    assert 'isolation_3d_floor_margin_m' in outcome['job']['sweep']['configs'][0]


def test_one_variant_per_estimator_carries_only_that_estimators_parameters(tmp_path):
    raw = {'variants': [
        {'name': 'projective', 'arguments': {'estimators': 'projective_ranging'}},
        {'name': 'euclidean', 'arguments': {'estimators': 'euclidean_reconstruction'}},
    ]}

    projective, euclidean = _ui_sweep(raw, 'mask-output', tmp_path)['configs']

    assert 'isolation_2d_band_m' in projective and 'isolation_3d_floor_margin_m' not in projective
    assert 'isolation_3d_floor_margin_m' in euclidean and 'isolation_2d_band_m' not in euclidean


def test_rendered_measurement_job_is_accepted_by_the_canonical_job_parser(tmp_path):
    rendered = render_job(_job(_dataset(tmp_path)), tmp_path)
    assert rendered['valid'] is True
    assert 'target_replay_benchmark' in rendered['command']
    assert parse_job(__import__('yaml').safe_load(rendered['yaml'])).profile == 'measurement'


def test_import_export_round_trip_is_stable(tmp_path):
    first = render_job(_job(_dataset(tmp_path)), tmp_path)
    imported = import_job(first['yaml'], tmp_path)
    second = render_job(imported['job'], tmp_path)
    assert second['yaml'] == first['yaml']


def test_mask_model_is_enabled_and_reports_its_required_sensor_input():
    result = validate_job({'profile': 'mask-model', 'variants': [{'name': 'baseline'}]})
    assert result['valid'] is False
    assert result['issues'][0]['field'] == 'inputs.sensor_capture'


def test_canonical_job_import_round_trips(tmp_path):
    rendered = render_job(_job(_dataset(tmp_path)), tmp_path)
    imported = import_job(rendered['yaml'], tmp_path)
    assert imported['job']['profile'] == 'measurement'
    assert imported['valid'] is True


def test_server_requires_token_and_serves_assets_through_a_symlink(tmp_path, monkeypatch):
    import ridgeback_autonomy.benchmarking.configurator as configurator

    assets_link = tmp_path / 'installed-assets'
    assets_link.symlink_to(
        Path(configurator.__file__).with_name('configurator_assets'), target_is_directory=True)
    monkeypatch.setattr(configurator, '_assets_directory', lambda: assets_link)
    server = make_server(token='test-token', workspace_root=tmp_path)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address[:2]
        connection = http.client.HTTPConnection(host, port)
        connection.request('GET', '/api/capabilities')
        assert connection.getresponse().status == 403
        connection.close()

        connection = http.client.HTTPConnection(host, port)
        connection.request('GET', '/')
        response = connection.getresponse()
        assert response.status == 200
        assert response.getheader('Content-Type') == 'text/html; charset=utf-8'
        assert b'Configure a<br><em>reproducible</em> benchmark.' in response.read()
        connection.close()

        connection = http.client.HTTPConnection(host, port)
        connection.request('GET', '/api/capabilities', headers={'X-Configurator-Token': 'test-token'})
        response = connection.getresponse()
        assert response.status == 200
        assert json.loads(response.read())['profiles']['live-system']['available'] is True
        connection.close()
        assert list(tmp_path.iterdir()) == [assets_link]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


# -- the job the server writes ------------------------------------------------


def test_rendered_command_names_the_saved_job_by_absolute_path(tmp_path):
    root, draft = _workspace(tmp_path)
    rendered = render_job(draft, root)

    saved = Path(rendered['job_path'])
    assert saved.parent == root / 'artifacts' / 'benchmark-jobs'
    assert saved.is_file()
    # A downloaded job lands in a directory the page cannot name, so the command
    # may never depend on the shell's working directory to find it.
    assert './' not in rendered['command']
    assert str(saved) in rendered['command']


def test_relative_paths_resolve_the_same_way_for_inspection_and_execution(tmp_path):
    root, draft = _workspace(tmp_path)

    inspected = inspect_path('replay', root)
    document = validate_job(draft, root)['job']

    assert inspected['path'] == document['inputs']['measurement_dataset']
    assert document['inputs']['measurement_dataset'] == str((root / 'replay').resolve())
    assert document['output_dir'] == str((root / 'artifacts/benchmarks/first').resolve())


def test_a_written_job_runs_from_an_unrelated_working_directory(tmp_path, monkeypatch):
    from ridgeback_autonomy.benchmarking import target_replay_benchmark

    root, draft = _workspace(tmp_path)
    rendered = render_job(draft, root)
    elsewhere = tmp_path / 'unrelated'
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)

    assert target_replay_benchmark.main([rendered['job_path']]) == 0
    assert (root / 'artifacts/benchmarks/first/results').is_dir()


# -- spawn, liveness, and settling --------------------------------------------


def test_a_spawned_run_is_recorded_then_settles_from_its_exit_code(tmp_path, fixed_command):
    root, draft = _workspace(tmp_path)
    fixed_command('true')
    supervisor = RunSupervisor(root)

    record = start_run(draft, supervisor, root)['run']
    assert record['state'] == 'running'
    assert (root / 'artifacts/configurator/runs' / record['run_id'] / 'run.json').is_file()
    assert Path(record['job_path']).is_file()

    settled = _settled(supervisor, record['run_id'])
    assert (settled['state'], settled['returncode']) == ('succeeded', 0)
    assert settled['finished']


def test_a_failing_run_settles_as_failed(tmp_path, fixed_command):
    root, draft = _workspace(tmp_path)
    fixed_command('false')
    supervisor = RunSupervisor(root)

    settled = _settled(supervisor, start_run(draft, supervisor, root)['run']['run_id'])
    assert (settled['state'], settled['returncode']) == ('failed', 1)


def test_liveness_rejects_a_reused_pid_whose_command_line_differs():
    own_argv = Path(f'/proc/{os.getpid()}/cmdline').read_bytes().rstrip(b'\0').split(b'\0')

    assert is_alive({
        'pid': os.getpid(),
        'spawn_argv': [part.decode() for part in own_argv],
    }) is True
    # The same live PID, recorded by a run that is long gone.
    assert is_alive({'pid': os.getpid(), 'spawn_argv': ['bash', '-c', 'other']}) is False


def test_single_flight_refuses_a_second_start(tmp_path, fixed_command):
    root, draft = _workspace(tmp_path)
    fixed_command('sleep', '30')
    supervisor = RunSupervisor(root, escalation=QUICK_ESCALATION)

    first = start_run(draft, supervisor, root)['run']
    try:
        second = dict(draft, output_dir='artifacts/benchmarks/second')
        with pytest.raises(ConfiguratorError) as raised:
            start_run(second, supervisor, root)
        assert raised.value.code == 'run_in_progress'
    finally:
        supervisor.cancel(first['run_id'])
        supervisor.await_cancellation(first['run_id'], timeout=10)


def test_an_existing_output_directory_is_refused_before_anything_is_spawned(tmp_path, fixed_command):
    root, draft = _workspace(tmp_path)
    fixed_command('true')
    (root / 'artifacts/benchmarks/first').mkdir(parents=True)
    supervisor = RunSupervisor(root)

    with pytest.raises(ConfiguratorError) as raised:
        start_run(draft, supervisor, root)

    assert raised.value.field == 'output_dir'
    assert raised.value.code == 'output_exists'
    assert supervisor.records() == []


def test_live_system_is_not_startable_from_the_page(tmp_path):
    supervisor = RunSupervisor(tmp_path)
    sweep = tmp_path / 'sweep.yaml'
    sweep.write_text('name: s\nconfigs:\n  - name: baseline\n', encoding='utf-8')
    draft = {'profile': 'live-system', 'question': 'measure-live-system',
             'sweep_path': str(sweep), 'output_dir': 'artifacts/benchmarks/live'}

    with pytest.raises(ConfiguratorError) as raised:
        start_run(draft, supervisor, tmp_path)

    assert raised.value.code == 'live_run_unsupported'


# -- cancellation --------------------------------------------------------------


def test_cancel_interrupts_first_and_escalates_only_after_the_grace_period(tmp_path, fixed_command):
    root, draft = _workspace(tmp_path)
    # SIG_IGN is inherited, so neither this shell nor its child honours SIGINT.
    # The run can therefore only end once cancellation escalates past it.
    fixed_command('bash', '-c', "trap '' INT; sleep 30")
    supervisor = RunSupervisor(root, escalation=QUICK_ESCALATION)
    record = start_run(draft, supervisor, root)['run']

    requested = time.monotonic()
    cancelling = supervisor.cancel(record['run_id'])
    assert cancelling['state'] == 'cancelling'
    supervisor.await_cancellation(record['run_id'], timeout=20)
    elapsed = time.monotonic() - requested

    assert elapsed >= GRACE_SEC
    assert supervisor.detail(record['run_id'])['state'] == 'cancelled'


def test_cancelling_a_finished_run_is_refused(tmp_path, fixed_command):
    root, draft = _workspace(tmp_path)
    fixed_command('true')
    supervisor = RunSupervisor(root)
    record = _settled(supervisor, start_run(draft, supervisor, root)['run']['run_id'])

    with pytest.raises(ConfiguratorError) as raised:
        supervisor.cancel(record['run_id'])

    assert raised.value.code == 'not_running'


def test_a_run_whose_process_vanished_without_an_exit_code_is_unknown(tmp_path, fixed_command):
    root, draft = _workspace(tmp_path)
    fixed_command('true')
    supervisor = RunSupervisor(root)
    run_id = _settled(supervisor, start_run(draft, supervisor, root)['run']['run_id'])['run_id']

    # Re-open the run the way a restarted configurator would: no child handle,
    # no exit code on disk, and a PID that is no longer the recorded process.
    directory = root / 'artifacts/configurator/runs' / run_id
    (directory / 'returncode').unlink()
    record = json.loads((directory / 'run.json').read_text(encoding='utf-8'))
    (directory / 'run.json').write_text(
        json.dumps({**record, 'state': 'running', 'finished': None}), encoding='utf-8')

    assert RunSupervisor(root).detail(run_id)['state'] == 'unknown'


# -- results browser and rename ------------------------------------------------


def _benchmarks(root: Path) -> Path:
    directory = root / 'artifacts' / 'benchmarks'
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def _sweep(root: Path, name: str = '20260101_000000_demo') -> Path:
    sweep = _benchmarks(root) / name
    (sweep / 'alpha').mkdir(parents=True)
    (sweep / 'alpha' / 'run.json').write_text(
        json.dumps({'run': {'label': '20260101_000001'}}), encoding='utf-8')
    (sweep / 'sweep.json').write_text(json.dumps({
        'configs': [
            {'name': 'alpha', 'arguments': {'output_dir': str(sweep), 'run_dir_name': 'alpha'}},
            {'name': 'beta', 'arguments': {'output_dir': '/stale/elsewhere'}},
        ],
    }), encoding='utf-8')
    return sweep


def _replay_output(root: Path, name: str = 'replay_out') -> Path:
    output = _benchmarks(root) / name
    (output / 'results').mkdir(parents=True)
    (output / 'job.json').write_text(
        json.dumps({'profile': 'measurement', 'sweep': {'configs': [{'name': 'baseline'}]}}),
        encoding='utf-8')
    return output


def test_directories_are_classified_by_what_they_contain(tmp_path):
    sweep = _sweep(tmp_path)
    replay = _replay_output(tmp_path)
    trial = _benchmarks(tmp_path) / 'lone_trial'
    trial.mkdir()
    (trial / 'run.json').write_text(json.dumps({'run': {'label': 'x'}}), encoding='utf-8')
    staging = _benchmarks(tmp_path) / '.replay_out.partial-123-abc'
    staging.mkdir()

    assert classify(sweep) == 'sweep'
    assert classify(sweep / 'alpha') == 'sweep-config'
    assert classify(replay) == 'replay-output'
    assert classify(trial) == 'trial-output'
    assert classify(staging) == 'staging'
    assert classify(_dataset(tmp_path)) == 'artifact'


def test_the_browser_lists_both_kinds_and_hides_staging_directories(tmp_path):
    _sweep(tmp_path)
    _replay_output(tmp_path)
    staging = _benchmarks(tmp_path) / '.replay_out.partial-123-abc'
    staging.mkdir()
    (staging / 'manifest.json').write_text('{}', encoding='utf-8')
    dataset = _dataset(tmp_path)
    (dataset).rename(_benchmarks(tmp_path) / 'evidence')

    listed = {entry['name']: entry for entry in discover_results(tmp_path)}

    assert set(listed) == {'20260101_000000_demo', 'replay_out', 'evidence'}
    assert listed['evidence']['category'] == 'artifact'
    assert listed['replay_out']['category'] == 'result'


def test_renaming_a_replay_output_is_free(tmp_path):
    output = _replay_output(tmp_path)

    renamed = rename_result(tmp_path, str(output), 'named_by_hand')

    assert renamed['name'] == 'named_by_hand'
    assert (output.parent / 'named_by_hand' / 'job.json').is_file()
    assert not output.exists()


def test_renaming_a_sweep_rewrites_every_recorded_config_output_path(tmp_path):
    sweep = _sweep(tmp_path)

    renamed = rename_result(tmp_path, str(sweep), 'polar_rebaseline')

    destination = sweep.parent / 'polar_rebaseline'
    manifest = json.loads((destination / 'sweep.json').read_text(encoding='utf-8'))
    assert [config['arguments']['output_dir'] for config in manifest['configs']] == [
        str(destination), str(destination)]
    assert renamed['kind'] == 'sweep'


def test_renaming_a_config_inside_a_sweep_is_refused(tmp_path):
    sweep = _sweep(tmp_path)

    with pytest.raises(ConfiguratorError) as raised:
        rename_result(tmp_path, str(sweep / 'alpha'), 'gamma')

    assert raised.value.code == 'rename_refused'
    assert (sweep / 'alpha').is_dir()


def test_renaming_a_staging_directory_is_refused(tmp_path):
    staging = _benchmarks(tmp_path) / '.replay_out.partial-123-abc'
    staging.mkdir()

    with pytest.raises(ConfiguratorError) as raised:
        rename_result(tmp_path, str(staging), 'recovered')

    assert raised.value.code == 'rename_refused'


def test_renaming_a_typed_artifact_warns_about_the_jobs_that_name_it(tmp_path):
    dataset = _dataset(tmp_path)
    evidence = _benchmarks(tmp_path) / 'evidence'
    dataset.rename(evidence)
    render_job({**_job(evidence), 'output_dir': 'artifacts/benchmarks/out'}, tmp_path)

    entry = next(item for item in discover_results(tmp_path) if item['name'] == 'evidence')
    assert entry['rename']['allowed'] is True
    assert len(entry['rename']['referencing_jobs']) == 1
    assert 'name inputs by path' in entry['rename']['warning']

    renamed = rename_result(tmp_path, str(evidence), 'evidence_2026')
    assert renamed['summary']['trials'] == 1


@pytest.mark.parametrize('name', ['../escape', 'nested/name', '.hidden', '', '..'])
def test_a_rename_target_must_be_one_plain_path_component(tmp_path, name):
    output = _replay_output(tmp_path)

    with pytest.raises(ConfiguratorError) as raised:
        rename_result(tmp_path, str(output), name)

    assert raised.value.code == 'invalid_name'


def test_a_rename_outside_the_benchmark_root_is_refused(tmp_path):
    outside = tmp_path / 'somewhere_else'
    outside.mkdir()
    _benchmarks(tmp_path)

    with pytest.raises(ConfiguratorError) as raised:
        rename_result(tmp_path, str(outside), 'renamed')

    assert raised.value.code == 'outside_benchmarks'


def test_renaming_onto_an_existing_name_is_refused(tmp_path):
    output = _replay_output(tmp_path)
    (output.parent / 'taken').mkdir()

    with pytest.raises(ConfiguratorError) as raised:
        rename_result(tmp_path, str(output), 'taken')

    assert raised.value.code == 'name_taken'
