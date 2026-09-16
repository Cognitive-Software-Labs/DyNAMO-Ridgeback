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
import yaml

from ridgeback_autonomy.benchmarking.configurator import (
    ConfiguratorError,
    _evidence_issues,
    _packaged_scenarios,
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
from ridgeback_autonomy.benchmarking.replay import (
    REPLAY_SCHEMA_VERSION,
    ReplayDatasetWriter,
)
from ridgeback_autonomy.benchmarking.replay_jobs import command_argv, parse_job
from ridgeback_autonomy.benchmarking.replay_profiles import (
    ProfileValidationError,
    offered_estimators,
)
from ridgeback_autonomy.benchmarking.sweep import estimate_trials
from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    PUBLIC_ESTIMATOR_ORDER,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
# Globbed rather than listed, so a sweep added to the repository is covered here
# without anyone remembering to extend the list.
SHIPPED_SWEEPS = sorted(
    (REPO_ROOT / 'src/ridgeback_autonomy/config').glob('benchmark_sweep_*.yaml'))

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


def test_each_profile_offers_exactly_the_estimators_it_runs():
    offered = {
        name: next(field['options'] for field in profile['variant_fields']
                   if field['key'] == 'estimators')
        for name, profile in capabilities()['profiles'].items()
    }

    # No profile may offer an estimator and then refuse it, nor offer one its
    # executor has no path for.
    assert set(offered) == {'measurement', 'mask-output', 'mask-model', 'live-system'}
    # Both frozen formats record the LiDAR scan now, so every offline profile
    # runs every mask estimator. Whether the file in hand actually carries beams
    # is a question about that artifact, asked separately, because two files of
    # the same kind differ there.
    for name in ('measurement', 'mask-output', 'mask-model'):
        assert offered[name] == [
            'projective_ranging', 'euclidean_reconstruction', 'polar_profiling']
    # Only the live system reads the organized point cloud, which no frozen
    # evidence records.
    assert offered['live-system'] == list(PUBLIC_ESTIMATOR_ORDER)


def test_every_shipped_sweep_yaml_is_referenceable_from_the_live_profile(tmp_path):
    """The live profile must reach the sweeps the repository actually ships."""

    assert SHIPPED_SWEEPS

    # Rendered from a draft the page could actually be holding, variants and
    # baseline included: a live job carrying leftovers from the authoring panel
    # is the normal case, since one draft outlives every profile switch.
    rendered = {
        path.name: render_job(_live_draft(sweep_path=str(path)), tmp_path)
        for path in SHIPPED_SWEEPS
    }

    # The baseline sweep is the canonical four-estimator comparison and the
    # isolation sweep opens on a point cloud reference, so an estimator axis
    # narrowed to the depth paths put both of them out of the GUI's reach.
    assert {name: outcome['issues'] for name, outcome in rendered.items()
            if not outcome['valid']} == {}
    assert 'target_benchmark_sweep' in rendered['benchmark_sweep_baseline.yaml']['command']


def _live_draft(**overrides) -> dict:
    draft = {
        'question': 'measure-live-system', 'profile': 'live-system', 'sweep_path': '',
        'sweep_defaults': {'scenario': '', 'repeats': 1},
        'variants': [{'name': 'polar', 'arguments': {
            'estimators': 'polar_profiling', 'polar_range_band_m': 0.4}}],
        'baseline': 'polar', 'output_dir': '',
    }
    return {**draft, **overrides}


def test_an_authored_live_sweep_carries_the_polar_settings(tmp_path):
    rendered = render_job(_live_draft(), tmp_path)

    assert rendered['valid'] is True
    # target_benchmark_sweep opens a YAML file, so the page cannot hand it a
    # draft: the sweep has to be written before a command can name it.
    written = Path(rendered['command'].split()[-1])
    assert written.parent == tmp_path / 'artifacts' / 'benchmark-jobs'
    assert rendered['job']['sweep'] == str(written)

    document = yaml.safe_load(written.read_text())
    assert document['defaults'] == {'scenario': '', 'repeats': 1}
    assert document['configs'][0]['estimators'] == 'polar_profiling'
    assert document['configs'][0]['polar_range_band_m'] == 0.4
    # An input element only ever yields text. Written back as text, the authored
    # file would quote every number and read unlike the sweeps it sits beside.
    typed = render_job(_live_draft(sweep_defaults={'scenario': '', 'repeats': '2'},
                                   variants=[{'name': 'polar', 'arguments': {
                                       'estimators': 'polar_profiling',
                                       'polar_range_band_m': '0.55'}}]), tmp_path)
    authored = yaml.safe_load(Path(typed['command'].split()[-1]).read_text())
    assert authored['defaults']['repeats'] == 2
    assert authored['configs'][0]['polar_range_band_m'] == 0.55


def test_naming_a_live_sweep_runs_that_file_rather_than_the_draft(tmp_path):
    referenced = SHIPPED_SWEEPS[0]

    rendered = render_job(_live_draft(sweep_path=str(referenced)), tmp_path)

    # A referenced sweep is already validated and already says what it runs, so
    # the variants on the page are not merged into it.
    assert rendered['command'].endswith(str(referenced))
    assert rendered['job']['sweep'] == str(referenced)


def test_a_live_config_carries_the_gate_only_where_a_mask_row_reads_it(tmp_path):
    rendered = render_job(_live_draft(baseline='polar', variants=[
        {'name': 'polar', 'arguments': {
            'estimators': 'polar_profiling', 'mask_gate': 'silhouette'}},
        {'name': 'cloud', 'arguments': {
            'estimators': 'pointcloud', 'mask_gate': 'silhouette'}},
    ]), tmp_path)

    polar, cloud = yaml.safe_load(
        Path(rendered['command'].split()[-1]).read_text())['configs']

    # The point cloud reads the organized cloud and never sees a mask, so a gate
    # on that config is a knob that does nothing -- which the sweep parser
    # refuses outright rather than running twice.
    assert polar['mask_gate'] == 'silhouette'
    assert 'mask_gate' not in cloud


def test_a_live_job_estimates_trials_the_way_its_dry_run_does(tmp_path):
    outcome = validate_job(_live_draft(sweep_defaults={
        'scenario': '', 'repeats': 2}), tmp_path)

    job = parse_job(outcome['job'])
    expected = sum(
        estimate_trials(config, _packaged_scenarios() or '')
        for config in job.sweep.configs)

    # Authoring a sweep without stating repeats inherits five, and the packaged
    # scenario set is 88 scenes: hours of simulation the page would not show.
    assert outcome['estimates']['trials'] == expected
    assert 'h at' in outcome['estimates']['capture_duration']


def test_an_authored_live_sweep_survives_export_and_reimport(tmp_path):
    first = render_job(_live_draft(), tmp_path)

    imported = import_job(first['yaml'], tmp_path)

    # The written sweep is a real file by now, so reopening the job references
    # it rather than authoring a second copy from the same draft.
    assert imported['valid'] is True
    assert imported['job']['sweep_path'] == first['job']['sweep']
    assert render_job(imported['job'], tmp_path)['command'] == first['command']


def test_a_live_command_refuses_a_sweep_that_was_never_written(tmp_path):
    document = validate_job(_live_draft(), tmp_path)['job']

    with pytest.raises(ProfileValidationError) as caught:
        command_argv('/jobs/job.yaml', parse_job(document))

    assert caught.value.code == 'live_sweep_requires_path'


def test_an_estimator_the_profile_cannot_run_names_the_estimator(tmp_path):
    draft = {**_job(_dataset(tmp_path)), 'variants': [
        {'name': 'baseline', 'arguments': {'estimators': 'pointcloud'}}]}

    issue = validate_job(draft, tmp_path)['issues'][0]

    # Membership is a profile question: the value is a well-formed estimator
    # list, so reporting it as a malformed one named the field but not the cause.
    assert issue['code'] == 'incompatible_estimator'
    assert 'pointcloud' in issue['message']
    # The organized cloud exists only while the system is running, so the live
    # system is the one profile that can measure this row.
    assert issue['suggested_profile'] == 'live-system'


def test_every_profile_renders_and_accepts_the_polar_axes():
    """The blanket offline freeze is gone; availability moved to the artifact.

    Both frozen formats record a scan now, so no profile can say in advance that
    a polar knob is unreachable. What can still be unreachable is the individual
    file, and the evidence rule answers that with its own reason.
    """

    rendered = {
        name: {field['key'] for field in profile['variant_fields']}
        for name, profile in capabilities()['profiles'].items()
    }
    polar_axes = {'polar_range_band_m', 'polar_range_jump_m', 'polar_min_valid_rays'}

    def document(profile: str, inputs: dict) -> dict:
        return {
            'job_version': 1, 'profile': profile, 'inputs': inputs,
            'sweep': {'sweep': {'name': 'polar'}, 'configs': [{
                'name': 'baseline', 'estimators': 'polar_profiling',
                'polar_range_band_m': 0.5}]},
        }

    parse_job(document('measurement', {'measurement_dataset': '/legacy'}))
    parse_job(document(
        'mask-output', {'sensor_capture': '/sensor', 'mask_caches': ['/box']}))
    for name in ('measurement', 'mask-output', 'mask-model', 'live-system'):
        assert polar_axes <= rendered[name]


def test_the_measurement_profile_accepts_euclidean_against_legacy_evidence(tmp_path):
    draft = {**_job(_dataset(tmp_path)), 'variants': [
        {'name': 'baseline', 'arguments': {'estimators': 'euclidean_reconstruction'}}]}

    outcome = validate_job(draft, tmp_path)

    assert outcome['valid'] is True
    assert outcome['job']['sweep']['configs'][0]['estimators'] == 'euclidean_reconstruction'
    assert 'isolation_3d_floor_margin_m' in outcome['job']['sweep']['configs'][0]


def test_loaded_evidence_says_which_estimators_it_cannot_feed(tmp_path):
    """The artifact explains itself, so an absent option is never unexplained."""

    summary = inspect_path(str(_dataset(tmp_path)), tmp_path)

    unavailable = summary['unavailable_estimators']
    # A dataset written at the current schema carries a scan, so polar is not
    # refused here; the organized point cloud is never recorded by any offline
    # format and stays refused with the reason.
    assert set(unavailable) == {'pointcloud'}
    assert 'point cloud' in unavailable['pointcloud']
    assert summary['schema_version'] == REPLAY_SCHEMA_VERSION


def test_the_page_can_name_every_estimator_it_reports_on():
    """Reasons are keyed by estimator id; the page shows them to a human.

    Without this the browser would either print the raw key, which reads
    differently than every run summary, or invent its own prose for names this
    contract already owns.
    """

    labels = capabilities()['contract']['estimator_labels']

    assert set(labels) == set(PUBLIC_ESTIMATOR_ORDER)
    assert labels['polar_profiling'] == 'Polar Profiling'


def test_validation_hands_the_page_the_evidence_it_must_narrow_by(tmp_path):
    """The narrowing data rides on the validation response, not a second fetch.

    The page redraws its estimator menu from this. Were it to ask separately, it
    could narrow the menu for a different artifact than the one being validated,
    and the menu and the issues below it would disagree.
    """

    outcome = validate_job(_job(_dataset(tmp_path)), tmp_path)

    summary = outcome['artifacts']['measurement_dataset']
    assert 'pointcloud' in summary['unavailable_estimators']
    assert summary['schema_version'] == REPLAY_SCHEMA_VERSION


def test_offered_estimators_intersect_the_profile_with_the_evidence(tmp_path):
    """Two unlike refusals, one list, each keeping the reason that applies.

    The profile answers for its executor and the artifact answers for its
    channels. A capture written before scan recording is the case where the
    profile runs polar and the file still cannot feed it, so the evidence reason
    is the one the operator needs.
    """

    predates_scan = {'polar_profiling': 'this sensor capture is payload version 1'}

    on_a_v1_capture = offered_estimators('mask-output', predates_scan)
    on_a_v2_capture = offered_estimators('mask-output')

    assert on_a_v2_capture['offered'] == (
        'projective_ranging', 'euclidean_reconstruction', 'polar_profiling')
    assert on_a_v1_capture['offered'] == (
        'projective_ranging', 'euclidean_reconstruction')
    assert on_a_v1_capture['unavailable']['polar_profiling'] == (
        predates_scan['polar_profiling'])
    # The point cloud is refused by both profiles for the same reason -- no
    # offline executor reads the organized cloud -- and that reason names where
    # it can be measured instead.
    for offered in (on_a_v1_capture, on_a_v2_capture):
        assert 'live-system' in offered['unavailable']['pointcloud']


def test_a_capture_without_a_scan_refuses_polar_instead_of_dropping_it():
    """The page refuses before the run, naming the estimator and the reason.

    The evidence summary is stated rather than captured: that a payload-version-1
    capture reports this is covered against a real one in the layered-replay
    tests, and what is under test here is the page turning it into an issue
    instead of letting the executor discover it after loading every trial.
    """

    job = parse_job({
        'job_version': 1, 'profile': 'mask-output',
        'inputs': {'sensor_capture': '/sensor', 'mask_caches': ['/box']},
        'sweep': {'sweep': {'name': 'polar'}, 'configs': [
            {'name': 'depth', 'estimators': 'projective_ranging'},
            {'name': 'polar', 'estimators': 'projective_ranging,polar_profiling'},
            {'name': 'polar_wide', 'estimators': 'polar_profiling',
             'polar_range_band_m': 0.5},
        ]},
    })
    artifacts = {'sensor_capture': {'unavailable_estimators': {
        'polar_profiling': (
            'this sensor capture is payload version 1, written before the LiDAR '
            'scan was recorded'),
        'pointcloud': 'the organized point cloud is never recorded',
    }}}

    issues = _evidence_issues(job, artifacts)

    # One issue for two offending variants: a sweep that selects polar in
    # twenty configs has one problem, and twenty copies would bury every other
    # field on the page.
    assert len(issues) == 1
    assert issues[0]['field'] == 'estimators'
    assert issues[0]['code'] == 'evidence_cannot_feed_estimator'
    assert 'Polar Profiling' in issues[0]['message']
    assert 'payload version 1' in issues[0]['message']
    # The point cloud is equally unfeedable and equally unselected, so it is not
    # reported: the page answers for the job in hand, not for the artifact.
    assert 'Point Cloud' not in issues[0]['message']


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


def test_a_missing_live_sweep_is_reported_as_a_field_issue(tmp_path):
    server = make_server(token='test-token', workspace_root=tmp_path)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address[:2]
        connection = http.client.HTTPConnection(host, port)
        connection.request(
            'POST', '/api/validate',
            body=json.dumps({'job': {
                'profile': 'live-system', 'sweep_path': 'config/absent.yaml'}}),
            headers={'X-Configurator-Token': 'test-token',
                     'Content-Type': 'application/json'})
        response = connection.getresponse()
        payload = json.loads(response.read())
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)

    # load_sweep reports a missing file as an OSError, which is not a ValueError.
    # Uncaught it killed the handler thread, and the page reported an unreachable
    # service for a path the operator simply mistyped.
    assert response.status == 200
    assert payload['valid'] is False
    assert payload['issues'][0]['field'] == 'sweep_path'
    assert 'config/absent.yaml' in payload['issues'][0]['message']


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
