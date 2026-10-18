from __future__ import annotations

import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from ridgeback_autonomy.benchmarking import backends
from ridgeback_autonomy.benchmarking import target_benchmark_sweep as sweep_module
from ridgeback_autonomy.benchmarking.backends import resolve_benchmark_environment
from ridgeback_autonomy.benchmarking.target_benchmark_sweep import (
    _parser,
    _launch_argument_tokens,
    reap_orphan_benchmark_entities,
    run_preflight_cleanup,
)
from ridgeback_autonomy.benchmarking.scene_control import (
    SceneControlError,
    SceneControlFailure,
)
from ridgeback_autonomy.benchmarking.sweep import load_sweep, parse_sweep


REPO_ROOT = Path(__file__).resolve().parents[3]
ISAAC_SETTINGS = {
    'headless': 'true', 'rtf': '1.0', 'sim_mode': 'realtime', 'depth_fidelity': 'ideal'}
GZ_SETTINGS = {'gz_gui': 'true', 'headless_rendering': 'false'}


def _source_share(package: str) -> str:
    """Adapter packages straight from the source tree, which holds their catalogs."""

    return str(REPO_ROOT / 'src' / package)


def _environment(spec, package_share=_source_share):
    return resolve_benchmark_environment(
        spec.backend, 'target_distance_calibration', spec.backend_settings,
        package_share=package_share)


def _scenario(tmp_path):
    path = tmp_path / 'scenes.yaml'
    path.write_text(
        'scenes:\n'
        '  - id: one\n'
        '    robots: [{ x: 2.0, y: 0.0 }]\n',
        encoding='utf-8',
    )
    return path


def _document(tmp_path):
    return {
        'sweep': {'name': 'unit'},
        'defaults': {'scenario': str(_scenario(tmp_path)), 'repeats': 1},
        'configs': [{'name': 'pointcloud', 'estimators': 'pointcloud'}],
    }


def test_launch_argument_tokens_omit_empty_launch_defaults() -> None:
    assert _launch_argument_tokens({
        'estimators': 'pointcloud',
        'scenario': '',
        'repeats': '1',
    }) == [
        'estimators:=pointcloud',
        'repeats:=1',
    ]


def test_sweep_runs_repo_cleanup_before_starting_the_environment(tmp_path, monkeypatch) -> None:
    cleanup = tmp_path / 'cleanup.sh'
    cleanup.write_text('exit 0\n', encoding='utf-8')
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))

    monkeypatch.setattr(
        'ridgeback_autonomy.benchmarking.target_benchmark_sweep.subprocess.run',
        fake_run,
    )

    run_preflight_cleanup(str(tmp_path))

    assert calls == [
        (['bash', str(cleanup)], {
            'cwd': str(tmp_path),
            'stdin': subprocess.DEVNULL,
            'check': True,
        }),
    ]


def test_sweep_refuses_to_start_without_its_cleanup_script(tmp_path) -> None:
    with pytest.raises(RuntimeError, match='cleanup script is missing'):
        run_preflight_cleanup(str(tmp_path))


def test_sweep_cleanup_can_be_skipped_for_a_shared_process_session() -> None:
    args = _parser().parse_args(['sweep.yaml', '--skip-preflight-cleanup'])

    assert args.skip_preflight_cleanup is True


def test_sweep_reaps_only_entities_returned_by_prefix_scoped_discovery() -> None:
    class SceneControl:
        def __init__(self):
            self.discovery = [
                ('bench_old_target_0', 'bench_old_obj_0'),
                (),
            ]
            self.deleted = []

        def find_benchmark_entities(self):
            return self.discovery.pop(0)

        def delete_entity(self, name):
            self.deleted.append(name)

    scene_control = SceneControl()

    reaped = reap_orphan_benchmark_entities(scene_control)

    assert reaped == ['bench_old_target_0', 'bench_old_obj_0']
    assert scene_control.deleted == ['bench_old_target_0', 'bench_old_obj_0']


def test_sweep_refuses_to_continue_when_orphan_removal_is_incomplete() -> None:
    scene_control = SimpleNamespace(
        find_benchmark_entities=lambda: ('bench_stuck_target_0',),
        delete_entity=lambda _name: None,
    )

    with pytest.raises(RuntimeError, match='bench_stuck_target_0'):
        reap_orphan_benchmark_entities(scene_control)


def test_sweep_rejects_unknown_config_key(tmp_path) -> None:
    document = _document(tmp_path)
    document['configs'][0]['mystery'] = 1

    with pytest.raises(ValueError, match=r'config "pointcloud".*unknown field "mystery"'):
        parse_sweep(document)


def test_sweep_rejects_duplicate_and_unsafe_names(tmp_path) -> None:
    document = _document(tmp_path)
    document['configs'].append({'name': 'pointcloud', 'estimators': 'pointcloud'})
    with pytest.raises(ValueError, match='duplicate config name "pointcloud"'):
        parse_sweep(document)

    document = _document(tmp_path)
    document['configs'][0]['name'] = '../pointcloud'
    with pytest.raises(ValueError, match='path-safe'):
        parse_sweep(document)


@pytest.mark.parametrize(
    'field',
    [
        'world', 'setup_path', 'namespace', 'use_sim_time', 'color_topic',
        'headless_rendering', 'camera_profile', 'detector_fps', 'detector_debug',
        'backend', 'gz_gui', 'headless', 'rtf', 'sim_mode', 'depth_fidelity',
    ],
)
def test_sweep_rejects_environment_key_on_config(tmp_path, field) -> None:
    document = _document(tmp_path)
    document['configs'][0][field] = 'changed'

    with pytest.raises(ValueError, match=rf'field "{field}" belongs to the persistent environment'):
        parse_sweep(document)


@pytest.mark.parametrize(
    ('field', 'value', 'expected'),
    [('detector_fps', 5.0, '5.0'), ('detector_debug', True, 'true')])
def test_sweep_accepts_detector_settings_as_defaults(
    tmp_path, field, value, expected,
) -> None:
    """The detector belongs to the layer that outlives every configuration.

    Its rate and diagnostic are therefore sweep-wide: legal in ``defaults``,
    rejected per config (above), and forwarded to the environment launch rather
    than to the per-config one.
    """

    document = _document(tmp_path)
    document['defaults'][field] = value

    spec = parse_sweep(document)

    assert spec.defaults[field] == expected
    assert field not in spec.configs[0].arguments


def test_sweep_accepts_headless_rendering_as_environment_default(tmp_path) -> None:
    document = _document(tmp_path)
    document['defaults']['headless_rendering'] = True

    spec = parse_sweep(document)

    assert spec.defaults['headless_rendering'] == 'true'
    assert spec.backend_settings == {**GZ_SETTINGS, 'headless_rendering': 'true'}
    # Like the camera profile: the persistent environment applies it, and the
    # config runner only receives it to record what the simulator ran.
    assert spec.configs[0].arguments['headless_rendering'] == 'true'
    assert sweep_module._environment_arguments(spec)['headless_rendering'] == 'true'


def test_sweep_accepts_camera_profile_as_environment_default(tmp_path) -> None:
    document = _document(tmp_path)
    document['defaults']['camera_profile'] = '1280x720'

    spec = parse_sweep(document)

    assert spec.defaults['camera_profile'] == '1280x720'
    # The config runner records it in run.json, but the validator still rejects
    # per-config changes because the persistent simulator owns the renderer.
    assert spec.configs[0].arguments['camera_profile'] == '1280x720'


def test_sweep_rejects_unknown_camera_profile_early(tmp_path) -> None:
    document = _document(tmp_path)
    document['defaults']['camera_profile'] = '1920x1080'

    with pytest.raises(ValueError, match='unknown camera profile'):
        parse_sweep(document)


@pytest.mark.parametrize(
    ('estimators', 'field', 'value'),
    [
        ('pointcloud', 'depth_source', 'monocular'),
        ('pointcloud', 'mask_depth_max_meters', 10.0),
        ('pointcloud', 'mask_gate', 'silhouette'),
        ('pointcloud', 'isolation_3d', 'range_band'),
        ('pointcloud', 'isolation_3d_floor_margin_m', 0.08),
        ('euclidean_reconstruction', 'isolation_2d', 'otsu'),
        ('projective_ranging', 'isolation_3d', 'range_band'),
        ('projective_ranging', 'isolation_3d_bin_width_m', 0.02),
    ],
)
def test_sweep_rejects_inapplicable_explicit_knob(
    tmp_path,
    estimators,
    field,
    value,
) -> None:
    document = _document(tmp_path)
    document['configs'][0].update(estimators=estimators, **{field: value})

    with pytest.raises(ValueError, match=rf'field "{field}" does not apply'):
        parse_sweep(document)


def test_sweep_accepts_shared_sufficiency_floor_for_euclidean(tmp_path) -> None:
    document = _document(tmp_path)
    document['configs'][0].update(
        estimators='euclidean_reconstruction', min_valid_pixels=25)

    spec = parse_sweep(document)

    assert spec.configs[0].arguments['min_valid_pixels'] == '25'


def test_sweep_rejects_bad_estimator(tmp_path) -> None:
    document = _document(tmp_path)
    document['configs'][0]['estimators'] = 'laser_magic'

    with pytest.raises(ValueError, match=r'config "pointcloud".*field "estimators".*laser_magic'):
        parse_sweep(document)


def test_sweep_rejects_missing_or_invalid_scenario(tmp_path) -> None:
    document = _document(tmp_path)
    document['defaults']['scenario'] = str(tmp_path / 'missing.yaml')
    with pytest.raises(ValueError, match=r'field "scenario" does not name a file'):
        parse_sweep(document)

    invalid = tmp_path / 'invalid.yaml'
    invalid.write_text('scenes: []\n', encoding='utf-8')
    document['defaults']['scenario'] = str(invalid)
    with pytest.raises(ValueError, match=r'field "scenario" is invalid'):
        parse_sweep(document)


def test_config_values_override_defaults_and_estimators_are_canonical(tmp_path) -> None:
    document = _document(tmp_path)
    document['defaults'].update(
        repeats=5, overlay=False, estimators='polar_profiling,pointcloud')
    document['configs'][0].update(repeats=2, estimators='polar_profiling,pointcloud')

    spec = parse_sweep(document)

    assert spec.defaults['repeats'] == '5'
    assert spec.defaults['overlay'] == 'false'
    assert spec.configs[0].arguments['repeats'] == '2'
    assert spec.configs[0].arguments['estimators'] == 'pointcloud,polar_profiling'


def test_inherited_defaults_are_exempt_from_applicability_rule(tmp_path) -> None:
    document = _document(tmp_path)
    document['defaults'].update(
        depth_source='stereoscopic',
        mask_gate='box',
        isolation_2d='nearest_mode_histogram',
        isolation_3d='height_crop_nearest_mode_band',
    )
    document['configs'] = [
        {'name': 'pointcloud', 'estimators': 'pointcloud'},
        {'name': 'mask_3d', 'estimators': 'euclidean_reconstruction'},
    ]

    spec = parse_sweep(document)

    assert spec.configs[0].arguments['isolation_3d'] == 'height_crop_nearest_mode_band'
    assert spec.configs[1].arguments['depth_source'] == 'stereoscopic'


def test_relative_scenario_path_resolves_against_sweep_source(tmp_path) -> None:
    scenario = _scenario(tmp_path)
    document = _document(tmp_path)
    document['defaults']['scenario'] = scenario.name
    source = str(tmp_path / 'sweep.yaml')

    spec = parse_sweep(document, source=source)

    assert spec.configs[0].arguments['scenario'] == str(scenario)


def test_model_concurrency_sweep_is_the_ordered_factorial_contract() -> None:
    path = (
        REPO_ROOT
        / 'src/ridgeback_autonomy/config/benchmark_sweep_model_concurrency.yaml'
    )

    spec = load_sweep(str(path))

    assert spec.name == 'model_concurrency'
    assert spec.defaults['repeats'] == '3'
    assert spec.defaults['estimators'] == (
        'projective_ranging,euclidean_reconstruction')
    assert Path(spec.configs[0].arguments['scenario']).name == (
        'benchmark_scenarios_examples.yaml')
    assert [config.name for config in spec.configs] == [
        'aa_combined_diagnostics_off',
        'aa_combined_diagnostics_on',
        'r1_a_box_stereo',
        'r1_b_silhouette_stereo',
        'r1_c_box_monocular',
        'r1_d_silhouette_monocular',
        'r2_d_silhouette_monocular',
        'r2_c_box_monocular',
        'r2_b_silhouette_stereo',
        'r2_a_box_stereo',
        'r3_b_silhouette_stereo',
        'r3_d_silhouette_monocular',
        'r3_a_box_stereo',
        'r3_c_box_monocular',
    ]

    assert spec.configs[0].arguments['depth_match_debug'] == 'false'
    assert spec.configs[1].arguments['depth_match_debug'] == 'true'
    expected_matrix = [
        ('box', 'stereoscopic'),
        ('silhouette', 'stereoscopic'),
        ('box', 'monocular'),
        ('silhouette', 'monocular'),
        ('silhouette', 'monocular'),
        ('box', 'monocular'),
        ('silhouette', 'stereoscopic'),
        ('box', 'stereoscopic'),
        ('silhouette', 'stereoscopic'),
        ('silhouette', 'monocular'),
        ('box', 'stereoscopic'),
        ('box', 'monocular'),
    ]
    assert [
        (config.arguments['mask_gate'], config.arguments['depth_source'])
        for config in spec.configs[2:]
    ] == expected_matrix
    assert all(
        config.arguments['depth_match_debug'] == 'true'
        for config in spec.configs[2:]
    )


def test_sweep_defaults_to_the_gazebo_backend_with_its_settings(tmp_path) -> None:
    spec = parse_sweep(_document(tmp_path))

    assert spec.backend == 'gz'
    assert spec.backend_settings == GZ_SETTINGS
    environment = sweep_module._environment_arguments(spec)
    assert environment['backend'] == 'gz'
    assert {name: environment[name] for name in GZ_SETTINGS} == GZ_SETTINGS
    assert not set(ISAAC_SETTINGS) & set(environment)
    config = sweep_module._resolved_arguments(spec, spec.configs[0], str(tmp_path))
    assert config['backend'] == 'gz'
    assert config['world'] == 'target_distance_calibration'


def test_sweep_selects_isaac_once_for_environment_and_every_config(tmp_path) -> None:
    document = _document(tmp_path)
    document['defaults'].update(backend='isaac', rtf=1, headless=True, depth_fidelity='d455')
    document['configs'].append({'name': 'polar', 'estimators': 'polar_profiling'})

    spec = parse_sweep(document)

    expected = {**ISAAC_SETTINGS, 'depth_fidelity': 'd455'}
    # Normalizing the settings must not disturb the rest of the spec.
    assert spec.name == 'unit'
    assert spec.backend == 'isaac'
    assert spec.backend_settings == expected
    # Normalized, so ``rtf: 1`` and ``rtf: 1.0`` are the same environment.
    assert spec.defaults['rtf'] == '1.0'
    command = sweep_module._environment_command(spec)
    assert 'backend:=isaac' in command
    assert 'world:=target_distance_calibration' in command
    assert {'rtf:=1.0', 'headless:=true', 'sim_mode:=realtime',
            'depth_fidelity:=d455'} <= set(command)
    assert not any(token.startswith(('gz_gui:=', 'headless_rendering:='))
                   for token in command)
    for config in spec.configs:
        arguments = sweep_module._resolved_arguments(spec, config, str(tmp_path))
        assert arguments['backend'] == 'isaac'
        assert {name: arguments[name] for name in expected} == expected


@pytest.mark.parametrize(
    ('backend', 'message'),
    [
        ('hardware', r'backend "hardware" cannot run the target-distance benchmark'),
        ('webots', r"Unknown benchmark backend 'webots'"),
    ],
)
def test_sweep_rejects_hardware_and_unknown_backends(tmp_path, backend, message) -> None:
    document = _document(tmp_path)
    document['defaults']['backend'] = backend

    with pytest.raises(ValueError, match=message):
        parse_sweep(document)


@pytest.mark.parametrize(
    ('backend', 'field', 'value', 'owner'),
    [
        ('gz', 'depth_fidelity', 'd455', 'isaac'),
        ('gz', 'rtf', 0.5, 'isaac'),
        ('isaac', 'gz_gui', False, 'gz'),
        ('isaac', 'headless_rendering', True, 'gz'),
    ],
)
def test_sweep_rejects_settings_of_the_other_backend(
    tmp_path, backend, field, value, owner,
) -> None:
    document = _document(tmp_path)
    document['defaults'].update({'backend': backend, field: value})

    with pytest.raises(
            ValueError, match=rf'field "{field}" applies only to backend {owner}'):
        parse_sweep(document)


@pytest.mark.parametrize(
    ('field', 'value'),
    [('rtf', -1.0), ('rtf', 'fast'), ('sim_mode', 'turbo'),
     ('depth_fidelity', 'lidar'), ('headless', 'maybe')],
)
def test_sweep_rejects_invalid_backend_setting_values(tmp_path, field, value) -> None:
    document = _document(tmp_path)
    document['defaults'].update({'backend': 'isaac', field: value})

    with pytest.raises(ValueError, match=rf'field "{field}" must be'):
        parse_sweep(document)


def test_sweep_resolves_the_public_world_through_the_selected_catalog(tmp_path) -> None:
    looked_up = []

    def share(package):
        looked_up.append(package)
        return _source_share(package)

    gz = parse_sweep(_document(tmp_path))
    document = _document(tmp_path)
    document['defaults']['backend'] = 'isaac'
    isaac = parse_sweep(document)

    gz_environment = sweep_module.resolve_sweep_environment(
        gz, gz.configs, '', package_share=share)
    isaac_environment = sweep_module.resolve_sweep_environment(
        isaac, isaac.configs, '', package_share=share)

    assert looked_up == ['ridgeback_autonomy_gz', 'ridgeback_autonomy_isaac']
    assert (gz_environment.public_world, gz_environment.native_world) == (
        'target_distance_calibration', 'target_distance_calibration')
    assert (isaac_environment.public_world, isaac_environment.native_world) == (
        'target_distance_calibration', 'g1_distance_calibration')
    assert isaac_environment.backend.scene_control_service_namespace == '/isaac_sim'


def test_sweep_refuses_a_scenario_object_the_selected_catalog_lacks(tmp_path) -> None:
    scenario = tmp_path / 'clutter.yaml'
    scenario.write_text(
        'scenes:\n'
        '  - id: one\n'
        '    robots: [{ x: 2.0, y: 0.0 }]\n'
        '    objects: [{ model: table, x: 1.0, y: 0.5 }]\n',
        encoding='utf-8',
    )
    share = tmp_path / 'ridgeback_autonomy_isaac'
    (share / 'sim').mkdir(parents=True)
    (share / 'sim' / 'world.usda').write_text('#usda 1.0\n', encoding='utf-8')
    (share / 'sim' / 'benchmark_assets.yaml').write_text(
        'schema: ridgeback_benchmark_assets/1\n'
        'worlds:\n'
        '  target_distance_calibration:\n'
        '    {native_name: g1_distance_calibration, resource: world.usda, format: usd}\n'
        'targets:\n'
        '  g1: {resource: world.usda, format: usd}\n',
        encoding='utf-8',
    )
    document = _document(tmp_path)
    document['defaults'].update(backend='isaac', scenario=str(scenario))
    spec = parse_sweep(document)

    with pytest.raises(ValueError, match=r'config "pointcloud".*object "table"'):
        sweep_module.resolve_sweep_environment(
            spec, spec.configs, '', package_share=lambda _package: str(share))


@pytest.mark.parametrize(
    'change',
    [
        {'backend': 'isaac'},
        {'backend': 'isaac', 'depth_fidelity': 'd455'},
        {'backend': 'isaac', 'sim_mode': 'deterministic'},
        {'gz_gui': False},
    ],
)
def test_resume_signature_changes_with_backend_or_backend_settings(tmp_path, change) -> None:
    baseline_document = _document(tmp_path)
    if change.get('backend') == 'isaac' and len(change) > 1:
        baseline_document['defaults']['backend'] = 'isaac'
    baseline = parse_sweep(baseline_document)
    document = _document(tmp_path)
    document['defaults'].update(change)
    changed = parse_sweep(document)

    assert (
        sweep_module._resume_signature(baseline, baseline.configs, _environment(baseline))
        != sweep_module._resume_signature(changed, changed.configs, _environment(changed)))


def test_resume_signature_pins_native_world_and_catalog_bytes(tmp_path) -> None:
    share = tmp_path / 'ridgeback_autonomy_gz'
    (share / 'sim').mkdir(parents=True)
    (share / 'sim' / 'world.sdf').write_text('<sdf version="1.9"/>\n', encoding='utf-8')
    catalog = share / 'sim' / 'benchmark_assets.yaml'

    def write_catalog(native, comment=''):
        catalog.write_text(
            f'schema: ridgeback_benchmark_assets/1\n{comment}'
            'worlds:\n'
            '  target_distance_calibration:\n'
            f'    {{native_name: {native}, resource: world.sdf, format: sdf}}\n'
            'targets:\n'
            '  g1: {resource: world.sdf, format: sdf}\n',
            encoding='utf-8')

    spec = parse_sweep(_document(tmp_path))

    def signature():
        environment = _environment(spec, package_share=lambda _package: str(share))
        return sweep_module._resume_signature(spec, spec.configs, environment)

    write_catalog('target_distance_calibration')
    original = signature()
    write_catalog('target_distance_calibration', comment='# edited\n')
    assert signature()['simulation']['asset_catalog_sha256'] != (
        original['simulation']['asset_catalog_sha256'])
    write_catalog('another_native_world')
    assert signature()['simulation']['native_world'] == 'another_native_world'
    assert signature() != original


def test_resume_finds_no_sweep_recorded_under_another_backend(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(sweep_module, 'git_provenance', lambda _root: {})
    monkeypatch.setattr(sweep_module, 'gl_renderer_provenance', lambda: {})
    monkeypatch.setattr(sweep_module, 'simulator_provenance', lambda _backend: {})
    source = tmp_path / 'sweep.yaml'
    source.write_text('stable sweep bytes\n', encoding='utf-8')
    gz = parse_sweep(_document(tmp_path), source=str(source))
    document = _document(tmp_path)
    document['defaults']['backend'] = 'isaac'
    isaac = parse_sweep(document, source=str(source))
    output_root = tmp_path / 'sweeps'
    sweep_dir = output_root / '20261018_120000_unit'
    sweep_dir.mkdir(parents=True)
    sweep_module._write_manifest(str(sweep_dir), sweep_module._new_manifest(
        gz, gz.configs, str(sweep_dir), str(tmp_path), _environment(gz)))

    assert sweep_module._find_resumable_sweep(
        str(output_root), gz, gz.configs, _environment(gz)) == str(sweep_dir)
    assert sweep_module._find_resumable_sweep(
        str(output_root), isaac, isaac.configs, _environment(isaac)) is None


def test_dry_run_validates_the_backend_without_launching_or_cleaning(
    tmp_path, monkeypatch, capsys,
) -> None:
    source = tmp_path / 'sweep.yaml'
    source.write_text(
        'sweep: {name: unit}\n'
        f'defaults: {{scenario: {_scenario(tmp_path)}, repeats: 1, backend: isaac}}\n'
        'configs: [{name: pointcloud, estimators: pointcloud}]\n',
        encoding='utf-8')
    looked_up = []

    def share(package):
        looked_up.append(package)
        return _source_share(package)

    def refuse(*_args, **_kwargs):
        raise AssertionError('a dry run must not launch, clean up, or run commands')

    monkeypatch.setattr(backends, '_default_package_share', share)
    monkeypatch.setattr(sweep_module, 'run_preflight_cleanup', refuse)
    monkeypatch.setattr(sweep_module, 'run_sweep', refuse)
    monkeypatch.setattr(sweep_module.subprocess, 'Popen', refuse)
    monkeypatch.setattr(sweep_module.subprocess, 'run', refuse)
    monkeypatch.setattr(sweep_module, 'gl_renderer_provenance', lambda: {})

    assert sweep_module.main([str(source), '--dry-run']) == 0

    output = capsys.readouterr().out
    assert 'Backend isaac: world "target_distance_calibration" runs as ' \
        '"g1_distance_calibration"' in output
    assert 'depth_fidelity=ideal' in output
    assert looked_up == ['ridgeback_autonomy_isaac']


def test_dry_run_rejects_hardware_before_any_lookup(tmp_path, monkeypatch, capsys) -> None:
    source = tmp_path / 'sweep.yaml'
    source.write_text(
        'sweep: {name: unit}\n'
        f'defaults: {{scenario: {_scenario(tmp_path)}, backend: hardware}}\n'
        'configs: [{name: pointcloud, estimators: pointcloud}]\n',
        encoding='utf-8')
    monkeypatch.setattr(
        backends, '_default_package_share',
        lambda package: pytest.fail(f'looked up {package}'))

    assert sweep_module.main([str(source), '--dry-run']) == 2
    assert 'cannot run the target-distance benchmark' in capsys.readouterr().err


class _SweepSceneControl:
    """Records the order of the sweep's readiness steps and scene operations."""

    def __init__(self, events, *, clock_error=None):
        self.events = events
        self.clock_error = clock_error

    def wait_for_services(self, _timeout):
        self.events.append('services')

    def require_capabilities(self):
        self.events.append('capabilities')

    def wait_for_advancing_clock(self, _timeout):
        self.events.append('clock')
        if self.clock_error is not None:
            raise self.clock_error

    def readiness_timeline(self):
        return {'clock_advance_after_services_sec': 1.5, 'clock_advanced_wall': 2.0}


def _run_sweep_harness(tmp_path, monkeypatch, scene_control, events, backend='isaac'):
    from types import SimpleNamespace as Namespace

    document = _document(tmp_path)
    document['defaults'].update(backend=backend, output_dir=str(tmp_path / 'out'))
    spec = parse_sweep(document, source=str(tmp_path / 'sweep.yaml'))
    commands = []

    class FakeProcess:
        returncode = 0

        def __init__(self, command, **_kwargs):
            commands.append(command)
            if command[3] == 'target_benchmark_config.launch.py':
                arguments = dict(token.split(':=', 1) for token in command[4:])
                run = Path(arguments['output_dir']) / arguments['run_dir_name']
                run.mkdir(parents=True)
                (run / 'run.json').write_text(
                    '{"run": {}, "parameters": {}, "estimators": []}', encoding='utf-8')

        def poll(self):
            return None

    monkeypatch.setenv('RIDGEBACK_WORKSPACE', str(tmp_path))
    monkeypatch.setattr(sweep_module.subprocess, 'Popen', FakeProcess)
    monkeypatch.setattr(sweep_module.time, 'sleep', lambda _: None)
    monkeypatch.setattr(sweep_module, 'git_provenance', lambda _: {})
    monkeypatch.setattr(sweep_module, 'gl_renderer_provenance', lambda: {})
    monkeypatch.setattr(sweep_module, 'simulator_provenance', lambda _backend: {})
    monkeypatch.setattr(sweep_module, 'stop_process_group', lambda *_: None)
    monkeypatch.setattr(sweep_module, 'run_preflight_cleanup', lambda _root: None)
    monkeypatch.setattr(
        sweep_module, 'reap_orphan_benchmark_entities',
        lambda _client: events.append('reap') or [])
    monkeypatch.setattr(sweep_module, 'sample_real_time_factor', lambda: 1.0)
    monkeypatch.setattr(sweep_module, '_wait_for_config', lambda *_: ('exited', None))
    monkeypatch.setattr(sweep_module.rclpy, 'init', lambda: None)
    monkeypatch.setattr(
        sweep_module.rclpy, 'create_node', lambda _name: Namespace(destroy_node=lambda: None))
    monkeypatch.setattr(sweep_module.rclpy, 'ok', lambda: False)
    namespaces = []
    monkeypatch.setattr(
        sweep_module, 'create_scene_control_client',
        lambda _node, namespace: namespaces.append(namespace) or scene_control)
    share = tmp_path / 'install/ridgeback_autonomy/share/ridgeback_autonomy'
    sweep_dir, success = sweep_module.run_sweep(
        spec, spec.configs, package_share=str(share), simulation=_environment(spec))
    manifest = json.loads((Path(sweep_dir) / 'sweep.json').read_text(encoding='utf-8'))
    return manifest, success, commands, namespaces


def test_sweep_waits_for_the_simulation_clock_before_reaping(tmp_path, monkeypatch) -> None:
    events = []

    manifest, success, commands, namespaces = _run_sweep_harness(
        tmp_path, monkeypatch, _SweepSceneControl(events), events)

    assert success
    assert namespaces == ['/isaac_sim']
    assert events == ['services', 'capabilities', 'clock', 'reap']
    assert manifest['environment_readiness']['clock_advance_after_services_sec'] == 1.5
    simulation = manifest['simulation']
    assert simulation['backend'] == 'isaac'
    assert simulation['public_world'] == 'target_distance_calibration'
    assert simulation['native_world'] == 'g1_distance_calibration'
    assert simulation['asset_catalog_schema'] == 'ridgeback_benchmark_assets/1'
    assert simulation['settings'] == ISAAC_SETTINGS
    assert manifest['resume_signature']['simulation']['native_world'] == (
        'g1_distance_calibration')
    assert [command[3] for command in commands] == [
        'target_benchmark_env.launch.py', 'target_benchmark_config.launch.py']


def test_a_stalled_clock_fails_the_sweep_before_any_scene_operation(
    tmp_path, monkeypatch,
) -> None:
    events = []
    stalled = SceneControlError(
        SceneControlFailure.TIMEOUT, 'wait for the simulation clock to advance',
        '/clock stayed at 0.000 s; no scene operation was sent')

    manifest, success, commands, _namespaces = _run_sweep_harness(
        tmp_path, monkeypatch, _SweepSceneControl(events, clock_error=stalled), events)

    assert not success
    assert events == ['services', 'capabilities', 'clock']
    assert manifest['sweep']['status'] == 'failed'
    assert 'no scene operation was sent' in manifest['sweep']['error']
    assert [command[3] for command in commands] == ['target_benchmark_env.launch.py']
