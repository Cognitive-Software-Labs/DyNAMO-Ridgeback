from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

from ridgeback_autonomy.benchmarking.target_benchmark_sweep import (
    _parser,
    _launch_argument_tokens,
    run_preflight_cleanup,
)
from ridgeback_autonomy.benchmarking.sweep import load_sweep, parse_sweep


REPO_ROOT = Path(__file__).resolve().parents[3]


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
        'detector_fps', 'detector_debug',
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


@pytest.mark.parametrize(
    ('estimators', 'field', 'value'),
    [
        ('pointcloud', 'depth_source', 'monocular'),
        ('pointcloud', 'mask_depth_max_meters', 10.0),
        ('pointcloud', 'mask_gate', 'silhouette'),
        ('pointcloud', 'isolation_3d', 'range_band'),
        ('euclidean_reconstruction', 'isolation_2d', 'otsu'),
        ('projective_ranging', 'isolation_3d', 'range_band'),
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
