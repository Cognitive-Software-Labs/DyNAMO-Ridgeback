from __future__ import annotations

import json

from ridgeback_autonomy.benchmarking.sweep_report import (
    collect_comparison_rows,
    render_sweep_report,
    write_sweep_report,
)


def _estimator(key, mae):
    return {
        'key': key,
        'display_name': key.replace('_', ' ').title(),
        'trial_count': 10,
        'scored_count': 8,
        'mean_abs_error_m': mae,
        'median_abs_error_m': mae - 0.01,
        'p95_abs_error_m': mae + 0.1,
        'mean_rel_error': mae / 4.0,
        'missed': {'total': 2, 'detector': 1, 'gate': 1, 'no_value': 0},
        'observations': {'total': 20, 'ok': 16, 'coverage': 0.8},
        'reason_histogram': {},
    }


def _write_run(path, estimators):
    path.mkdir()
    (path / 'run.json').write_text(json.dumps({
        'run': {'label': path.name},
        'parameters': {'estimators': ','.join(row['key'] for row in estimators)},
        'estimators': estimators,
    }), encoding='utf-8')


def _manifest():
    return {
        'sweep': {
            'name': 'synthetic',
            'status': 'complete_with_failures',
            'source': '/tmp/sweep.yaml',
            'started': '2026-08-28 10:00:00',
            'finished': '2026-08-28 11:00:00',
        },
        'provenance': {'commit': 'abc1234', 'branch': 'main', 'dirty_count': 0},
        'configs': [
            {'name': 'single', 'status': 'success', 'output_path': 'single',
             'wall_time_sec': 100.0, 'real_time_factor': 0.95},
            {'name': 'multi', 'status': 'success', 'output_path': 'multi',
             'wall_time_sec': 200.0, 'real_time_factor': 0.75},
            {'name': 'failed', 'status': 'failed', 'output_path': 'failed',
             'error': 'runner exited'},
        ],
    }


def test_sweep_rollup_handles_multi_estimator_and_missing_run(tmp_path) -> None:
    _write_run(tmp_path / 'single', [_estimator('polar_profiling', 0.25)])
    _write_run(tmp_path / 'multi', [
        _estimator('pointcloud', 0.40),
        _estimator('projective_ranging', 0.10),
    ])

    rows, notes = collect_comparison_rows(_manifest(), str(tmp_path))

    assert [(row['config'], row['estimator']) for row in rows] == [
        ('multi', 'Projective Ranging'),
        ('single', 'Polar Profiling'),
        ('multi', 'Pointcloud'),
    ]
    assert any('failed' in note and 'runner exited' in note for note in notes)
    assert rows[0]['rtf'] == 0.75


def test_sweep_report_is_regenerable_from_manifest_and_runs(tmp_path) -> None:
    _write_run(tmp_path / 'single', [_estimator('polar_profiling', 0.25)])
    _write_run(tmp_path / 'multi', [
        _estimator('pointcloud', 0.40),
        _estimator('projective_ranging', 0.10),
    ])
    manifest = _manifest()
    (tmp_path / 'sweep.json').write_text(json.dumps(manifest), encoding='utf-8')

    report_path = write_sweep_report(str(tmp_path))
    report = (tmp_path / 'summary.md').read_text(encoding='utf-8')

    assert report_path == str(tmp_path / 'summary.md')
    assert '# Benchmark sweep synthetic' in report
    assert 'Cross-config comparison' in report
    assert report.index('Projective Ranging') < report.index(
        'Polar Profiling') < report.index('Pointcloud')
    assert 'runner exited' in report
    assert '0.750' in report


def test_render_sweep_report_marks_skipped_completed_config(tmp_path) -> None:
    _write_run(tmp_path / 'single', [_estimator('polar_profiling', 0.25)])
    manifest = _manifest()
    manifest['configs'] = [
        {'name': 'single', 'status': 'skipped', 'output_path': 'single'},
    ]

    report = render_sweep_report(manifest, str(tmp_path))

    assert 'existing run.json reused' in report
    assert 'Polar Profiling' in report
