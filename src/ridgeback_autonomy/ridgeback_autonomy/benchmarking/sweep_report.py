"""Cross-configuration markdown report for a benchmark sweep."""

from __future__ import annotations

import json
import math
import os

from ridgeback_autonomy.benchmarking.report import (
    format_metres,
    format_percent,
    markdown_table,
    render_key_value_table,
)


def load_json(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as handle:
        document = json.load(handle)
    if not isinstance(document, dict):
        raise ValueError(f'Expected a JSON object in {path}.')
    return document


def _config_output_path(entry: dict, sweep_dir: str) -> str:
    path = str(entry.get('output_path') or entry.get('name') or '')
    if not os.path.isabs(path):
        path = os.path.join(sweep_dir, path)
    return os.path.abspath(path)


def collect_comparison_rows(manifest: dict, sweep_dir: str) -> tuple[list[dict], list[str]]:
    """Read completed run documents into sortable comparison rows."""

    rows: list[dict] = []
    notes: list[str] = []
    for entry in manifest.get('configs', []):
        name = str(entry.get('name', '<unnamed>'))
        status = str(entry.get('status', 'unknown'))
        output_path = _config_output_path(entry, sweep_dir)
        run_json_path = os.path.join(output_path, 'run.json')
        if not os.path.isfile(run_json_path):
            error = entry.get('error') or 'run.json is missing'
            notes.append(f'- **{name}** — {status}: {error}')
            continue
        try:
            run_document = load_json(run_json_path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            notes.append(f'- **{name}** — invalid run.json: {exc}')
            continue

        if status not in {'success', 'skipped'}:
            notes.append(f'- **{name}** — {status}: completed output was retained')
        elif status == 'skipped':
            notes.append(f'- **{name}** — skipped: existing run.json reused')

        estimators = run_document.get('estimators') or []
        if not isinstance(estimators, list):
            notes.append(f'- **{name}** — invalid run.json: "estimators" is not a list')
            continue
        for estimator in estimators:
            if not isinstance(estimator, dict):
                continue
            observations = estimator.get('observations') or {}
            missed = estimator.get('missed') or {}
            rows.append({
                'config': name,
                'estimator': str(estimator.get('display_name') or estimator.get('key') or '—'),
                'mean_abs_error_m': estimator.get('mean_abs_error_m'),
                'median_abs_error_m': estimator.get('median_abs_error_m'),
                'p95_abs_error_m': estimator.get('p95_abs_error_m'),
                'mean_rel_error': estimator.get('mean_rel_error'),
                'scored_count': int(estimator.get('scored_count') or 0),
                'trial_count': int(estimator.get('trial_count') or 0),
                'coverage': observations.get('coverage'),
                'missed': int(missed.get('total') or 0),
                'rtf': entry.get('real_time_factor'),
                'wall_time_sec': entry.get('wall_time_sec'),
            })

    rows.sort(key=lambda row: (
        row['mean_abs_error_m'] is None,
        math.inf if row['mean_abs_error_m'] is None else float(row['mean_abs_error_m']),
        row['config'],
        row['estimator'],
    ))
    return rows, notes


def _format_rtf(value) -> str:
    return f'{float(value):.3f}' if value is not None else '—'


def _format_duration(seconds) -> str:
    if seconds is None:
        return '—'
    total = max(0, int(round(float(seconds))))
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    if hours:
        return f'{hours}h {minutes:02d}m'
    if minutes:
        return f'{minutes}m {secs:02d}s'
    return f'{secs}s'


def render_sweep_report(manifest: dict, sweep_dir: str) -> str:
    """Render the regenerable comparison report for one sweep directory."""

    rows, notes = collect_comparison_rows(manifest, sweep_dir)
    sweep = manifest.get('sweep') or {}
    provenance = manifest.get('provenance') or {}
    commit = provenance.get('commit') or 'unknown'
    dirty_count = provenance.get('dirty_count')
    if dirty_count:
        commit = f'{commit} + {dirty_count} uncommitted file(s)'

    metadata = {
        'Name': str(sweep.get('name') or os.path.basename(sweep_dir)),
        'Status': str(sweep.get('status') or 'unknown'),
        'Started': str(sweep.get('started') or '—'),
        'Finished': str(sweep.get('finished') or '—'),
        'Source': str(sweep.get('source') or '—'),
        'Source SHA-256': str(sweep.get('source_sha256') or '—'),
        'Commit': str(commit),
        'Branch': str(provenance.get('branch') or 'unknown'),
    }

    table_rows = [
        [
            row['config'],
            row['estimator'],
            format_metres(row['mean_abs_error_m']),
            format_metres(row['median_abs_error_m']),
            format_metres(row['p95_abs_error_m']),
            format_percent(row['mean_rel_error']),
            f'{row["scored_count"]}/{row["trial_count"]}',
            format_percent(row['coverage']),
            str(row['missed']),
            _format_rtf(row['rtf']),
            _format_duration(row['wall_time_sec']),
        ]
        for row in rows
    ]
    comparison = markdown_table(
        ['Config', 'Estimator', 'MAE', 'Median', 'P95', 'Mean rel',
         'Scored', 'Coverage', 'Missed', 'RTF', 'Wall'],
        table_rows,
    )

    notes_text = '\n'.join(notes) if notes else 'Every selected configuration completed in this sweep.'
    return '\n'.join([
        f'# Benchmark sweep {metadata["Name"]}',
        '',
        render_key_value_table(metadata, 'Sweep', ''),
        '',
        '## Cross-config comparison',
        '',
        comparison,
        '',
        'Rows are sorted by mean absolute error. RTF is sampled immediately',
        'before the configuration and is recorded for diagnosing coverage drift;',
        'it never changes execution.',
        '',
        '## Failed and skipped configurations',
        '',
        notes_text,
        '',
    ])


def write_sweep_report(sweep_dir: str, manifest: dict | None = None) -> str:
    """Regenerate ``summary.md`` from ``sweep.json`` and completed runs."""

    sweep_dir = os.path.abspath(os.path.expanduser(sweep_dir))
    if manifest is None:
        manifest = load_json(os.path.join(sweep_dir, 'sweep.json'))
    output_path = os.path.join(sweep_dir, 'summary.md')
    with open(output_path, 'w', encoding='utf-8') as handle:
        handle.write(render_sweep_report(manifest, sweep_dir))
    return output_path
