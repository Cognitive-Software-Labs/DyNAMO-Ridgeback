"""Human-readable markdown report for one benchmark run.

The readable view of a run. ``run.json`` holds the same numbers for machines,
at full precision; this one is formatted for a person, which is exactly why it
is not a data source: values are rounded, nulls print as an em dash, rows are
keyed by display name, and the layout changes whenever the report improves.

Its own reason to exist is the per-scene table -- which estimator failed on
which robot -- a view that otherwise requires joining several CSVs by hand.

Pure module (no ROS, no I/O): takes the rows the runner already built and
returns a string.
"""

from __future__ import annotations

import os

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    ESTIMATOR_LABELS,
    PUBLIC_ESTIMATOR_ORDER,
)
from ridgeback_autonomy.benchmarking.scoring import OUTCOME_NO_VALUE, OUTCOME_SCORED
from ridgeback_autonomy.common.miss_reason import (
    MissReason,
    observation_totals,
    reason_name,
)


MISS_MARK = '✗'  # ✗


def format_metres(value) -> str:
    return f'{float(value):.3f}' if value is not None else '—'


def format_percent(value) -> str:
    return f'{float(value) * 100:.1f}%' if value is not None else '—'


def markdown_table(headers: list[str], rows: list[list[str]]) -> str:
    """A pipe table padded to even columns, so the RAW file reads as a table too."""

    widths = [len(header) for header in headers]
    for row in rows:
        for index, cell in enumerate(row):
            widths[index] = max(widths[index], len(cell))

    def line(cells: list[str]) -> str:
        padded = [cell.ljust(widths[index]) for index, cell in enumerate(cells)]
        return f'| {" | ".join(padded)} |'

    separator = f'|{"|".join("-" * (width + 2) for width in widths)}|'
    return '\n'.join([line(headers), separator] + [line(row) for row in rows])


def selected_in_canonical_order(estimator_rows: dict[str, list[dict]]) -> list[str]:
    return [
        estimator for estimator in PUBLIC_ESTIMATOR_ORDER
        if estimator in estimator_rows
    ]


def render_accuracy_table(summary_rows: list[dict], display_names: dict[str, str]) -> str:
    rows = [
        [
            display_names.get(row['estimator'], row['estimator']),
            f'{row["scored_count"]}/{row["trial_count"]}',
            format_metres(row['mean_abs_error_m']),
            format_metres(row['median_abs_error_m']),
            format_metres(row['p95_abs_error_m']),
        ]
        for row in summary_rows
    ]
    return markdown_table(['Estimator', 'Scored', 'MAE', 'Median', 'P95'], rows)


def render_reliability_table(summary_rows: list[dict], display_names: dict[str, str]) -> str:
    rows = [
        [
            display_names.get(row['estimator'], row['estimator']),
            format_percent(row['observation_coverage']),
            str(row['missed_instance_count']),
            str(row['detector_missed_count']),
            str(row['gate_missed_count']),
            str(row['no_value_missed_count']),
        ]
        for row in summary_rows
    ]
    return markdown_table(
        ['Estimator', 'Coverage', 'Missed', 'Detector', 'Gate', 'No value'], rows)


def render_failure_reasons(
    status_histograms: dict[str, dict[int, int]],
    display_names: dict[str, str],
    selected: list[str],
) -> str:
    """One bullet per estimator that failed anything, worst reason first.

    Replaces the JSON blob in the CSV cell -- parseable, but not readable.
    """

    lines = []
    for estimator in selected:
        code_counts = status_histograms.get(estimator) or {}
        total, ok = observation_totals(code_counts)
        misses = {
            code: count for code, count in code_counts.items()
            if code != int(MissReason.OK) and count
        }
        if not misses:
            continue
        tally = ', '.join(
            f'{reason_name(code)} ×{count}'
            for code, count in sorted(misses.items(), key=lambda item: -item[1])
        )
        name = display_names.get(estimator, estimator)
        lines.append(f'- **{name}** — {tally} ({ok} OK of {total})')

    if not lines:
        return 'Every estimator produced a value on every detected box.'
    return '\n'.join(lines)


def render_per_scene_table(estimator_rows: dict[str, list[dict]], selected: list[str]) -> str:
    """Scene x instance down the rows, estimators across the columns.

    The view that needed a manual join across per-estimator CSVs, and the one
    where a single estimator's failure against its peers is visible at a glance.
    """

    # Key on the trial so repeats stay distinct rows, but label with the scene.
    by_instance: dict[tuple, dict] = {}
    for estimator in selected:
        for row in estimator_rows.get(estimator, []):
            key = (row['trial_id'], row['instance_index'])
            entry = by_instance.setdefault(
                key,
                {
                    'scene_id': row['scene_id'],
                    'instance_index': row['instance_index'],
                    'true_distance_m': row['true_distance_m'],
                    'cells': {},
                },
            )
            entry['cells'][estimator] = cell_for(row)

    rows = [
        [
            entry['scene_id'],
            str(entry['instance_index']),
            format_metres(entry['true_distance_m']),
        ] + [entry['cells'].get(estimator, '—') for estimator in selected]
        # Insertion order, which is trial execution order, which is the order
        # the scenes are declared in the spec. Sorting would shuffle a curated
        # set (a control scene placed first would land wherever its name falls).
        for entry in by_instance.values()
    ]
    headers = ['Scene', 'Inst', 'True'] + [ESTIMATOR_LABELS[e] for e in selected]
    return markdown_table(headers, rows)


def cell_for(row: dict) -> str:
    """A scored estimate, or the marked reason it is missing."""

    if row.get('outcome') == OUTCOME_SCORED and row.get('trial_estimate_m') is not None:
        return format_metres(row['trial_estimate_m'])
    if row.get('outcome') == OUTCOME_NO_VALUE and not row.get('miss_reason'):
        return f'{MISS_MARK} {OUTCOME_NO_VALUE} (reason unknown)'
    return f'{MISS_MARK} {row.get("miss_reason") or row.get("outcome") or "miss"}'


def render_key_value_table(entries: dict[str, str], key_header: str, value_header: str) -> str:
    rows = [[str(key), str(value)] for key, value in entries.items()]
    return markdown_table([key_header, value_header], rows)


def render_run_report(
    run_label: str,
    scenario_path: str,
    summary_rows: list[dict],
    estimator_rows: dict[str, list[dict]],
    status_histograms: dict[str, dict[int, int]],
    display_names: dict[str, str],
    included_trials: int,
    skipped_trials: int,
    scenes: int,
    instances: int,
    metadata: dict[str, str],
    parameters: dict[str, str],
) -> str:
    selected = selected_in_canonical_order(estimator_rows)
    provenance = dict(metadata)
    provenance['Scenario'] = os.path.basename(provenance.get('Scenario', scenario_path))
    provenance['Scenes'] = str(scenes)
    provenance['Instances'] = str(instances)
    provenance['Trials'] = f'{included_trials} ({skipped_trials} skipped)'

    return '\n'.join([
        f'# Benchmark run {run_label}',
        '',
        render_key_value_table(provenance, 'Run', ''),
        '',
        '## Accuracy',
        '',
        render_accuracy_table(summary_rows, display_names),
        '',
        'Errors are metres against the sim ground truth, over the instances each',
        'estimator actually scored. `Scored` is that count out of the instances it',
        'was asked about, so the gap is its miss total.',
        '',
        '## Reliability',
        '',
        render_reliability_table(summary_rows, display_names),
        '',
        '`Coverage` is the share of detected boxes this estimator measured. The',
        'three miss columns split why an instance went unscored: `Detector` means',
        'nobody located it, `Gate` means this estimator located something too far',
        'from the robot to be it, `No value` means it produced nothing at all.',
        '',
        # Not "why estimators failed": an estimator that scored every instance
        # still shows the odd unmeasured box here, and calling that a failure
        # would misread it.
        '## Why boxes went unmeasured',
        '',
        render_failure_reasons(status_histograms, display_names, selected),
        '',
        # Ahead of the per-scene table because that table is the only section
        # that grows without bound -- ~92 rows for the 68-scene v2 set -- so a
        # fixed-size block placed after it cannot be reached without scrolling
        # past the whole thing.
        '## Run configuration',
        '',
        'Every parameter the runner was launched with.',
        '',
        render_key_value_table(parameters, 'Parameter', 'Value'),
        '',
        '## Per scene',
        '',
        render_per_scene_table(estimator_rows, selected),
        '',
    ])
