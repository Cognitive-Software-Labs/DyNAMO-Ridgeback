from __future__ import annotations

import csv
import json
import statistics

import numpy as np

from ridgeback_autonomy.benchmarking.estimators import PUBLIC_ESTIMATOR_ORDER
from ridgeback_autonomy.common.miss_reason import MissReason, reason_name


TRIAL_CSV_COLUMNS = [
    'trial_id',
    'repeat_index',
    'scene_id',
    'instance_index',
    'spawn_forward_m',
    'spawn_lateral_m',
    'spawn_world_x',
    'spawn_world_y',
    'spawn_yaw_rad',
    'true_forward_m',
    'true_lateral_m',
    'true_distance_m',
    'estimator',
    'trial_estimate_m',
    'abs_error_m',
    'rel_error',
    'usable_aligned_events',
    'image_path',
]

SUMMARY_CSV_COLUMNS = [
    'estimator',
    'trial_count',
    'mean_abs_error_m',
    'median_abs_error_m',
    'p95_abs_error_m',
    'mean_rel_error',
    'missed_instance_count',
    'extra_detection_count',
]

COVERAGE_CSV_COLUMNS = [
    'estimator',
    'events',
    'ok',
    'coverage',
    'reason_histogram',
]


def format_reason_histogram(code_counts: dict[int, int] | None) -> str:
    """JSON of ``reason name -> count``, ordered by code, for a CSV cell."""

    code_counts = code_counts or {}
    return json.dumps(
        {reason_name(code): count for code, count in sorted(code_counts.items())})


def write_trial_csv(path: str, rows: list[dict]) -> None:
    with open(path, 'w', encoding='utf-8', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=TRIAL_CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def build_summary_rows(
    estimator_rows: dict[str, list[dict]],
    missed_instance_counts: dict[str, int] | None = None,
    extra_detection_count: int = 0,
) -> list[dict]:
    # missed counts are per estimator: detector-level misses (instance never
    # matched by any detection) plus estimator-level misses (matched but this
    # estimator produced no value). extra stays scene-level (detector concept).
    # Reason histograms live in coverage.csv only (single source of truth).
    missed_instance_counts = missed_instance_counts or {}
    summary_rows: list[dict] = []
    for estimator in PUBLIC_ESTIMATOR_ORDER:
        rows = estimator_rows.get(estimator)
        if rows is None:
            continue

        abs_errors = [row['abs_error_m'] for row in rows if row.get('abs_error_m') is not None]
        rel_errors = [row['rel_error'] for row in rows if row.get('rel_error') is not None]
        if abs_errors:
            mean_abs_error = statistics.fmean(abs_errors)
            median_abs_error = statistics.median(abs_errors)
            p95_abs_error = float(np.percentile(np.asarray(abs_errors, dtype=np.float64), 95))
        else:
            mean_abs_error = None
            median_abs_error = None
            p95_abs_error = None

        mean_rel_error = statistics.fmean(rel_errors) if rel_errors else None
        summary_rows.append({
            'estimator': estimator,
            'trial_count': len(rows),
            'mean_abs_error_m': mean_abs_error,
            'median_abs_error_m': median_abs_error,
            'p95_abs_error_m': p95_abs_error,
            'mean_rel_error': mean_rel_error,
            'missed_instance_count': missed_instance_counts.get(estimator, 0),
            'extra_detection_count': extra_detection_count,
        })
    return summary_rows


def build_coverage_rows(status_histograms: dict[str, dict[int, int]]) -> list[dict]:
    """One row per estimator over all captured events: OK count and coverage."""

    coverage_rows: list[dict] = []
    for estimator in PUBLIC_ESTIMATOR_ORDER:
        code_counts = status_histograms.get(estimator)
        if code_counts is None:
            continue
        events = sum(code_counts.values())
        ok = code_counts.get(int(MissReason.OK), 0)
        coverage_rows.append({
            'estimator': estimator,
            'events': events,
            'ok': ok,
            'coverage': (ok / events) if events else 0.0,
            'reason_histogram': format_reason_histogram(code_counts),
        })
    return coverage_rows


def write_summary_csv(path: str, rows: list[dict]) -> None:
    with open(path, 'w', encoding='utf-8', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=SUMMARY_CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_coverage_csv(path: str, rows: list[dict]) -> None:
    with open(path, 'w', encoding='utf-8', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=COVERAGE_CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
