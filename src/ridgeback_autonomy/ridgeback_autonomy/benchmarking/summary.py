from __future__ import annotations

import csv
import statistics

import numpy as np

from ridgeback_autonomy.benchmarking.estimators import PUBLIC_ESTIMATOR_ORDER


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
]


def write_trial_csv(path: str, rows: list[dict]) -> None:
    with open(path, 'w', encoding='utf-8', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=TRIAL_CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def build_summary_rows(estimator_rows: dict[str, list[dict]]) -> list[dict]:
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
        })
    return summary_rows


def write_summary_csv(path: str, rows: list[dict]) -> None:
    with open(path, 'w', encoding='utf-8', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=SUMMARY_CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
