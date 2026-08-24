from __future__ import annotations

import csv
import json
import statistics

import numpy as np

from ridgeback_autonomy.benchmarking.estimators import PUBLIC_ESTIMATOR_ORDER
from ridgeback_autonomy.benchmarking.scoring import (
    OUTCOME_DETECTOR_MISS,
    OUTCOME_GATE_MISS,
    OUTCOME_NO_VALUE,
)
from ridgeback_autonomy.common.miss_reason import observation_totals, reason_name


TRIAL_CSV_COLUMNS = [
    'trial_id',
    'repeat_index',
    'scene_id',
    'instance_index',
    'spawn_world_x',
    'spawn_world_y',
    'spawn_yaw_rad',
    # Ground truth in the robot's planar frame, all three off the robot front
    # (the reference every estimator publishes against).
    'true_forward_m',
    'true_lateral_m',
    'true_distance_m',
    'estimator',
    'outcome',
    # Only set when ``outcome`` is ``no_value`` -- the case where a MissReason
    # exists. The other outcomes are already fully named by ``outcome`` itself.
    'miss_reason',
    'trial_estimate_m',
    'abs_error_m',
    'rel_error',
    # Frame yield for this trial: how many frames this estimator produced a
    # value on, out of how many the window captured.
    'usable_aligned_events',
    'frames_captured',
    'image_path',
]

def write_trial_csv(path: str, rows: list[dict]) -> None:
    with open(path, 'w', encoding='utf-8', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=TRIAL_CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def build_summary_rows(
    estimator_rows: dict[str, list[dict]],
    extra_detection_count: int = 0,
    outcome_counts: dict[str, dict[str, int]] | None = None,
    status_histograms: dict[str, dict[int, int]] | None = None,
) -> list[dict]:
    # ``trial_count`` counts every instance this estimator was asked about;
    # ``scored_count`` counts the ones it answered, so the gap between them IS
    # the miss total. That total is broken out per outcome (detector / gate /
    # no_value) because pooling "occluded" with "reported the wrong object"
    # hides which one a scene actually exercised, and ``missed_instance_count``
    # is derived from those three rather than counted separately -- two paths to
    # the same total can only ever disagree. extra stays scene-level (a detector
    # concept).
    outcome_counts = outcome_counts or {}
    status_histograms = status_histograms or {}
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
        counts = outcome_counts.get(estimator, {})
        code_counts = status_histograms.get(estimator, {})
        observations, observations_ok = observation_totals(code_counts)
        detector_missed = counts.get(OUTCOME_DETECTOR_MISS, 0)
        gate_missed = counts.get(OUTCOME_GATE_MISS, 0)
        no_value_missed = counts.get(OUTCOME_NO_VALUE, 0)
        summary_rows.append({
            'estimator': estimator,
            'trial_count': len(rows),
            'scored_count': len(abs_errors),
            'mean_abs_error_m': mean_abs_error,
            'median_abs_error_m': median_abs_error,
            'p95_abs_error_m': p95_abs_error,
            'mean_rel_error': mean_rel_error,
            'missed_instance_count': detector_missed + gate_missed + no_value_missed,
            'detector_missed_count': detector_missed,
            'gate_missed_count': gate_missed,
            'no_value_missed_count': no_value_missed,
            'extra_detection_count': extra_detection_count,
            'observations': observations,
            'observations_ok': observations_ok,
            'observation_coverage': (
                (observations_ok / observations) if observations else 0.0),
        })
    return summary_rows


def build_run_document(
    summary_rows: list[dict],
    run_metadata: dict,
    parameters: dict[str, str],
    display_names: dict[str, str],
    status_histograms: dict[str, dict[int, int]] | None = None,
) -> dict:
    """The run's numbers as one machine-readable document.

    Replaces the old ``comparison_summary.csv``, which duplicated most of
    ``summary.md`` while being a poor machine format for this shape: it embedded
    JSON inside a cell for the reason histogram, and mixed instance-level
    accuracy with box-level observation counts in one flat row. Here the two
    granularities are separate objects and the histogram is a real mapping.

    Provenance and parameters live here too. They used to exist only in the
    markdown report, which is a presentation format -- rounded to three
    decimals, nulls written as an em dash, rows keyed by display name, layout
    changing whenever the report is improved -- so "which runs used silhouette,
    on which commit" was unanswerable without parsing prose.
    """

    status_histograms = status_histograms or {}
    return {
        'run': dict(run_metadata),
        'parameters': dict(parameters),
        'estimators': [
            {
                # Both names on purpose: ``key`` is the stable identifier to
                # query on, ``display_name`` is what the report prints. Carrying
                # both is what lets the runner stop rewriting the key in place.
                'key': row['estimator'],
                'display_name': display_names.get(row['estimator'], row['estimator']),
                'trial_count': row['trial_count'],
                'scored_count': row['scored_count'],
                'mean_abs_error_m': row['mean_abs_error_m'],
                'median_abs_error_m': row['median_abs_error_m'],
                'p95_abs_error_m': row['p95_abs_error_m'],
                'mean_rel_error': row['mean_rel_error'],
                'missed': {
                    'total': row['missed_instance_count'],
                    'detector': row['detector_missed_count'],
                    'gate': row['gate_missed_count'],
                    'no_value': row['no_value_missed_count'],
                },
                'extra_detection_count': row['extra_detection_count'],
                'observations': {
                    'total': row['observations'],
                    'ok': row['observations_ok'],
                    'coverage': row['observation_coverage'],
                },
                'reason_histogram': {
                    reason_name(code): count
                    for code, count in sorted(
                        (status_histograms.get(row['estimator']) or {}).items())
                },
            }
            for row in summary_rows
        ],
    }


def write_run_json(path: str, document: dict) -> None:
    with open(path, 'w', encoding='utf-8') as json_file:
        json.dump(document, json_file, indent=2, sort_keys=False)
        json_file.write('\n')
