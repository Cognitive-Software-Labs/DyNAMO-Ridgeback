from __future__ import annotations

from ridgeback_autonomy.benchmarking.report import (
    cell_for,
    markdown_table,
    render_failure_reasons,
    render_per_scene_table,
    render_run_report,
)
from ridgeback_autonomy.benchmarking.scoring import (
    OUTCOME_GATE_MISS,
    OUTCOME_NO_VALUE,
    OUTCOME_SCORED,
)
from ridgeback_autonomy.common.miss_reason import MissReason


def _row(scene, instance, estimator_label, outcome, estimate=None, reason=None, trial=None):
    return {
        'trial_id': trial or f'{scene}_rep1',
        'scene_id': scene,
        'instance_index': instance,
        'true_distance_m': 3.0,
        'estimator': estimator_label,
        'outcome': outcome,
        'miss_reason': reason,
        'trial_estimate_m': estimate,
    }


def test_markdown_table_pads_columns_so_the_raw_file_is_readable() -> None:
    table = markdown_table(['A', 'Long header'], [['x', 'y']])

    lines = table.splitlines()
    assert lines[0] == '| A | Long header |'
    assert lines[2] == '| x | y           |'
    # Every row is the same width, which is the point of padding at all.
    assert len({len(line) for line in lines}) == 1


def test_cell_shows_the_estimate_or_the_marked_reason() -> None:
    scored = _row('s', 0, 'LiDAR', OUTCOME_SCORED, estimate=2.5)
    no_value = _row('s', 0, 'LiDAR', OUTCOME_NO_VALUE, reason='SCAN_INVALID')
    gate = _row('s', 0, 'LiDAR', OUTCOME_GATE_MISS)

    assert cell_for(scored) == '2.500'
    # no_value carries a MissReason; gate_miss has none, so the outcome names it.
    assert cell_for(no_value).endswith('SCAN_INVALID')
    assert cell_for(gate).endswith('gate_miss')


def test_per_scene_table_keeps_execution_order_not_alphabetical() -> None:
    # A curated set puts its control scene first; sorting by name would bury it.
    estimator_rows = {
        'lidar': [
            _row('zulu_control', 0, 'LiDAR', OUTCOME_SCORED, estimate=3.0),
            _row('alpha_blocked', 0, 'LiDAR', OUTCOME_GATE_MISS),
        ],
    }

    table = render_per_scene_table(estimator_rows, ['lidar'])

    body = [line for line in table.splitlines() if line.startswith('| zulu')
            or line.startswith('| alpha')]
    assert body[0].startswith('| zulu_control')
    assert body[1].startswith('| alpha_blocked')


def test_per_scene_table_puts_each_estimator_in_its_own_column() -> None:
    estimator_rows = {
        'rgb': [_row('blocked', 0, 'RGB', OUTCOME_SCORED, estimate=3.2)],
        'lidar': [_row('blocked', 0, 'LiDAR', OUTCOME_GATE_MISS)],
    }

    table = render_per_scene_table(estimator_rows, ['rgb', 'lidar'])

    header, _sep, row = table.splitlines()
    assert 'RGB' in header and 'LiDAR' in header
    # One instance, one row: the estimator that scored and the one that missed
    # sit side by side, which is the view the CSVs could not give.
    assert '3.200' in row and 'gate_miss' in row


def test_failure_reasons_orders_by_count_and_skips_clean_estimators() -> None:
    histograms = {
        'lidar': {int(MissReason.OK): 70},
        'polar_profiling': {
            int(MissReason.OK): 30,
            int(MissReason.UNSET): 8,
            int(MissReason.TOO_FEW_RAYS_SELECTED): 32,
        },
    }

    text = render_failure_reasons(
        histograms, {'polar_profiling': 'polar'}, ['lidar', 'polar_profiling'])

    assert 'lidar' not in text  # nothing went unmeasured for it
    assert text.index('TOO_FEW_RAYS_SELECTED') < text.index('UNSET')
    assert '30 OK of 70' in text


def test_failure_reasons_says_so_when_nothing_was_missed() -> None:
    text = render_failure_reasons({'lidar': {int(MissReason.OK): 5}}, {}, ['lidar'])

    assert 'Every estimator produced a value' in text


def test_run_report_has_every_section_and_survives_null_metrics() -> None:
    # An estimator that scored nothing has None for every error metric; the
    # report must render it rather than crash formatting a null.
    summary_rows = [{
        'estimator': 'lidar', 'trial_count': 2, 'scored_count': 0,
        'mean_abs_error_m': None, 'median_abs_error_m': None,
        'p95_abs_error_m': None, 'missed_instance_count': 2,
        'detector_missed_count': 2, 'gate_missed_count': 0,
        'no_value_missed_count': 0, 'observation_coverage': 0.0,
    }]
    estimator_rows = {'lidar': [_row('occ', 0, 'LiDAR', 'detector_miss')]}

    report = render_run_report(
        run_label='20260822_202851',
        scenario_path='/tmp/verify_gate.yaml',
        summary_rows=summary_rows,
        estimator_rows=estimator_rows,
        status_histograms={'lidar': {int(MissReason.UNSET): 4}},
        display_names={'lidar': 'LiDAR'},
        included_trials=1,
        skipped_trials=0,
        scenes=1,
        instances=1,
        metadata={
            'Started': '2026-08-22 20:28:51',
            'Commit': 'e1bdc17 + 22 uncommitted file(s)',
            'Branch': 'g1-distance-benchmarks',
            'Scenario': '/tmp/verify_gate.yaml',
        },
        parameters={'mask_gate': 'box', 'repeats': '1'},
    )

    for heading in ('# Benchmark run 20260822_202851', '## Accuracy',
                    '## Reliability', '## Why boxes went unmeasured', '## Per scene',
                    '## Run configuration'):
        assert heading in report
    # Configuration precedes the per-scene table: that table is the only one
    # that grows without bound (~92 rows for the 68-scene v2 set), so anything
    # placed after it is unreachable without scrolling past all of it.
    assert report.index('## Run configuration') < report.index('## Per scene')
    # Provenance: enough to identify the run later.
    assert '2026-08-22 20:28:51' in report
    assert 'e1bdc17 + 22 uncommitted file(s)' in report
    assert 'g1-distance-benchmarks' in report
    assert 'verify_gate.yaml' in report
    # Parameters are dumped verbatim so the run can be reproduced.
    assert 'mask_gate' in report and 'repeats' in report
    assert '—' in report          # null metrics rendered, not crashed
    assert '0.0%' in report
