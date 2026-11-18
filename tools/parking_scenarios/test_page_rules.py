"""Rules the two parking pages rely on, on synthetic runs and captures (no evidence folders).

Run: python3 -m pytest tools/parking_scenarios/test_page_rules.py
"""
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
yaml = pytest.importorskip('yaml')
import make_results_page as results  # noqa: E402
import make_scenarios_page as scenes  # noqa: E402
import scenario_metrics  # noqa: E402


def run(outcome='MISSION_COMPLETE', reason='returned to start pose', scenario='standard', **m):
    base = {'outcome': outcome, 'final_reason': reason, 'scenario': scenario, 'attempts': 1,
            'returned_to_start': True, 'simulator': 'gz', 'commit': 'b519ab0', 'world': 'parking_demo'}
    base.update(m)
    return {'m': base, 'vetoes': [], 'log': ''}


def test_a_completion_outside_the_reach_band_is_not_counted_complete():
    assert results.classify(run(reach_truth_m=0.607))[2] == results.REACH_OUTSIDE
    assert results.classify(run(reach_truth_m=0.509))[2] == 'Complete'
    assert results.classify(run(reach_truth_m=1.717))[2] == results.FALSE_COMPLETION


def test_a_parked_run_whose_stop_was_not_confirmed_and_a_stopped_run_keep_their_cause():
    late = run('MISSION_FAILED', 'returned to start after: parking attempt 3/3: PARKED deadline expired',
               attempts=3)
    assert results.classify(late)[2] == results.STOP_UNCONFIRMED
    assert results.classify(run('stopped', ''))[2] == results.OPERATOR_STOP


def test_only_a_safe_refusal_without_an_attempt_counts_as_one():
    refusal = run('MISSION_FAILED', 'returned to start after: no_valid_candidate: no feasible parking pose',
                  scenario='no_feasible', attempts=0)
    assert results.classify(refusal)[2] == 'Safe refusal'
    missed = run('MISSION_FAILED', 'returned to start after: parking attempt 1/3: lidar_clearance',
                 scenario='no_feasible', localization_errors_m=[0.57])
    assert results.classify(missed)[2] == results.MISSED_LOCALIZATION
    lost = run('MISSION_FAILED', 'returned to start after: target_localization_timeout', scenario='no_feasible',
               attempts=0)
    assert results.classify(lost)[2] == 'Perception failure'


def test_warehouse_runs_are_named_and_never_show_captures():
    warehouse = run(world=None, commit='5b5133c')
    assert results.sim_name(warehouse) == 'Gazebo warehouse'
    assert not results.ward_run(warehouse)
    assert results.ward_run(run(world=None, commit='822ae6d'))     # the 2026-11-17 ward trial
    assert results.sim_name(run()) == 'Gazebo ward'


def test_featured_specs_name_ward_runs_only():
    for spec in results.FEATURED + results.ARCHIVED_FEATURED:
        assert 'hospital' in spec['run'], spec['run']


def test_scene_captures_of_another_world_or_layout_are_skipped(tmp_path):
    data = yaml.safe_load(open(scenes.scene_views.SCENARIO_FILE))
    manifest = [
        {'ok': True, 'stage': 'scene', 'backend': 'gz', 'scenario': 'standard', 'view': 'closeup',
         'world': 'depot', 'file': 'a.png'},
        {'ok': True, 'stage': 'scene', 'backend': 'gz', 'scenario': 'standard', 'view': 'task',
         'file': 'b.png'},                                    # no world recorded: the warehouse
        {'ok': True, 'stage': 'scene', 'backend': 'gz', 'scenario': 'standard', 'view': 'workstation',
         'world': 'parking_demo', 'file': 'c.png'},
        {'ok': False, 'stage': 'scene', 'backend': 'gz', 'scenario': 'standard', 'view': 'isometric',
         'world': 'parking_demo', 'file': 'd.png'},
    ]
    (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    (tmp_path / 'gz_standard_commit.txt').write_text('HEAD\n')
    best = scenes.load_captures([str(tmp_path)], data)
    assert set(best) == {('gz', 'standard', 'workstation')}
    moved = yaml.safe_load(open(scenes.scene_views.SCENARIO_FILE))
    moved['scenarios']['standard']['objects'][0]['forward'] += 0.3
    assert scenes.placement(moved, 'gz', 'standard') != scenes.placement(data, 'gz', 'standard')


def test_metrics_read_a_run_stopped_by_hand(tmp_path):
    d = tmp_path / 'gz_no_feasible'
    d.mkdir()
    (d / 'result.txt').write_text('stopped\n')
    (d / 'console.log').write_text('')
    (d / 'commit.txt').write_text('f568789\n')
    m = scenario_metrics.run(str(d))
    assert (m['scenario'], m['simulator'], m['outcome']) == ('no_feasible', 'gz', 'no mission transitions')
