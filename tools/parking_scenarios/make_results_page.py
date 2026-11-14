"""Build docs/exploration/autonomous_parking_results.html from recorded scenario runs.

Usage: make_results_page.py --runs RUN_DIR [RUN_DIR ...] [--images RUN_DIR ...]
Every run gets a row of recorded metrics (scenario_metrics.py); missing measurements
are shown as unavailable, never estimated. Runs listed under --images also get their
stage screenshots, each verified before use: it must not be blank, and it is labelled
with the mission state that was active when it was captured (from the mission's own
log), so an image is never shown as a stage it does not show. A top-down view is
shown as the parked position only if the robot was PARKED at that moment.
"""
import argparse
import datetime
import glob
import json
import os
import re
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pages  # noqa: E402
from capture_scene_views import check  # noqa: E402
from scenario_metrics import run as metrics  # noqa: E402

OUT = os.path.join(pages.DOCS, pages.RESULTS_PAGE)
ASSETS = os.path.join(pages.DOCS, 'assets/parking_results')
ORDER = ['standard', 'worktable', 'right_blocked', 'left_blocked', 'both_sides_blocked',
         'restricted', 'no_feasible', 'heading_90', 'heading_away']
# Recorded HEAD -> (campaign, the code that ran). A run folder's code.txt overrides it
# when that run's working tree differed from the campaign's final code.
CODE = {'fe60a88': ('P7 scenarios (G1 in front of the start)',
                    'fe60a88 plus the changes committed as 28769c2'),
        '622a0c0': ('Detection gaps (G1 in front of the start)',
                    '622a0c0 plus the changes committed as 2ecaecf'),
        'bb8aa94': ('Workstation scenes (G1 out of sight)',
                    'bb8aa94 plus the changes committed as b4f855b'),
        'b4f855b': ('Workstation scenes (G1 out of sight)', 'b4f855b'),
        '8dd2a1e': ('G1 position check (G1 out of sight)',
                    '8dd2a1e plus the changes committed as 3becd27'),
        'a67d3c6': ('Reacquisition and recorded demonstration (G1 out of sight)',
                    'a67d3c6 plus the changes committed as 640e8ec'),
        '64b3176': ('After merging the group dev (P8)', '64b3176'),
        'c1435d9': ('After merging the group dev (P8)', 'c1435d9'),
        '99387ae': ('After merging the group dev (P8)', '99387ae'),
        '5b5133c': ('After merging the group dev (P8)', '5b5133c')}
LATEST = ('5b5133c', '99387ae', 'c1435d9', '64b3176')


def run_name(run):
    """A run folder's name; recorded demonstrations keep their run in OUT_DIR/run."""
    name = os.path.basename(os.path.normpath(run))
    return os.path.basename(os.path.dirname(os.path.normpath(run))) if name == 'run' else name
STAGES = [('EXPLORING', 'Searching for G1'), ('G1_LOCALIZED', 'G1 detected and localized'),
          ('NAVIGATING_TO_PREPARKING', 'Parking pose selected: rays, footprint and route'),
          ('NAVIGATING_TO_PARKING', 'Pre-parking pose reached'),
          ('PARKED', 'Parked at the selected pose'), ('END', 'Mission result')]
# The states a stage's screenshot may show; otherwise the image is labelled by the state
# that was active when it was captured, never by the stage it was meant to show.
STAGE_STATES = {
    'EXPLORING': {'SEARCH_G1'},
    'G1_LOCALIZED': {'G1_LOCALIZED', 'EXPLORING_AROUND_G1', 'PARKING_CANDIDATES_READY',
                     'EVALUATING_PARKING_PATHS', 'PARKING_PATH_FEASIBLE'},
    'NAVIGATING_TO_PREPARKING': {'NAVIGATING_TO_PREPARKING'},
    'NAVIGATING_TO_PARKING': {'NAVIGATING_TO_PARKING'},
    'PARKED': {'PARKED'},
    'END': {'MISSION_COMPLETE', 'MISSION_FAILED', 'PARKING_PLANNING_FAILED',
            'PARKING_EXECUTION_FAILED', 'MOTION_SETTLEMENT_FAILED', 'MISSION_CANCELED'},
}
TRANSITION = re.compile(r'\[(\d+\.\d+)\] \[[^\]]*mission_manager_node\]: '
                        r'Mission state transition: (\w+) -> (\w+);')


def badge(m):
    reason = m.get('final_reason') or ''
    back = ', returned' if m.get('returned_to_start') else ', stopped'
    if m['outcome'] == 'MISSION_COMPLETE':
        # The mission's own success can rest on a wrong G1 estimate; Gazebo truth shows it.
        reach, error = m.get('reach_truth_m'), m.get('localization_error_m')
        if reach is not None and not 0.30 - 0.10 <= reach <= 0.60 + 0.10:
            return 'bad', (f'false completion: G1 mislocalized'
                           f'{"" if error is None else f" {error:.2f} m"}, parked {reach:.2f} m from G1')
        attempts = m.get('attempts') or 0
        return 'ok', 'parked, returned' if attempts <= 1 else f'parked on attempt {attempts}, returned'
    vetoes = sum('lidar' in f for f in m.get('attempt_failures') or [])
    if vetoes or 'lidar_clearance' in reason:
        vetoes = max(vetoes, m.get('attempts') or 1)
        then = '; observation return timed out' if 'observation pose' in reason else ''
        return 'warn', f'LiDAR safety monitor cancelled {vetoes} attempt{"s" if vetoes > 1 else ""}{then}{back}'
    if 'target_position_unverified' in reason or 'unverified' in reason:
        return 'bad', 'perception: G1 position not verified' + back
    if 'no_valid_candidate' in reason or 'no feasible' in reason or 'bounded_search_exhausted' in reason:
        return 'safe', 'refused to park: no safe pose or route' + back
    if m['outcome'] == 'timeout' and not m.get('attempts'):
        return 'bad', 'G1 not found within the run limit'
    if 'localization' in reason or 'not relocalized' in reason:
        return 'bad', 'perception: G1 not localized' + back
    return 'warn', 'stopped by a check' + back


def state_at(transitions, t):
    state = 'WAITING_FOR_START_POSE'
    for when, _, new in transitions:
        if when <= t:
            state = new
    return state


def fmt(value, unit='', missing='not recorded'):
    if value is None or value == []:
        return missing
    if isinstance(value, list):
        value = min(value)
    return f'{value}{unit}'


def stage_images(run, m, transitions, tag):
    """[(stage label, image path, caption)] for the verified captures of one run."""
    out = []
    stage_times = {}
    if os.path.exists(f'{run}/shots/stages.txt'):
        for line in open(f'{run}/shots/stages.txt'):
            name, when = line.split()
            stage_times.setdefault(name.split('_')[0] if name.startswith('END') else name,
                                   (name, float(when)))
    for stage, label in STAGES:
        if stage not in stage_times:
            continue
        name, when = stage_times[stage]
        files = sorted(glob.glob(f'{run}/shots/{name}_rviz2_*.png'), key=os.path.getsize)
        if not files:
            continue
        import cv2
        rgb = cv2.imread(files[-1])[:, :, ::-1]
        ok, why = check(rgb)
        if not ok:
            print(f'{tag} {stage}: excluded ({why})')
            continue
        actual = state_at(transitions, when)
        if actual not in STAGE_STATES[stage]:
            label = f'Captured during {actual}'
        rel = pages.export_jpeg(files[-1], os.path.join(ASSETS, f'{tag}_{stage.lower()}.jpg'), 1300)
        out.append((label, rel, f'RViz, mission state at capture: {actual} '
                                f'(+{when - transitions[0][0]:.0f} s)'))
    manifest = f'{run}/shots/manifest.json'
    for entry in json.load(open(manifest)) if os.path.exists(manifest) else []:
        if not entry.get('ok'):
            continue
        actual = state_at(transitions, entry['time'])
        if entry['stage'] == 'parked' and actual != 'PARKED':
            print(f'{tag} parked view excluded: captured in {actual}')
            continue
        label = {'parked': 'Actual parked position, ground truth (top-down)',
                 'end': 'Robot after the mission (top-down)'}[entry['stage']]
        rel = pages.export_jpeg(f'{run}/shots/{entry["file"]}', os.path.join(
            ASSETS, f'{tag}_{entry["stage"]}_{entry["view"]}.jpg'), 1100)
        sim = 'Gazebo' if m['simulator'] == 'gz' else 'Isaac Sim'
        out.append((f'{label}, {entry["view"]}', rel,
                    f'{sim} server-rendered view, mission state at capture: {actual}'))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', nargs='+', required=True)
    ap.add_argument('--images', nargs='*', default=[])
    args = ap.parse_args()
    image_runs = {os.path.abspath(r) for r in args.images}
    rows, cards = [], []
    data = []
    for run in args.runs:
        m = metrics(run)
        log = open(f'{run}/console.log', errors='replace').read()
        transitions = [(float(t), a, b) for t, a, b in TRANSITION.findall(log)]
        start = transitions[0][0] if transitions else os.path.getmtime(f'{run}/result.txt')
        campaign, code = CODE.get(m['commit'], ('unlisted', m['commit']))
        if os.path.exists(f'{run}/code.txt'):
            code = open(f'{run}/code.txt').readline().strip()
        data.append((m, run, transitions, start, campaign, code))
    data.sort(key=lambda d: (d[0]['commit'] not in LATEST,
                             ORDER.index(d[0]['scenario']) if d[0]['scenario'] in ORDER else 99,
                             d[0]['simulator'], d[3]))
    for m, run, transitions, start, campaign, code in data:
        tag = f'{m["simulator"]}_{m["scenario"]}_{m["commit"]}_{run_name(run)}'
        kind, label = badge(m)
        latest = m['commit'] in LATEST
        when = datetime.datetime.fromtimestamp(start).strftime('%Y-%m-%d %H:%M')
        truth = m['simulator'] == 'gz'
        reach = fmt(m.get('reach_truth_m'), ' m', 'no truth (Isaac)' if not truth else 'not parked')
        error = fmt(m.get('pose_error_m'), ' m', 'no truth (Isaac)' if not truth else 'not parked')
        clearance = (f"{fmt(m.get('min_truth_clearance_m'), ' m')} ({m.get('min_truth_clearance_pair', '')})"
                     if truth else f"LiDAR {fmt(m.get('lidar_min_m'), ' m', 'not recorded')}")
        failure = '' if m['outcome'] == 'MISSION_COMPLETE' else (m.get('final_reason') or m['outcome'])
        g1_error = fmt(m.get('localization_error_m'), ' m',
                       'no truth (Isaac)' if not truth else 'not localized')
        heading_error = fmt(m.get('heading_error_deg'), ' deg',
                            'no truth (Isaac)' if not truth else 'no heading')
        cells = [f'<a href="#{tag}">{pages.esc(m["scenario"])}</a>' if latest else pages.esc(m['scenario']),
                 'Gazebo' if truth else 'Isaac', pages.esc(campaign), f'<code>{pages.esc(run_name(run))}</code>',
                 when, pages.esc(m['outcome']), f'<span class="badge {kind}">{pages.esc(label)}</span>',
                 pages.esc(', '.join(m.get('selected_sides') or []) or '-'), fmt(m.get('attempts')),
                 fmt(m.get('exploration_s'), ' s'), fmt(m.get('mission_s'), ' s'), g1_error, heading_error,
                 reach, error,
                 pages.esc(clearance), fmt(m.get('returned_to_start')), pages.esc(failure[:140])]
        rows.append(f'<tr{"" if latest else " class=\"historical\""}>'
                    + ''.join(f'<td>{c}</td>' for c in cells) + '</tr>')
        if os.path.abspath(run) not in image_runs:
            continue
        figures = stage_images(run, m, transitions, tag)
        facts = [('Simulator', 'Gazebo' if truth else 'Isaac Sim'), ('Code', code),
                 ('Run', run_name(run)), ('Started', when), ('Final state', m['outcome']),
                 ('Sides tried (estimated)', ', '.join(m.get('selected_sides') or []) or '-'),
                 ('Attempts', fmt(m.get('attempts'))),
                 ('Exploration / mission', f"{fmt(m.get('exploration_s'), ' s')} / {fmt(m.get('mission_s'), ' s')}"),
                 ('G1 position used', ', '.join(f'({x:.2f}, {y:.2f})' for x, y in m.get('g1_estimates') or [])
                  or 'none'),
                 ('G1 localization error, ground truth', g1_error),
                 ('G1 heading estimate / error', f"{fmt(m.get('g1_heading_log'), ' deg', 'none')} / {heading_error}"),
                 ('Positions re-seeded on the head / refused unverified',
                  f"{m.get('reseeded_on_head', 0)} / {m.get('unverified_localizations', 0)}"),
                 ('Reach, ground truth', reach), ('Bearing from G1 heading, ground truth',
                                                  fmt(m.get('bearing_from_g1_heading_deg'), ' deg', 'not available')),
                 ('Parked pose error to the goal', error), ('Minimum clearance', clearance),
                 ('Minimum LiDAR return, final leg', fmt(m.get('lidar_min_m'), ' m')),
                 ('Returned to start', fmt(m.get('returned_to_start'))),
                 ('Failure reason', failure or '-')]
        fact_html = ''.join(f'<div><b>{pages.esc(k)}:</b> {pages.esc(v)}</div>' for k, v in facts)
        figs = ''.join(f'<div>{pages.figure(rel, f"{label}. {caption}")}</div>'
                       for label, rel, caption in figures) or pages.missing('No verified screenshots')
        cards.append(f'<section class="card" id="{tag}"><header><h3>{pages.esc(m["scenario"])}</h3>'
                     f'<span class="badge {kind}">{pages.esc(label)}</span>'
                     f'<span class="meta">{"Gazebo" if truth else "Isaac"} · {pages.esc(run_name(run))}</span>'
                     f'</header><p class="note">{pages.esc(m["description"])}. Expected: {pages.esc(m["expected"])}.</p>'
                     f'<div class="facts">{fact_html}</div><div class="stages">{figs}</div></section>')
    head = ['Scenario', 'Simulator', 'Campaign', 'Run', 'Date', 'Final state', 'Outcome', 'Sides tried',
            'Attempts', 'Exploration', 'Mission', 'G1 error', 'Heading error', 'Reach', 'Pose error',
            'Min clearance', 'Returned',
            'Failure reason']
    intro = (
        '<p>What happened when the mission ran each scenario. Rows list every recorded run by the code '
        'that ran; the newest campaign comes first, earlier campaigns stay as historical evidence. '
        'G1 error, heading error, reach, pose error and clearance are Gazebo ground truth, sampled every '
        '1-2.5 s and never given to the mission; Isaac runs record no ground truth, so those cells say so. The scene layouts are '
        f'on the <a href="{pages.SCENARIOS_PAGE}">scenarios page</a>.</p>'
        '<p class="note">Screenshots appear only for runs whose captures were verified: each image is a '
        'non-blank capture of that run, and its caption names the mission state active when it was '
        'taken. Screenshots of the earlier campaigns were withdrawn after an audit found simulator views '
        'dominated by a bystander, wrong areas and overlapping overlays; their runs, logs and images stay '
        'in the dated records and run folders.</p>'
        '<p class="note">A video of one complete run, <code>isaac_standard_run3</code> (2026-11-11, '
        '<code>MISSION_COMPLETE</code>, parked at G1\'s right), shows its Isaac views, RViz and mission '
        'state together at 8x speed: <code>artifacts/exploration/demo_2026-11-11/isaac_standard_run3/'
        'demo.mp4</code>, kept outside the repository; the same recording of the merged code\'s Isaac '
        'run is <code>artifacts/exploration/p8_2026-11-14/isaac_standard/demo.mp4</code>. '
        '<code>tools/parking_scenarios/record_demo_run.sh</code> records another.</p>')
    legend = ('<p><span class="badge ok">parked, returned</span> parking completed &nbsp; '
              '<span class="badge safe">refused to park</span> the planner found no safe pose and said why &nbsp; '
              '<span class="badge bad">perception</span> G1 not (re)localized in time, its position not '
              'verified in depth, or localized in the wrong place (ground truth); a <em>false completion</em> '
              'is a run the mission reported complete although it parked beside a wrong G1 position &nbsp; '
              '<span class="badge warn">LiDAR safety monitor</span> the live LiDAR check cancelled parking '
              'attempts &nbsp; <span class="badge warn">stopped by a check</span> another safety, planning or '
              'precision check</p>')
    body = (f'<main class="shell"><h2>All runs</h2>{legend}<div class="wrap"><table><tr>'
            + ''.join(f'<th>{h}</th>' for h in head) + f'</tr>{"".join(rows)}</table></div>'
            f'<h2>Runs with verified screenshots</h2>{"".join(cards)}'
            '<p class="note">Records: <a href="../../archive/engineering/2026-11-05-parking-scenarios.md">P7 '
            'scenarios</a>, <a href="../../archive/engineering/2026-11-05-parking-detection-gaps.md">detection '
            'gaps</a>, <a href="../../archive/engineering/2026-11-05-parking-workstation-scenes.md">workstation '
            'scenes</a>, <a href="../../archive/engineering/2026-11-09-parking-g1-position-check.md">G1 '
            'position check</a>, <a href="../../archive/engineering/2026-11-11-parking-reacquisition-and-demo.md">'
            'reacquisition and demonstration</a>, <a href="../../archive/engineering/2026-11-14-parking-dev-integration.md">'
            'after merging dev</a>.</p></main>')
    with open(OUT, 'w') as stream:
        stream.write(pages.page('Autonomous parking: experiments and results', pages.RESULTS_PAGE, intro, body))
    print('wrote', OUT, len(data), 'runs', len(cards), 'with screenshots')


if __name__ == '__main__':
    main()
