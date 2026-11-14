"""Build docs/exploration/autonomous_parking_results.html: the experimental results dashboard.

Usage: make_results_page.py [RUN_DIR ...]   (default: the runs in results_runs.txt)

Sections: an overview of the post-merge runs (the final code), Gazebo against Isaac,
three featured experiments with their own screenshots and timelines, charts drawn from
recorded measurements, and every recorded run in a filterable table.

Every number comes from scenario_metrics.py (the run's console log; Gazebo ground truth
sampled every 1-2.5 s) or the run's mission-state transitions; nothing is estimated.
Every screenshot belongs to the run it illustrates and is checked before use: not blank,
and captured in a mission state the caption names (RViz stage shots and simulator views
by their logged capture time, recording frames by the state the recorder logged).
"""
import datetime
import glob
import html
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '../..'))
sys.path.insert(0, HERE)
import pages  # noqa: E402
from capture_scene_views import check  # noqa: E402
from scenario_metrics import run as metrics  # noqa: E402

OUT = os.path.join(pages.DOCS, pages.RESULTS_PAGE)
ASSETS = os.path.join(pages.DOCS, 'assets/parking_results')
RUN_LIST = os.path.join(HERE, 'results_runs.txt')
SCENARIO_FILE = os.path.join(ROOT, 'src/ridgeback_autonomy/config/parking_scenarios.yaml')
REACH = (0.30, 0.60)
FALSE_COMPLETION = 'False completion: incorrect G1 localization (historical bug)'
STALE_LIDAR = 'Stale LiDAR scans: mission node overloaded'

# Recorded HEAD -> (campaign, the code that ran, archive record). A run folder's code.txt
# overrides the code when that run's working tree differed from the campaign's code.
CODE = {'fe60a88': ('P7 scenarios', 'fe60a88 plus the changes committed as 28769c2',
                    '2026-11-05-parking-scenarios.md'),
        '622a0c0': ('Detection gaps', '622a0c0 plus the changes committed as 2ecaecf',
                    '2026-11-05-parking-detection-gaps.md'),
        'bb8aa94': ('Workstation scenes', 'bb8aa94 plus the changes committed as b4f855b',
                    '2026-11-05-parking-workstation-scenes.md'),
        'b4f855b': ('Workstation scenes', 'b4f855b', '2026-11-05-parking-workstation-scenes.md'),
        '8dd2a1e': ('G1 position check', '8dd2a1e plus the changes committed as 3becd27',
                    '2026-11-09-parking-g1-position-check.md'),
        'a67d3c6': ('Reacquisition and demonstration', 'a67d3c6 plus the changes committed as 640e8ec',
                    '2026-11-11-parking-reacquisition-and-demo.md'),
        '64b3176': ('After merging dev (P8)', '64b3176', '2026-11-14-parking-dev-integration.md'),
        'c1435d9': ('After merging dev (P8)', 'c1435d9', '2026-11-14-parking-dev-integration.md'),
        '99387ae': ('After merging dev (P8)', '99387ae', '2026-11-14-parking-dev-integration.md'),
        '5b5133c': ('After merging dev (P8)', '5b5133c', '2026-11-14-parking-dev-integration.md')}
# The post-merge runs of 2026-11-14 (merged code 64b3176 and its three fixes): the set
# every overview number on the page is computed from.
FINAL_SET = ('64b3176', 'c1435d9', '99387ae', '5b5133c')
FINAL_CODE = '5b5133c'
TRANSITION = re.compile(r'\[(\d+\.\d+)\] \[[^\]]*mission_manager_node\]: '
                        r'Mission state transition: (\w+) -> (\w+); reason=([^\n]*)')
ATTEMPT_FAIL = re.compile(r'\[(\d+\.\d+)\] \[[^\]]*mission_manager_node\]: Parking attempt (\d)/\d '
                          r'failed: ([^;\n]+)')
VETO_DETAIL = re.compile(r'Parking LiDAR veto in \w+: \w+ \(nearest return ([\d.]+) m, G1 gap ([\d.]+) m\)')
PHASES = (('search', 'Search for G1'), ('locate', 'Localize, map and plan'),
          ('park', 'Parking attempts'), ('wait', 'Parked wait'), ('return', 'Return home'))
SHORT = {'Complete': 'complete', 'Safe refusal': 'safe refusal', 'Perception failure': 'G1 not localized',
         'Planning gate': 'plan expired (fixed)', STALE_LIDAR: 'stale LiDAR (fixed)'}
VETO_TEXT = {'lidar_obstacle_on_route': 'a LiDAR return lay on the planned route',
             'lidar_obstacle_in_stopping_path': 'a LiDAR return lay in the stopping path',
             'g1_protected_region': "the robot reached G1's protected region"}

# The three featured experiments: the run, and the captures shown for it (file in the
# run's shots folder, or a recording frame) with the mission states they must show.
FEATURED = [
    {'key': 'isaac_side', 'run': 'artifacts/exploration/p8_2026-11-14/isaac_standard/run',
     'title': 'A. Side parking in Isaac Sim',
     'images': [('simview', 'isaac_standard_parked_workstation.png', {'PARKED'},
                 'Isaac top-down view at the parked pose: Ridgeback at G1\'s left, its basket toward G1'),
                ('frame', 'isometric', {'PARKED'},
                 'Isaac angled view from the run\'s recording, while parked'),
                ('rviz', 'PARKED', {'PARKED'},
                 'RViz while parked: the parking donut, the selected footprint and the state machine')]},
    {'key': 'gazebo_rear', 'run': 'artifacts/exploration/p8_2026-11-14/gz_standard_run5',
     'title': 'B. Rear parking in Gazebo Sim',
     'images': [('simview', 'gz_standard_parked_workstation.png', {'PARKED'},
                 'Gazebo top-down view at the parked pose: G1 facing its worktable, Ridgeback behind it'),
                ('rviz', 'PARKED', {'PARKED'},
                 'RViz while parked: the parking donut, the selected footprint and the state machine'),
                ('simview', 'gz_standard_end_overview.png', {'MISSION_COMPLETE'},
                 'Gazebo overview after the mission: Ridgeback back at its start')]},
    {'key': 'gazebo_refusal', 'run': 'artifacts/exploration/p8_2026-11-14/gz_no_feasible',
     'title': 'C. No feasible parking position (Gazebo Sim)',
     'images': [('simview', 'gz_no_feasible_end_workstation.png', {'MISSION_FAILED'},
                 'Gazebo top-down view of the scene after the mission: G1 between two panels, '
                 'its worktable ahead and a bed behind'),
                ('rviz', 'G1_LOCALIZED', {'EXPLORING_AROUND_G1'},
                 'RViz after the first localization, while the robot maps the floor around G1: the parking '
                 'donut, with no valid pose among the cells already mapped'),
                ('simview', 'gz_no_feasible_end_overview.png', {'MISSION_FAILED'},
                 'Gazebo overview after the mission: Ridgeback back near its start')]},
]


def esc(text):
    return html.escape(str(text))


def run_name(run):
    """A run folder's name; recorded demonstrations keep their run in OUT_DIR/run."""
    name = os.path.basename(os.path.normpath(run))
    return os.path.basename(os.path.dirname(os.path.normpath(run))) if name == 'run' else name


def load(run):
    path = run if os.path.isabs(run) else os.path.join(ROOT, run)
    m = metrics(path)
    log = open(os.path.join(path, 'console.log'), errors='replace').read()
    transitions = [(float(t), a, b, w) for t, a, b, w in TRANSITION.findall(log)]
    campaign, code, record = CODE.get(m['commit'], ('unlisted', m['commit'], None))
    if os.path.exists(os.path.join(path, 'code.txt')):
        code = open(os.path.join(path, 'code.txt')).readline().strip()
    start = transitions[0][0] if transitions else os.path.getmtime(os.path.join(path, 'result.txt'))
    return {'path': path, 'rel': os.path.relpath(path, ROOT), 'name': run_name(path), 'm': m,
            'log': log, 'transitions': transitions, 'campaign': campaign, 'code': code,
            'record': record, 'start': start, 'final': m['commit'] in FINAL_SET,
            'vetoes': [(float(t), int(n), r.strip()) for t, n, r in ATTEMPT_FAIL.findall(log)],
            'veto_detail': [(float(a), float(b)) for a, b in VETO_DETAIL.findall(log)]}


def classify(r):
    """(badge class, short outcome, filter category) from the run's own record."""
    m = r['m']
    reason = m.get('final_reason') or ''
    attempts = m.get('attempts') or 0
    back = 'returned home' if m.get('returned_to_start') else 'stopped'
    if m['outcome'] == 'MISSION_COMPLETE':
        reach = m.get('reach_truth_m')
        if reach is not None and not REACH[0] - 0.10 <= reach <= REACH[1] + 0.10:
            return 'bad', 'False completion: incorrect G1 localization (historical bug)', FALSE_COMPLETION
        return 'ok', ('Parked and returned' if attempts <= 1 else
                      f'Parked on attempt {attempts} and returned'), 'Complete'
    if 'lidar_stale' in reason:
        return ('warn', f'Stopped: stale LiDAR scans while the mission node was overloaded (fixed in '
                        f'5b5133c); {back}', STALE_LIDAR)
    if any(k in reason for k in ('no_valid_candidate', 'no feasible', 'bounded_search_exhausted')):
        if m['scenario'] == 'no_feasible':
            return 'safe', f'Safe refusal as expected: no safe pose; {back}', 'Safe refusal'
        return ('warn', f'Refused although the scene has a parking pose (expected: {m.get("expected")}); '
                        f'{back}', 'Refused (pose expected)')
    if m['outcome'] == 'timeout':
        return 'bad', 'Run limit: G1 not found', 'G1 not found'
    if 'localization' in reason or 'not relocalized' in reason:
        return 'bad', f'Perception: G1 not localized; {back}', 'Perception failure'
    if 'observation pose' in reason:
        vetoes = len(r['vetoes'])
        return ('warn', (f'{vetoes} LiDAR veto, then ' if vetoes else '') +
                'the observation return failed; stopped', 'Observation return failed')
    if any(k in reason for k in ('target_stale', 'motion_evidence_lost', 'odometry_timestamp')):
        return 'warn', 'Plan expired before driving; stopped', 'Planning gate'
    if r['vetoes'] or 'lidar_obstacle' in reason:
        n = max(len(r['vetoes']), attempts)
        return 'warn', f'LiDAR safety vetoes on {n} attempt{"s" if n > 1 else ""}; {back}', 'LiDAR vetoes'
    return 'warn', f'Stopped by a check; {back}', 'Other check'


def phase_segments(r):
    """[(phase, start, end)] from the transitions: search until G1 is confirmed, then
    localization and planning until the first parking leg, the attempts (with their
    retries) until PARKED, the wait, and the return."""
    tr = r['transitions']
    if not tr:
        return []
    segments, phase = [], 'search'
    t_prev = tr[0][0]
    attempted = False
    for t, previous, state, _ in tr:
        if previous == 'SEARCH_G1':
            new = 'locate'
        elif state == 'NAVIGATING_TO_PREPARKING':
            attempted, new = True, 'park'
        elif state == 'PARKED':
            new = 'wait'
        elif state == 'RETURNING_TO_START':
            new = 'return'
        elif state in ('MISSION_COMPLETE', 'MISSION_FAILED', 'PARKING_PLANNING_FAILED',
                       'PARKING_EXECUTION_FAILED', 'MOTION_SETTLEMENT_FAILED', 'MISSION_CANCELED'):
            new = None
        elif phase == 'return' or (attempted and phase in ('park',)):
            new = phase
        else:
            new = 'park' if attempted else phase
        if new != phase:
            if phase is not None and t > t_prev:
                segments.append((phase, t_prev, t))
            phase, t_prev = new, t
    return segments


def state_at(r, t):
    state = 'WAITING_FOR_START_POSE'
    for when, _, new, _ in r['transitions']:
        if when <= t:
            state = new
    return state


def clock(seconds):
    seconds = max(0, int(round(seconds)))
    return f'{seconds // 60}:{seconds % 60:02d}'


# Images -----------------------------------------------------------------------------

def featured_image(r, kind, ref, states, caption, tag):
    """(relative path, caption) for one verified capture of run ``r``, or a reason why not."""
    shots = os.path.join(r['path'], 'shots')
    t0 = r['transitions'][0][0]
    if kind == 'rviz':
        stage_time = None
        for line in open(os.path.join(shots, 'stages.txt')):
            name, when = line.split()
            if name == ref:
                stage_time = float(when)
        files = sorted(glob.glob(os.path.join(shots, f'{ref}_rviz2_*.png')), key=os.path.getsize)
        if stage_time is None or not files:
            return None, f'no RViz capture at {ref}'
        src, when = files[-1], stage_time
    elif kind == 'simview':
        entry = next((e for e in json.load(open(os.path.join(shots, 'manifest.json')))
                      if e.get('file') == ref and e.get('ok')), None)
        if entry is None:
            return None, f'{ref} not captured'
        src, when = os.path.join(shots, ref), float(entry['time'])
    else:   # a recording frame: the recorder logged the mission state of every frame
        base = os.path.dirname(r['path'])
        rows = [line.strip().split(',') for line in open(os.path.join(base, 'frames.csv'))]
        rows = [row for row in rows if row[2] in states]
        if not rows:
            return None, 'no recording frame in that state'
        index, when, _ = rows[len(rows) // 2]
        when = float(when)
        frame = os.path.join(base, 'frames', f'{int(index):06d}.jpg')
        from PIL import Image
        tile = Image.open(frame).crop((960, 44, 1920, 644))     # the recorder's right view tile
        src = os.path.join(ASSETS, f'_{tag}_frame.png')
        os.makedirs(ASSETS, exist_ok=True)
        tile.save(src)
    actual = state_at(r, when)
    if actual not in states:
        return None, f'captured during {actual}, not {sorted(states)}'
    import cv2
    rgb = cv2.imread(src)[:, :, ::-1]
    ok, why = check(rgb)
    if not ok:
        return None, f'blank or invalid capture ({why})'
    rel = pages.export_jpeg(src, os.path.join(ASSETS, f'{tag}.jpg'), 1600)
    if kind == 'frame':
        os.remove(src)
    return rel, f'{caption}. Mission state {actual}, +{clock(when - t0)} after the mission started.'


# Charts (inline SVG, colours from the page's chart tokens) ------------------------------

def svg(width, height, body, label):
    return (f'<svg class="chart" viewBox="0 0 {width} {height}" role="img" aria-label="{esc(label)}" '
            f'preserveAspectRatio="xMinYMin meet">{body}</svg>')


def axis(x0, x1, y, ticks, scale, unit, label=None):
    out = [f'<line x1="{x0}" y1="{y}" x2="{x1}" y2="{y}" class="axis"/>']
    for v in ticks:
        x = scale(v)
        text = label(v) if label else f'{v:g}{unit}'
        out.append(f'<line x1="{x:.1f}" y1="12" x2="{x:.1f}" y2="{y}" class="grid"/>'
                   f'<text x="{x:.1f}" y="{y + 16}" class="tick" text-anchor="middle">{text}</text>')
    return ''.join(out)


def duration_chart(runs):
    rows = [r for r in runs if phase_segments(r)]
    label_w, width, bar_h, gap = 230, 940, 18, 16
    span = max(seg[2] - phase_segments(r)[0][1] for r in rows for seg in phase_segments(r)[-1:])
    top = 14
    x0, x1 = label_w, width - 190
    scale = lambda s: x0 + (x1 - x0) * s / span
    body = []
    for i, r in enumerate(rows):
        y = top + i * (bar_h + gap)
        segs = phase_segments(r)
        t0 = segs[0][1]
        sim = 'Gazebo' if r['m']['simulator'] == 'gz' else 'Isaac'
        body.append(f'<text x="{x0 - 10}" y="{y + 13}" class="label" text-anchor="end">'
                    f'{esc(sim)} · {esc(r["m"]["scenario"])} · {esc(r["name"])}</text>')
        for phase, a, b in segs:
            xa, xb = scale(a - t0), scale(b - t0)
            w = max(1.0, xb - xa - 2)
            body.append(f'<rect x="{xa:.1f}" y="{y}" width="{w:.1f}" height="{bar_h}" rx="3" class="ph-{phase}">'
                        f'<title>{esc(dict(PHASES)[phase])}: {b - a:.0f} s</title></rect>')
        end = segs[-1][2] - t0
        _, _, category = classify(r)
        body.append(f'<text x="{scale(end) + 8:.1f}" y="{y + 13}" class="value">{clock(end)} · '
                    f'{esc(SHORT.get(category, category))}</text>')
    y_axis = top + len(rows) * (bar_h + gap)
    ticks = [m * 60 for m in range(0, int(span // 60) + 2, 2) if m * 60 <= span]
    body.append(axis(x0, x1, y_axis, ticks, scale, '', lambda v: f'{v // 60:g} min'))
    chart = svg(width, y_axis + 30, ''.join(body), 'Mission duration by phase for the post-merge runs')
    legend = ''.join(f'<span><i class="sw ph-{k}"></i>{esc(v)}</span>' for k, v in PHASES)
    table = ''.join(
        f'<tr><td>{esc(r["name"])}</td><td>{"Gazebo" if r["m"]["simulator"] == "gz" else "Isaac"}</td>'
        + ''.join(f'<td>{sum(b - a for p, a, b in phase_segments(r) if p == k):.0f}</td>' for k, _ in PHASES)
        + f'<td>{phase_segments(r)[-1][2] - phase_segments(r)[0][1]:.0f}</td><td>{esc(classify(r)[1])}</td></tr>'
        for r in rows)
    head = ''.join(f'<th>{esc(v)} (s)</th>' for _, v in PHASES)
    return (f'<div class="legend">{legend}</div>{chart}'
            f'<details class="data"><summary>Durations by phase (table)</summary><div class="wrap"><table>'
            f'<tr><th>Run</th><th>Simulator</th>{head}<th>Total (s)</th><th>Outcome</th></tr>{table}'
            f'</table></div></details>')


def localization_chart(runs):
    """Gazebo ground-truth error of every position the mission used, per run."""
    rows = [r for r in runs if r['m']['simulator'] == 'gz' and r['m'].get('localization_errors_m')]
    label_w, width, row_h = 230, 940, 26
    x0, x1, top = label_w, width - 60, 14
    hi = 0.20
    scale = lambda v: x0 + (x1 - x0) * min(v, hi) / hi
    body = []
    for i, r in enumerate(rows):
        y = top + i * row_h + 10
        body.append(f'<text x="{x0 - 10}" y="{y + 4}" class="label" text-anchor="end">{esc(r["m"]["scenario"])} · '
                    f'{esc(r["name"])}</text><line x1="{x0}" y1="{y}" x2="{x1}" y2="{y}" class="row"/>')
        for k, e in enumerate(r['m']['localization_errors_m']):
            body.append(f'<circle cx="{scale(e):.1f}" cy="{y}" r="4.5" class="dot-gz">'
                        f'<title>{esc(r["name"])}, localization {k + 1}: {e:.3f} m from G1\'s ground truth</title></circle>')
    y_axis = top + len(rows) * row_h + 4
    body.append(axis(x0, x1, y_axis, [0, 0.05, 0.10, 0.15, 0.20], scale, ' m'))
    values = [e for r in rows for e in r['m']['localization_errors_m']]
    note = (f'{len(values)} positions in {len(rows)} post-merge Gazebo runs: '
            f'{min(values):.3f}-{max(values):.3f} m. Isaac records no ground truth, so it has no points here.')
    return svg(width, y_axis + 30, ''.join(body), 'G1 localization error against Gazebo ground truth') + \
        f'<p class="note">{esc(note)}</p>'


def attempts_chart(runs):
    """Completed missions on the current mission logic: each attempt, vetoed or parked."""
    rows = [r for r in runs if r['m']['outcome'] == 'MISSION_COMPLETE' and classify(r)[2] == 'Complete']
    out = []
    for r in rows:
        sim = 'Gazebo' if r['m']['simulator'] == 'gz' else 'Isaac'
        sides = r['m'].get('selected_sides') or []
        cells = []
        vetoes = {n: reason for _, n, reason in r['vetoes']}
        for n in range(1, (r['m'].get('attempts') or 1) + 1):
            side = sides[n - 1].replace('g1_', '').replace('_', ' ') if n - 1 < len(sides) else '?'
            if n in vetoes:
                why = vetoes[n].split(':')[-1].strip()
                cells.append(f'<span class="att veto" title="{esc(VETO_TEXT.get(why, why))}">'
                             f'<b>✕</b> {n}: {esc(side)}<small>vetoed: {esc(VETO_TEXT.get(why, why))}</small></span>')
            else:
                cells.append(f'<span class="att parked"><b>✓</b> {n}: {esc(side)}<small>parked</small></span>')
        out.append(f'<div class="attrow"><div class="attlabel">{esc(sim)} · {esc(r["name"])}<small>'
                   f'{esc(datetime.datetime.fromtimestamp(r["start"]).strftime("%Y-%m-%d"))} · '
                   f'{esc(r["campaign"])}</small></div><div class="atts">{"".join(cells)}</div></div>')
    return ''.join(out)


def reach_chart(runs):
    """Gazebo ground-truth reach of every completed Gazebo mission (Isaac has none)."""
    rows = [r for r in runs if r['m']['simulator'] == 'gz' and r['m']['outcome'] == 'MISSION_COMPLETE'
            and r['m'].get('reach_truth_m') is not None]
    rows.sort(key=lambda r: (not r['final'], r['start']))
    label_w, width, row_h = 250, 940, 24
    x0, x1, top = label_w, width - 170, 24
    hi = 0.90
    scale = lambda v: x0 + (x1 - x0) * min(v, hi) / hi
    body = [f'<rect x="{scale(REACH[0]):.1f}" y="10" width="{scale(REACH[1]) - scale(REACH[0]):.1f}" '
            f'height="{len(rows) * row_h + 14}" class="band"><title>Required reach 0.30-0.60 m</title></rect>',
            f'<text x="{(scale(REACH[0]) + scale(REACH[1])) / 2:.1f}" y="8" class="tick" text-anchor="middle">'
            f'required 0.30-0.60 m</text>']
    for i, r in enumerate(rows):
        y = top + i * row_h
        reach = r['m']['reach_truth_m']
        false = classify(r)[2] == FALSE_COMPLETION
        when = datetime.datetime.fromtimestamp(r['start']).strftime('%d %b')
        body.append(f'<text x="{x0 - 10}" y="{y + 4}" class="label" text-anchor="end">{esc(r["name"])} · '
                    f'{when}</text>')
        if reach > hi:
            body.append(f'<line x1="{scale(hi) - 30:.1f}" y1="{y}" x2="{scale(hi):.1f}" y2="{y}" class="arrow"/>'
                        f'<text x="{scale(hi) + 6:.1f}" y="{y + 4}" class="value bad">{reach:.2f} m: false completion</text>')
        else:
            body.append(f'<circle cx="{scale(reach):.1f}" cy="{y}" r="5" class="{"dot-bad" if false else "dot-gz"}">'
                        f'<title>{esc(r["name"])}: {reach:.3f} m (ground truth)</title></circle>'
                        f'<text x="{scale(reach) + 9:.1f}" y="{y + 4}" class="value">{reach:.2f}</text>')
    y_axis = top + len(rows) * row_h
    body.append(axis(x0, x1, y_axis, [0, 0.15, 0.30, 0.45, 0.60, 0.75, 0.90], scale, ' m'))
    return svg(width, y_axis + 30, ''.join(body), 'Parked reach against Gazebo ground truth')


# Page sections ------------------------------------------------------------------------

def stat(value, label, detail):
    return (f'<div class="stat"><div class="v">{value}</div><div class="l">{esc(label)}</div>'
            f'<div class="d">{esc(detail)}</div></div>')


def overview(final, scenario_count):
    complete = [r for r in final if classify(r)[2] == 'Complete']
    refusals = [r for r in final if classify(r)[2] == 'Safe refusal']
    others = [r for r in final if r not in complete and r not in refusals]
    scen = sorted({r['m']['scenario'] for r in final})
    durations = [r['m']['mission_s'] for r in complete]
    gz_reach = [r['m']['reach_truth_m'] for r in complete if r['m'].get('reach_truth_m') is not None]
    cards = [
        stat(scenario_count, 'scenarios defined', f'{len(scen)} evaluated after the merge: {", ".join(scen)}'),
        stat(len(final), 'post-merge runs', f'14 Nov 2026, code {FINAL_SET[0]}-{FINAL_SET[-1]}'),
        stat(len(complete), 'complete missions', ' and '.join(
            ('Gazebo' if r['m']['simulator'] == 'gz' else 'Isaac') for r in complete) + ': parked, waited, returned'),
        stat(len(refusals), 'safe refusal', 'no_feasible: no pose fits, no attempt, returned'),
        stat(len(others), 'stopped or failed', '3 integration defects found and fixed during the runs, '
                                               '1 G1 not detected'),
        stat(f'{gz_reach[0]:.3f} m' if gz_reach else '-', 'parked reach (Gazebo, ground truth)',
             f'required {REACH[0]:.2f}-{REACH[1]:.2f} m; Isaac records no ground truth'),
        stat(' / '.join(f'{d / 60:.1f}' for d in sorted(durations)) + ' min', 'complete mission duration',
             'Isaac / Gazebo; most of it is the search for G1'),
    ]
    return (f'<section id="overview"><div class="stats">{"".join(cards)}</div>'
            f'<p class="note">Every number above comes from the {len(final)} runs on the merged code '
            f'(14 November 2026); earlier runs used earlier code and are counted only in the '
            f'<a href="#history">records</a>. {len(complete)} completed missions do not establish a '
            f'success rate.</p></section>')


def mission_summary(r):
    m = r['m']
    sides = [s.replace('g1_', 'G1 ').replace('_', ' ') for s in m.get('selected_sides') or []]
    vetoes = {n: reason.split(':')[-1].strip() for _, n, reason in r['vetoes']}
    items = []
    for n, side in enumerate(sides, 1):
        if n in vetoes:
            items.append(f'Attempt {n} at {side}: vetoed, {VETO_TEXT.get(vetoes[n], vetoes[n])}')
        elif m['outcome'] == 'MISSION_COMPLETE' and n == len(sides):
            items.append(f'Attempt {n} at {side}: parked')
    return items


def comparison(by_key, refusal):
    def column(r, sim, truth):
        m = r['m']
        facts = [f'Final state <code>{esc(m["outcome"])}</code>, {m["mission_s"] / 60:.1f} min']
        facts += [esc(x) for x in mission_summary(r)]
        if truth and m.get('localization_errors_m'):
            errs = m['localization_errors_m']
            facts.append(f'G1 localization error {min(errs):.3f}-{max(errs):.3f} m (ground truth)')
        if truth and m.get('reach_truth_m') is not None:
            facts.append(f'Parked reach {m["reach_truth_m"]:.3f} m, basket {m["parked_truth_clearance_m"]:.3f} m '
                         f'from G1\'s lower body (ground truth)')
        if not truth:
            facts.append('No ground truth recorded: positions and reach are the mission\'s own estimates')
        facts.append('Returned to its start' + (f', {m["final_distance_from_start_m"]:.2f} m from it (ground truth)'
                                                if truth and m.get('final_distance_from_start_m') is not None else ''))
        return (f'<div class="col"><h3>{esc(sim)}</h3><p class="sub">{esc(m["scenario"])} · '
                f'<code>{esc(r["name"])}</code> · code {esc(r["code"])}</p><ul>'
                + ''.join(f'<li>{f}</li>' for f in facts) + '</ul></div>')
    rm = refusal['m']
    errs = rm.get('localization_errors_m') or []
    refusal_html = (
        f'<div class="col wide"><h3>Gazebo Sim · safety test</h3><p class="sub">no_feasible · '
        f'<code>{esc(refusal["name"])}</code> · code {esc(refusal["code"])}</p><ul>'
        f'<li>G1 between two panels with a bed behind it: no robot footprint fits beside it</li>'
        f'<li>G1 localized {", ".join(f"{e:.3f}" for e in errs)} m from its ground truth</li>'
        f'<li>Every parking pose rejected (<code>no_valid_candidate</code>); no parking attempt</li>'
        f'<li>Returned {rm["final_distance_from_start_m"]:.2f} m from its start (ground truth); final state '
        f'<code>{esc(rm["outcome"])}</code> with the planner\'s reason</li></ul></div>')
    return (f'<section id="compare"><h2>Gazebo Sim and Isaac Sim</h2>'
            f'<p class="lead">Both simulators run the same autonomy code on the same scene. In Isaac the live '
            f'LiDAR check cancelled the approach to G1\'s right and the robot parked at G1\'s left; in Gazebo '
            f'it cancelled both sides and the mission parked behind G1, the next class in its priority '
            f'order. Gazebo measurements are ground truth; Isaac records only the mission\'s own estimates. '
            f'One run per simulator: a demonstration, not a rate.</p><div class="cmp">{column(by_key["isaac_side"], "Isaac Sim", False)}'
            f'{column(by_key["gazebo_rear"], "Gazebo Sim", True)}{refusal_html}</div></section>')


def _reason(r, t):
    return next((w for when, _, _, w in r['transitions'] if when == t), '')


def timeline(r):
    segs = phase_segments(r)
    if not segs:
        return ''
    t0, t1 = segs[0][1], segs[-1][2]
    bar = ''.join(f'<span class="seg ph-{p}" style="flex:{max(b - a, 1):.1f}" title="{esc(dict(PHASES)[p])}: '
                  f'{b - a:.0f} s"></span>' for p, a, b in segs)
    events, attempt = [], 0
    sides = r['m'].get('selected_sides') or []
    for t, previous, state, _ in r['transitions']:
        if previous == 'SEARCH_G1':
            events.append((t, 'G1 confirmed', ''))
        elif state == 'G1_LOCALIZED':
            events.append((t, 'G1 localized and verified', ''))
        elif state == 'NAVIGATING_TO_PREPARKING':
            attempt += 1
            side = sides[attempt - 1].replace('g1_', "G1's ") if attempt <= len(sides) else 'next side'
            events.append((t, f'Attempt {attempt} started: {side}', ''))
        elif state == 'PARKED':
            events.append((t, 'Parked', 'ok'))
        elif state == 'RETURNING_TO_START' and 'no feasible parking pose' in _reason(r, t):
            events.append((t, 'No safe parking pose: refused, returning home', 'safe'))
        elif state == 'RETURNING_TO_START':
            events.append((t, 'Returning to start', ''))
        elif state in ('MISSION_COMPLETE', 'MISSION_FAILED'):
            events.append((t, state, 'ok' if state == 'MISSION_COMPLETE' else 'safe'))
    for t, n, reason in r['vetoes']:
        events.append((t, f'Attempt {n} cancelled by the LiDAR check', 'bad'))
    events.sort()
    # Repeated localizations: keep the first of each kind between other events.
    rows, last = [], None
    for t, text, cls in events:
        if text == last:
            continue
        last = text
        rows.append(f'<li class="{cls}"><span>{clock(t - t0)}</span>{esc(text)}</li>')
    legend = ''.join(f'<span><i class="sw ph-{k}"></i>{esc(v)}</span>' for k, v in PHASES
                     if any(p == k for p, _, _ in segs))
    return (f'<div class="timeline"><div class="bar">{bar}</div><div class="tl-axis"><span>0:00</span>'
            f'<span>{clock(t1 - t0)}</span></div><div class="legend small">{legend}</div>'
            f'<ol class="events">{"".join(rows)}</ol></div>')


def featured(r, spec, explanation):
    figs, missing = [], []
    for i, (kind, ref, states, caption) in enumerate(spec['images']):
        rel, text = featured_image(r, kind, ref, states, caption, f'{spec["key"]}_{i}')
        if rel is None:
            print(f'{spec["key"]} image {ref} excluded: {text}')
            missing.append(text)
            continue
        figs.append(pages.figure(rel, text))
    lead = figs[0] if figs else pages.missing('No verified capture')
    rest = ''.join(f'<div>{f}</div>' for f in figs[1:])
    badge_cls, outcome, _ = classify(r)
    facts = '<div class="factlist">' + ''.join(f'<div><b>{esc(k)}</b><span>{v}</span></div>'
                                               for k, v in explanation) + '</div>'
    return (f'<article class="exp" id="{spec["key"]}"><header><h3>{esc(spec["title"])}</h3>'
            f'<span class="badge {badge_cls}">{esc(outcome)}</span><span class="meta">{esc(r["name"])} · '
            f'code {esc(r["code"])} · {esc(datetime.datetime.fromtimestamp(r["start"]).strftime("%Y-%m-%d %H:%M"))}'
            f'</span></header><div class="exp-grid"><div class="lead-fig">{lead}</div>'
            f'<div class="exp-facts">{facts}{timeline(r)}</div></div>'
            f'<div class="exp-more">{rest}</div></article>')


def explanations(by_key):
    a, b, c = by_key['isaac_side'], by_key['gazebo_rear'], by_key['gazebo_refusal']
    am, bm, cm = a['m'], b['m'], c['m']

    def veto_text(r):
        parts = []
        for (_, n, reason), (nearest, gap) in zip(r['vetoes'], r['veto_detail']):
            why = reason.split(':')[-1].strip()
            parts.append(f'attempt {n}: {VETO_TEXT.get(why, why)}, {nearest:.2f} m from the robot while '
                         f'the robot was {gap:.2f} m from G1')
        return 'The live LiDAR check cancelled both side approaches before contact (' + '; '.join(
            esc(p) for p in parts) + '; LiDAR measurements)'

    def attempts_text(r):
        return '<br>'.join(esc(x) for x in mission_summary(r)) or '-'
    return {
        'isaac_side': [
            ('Parking side', 'G1\'s left (its right was tried first)'),
            ('Attempts', attempts_text(a)),
            ('Outcome', f'<code>{esc(am["outcome"])}</code>: parked, waited 5 s, returned'),
            ('Measured', 'Isaac records no ground truth; the arrival error to the planned pose was '
                         + esc(re.findall(r'Arrived at parking: error xy=([\d.]+) m', a['log'])[-1]) + ' m'),
            ('Return to start', 'Reached within the shared 0.40 m goal tolerance, any heading'),
        ],
        'gazebo_rear': [
            ('Why not beside G1', veto_text(b)),
            ('Why behind G1', 'After two vetoed sides, the next class in the priority order was the rear'),
            ('Attempts', attempts_text(b)),
            ('Reach (ground truth)', f'{bm["reach_truth_m"]:.3f} m, inside {REACH[0]:.2f}-{REACH[1]:.2f} m; '
                                     f'basket {bm["parked_truth_clearance_m"]:.3f} m from G1\'s lower body'),
            ('G1 localization (ground truth)', f'{min(bm["localization_errors_m"]):.3f}-'
                                               f'{max(bm["localization_errors_m"]):.3f} m error, heading '
                                               f'error {bm["heading_error_deg"]:.1f} deg'),
            ('Outcome', f'<code>{esc(bm["outcome"])}</code>; returned {bm["final_distance_from_start_m"]:.2f} m '
                        f'from the start (ground truth)'),
        ],
        'gazebo_refusal': [
            ('Scene', 'G1 at its worktable between two 1.7 m panels 0.8 m to each side, a bed 1.25 m behind'),
            ('Why nothing fits', 'Every footprint within reach of G1 overlaps a panel, the table or the bed '
                                 '(<code>no_valid_candidate</code>)'),
            ('G1 localization (ground truth)', ', '.join(f'{e:.3f}' for e in cm['localization_errors_m'])
             + f' m error, heading error {cm["heading_error_deg"]:.1f} deg'),
            ('Parking attempts', 'None'),
            ('Outcome', f'<code>{esc(cm["outcome"])}</code> with the reason; returned '
                        f'{cm["final_distance_from_start_m"]:.2f} m from the start (ground truth)'),
        ],
    }


def history(runs):
    rows = []
    for r in sorted(runs, key=lambda r: -r['start']):
        m = r['m']
        cls, outcome, category = classify(r)
        sim = 'Gazebo' if m['simulator'] == 'gz' else 'Isaac'
        when = datetime.datetime.fromtimestamp(r['start']).strftime('%Y-%m-%d %H:%M')
        meas = []
        if m.get('localization_errors_m'):
            meas.append(f'G1 error {span(m["localization_errors_m"])} m')
        if m.get('reach_truth_m') is not None:
            meas.append(f'reach {m["reach_truth_m"]:.3f} m')
        if m.get('attempts'):
            meas.append(f'{m["attempts"]} attempt{"s" if m["attempts"] > 1 else ""}')
        if m.get('mission_s'):
            meas.append(f'{m["mission_s"] / 60:.1f} min')
        if m['simulator'] != 'gz':
            meas.append('no ground truth')
        details = [('Code', r['code']), ('Campaign', r['campaign']), ('Final reason', (m.get('final_reason') or '-')[:220]),
                   ('Sides tried', ', '.join(m.get('selected_sides') or []) or '-'),
                   ('G1 heading estimate', fmt(m.get('g1_heading_log'), ' deg')),
                   ('Minimum clearance (ground truth)', f'{m["min_truth_clearance_m"]} m ({m.get("min_truth_clearance_pair")})'
                    if m.get('min_truth_clearance_m') is not None else '-'),
                   ('Evidence', f'<code>{esc(r["rel"])}</code> (outside the repository)')]
        if 'lidar_stale' in (m.get('final_reason') or ''):
            details.append(('Cause', 'Both LiDARs published at 29.5 Hz (largest gap 0.15 s); the parking LiDAR '
                                     'check, 14 ms per scan after the merge, held the mission node back so it '
                                     'judged fresh scans stale. Not an obstacle. Fixed in 5b5133c.'))
        if category == FALSE_COMPLETION:
            details.append(('Cause', 'A worktable edge in G1\'s detection box became G1\'s position (1.25 m off); '
                                     'the mission parked beside it. Fixed by the G1 position check (3becd27).'))
        record = (f'<a href="../../archive/engineering/{r["record"]}">record</a>' if r['record'] else '-')
        detail_html = ''.join(f'<div><b>{esc(k)}</b><span>{v if k == "Evidence" else esc(v)}</span></div>'
                              for k, v in details)
        rows.append(
            f'<tr class="run" data-sim="{sim}" data-scenario="{esc(m["scenario"])}" data-code="{esc(r["campaign"])}" '
            f'data-outcome="{esc(category)}"><td>{when}</td><td><code>{esc(r["name"])}</code></td>'
            f'<td>{esc(m["scenario"])}</td><td>{sim}</td><td>{esc(r["campaign"])}<br><code>{esc(m["commit"])}</code></td>'
            f'<td><span class="badge {cls}">{esc(category)}</span></td><td>{esc(", ".join(meas)) or "-"}</td>'
            f'<td>{record}</td><td><button type="button" class="more" aria-expanded="false">details</button></td></tr>'
            f'<tr class="detail" hidden><td colspan="9"><p class="outcome">{esc(outcome)}</p>'
            f'<div class="rowdetail">{detail_html}</div></td></tr>')

    def select(name, label, values):
        return (f'<label>{esc(label)} <select data-filter="{name}"><option value="">All</option>'
                + ''.join(f'<option>{esc(v)}</option>' for v in values) + '</select></label>')
    filters = (select('sim', 'Simulator', ['Gazebo', 'Isaac'])
               + select('scenario', 'Scenario', sorted({r['m']['scenario'] for r in runs}))
               + select('code', 'Code version', list(dict.fromkeys(r['campaign'] for r in sorted(runs, key=lambda r: r['start']))))
               + select('outcome', 'Outcome', sorted({classify(r)[2] for r in runs}))
               + '<label>Search <input type="search" data-filter="text" placeholder="run, reason..."></label>')
    script = """<script>
  (function () {
    const table = document.getElementById('runs');
    const controls = document.querySelectorAll('[data-filter]');
    const count = document.getElementById('runcount');
    function apply() {
      let shown = 0;
      table.querySelectorAll('tbody tr.run').forEach(row => {
        let ok = true;
        controls.forEach(c => {
          const v = c.value.trim().toLowerCase();
          if (!v) return;
          if (c.dataset.filter === 'text') ok = ok && row.textContent.toLowerCase().includes(v);
          else ok = ok && (row.dataset[c.dataset.filter] || '').toLowerCase() === v;
        });
        row.hidden = !ok; if (ok) shown++;
        const more = row.querySelector('button.more');
        row.nextElementSibling.hidden = !ok || more.getAttribute('aria-expanded') !== 'true';
      });
      count.textContent = shown + ' of ' + table.querySelectorAll('tbody tr.run').length + ' runs shown';
    }
    table.querySelectorAll('button.more').forEach(b => b.addEventListener('click', () => {
      const open = b.getAttribute('aria-expanded') !== 'true';
      b.setAttribute('aria-expanded', String(open));
      b.textContent = open ? 'hide' : 'details';
      b.closest('tr').nextElementSibling.hidden = !open;
    }));
    controls.forEach(c => c.addEventListener('input', apply));
    apply();
  })();
</script>"""
    return (f'<section id="history"><details class="archive"><summary><h2>Historical experiments &amp; '
            f'detailed records</h2><span class="note">{len(runs)} recorded runs, newest first, each under the '
            f'code that ran</span></summary><div class="filters">{filters}<span id="runcount" class="note"></span>'
            f'</div><div class="wrap"><table id="runs"><thead><tr><th>Date</th><th>Run</th><th>Scenario</th>'
            f'<th>Simulator</th><th>Code version</th><th>Outcome</th><th>Measurements</th><th>Record</th>'
            f'<th></th></tr></thead><tbody>{"".join(rows)}</tbody></table></div>'
            f'<p class="note">Evidence folders live under <code>artifacts/</code> outside the repository; the '
            f'dated records explain each campaign. Gazebo measurements are ground truth sampled every '
            f'1-2.5 s; Isaac runs record the mission\'s own estimates only.</p></details>{script}</section>')


def span(values):
    lo, hi = min(values), max(values)
    return f'{lo:.3f}' if f'{lo:.3f}' == f'{hi:.3f}' else f'{lo:.3f}-{hi:.3f}'


def fmt(value, unit=''):
    return '-' if value in (None, []) else f'{value}{unit}'


RESULTS_CSS = """
  :root { --gz: #2a78d6; --isaac: #eb6834;
          --ph-search: #1baf7a; --ph-locate: #eda100; --ph-park: #6250d6; --ph-wait: #e87ba4;
          --ph-return: #008300; --st-good: #0ca30c; --st-crit: #d03b3b; --band: rgba(12,163,12,.10); }
  @media (prefers-color-scheme: dark) {
    :root { --gz: #3987e5; --isaac: #d95926;
            --ph-search: #199e70; --ph-locate: #c98500; --ph-park: #9085e9; --ph-wait: #d55181;
            --ph-return: #008300; --band: rgba(12,163,12,.16); }
  }
  .subtitle { font-size: 1.05rem; color: var(--body); margin: 0 0 .6rem; }
  section { scroll-margin-top: 3rem; }
  .lead { max-width: 90ch; }
  .stats { display: grid; gap: .8rem; grid-template-columns: repeat(auto-fill, minmax(min(100%, 230px), 1fr));
           margin: 1.6rem 0 .6rem; }
  .stat { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: .9rem 1rem; }
  .stat .v { color: var(--ink); font-size: 1.7rem; font-weight: 600; line-height: 1.2; }
  .stat .l { color: var(--ink); font-size: .9rem; margin-top: .15rem; }
  .stat .d { color: var(--faint); font-size: .8rem; margin-top: .25rem; }
  .cmp { display: grid; gap: .9rem; grid-template-columns: 1fr 1fr; }
  .cmp .col { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 1rem 1.2rem; }
  .cmp .col.wide { grid-column: 1 / -1; }
  .cmp ul { margin: .4rem 0 0; padding-left: 1.1rem; font-size: .9rem; }
  .cmp li { margin: .2rem 0; }
  .sub { color: var(--faint); font-size: .82rem; margin: .2rem 0 0; }
  .exp { background: var(--card); border: 1px solid var(--line); border-radius: 12px; padding: 1.2rem 1.3rem;
         margin: 1.4rem 0; scroll-margin-top: 3rem; }
  .exp > header { display: flex; flex-wrap: wrap; gap: .4rem .8rem; align-items: center; margin-bottom: .9rem; }
  .exp > header .meta { margin-left: auto; font-family: var(--mono); font-size: .76rem; color: var(--faint); }
  .exp-grid { display: grid; gap: 1.2rem; grid-template-columns: minmax(0, 1.45fr) minmax(0, 1fr); align-items: start; }
  .factlist > div { display: grid; grid-template-columns: 11rem 1fr; gap: .6rem; font-size: .88rem;
                     padding: .35rem 0; border-bottom: 1px solid var(--line); }
  .exp-facts b { color: var(--ink); font-weight: 600; }
  .exp-more { display: grid; gap: .9rem; grid-template-columns: 1fr 1fr; margin-top: 1rem; }
  .timeline { margin-top: 1rem; }
  .timeline .bar { display: flex; gap: 2px; height: 14px; border-radius: 4px; overflow: hidden; }
  .timeline .seg { display: block; min-width: 3px; }
  .tl-axis { display: flex; justify-content: space-between; font-size: .72rem; color: var(--faint); margin-top: .15rem; }
  .events { list-style: none; padding: 0; margin: .5rem 0 0; font-size: .82rem; columns: 2; column-gap: 1.2rem; }
  .events li { break-inside: avoid; padding: .12rem 0; }
  .events li span { font-family: var(--mono); color: var(--faint); display: inline-block; width: 3.4rem; }
  .events li.bad { color: var(--bad); } .events li.ok { color: var(--ok); } .events li.safe { color: var(--safe); }
  .legend { display: flex; flex-wrap: wrap; gap: .3rem 1rem; font-size: .8rem; color: var(--body); margin: .4rem 0; }
  .legend.small { font-size: .74rem; margin-top: .5rem; }
  .legend .sw { display: inline-block; width: .8rem; height: .8rem; border-radius: 3px; margin-right: .35rem;
                vertical-align: -1px; }
  .ph-search { background: var(--ph-search); fill: var(--ph-search); }
  .ph-locate { background: var(--ph-locate); fill: var(--ph-locate); }
  .ph-park { background: var(--ph-park); fill: var(--ph-park); }
  .ph-wait { background: var(--ph-wait); fill: var(--ph-wait); }
  .ph-return { background: var(--ph-return); fill: var(--ph-return); }
  .charts { display: grid; gap: 1rem; grid-template-columns: 1fr; }
  .chartcard { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 1rem 1.2rem; }
  .chartcard h3 { color: var(--ink); font-size: 1rem; }
  .chartcard p.q { color: var(--faint); font-size: .84rem; margin: .2rem 0 .6rem; }
  svg.chart { width: 100%; height: auto; display: block; }
  svg .axis { stroke: var(--line); stroke-width: 1; } svg .grid { stroke: var(--line); stroke-width: 1; opacity: .6; }
  svg .row { stroke: var(--line); stroke-width: 1; }
  svg .tick { fill: var(--faint); font-size: 11px; } svg .label { fill: var(--body); font-size: 12px; }
  svg .value { fill: var(--ink); font-size: 11px; } svg .value.bad { fill: var(--bad); }
  svg .dot-gz { fill: var(--gz); stroke: var(--card); stroke-width: 2; }
  svg .dot-bad { fill: var(--st-crit); stroke: var(--card); stroke-width: 2; }
  svg .band { fill: var(--band); } svg .arrow { stroke: var(--bad); stroke-width: 2; }
  details.data { margin-top: .5rem; } details.data summary { cursor: pointer; color: var(--faint); font-size: .82rem; }
  .attrow { display: grid; grid-template-columns: 16rem 1fr; gap: .8rem; padding: .5rem 0; border-bottom: 1px solid var(--line); }
  .attlabel { font-size: .86rem; color: var(--ink); } .attlabel small { display: block; color: var(--faint); font-size: .76rem; }
  .atts { display: flex; flex-wrap: wrap; gap: .4rem; }
  .att { border: 1px solid var(--line); border-radius: 8px; padding: .3rem .6rem; font-size: .84rem; color: var(--ink); }
  .att b { margin-right: .2rem; } .att small { display: block; color: var(--faint); font-size: .74rem; }
  .att.veto b { color: var(--st-crit); } .att.parked b { color: var(--st-good); }
  .att.parked { border-color: var(--st-good); }
  details.archive > summary { cursor: pointer; list-style: none; }
  details.archive > summary::-webkit-details-marker { display: none; }
  details.archive > summary h2 { display: inline; }
  details.archive > summary h2::before { content: '▸ '; color: var(--accent); }
  details.archive[open] > summary h2::before { content: '▾ '; }
  details.archive > summary .note { display: block; margin: .2rem 0 .8rem; }
  .filters { display: flex; flex-wrap: wrap; gap: .5rem 1rem; align-items: center; margin: .6rem 0; font-size: .84rem; }
  .filters select, .filters input { font: inherit; padding: .2rem .4rem; border: 1px solid var(--line);
                                     border-radius: 6px; background: var(--card); color: var(--ink); }
  button.more { font: inherit; font-size: .78rem; color: var(--accent); background: none; cursor: pointer;
                border: 1px solid var(--line); border-radius: 6px; padding: .1rem .5rem; }
  #runs tr.detail td { background: var(--paper); }
  #runs tr.detail .outcome { margin: .2rem 0 .5rem; color: var(--ink); font-weight: 600; font-size: .86rem; }
  .rowdetail { display: grid; gap: .3rem 1.4rem; grid-template-columns: repeat(auto-fill, minmax(min(100%, 320px), 1fr));
               font-size: .8rem; }
  .rowdetail div { display: grid; grid-template-columns: 9.5rem 1fr; gap: .5rem; }
  .rowdetail b { color: var(--ink); font-weight: 600; }
  #runs td { vertical-align: top; }
  @media (max-width: 960px) {
    .exp-grid, .cmp, .exp-more { grid-template-columns: 1fr; }
    .attrow { grid-template-columns: 1fr; } .events { columns: 1; }
  }
  @media (max-width: 560px) {
    .factlist > div, .rowdetail div { grid-template-columns: 7rem 1fr; }
    .exp > header .meta { margin-left: 0; }
  }
"""


def main():
    names = sys.argv[1:] or [l.strip() for l in open(RUN_LIST) if l.strip() and not l.startswith('#')]
    runs = [load(n) for n in names]
    final = [r for r in runs if r['final']]
    import yaml
    scenario_count = len(yaml.safe_load(open(SCENARIO_FILE))['scenarios'])
    by_key = {spec['key']: next(r for r in runs if os.path.normpath(r['path']) ==
                                os.path.normpath(os.path.join(ROOT, spec['run'])))
              for spec in FEATURED}
    explain = explanations(by_key)
    featured_html = ''.join(featured(by_key[spec['key']], spec, explain[spec['key']]) for spec in FEATURED)
    charts = (
        '<div class="charts">'
        '<div class="chartcard"><h3>Mission duration by phase</h3><p class="q">The post-merge runs, from the '
        'first mission state to the last; hover a segment for its seconds.</p>'
        f'{duration_chart(final)}</div>'
        '<div class="chartcard"><h3>G1 localization error (Gazebo ground truth)</h3><p class="q">Distance from '
        'each G1 position the mission used to G1\'s true position, post-merge Gazebo runs.</p>'
        f'{localization_chart(final)}</div>'
        '<div class="chartcard"><h3>Parking attempts in completed missions</h3><p class="q">Every completed '
        'mission since the G1 position check (9 November): each attempt, and why it was vetoed. Three '
        'attempts is the limit.</p>'
        f'{attempts_chart([r for r in runs if r["start"] >= by_key["isaac_side"]["start"] - 6 * 86400])}</div>'
        '<div class="chartcard"><h3>Parked reach against the requirement (Gazebo ground truth)</h3><p class="q">'
        'Distance from G1\'s axis to the basket centre at every completed Gazebo mission, all campaigns; the '
        'shaded band is the required 0.30-0.60 m. Isaac records no ground truth and is not shown.</p>'
        f'{reach_chart(runs)}</div></div>')
    jump = ''.join(f'<a href="#{k}">{v}</a>' for k, v in (
        ('overview', 'Overview'), ('compare', 'Gazebo and Isaac'), ('featured', 'Featured experiments'),
        ('performance', 'Performance'), ('history', 'All runs')))
    intro = ('<p class="subtitle">Evaluation in Gazebo Sim and NVIDIA Isaac Sim</p>'
             '<p>Ridgeback searches a room for a Unitree G1 standing at its worktable, localizes it, parks '
             'with its basket within reach of G1, waits five seconds and drives back to its start. This page '
             'shows how the final code performed; the scenes are on the '
             f'<a href="{pages.SCENARIOS_PAGE}">scenarios page</a>, the design in the '
             '<a href="parking_review.md">technical review</a>.</p>')
    body = (f'<nav class="jump"><div class="shell">{jump}</div></nav><main class="shell">'
            f'{overview(final, scenario_count)}'
            f'{comparison(by_key, by_key["gazebo_refusal"])}'
            f'<section id="featured"><h2>Featured experiments</h2><p class="lead">Three runs on the merged '
            f'code, each with captures from that run only. Click an image to enlarge it.</p>{featured_html}</section>'
            f'<section id="performance"><h2>Performance</h2>{charts}</section>'
            f'{history(runs)}</main>')
    with open(OUT, 'w') as stream:
        stream.write(pages.page('Autonomous Parking — Experimental Results', pages.RESULTS_PAGE, intro, body,
                                extra_css=RESULTS_CSS))
    # The folder holds this page's exports only: drop any the page no longer shows.
    used = set(re.findall(r'assets/parking_results/([^"]+)', open(OUT).read()))
    for name in sorted(set(os.listdir(ASSETS)) - used):
        os.remove(os.path.join(ASSETS, name))
    print('wrote', OUT, len(runs), 'runs,', len(final), 'post-merge,', len(used), 'images')


if __name__ == '__main__':
    main()
