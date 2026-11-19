"""Build docs/exploration/autonomous_parking_results.html: the experimental results dashboard.

Usage: make_results_page.py [RUN_DIR ...]   (default: the runs in results_runs.txt)

Sections: an overview of the hospital-ward validation runs, Gazebo against Isaac on the
final code, three featured experiments with their own screenshots and timelines, charts
drawn from recorded measurements, and every recorded run in a filterable table. Only
captures of runs that loaded the hospital ward are shown; warehouse runs keep their
measurements in the table, and their screenshots stay in their evidence folders.

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
import math
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
MISSED_LOCALIZATION = 'Refusal missed: G1 mislocalized'
MISLOCALIZED_M = 0.25        # a G1 position this far from ground truth names the cause of a missed refusal

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
        '5b5133c': ('After merging dev (P8)', '5b5133c', '2026-11-14-parking-dev-integration.md'),
        '822ae6d': ('Hospital ward trial', '822ae6d', '2026-11-17-parking-gazebo-hospital-ward.md'),
        '23b70ff': ('Hospital ward trial', '23b70ff', '2026-11-17-parking-gazebo-hospital-ward.md'),
        '9400c10': ('Hospital ward validation', '9400c10', '2026-11-18-parking-hospital-validation.md'),
        'f568789': ('Hospital ward validation', 'f568789', '2026-11-18-parking-hospital-validation.md'),
        'f6ade2f': ('Hospital ward validation', 'f6ade2f', '2026-11-18-parking-hospital-validation.md'),
        'b519ab0': ('Hospital ward validation', 'b519ab0', '2026-11-18-parking-hospital-validation.md'),
        '544ead1': ('Hospital ward validation', '544ead1 (the mission code of b519ab0; comments changed)',
                    '2026-11-18-parking-hospital-validation.md'),
        '0890e13': ('Hospital ward validation', '0890e13 (the mission code of b519ab0; documentation changed)',
                    '2026-11-18-parking-hospital-validation.md'),
        '2c6c5db': ('Hospital ward validation', '2c6c5db (the mission code of b519ab0; documentation changed)',
                    '2026-11-18-parking-hospital-validation.md')}
# The 2026-11-17 trial: Gazebo in the hospital ward Isaac uses (parking_demo), same mission
# code as the post-merge runs. Gazebo kept the warehouse after it until the validation below.
WARD_TRIAL = ('822ae6d', '23b70ff')
# The post-merge warehouse runs of 2026-11-14 (merged code 64b3176 and its three fixes).
POST_MERGE = ('64b3176', 'c1435d9', '99387ae', '5b5133c')
# The hospital-ward validation of 2026-11-17/18, both simulators in the ward: the set every
# overview number on the page is computed from. Its runs span the localization fixes; the
# final mission code is b519ab0's (544ead1 and 0890e13 changed comments and documentation).
FINAL_SET = ('9400c10', 'f568789', 'f6ade2f', 'b519ab0', '544ead1', '0890e13', '2c6c5db')
FINAL_CODE = ('b519ab0', '544ead1', '0890e13', '2c6c5db')
# Runs that need a cause the log alone does not give, by evidence folder.
CAUSES = {
    'artifacts/exploration/hospital_2026-11-18/gz_standard':
        'The robot parked behind G1 on attempt 3, but three Gazebo view cameras created on arrival stalled the '
        'server and the stop was not confirmed within its 10 s limit. The views are taken after the stop is '
        'confirmed since f568789. The installed mission node was still 5b5133c\'s.',
    'artifacts/exploration/hospital_2026-11-18/gz_no_feasible':
        'G1 was never seen: the robot wedged sideways in the 1.10 m strip between the pen\'s bed and the corridor '
        'wall (its footprint is 1.05 m across the diagonal) and the operator stopped the run. The bed was moved '
        'closer to G1 in f6ade2f, leaving a 1.40 m strip.',
    'artifacts/exploration/hospital_2026-11-18/gz_standard_run2':
        'Parked 0.607 m from G1\'s axis, 7 mm outside the required band: the plan measured 0.517 m from G1\'s '
        'estimate, and the 0.053 m localization error and the robot\'s map-pose error added the rest. G1\'s '
        'heading was estimated 65 deg off from a 0.68 m view (unexplained; not reproduced), so the bin stood '
        'behind G1\'s right shoulder rather than at its front right.',
    'artifacts/exploration/final_demo_2026-11-19/isaac_standard/run':
        'The recorded demonstration run (complete.mp4 and presentation.mp4 beside it). G1\'s heading was not '
        'estimated from this view (no reliable forearm view), so parking poses were ranked by the approach side, '
        'not by G1\'s sides: the robot parked behind G1, the bin about 0.46 m from G1\'s configured position by '
        'the mission\'s own map pose. The standard scene expects G1\'s left or right.',
    'artifacts/exploration/hospital_2026-11-18/isaac_standard':
        'The head region measured the south wall behind G1, and the position re-seeded onto the wall was '
        'refused five times until the localization deadline. The surface fallback (b519ab0) keeps the '
        'detector\'s estimate in this case.',
}
TRANSITION = re.compile(r'\[(\d+\.\d+)\] \[[^\]]*mission_manager_node\]: '
                        r'Mission state transition: (\w+) -> (\w+); reason=([^\n]*)')
ATTEMPT_FAIL = re.compile(r'\[(\d+\.\d+)\] \[[^\]]*mission_manager_node\]: Parking attempt (\d)/\d '
                          r'failed: ([^;\n]+)')
VETO_DETAIL = re.compile(r'Parking LiDAR veto in \w+: \w+ \(nearest return ([\d.]+) m, G1 gap ([\d.]+) m\)')
PHASES = (('search', 'Search for G1'), ('locate', 'Localize, map and plan'),
          ('park', 'Parking attempts'), ('wait', 'Parked wait'), ('return', 'Return home'))
REACH_OUTSIDE = 'Parked outside the reach band'
STOP_UNCONFIRMED = 'Parked, stop not confirmed'
OPERATOR_STOP = 'Stopped by the operator'
SHORT = {MISSED_LOCALIZATION: 'refusal missed (G1 mislocalized)', 'Refusal missed': 'refusal missed', 'Complete': 'complete', 'Safe refusal': 'safe refusal', 'Perception failure': 'G1 not localized',
         'Planning gate': 'plan expired (fixed)', STALE_LIDAR: 'stale LiDAR (fixed)',
         REACH_OUTSIDE: 'reach outside band', STOP_UNCONFIRMED: 'stop not confirmed (fixed)',
         OPERATOR_STOP: 'robot wedged, stopped (fixed)'}
VETO_TEXT = {'lidar_obstacle_on_route': 'a LiDAR return lay on the planned route',
             'lidar_obstacle_in_stopping_path': 'a LiDAR return lay in the stopping path',
             'g1_protected_region': "the robot reached G1's protected region"}

# The three featured experiments, all in the hospital ward on the final mission code (Isaac
# side parking, Gazebo parking, the Gazebo safety test), and the formerly featured cards kept
# in the archive section: the run, and the captures shown for it. Each capture is a file in
# the run's shots folder (or a frame of the run's recording), the mission states it must have
# been taken in, the crop shown on the page (x, y, width, height in source pixels; the
# enlarged view shows the whole capture), and labels drawn over the crop as a separate layer
# (text, the point labelled, and where the label sits, in crop pixels; no position means the
# label sits on the object). Labels never alter the image. Only hospital-ward captures are
# shown: a featured run must have loaded the ward (WARD_WORLD), and the warehouse runs appear
# in the table with their measurements, their screenshots kept in their evidence folders.
WARD_WORLD = 'parking_demo'
RVIZ_CROP = (560, 120, 700, 360)
FEATURED = [
    {'key': 'isaac_side', 'run': 'artifacts/exploration/hospital_2026-11-18/isaac_standard_run2',
     'letter': 'A', 'title': 'Side parking in Isaac Sim (hospital ward)',
     'images': [
         {'kind': 'simview', 'ref': 'isaac_standard_parked_closeup.png', 'states': {'PARKED'},
          'crop': (380, 170, 1150, 830),
          'caption': 'Close-up while parked: Ridgeback at G1\'s right, its basket beside G1 at the worktable',
          'labels': [('G1', (520, 330), (300, 560)), ('Basket', (750, 400), (760, 120)),
                     ('Worktable', (320, 230), None)]},
         {'kind': 'simview', 'ref': 'isaac_standard_parked_workstation.png', 'states': {'PARKED'},
          'crop': (500, 150, 700, 640),
          'caption': 'Top-down view while parked: G1 facing its worktable, Ridgeback at G1\'s right',
          'labels': [('Worktable', (300, 140), None), ('G1', (295, 345), (130, 470)),
                     ('Ridgeback', (475, 450), (520, 600))]},
         {'kind': 'rviz', 'ref': 'PARKED', 'states': {'RETURNING_TO_START'}, 'crop': RVIZ_CROP,
          'caption': 'RViz as the five-second wait ended, the robot still at its parking pose: the parking '
                     'donut (one ray per degree; green = a valid pose) and the state machine entering '
                     'RETURNING_TO_START', 'labels': []}]},
    {'key': 'gazebo_ward', 'run': 'artifacts/exploration/hospital_2026-11-18/gz_standard_final',
     'letter': 'B', 'title': 'Parking in Gazebo Sim (hospital ward)',
     'images': [
         {'kind': 'simview', 'ref': 'gz_standard_parked_angled.png', 'states': {'PARKED'},
          'crop': (250, 100, 1000, 720),
          'caption': 'Angled view while parked: Ridgeback behind G1\'s left hip, its basket toward G1 at the '
                     'worktable',
          'labels': [('G1', (455, 140), (640, 70)), ('Basket', (465, 345), (790, 330)),
                     ('Ridgeback', (500, 560), (800, 600)), ('Worktable', (290, 170), None)]},
         {'kind': 'simview', 'ref': 'gz_standard_parked_workstation.png', 'states': {'PARKED'},
          'crop': (420, 150, 760, 680),
          'caption': 'Top-down view while parked: G1 facing its worktable, Ridgeback behind its left hip',
          'labels': [('Worktable', (380, 140), None), ('G1', (377, 345), (600, 330)),
                     ('Ridgeback', (250, 500), (560, 600))]},
         {'kind': 'rviz', 'ref': 'PARKED', 'states': {'PARKED'}, 'crop': RVIZ_CROP,
          'caption': 'RViz while parked: the parking donut around G1, the robot footprint at the selected '
                     'pose and the state machine in PARKED', 'labels': []}]},
    {'key': 'gazebo_refusal', 'run': 'artifacts/exploration/hospital_2026-11-18/gz_no_feasible_final',
     'letter': 'C', 'title': 'Safety test: no feasible parking position (Gazebo Sim, hospital ward)',
     'compact': True,
     'images': [
         {'kind': 'simview', 'ref': 'gz_no_feasible_end_angled.png', 'states': {'MISSION_FAILED'},
          'crop': (230, 0, 1050, 900),
          'caption': 'Angled view after the mission: G1 at its worktable inside the pen, two panels beside it '
                     'and the bed behind it',
          'labels': [('G1', (475, 250), (150, 120)), ('Bed', (680, 640), None)]},
         {'kind': 'simview', 'ref': 'gz_no_feasible_end_workstation.png', 'states': {'MISSION_FAILED'},
          'crop': (420, 150, 760, 760),
          'caption': 'Top-down view of the pen after the mission: the worktable in front of G1, panels on '
                     'both sides and the bed behind; Ridgeback had driven back to its start',
          'labels': [('Worktable', (380, 140), None), ('G1', (377, 345), (377, 450)), ('Bed', (380, 640), None),
                     ('Panel', (103, 350), None), ('Panel', (657, 350), None)]},
         {'kind': 'rviz', 'ref': 'END_MISSION_FAILED', 'states': {'MISSION_FAILED'}, 'crop': (440, 120, 820, 360),
          'caption': 'RViz at the end: every ray of the parking donut around G1 red (no valid pose, '
                     'no_valid_candidate) and Ridgeback back at its start', 'labels': []}]},
]
# Formerly featured, kept with their captures in the archive section: the 2026-11-17
# hospital-ward trial on the post-merge mission code (both hospital ward).
ARCHIVED_FEATURED = [
    {'key': 'ward_standard', 'run': 'artifacts/exploration/hospital_2026-11-17/gz_standard',
     'letter': 'D', 'title': 'Hospital-ward trial: rear parking in Gazebo Sim (17 Nov)',
     'images': [
         {'kind': 'simview', 'ref': 'gz_standard_parked_workstation.png', 'states': {'PARKED'},
          'crop': (420, 150, 760, 720),
          'caption': 'Top-down view while parked: G1 facing its worktable, Ridgeback behind G1',
          'labels': [('Worktable', (380, 140), None), ('G1', (380, 340), (610, 330)),
                     ('Ridgeback', (520, 560), (640, 660))]},
         {'kind': 'rviz', 'ref': 'PARKED', 'states': {'PARKED'}, 'crop': (640, 120, 620, 360),
          'caption': 'RViz while parked: the parking donut and the robot footprint behind G1',
          'labels': []}]},
    {'key': 'ward_no_feasible', 'run': 'artifacts/exploration/hospital_2026-11-17/gz_no_feasible',
     'letter': 'E', 'title': 'Hospital-ward trial: no feasible position, localization failure (17 Nov)',
     'images': [
         {'kind': 'simview', 'ref': 'gz_no_feasible_end_workstation.png', 'states': {'MISSION_FAILED'},
          'crop': (380, 150, 840, 830),
          'caption': 'Top-down view after the mission: G1 boxed in by two panels, its worktable and a bed; '
                     'Ridgeback had driven back to its start',
          'labels': [('Worktable', (420, 143), None), ('Bed', (420, 690), None),
                     ('G1', (420, 343), (420, 470)), ('Panel', (195, 300), (70, 200)),
                     ('Panel', (660, 300), (770, 200))]},
         {'kind': 'rviz', 'ref': 'NAVIGATING_TO_PARKING', 'states': {'NAVIGATING_TO_PARKING'},
          'crop': (640, 140, 620, 340),
          'caption': 'RViz during the attempt: the pose planned from the wrong G1 position, beside the '
                     'panel; the LiDAR check cancelled this leg 3 s later', 'labels': []}]},
]
SIDE_NAME = {'g1_left': "G1's left", 'g1_right': "G1's right", 'g1_rear': 'behind G1',
             'g1_front': 'in front of G1'}
VETO_SHORT = {'lidar_obstacle_on_route': 'obstacle on the route (LiDAR)',
              'lidar_obstacle_in_stopping_path': 'obstacle in the stopping path (LiDAR)',
              'g1_protected_region': "entered G1's protected region"}


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
            'cause': CAUSES.get(os.path.relpath(path, ROOT)),
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
        if reach is not None and not REACH[0] <= reach <= REACH[1]:
            return ('warn', f'Parked and returned, but the reach of {reach:.3f} m (ground truth) is outside the '
                            f'required {REACH[0]:.2f}-{REACH[1]:.2f} m', REACH_OUTSIDE)
        return 'ok', ('Parked and returned' if attempts <= 1 else
                      f'Parked on attempt {attempts} and returned'), 'Complete'
    if m['outcome'] == 'stopped':
        return 'bad', 'Stopped by the operator during the search: the robot was wedged; G1 not seen', OPERATOR_STOP
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
    if m['scenario'] == 'no_feasible' and attempts:
        errors = m.get('localization_errors_m') or []
        if errors and max(errors) > MISLOCALIZED_M:
            return ('bad', f'Refusal missed: G1 mislocalized by {max(errors):.2f} m (ground truth), so a pose '
                           f'looked feasible; the LiDAR check stopped the attempt; {back}', MISSED_LOCALIZATION)
        return ('bad', f'Refusal missed: a parking attempt in a scene with no safe pose, stopped by the LiDAR '
                       f'check; {back}', 'Refusal missed')
    if 'PARKED deadline' in reason:
        return ('warn', f'Parked, but the stop was not confirmed before the PARKED deadline; {back}',
                STOP_UNCONFIRMED)
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

def capture(r, image):
    """(source file, capture time) of one featured capture of run ``r``, or (None, reason)."""
    shots = os.path.join(r['path'], 'shots')
    kind, ref, states = image['kind'], image['ref'], image['states']
    if kind == 'rviz':
        stage_time = None
        for line in open(os.path.join(shots, 'stages.txt')):
            name, when = line.split()
            if name == ref:
                stage_time = float(when)
        files = sorted(glob.glob(os.path.join(shots, f'{ref}_rviz2_*.png')), key=os.path.getsize)
        if stage_time is None or not files:
            return None, f'no RViz capture at {ref}'
        return files[-1], stage_time        # the largest window is the main RViz view
    if kind == 'simview':
        entry = next((e for e in json.load(open(os.path.join(shots, 'manifest.json')))
                      if e.get('file') == ref and e.get('ok')), None)
        if entry is None:
            return None, f'{ref} not captured'
        return os.path.join(shots, ref), float(entry['time'])
    # A recording frame: the recorder logged the mission state of every frame.
    base = os.path.dirname(r['path'])
    rows = [line.strip().split(',') for line in open(os.path.join(base, 'frames.csv'))]
    rows = [row for row in rows if row[2] in states]
    if not rows:
        return None, 'no recording frame in that state'
    index, when, _ = rows[len(rows) // 2]
    return os.path.join(base, 'frames', f'{int(index):06d}.jpg'), float(when)


def featured_image(r, image, tag):
    """The figure HTML of one verified capture of run ``r``, or (None, reason)."""
    src, when = capture(r, image)
    if src is None:
        return None, when
    actual = state_at(r, when)
    if actual not in image['states']:
        return None, f'captured during {actual}, not {sorted(image["states"])}'
    import cv2
    x, y, w, h = image['crop']
    whole = cv2.imread(src)
    ok, why = check(whole[y:y + h, x:x + w, ::-1])
    if not ok:
        return None, f'blank or invalid capture ({why})'
    shown = pages.export_jpeg(src, os.path.join(ASSETS, f'{tag}.jpg'), 1400, crop=f'{w}x{h}+{x}+{y}')
    full = pages.export_jpeg(src, os.path.join(ASSETS, f'{tag}_full.jpg'), 1600)
    t0 = r['transitions'][0][0]
    source = ('frame of the run\'s recording' if image['kind'] == 'frame'
              else os.path.basename(src))
    caption = (f'{image["caption"]}. State {actual}, {clock(when - t0)} into the mission. Cropped from '
               f'{source}{"; labels added for this page" if image["labels"] else ""}; click for the '
               f'whole capture.')
    overlay = annotations(w, h, image['labels'])
    toggle = ('<button type="button" class="labels-toggle" aria-pressed="true">Labels on</button>'
              if image['labels'] else '')
    return (f'<figure class="shot"><div class="frame"><img loading="lazy" src="{esc(shown)}" '
            f'data-full="{esc(full)}" alt="{esc(image["caption"])}">{overlay}</div>{toggle}'
            f'<figcaption>{esc(caption)}</figcaption></figure>'), None


def annotations(w, h, labels):
    """Labels drawn over a crop as their own layer: a ring on the object and a tag, joined by
    a dashed leader, in a colour the simulators do not use."""
    if not labels:
        return ''
    size = round(w / 34)
    out = []
    for text, (ax, ay), pill in labels:
        px, py = pill or (ax, ay)
        tw = len(text) * size * 0.62 + size
        if pill:
            out.append(f'<line x1="{ax}" y1="{ay}" x2="{px}" y2="{py}" class="lead"/>'
                       f'<circle cx="{ax}" cy="{ay}" r="{size * 0.45:.0f}" class="ring"/>')
        out.append(f'<rect x="{px - tw / 2:.0f}" y="{py - size * 0.8:.0f}" width="{tw:.0f}" '
                   f'height="{size * 1.6:.0f}" rx="{size * 0.35:.0f}" class="tag"/>'
                   f'<text x="{px}" y="{py + size * 0.36:.0f}" font-size="{size}" text-anchor="middle">'
                   f'{esc(text)}</text>')
    return (f'<svg class="annot" viewBox="0 0 {w} {h}" aria-hidden="true">{"".join(out)}</svg>')


# Mission steps and timelines ------------------------------------------------------------

def side_name(label):
    return SIDE_NAME.get(label, (label or '?').replace('g1_', '').replace('_', ' '))


def candidates(r):
    """(candidate poses, valid poses) of the last plan that found none, if any."""
    found = re.findall(r"No feasible parking pose \[\w+\]: [^\n]*?'candidates': (\d+), 'valid': (\d+)", r['log'])
    return tuple(int(v) for v in found[-1]) if found else None


def steps(r):
    """The mission's decisive steps: each parking attempt, a refusal, the return, the end."""
    m = r['m']
    sides = m.get('selected_sides') or []
    vetoes = {n: reason.split(':')[-1].strip() for _, n, reason in r['vetoes']}
    out = []
    for n in range(1, (m.get('attempts') or 0) + 1):
        side = (side_name(sides[n - 1]) if n <= len(sides) else
                'side not classified (G1 heading unknown)' if not sides else 'next side')
        if n in vetoes:
            out.append(('fail', '✗', f'Attempt {n}: {side}', 'rejected: ' + VETO_SHORT.get(vetoes[n], vetoes[n])))
        elif m['outcome'] == 'MISSION_COMPLETE':
            out.append(('done', '✓', f'Attempt {n}: {side}', 'parked, waited 5 s'))
    refused = candidates(r)
    if refused and not m.get('attempts'):
        out.append(('safe', '⊘', 'No safe parking pose', f'{refused[1]} of {refused[0]} candidate poses valid'))
    if m.get('returned_to_start'):
        out.append(('done', '✓', 'Returned to start', ''))
    cls, _, category = classify(r)
    end = {'Complete': 'done', 'Safe refusal': 'safe'}.get(category, 'fail')
    out.append((end + ' end', '■', m['outcome'], 'expected for this scene' if category == 'Safe refusal' else ''))
    return out


def ladder(r, vertical=False):
    items = ''.join(f'<li class="{c}"><b>{i}</b><span>{esc(t)}{f"<small>{esc(s)}</small>" if s else ""}</span></li>'
                    for c, i, t, s in steps(r))
    return f'<ol class="steps{" vertical" if vertical else ""}">{items}</ol>'


def timeline(r):
    """The mission's phases as one bar with their durations, and the decisive steps below."""
    segs = phase_segments(r)
    if not segs:
        return ''
    t0, t1 = segs[0][1], segs[-1][2]
    total = t1 - t0
    bar = []
    for p, a, b in segs:
        share = (b - a) / total
        inside = f'<span>{esc(dict(PHASES)[p])}</span>' if share > 0.16 else ''
        bar.append(f'<div class="seg ph-{p}" style="left:{(a - t0) / total * 100:.2f}%;width:{share * 100:.2f}%" '
                   f'title="{esc(dict(PHASES)[p])}: {clock(b - a)}">{inside}</div>')
    for t, n, _ in r['vetoes']:
        bar.append(f'<i class="mark fail" style="left:{(t - t0) / total * 100:.2f}%" '
                   f'title="Attempt {n} rejected at {clock(t - t0)}"></i>')
    phases = ''.join(f'<li><i class="sw ph-{p}"></i>{esc(dict(PHASES)[p])} <b>{clock(sum(b - a for q, a, b in segs if q == p))}</b></li>'
                     for p, _ in PHASES if any(q == p for q, _, _ in segs))
    return (f'<div class="timeline"><div class="tl-head"><span>Mission timeline</span><span>total '
            f'<b>{clock(total)}</b> (min:s)</span></div><div class="tl-bar">{"".join(bar)}</div>'
            f'<ul class="tl-phases">{phases}</ul>{ladder(r)}</div>')


def event_list(r):
    """Every decisive state change, for the expandable detail."""
    t0 = r['transitions'][0][0]
    events, attempt = [], 0
    sides = r['m'].get('selected_sides') or []
    for t, previous, state, reason in r['transitions']:
        if previous == 'SEARCH_G1':
            events.append((t, 'G1 confirmed by the detector', ''))
        elif state == 'G1_LOCALIZED':
            events.append((t, 'G1 localized and verified in depth', ''))
        elif state == 'EXPLORING_AROUND_G1':
            events.append((t, 'Mapping the floor around G1', ''))
        elif state == 'NAVIGATING_TO_PREPARKING':
            attempt += 1
            side = side_name(sides[attempt - 1]) if attempt <= len(sides) else 'next side'
            events.append((t, f'Attempt {attempt} started: {side}', ''))
        elif state == 'PARKED':
            events.append((t, 'Parked', 'done'))
        elif state == 'RETURNING_TO_START':
            refused = 'no feasible parking pose' in reason
            events.append((t, 'No safe parking pose: returning to start' if refused else 'Returning to start',
                           'safe' if refused else ''))
        elif state in ('MISSION_COMPLETE', 'MISSION_FAILED'):
            events.append((t, state, 'done' if state == 'MISSION_COMPLETE' else 'safe'))
    for t, n, reason in r['vetoes']:
        events.append((t, f'Attempt {n} rejected: {VETO_SHORT.get(reason.split(":")[-1].strip(), reason)}', 'fail'))
    rows, last = [], None
    for t, text, cls in sorted(events):
        if text != last:
            rows.append(f'<li class="{cls}"><span>{clock(t - t0)}</span>{esc(text)}</li>')
        last = text
    return f'<ol class="events">{"".join(rows)}</ol>'


# Charts (inline SVG, colours from the page's chart tokens) ------------------------------

CHART_W = 1000


def svg(height, body, label):
    return (f'<svg class="chart" viewBox="0 0 {CHART_W} {height}" role="img" aria-label="{esc(label)}">'
            f'{body}</svg>')


def axis(x0, x1, y, ticks, scale, text, title):
    out = [f'<line x1="{x0}" y1="{y}" x2="{x1}" y2="{y}" class="axis"/>']
    for v in ticks:
        x = scale(v)
        out.append(f'<line x1="{x:.1f}" y1="8" x2="{x:.1f}" y2="{y}" class="grid"/>'
                   f'<text x="{x:.1f}" y="{y + 20}" class="tick" text-anchor="middle">{text(v)}</text>')
    out.append(f'<text x="{(x0 + x1) / 2:.0f}" y="{y + 44}" class="axtitle" text-anchor="middle">{esc(title)}</text>')
    return ''.join(out)


def sim_name(r):
    """The simulator, and for Gazebo the world: the hospital ward or the warehouse (Isaac
    has only the ward)."""
    if r['m']['simulator'] != 'gz':
        return 'Isaac'
    return 'Gazebo ward' if ward_run(r) else 'Gazebo warehouse'


def run_label(r):
    return f'{sim_name(r)} · {r["name"]} · {r["m"]["commit"]}'


def duration_chart(runs):
    rows = [r for r in runs if phase_segments(r)]
    x0, x1, top, bar_h, gap = 380, CHART_W - 200, 10, 24, 18
    span_s = max(phase_segments(r)[-1][2] - phase_segments(r)[0][1] for r in rows)
    scale = lambda s: x0 + (x1 - x0) * s / span_s
    body = []
    for i, r in enumerate(rows):
        y = top + i * (bar_h + gap)
        segs = phase_segments(r)
        t0 = segs[0][1]
        body.append(f'<text x="{x0 - 12}" y="{y + 17}" class="label" text-anchor="end">{esc(run_label(r))}</text>')
        for phase, a, b in segs:
            xa, xb = scale(a - t0), scale(b - t0)
            body.append(f'<rect x="{xa:.1f}" y="{y}" width="{max(1.5, xb - xa - 1.5):.1f}" height="{bar_h}" '
                        f'class="ph-{phase}"><title>{esc(dict(PHASES)[phase])}: {clock(b - a)} (min:s)</title></rect>')
        end = segs[-1][2] - t0
        category = classify(r)[2]
        body.append(f'<text x="{scale(end) + 10:.1f}" y="{y + 17}" class="value">{clock(end)}  '
                    f'<tspan class="{"good" if category == "Complete" else "muted"}">'
                    f'{esc(SHORT.get(category, category))}</tspan></text>')
    y_axis = top + len(rows) * (bar_h + gap)
    ticks = [m * 60 for m in range(0, int(span_s // 60) + 1, 2)]
    body.insert(0, axis(x0, x1, y_axis, ticks, scale, lambda v: f'{v // 60:g}', 'minutes from mission start'))
    legend = ''.join(f'<span><i class="sw ph-{k}"></i>{esc(v)}</span>' for k, v in PHASES)
    table = ''.join(
        f'<tr><td>{esc(run_label(r))}</td>'
        + ''.join(f'<td>{sum(b - a for p, a, b in phase_segments(r) if p == k):.0f}</td>' for k, _ in PHASES)
        + f'<td>{phase_segments(r)[-1][2] - phase_segments(r)[0][1]:.0f}</td><td>{esc(classify(r)[1])}</td></tr>'
        for r in rows)
    head = ''.join(f'<th>{esc(v)} (s)</th>' for _, v in PHASES)
    return (f'<div class="legend">{legend}</div>'
            + svg(y_axis + 54, ''.join(body), 'Mission duration by phase for the post-merge runs')
            + f'<details class="data"><summary>Show the durations as a table</summary><div class="wrap"><table>'
              f'<tr><th>Run</th>{head}<th>Total (s)</th><th>Outcome</th></tr>{table}</table></div></details>')


def localization_chart(runs):
    """Gazebo ground-truth error of every position the mission used, per run."""
    rows = [r for r in runs if r['m']['simulator'] == 'gz' and r['m'].get('localization_errors_m')]
    x0, x1, top, row_h = 380, CHART_W - 190, 14, 36
    hi = 0.15
    scale = lambda v: x0 + (x1 - x0) * min(v, hi) / hi
    body = []
    for i, r in enumerate(rows):
        y = top + i * row_h + 12
        body.append(f'<text x="{x0 - 12}" y="{y + 5}" class="label" text-anchor="end">{esc(run_label(r))}</text>'
                    f'<line x1="{x0}" y1="{y}" x2="{x1}" y2="{y}" class="row"/>')
        for k, e in enumerate(r['m']['localization_errors_m']):
            if e > hi:     # off the scale: an arrow at its edge and the value
                body.append(f'<line x1="{scale(hi) - 40:.1f}" y1="{y}" x2="{scale(hi):.1f}" y2="{y}" class="arrow"/>'
                            f'<text x="{scale(hi) + 8:.1f}" y="{y + 5}" class="value bad">{e:.2f} m</text>')
                continue
            body.append(f'<circle cx="{scale(e):.1f}" cy="{y}" r="7" class="dot-gz">'
                        f'<title>{esc(r["name"])}, localization {k + 1}: {e:.3f} m from G1\'s true position</title></circle>')
    y_axis = top + len(rows) * row_h + 6
    body.insert(0, axis(x0, x1, y_axis, [0, 0.03, 0.06, 0.09, 0.12, 0.15], scale, lambda v: f'{v:.2f}',
                     'distance from G1\'s true position (m), Gazebo ground truth'))
    return svg(y_axis + 54, ''.join(body), 'G1 localization error against Gazebo ground truth')


def attempts_chart(runs):
    """Completed missions on the current mission logic: each attempt, rejected or parked."""
    rows = [r for r in runs if r['m']['outcome'] == 'MISSION_COMPLETE' and classify(r)[2] == 'Complete']
    out = []
    for r in rows:
        cells = ''.join(f'<li class="{c}"><b>{i}</b><span>{esc(t)}{f"<small>{esc(s)}</small>" if s else ""}</span></li>'
                        for c, i, t, s in steps(r) if t.startswith('Attempt'))
        out.append(f'<div class="attrow"><div class="attlabel">{esc(run_label(r))}<small>'
                   f'{esc(datetime.datetime.fromtimestamp(r["start"]).strftime("%d %b %Y"))} · '
                   f'{esc(r["campaign"])}</small></div><ol class="steps">{cells}</ol></div>')
    return ''.join(out)


def reach_chart(runs):
    """Gazebo ground-truth reach of every completed Gazebo mission (Isaac has none)."""
    rows = [r for r in runs if r['m']['simulator'] == 'gz' and r['m']['outcome'] == 'MISSION_COMPLETE'
            and r['m'].get('reach_truth_m') is not None]
    rows.sort(key=lambda r: (not r['final'], r['start']))
    x0, x1, top, row_h = 380, CHART_W - 210, 34, 32
    hi = 0.90
    scale = lambda v: x0 + (x1 - x0) * min(v, hi) / hi
    body = [f'<rect x="{scale(REACH[0]):.1f}" y="18" width="{scale(REACH[1]) - scale(REACH[0]):.1f}" '
            f'height="{len(rows) * row_h + 4}" class="band"><title>Required reach 0.30-0.60 m</title></rect>',
            f'<text x="{(scale(REACH[0]) + scale(REACH[1])) / 2:.1f}" y="12" class="bandlabel" '
            f'text-anchor="middle">required 0.30-0.60 m</text>']
    for i, r in enumerate(rows):
        y = top + i * row_h
        reach = r['m']['reach_truth_m']
        when = datetime.datetime.fromtimestamp(r['start']).strftime('%d %b')
        body.append(f'<text x="{x0 - 12}" y="{y + 5}" class="label" text-anchor="end">'
                    f'{esc(sim_name(r).replace("Gazebo ", ""))} · {esc(r["name"])} · {r["m"]["commit"]}'
                    f'<title>{when}</title></text>')
        if reach > hi:
            body.append(f'<line x1="{scale(hi) - 40:.1f}" y1="{y}" x2="{scale(hi):.1f}" y2="{y}" class="arrow"/>'
                        f'<text x="{scale(hi) + 8:.1f}" y="{y + 5}" class="value bad">{reach:.2f} m: false '
                        f'completion</text>')
        else:
            outside = not REACH[0] <= reach <= REACH[1]
            note = '; outside the required band' if outside else ''
            text = f'{reach:.3f} m: outside the band' if outside else f'{reach:.2f}'
            body.append(f'<circle cx="{scale(reach):.1f}" cy="{y}" r="7" class="dot-gz">'
                        f'<title>{esc(r["name"])}: {reach:.3f} m (ground truth){note}</title></circle>'
                        f'<text x="{scale(reach) + 12:.1f}" y="{y + 5}" class="value{" bad" if outside else ""}">'
                        f'{text}</text>')
    y_axis = top + len(rows) * row_h - 8
    body.insert(1, axis(x0, x1, y_axis, [0, 0.15, 0.30, 0.45, 0.60, 0.75, 0.90], scale, lambda v: f'{v:.2f}',
                     'distance from G1\'s axis to the basket centre (m), Gazebo ground truth'))
    return svg(y_axis + 54, ''.join(body), 'Parked reach against Gazebo ground truth')


# Page sections ------------------------------------------------------------------------

# What each earlier hospital-ward validation run led to, and what later evidence shows about it.
# Status says "addressed" only where a later run exercised the change; nothing here is a rate.
FOLLOW_UP = {
    'artifacts/exploration/hospital_2026-11-18/gz_standard': (
        'f568789 takes the parked views only after the mission confirms the stop',
        'Addressed in the later runs: both later Gazebo standard runs confirmed the stop with views taken '
        'during the wait'),
    'artifacts/exploration/hospital_2026-11-18/gz_no_feasible': (
        'f6ade2f moved the scene\'s bed 0.3 m closer to G1 (a scenario change; navigation unchanged)',
        'Scenario addressed: the two later no_feasible runs did not wedge. Orientation-aware navigation '
        'through narrow passages is not implemented (backlog)'),
    'artifacts/exploration/hospital_2026-11-18/gz_standard_run2': (
        'No change for the heading; b519ab0 changed the first-localization case this run met',
        'Unresolved: the 65 deg heading error was neither reproduced nor explained. The final run\'s headings '
        'were within 6.2 deg and its reach 0.509 m, which does not show the error cannot recur'),
    'artifacts/exploration/hospital_2026-11-18/gz_no_feasible_run2': (
        'None needed', 'Correct refusal on the code before the surface fallback; repeated on the final code'),
    'artifacts/exploration/hospital_2026-11-18/isaac_standard': (
        'b519ab0 keeps the surface estimate when the head region shows something taller than G1, if G1\'s legs '
        'agree',
        'Addressed in one later run: Isaac\'s final run used the fallback and completed; the case is also '
        'reproduced in tests from Gazebo views'),
}
# One verified capture per earlier run where one shows the problem (full frame, no labels).
DEV_IMAGES = {
    'artifacts/exploration/hospital_2026-11-18/gz_standard': {
        'kind': 'simview', 'ref': 'gz_standard_parked_workstation.png', 'states': {'PARKED'}, 'crop': (0, 0, 1600, 1000),
        'caption': 'gz_standard: parked behind G1 on attempt 3; the stop was then not confirmed in time', 'labels': []},
    'artifacts/exploration/hospital_2026-11-18/gz_standard_run2': {
        'kind': 'simview', 'ref': 'gz_standard_parked_workstation.png', 'states': {'PARKED'}, 'crop': (0, 0, 1600, 1000),
        'caption': 'gz_standard_run2: parked behind G1\'s right shoulder, reach 0.607 m (ground truth), after a '
                   '65 deg heading error', 'labels': []},
    'artifacts/exploration/hospital_2026-11-18/gz_no_feasible_run2': {
        'kind': 'rviz', 'ref': 'END_MISSION_FAILED', 'states': {'MISSION_FAILED'}, 'crop': (0, 60, 1280, 440),
        'caption': 'gz_no_feasible_run2: no valid pose around G1 and the robot back at its start', 'labels': []},
    'artifacts/exploration/hospital_2026-11-18/isaac_standard': {
        'kind': 'rviz', 'ref': 'END_MISSION_FAILED', 'states': {'MISSION_FAILED'}, 'crop': (0, 60, 1280, 440),
        'caption': 'isaac_standard: the mission ended without a localized G1 (the head region measured the wall)',
        'labels': []},
}


def demonstration_card(r):
    m = r['m']
    cls, _, category = classify(r)
    if category == 'Complete' and m.get('selected_sides'):
        value = f'Parked at {side_name(m["selected_sides"][-1])}'
    elif category == 'Complete':
        value = 'Parked behind G1' if r['cause'] and 'behind G1' in r['cause'] else 'Parked'
    elif category == 'Safe refusal':
        value = 'Refused safely'
    else:
        value = SHORT.get(category, category)
    facts = [f'<code>{esc(m["outcome"])}</code>']
    if m['outcome'] == 'MISSION_COMPLETE':
        vetoed = len(r['vetoes'])
        facts.append(f'{m["attempts"]} attempt{"s" if m["attempts"] > 1 else ""}'
                     + (f' ({vetoed} cancelled by the LiDAR check)' if vetoed else ''))
        facts.append('waited 5 s, returned')
    elif candidates(r):
        n, ok = candidates(r)
        facts.append(f'{ok} of {n} poses valid, no attempt, returned')
    if m.get('reach_truth_m') is not None:
        facts.append(f'reach {m["reach_truth_m"]:.3f} m')
    if m.get('side_truth') and m.get('selected_sides') and m['side_truth'] != m['selected_sides'][-1]:
        facts.append(f'by ground truth the bin stood {abs(m["bearing_from_g1_heading_deg"]):.0f} deg from G1\'s '
                     f'heading, just outside the side sector')
    if m.get('localization_errors_m'):
        facts.append(f'G1 {span(m["localization_errors_m"])} m and {deg_span(heading_errors(r))} deg off')
    if m.get('final_distance_from_start_m') is not None:
        facts.append(f'{m["final_distance_from_start_m"]:.2f} m from the start')
    if m['simulator'] != 'gz':
        facts.append('no ground truth in Isaac')
    if r['cause'] and r['m']['commit'] in FINAL_CODE:
        facts.append(esc(r['cause']))
    sim = 'Gazebo' if m['simulator'] == 'gz' else 'Isaac Sim'
    return (f'<div class="kpi {cls}"><div class="v demo">{esc(value)}</div>'
            f'<div class="l">{sim} · {esc(m["scenario"])}</div>'
            f'<div class="d">{" · ".join(facts)}</div>'
            f'<div class="d"><code>{esc(r["name"])}</code> · code {esc(r["m"]["commit"])} · '
            f'{clock(m["mission_s"])} min:s</div></div>')


def development_history(earlier):
    rows, figs = [], []
    for r in sorted(earlier, key=lambda r: r['start']):
        m = r['m']
        cls, outcome, category = classify(r)
        follow, status = FOLLOW_UP.get(r['rel'], ('-', '-'))
        meas = []
        if m.get('localization_errors_m'):
            meas.append(f'G1 {span(m["localization_errors_m"])} m')
        if m.get('reach_truth_m') is not None:
            meas.append(f'reach {m["reach_truth_m"]:.3f} m')
        if heading_errors(r):
            meas.append(f'heading {deg_span(heading_errors(r))} deg')
        if m.get('attempts'):
            meas.append(f'{m["attempts"]} attempt{"s" if m["attempts"] > 1 else ""}')
        rows.append(
            f'<tr><td><code>{esc(r["name"])}</code><br>{esc(sim_name(r))}</td><td><code>{esc(r["code"])}</code></td>'
            f'<td><span class="badge {cls}">{esc(category)}</span><br><small>{esc(outcome)}</small></td>'
            f'<td>{esc(", ".join(meas)) or "-"}</td><td>{esc(r["cause"] or outcome)}</td>'
            f'<td>{esc(follow)}</td><td>{esc(status)}</td></tr>')
        image = DEV_IMAGES.get(r['rel'])
        if image and ward_run(r):
            html_, why = featured_image(r, image, f'dev_{r["name"]}')
            if html_ is None:
                print(f'development image of {r["name"]} excluded: {why}')
            else:
                figs.append(html_)
    return (f'<details class="archive dev"><summary><h2>Development and debugging history</h2>'
            f'<span class="note">{len(earlier)} earlier hospital-ward runs (17 Nov), each on the code it ran: what '
            f'failed, why, what changed afterwards, and what is still open</span></summary>'
            f'<div class="wrap"><table class="devtable"><tr><th>Run</th><th>Code</th><th>Outcome</th>'
            f'<th>Measurements (Gazebo ground truth)</th><th>What happened</th><th>What changed afterwards</th>'
            f'<th>Status</th></tr>{"".join(rows)}</table></div>'
            f'<p class="note">gz_no_feasible was stopped by the operator during its search; no capture shows the '
            f'wedged robot. The 2026-11-17 hospital-ward trial and every warehouse run are in the '
            f'<a href="#history">records</a>.</p><div class="media row">{"".join(figs)}</div></details>')


def overview(final, scenario_count):
    demos = [r for r in final if r['m']['commit'] in FINAL_CODE]
    earlier = [r for r in final if r not in demos]
    order = {'standard': 0, 'no_feasible': 1}
    demos.sort(key=lambda r: (order.get(r['m']['scenario'], 2), r['m']['simulator'] != 'gz', r['start']))
    return (f'<section id="overview"><h2>Final implementation: demonstrated behaviours</h2>'
            f'<p class="caveat"><b>Demonstrations, not reliability statistics.</b> {len(demos)} runs in the '
            f'hospital ward on the final mission code ({FINAL_CODE[0]}; {", ".join(FINAL_CODE[1:])} changed only '
            f'comments and documentation). Each shows that the behaviour happens, not how often: no success rate is '
            f'computed from these {len(demos)} runs. Gazebo measurements are ground truth used only for evaluation; '
            f'Isaac records the mission\'s own estimates.</p>'
            f'<div class="kpis-main three">{"".join(demonstration_card(r) for r in demos)}</div>'
            f'{development_history(earlier)}</section>')


def comparison(by_key, refusal):
    isaac, gz = by_key['isaac_side'], by_key['gazebo_ward']

    def arrival(r):
        found = re.findall(r'Arrived at parking: error xy=([\d.]+) m', r['log'])
        return f'{float(found[-1]):.3f} m' if found else '-'

    def phase(r, key):
        return clock(sum(b - a for p, a, b in phase_segments(r) if p == key))
    im, gm = isaac['m'], gz['m']
    na = '<span class="na">no ground truth</span>'
    gt, est, log = ('<span class="src gt">ground truth</span>', '<span class="src est">mission estimate</span>',
                    '<span class="src log">mission log</span>')
    rows = [
        ('Mission duration', clock(im['mission_s']), clock(gm['mission_s']), log),
        ('of which: search for G1', phase(isaac, 'search'), phase(gz, 'search'), log),
        ('Arrival error at the parking pose', arrival(isaac), arrival(gz), est),
        ('G1 localization error', na, f'{span(gm["localization_errors_m"])} m', gt),
        ('G1 heading estimate', f'{im["g1_heading_log"]} deg', f'{gm["g1_heading_log"]} deg (error '
         f'{gm["heading_error_deg"]:.1f} deg)', est),
        ('Parked', side_name(im['selected_sides'][-1]) + f', attempt {im["attempts"]}',
         side_name(gm['selected_sides'][-1]) + f', attempt {gm["attempts"]}', log),
        ('Parked reach (required 0.30-0.60 m)', na, f'{gm["reach_truth_m"]:.3f} m', gt),
        ('Basket to G1\'s lower body', na, f'{gm["parked_truth_clearance_m"]:.3f} m', gt),
        ('Final distance from the start', na, f'{gm["final_distance_from_start_m"]:.2f} m', gt),
    ]
    table = ''.join(f'<tr><th>{esc(k)}</th><td>{a}</td><td>{b}</td><td>{src}</td></tr>' for k, a, b, src in rows)
    errs = refusal['m'].get('localization_errors_m') or []
    return (f'<section id="compare"><h2>Gazebo Sim and Isaac Sim</h2>'
            f'<p class="lead">The same autonomy code, hospital ward, scene and parking priorities in both '
            f'simulators. One run each on the final mission code: a demonstration, not a rate.</p>'
            f'<div class="cmp"><div class="col isaac"><h3>Isaac Sim</h3><p class="sub"><code>{esc(isaac["name"])}</code>'
            f' · code {esc(im["commit"])}</p>{ladder(isaac, True)}</div>'
            f'<div class="col gz"><h3>Gazebo Sim</h3><p class="sub"><code>{esc(gz["name"])}</code> · code '
            f'{esc(gm["commit"])}</p>{ladder(gz, True)}</div></div>'
            f'<div class="wrap"><table class="measures"><tr><th>Measurement</th><th>Isaac Sim</th><th>Gazebo Sim</th>'
            f'<th>Source</th></tr>{table}</table></div>'
            f'<p class="note">Gazebo samples the true poses of the robot and G1; Isaac records only what the '
            f'mission itself estimated, so its ground-truth cells are empty rather than estimated.</p>'
            f'<div class="col safety"><h3>Safety test: no feasible parking position (Gazebo)</h3>'
            f'<p class="sub"><code>{esc(refusal["name"])}</code> · code {esc(refusal["m"]["commit"])} · '
            f'{esc(SHORT.get(classify(refusal)[2], classify(refusal)[2]))}'
            + (f' · G1 localized {", ".join(f"{e:.3f}" for e in errs)} m from its true position (ground truth)'
               if errs else '') + f'</p>{ladder(refusal)}</div>'
            f'<details class="more"><summary>Why the two simulators parked differently</summary>'
            f'<p>Each parking leg is checked live against both LiDAR scans; a return on the route ahead or in '
            f'the stopping path cancels the leg before contact, and the mission tries the next direction in its '
            f'priority order (G1\'s sides, then behind, then in front). {esc(parking_story(isaac, "Isaac"))} '
            f'{esc(parking_story(gz, "Gazebo"))} {esc(veto_sentence(gz))}</p></details></section>')


def parking_story(r, sim):
    m = r['m']
    sides = [side_name(s) for s in m.get('selected_sides') or []]
    if not sides:
        return ''
    if len(sides) == 1:
        return f'In {sim} the first approach, to {sides[0]}, passed the check and parked.'
    return (f'In {sim} the check rejected {", ".join(sides[:-1])} and the approach to {sides[-1]} '
            f'parked (attempt {m["attempts"]}).')


def veto_sentence(r):
    parts = []
    for (_, n, reason), (nearest, gap) in zip(r['vetoes'], r['veto_detail']):
        why = reason.split(':')[-1].strip()
        parts.append(f'attempt {n}: {VETO_TEXT.get(why, why)}, {nearest:.2f} m from the robot while the '
                     f'robot was {gap:.2f} m from G1')
    return ('In the Gazebo run, ' + '; '.join(parts) + ' (LiDAR measurements).') if parts else ''


def kpis(r, key):
    m = r['m']
    cls, _, category = classify(r)
    sides = m.get('selected_sides') or []
    expected = ('expected refusal' if category == 'Safe refusal' else
                'expected: a refusal without an attempt' if m['scenario'] == 'no_feasible' else '')
    items = [('Outcome', f'<code>{esc(m["outcome"])}</code>', expected)]
    if m['outcome'] == 'MISSION_COMPLETE':
        items.append(('Parked', esc(side_name(sides[-1])) if sides else 'yes', '' if sides else 'side not classified'))
    else:
        items.append(('Parked', 'no', 'no safe pose' if candidates(r) and not m.get('attempts') else ''))
    items.append(('Parking attempts', str(m.get('attempts') or 0),
                  f'{len(r["vetoes"])} rejected by the LiDAR check' if r['vetoes'] else ''))
    if m.get('reach_truth_m') is not None:
        items.append(('Parked reach', f'{m["reach_truth_m"]:.3f} m', 'ground truth; required 0.30-0.60'))
    if m.get('localization_errors_m'):
        items.append(('G1 localization error', f'{span(m["localization_errors_m"])} m', 'ground truth'))
    if heading_errors(r):
        items.append(('G1 heading error', f'{deg_span(heading_errors(r))} deg', 'ground truth'))
    if m.get('final_distance_from_start_m') is not None:
        items.append(('Back at start', f'{m["final_distance_from_start_m"]:.2f} m away', 'ground truth'))
    elif m.get('returned_to_start'):
        items.append(('Back at start', 'yes', 'within 0.40 m; mission estimate'))
    if key == 'isaac_side':
        found = re.findall(r'Arrived at parking: error xy=([\d.]+) m', r['log'])
        items.append(('Arrival error', f'{float(found[-1]):.3f} m', 'mission estimate; no ground truth'))
    if key in ('gazebo_refusal', 'ward_no_feasible') and candidates(r):
        n, ok = candidates(r)
        items.append(('Valid parking poses', f'{ok} of {n}', 'all blocked by the scene'))
    items.append(('Duration', clock(m['mission_s']), 'min:s'))
    return ''.join(f'<div class="k"><span class="l">{esc(l)}</span><span class="v">{v}</span>'
                   f'{f"<span class=d>{esc(d)}</span>" if d else ""}</div>' for l, v, d in items)


def ward_run(r):
    """Whether the run loaded the hospital ward: only its captures are shown on this page."""
    return r['m'].get('world') == WARD_WORLD or r['m']['commit'] in WARD_TRIAL


def featured(r, spec, summary, details, note=''):
    figs = []
    if not ward_run(r):
        print(f'{spec["key"]} images excluded: {r["rel"]} did not load the hospital ward')
    for i, image in enumerate(spec['images'] if ward_run(r) else []):
        html_, why = featured_image(r, image, f'{spec["key"]}_{i}')
        if html_ is None:
            print(f'{spec["key"]} image {image["ref"]} excluded: {why}')
            continue
        figs.append(html_)
    lead = figs[0] if figs else pages.missing('No verified capture')
    if not figs:
        figs = [lead]
    side = ''.join(figs[1:])
    badge_cls, outcome, _ = classify(r)
    facts = ''.join(f'<div><b>{esc(k)}</b><span>{v}</span></div>' for k, v in details)
    return (f'<article class="exp" id="{spec["key"]}"><header><span class="letter">{spec["letter"]}</span>'
            f'<h3>{esc(spec["title"])}</h3><span class="badge {badge_cls}">{esc(outcome)}</span>'
            f'<span class="meta">{esc(r["name"])} · code {esc(r["m"]["commit"])} · '
            f'{esc(datetime.datetime.fromtimestamp(r["start"]).strftime("%d %b %Y %H:%M"))}</span></header>'
            f'{note}<p class="summary">{summary}</p><div class="kpis">{kpis(r, spec["key"])}</div>'
            + (f'<div class="media row">{"".join(figs)}</div>' if spec.get('compact') else
               f'<div class="media"><div class="main">{lead}</div><div class="aside">{side}</div></div>')
            + f'{timeline(r)}<details class="more"><summary>Details: measurements and every state change'
            f'</summary><div class="factlist">{facts}</div>{event_list(r)}</details></article>')


def heading_errors(r):
    """Every G1 heading the mission estimated, against Gazebo ground truth (deg); [] in Isaac."""
    truth = r['m'].get('g1_truth')
    if not truth:
        return []
    out = []
    for value in re.findall(r'G1 heading (-?[\d.]+) deg from depth', r['log']):
        diff = (float(value) - math.degrees(truth[2]) + 180.0) % 360.0 - 180.0
        out.append(abs(diff))
    return out


def deg_span(values):
    lo, hi = min(values), max(values)
    return f'{lo:.1f}' if f'{lo:.1f}' == f'{hi:.1f}' else f'{lo:.1f}-{hi:.1f}'


def explanations(by_key):
    """(one-sentence summary, detail rows) per featured experiment, from the runs' records."""
    a, b, c = by_key['isaac_side'], by_key['gazebo_ward'], by_key['gazebo_refusal']
    d, e = by_key['ward_standard'], by_key['ward_no_feasible']
    am, bm, cm, dm, em = a['m'], b['m'], c['m'], d['m'], e['m']
    reseed = re.findall(r'position re-seeded on the head \((-?[\d.]+), (-?[\d.]+)\)', e['log'])
    out = {
        'isaac_side': (
            f'Ridgeback searched the ward, found and localized G1, parked at '
            f'{side_name(am["selected_sides"][-1])} on attempt {am["attempts"]}, waited five seconds and drove '
            f'back to its start.',
            [('First localization', 'The head region measured the wall behind G1; depth there stands taller than '
                                    'G1, so the detector\'s estimate on G1 was kept and G1\'s legs confirmed it '
                                    '(the surface fallback of b519ab0)'),
             ('G1 heading estimate', f'{am["g1_heading_log"]} deg (configured -90 deg)'),
             ('Return', 'Ended within the shared 0.40 m goal tolerance of the start, any heading (mission estimate)'),
             ('Ground truth', 'Isaac records none: positions and distances here are the mission\'s own estimates'),
             ('Evidence', f'<code>{esc(a["rel"])}</code>, outside the repository')]),
        'gazebo_ward': (
            f'Ridgeback searched the ward, found and localized G1; the LiDAR check rejected the approach to '
            f'{side_name(bm["selected_sides"][0])}, and it parked on attempt {bm["attempts"]}, '
            f'{bm["reach_truth_m"]:.3f} m from G1\'s axis (ground truth; required {REACH[0]:.2f}-{REACH[1]:.2f} m), '
            f'waited five seconds and drove back to {bm["final_distance_from_start_m"]:.2f} m from its start.',
            [('Why not the first side', esc(veto_sentence(b))),
             ('Where it parked', f'Planned as {side_name(bm["selected_sides"][-1])}; by ground truth the bin stood '
                                 f'{abs(bm["bearing_from_g1_heading_deg"]):.0f} deg from G1\'s heading, behind its '
                                 f'left hip, just past the 45 deg side sector (sectors only order the checks)'),
             ('G1 heading error', f'{deg_span(heading_errors(b))} deg over its {len(heading_errors(b))} '
                                  f'localizations (ground truth)'),
             ('Basket to G1', f'{bm["parked_truth_clearance_m"]:.3f} m from G1\'s lower body (ground truth)'),
             ('Earlier ward completion', 'gz_standard_run2 (f6ade2f, before the surface fallback) parked 0.607 m from '
                                         'G1, outside the required band, after a 65 deg heading error; see the table'),
             ('Return', f'Ended {bm["final_distance_from_start_m"]:.2f} m from the true start (ground truth)'),
             ('Evidence', f'<code>{esc(b["rel"])}</code>, outside the repository')]),
        'ward_standard': (
            f'Ridgeback searched the ward, found and localized G1, and the LiDAR check rejected both sides of G1; '
            f'it parked behind G1 on attempt {dm["attempts"]}, {dm["reach_truth_m"]:.3f} m from G1\'s axis '
            f'(ground truth; required {REACH[0]:.2f}-{REACH[1]:.2f} m), waited five seconds and drove back to '
            f'{dm["final_distance_from_start_m"]:.2f} m from its start.',
            [('World', 'Gazebo <code>parking_demo</code> of trial commit 822ae6d, on the post-merge mission code'),
             ('Why not beside G1', esc(veto_sentence(d))),
             ('Basket to G1', f'{dm["parked_truth_clearance_m"]:.3f} m from G1\'s lower body (ground truth)'),
             ('G1 heading error', f'{dm["heading_error_deg"]:.1f} deg (ground truth)'),
             ('Return', f'Ended {dm["final_distance_from_start_m"]:.2f} m from the true start (ground truth)'),
             ('Evidence', f'<code>{esc(d["rel"])}</code>, outside the repository')]),
        'ward_no_feasible': (
            f'The safety scene did not refuse as intended: G1 was localized {span(em["localization_errors_m"])} m '
            f'from its true position, so a pose beside the panel looked feasible. The live LiDAR check cancelled '
            f'that attempt before contact; G1 was not relocalized, and the robot drove back to its start.',
            [('What went wrong', 'The head region measured the far panel behind the half-hidden G1, and the check '
                                 're-seeded G1 at '
                                 + (f'({float(reseed[-1][0]):.2f}, {float(reseed[-1][1]):.2f})' if reseed else '?')
                                 + ', on that panel; the planned pose lay beyond the near panel'),
             ('Safety', f'LiDAR check cancelled the final leg in the stopping path; minimum ground-truth clearance '
                        f'{em["min_truth_clearance_m"]:.3f} m ({esc(em.get("min_truth_clearance_pair") or "")})'),
             ('Final reason', f'<code>{esc((em.get("final_reason") or "")[:140])}</code>'),
             ('Consequence', 'A position with depth above G1\'s height is now refused, with the surface fallback; '
                             'both simulators then moved to the ward (C is the rerun)'),
             ('Evidence', f'<code>{esc(e["rel"])}</code>, outside the repository')]),
    }
    n, ok = candidates(c) or (0, 0)
    if classify(c)[2] == 'Safe refusal':
        summary = (f'Two panels, the worktable and a bed leave no room for Ridgeback near G1: G1 was localized '
                   f'{span(cm["localization_errors_m"])} m from its true position, none of the {n} candidate poses '
                   f'was valid, so the mission made no parking attempt, drove back to its start and reported why.')
    else:
        summary = (f'This run did not show a safe refusal: {esc(classify(c)[1])}. It is reported as it ended.')
    reseeded = re.findall(r'lies ([+-][\d.]+) m (?:in front of|behind) G1\'s head at ([\d.]+) m', c['log'])
    verified = re.findall(r'G1 verified in depth: head [\d.]+ m from the camera, torso points \[(\d+)[^\]]*\], '
                          r'points above G1 \[(\d+)', c['log'])
    checks = ('The detector\'s surface estimate lay ' + f'{abs(float(reseeded[-1][0])):.2f} m beyond G1\'s head '
              'on the worktable; the head check moved the position onto G1\'s head and its legs confirmed the axis'
              if reseeded else 'The surface estimate agreed with G1\'s head')
    if verified:
        checks += (f'; depth showed {verified[-1][0]} torso points and {verified[-1][1]} above G1\'s height. '
                   'Planning used that verified position')
    out['gazebo_refusal'] = (summary, [
        ('Localization checks', checks),
        ('Scene', 'G1 at its worktable inside a pen: panels 0.8 m to each side and a 0.5 m high bed 0.95 m behind'),
        ('Planner result', f'<code>{esc((cm.get("final_reason") or "-")[:160])}</code>'
                           + (f'; {ok} of {n} candidate poses valid' if n else '')),
        ('G1 localization error', f'{span(cm["localization_errors_m"])} m (ground truth)'
                                  if cm.get('localization_errors_m') else 'G1 not localized'),
        ('G1 heading error', f'{deg_span(heading_errors(c))} deg (ground truth)' if heading_errors(c) else '-'),
        ('Return', f'Ended {cm["final_distance_from_start_m"]:.2f} m from the true start (ground truth)'
                   if cm.get('final_distance_from_start_m') is not None else '-'),
        ('Earlier refusals', 'gz_no_feasible_run2 (ward, f6ade2f) and the post-merge warehouse run (5b5133c) also '
                             'refused with no attempt; both are in the table'),
        ('Evidence', f'<code>{esc(c["rel"])}</code>, outside the repository')])
    return out


def history(runs, archived=''):
    rows = []
    for r in sorted(runs, key=lambda r: -r['start']):
        m = r['m']
        cls, outcome, category = classify(r)
        sim = sim_name(r)
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
        if r['cause']:
            details.append(('Cause', r['cause']))
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
    filters = (select('sim', 'Simulator', sorted({sim_name(r) for r in runs}))
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
            f'1-2.5 s; Isaac runs record the mission\'s own estimates only.</p>'
            f'<p class="note">Warehouse runs (Gazebo <code>depot</code>, still available with '
            f'<code>world:=depot</code>) keep their measurements, outcomes and code versions here; their '
            f'screenshots stay in their evidence folders and are not shown on this page.</p>'
            f'<h3 class="archived">Formerly featured: the hospital-ward trial of 17 November</h3>'
            f'<p class="note">Gazebo\'s first runs in the ward, on the post-merge mission code. Its safety scene '
            f'mislocalized G1, which led to the localization changes the validation runs above tested.</p>'
            f'{archived}</details>{script}</section>')


def span(values):
    lo, hi = min(values), max(values)
    return f'{lo:.3f}' if f'{lo:.3f}' == f'{hi:.3f}' else f'{lo:.3f}-{hi:.3f}'


def fmt(value, unit=''):
    return '-' if value in (None, []) else f'{value}{unit}'


RESULTS_CSS = """
  /* Status colours follow the RViz mission state machine (hud_style.py): completed green,
     active blue, failed red, pending grey; light mode uses darker steps of the same hues. */
  :root { --done: #1f8f4a; --done-bg: #e7f5ec; --now: #1f6fc2; --fail: #c93a3a; --fail-bg: #fbeceb;
          --safe-c: #9a6b00; --safe-bg2: #fbf3dc; --muted: #6f7884;
          --gz: #2a78d6; --annot: #ffb000;
          --ph-search: #7f8b99; --ph-locate: #c9dff5; --ph-park: #1f6fc2; --ph-wait: #2e9b57; --ph-return: #4f5a66;
          --ph-ink-locate: #1d2530; --band: rgba(31,143,74,.12); }
  @media (prefers-color-scheme: dark) {
    :root { --done: #5ac878; --done-bg: #183324; --now: #50aaff; --fail: #ff5555; --fail-bg: #3a1d1d;
            --safe-c: #ebb450; --safe-bg2: #33280f; --muted: #8b96a3; --gz: #50aaff;
            --ph-search: #5b6673; --ph-locate: #2d4a66; --ph-park: #3a8de0; --ph-wait: #3fa864; --ph-return: #8b96a3;
            --ph-ink-locate: #e8edf3; --band: rgba(90,200,120,.14); }
  }
  .subtitle { font-size: 1.1rem; color: var(--body); margin: 0 0 .6rem; }
  section { scroll-margin-top: 3rem; }
  .lead { max-width: 90ch; font-size: 1rem; }
  h2 { font-size: 1.45rem; margin-top: 2.8rem; }
  /* Overview */
  .kpis-main { display: grid; gap: 1rem; grid-template-columns: repeat(4, minmax(0, 1fr)); margin: 1.8rem 0 1rem; }
  .kpi { background: var(--card); border: 1px solid var(--line); border-top: 4px solid var(--muted);
         border-radius: 10px; padding: 1rem 1.2rem 1.1rem; }
  .kpi.ok { border-top-color: var(--done); } .kpi.safe { border-top-color: var(--safe-c); }
  .kpi.bad { border-top-color: var(--fail); } .kpi.neutral { border-top-color: var(--now); }
  .kpis-main.three { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .kpi .v.demo { font-size: 1.55rem; }
  details.dev { margin-top: 1.2rem; }
  .devtable td { font-size: .86rem; vertical-align: top; }
  .kpi .v { color: var(--ink); font-size: 2.6rem; font-weight: 650; line-height: 1.05; }
  .kpi .l { color: var(--ink); font-size: 1.02rem; font-weight: 600; margin-top: .25rem; }
  .kpi .d { color: var(--faint); font-size: .86rem; margin-top: .3rem; }
  .pips { list-style: none; display: flex; flex-wrap: wrap; gap: .5rem; padding: 0; margin: 0 0 1.2rem; max-width: none; }
  .pips li, .steps li { display: flex; gap: .5rem; align-items: flex-start; background: var(--card);
         border: 1px solid var(--line); border-left: 4px solid var(--muted); border-radius: 8px;
         padding: .4rem .7rem; font-size: .9rem; color: var(--ink); }
  .pips small, .steps small { display: block; color: var(--faint); font-size: .8rem; }
  .pips b, .steps b { font-size: 1rem; line-height: 1.35; }
  li.done { border-left-color: var(--done); } li.done > b { color: var(--done); }
  li.fail { border-left-color: var(--fail); } li.fail > b { color: var(--fail); }
  li.safe { border-left-color: var(--safe-c); } li.safe > b { color: var(--safe-c); }
  li.end.done { background: var(--done-bg); } li.end.safe { background: var(--safe-bg2); }
  li.end.fail { background: var(--fail-bg); }
  li.end span { font-family: var(--mono); font-weight: 600; font-size: .86rem; }
  .secondary { display: grid; gap: 1.2rem; grid-template-columns: minmax(0, 1.3fr) minmax(0, 1fr); align-items: start; }
  .secondary h3 { color: var(--ink); font-size: 1rem; margin-bottom: .5rem; }
  table.compact th, table.compact td, table.measures th, table.measures td { padding: .45rem .7rem; font-size: .92rem; }
  table.compact td { white-space: nowrap; }
  .caveat { background: var(--accent-bg); border-radius: 10px; padding: .9rem 1.1rem; margin: 2rem 0 0; font-size: .92rem;
            color: var(--body); }
  .caveat b { color: var(--ink); }
  .caveat.trial { margin: .4rem 0 1rem; border-left: 4px solid var(--safe-c); }
  .src { font-size: .72rem; font-family: var(--mono); padding: .05rem .45rem; border-radius: 999px; white-space: nowrap;
         border: 1px solid var(--line); color: var(--faint); }
  .src.gt { color: var(--done); border-color: var(--done); }
  .src.est { color: var(--now); border-color: var(--now); }
  .na { color: var(--faint); font-style: italic; font-size: .88rem; }
  /* Comparison */
  .cmp { display: grid; gap: 1rem; grid-template-columns: 1fr 1fr; margin-bottom: 1rem; }
  .col { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 1rem 1.2rem; }
  .col h3 { color: var(--ink); font-size: 1.15rem; }
  .col.safety { margin-top: 1rem; }
  .sub { color: var(--faint); font-size: .84rem; margin: .2rem 0 .7rem; }
  .steps { list-style: none; padding: 0; margin: .6rem 0 0; display: flex; flex-wrap: wrap; gap: .5rem; max-width: none;
           counter-reset: none; }
  .steps:not(.vertical) li:not(:last-child)::after { content: '→'; color: var(--faint); margin-left: .3rem; }
  .steps.vertical { flex-direction: column; }
  .steps.vertical li { font-size: .98rem; }
  table.measures th { width: 30%; font-weight: 600; }
  table.measures tr:first-child th { width: auto; }
  details.more { margin-top: .9rem; }
  details.more > summary { cursor: pointer; color: var(--accent); font-size: .92rem; }
  details.more p { font-size: .92rem; }
  /* Featured experiments */
  .exp { background: var(--card); border: 1px solid var(--line); border-radius: 14px; padding: 1.4rem 1.5rem;
         margin: 1.6rem 0; scroll-margin-top: 3rem; }
  .exp > header { display: flex; flex-wrap: wrap; gap: .5rem .9rem; align-items: center; }
  .exp > header h3 { color: var(--ink); font-size: 1.3rem; }
  .exp > header .meta { margin-left: auto; font-family: var(--mono); font-size: .78rem; color: var(--faint); }
  .letter { display: inline-grid; place-items: center; width: 2rem; height: 2rem; border-radius: 50%;
            background: var(--accent); color: var(--card); font-weight: 700; }
  .summary { font-size: 1.05rem; color: var(--ink); margin: .7rem 0 1rem; max-width: 100ch; }
  .kpis { display: flex; flex-wrap: wrap; gap: .6rem; margin-bottom: 1.1rem; }
  .kpis .k { background: var(--paper); border: 1px solid var(--line); border-radius: 8px; padding: .45rem .8rem;
             min-width: 9rem; }
  .kpis .l { display: block; font-size: .76rem; color: var(--faint); text-transform: uppercase; letter-spacing: .04em; }
  .kpis .v { display: block; font-size: 1.15rem; color: var(--ink); font-weight: 600; }
  .kpis .v code { font-size: .92rem; }
  .kpis .d { display: block; font-size: .76rem; color: var(--faint); }
  .media { display: grid; gap: 1rem; grid-template-columns: minmax(0, 1.7fr) minmax(0, 1fr); align-items: start; }
  .aside { display: grid; gap: 1rem; }
  .media.row { grid-template-columns: repeat(3, minmax(0, 1fr)); }
  h3.archived { margin: 2rem 0 .3rem; font-size: 1.05rem; color: var(--ink); }
  figure.shot .frame { position: relative; }
  figure.shot img { background: var(--paper); }
  figure.shot figcaption { font-size: .84rem; line-height: 1.45; }
  svg.annot { position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; }
  svg.annot .lead { stroke: var(--annot); stroke-width: 3; stroke-dasharray: 8 6; }
  svg.annot .ring { fill: none; stroke: var(--annot); stroke-width: 4; }
  svg.annot .tag { fill: rgba(17,20,26,.82); stroke: var(--annot); stroke-width: 2.5; }
  svg.annot text { fill: #fff; font-weight: 600; font-family: -apple-system, "Segoe UI", Roboto, sans-serif; }
  figure.labels-off svg.annot { display: none; }
  .labels-toggle { font: inherit; font-size: .76rem; margin-top: .35rem; color: var(--accent); background: var(--card);
                   border: 1px solid var(--line); border-radius: 6px; padding: .1rem .5rem; cursor: pointer; }
  .timeline { margin-top: 1.3rem; border-top: 1px solid var(--line); padding-top: 1rem; }
  .tl-head { display: flex; justify-content: space-between; font-size: .92rem; color: var(--ink); font-weight: 600;
             margin-bottom: .4rem; }
  .tl-head b { font-family: var(--mono); }
  .tl-bar { position: relative; height: 34px; border-radius: 6px; overflow: hidden; background: var(--line); }
  .tl-bar .seg { position: absolute; top: 0; bottom: 0; border-right: 2px solid var(--card); display: flex;
                 align-items: center; padding: 0 .6rem; overflow: hidden; white-space: nowrap; }
  .tl-bar .seg span { color: #fff; font-size: .86rem; font-weight: 600; }
  .tl-bar .ph-locate span { color: var(--ph-ink-locate); }
  .tl-bar .mark { position: absolute; top: 0; bottom: 0; width: 3px; margin-left: -1px; background: var(--fail); }
  .tl-phases { list-style: none; display: flex; flex-wrap: wrap; gap: .3rem 1.3rem; padding: 0; margin: .55rem 0 .2rem;
               font-size: .92rem; max-width: none; }
  .tl-phases b { font-family: var(--mono); color: var(--ink); font-weight: 600; }
  .sw { display: inline-block; width: .85rem; height: .85rem; border-radius: 3px; margin-right: .4rem; vertical-align: -1px; }
  .ph-search { background: var(--ph-search); fill: var(--ph-search); }
  .ph-locate { background: var(--ph-locate); fill: var(--ph-locate); }
  .ph-park { background: var(--ph-park); fill: var(--ph-park); }
  .ph-wait { background: var(--ph-wait); fill: var(--ph-wait); }
  .ph-return { background: var(--ph-return); fill: var(--ph-return); }
  .factlist > div { display: grid; grid-template-columns: 13rem 1fr; gap: .8rem; font-size: .92rem;
                    padding: .45rem 0; border-bottom: 1px solid var(--line); }
  .factlist b { color: var(--ink); font-weight: 600; }
  .events { list-style: none; padding: 0; margin: .8rem 0 0; font-size: .9rem; columns: 2; column-gap: 2rem; max-width: none; }
  .events li { break-inside: avoid; padding: .15rem 0; }
  .events li span { font-family: var(--mono); color: var(--faint); display: inline-block; width: 3.6rem; }
  .events li.fail { color: var(--fail); } .events li.done { color: var(--done); } .events li.safe { color: var(--safe-c); }
  /* Charts */
  .charts { display: grid; gap: 1.2rem; }
  .chartcard { background: var(--card); border: 1px solid var(--line); border-radius: 12px; padding: 1.2rem 1.4rem; }
  .chartcard h3 { color: var(--ink); font-size: 1.15rem; }
  .chartcard p.q { color: var(--body); font-size: .95rem; margin: .35rem 0 .8rem; }
  .chartcard p.q b { color: var(--ink); }
  .chartcard .note { margin-top: .6rem; }
  .legend { display: flex; flex-wrap: wrap; gap: .3rem 1.2rem; font-size: .9rem; color: var(--body); margin: .2rem 0 .6rem; }
  svg.chart { width: 100%; max-width: 1080px; height: auto; display: block; font-family: inherit; }
  svg .axis { stroke: var(--faint); stroke-width: 1; } svg .grid { stroke: var(--line); stroke-width: 1; }
  svg .row { stroke: var(--line); stroke-width: 1; }
  svg .tick { fill: var(--body); font-size: 14px; } svg .axtitle { fill: var(--faint); font-size: 14px; }
  svg .label { fill: var(--ink); font-size: 15px; }
  svg .value { fill: var(--ink); font-size: 14px; font-weight: 600; } svg .value.bad { fill: var(--fail); }
  svg .good { fill: var(--done); font-weight: 600; } svg .muted { fill: var(--faint); font-weight: 400; }
  svg .bandlabel { fill: var(--done); font-size: 14px; font-weight: 600; }
  svg .dot-gz { fill: var(--gz); stroke: var(--card); stroke-width: 2.5; }
  svg .band { fill: var(--band); } svg .arrow { stroke: var(--fail); stroke-width: 2.5; }
  details.data { margin-top: .5rem; } details.data summary { cursor: pointer; color: var(--accent); font-size: .88rem; }
  .attrow { display: grid; grid-template-columns: 19rem 1fr; gap: 1rem; padding: .7rem 0; border-bottom: 1px solid var(--line); }
  .attrow .steps { margin: 0; }
  .attlabel { font-size: .95rem; color: var(--ink); } .attlabel small { display: block; color: var(--faint); font-size: .82rem; }
  /* Historical records */
  #history { margin-top: 3rem; }
  details.archive { background: var(--card); border: 1px solid var(--line); border-radius: 12px; padding: 1rem 1.3rem; }
  details.archive > summary { cursor: pointer; list-style: none; }
  details.archive > summary::-webkit-details-marker { display: none; }
  details.archive > summary h2 { display: inline; font-size: 1.2rem; margin: 0; }
  details.archive > summary h2::before { content: '▸ '; color: var(--accent); }
  details.archive[open] > summary h2::before { content: '▾ '; }
  details.archive > summary .note { display: block; margin: .3rem 0 0; }
  details.archive[open] > summary .note { margin-bottom: .8rem; }
  .filters { display: flex; flex-wrap: wrap; gap: .5rem 1rem; align-items: center; margin: .6rem 0; font-size: .88rem; }
  .filters select, .filters input { font: inherit; padding: .25rem .45rem; border: 1px solid var(--line);
                                     border-radius: 6px; background: var(--card); color: var(--ink); }
  button.more { font: inherit; font-size: .8rem; color: var(--accent); background: none; cursor: pointer;
                border: 1px solid var(--line); border-radius: 6px; padding: .1rem .5rem; }
  #runs tr.detail td { background: var(--paper); }
  #runs tr.detail .outcome { margin: .2rem 0 .5rem; color: var(--ink); font-weight: 600; font-size: .88rem; }
  .rowdetail { display: grid; gap: .3rem 1.4rem; grid-template-columns: repeat(auto-fill, minmax(min(100%, 320px), 1fr));
               font-size: .84rem; }
  .rowdetail div { display: grid; grid-template-columns: 9.5rem 1fr; gap: .5rem; }
  .rowdetail b { color: var(--ink); font-weight: 600; }
  #runs td { vertical-align: top; }
  @media (max-width: 1100px) {
    .kpis-main, .kpis-main.three { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .secondary, .media { grid-template-columns: minmax(0, 1fr); }
    .aside { grid-template-columns: 1fr 1fr; }
  }
  @media (max-width: 760px) {
    .cmp, .aside, .media.row { grid-template-columns: minmax(0, 1fr); }
    .attrow { grid-template-columns: 1fr; } .events { columns: 1; }
    .factlist > div, .rowdetail div { grid-template-columns: 8rem 1fr; }
    .exp > header .meta { margin-left: 0; }
    .tl-bar .seg span { display: none; }
  }
  @media (max-width: 480px) { .kpis-main { grid-template-columns: 1fr 1fr; } .kpis-main.three { grid-template-columns: 1fr; } .kpi .v { font-size: 2rem; } .kpi .v.demo { font-size: 1.4rem; } }
"""

RESULTS_JS = """<script>
  // Enlarging a cropped capture shows the whole capture it was cut from.
  window.addEventListener('load', () => {
    document.querySelectorAll('figure img[data-full]').forEach(img => img.addEventListener('click', () => {
      document.querySelector('#viewer img').src = img.dataset.full;
    }));
  });
  document.querySelectorAll('.labels-toggle').forEach(b => b.addEventListener('click', () => {
    const fig = b.closest('figure');
    const off = fig.classList.toggle('labels-off');
    b.setAttribute('aria-pressed', String(!off));
    b.textContent = off ? 'Labels off' : 'Labels on';
  }));
</script>"""


def main():
    names = sys.argv[1:] or [l.strip() for l in open(RUN_LIST) if l.strip() and not l.startswith('#')]
    runs = [load(n) for n in names]
    final = [r for r in runs if r['final']]
    import yaml
    scenario_count = len(yaml.safe_load(open(SCENARIO_FILE))['scenarios'])
    by_key = {spec['key']: next(r for r in runs if os.path.normpath(r['path']) ==
                                os.path.normpath(os.path.join(ROOT, spec['run'])))
              for spec in FEATURED + ARCHIVED_FEATURED}
    explain = explanations(by_key)
    ward = [r for r in runs if r['m']['commit'] in WARD_TRIAL]
    post_merge = [r for r in runs if r['m']['commit'] in POST_MERGE]
    record = '<a href="../../archive/engineering/2026-11-18-parking-hospital-validation.md">dated record</a>'
    featured_html = ''.join(featured(by_key[spec['key']], spec, *explain[spec['key']]) for spec in FEATURED)
    archived_html = ''.join(featured(by_key[spec['key']], spec, *explain[spec['key']]) for spec in ARCHIVED_FEATURED)
    since = by_key['isaac_side']['start'] - 6 * 86400        # the G1 position check, 9 November

    def card(title, shows, body, note=''):
        return (f'<div class="chartcard"><h3>{title}</h3><p class="q"><b>What it shows:</b> {shows}</p>{body}'
                f'{f"<p class=note>{note}</p>" if note else ""}</div>')
    charted = post_merge + ward + final
    gz_errors = [e for r in post_merge if r['m']['simulator'] == 'gz' for e in r['m'].get('localization_errors_m') or []]
    charts = (
        '<div class="charts">'
        + card('Mission duration by phase',
               'how long each post-merge warehouse run, hospital-ward trial run and hospital-ward validation run '
               'spent searching for G1, localizing and planning, parking, waiting and returning. Searching '
               'dominates every run; hover a segment for its duration.',
               duration_chart(charted),
               'Times from the mission\'s logged state changes (mission log), not from ground truth.')
        + card('G1 localization error',
               'how far each G1 position the mission planned with was from G1\'s true position, in the '
               'post-merge Gazebo warehouse runs, the hospital-ward trial and the hospital-ward validation '
               '(one dot per localization; an arrow marks a value off the scale).',
               localization_chart(charted),
               f'Post-merge warehouse runs: {len(gz_errors)} localizations, {min(gz_errors):.3f}-{max(gz_errors):.3f} m. '
               f'Hospital ward: ' + '; '.join(f'{r["name"]} ({r["m"]["commit"]}) {span(r["m"]["localization_errors_m"])} m'
                                              for r in ward + final if r['m']['simulator'] == 'gz'
                                              and r['m'].get('localization_errors_m'))
               + '. Gazebo ground truth only; Isaac records no ground truth, so it has no points here.')
        + card('Parking attempts in completed missions',
               'every completed mission since the G1 position check (9 November): each attempt in order, '
               'whether the live LiDAR check rejected it and why. A mission makes at most three attempts.',
               attempts_chart([r for r in runs if r['start'] >= since]))
        + card('Parked reach against the requirement',
               'the distance from G1\'s axis to the basket centre at every completed Gazebo mission, all '
               'campaigns, against the required 0.30-0.60 m band.',
               reach_chart(runs),
               'Gazebo ground truth only; Isaac records no ground truth and is not shown. The historical false '
               'completion parked 1.72 m away, off the scale; a red value lies outside the required band.')
        + '</div>')
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
            f'<section id="featured"><h2>Featured experiments</h2><p class="lead">Side parking in Isaac Sim (A), '
            f'parking in Gazebo Sim (B) and the Gazebo safety test (C), all in the hospital ward on the final '
            f'mission code ({FINAL_CODE[0]}). Each shows captures from that run only, cropped to the robot and G1. '
            f'Labels are drawn over the images and can be switched off; click an image to see the whole capture. '
            f'Earlier runs, including every warehouse run, are in the <a href="#history">records</a> with their '
            f'measurements; see the {record}.</p>'
            f'{featured_html}</section>'
            f'<section id="performance"><h2>Performance</h2>{charts}</section>'
            f'{history(runs, archived_html)}</main>{RESULTS_JS}')
    with open(OUT, 'w') as stream:
        stream.write(pages.page('Autonomous Parking — Experimental Results', pages.RESULTS_PAGE, intro, body,
                                extra_css=RESULTS_CSS))
    # The folder holds this page's exports only: drop any the page no longer shows.
    used = set(re.findall(r'assets/parking_results/([^"]+)', open(OUT).read()))
    for name in sorted(set(os.listdir(ASSETS)) - used):
        os.remove(os.path.join(ASSETS, name))
    print('wrote', OUT, len(runs), 'runs,', len(final), 'hospital-ward validation runs,', len(used), 'images')


if __name__ == '__main__':
    main()
