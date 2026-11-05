"""Build docs/exploration/autonomous_parking_gallery.html from recorded scenario runs.

Usage: make_gallery.py RUN_DIR [RUN_DIR ...]
Each RUN_DIR holds a run_scenario.sh result under the repository. Screenshots are
cropped and saved as small JPEGs beside the page; metrics come from scenario_metrics.py.
Runs are grouped by the code they ran (CODE below); for a scenario the newest code
comes first and older cards stay as historical evidence.
"""
import glob, html, json, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '../..'))
sys.path.insert(0, HERE)
from scenario_metrics import run as metrics, SCEN  # noqa: E402

OUT = os.path.join(ROOT, 'docs/exploration/autonomous_parking_gallery.html')
ASSETS = os.path.join(ROOT, 'docs/exploration/assets/parking_gallery')
ORDER = ['standard', 'worktable', 'right_blocked', 'left_blocked', 'both_sides_blocked',
         'restricted', 'no_feasible', 'heading_90', 'heading_away']
TITLES = {'standard': 'Standard scene', 'worktable': 'A - G1 at a worktable',
          'right_blocked': 'B - right side blocked', 'left_blocked': 'C - left side blocked',
          'both_sides_blocked': 'D - both sides blocked', 'restricted': 'E - restricted space',
          'no_feasible': 'F - no feasible position', 'heading_90': 'G - G1 heading 90 deg',
          'heading_away': 'G - G1 facing away'}
# Recorded HEAD -> (campaign, code that ran: HEAD plus the then-uncommitted changes).
# A run folder's code.txt overrides the code text when the working tree differed.
CODE = {'fe60a88': ('P7 scenarios, 2026-11-05',
                    'fe60a88 plus the changes committed as 28769c2'),
        '622a0c0': ('Validation gaps, 2026-11-05',
                    '622a0c0 plus the detection and fallback changes committed as 2ecaecf')}
CAMPAIGNS = [c[0] for c in CODE.values()]


def largest(paths):
    def area(p):
        w, h = subprocess.run(['identify', '-format', '%w %h', p], capture_output=True,
                              text=True).stdout.split()
        return int(w) * int(h)
    return max(paths, key=area) if paths else None


def shot(run, stages, app):
    for stage in stages:
        found = largest(glob.glob(f'{run}/shots/{stage}_{app}_*.png'))
        if found:
            return found, stage
    return None, None


def save(src, name, crop=None):
    os.makedirs(ASSETS, exist_ok=True)
    dst = os.path.join(ASSETS, name)
    cmd = ['convert', src] + (['-crop', crop, '+repage'] if crop else []) + [
        '-resize', '960x>', '-strip', '-quality', '72', dst]
    subprocess.run(cmd, check=True)
    return os.path.relpath(dst, os.path.dirname(OUT))


def fmt(v, unit=''):
    return '-' if v is None or v == [] else f'{v}{unit}'


def outcome_badge(m):
    """Parked, refused to park safely, a perception failure, or another stop."""
    reason = m.get('final_reason') or ''
    back = ', returned' if m.get('returned_to_start') else ', stopped'
    if m['outcome'] == 'MISSION_COMPLETE':
        return 'ok', 'parked, returned'
    if 'no_valid_candidate' in reason or 'no feasible' in reason:
        return 'safe', 'refused to park' + back
    if 'localization' in reason or 'not relocalized' in reason:
        return 'bad', 'perception: G1 not localized' + back
    return 'warn', 'stopped by a check' + back


def tag_of(m):
    return f"{m['scenario']}_{m['simulator']}_{m['commit']}_{m['run']}"


def card(m, run, latest):
    name = m['scenario']
    tag = tag_of(m)
    imgs = []
    # RViz map pane (donut rays, footprint, route, legend) when the pose was selected and at the end.
    for stages, label in ((['NAVIGATING_TO_PREPARKING', 'G1_LOCALIZED'], 'selection'),
                          (['PARKED', 'END_MISSION_COMPLETE', 'END_MISSION_FAILED', 'END_timeout',
                            'END_PARKING_PLANNING_FAILED'], 'final')):
        src, stage = shot(run, stages, 'rviz2')
        if src:
            imgs.append((save(src, f'{tag}_rviz_{label}.jpg', '1425x595+482+60'),
                         f'RViz, {stage.replace("END_", "end: ").lower()}'))
    app, sim = ('ruby', 'Gazebo') if m['simulator'] == 'gz' else ('python', 'Isaac Sim')
    src, stage = shot(run, ['PARKED', 'END_MISSION_COMPLETE', 'END_MISSION_FAILED', 'END_timeout',
                            'NAVIGATING_TO_PREPARKING'], app)
    if src:
        crop = None if app == 'ruby' else '957x584+48+24'   # the Isaac viewport pane
        imgs.append((save(src, f'{tag}_sim.jpg', crop), f'{sim}, {stage.replace("END_", "end: ").lower()}'))
    badge = outcome_badge(m)
    campaign, code = CODE.get(m['commit'], ('-', m['commit']))
    if os.path.exists(f'{run}/code.txt'):   # a run on an intermediate working tree says so
        code = open(f'{run}/code.txt').readline().strip()
    rows = [('Simulator', 'Gazebo' if m['simulator'] == 'gz' else 'Isaac Sim'),
            ('Campaign', campaign), ('Code', code), ('Run', m['run']),
            ('G1 (world)', m['g1_pose']), ('Objects', m['objects']),
            ('Intended', m['expected']), ('Outcome', m['outcome']),
            ('Sides tried (estimated)', ', '.join(m.get('selected_sides') or []) or '-'),
            ('Attempts', m.get('attempts')),
            ('Failed attempts', '; '.join(m.get('attempt_failures') or []) or '-'),
            ('Exploration / mission', f"{fmt(m.get('exploration_s'), ' s')} / {fmt(m.get('mission_s'), ' s')}"),
            ('Reach, ground truth', fmt(m.get('reach_truth_m'), ' m')),
            ('Bearing from G1 heading, ground truth', fmt(m.get('bearing_from_g1_heading_deg'), ' deg')),
            ('Pose error to goal', fmt(m.get('pose_error_m'), ' m')),
            ('Min clearance, ground truth', f"{fmt(m.get('min_truth_clearance_m'), ' m')} ({m.get('min_truth_clearance_pair', '-')})"),
            ('Min clearance to objects, ground truth', f"{fmt(m.get('min_object_clearance_m'), ' m')} ({m.get('min_object_clearance_pair', '-')})"),
            ('Min LiDAR return, final leg', fmt(m.get('lidar_min_m'), ' m')),
            ('Basket access plausible', fmt(m.get('basket_access_plausible'))),
            ('Returned to start', fmt(m.get('returned_to_start'))),
            ('Final reason', m.get('final_reason') or '-')]
    table = ''.join(f'<tr><th>{html.escape(k)}</th><td>{html.escape(str(v))}</td></tr>' for k, v in rows)
    figs = ''.join(f'<figure><img loading="lazy" src="{html.escape(p)}" alt="{html.escape(c)}">'
                   f'<figcaption>{html.escape(c)}</figcaption></figure>' for p, c in imgs)
    link = '../../' + os.path.relpath(os.path.abspath(run), ROOT)
    old = '' if latest else '<span class="badge old">historical</span>'
    return (f'<section class="card{"" if latest else " historical"}" id="{tag}"><header>'
            f'<h3>{html.escape(TITLES.get(name, name))}</h3>'
            f'<span class="badge {badge[0]}">{badge[1]}</span>{old}<code>{html.escape(name)}</code>'
            f'<span class="sim">{"Gazebo" if m["simulator"] == "gz" else "Isaac"} - '
            f'{html.escape(campaign)}</span></header>'
            f'<p class="desc">{html.escape(m["description"])}</p>'
            f'<div class="shots">{figs}</div><table>{table}</table>'
            f'<p class="links">Evidence: <a href="{html.escape(link)}/">run folder</a> '
            f'(<a href="{html.escape(link)}/console.log">console</a>, '
            f'<a href="{html.escape(link)}/metrics.json">metrics</a>)</p></section>')


def main():
    runs = sys.argv[1:]
    data = []
    for run in runs:
        m = metrics(run)
        with open(f'{run}/metrics.json', 'w') as f:
            json.dump(m, f, indent=1)
        data.append((m, run))
    def campaign(m):
        return CAMPAIGNS.index(CODE[m['commit']][0]) if m['commit'] in CODE else -1

    # Per scenario and simulator: newest code first, then in the order the runs ended.
    data.sort(key=lambda d: (ORDER.index(d[0]['scenario']) if d[0]['scenario'] in ORDER else 99,
                             d[0]['simulator'], -campaign(d[0]),
                             os.path.getmtime(f'{d[1]}/result.txt')))
    newest = {}
    for m, _ in data:
        key = (m['scenario'], m['simulator'])
        newest[key] = max(newest.get(key, -1), campaign(m))
    rows = []
    for m, _ in data:
        latest = campaign(m) == newest[(m['scenario'], m['simulator'])]
        kind, label = outcome_badge(m)
        rows.append(
            f"<tr{'' if latest else ' class=\"historical\"'}><td><a href=\"#{tag_of(m)}\">"
            f"{html.escape(TITLES.get(m['scenario'], m['scenario']))}</a></td>"
            f"<td>{'Gazebo' if m['simulator'] == 'gz' else 'Isaac'}</td>"
            f"<td>{html.escape(CODE.get(m['commit'], (m['commit'],))[0])}</td>"
            f"<td><span class=\"badge {kind}\">{html.escape(label)}</span></td>"
            f"<td>{html.escape(', '.join(m.get('selected_sides') or []) or '-')}</td><td>{fmt(m.get('attempts'))}</td>"
            f"<td>{fmt(m.get('reach_truth_m'), ' m')}</td><td>{fmt(m.get('min_truth_clearance_m'), ' m')}</td>"
            f"<td>{fmt(m.get('mission_s'), ' s')}</td></tr>")
    cards = ''.join(card(m, run, campaign(m) == newest[(m['scenario'], m['simulator'])])
                    for m, run in data)
    page = open(os.path.join(HERE, 'gallery_template.html')).read()
    page = page.replace('{{SUMMARY}}', ''.join(rows)).replace('{{CARDS}}', cards)
    with open(OUT, 'w') as f:
        f.write(page)
    print('wrote', OUT, len(data), 'runs')


if __name__ == '__main__':
    main()
