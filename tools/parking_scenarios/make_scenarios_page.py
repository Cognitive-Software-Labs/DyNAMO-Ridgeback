"""Build docs/exploration/autonomous_parking_scenarios.html: what each scene looks like.

Usage: make_scenarios_page.py SCENES_DIR
SCENES_DIR holds preview_scenario.sh captures (manifest.json plus PNGs). Only images
whose manifest entry passed its check are used; a view that is missing or failed is
shown as missing, never replaced by another simulator's image. Layouts come from
src/ridgeback_autonomy/config/parking_scenarios.yaml.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pages  # noqa: E402
import scene_views  # noqa: E402
from scene_views import parking_scenarios  # noqa: E402

OUT = os.path.join(pages.DOCS, pages.SCENARIOS_PAGE)
ASSETS = os.path.join(pages.DOCS, 'assets/parking_scenes')
ORDER = ['standard', 'right_blocked', 'left_blocked', 'both_sides_blocked', 'restricted',
         'no_feasible', 'heading_90', 'heading_away']
TITLES = {'standard': 'Standard: G1 at its worktable', 'right_blocked': 'Right side blocked',
          'left_blocked': 'Left side blocked', 'both_sides_blocked': 'Both sides blocked',
          'restricted': 'Restricted space', 'no_feasible': 'No feasible parking position',
          'heading_90': 'G1 turned 90 deg', 'heading_away': 'G1 facing away from its table'}
VIEWS = [('overview', 'Top-down, whole task'), ('workstation', 'Top-down, workstation'),
         ('closeup', 'G1 and table'), ('oblique', 'Oblique (heights)'),
         ('robot_camera', 'Robot camera at start')]
CAPTION = {
    'overview': ('Top-down from {h:.0f} m: the robot at its start on the left (facing up), '
                 'G1 at its worktable on the right'),
    'workstation': 'Top-down from 5.5 m over G1 (worktable at the top): the floor available for parking',
    'closeup': 'G1 and its worktable from behind G1\'s left shoulder',
    'oblique': 'Oblique view: tabletop, panel and bed heights against the robot\'s mast and basket',
    'robot_camera': 'The robot\'s own camera at its start: G1 is not in view',
}
SIM = {'gz': 'Gazebo (warehouse)', 'isaac': 'Isaac Sim (hospital floor)'}
ASSET_NAMES = {'table': 'table 1.20 x 0.70 m, top 0.75 m', 'privacy_curtain': 'panel 1.70 m long, 1.60 m high',
               'hospital_bed': 'bed 1.80 x 0.90 m, 0.50 m high'}


def pose(p):
    return f'({p["x"]:.2f}, {p["y"]:.2f}) m, yaw {p["yaw_deg"]:.0f} deg'


def load_captures(scenes_dir):
    manifest = json.load(open(os.path.join(scenes_dir, 'manifest.json')))
    best = {}
    for entry in manifest:     # the latest passing capture of each view wins
        if entry.get('ok') and entry.get('stage') == 'scene':
            best[(entry['backend'], entry['scenario'], entry['view'])] = entry
    commits = {}
    for backend in ('gz', 'isaac'):
        for name in ORDER:
            path = os.path.join(scenes_dir, f'{backend}_{name}_commit.txt')
            if os.path.exists(path):
                lines = open(path).read().split('\n')
                commits[(backend, name)] = (lines[0][:7], any(
                    l.strip() and not l.startswith('??') and 'start_parking_demo.sh' not in l
                    for l in lines[1:]))
    return best, commits


def layout_facts(data, name, backend):
    if backend not in parking_scenarios.backends(data['scenarios'][name]):
        return None
    layout = parking_scenarios.world_layout(data, name, backend)
    start = data.get('robot_start', {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})
    g1 = layout['g1']
    dist = math.hypot(g1['x'] - start['x'], g1['y'] - start['y'])
    objects = [f'{o["name"]}: {ASSET_NAMES.get(o["asset"], o["asset"])} at {pose(o)}'
               for o in layout['objects']]
    return {'G1': f'{pose(g1)}; {dist:.1f} m from the start', 'Robot start': pose(start),
            'Worktable': next((o for o in objects if o.startswith('worktable')), '-'),
            'Other objects': '; '.join(o for o in objects if not o.startswith('worktable')) or 'none'}


def main():
    scenes_dir = sys.argv[1]
    data = parking_scenarios.load(scene_views.SCENARIO_FILE)
    best, commits = load_captures(scenes_dir)
    stations = data['workstations']
    rows, cards = [], []
    for name in ORDER:
        scene = data['scenarios'][name]
        supported = parking_scenarios.backends(scene)
        rows.append(f'<tr><td><a href="#{name}">{pages.esc(TITLES[name])}</a></td>'
                    f'<td>{pages.esc(scene["purpose"])}</td><td>{pages.esc(scene["expected"])}</td>'
                    f'<td>{", ".join("Gazebo" if b == "gz" else "Isaac" for b in supported)}</td></tr>')
        facts = [('Description', scene['description']), ('Purpose', scene['purpose']),
                 ('Expected behaviour', scene['expected']),
                 ('G1 relative to its table', {0: 'facing it', 90: 'turned 90 deg to its left',
                                               180: 'its back to the table'}.get(
                                                   int(scene.get('g1_turn_deg', 0)), '-'))]
        grid = ['<div></div>'] + [f'<div class="colhead">{SIM[b]}</div>' for b in ('gz', 'isaac')]
        per_sim = {}
        for backend in ('gz', 'isaac'):
            per_sim[backend] = layout_facts(data, name, backend)
        for key in ('G1', 'Worktable', 'Other objects', 'Robot start'):
            grid.append(f'<div class="rowlabel">{key}</div>')
            for backend in ('gz', 'isaac'):
                value = per_sim[backend][key] if per_sim[backend] else 'not supported in this world'
                grid.append(f'<div style="font-size:.84rem">{pages.esc(value)}</div>')
        for view, label in VIEWS:
            grid.append(f'<div class="rowlabel">{label}</div>')
            for backend in ('gz', 'isaac'):
                entry = best.get((backend, name, view))
                if per_sim[backend] is None:
                    grid.append(pages.missing(f'{TITLES[name]} is not set up in this world'))
                elif entry is None:
                    grid.append(pages.missing('Not captured'))
                else:
                    src = os.path.join(scenes_dir, entry['file'])
                    rel = pages.export_jpeg(src, os.path.join(
                        ASSETS, entry['file'].replace('.png', '.jpg')))
                    commit, dirty = commits.get((backend, name), ('unknown', False))
                    caption = CAPTION[view].format(h=scene_views.OVERVIEW_HEIGHT_M[backend])
                    grid.append(pages.figure(rel, f'{SIM[backend].split(" ")[0]}: {caption} '
                                                  f'(captured at {commit}{" + working tree" if dirty else ""})'))
        fact_html = ''.join(f'<div><b>{pages.esc(k)}:</b> {pages.esc(v)}</div>' for k, v in facts)
        cards.append(f'<section class="card" id="{name}"><header><h3>{pages.esc(TITLES[name])}</h3>'
                     f'<code>demo_scenario:={name}</code></header><div class="facts">{fact_html}</div>'
                     f'<div class="compare">{"".join(grid)}</div></section>')
    jump = ''.join(f'<a href="#{n}">{pages.esc(TITLES[n])}</a>' for n in ORDER)
    intro = (
        '<p>Each scene places G1 at a <strong>workstation</strong>: standing 0.75 m from a worktable and '
        'facing it, as in a stationary pick-and-place task. The robot starts at its usual spawn at the '
        'world origin, facing +x, several metres away; walls or shelving hide G1 from its camera. The '
        'mission has to explore, detect G1, localize it, plan a parking pose beside it, park, wait '
        'five seconds and drive back. The scenarios only change what stands around G1 and which way G1 '
        'faces; the mission, planner and safety checks are the same, and nothing about the scene is '
        'given to the mission.</p>'
        '<p class="note">Every image on this page is a capture of the running simulator with the '
        'robot at its start: Gazebo images come from temporary camera sensors rendered by the Gazebo '
        'server, Isaac images from opt-in view cameras in the Isaac runner, and the robot-camera row '
        'from the robot\'s own camera topic. Results of running the scenarios are on the '
        f'<a href="{pages.RESULTS_PAGE}">results page</a>.</p>')
    worlds = (
        '<h2>The two worlds</h2><div class="wrap"><table><tr><th></th><th>Gazebo</th><th>Isaac Sim</th></tr>'
        f'<tr><th>World</th><td>Clearpath warehouse (<code>depot</code>): shelving, pillars and three '
        'people, one standing beside the start</td><td>Hospital floor (<code>parking_demo</code>): a '
        'corridor with rooms on both sides</td></tr>'
        f'<tr><th>G1 workstation</th><td>{pose(stations["gz"])}: the aisle south of the long shelf '
        'beside the start; the robot reaches it round the shelf\'s east end and arrives behind G1</td>'
        f'<td>{pose(stations["isaac"])}: the south room, entered through the doorway at x 2.7-5.3 m; the '
        'robot arrives behind G1\'s left shoulder</td></tr>'
        '<tr><th>Robot start</th><td colspan="2">(0, 0) m, yaw 0 deg: the default spawn of both '
        'simulators, so the SLAM map frame is the world frame</td></tr>'
        '<tr><th>Frames</th><td colspan="2"><code>forward</code> points from G1 to its table and '
        '<code>left</code> to G1\'s left when it faces the table; objects are placed in that frame, so '
        'a scenario has the same layout around G1 in both worlds</td></tr></table></div>')
    body = (f'<nav class="jump"><div class="shell">{jump}</div></nav><main class="shell">'
            f'{worlds}<h2>Scenarios</h2><div class="wrap"><table><tr><th>Scenario</th><th>Purpose</th>'
            f'<th>Expected parking behaviour</th><th>Simulators</th></tr>{"".join(rows)}</table></div>'
            f'{"".join(cards)}<p class="note">Scene definitions: '
            f'<code>src/ridgeback_autonomy/config/parking_scenarios.yaml</code>; reference: '
            f'<a href="parking_scenarios.md">parking scenarios</a>.</p></main>')
    with open(OUT, 'w') as stream:
        stream.write(pages.page('Autonomous parking scenarios', pages.SCENARIOS_PAGE, intro, body))
    print('wrote', OUT)


if __name__ == '__main__':
    main()
