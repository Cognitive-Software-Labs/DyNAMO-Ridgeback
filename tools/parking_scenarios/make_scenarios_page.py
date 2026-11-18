"""Build docs/exploration/autonomous_parking_scenarios.html: what each scene looks like.

Usage: make_scenarios_page.py SCENES_DIR [SCENES_DIR ...]
Each SCENES_DIR holds preview_scenario.sh captures (manifest.json plus PNGs); a later
directory's passing capture of a view replaces an earlier one. Only images whose
manifest entry passed its check are used, and only captures of the hospital ward
(PAGE_WORLD) whose scene still has the current layout: a capture records its world in
the manifest (older Isaac captures, before worlds were recorded, had only the ward) and
its commit beside it, and the scenario file at that commit must place G1 and every
object where the current file does. Isaac Sim captures lead each scenario in three slots
(top-down, close-up of G1 at its table, angled view), each filled by the first captured
view in its preference list; Gazebo captures of the same scene follow in a collapsed
section. A missing view is shown as missing, never replaced by another simulator's or
another layout's image. Layouts come from src/ridgeback_autonomy/config/parking_scenarios.yaml.
"""
import json
import math
import os
import re
import subprocess
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
# (slot label, views in order of preference) for the leading Isaac Sim images, and the
# Gazebo views shown in the collapsed section.
PRIMARY = [('Top-down', ('task', 'workstation')), ('G1 at its table', ('closeup',)),
           ('Angled view', ('isometric',))]
SECONDARY = (('Top-down', ('task', 'workstation')), ('G1 at its table', ('closeup',)),
             ('Angled view', ('isometric', 'angled')))
CAPTION = {
    'overview': ('Top-down from {h:.0f} m: the robot at its start on the left, '
                 'G1 at its worktable on the right'),
    'workstation': 'Top-down from 5.5 m over G1 (worktable at the top): the floor available for parking',
    'closeup': 'G1 and its worktable from behind G1\'s left shoulder',
    'oblique': 'Oblique view: tabletop, panel and bed heights against the robot\'s mast and basket',
    'robot_camera': 'The robot\'s own camera at its start: G1 is not in view',
    'task': 'Top-down: the robot at its start and G1 at its workstation, framed together',
    'isometric': 'Angled view from above: the robot with its mast and basket at its start, and G1 at its table',
    'angled': 'Angled view of G1 at its worktable and the objects around it',
}
SIM = {'gz': 'Gazebo (hospital ward)', 'isaac': 'Isaac Sim (hospital ward)'}
# Recorded HEAD -> the code that ran (previews then carried no other code change).
CODE = {'bb8aa94': 'bb8aa94 plus the changes committed as b4f855b', 'b4f855b': 'b4f855b',
        '16bf79c': '16bf79c plus the view framing committed with these images'}
ASSET_NAMES = {'table': 'table 1.20 x 0.70 m, top 0.75 m', 'privacy_curtain': 'panel 1.70 m long, 1.60 m high',
               'hospital_bed': 'bed 1.80 x 0.90 m, 0.50 m high'}


def pose(p):
    return f'({p["x"]:.2f}, {p["y"]:.2f}) m, yaw {p["yaw_deg"]:.0f} deg'


def scenario_file_at(commit, cache={}):
    """The scenario file as ``commit`` recorded it, or None when it cannot be read."""
    if commit not in cache:
        try:
            import yaml
            cache[commit] = yaml.safe_load(subprocess.run(
                ['git', 'show', f'{commit}:src/ridgeback_autonomy/config/parking_scenarios.yaml'],
                cwd=pages.ROOT, capture_output=True, text=True, check=True).stdout)
        except (subprocess.CalledProcessError, OSError):
            cache[commit] = None
    return cache[commit]


def placement(data, backend, name):
    """G1 and the objects of scenario ``name`` in the page world, or None."""
    try:
        layout = parking_scenarios.world_layout(data, name, backend, PAGE_WORLD[backend])
    except (ValueError, KeyError, TypeError):
        return None
    return (layout['g1'], sorted((o['name'], o['asset'], o['x'], o['y'], o['yaw_deg']) for o in layout['objects']))


def load_captures(scenes_dirs, data):
    """{(backend, scenario, view): (entry, dir, (commit, dirty))}: the latest passing capture
    of the page world whose scene still has the current layout."""
    best = {}
    for scenes_dir in scenes_dirs:
        manifest = json.load(open(os.path.join(scenes_dir, 'manifest.json')))
        for entry in manifest:
            if not (entry.get('ok') and entry.get('stage') == 'scene'):
                continue
            backend, name = entry['backend'], entry['scenario']
            world = entry.get('world') or ('parking_demo' if backend == 'isaac' else 'depot')
            if world != PAGE_WORLD.get(backend):
                continue
            path = os.path.join(scenes_dir, f'{backend}_{name}_commit.txt')
            commit = ('unknown', False)
            if os.path.exists(path):
                lines = open(path).read().split('\n')
                commit = (lines[0][:7], any(
                    l.strip() and not l.startswith('??') and 'start_parking_demo.sh' not in l
                    for l in lines[1:]))
            if commit[0] == 'unknown' or (placement(scenario_file_at(commit[0]), backend, name)
                                          != placement(data, backend, name)):
                print(f'{scenes_dir}: {backend} {name} {entry["view"]} skipped: not the current layout')
                continue
            best[(backend, name, entry['view'])] = (entry, scenes_dir, commit)
    return best


def image(best, backend, name, view, scene):
    entry, scenes_dir, (commit, dirty) = best[(backend, name, view)]
    rel = pages.export_jpeg(os.path.join(scenes_dir, entry['file']), os.path.join(
        ASSETS, entry['file'].replace('.png', '.jpg')), 1600)
    code = CODE.get(commit, commit + (' + working tree' if dirty else ''))
    caption = CAPTION[view].format(h=scene_views.OVERVIEW_HEIGHT_M[PAGE_WORLD[backend]])
    return pages.figure(rel, f'{SIM[backend].split(" ")[0]}: {caption} (captured with {code})')


# The world this page's captures show: the hospital ward, in both simulators. Gazebo's
# warehouse (depot) captures stay in their evidence folders.
PAGE_WORLD = {'gz': 'parking_demo', 'isaac': 'parking_demo'}


def layout_facts(data, name, backend):
    if backend not in parking_scenarios.backends(data['scenarios'][name]):
        return None
    layout = parking_scenarios.world_layout(data, name, backend, PAGE_WORLD[backend])
    start = data.get('robot_start', {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})
    g1 = layout['g1']
    dist = math.hypot(g1['x'] - start['x'], g1['y'] - start['y'])
    objects = [f'{o["name"]}: {ASSET_NAMES.get(o["asset"], o["asset"])} at {pose(o)}'
               for o in layout['objects']]
    return {'G1': f'{pose(g1)}; {dist:.1f} m from the start', 'Robot start': pose(start),
            'Worktable': next((o for o in objects if o.startswith('worktable')), '-'),
            'Other objects': '; '.join(o for o in objects if not o.startswith('worktable')) or 'none'}


def main():
    data = parking_scenarios.load(scene_views.SCENARIO_FILE)
    best = load_captures(sys.argv[1:], data)
    stations = {b: parking_scenarios.workstation(data, b, w) for b, w in PAGE_WORLD.items()}
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
        per_sim = {backend: layout_facts(data, name, backend) for backend in ('gz', 'isaac')}
        lead = 'isaac' if per_sim['isaac'] else 'gz'
        slots = []
        for label, preferred in PRIMARY:
            view = next((v for v in preferred if (lead, name, v) in best), None)
            slots.append(f'<div><div class="rowlabel">{pages.esc(label)}</div>'
                         + (image(best, lead, name, view, scene) if view
                            else pages.missing(f'{label}: not captured in {SIM[lead]}')) + '</div>')
        shown = [b for b in ('isaac', 'gz') if per_sim[b]]
        if len(shown) == 2 and per_sim['isaac'] == per_sim['gz']:
            shown, label = ['isaac'], {'isaac': 'Both simulators'}
        else:
            label = SIM
        positions = ''.join(
            f'<div><b>{label[b]}:</b> G1 {pages.esc(per_sim[b]["G1"])}; {pages.esc(per_sim[b]["Worktable"])}; '
            f'other objects: {pages.esc(per_sim[b]["Other objects"])}</div>'
            for b in shown)
        other = 'gz' if lead == 'isaac' else 'isaac'
        secondary = ''
        if per_sim[other]:
            shots = []
            for label, preferred in SECONDARY:
                view = next((v for v in preferred if (other, name, v) in best), None)
                shots.append(f'<div><div class="rowlabel">{pages.esc(label)}</div>'
                             + (image(best, other, name, view, scene) if view
                                else pages.missing(f'{label}: not captured in {SIM[other]}')) + '</div>')
            secondary = (f'<details class="secondary"><summary>{SIM[other]}: the same scene in the same '
                         f'ward</summary><div class="views">{"".join(shots)}</div></details>')
        note = ('<p class="note">Both simulators load the same hospital ward and place the same objects '
                'around G1. The leading images are Isaac Sim\'s; Gazebo\'s views of the same scene are '
                'below them.</p>' if lead == 'isaac' else
                '<p class="note">This scenario exists only in Gazebo.</p>')
        grid = [f'<div class="facts">{positions}</div>{note}<div class="views">{"".join(slots)}</div>{secondary}']
        fact_html = ''.join(f'<div><b>{pages.esc(k)}:</b> {pages.esc(v)}</div>' for k, v in facts)
        cards.append(f'<section class="card" id="{name}"><header><h3>{pages.esc(TITLES[name])}</h3>'
                     f'<code>demo_scenario:={name}</code></header><div class="facts">{fact_html}</div>'
                     f'{"".join(grid)}</section>')
    jump = ''.join(f'<a href="#{n}">{pages.esc(TITLES[n])}</a>' for n in ORDER)
    intro = (
        '<p>Each scene places G1 at a <strong>workstation</strong>: standing 0.75 m from a worktable and '
        'facing it, as in a stationary pick-and-place task. The robot starts at its usual spawn at the '
        'world origin, facing +x, several metres away; the corridor wall hides G1 from its camera. The '
        'mission has to explore, detect G1, localize it, plan a parking pose beside it, park, wait '
        'five seconds and drive back. The scenarios only change what stands around G1 and which way G1 '
        'faces; the mission, planner and safety checks are the same, and nothing about the scene is '
        'given to the mission.</p>'
        '<p class="note">Every image on this page is a capture of the running simulator in the hospital '
        'ward with the robot at its start: Isaac images from opt-in view cameras in the Isaac runner, '
        'Gazebo images from temporary camera sensors rendered by the Gazebo server. Each scenario leads '
        'with Isaac Sim; the Gazebo views of the same scene are under each scenario. Click an image to '
        'enlarge it. Results of running the scenarios are on the '
        f'<a href="{pages.RESULTS_PAGE}">results page</a>.</p>')
    worlds = (
        '<h2>The hospital ward</h2><div class="wrap"><table><tr><th></th><th>Gazebo</th><th>Isaac Sim</th></tr>'
        '<tr><th>World</th><td colspan="2">The hospital ward (<code>parking_demo</code>): a corridor with '
        'rooms on both sides. Gazebo loads <code>parking_demo.sdf</code> and Isaac a USD layer over the same '
        'ward; walls, doorways, beds, tables and chairs have the same collision boxes in both</td></tr>'
        f'<tr><th>G1 workstation</th><td colspan="2">{pose(stations["isaac"])}: the south room, entered '
        'through the doorway at x 2.7-5.3 m; the robot arrives behind G1\'s left shoulder. The room\'s own '
        'exam table and chair stand to G1\'s right</td></tr>'
        '<tr><th>Robot start</th><td colspan="2">(0, 0) m, yaw 0 deg: the default spawn of both '
        'simulators, so the SLAM map frame is the world frame; the corridor wall hides G1 from it</td></tr>'
        '<tr><th>Frames</th><td colspan="2"><code>forward</code> points from G1 to its table and '
        '<code>left</code> to G1\'s left when it faces the table; objects are placed in that frame</td></tr>'
        '<tr><th>Warehouse</th><td colspan="2">Gazebo\'s earlier world, Clearpath\'s warehouse '
        '(<code>world:=depot</code>), stays available to reproduce the runs recorded there; this page '
        'shows the ward only</td></tr></table></div>')
    body = (f'<nav class="jump"><div class="shell">{jump}</div></nav><main class="shell">'
            f'{worlds}<h2>Scenarios</h2><div class="wrap"><table><tr><th>Scenario</th><th>Purpose</th>'
            f'<th>Expected parking behaviour</th><th>Simulators</th></tr>{"".join(rows)}</table></div>'
            f'{"".join(cards)}<p class="note">Scene definitions: '
            f'<code>src/ridgeback_autonomy/config/parking_scenarios.yaml</code>; reference: '
            f'<a href="parking_scenarios.md">parking scenarios</a>.</p></main>')
    with open(OUT, 'w') as stream:
        stream.write(pages.page('Autonomous parking scenarios', pages.SCENARIOS_PAGE, intro, body))
    # The folder holds this page's exports only: drop any the page no longer shows.
    used = set(re.findall(r'assets/parking_scenes/([^"]+)', open(OUT).read()))
    for name in sorted(set(os.listdir(ASSETS)) - used):
        os.remove(os.path.join(ASSETS, name))
    print('wrote', OUT, len(used), 'images')


if __name__ == '__main__':
    main()
