"""Per-run scenario metrics: mission log (estimated state) and Gazebo ground truth.

Usage: scen_metrics.py RUN_DIR [...]  -> one JSON object per run on stdout.
Ground truth comes from truth_track.csv (about 1 Hz) and never feeds the mission.
"""
import csv, json, math, os, re, sys
import xml.etree.ElementTree as ET
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../..'))
SCEN = yaml.safe_load(open(f'{ROOT}/src/ridgeback_autonomy/config/parking_scenarios.yaml'))['scenarios']
CAT = yaml.safe_load(open(f'{ROOT}/src/ridgeback_autonomy_gz/sim/benchmark_assets.yaml'))['objects']
TASK_X = -0.28
# Robot parts in base_link x/y and height above the floor (base_link 0.026 m up in Gazebo).
PARTS = {'base': ((-0.4706, 0.4618), (-0.3966, 0.3966), (0.0, 0.306)),
         'mast': ((0.1768, 0.2143), (-0.0188, 0.0188), (0.32, 1.121)),
         'camera': ((0.2143, 0.285), (-0.062, 0.062), (1.046, 1.075)),
         'basket': ((-0.48, -0.08), (-0.19, 0.19), (0.32, 0.68))}
G1_RADIUS = 0.19
END = ('MISSION_COMPLETE', 'MISSION_FAILED', 'PARKING_PLANNING_FAILED', 'PARKING_EXECUTION_FAILED',
       'MOTION_SETTLEMENT_FAILED')


def rect(cx, cy, yaw, xr, yr):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c*x - s*y, cy + s*x + c*y) for x, y in
            ((xr[0], yr[0]), (xr[1], yr[0]), (xr[1], yr[1]), (xr[0], yr[1]))]


def seg_dist(p, a, b):
    ax, ay = b[0]-a[0], b[1]-a[1]
    t = max(0, min(1, ((p[0]-a[0])*ax + (p[1]-a[1])*ay) / (ax*ax + ay*ay or 1)))
    return math.hypot(p[0]-a[0]-t*ax, p[1]-a[1]-t*ay)


def inside(p, poly):
    sign = None
    for a, b in zip(poly, poly[1:] + poly[:1]):
        cross = (b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0])
        if sign is None:
            sign = cross >= 0
        elif (cross >= 0) != sign:
            return False
    return True


def poly_dist(p, q):
    if any(inside(v, q) for v in p) or any(inside(v, p) for v in q):
        return 0.0
    return min(min(seg_dist(v, a, b) for a, b in zip(q, q[1:] + q[:1])) for v in p) if True else 0


def dist(p, q):
    return min(poly_dist(p, q), poly_dist(q, p))


def obstacles(scene):
    out = []
    for item in scene.get('objects') or []:
        sdf = ET.parse(f"{ROOT}/src/ridgeback_autonomy_gz/sim/{CAT[item['asset']]['resource']}").getroot()
        yaw = math.radians(item['yaw_deg'])
        for c in sdf.iter('collision'):
            box, cyl = c.find('geometry/box/size'), c.find('geometry/cylinder')
            if box is not None:
                sx, sy, sz = map(float, box.text.split())
            elif cyl is not None:      # a cylinder as its bounding square
                sx = sy = 2 * float(cyl.find('radius').text)
                sz = float(cyl.find('length').text)
            else:
                continue
            px, py, pz = map(float, (c.find('pose').text.split()[:3]))
            ox = item['x'] + px*math.cos(yaw) - py*math.sin(yaw)
            oy = item['y'] + px*math.sin(yaw) + py*math.cos(yaw)
            out.append((f"{item['name']}", rect(ox, oy, yaw, (-sx/2, sx/2), (-sy/2, sy/2)),
                        (pz - sz/2, pz + sz/2)))
    return out


def clearance(x, y, yaw, obs, g1):
    best = (math.inf, '')
    for part, (xr, yr, zr) in PARTS.items():
        poly = rect(x, y, yaw, xr, yr)
        for name, opoly, (z0, z1) in obs:
            if zr[0] < z1 and z0 < zr[1]:
                d = dist(poly, opoly)
                if d < best[0]:
                    best = (d, f'{part}-{name}')
        if g1 and zr[0] < 0.8:
            d = max(0.0, min(seg_dist(g1, a, b) for a, b in zip(poly, poly[1:] + poly[:1]))
                    - G1_RADIUS) if not inside(g1, poly) else 0.0
            if d < best[0]:
                best = (d, f'{part}-G1 lower body')
    return best


def side(task, g1, g1yaw):
    rel = math.atan2(task[1]-g1[1], task[0]-g1[0]) - g1yaw
    rel = math.degrees(math.atan2(math.sin(rel), math.cos(rel)))
    if abs(abs(rel) - 90) <= 45:
        return 'g1_left' if rel > 0 else 'g1_right'
    return 'g1_rear' if abs(rel) >= 150 else 'g1_front' if abs(rel) <= 30 else 'g1_other'


def run(d):
    log = open(f'{d}/console.log', errors='replace').read()
    res = open(f'{d}/result.txt').read().split()
    name, backend = res[0], res[1]
    # The layout that actually ran: definitions changed during testing.
    own = f'{d}/scenarios.yaml'
    scene = (yaml.safe_load(open(own))['scenarios'] if os.path.exists(own) else SCEN)[name]
    tr = [(float(t), a, b, w) for t, a, b, w in re.findall(
        r'\[(\d+\.\d+)\] \[[^\]]*mission_manager_node\]: Mission state transition: (\w+) -> (\w+); reason=([^\n]*)', log)]
    m = dict(scenario=name, simulator=backend, run=os.path.basename(d),
             commit=open(f'{d}/commit.txt').readline().strip()[:7], expected=scene['expected'],
             description=scene['description'])
    g = scene['g1']
    m['g1_pose'] = f"({g['x']}, {g['y']}) yaw {g['yaw_deg']} deg"
    m['objects'] = ', '.join(f"{o['asset']} {o['name']} ({o['x']}, {o['y']}) {o['yaw_deg']} deg"
                             for o in scene.get('objects') or []) or 'none'
    if not tr:
        m['outcome'] = 'no mission transitions'
        return m
    t0 = tr[0][0]
    end = next(((t, b, w) for t, a, b, w in reversed(tr) if b in END), None)
    m['outcome'] = end[1] if end else res[2]
    m['final_reason'] = end[2][:220] if end else ''
    m['mission_s'] = round((end[0] if end else tr[-1][0]) - t0, 1)
    m['exploration_s'] = round(sum(t2 - t for (t, a, b, w), (t2, *_) in zip(tr, tr[1:])
                                   if b in ('SEARCH_G1', 'EXPLORING_AROUND_G1')), 1)
    m['attempts'] = sum(1 for t, a, b, w in tr if b == 'NAVIGATING_TO_PREPARKING')
    m['selected_sides'] = re.findall(r'parking pose selected: .* (g1_\w+|front|side_\w+|back\w*)\n', log)
    goals = re.findall(r'parking pose selected: x=(-?[\d.]+), y=(-?[\d.]+), yaw=(-?[\d.]+)', log)
    m['attempt_failures'] = re.findall(r'parking attempt \d+ failed: ([^;\n]+)', log)
    m['lidar_min_m'] = [float(v) for v in re.findall(r"LiDAR clearance in NAVIGATING_TO_PARKING: \{'checks': \d+, 'min_nearest_m': ([\d.]+)", log)]
    m['g1_heading_estimated'] = 'facing_yaw' in log or 'G1 heading' in log
    heads = re.findall(r'G1 heading[^\n]*?(-?\d+\.\d+) ?(?:deg|°)', log)
    m['g1_heading_log'] = heads[-1] if heads else None
    m['returned_to_start'] = 'Arrived at start' in log or 'returned to start pose' in log
    m['parked'] = any(b == 'PARKED' for t, a, b, w in tr)
    path = f'{d}/truth_track.csv'
    if backend != 'gz' or not os.path.exists(path):
        return m
    rows = [r for r in csv.reader(open(path)) if r[1] and r[3]]
    track = [(float(r[0]), float(r[1]), float(r[2]), float(r[3])) for r in rows]
    g1 = next(((float(r[4]), float(r[5]), float(r[6])) for r in rows if r[4]), None)
    obs = obstacles(scene)
    m['g1_truth'] = [round(v, 3) for v in g1] if g1 else None
    if track:
        c = min((clearance(x, y, yaw, obs, g1[:2] if g1 else None) + (t,) for t, x, y, yaw in track),
                key=lambda v: v[0])
        m['min_truth_clearance_m'] = round(c[0], 3)
        m['min_truth_clearance_pair'] = c[1]
        if obs:   # objects only: did any robot part (mast, camera, basket, base) come near one?
            o = min((clearance(x, y, yaw, obs, None) for t, x, y, yaw in track), key=lambda v: v[0])
            m['min_object_clearance_m'], m['min_object_clearance_pair'] = round(o[0], 3), o[1]
        start = track[0]
        last = track[-1]
        m['final_truth_pose'] = [round(v, 3) for v in last[1:]]
        m['final_distance_from_start_m'] = round(math.hypot(last[1]-start[1], last[2]-start[2]), 3)
    parked_t = next((t for t, a, b, w in tr if b == 'PARKED'), None)
    left_t = next((t for t, a, b, w in tr if a == 'PARKED'), None)
    if parked_t and track and g1:
        # The robot stands still between PARKED and the next transition: the last sample there.
        held = [r for r in track if parked_t + 1.0 <= r[0] <= (left_t or parked_t + 10.0)]
        t, x, y, yaw = held[-1] if held else min(track, key=lambda r: abs(r[0] - (left_t or parked_t)))
        m['parked_truth_sample_s'] = round(t - parked_t, 1)
        task = (x + TASK_X*math.cos(yaw), y + TASK_X*math.sin(yaw))
        m['parked_truth_pose'] = [round(x, 3), round(y, 3), round(yaw, 3)]
        m['reach_truth_m'] = round(math.hypot(task[0]-g1[0], task[1]-g1[1]), 3)
        m['side_truth'] = side(task, g1, g1[2])
        rel = math.atan2(task[1]-g1[1], task[0]-g1[0]) - g1[2]
        m['bearing_from_g1_heading_deg'] = round(math.degrees(math.atan2(math.sin(rel), math.cos(rel))), 1)
        near = min(math.hypot(px-g1[0], py-g1[1]) for px, py in rect(x, y, yaw, (-0.48, -0.08), (-0.19, 0.19)))
        m['basket_nearest_edge_to_g1_m'] = round(near, 3)
        m['basket_access_plausible'] = 0.30 <= m['reach_truth_m'] <= 0.60
        c = clearance(x, y, yaw, obs, g1[:2])
        m['parked_truth_clearance_m'], m['parked_truth_clearance_pair'] = round(c[0], 3), c[1]
        if goals:
            gx, gy, gyaw = map(float, goals[-1])
            m['pose_error_m'] = round(math.hypot(x-gx, y-gy), 3)
            m['yaw_error_deg'] = round(math.degrees(abs(math.atan2(math.sin(yaw-gyaw), math.cos(yaw-gyaw)))), 1)
    return m


if __name__ == '__main__':
    for d in sys.argv[1:]:
        print(json.dumps(run(d)))
