"""Offline replay of the parking order on recorded runs: the pushed planner (875b3af,
sides equal, then rear, front, other; six routes per class) against the working tree's.

For each run directory (run_scenario.sh output with a bag): the map and the G1 estimate
and heading of the first plan that reached route checks (else the first plan), and the
last observation goal in the console as the robot's start and approach. Both planners
rank the same map; both route searches run as the mission runs them (the new one with
its 16-check budget). Routes are checked by an APPROXIMATION of Nav2, which is not
available offline:
  * the pre-parking pose must be reachable from the observation pose through free mapped
    cells at least the footprint's inscribed radius from any occupied or unknown cell;
  * the straight final approach (pre-parking -> parking pose) must pass the mission's own
    swept-footprint and keep-out check (path_rejection).
Ground truth is not used. Needs the workspace sourced (ROS 2, rosbag2_py) and scipy.

Usage: replay_parking_order.py OUT_JSON RUN_DIR...            (recorded headings)
       replay_parking_order.py --sweep OUT_JSON RUN_DIR...    (G1 turned in 30-degree steps:
                                                               counterfactual, not observed)
"""
import importlib.util, json, math, os, re, subprocess, sys, types
import numpy as np
from scipy import ndimage
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, f'{ROOT}/src/ridgeback_autonomy')
from ridgeback_autonomy.mission import parking_planner as new_planner
from ridgeback_autonomy.mission import parking_feasibility as new_feas
from ridgeback_autonomy.mission.parking_geometry import parking_footprint, preparking_pose
from ridgeback_autonomy.mission.mission_manager_node import load_nav2_footprint


def old_modules(rev='875b3af'):
    """The pushed planner and route grouping, loaded side by side with the new ones."""
    mods = {}
    for name in ('parking_planner', 'parking_feasibility'):
        src = subprocess.run(['git', '-C', ROOT, 'show', f'{rev}:src/ridgeback_autonomy/ridgeback_autonomy/mission/{name}.py'],
                             capture_output=True, text=True, check=True).stdout
        src = src.replace('from ridgeback_autonomy.mission.parking_planner import', 'from old_parking_planner import')
        mod = types.ModuleType(f'old_{name}'); sys.modules[f'old_{name}'] = mod
        exec(compile(src, f'old_{name}.py', 'exec'), mod.__dict__)
        mods[name] = mod
    return mods['parking_planner'], mods['parking_feasibility']


old_planner, old_feas = old_modules()
FOOTPRINT = parking_footprint(load_nav2_footprint(f'{ROOT}/install/ridgeback_autonomy/share/ridgeback_autonomy/config/nav2_params.yaml'))
def _inscribed(polygon):
    """Distance from the base origin to the nearest footprint edge."""
    best = math.inf
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, -(x1 * dx + y1 * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(x1 + t * dx, y1 + t * dy))
    return best


INSCRIBED = _inscribed(list(FOOTPRINT))


def config(mod):
    return mod.PlannerConfig(footprint=FOOTPRINT, reach_allowance_m=0.03 + 0.03, min_obstacle_clearance_m=0.05 + 0.03)


def bag_inputs(run):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=f'{run}/bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    maps, plan_state, live = [], None, None
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic.endswith('/map'):
            maps.append((ts, data))
        elif topic.endswith('parking_mission/state'):
            m = json.loads(deserialize_message(data, String).data)
            # The first plan that reached route checks; else the first plan made at all.
            if (m.get('parking_plan') or {}).get('target'):
                checks = m.get('state') in ('PARKING_CANDIDATES_READY', 'EVALUATING_PARKING_PATHS')
                if checks and (plan_state is None or not plan_state[2]):
                    plan_state = (ts, m, True)
                elif plan_state is None:
                    plan_state = (ts, m, False)
            if live is None and m.get('state') == 'PARKING_PATH_FEASIBLE' and (m.get('parking_plan') or {}).get('selected'):
                live = m['parking_plan']['selected']
    if plan_state is None:
        return None
    ts, m, _ = plan_state
    grid_msg = deserialize_message(max((x for x in maps if x[0] <= ts), key=lambda x: x[0])[1], OccupancyGrid)
    return grid_msg, m, live


def observation_pose(run, before_ns):
    pose = None
    for line in open(f'{run}/console.log', errors='replace'):
        hit = re.search(r'\[(\d+\.\d+)\].*RETURNING_TO_OBSERVATION: NavigateToPose x=(-?[\d.]+), y=(-?[\d.]+), yaw=(-?[\d.]+)', line)
        if hit and float(hit.group(1)) * 1e9 <= before_ns:
            pose = tuple(float(v) for v in hit.groups()[1:])
    return pose


class RouteOracle:
    """Approximate route feasibility and length (see the module docstring)."""

    def __init__(self, grid, start, target, keepout, cfg):
        self.grid, self.start, self.target, self.keepout, self.cfg = grid, start, target, keepout, cfg
        data = grid.data
        free = (data >= 0) & (data < cfg.occupied_threshold)
        clear = ndimage.distance_transform_edt(free) * grid.resolution
        # The robot stands on its own start cells even if they are unknown.
        r0, c0 = self.cell(start[0], start[1])
        self.passable = clear >= INSCRIBED
        self.passable[max(0, r0 - 2):r0 + 3, max(0, c0 - 2):c0 + 3] = True
        self.dist = self._dijkstra((r0, c0))

    def cell(self, x, y):
        lx, ly = self.grid.map_to_local([x], [y])
        return int(ly[0] // self.grid.resolution), int(lx[0] // self.grid.resolution)

    def _dijkstra(self, src):
        import heapq
        h, w = self.passable.shape
        dist = np.full((h, w), np.inf); dist[src] = 0.0; q = [(0.0, src)]
        steps = [(dr, dc, math.hypot(dr, dc)) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
        while q:
            d, (r, c) = heapq.heappop(q)
            if d > dist[r, c]:
                continue
            for dr, dc, s in steps:
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and self.passable[rr, cc] and d + s < dist[rr, cc]:
                    dist[rr, cc] = d + s; heapq.heappush(q, (d + s, (rr, cc)))
        return dist * self.grid.resolution

    def check(self, c):
        pre = preparking_pose(c.base_x, c.base_y, c.yaw, 0.50)
        r, col = self.cell(pre[0], pre[1])
        h, w = self.dist.shape
        if not (0 <= r < h and 0 <= col < w) or not math.isfinite(self.dist[r, col]):
            return 'no_free_space_route', None
        path = Path(); path.header.frame_id = self.grid.frame_id
        for x, y, yaw in (pre, (c.base_x, c.base_y, c.yaw)):
            p = PoseStamped(); p.header.frame_id = self.grid.frame_id
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(p)
        reason = new_feas.path_rejection(path, c, pre, self.grid, self.target, self.cfg, self.keepout)
        return reason, None if reason else float(self.dist[r, col]) + 0.50


def search(feas, plan, approach, oracle, *, per_class=6, budget=None, old=False):
    """The mission's route search: groups in order, select after a group with a route."""
    checked, valid_routes = [], []
    for group in feas.candidate_groups(plan, approach, per_class):
        for c in group:
            if budget is not None and len(checked) >= budget:
                break
            reason, length = oracle.check(c)
            c.path_status, c.path_reason, c.path_length_m = ('rejected', reason, None) if reason else ('valid', None, length)
            checked.append(c)
            if not reason:
                valid_routes.append(c)
        if valid_routes:
            return feas.select_path_candidate(valid_routes, plan.resolution_m), checked
        if budget is not None and len(checked) >= budget:
            return None, checked
    return None, checked


def run_one(run, heading_override=None):
    inputs = bag_inputs(run)
    if inputs is None:
        return {'run': run, 'skipped': 'no planning state in the bag'}
    grid_msg, state, live = inputs
    target_xy = state['parking_plan']['target']
    yaw = (state.get('g1_heading') or {}).get('yaw_rad')
    if heading_override is not None:
        yaw = heading_override
    obs = observation_pose(run, math.inf)
    out = {'run': os.path.relpath(run, f'{ROOT}/artifacts/exploration'), 'target': target_xy,
           'heading_deg': None if yaw is None else round(math.degrees(yaw), 1),
           'live_selected': None if live is None else {'class': live.get('preference'), 'reach_m': live.get('reach_m'),
                                                       'path_length_m': round(live.get('path_length_m') or 0, 2)}}
    for name, planner, feas, kw in (('old', old_planner, old_feas, {}), ('new', new_planner, new_feas, {'budget': 16})):
        grid = planner.GridSnapshot.from_msg(grid_msg)
        target = planner.TargetReference(target_xy[0], target_xy[1], grid.frame_id, 0, 'generic_target_position',
                                         False, 'replay', facing_yaw_rad=yaw)
        cfg = config(planner)
        plan = planner.plan_parking(grid, target, cfg, obs, rays=False)
        approach = obs
        valid = [c for c in plan.candidates if c.status in ('valid', 'selected')]
        classes = {}
        for c in valid:
            classes[planner.preference(c.task_x, c.task_y, target, approach)[1]] = classes.get(
                planner.preference(c.task_x, c.task_y, target, approach)[1], 0) + 1
        oracle = RouteOracle(grid, obs, target, plan.keepout_radius_m, cfg)
        if not plan.ok:
            out[name] = {'valid_by_class': classes, 'checks': 0, 'checked_classes': [], 'selected': None,
                         'geometric_failure': plan.failure_reason}
            if name == 'new':
                out[name]['route_search'] = new_feas.route_search_summary(plan, approach)
                out[name]['shortest_feasible_route_by_class_m'] = {}
            continue
        sel, checked = search(feas, plan, approach, oracle, **kw)
        entry = {'valid_by_class': classes, 'checks': len(checked),
                 'checked_classes': [planner.preference(c.task_x, c.task_y, target, approach)[1] for c in checked]}
        if sel is not None:
            rel = math.degrees(math.atan2(sel.task_y - target.y, sel.task_x - target.x) - (yaw or 0.0))
            entry['selected'] = {'class': planner.preference(sel.task_x, sel.task_y, target, approach)[1],
                                 'bearing_from_heading_deg': None if yaw is None else round((rel + 180) % 360 - 180, 1),
                                 'reach_m': round(sel.reach_m, 3), 'clearance_m': round(sel.clearance_m, 3),
                                 'route_m': round(sel.path_length_m, 2), 'base': [round(sel.base_x, 3), round(sel.base_y, 3)]}
        else:
            entry['selected'] = None
        if name == 'new':
            entry['route_search'] = new_feas.route_search_summary(plan, approach)
            # All feasible classes under the oracle (for "shorter lower-class route" comparisons).
            best = {}
            for c in valid:
                reason, length = oracle.check(c)
                if not reason:
                    lab = planner.preference(c.task_x, c.task_y, target, approach)[1]
                    best[lab] = min(best.get(lab, 1e9), round(length, 2))
            entry['shortest_feasible_route_by_class_m'] = best
        out[name] = entry
    return out


def sweep(runs, step_deg=30):
    """Counterfactual: each recorded map and G1 position with G1 turned to every heading."""
    rows = []
    for run in runs:
        base = bag_inputs(run)
        if base is None:
            continue
        for k in range(0, 360, step_deg):
            r = run_one(run, heading_override=math.radians(k - 180))
            o, n = r['old']['selected'], r['new']['selected']
            rows.append({'run': r['run'], 'heading_deg': k - 180,
                         'old': o and o['class'], 'new': n and n['class'],
                         'old_route_m': o and o['route_m'], 'new_route_m': n and n['route_m'],
                         'new_checks': r['new']['checks'], 'old_checks': r['old']['checks'],
                         'feasible_classes': sorted(r['new']['shortest_feasible_route_by_class_m'],
                                                    key=new_planner.G1_CLASSES.index),
                         'new_search': new_feas.describe_route_search(r['new']['route_search'])})
    return rows


if __name__ == '__main__':
    if sys.argv[1] == '--sweep':
        rows = sweep(sys.argv[3:])
        json.dump(rows, open(sys.argv[2], 'w'), indent=1)
        sys.exit(0)
    results = [run_one(run) for run in sys.argv[2:]]
    json.dump(results, open(sys.argv[1], 'w'), indent=1)
    for r in results:
        if 'skipped' in r:
            print(r['run'], r['skipped']); continue
        o, n = r['old'], r['new']
        if n.get('geometric_failure'):
            print(f"{r['run']}: heading {r['heading_deg']}; live {r['live_selected']}; old and new: "
                  f"no valid pose ({o.get('geometric_failure')} / {n['geometric_failure']}), refused before route checks")
            continue
        fmt = lambda s: 'none' if s is None else f"{s['class']} ({s.get('bearing_from_heading_deg')} deg, reach {s['reach_m']}, route {s['route_m']} m)"
        print(f"{r['run']}: heading {r['heading_deg']}; live {r['live_selected'] and r['live_selected']['class']}\n"
              f"   old: {fmt(o['selected'])} after {o['checks']} checks; valid {o['valid_by_class']}\n"
              f"   new: {fmt(n['selected'])} after {n['checks']} checks; valid {n['valid_by_class']}\n"
              f"   new search: {new_feas.describe_route_search(n['route_search'])}\n"
              f"   shortest feasible route per class: {n['shortest_feasible_route_by_class_m']}")
