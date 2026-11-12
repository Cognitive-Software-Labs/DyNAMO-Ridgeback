"""Navigation quality from simulator truth: camera view, smoothness, gap, behaviour near walkers."""
import bisect, csv, json, math, sys
run = sys.argv[1]
truth = {}
for r in csv.DictReader(open(f'{run}/ground_truth.csv')):
    truth.setdefault(float(r['sim_time']), {})[r['person']] = (r['role'], float(r['x']), float(r['y']))
times = sorted(truth)
robot = [(float(r['stamp']), float(r['x']), float(r['y']), float(r['yaw'])) for r in csv.DictReader(open(f'{run}/robot_truth.csv'))]
def at(t): return truth[times[min(bisect.bisect_left(times, t), len(times) - 1)]]
half_fov = math.radians(43)
in_view = gap = n = 0; bearings = []; speeds = []; yaw_rates = []; accels = []; near = []
for k, (t, x, y, yaw) in enumerate(robot):
    people = at(t)
    _, lx, ly = people['lf_leader']
    b = math.atan2(ly - y, lx - x) - yaw; b = math.atan2(math.sin(b), math.cos(b)); bearings.append(abs(b))
    in_view += abs(b) <= half_fov; n += 1
    if k:
        dt = t - robot[k-1][0]
        v = math.hypot(x - robot[k-1][1], y - robot[k-1][2]) / dt; speeds.append(v)
        dyaw = math.atan2(math.sin(yaw - robot[k-1][3]), math.cos(yaw - robot[k-1][3])); yaw_rates.append(abs(dyaw) / dt)
        if len(speeds) > 1: accels.append(abs(speeds[-1] - speeds[-2]) / dt)
        for name, (role, px, py) in people.items():
            if role == 'bystander' and math.hypot(px - x, py - y) < .9: near.append(v)
leader_path = sorted(set(truth))
print(json.dumps({
    'leader_in_camera_view_pct': round(100 * in_view / n, 1),
    'leader_bearing_median_deg': round(math.degrees(sorted(bearings)[len(bearings)//2]), 1),
    'leader_bearing_p95_deg': round(math.degrees(sorted(bearings)[int(.95*len(bearings))]), 1),
    'robot_speed_max': round(max(speeds), 2),
    'robot_yaw_rate_p95_deg_s': round(math.degrees(sorted(yaw_rates)[int(.95*len(yaw_rates))]), 1),
    'robot_accel_p95': round(sorted(accels)[int(.95*len(accels))], 2),
    'robot_path_m': round(sum(math.hypot(robot[k][1]-robot[k-1][1], robot[k][2]-robot[k-1][2]) for k in range(1, len(robot))), 1),
    'samples_within_0.9m_of_walker': len(near), 'robot_speed_there_max': round(max(near), 2) if near else None,
    'robot_speed_there_mean': round(sum(near)/len(near), 2) if near else None}))
