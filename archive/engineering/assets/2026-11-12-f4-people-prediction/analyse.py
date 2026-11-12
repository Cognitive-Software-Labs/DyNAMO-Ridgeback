"""Preliminary F4 per-run metrics from run dir + events.jsonl; odom targets mapped to world when odometry was recorded."""
import bisect, csv, json, math, statistics as st, sys
run, events = sys.argv[1], sys.argv[1] + '.events.jsonl'
truth = {}
for r in csv.DictReader(open(f'{run}/ground_truth.csv')):
    truth.setdefault(float(r['sim_time']), {})[r['person']] = (r['role'], float(r['x']), float(r['y']))
times = sorted(truth)
robot = [(float(r['stamp']), float(r['x']), float(r['y'])) for r in csv.DictReader(open(f'{run}/robot_truth.csv'))]
start, end = times[0], times[-1]
def people_at(t):
    i = min(max(bisect.bisect_left(times, t), 0), len(times) - 1); return truth[times[i]]
def label(t, x, y):
    d = sorted((math.hypot(px - x, py - y), role) for role, px, py in people_at(t).values())
    if not d or d[0][0] > .75 or (len(d) > 1 and d[1][0] - d[0][0] < .1): return 'unmatched'
    return d[0][1]
ev = [json.loads(l) for l in open(events)]
# Odometry-to-world correction: where odometry was recorded, map each odom-frame
# target through the truth pose at that time (world = T_truth * T_odom^-1 * p).
odom = [(e['odom_stamp'], e['ox'], e['oy'], e['oyaw']) for e in ev if 'odom_stamp' in e]
odom_t = [o[0] for o in odom]
robot_full = [(float(r['stamp']), float(r['x']), float(r['y']), float(r['yaw'])) for r in csv.DictReader(open(f'{run}/robot_truth.csv'))]
robot_t = [r[0] for r in robot_full]
def to_world(t, x, y):
    if not odom: return x, y
    i = min(bisect.bisect_left(odom_t, t), len(odom) - 1); j = min(bisect.bisect_left(robot_t, t), len(robot_full) - 1)
    _, ox, oy, oyaw = odom[i]; _, wx, wy, wyaw = robot_full[j]
    dx, dy = x - ox, y - oy; c, s_ = math.cos(-oyaw), math.sin(-oyaw); bx, by = c*dx - s_*dy, s_*dx + c*dy
    c, s_ = math.cos(wyaw), math.sin(wyaw); return wx + c*bx - s_*by, wy + s_*bx + c*by
states = [e for e in ev if 'state' in e and start <= e['stamp'] <= end]
stat = {'correct': 0., 'wrong': 0., 'unmatched': 0., 'not_locked': 0.}
for a, b in zip(states, states[1:]):
    dt = min(b['stamp'] - a['stamp'], .5)
    if a['state'] == 1 and a['has_leader']:
        l = label(a['stamp'], *to_world(a['stamp'], a['x'], a['y']))
        stat['correct' if l == 'leader' else 'wrong' if l == 'bystander' else 'unmatched'] += dt
    else: stat['not_locked'] += dt
dist, clear = [], []
for t, x, y in robot:
    for role, px, py in people_at(t).values():
        (dist if role == 'leader' else clear).append((t, math.hypot(px - x, py - y)))
moving = [d for t, d in dist if t - start > 3]
mov = [e['status'] for e in ev if 'status' in e]
summary = {'duration': round(end - start, 2), **{k + '_s': round(v, 2) for k, v in stat.items()},
           'distance_median': round(st.median(moving), 2), 'distance_error_median': round(st.median(abs(d - 1.2) for d in moving), 2),
           'distance_min': round(min(d for _, d in dist), 2), 'final_distance': round(dist[-1][1], 2),
           'min_bystander_clearance': round(min(d for _, d in clear), 2) if clear else None,
           'status_counts': {s.split(':')[0]: sum(m.startswith(s.split(':')[0]) for m in mov) for s in set(mov)}}
print(json.dumps(summary))
