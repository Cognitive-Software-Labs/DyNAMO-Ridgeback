"""Quick F4 dev metrics: robot-leader distance and follow status for one run dir."""
import csv, math, statistics as st, sys
run = sys.argv[1]
robot = {round(float(r['scenario_time']), 2): r for r in csv.DictReader(open(f'{run}/robot_truth.csv'))}
people = {}
for r in csv.DictReader(open(f'{run}/ground_truth.csv')):
    people.setdefault(round(float(r['scenario_time']), 2), {})[r['person']] = r
ds, clear = [], []
for t, R in sorted(robot.items()):
    if t not in people: continue
    rx, ry = float(R['x']), float(R['y'])
    for name, p in people[t].items():
        d = math.hypot(float(p['x']) - rx, float(p['y']) - ry)
        (ds if p['role'] == 'leader' else clear).append((t, d))
moving = [d for t, d in ds if t > 3]
print(f"leader distance after 3 s: median {st.median(moving):.2f} min {min(moving):.2f} max {max(moving):.2f}")
if clear: print(f"min bystander clearance (centre-centre) {min(d for _, d in clear):.2f}")
last = robot[max(robot)]; print(f"robot end ({float(last['x']):.2f}, {float(last['y']):.2f}) yaw {float(last['yaw']):.2f}")
