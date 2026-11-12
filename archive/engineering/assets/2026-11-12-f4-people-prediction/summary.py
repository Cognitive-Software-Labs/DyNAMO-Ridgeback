"""Aggregate one line per run: lock, wrong-follow, distance, clearance, behaviour near walkers, interventions."""
import json, subprocess, sys
from collections import Counter
P = sys.argv[1]; runs = sys.argv[2:]
print(f"{'run':34s} {'onLdr':>6s} {'wrong':>5s} {'unmat':>5s} {'off':>5s} {'medD':>5s} {'endD':>5s} {'clr':>5s} {'vNear':>5s} {'view%':>5s} {'stops':>5s} {'minLim':>6s}")
for r in runs:
    try:
        a = json.loads(subprocess.check_output(['python3', f'{P}/analyse.py', r]).decode())
        n = json.loads(subprocess.check_output(['python3', f'{P}/nav_check.py', r]).decode())
        ev = [json.loads(l) for l in open(r + '.events.jsonl')]
        stops = sum(1 for e in ev if e.get('polygon') == 'StopZone')
        lim = [e['speed_limit'] for e in ev if 'speed_limit' in e]
        clr = a['min_bystander_clearance']
        print(f"{r.split('/')[-1]:34s} {a['correct_s']:6.1f} {a['wrong_s']:5.1f} {a['unmatched_s']:5.1f} {a['not_locked_s']:5.1f} {a['distance_median']:5.2f} {a['final_distance']:5.2f} "
              f"{(clr if clr is not None else float('nan')):5.2f} {(n['robot_speed_there_max'] or 0):5.2f} {n['leader_in_camera_view_pct']:5.1f} {stops:5d} {min(lim) if lim else float('nan'):6.2f}")
    except Exception as error:
        print(f"{r.split('/')[-1]:34s} FAILED {error}")
