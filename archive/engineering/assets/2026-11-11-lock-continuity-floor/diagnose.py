"""For each LOCKED->UNCERTAIN(track_discontinuity) drop, report which continuity rule failed."""
import json, math, sys, collections
from pathlib import Path
SRC = Path('artifacts/leader-following/f3-live-validation-qualified-profile')
base = Path(sys.argv[1]); agg = collections.Counter(); detail = []
for case in sorted(p.name for p in base.iterdir() if p.is_dir()):
    people = [json.loads(l) for l in open(SRC / case / 'people.jsonl')]
    leaders = [json.loads(l) for l in open(base / case / 'leader.jsonl')]
    for i in range(1, len(leaders)):
        if not (leaders[i-1]['state'] == 1 and leaders[i]['reason'] == 'track_discontinuity'): continue
        tid = leaders[i-1]['track_id']; prev = {p['track_id']: p for p in people[i-1]['people']}
        cur = {p['track_id']: p for p in people[i]['people']}
        dt = people[i]['sim_time'] - people[i-1]['sim_time']
        why = []
        if tid not in prev: why.append('absent_prev')
        if tid not in cur: why.append('absent_now')
        if tid in prev and tid in cur:
            a, b = prev[tid], cur[tid]
            if not 0 < dt <= .75: why.append(f'dt={dt:.2f}')
            if b['confidence'] < .5: why.append(f'conf={b["confidence"]:.2f}')
            px, py = a['x'] + a['vx']*dt, a['y'] + a['vy']*dt
            err = math.hypot(b['x']-px, b['y']-py)
            if err > .75: why.append(f'err={err:.2f}')
            comp = [(o, math.hypot(c['x']-px, c['y']-py)) for o, c in cur.items() if o != tid and c['confidence'] >= .5]
            close = [(o, d) for o, d in comp if d <= err + .1]
            if close: why.append(f'competitor err={err:.2f} other={min(d for _, d in close):.2f}')
        key = why[0].split('=')[0].split(' ')[0] if why else 'none'
        agg[key] += 1; detail.append((case, round(people[i]['sim_time'], 2), why))
print(dict(agg))
for d in detail[:40]: print(d)
