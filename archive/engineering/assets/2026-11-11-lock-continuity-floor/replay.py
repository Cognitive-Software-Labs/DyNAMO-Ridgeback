"""Replay recorded People (people.jsonl) through LeaderIdentity offline.

usage: replay.py CASE_DIR OUT_DIR   (writes OUT_DIR/leader.jsonl, prints match vs original)
"""
import json, math, sys
from pathlib import Path
from types import SimpleNamespace as NS
from ridgeback_leader_follow.leader_identity_node import LeaderIdentity, STATE_CODES
from ridgeback_leader_follow.lock_on import LeaderLock

PROFILE = dict(acquire_interval=1, acquire_frames=3, lost_frames=8, match_threshold=.65, recovery_margin=.15)

def msg_of(row):
    sec = math.floor(row['sim_time']); ns = round((row['sim_time'] - sec) * 1e9)
    if ns >= 10**9: sec += 1; ns -= 10**9
    people = [NS(id=p['track_id'], confidence=p['confidence'],
                 position=NS(x=p['x'], y=p['y'], z=0.), velocity=NS(x=p['vx'], y=p['vy'], z=0.),
                 appearance=p['appearance']) for p in row['people']]
    return NS(header=NS(stamp=NS(sec=sec, nanosec=ns), frame_id=row['frame']), people=people)

def replay(case, out, lock_kwargs=None, identity_cls=LeaderIdentity, to_base=(0., 0., 0.)):
    lock = LeaderLock(**{**PROFILE, **(lock_kwargs or {})})
    ident = identity_cls(lock, 8.0, math.radians(30.0), .25, .1)
    rows = []
    for line in open(Path(case) / 'people.jsonl'):
        r = json.loads(line)
        s = ident.update(msg_of(r), to_base)
        pos = s.position
        rows.append({'sim_time': r['sim_time'], 'frame': r['frame'], 'x': float(pos.x), 'y': float(pos.y),
                     'state': int(s.state), 'reason': s.reason, 'has_leader': bool(s.has_leader),
                     'track_id': int(s.track_id)})
    Path(out).mkdir(parents=True, exist_ok=True)
    with open(Path(out) / 'leader.jsonl', 'w') as f:
        for r in rows: f.write(json.dumps(r) + '\n')
    return rows

if __name__ == '__main__':
    rows = replay(sys.argv[1], sys.argv[2])
    orig = [json.loads(l) for l in open(Path(sys.argv[1]) / 'leader.jsonl')]
    om = {round(o['sim_time'], 6): o for o in orig if o['reason'] != 'no_base_transform'}
    same = sum(1 for r in rows if (o := om.get(round(r['sim_time'], 6))) and o['state'] == r['state'] and o['track_id'] == r['track_id'])
    print(f"{Path(sys.argv[1]).name}: replayed={len(rows)} original={len(orig)} matched_state_and_id={same}")
