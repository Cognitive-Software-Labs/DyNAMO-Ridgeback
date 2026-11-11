import json, statistics as st, sys
ev=[json.loads(l) for l in open(sys.argv[1]+'.events.jsonl')]
for k in ('cmd_vel_nav','cmd_vel'):
    v=[e['vx'] for e in ev if e.get('cmd')==k]; nz=sorted(x for x in v if x>.05)
    print(f"{k:12s} max={max(v):.2f} p90={nz[int(.9*len(nz))]:.2f} median_moving={st.median(nz):.2f}")
