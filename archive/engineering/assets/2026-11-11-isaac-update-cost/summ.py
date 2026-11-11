import json, sys, statistics as st
s = json.load(open(sys.argv[1])); t0 = s[-1][2]
# last 6 s of wall time = scenario loop
win = [x for x in s if x[2] > t0 - 6]
w = sorted(x[0]*1e3 for x in win); c = sorted(x[1]*1e3 for x in win)
span = win[-1][2] - win[0][2]
print(f"{sys.argv[1]}: n={len(win)} updates/s={len(win)/span:.1f} wall median={st.median(w):.1f}ms p90={w[int(.9*len(w))]:.1f} cpu median={st.median(c):.1f}ms")
