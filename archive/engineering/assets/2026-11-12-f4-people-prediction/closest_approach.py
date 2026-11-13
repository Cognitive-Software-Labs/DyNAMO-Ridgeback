"""Min robot-leader and robot-bystander centre distance, aligned on scenario time."""
import csv, math, sys
for run in sys.argv[1:]:
    try:
        rob={round(float(r['scenario_time']),2):(float(r['x']),float(r['y'])) for r in csv.DictReader(open(f'{run}/robot_truth.csv'))}
    except (FileNotFoundError, NotADirectoryError): continue
    best={}
    for r in csv.DictReader(open(f'{run}/ground_truth.csv')):
        k=round(float(r['scenario_time']),2)
        if k in rob:
            d=math.hypot(rob[k][0]-float(r['x']),rob[k][1]-float(r['y']))
            key='leader' if r['role']=='leader' else 'other'
            if d<best.get(key,(9e9,))[0]: best[key]=(d,k,r['person'])
    print(f"{run:60s} leader {best.get('leader',(float('nan'),0))[0]:.2f}@{best.get('leader',(0,0))[1]:.1f}s  bystander {best.get('other',(float('nan'),0,''))[0]:.2f}@{best.get('other',(0,0))[1]:.1f}s {best.get('other',('','',''))[2]}")
