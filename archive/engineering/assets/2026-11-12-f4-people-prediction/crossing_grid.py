import math,sys
import numpy as np
sys.path.insert(0,'src/ridgeback_leader_follow')
from ridgeback_leader_follow.people_predictor import PredictionSettings, speed_limit
S=PredictionSettings()
def run(cap,off,sp,v0,delay=.3):
    dt=.05;x=0.;v=v0;t=0.;best=9;wx=v0*2.+off;wy=sp*2.;hit_speed=0
    while t<5:
        wy_now=wy-sp*t
        lim=cap((x,0.),(v,0.),[(wx,wy_now,0.,-sp)]) if t>=delay else 1.3
        target=min(1.3,lim)
        v=min(v+dt,target) if target>v else max(target,v-dt)
        x+=v*dt;t+=dt
        d=math.hypot(wx-x,wy-sp*t)
        if d<best: best=d;hit_speed=v
    return best,hit_speed
new=lambda r,v,p:speed_limit(r,v,p,S)
c={'new':0,'none':0};hs={'new':[],'none':[]};n=0
for v0 in (0.,.5,1.1,1.3):
 for sp in (1.35,.9,.5):
  for off in np.arange(-2,2.01,.25):
    n+=1
    for k,f,d in (('new',new,.3),('none',new,99)):
        b,h=run(f,off,sp,v0,d)
        if b<.7: c[k]+=1; hs[k].append(h)
print('contacts',c,'of',n,'mean robot speed at contact',{k:round(float(np.mean(v)),2) for k,v in hs.items()})
