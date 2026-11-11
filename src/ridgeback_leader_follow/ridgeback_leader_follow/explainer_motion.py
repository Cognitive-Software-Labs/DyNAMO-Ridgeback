"""Exact 24-second pedestrian choreography from the explainer's sim block.

SVG coordinates map to metres by x'=x-2.2, y'=4.6-y. The observer
starts at the origin. Robot following in the illustration is not replayed.
"""
import math
from .crowd_scenarios import CrowdScenario,PersonPath,Waypoint

LEADER=((0,3.6,4.6),(5,8.6,4.6),(9,12.2,3.4),(14,16.4,3.),(18,19.6,3.8),(24,21.6,4.8))
OTHERS=(
 ((0,9.3,9.6),(3.5,9.3,9.),(9.5,9.3,0.),(24,9.3,-12.)),
 ((0,14.9,9.1),(24,14.9,-1.7)),
 ((0,14.9,9.7),(24,14.9,-1.1)),
 ((0,17.6,1.4),(24,17.6,1.4)),
 ((0,18.2,1.1),(24,18.2,1.1)),
 ((0,18.15,1.85),(24,18.15,1.85)),
 ((0,26.,7.3),(24,15.5,3.7)),
 ((0,.8,7.8),(24,19.,8.3)),
 ((0,21.,7.6),(24,3.,6.9)),
 ((0,19.,.4),(24,1.5,.6)),
)


def load_scenario():
    def person(name,role,keys):
        points=tuple(Waypoint(t,x-2.2,4.6-y) for t,x,y in keys)
        a,b=points[:2];heading=math.atan2(b.y-a.y,b.x-a.x)
        return PersonPath(name,role,(.15,.35,.85) if name in ('lf_leader','lf_lookalike') else (.5,.5,.5),points,heading)
    return CrowdScenario('explainer',24.,(person('lf_leader','leader',LEADER),)+tuple(
        person('lf_lookalike' if i==6 else f'lf_illustration_{i}','bystander',keys)
        for i,keys in enumerate(OTHERS)))
