"""Supplementary F3 recognition tests; the six F1 scenario families stay fixed."""
from .crowd_scenarios import CrowdScenario, PersonPath, Waypoint

F3_SCENARIOS=('leader_out_of_view',)


def load_recognition_scenario(name):
    if name not in F3_SCENARIOS:
        raise ValueError(f'Unknown recognition scenario {name}')
    leader=PersonPath('lf_leader','leader',(.2,.4,.8),(
        Waypoint(0.,2.,0.),Waypoint(3.,2.,0.),Waypoint(7.,2.,2.3),
        Waypoint(12.,2.,2.3),Waypoint(16.,2.,0.),Waypoint(20.,4.,0.)))
    bystander=PersonPath('lf_bystander','bystander',(.8,.3,.1),(
        Waypoint(0.,5.,-1.3),Waypoint(20.,5.,-1.3)))
    return CrowdScenario(name,20.,(leader,bystander))
