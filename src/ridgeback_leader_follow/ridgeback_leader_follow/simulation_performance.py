"""Opt-in runtime costs; reference recordings retain their original contracts."""
from dataclasses import dataclass, replace
import numpy as np


@dataclass(frozen=True)
class RuntimeTuning:
    human_detail: str = 'full'
    sensor_quality: str = 'reference'
    collision_quality: str = 'reference'
    # Isaac updates (renders) per simulated second; None keeps the recorded
    # timing. Sensors keep their own tick rates either way.
    isaac_step_hz: int | None = None

    @property
    def reduced_sensors(self):
        return self.sensor_quality == 'reduced'

    @property
    def simple_collisions(self):
        return self.collision_quality == 'simple'

    def camera_profile(self):
        from ridgeback_common.camera_profiles import resolve_camera_profile
        original=resolve_camera_profile('640x480')
        return replace(original,name='interactive_320x240_15',width=320,height=240,
                       focal_length_px=original.focal_length_px/2,fps=15.) if self.reduced_sensors else original

    def setup_robot(self,source):
        """Modify a run-local declaration only, leaving hardware config untouched."""
        import copy
        result=copy.deepcopy(source)
        if self.reduced_sensors:
            for camera in result.get('sensors',{}).get('camera',[]):
                if camera['model']!='intel_realsense':continue
                parameters=camera.setdefault('ros_parameters',{}).setdefault('intel_realsense',{})
                parameters.update({'rgb_camera.color_profile':'320,240,15',
                                   'depth_module.depth_profile':'320,240,15',
                                   'pointcloud.enable':False})
        return result


def resolve_tuning(profile='reference',human_detail=None,sensor_quality=None,collision_quality=None,isaac_step_hz=None):
    defaults=RuntimeTuning('distant','reduced','simple') if profile=='balanced' else RuntimeTuning()
    return RuntimeTuning(human_detail or defaults.human_detail,sensor_quality or defaults.sensor_quality,
                         collision_quality or defaults.collision_quality,isaac_step_hz)


def minimum_route_distance(person,observer):
    """Exact distance to each line segment, including between waypoints."""
    points=np.array([(p.x,p.y) for p in person.waypoints],float)
    a=points[:-1];v=points[1:]-a;q=np.asarray(observer)-a
    fractions=np.clip(np.sum(q*v,axis=1)/np.maximum(np.sum(v*v,axis=1),1e-15),0,1)
    return float(np.linalg.norm(a+fractions[:,None]*v-observer,axis=1).min())


def use_distance_lod(person,tuning):
    # Protect identity-critical pairs and any walker approaching either observer.
    return tuning.human_detail=='distant' and person.name not in ('lf_leader','lf_lookalike') and all(
        minimum_route_distance(person,p)>=7. for p in ((0.,0.),(-3.,-2.6)))


def prop_bounds(data,scale=1.):
    points=np.asarray(data['points']);low=points.min(axis=0);high=points.max(axis=0)
    return (low+high)*(.5*scale),np.maximum((high-low)*scale,.01)
