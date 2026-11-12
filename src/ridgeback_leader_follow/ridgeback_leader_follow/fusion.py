"""Sensor-only RGB-D measurements and uncertainty-aware planar people fusion."""
from dataclasses import dataclass, field
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
from ridgeback_leader_follow.appearance import normalize_embedding


@dataclass
class Measurement:
    x: float
    y: float
    variance: float
    confidence: float
    source: str
    appearance: tuple = ()
    camera_id: int | None = None

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x,self.y,self.variance,self.confidence)):
            raise ValueError('Measurement must be finite')
        if self.variance <= 0 or not 0 <= self.confidence <= 1:
            raise ValueError('Invalid measurement uncertainty or confidence')


def rigid_matrix(translation, quaternion):
    from scipy.spatial.transform import Rotation
    q = np.asarray(quaternion,dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError('Invalid rotation')
    matrix = np.eye(4)
    matrix[:3,:3] = Rotation.from_quat(q).as_matrix()
    matrix[:3,3] = translation
    if not np.all(np.isfinite(matrix)):
        raise ValueError('Invalid translation')
    return matrix


def depth_in_color(depth, depth_k, color_k, color_shape, color_from_depth):
    """Register metric depth with a nearest-surface z-buffer, including baseline."""
    depth = np.asarray(depth)
    v,u = np.nonzero(np.isfinite(depth) & (depth > 0))
    z = depth[v,u]
    kd,kc = np.asarray(depth_k).reshape(3,3),np.asarray(color_k).reshape(3,3)
    xyz = np.stack(((u-kd[0,2])*z/kd[0,0],(v-kd[1,2])*z/kd[1,1],z,np.ones_like(z)))
    xyz = np.asarray(color_from_depth) @ xyz
    good = xyz[2] > 0
    xyz = xyz[:,good]
    uv = kc @ xyz[:3]
    u,v = np.rint(uv[:2]/uv[2]).astype(int)
    h,w = color_shape
    good = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    result = np.full(h*w,np.inf)
    np.minimum.at(result,v[good]*w+u[good],xyz[2,good])
    return result.reshape(h,w)


def box_measurement(box, confidence, registered_depth, color_k, output_from_color,
                    minimum_pixels=12):
    """Central torso median rejects background at box boundaries; no truth input."""
    x1,y1,x2,y2 = map(float,box)
    h,w = registered_depth.shape
    left,right = max(0,int(x1+.3*(x2-x1))),min(w,int(x2-.3*(x2-x1)))
    top,bottom = max(0,int(y1+.25*(y2-y1))),min(h,int(y1+.7*(y2-y1)))
    values = registered_depth[top:bottom,left:right]
    values = values[np.isfinite(values) & (values > .15) & (values < 15.)]
    if len(values) < minimum_pixels:
        return None
    z = float(np.median(values))
    k = np.asarray(color_k).reshape(3,3)
    if not np.all(np.isfinite(k)) or k[0,0] <= 0 or k[1,1] <= 0:
        raise ValueError('Invalid camera intrinsics')
    u,v = (x1+x2)/2,(y1+y2)/2
    xyz = np.asarray(output_from_color) @ np.array([(u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z,1.])
    spread = float(np.median(np.abs(values-z)))
    return Measurement(float(xyz[0]),float(xyz[1]),max(.04,.03*z+spread)**2,
                       float(confidence),'camera')


@dataclass
class FusedTrack:
    identity: int
    state: np.ndarray
    covariance: np.ndarray
    last_seen: float
    confidence: float
    appearance: tuple = field(default_factory=tuple)
    camera_id: int | None = None
    association_feature: tuple = field(default_factory=tuple)


class PeopleFusion:
    """CV Kalman filter; each sensor updates tracks at most once per tick.

    Lidar clusters alone never create people: camera confirmation is required.
    Confirmed tracks may be observed outside the camera through gated lidar.
    Unobserved tracks persist internally but are not published as detections.
    """
    def __init__(self, gate=.75, max_age=.75, acceleration=1.5, appearance_weight=0.):
        if not all(math.isfinite(v) and v > 0 for v in (gate,max_age,acceleration)):
            raise ValueError('Fusion parameters must be positive and finite')
        if not math.isfinite(appearance_weight) or appearance_weight < 0:
            raise ValueError("Appearance association weight must be finite and nonnegative")
        self.gate,self.max_age,self.acceleration = gate,max_age,acceleration
        self.appearance_weight=appearance_weight
        self.tracks=[]
        self.next_id=1
        self.stamp=None
        # Lidar people detections no track claimed on the latest update.
        self.unmatched_lidar=[]

    def update(self, stamp, measurements):
        if not math.isfinite(stamp) or self.stamp is not None and stamp <= self.stamp:
            raise ValueError('Fusion stamps must strictly increase')
        dt = 0. if self.stamp is None else stamp-self.stamp
        self.stamp=stamp
        self.tracks=[t for t in self.tracks if stamp-t.last_seen <= self.max_age]
        f=np.eye(4); f[0,2]=f[1,3]=dt
        g=np.array([[dt*dt/2,0],[0,dt*dt/2],[dt,0],[0,dt]])
        for t in self.tracks:
            t.state=f@t.state
            t.covariance=f@t.covariance@f.T+self.acceleration**2*g@g.T
            t.appearance=()
        observed=set()
        self.unmatched_lidar=[]
        # Camera first gives semantic confirmation; lidar refines existing tracks.
        sources=('camera','lidar','support') if self.appearance_weight else ('camera','lidar')
        for source in sources:
            samples=[m for m in measurements if m.source == source]
            if source == 'camera' and self.appearance_weight:
                unique=[]
                for measurement in sorted(samples,key=lambda m:m.confidence,reverse=True):
                    if all(math.hypot(measurement.x-old.x,measurement.y-old.y) >= .25 for old in unique):
                        unique.append(measurement)
                samples=unique
            matched=set()
            if samples and self.tracks:
                costs=np.array([[math.hypot(t.state[0]-m.x,t.state[1]-m.y) for m in samples] for t in self.tracks])
                penalty=(min(costs.shape)+1)*(self.gate+1)
                valid=costs <= (min(self.gate,.3) if source == 'support' else self.gate)
                if source == 'support':
                    for i,t in enumerate(self.tracks):
                        if t.identity in observed:valid[i,:]=False
                if source == 'camera':
                    for i,t in enumerate(self.tracks):
                        for j,m in enumerate(samples):
                            if m.camera_id is not None and m.camera_id == t.camera_id:
                                costs[i,j]*=.9 if self.appearance_weight else .1
                            if self.appearance_weight:
                                first=normalize_embedding(t.association_feature)
                                second=normalize_embedding(m.appearance)
                                if first is not None and second is not None and first.shape == second.shape:
                                    costs[i,j]+=self.appearance_weight*(1-float(np.clip(first@second,-1.,1.)))
                if self.appearance_weight:
                    if not np.isfinite(costs).all():raise ValueError("Nonfinite association costs")
                    penalty=(min(costs.shape)+1)*(float(np.max(costs))+1.)
                rows,cols=linear_sum_assignment(np.where(valid,costs,penalty))
                for i,j in zip(rows,cols):
                    if not valid[i,j]: continue
                    t,m=self.tracks[i],samples[j]
                    r=np.eye(2)*m.variance
                    gain=np.linalg.solve(t.covariance[:2,:2]+r,t.covariance[:,:2].T).T
                    t.state += gain@(np.array([m.x,m.y])-t.state[:2])
                    # Joseph form preserves a positive covariance under roundoff.
                    a=np.eye(4); a[:,:2]-=gain
                    t.covariance=a@t.covariance@a.T+gain@r@gain.T
                    t.last_seen=stamp
                    t.confidence=m.confidence
                    if source == 'camera':
                        t.appearance=m.appearance
                        feature=normalize_embedding(m.appearance)
                        if feature is not None:t.association_feature=tuple(feature)
                        if m.camera_id is not None:t.camera_id=m.camera_id
                    observed.add(t.identity); matched.add(j)
            if source == 'lidar':
                self.unmatched_lidar=[m for j,m in enumerate(samples) if j not in matched]
            if source == 'camera':
                for j,m in enumerate(samples):
                    if j in matched: continue
                    t=FusedTrack(self.next_id,np.array([m.x,m.y,0.,0.]),np.diag([m.variance,m.variance,1.,1.]),stamp,m.confidence,m.appearance,m.camera_id)
                    feature=normalize_embedding(m.appearance)
                    if feature is not None:t.association_feature=tuple(feature)
                    self.next_id+=1; self.tracks.append(t); observed.add(t.identity)
        return [t for t in self.tracks if t.identity in observed]


@dataclass
class Mover:
    identity: int
    state: np.ndarray
    covariance: np.ndarray
    last_seen: float
    origin: np.ndarray
    hits: int = 1


class LidarMovers:
    """Moving people seen only by lidar, for collision avoidance only.

    PeopleFusion requires the camera before a person exists, so someone
    approaching from outside the camera view is invisible to the identity and
    prediction layers until they enter it. This tracker follows the lidar
    people detections that no confirmed track claimed and reports those that
    are clearly walking: seen on ``min_hits`` updates, at least
    ``min_displacement`` from where they were first seen, and faster than
    ``min_speed``. Static detections (furniture false positives, standing
    people) stay out: the lidar costmap already handles them. These tracks
    carry no appearance and never become leader candidates.
    """
    def __init__(self, gate=.6, max_age=.5, acceleration=3., variance=.02,
                 min_hits=3, min_speed=.3, min_displacement=.3):
        if not all(math.isfinite(v) and v > 0 for v in (gate,max_age,acceleration,variance)) or min_hits < 1:
            raise ValueError('Mover parameters must be positive and finite')
        self.gate,self.max_age,self.acceleration,self.variance=gate,max_age,acceleration,variance
        self.min_hits,self.min_speed,self.min_displacement=min_hits,min_speed,min_displacement
        self.tracks=[]
        self.next_id=1
        self.stamp=None

    def update(self, stamp, points):
        if not math.isfinite(stamp):
            raise ValueError('Mover stamps must be finite')
        if self.stamp is not None and stamp <= self.stamp:
            self.tracks=[]  # Clock reset or duplicate stamp: start over.
        dt=0. if self.stamp is None or stamp <= self.stamp else stamp-self.stamp
        self.stamp=stamp
        self.tracks=[t for t in self.tracks if stamp-t.last_seen <= self.max_age]
        f=np.eye(4); f[0,2]=f[1,3]=dt
        g=np.array([[dt*dt/2,0],[0,dt*dt/2],[dt,0],[0,dt]])
        for t in self.tracks:
            t.state=f@t.state
            t.covariance=f@t.covariance@f.T+self.acceleration**2*g@g.T
        points=[np.asarray(p,dtype=float)[:2] for p in points]
        matched=set()
        if points and self.tracks:
            costs=np.array([[float(np.linalg.norm(t.state[:2]-p)) for p in points] for t in self.tracks])
            valid=costs <= self.gate
            rows,cols=linear_sum_assignment(np.where(valid,costs,(min(costs.shape)+1)*(self.gate+1)))
            r=np.eye(2)*self.variance
            for i,j in zip(rows,cols):
                if not valid[i,j]:continue
                t=self.tracks[i]
                gain=np.linalg.solve(t.covariance[:2,:2]+r,t.covariance[:,:2].T).T
                t.state=t.state+gain@(points[j]-t.state[:2])
                a=np.eye(4); a[:,:2]-=gain
                t.covariance=a@t.covariance@a.T+gain@r@gain.T
                t.last_seen=stamp; t.hits+=1
                matched.add(j)
        for j,p in enumerate(points):
            if j in matched:continue
            self.tracks.append(Mover(self.next_id,np.array([p[0],p[1],0.,0.]),
                                     np.diag([self.variance,self.variance,4.,4.]),stamp,p.copy()))
            self.next_id+=1
        return [t for t in self.tracks if t.last_seen == stamp and t.hits >= self.min_hits
                and math.hypot(*t.state[2:]) >= self.min_speed
                and float(np.linalg.norm(t.state[:2]-t.origin)) >= self.min_displacement]
