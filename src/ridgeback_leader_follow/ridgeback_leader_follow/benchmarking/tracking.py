"""Deterministic constant-velocity association baseline for replay plumbing."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass
class Track:
    identity: int
    stamp: float
    x: float
    y: float
    vx: float = 0.
    vy: float = 0.


class ConstantVelocityTracker:
    def __init__(self, gate=.75, max_age=.5):
        if not all(math.isfinite(v) and v > 0 for v in (gate,max_age)):
            raise ValueError('Tracker gate and maximum age must be positive and finite')
        self.gate,self.max_age = gate,max_age
        self.tracks = []
        self.next_id = 1
        self.last_stamp = -math.inf

    def update(self, stamp, points):
        if not math.isfinite(stamp) or stamp <= self.last_stamp:
            raise ValueError('Tracker timestamps must strictly increase')
        if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('Detections must be finite 2D points')
        self.last_stamp = stamp
        self.tracks = [t for t in self.tracks if stamp-t.stamp <= self.max_age]
        assigned = {}
        if self.tracks and points:
            costs = np.array([[math.hypot(t.x+t.vx*(stamp-t.stamp)-x,
                                          t.y+t.vy*(stamp-t.stamp)-y)
                               for x,y in points] for t in self.tracks])
            penalty = (min(costs.shape)+1)*(self.gate+1)
            rows,cols = linear_sum_assignment(np.where(costs <= self.gate,costs,penalty))
            for i,j in zip(rows,cols):
                if costs[i,j] > self.gate:
                    continue
                track = self.tracks[i]
                x,y = points[j]
                dt = stamp-track.stamp
                track.vx = .5*track.vx+.5*(x-track.x)/dt
                track.vy = .5*track.vy+.5*(y-track.y)/dt
                track.x,track.y,track.stamp = x,y,stamp
                assigned[int(j)] = track
        for index,(x,y) in enumerate(points):
            if index not in assigned:
                track = Track(self.next_id,stamp,x,y)
                self.next_id += 1
                self.tracks.append(track)
                assigned[index] = track
        # Retain unseen tracks for association, but never emit invented detections.
        return [{'track_id':assigned[i].identity,'x':p[0],'y':p[1]} for i,p in enumerate(points)]


def transform_points(points, translation, quaternion, world_from_odom):
    """Apply a TF sensor->odom transform, then an explicit surveyed world anchor."""
    tx,ty,tz = translation
    qx,qy,qz,qw = quaternion
    norm = math.sqrt(sum(v*v for v in quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid transform quaternion')
    qx,qy,qz,qw = [v/norm for v in quaternion]
    ox,oy,oz,yaw = world_from_odom
    if not all(math.isfinite(v) for v in (*translation,*world_from_odom)):
        raise ValueError('Non-finite transform')
    c,s = math.cos(yaw),math.sin(yaw)
    result = []
    for x,y in points:
        # Sensor points lie in the local scan plane (z=0); use the full TF
        # rotation so sensor roll/pitch is not silently discarded.
        px = (1-2*(qy*qy+qz*qz))*x + 2*(qx*qy-qz*qw)*y + tx
        py = 2*(qx*qy+qz*qw)*x + (1-2*(qx*qx+qz*qz))*y + ty
        result.append((ox+c*px-s*py,oy+s*px+c*py))
    return result
