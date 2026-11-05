"""Who the robot locks onto: the nearest person standing in front of it."""

from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple

from ridgeback_leader_follow.tracks import Track


def nearest_in_front(candidates: Iterable[Tuple[Track, float, float]], max_range: float = 2.0,
                     half_angle: float = math.radians(30.0), distance_tolerance: float = 0.,
                     lateral_tolerance: float = 0.) -> Tuple[Optional[Track], str]:
    """Nearest in the front sector; optional F3 uncertainty ties use geometry.

    Within the configured range tolerance, prefer the central group, then the
    robot's right (negative Y), then distance/ID for deterministic equal poses.
    Zero tolerances retain the original strict nearest-person policy.
    """
    if not all(math.isfinite(v) and v >= 0 for v in (distance_tolerance,lateral_tolerance)):
        raise ValueError('Acquisition tolerances must be finite and nonnegative')
    placed=[]
    for track,x,y in candidates:
        distance=math.hypot(x,y)
        if not math.isfinite(distance) or x<=0 or distance>max_range or abs(math.atan2(y,x))>half_angle:
            continue
        placed.append((distance,x,y,track))
    if not placed:return None,"nobody_in_front"
    nearest=min(placed,key=lambda item:item[0])
    if distance_tolerance==0 and lateral_tolerance==0:
        return nearest[3],"nearest_in_front"
    nearby=[item for item in placed if item[0]<=nearest[0]+distance_tolerance]
    central=min(abs(item[2]) for item in nearby)
    nearby=[item for item in nearby if abs(item[2])<=central+lateral_tolerance]
    chosen=min(nearby,key=lambda item:(item[2],item[0],item[3].track_id))
    return chosen[3],"nearest_front_with_geometric_tie_break"
