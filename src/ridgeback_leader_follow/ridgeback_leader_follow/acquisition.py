"""Who the robot locks onto: the nearest person standing in front of it."""

from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple

from ridgeback_leader_follow.tracks import Track


def nearest_in_front(candidates: Iterable[Tuple[Track, float, float]], max_range: float = 2.0,
                     half_angle: float = math.radians(30.0)) -> Tuple[Optional[Track], str]:
    """Pick the closest ``(track, x, y)`` inside the front sector; x forward, y left, metres in base_link."""
    best = None
    for track, x, y in candidates:
        distance = math.hypot(x, y)
        if x <= 0.0 or distance > max_range or abs(math.atan2(y, x)) > half_angle:
            continue
        if best is None or distance < best[0]:
            best = (distance, track)
    return (best[1], "nearest_in_front") if best else (None, "nobody_in_front")
