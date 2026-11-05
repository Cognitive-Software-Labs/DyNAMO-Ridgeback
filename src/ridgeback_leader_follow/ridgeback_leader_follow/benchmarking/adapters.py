"""Truth-free F2 adapter boundary and the planned scan-clustering baseline."""
from dataclasses import dataclass
import math
from typing import Protocol


@dataclass(frozen=True)
class ScanInput:
    stamp: float
    frame: str
    angle_min: float
    angle_increment: float
    ranges: tuple[float, ...]
    range_min: float
    range_max: float


class ScanDetector(Protocol):
    def detect(self, scan: ScanInput) -> list[tuple[float,float]]: ...


class ScanClustering:
    """Contiguous scan clusters; centroids are candidates, not confirmed people.

    Returns sensor-frame metres. Invalid beams split clusters; the first and
    last beams are never joined across the scan's blind sector. Neither world
    transforms nor truth annotations are available to this detector.
    """
    def __init__(self, split_distance=.15, minimum_points=3, maximum_width=.8):
        if split_distance <= 0 or minimum_points < 1 or maximum_width <= 0:
            raise ValueError('Invalid clustering parameters')
        self.split_distance = split_distance
        self.minimum_points = minimum_points
        self.maximum_width = maximum_width

    def detect(self, scan):
        groups = []
        current = []
        for index,distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
                if current: groups.append(current)
                current = []
                continue
            angle = scan.angle_min + index*scan.angle_increment
            point = (distance*math.cos(angle),distance*math.sin(angle))
            if current and math.dist(point,current[-1]) > self.split_distance:
                groups.append(current)
                current = []
            current.append(point)
        if current: groups.append(current)
        return [(sum(p[0] for p in group)/len(group),sum(p[1] for p in group)/len(group))
                for group in groups if len(group) >= self.minimum_points
                and math.dist(group[0],group[-1]) <= self.maximum_width
                and max(math.dist(a,b) for a in group for b in group) <= self.maximum_width]
