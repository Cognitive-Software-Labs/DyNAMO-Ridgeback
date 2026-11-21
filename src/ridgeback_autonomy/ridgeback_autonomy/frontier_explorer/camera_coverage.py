"""Where the forward camera has usefully looked during the G1 search, and where to look next.

LiDAR frontiers say where the map is unknown; they say nothing about the camera, whose
view is 80 deg wide and useful for G1 only between about 1.3 and 3.5 m (``ViewModel``).
A cell counts as seen when a usable camera frame covered it: inside the view, within that
range band, with an unbroken line of sight on the SLAM map. Only map-known cells count;
a ray stops at the first occupied cell (which is itself seen: G1 is an obstacle in the
map) and at unknown or off-map cells.

``select_views`` picks at most ``max_views`` headings from a stop, greedily by the number
of visible cells not seen yet. ``rotation_problem`` refuses a turn whose swept footprint
(basket included) comes within a margin of an obstacle or crosses unknown space.
Pure numpy: no ROS, so the recorded runs can be replayed against it.
"""
from dataclasses import dataclass
import math

import numpy as np

from ridgeback_autonomy.mission.parking_geometry import normalize_angle, rotated_footprint_vertices
from ridgeback_autonomy.mission.parking_planner import points_to_polygon_distance


@dataclass(frozen=True)
class ViewModel:
    # Inside the 640x480 profile's 39.8 deg half-view, so G1 is wholly in the image.
    half_fov_rad: float = math.radians(30.0)
    # G1 is detected and localized from 1.3 m (its feet leave the image nearer) and is
    # relocalized out to 3.5 m: the mission's own reacquisition band.
    min_range_m: float = 1.3
    max_range_m: float = 3.5
    ray_step_rad: float = math.radians(1.0)
    occupied_threshold: int = 65


@dataclass(frozen=True)
class MapView:
    """The part of an OccupancyGrid the coverage needs; ``data`` is (height, width) int."""
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def shape(self):
        return self.data.shape

    def cell_of(self, x, y):
        return (np.floor((np.asarray(y) - self.origin_y) / self.resolution).astype(int),
                np.floor((np.asarray(x) - self.origin_x) / self.resolution).astype(int))


def map_view(msg):
    """MapView of a nav_msgs/OccupancyGrid, or None for a rotated or empty grid."""
    info = msg.info
    q = info.origin.orientation
    if info.width == 0 or info.height == 0 or abs(q.z) > 1e-6 or abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
        return None
    data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
    return MapView(data, float(info.resolution), float(info.origin.position.x),
                   float(info.origin.position.y))


def visible_cells(grid, model, x, y, yaw, half_fov_rad=None):
    """Flat indices of the cells a camera at (x, y) facing ``yaw`` sees within the range band."""
    half = model.half_fov_rad if half_fov_rad is None else half_fov_rad
    count = max(1, int(round(2 * half / model.ray_step_rad)) + 1)
    angles = yaw + np.linspace(-half, half, count)
    ranges = np.arange(0.0, model.max_range_m, grid.resolution / 2.0)
    px = x + np.cos(angles)[:, None] * ranges[None, :]
    py = y + np.sin(angles)[:, None] * ranges[None, :]
    rows, cols = grid.cell_of(px, py)
    h, w = grid.shape
    inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
    value = np.full(rows.shape, -1, dtype=np.int16)
    value[inside] = grid.data[rows[inside], cols[inside]]
    blocking = (value < 0) | (value >= model.occupied_threshold) | ~inside
    # A sample is seen when nothing blocked the ray before it; the first occupied cell is.
    blocked_before = (np.cumsum(blocking, axis=1) - blocking) > 0
    seen = (~blocked_before & inside & (value >= 0) & (ranges[None, :] >= model.min_range_m))
    return np.unique(rows[seen] * w + cols[seen])


class CameraCoverage:
    """Boolean 'seen by a usable camera frame' grid that follows the SLAM map's geometry."""

    def __init__(self, model=ViewModel()):
        self.model = model
        self.seen = None
        self._geometry = None

    def _align(self, grid):
        geometry = (grid.resolution, grid.origin_x, grid.origin_y, grid.shape)
        if geometry == self._geometry:
            return
        old, old_geometry = self.seen, self._geometry
        self.seen = np.zeros(grid.shape, dtype=bool)
        self._geometry = geometry
        if old is None or not math.isclose(old_geometry[0], grid.resolution):
            return
        # The map grows by whole cells: copy the overlap at its integer offset.
        dc = int(round((old_geometry[1] - grid.origin_x) / grid.resolution))
        dr = int(round((old_geometry[2] - grid.origin_y) / grid.resolution))
        oh, ow = old.shape
        h, w = grid.shape
        r0, c0 = max(0, dr), max(0, dc)
        r1, c1 = min(h, dr + oh), min(w, dc + ow)
        if r0 < r1 and c0 < c1:
            self.seen[r0:r1, c0:c1] = old[r0 - dr:r1 - dr, c0 - dc:c1 - dc]

    def mark(self, grid, x, y, yaw):
        """Record one usable frame from the camera pose; returns the newly seen cell count."""
        self._align(grid)
        cells = visible_cells(grid, self.model, x, y, yaw)
        flat = self.seen.reshape(-1)
        new = int(np.count_nonzero(~flat[cells]))
        flat[cells] = True
        return new

    def unseen(self, grid, x, y, yaw):
        self._align(grid)
        cells = visible_cells(grid, self.model, x, y, yaw)
        return cells[~self.seen.reshape(-1)[cells]]

    def select_views(self, grid, base, camera_offset, *, max_views=3, min_new_cells=100,
                     candidate_step_rad=math.radians(10.0), excluded=()):
        """[(heading, new cells)]: at most ``max_views`` base headings, each adding at least
        ``min_new_cells`` unseen visible cells to the ones before it, in turning order from
        the current heading. ``excluded`` headings (within the half view) are never chosen."""
        x, y, yaw = base
        ox, oy = camera_offset
        candidates = []
        for k in range(int(round(2 * math.pi / candidate_step_rad))):
            heading = normalize_angle(yaw + k * candidate_step_rad)
            if any(abs(normalize_angle(heading - e)) < self.model.half_fov_rad for e in excluded):
                continue
            cx = x + ox * math.cos(heading) - oy * math.sin(heading)
            cy = y + ox * math.sin(heading) + oy * math.cos(heading)
            candidates.append((heading, self.unseen(grid, cx, cy, heading)))
        chosen, covered = [], np.zeros(0, dtype=np.int64)
        while len(chosen) < max_views:
            best = None
            for heading, cells in candidates:
                gain = int(np.count_nonzero(~np.isin(cells, covered)))
                if gain >= min_new_cells and (best is None or gain > best[1]):
                    best = (heading, gain, cells)
            if best is None:
                break
            chosen.append((best[0], best[1]))
            covered = np.union1d(covered, best[2])
            candidates = [c for c in candidates if c[0] != best[0]]
        return turning_order(yaw, chosen)


def turning_order(yaw, views):
    """[(heading, signed turn from the previous heading, new cells)] with the least total
    rotation: one sweep to the left or the right, or one side first and then back across
    to the other. Each turn is signed, so a sweep never takes the short way back."""
    if not views:
        return []
    two_pi = 2 * math.pi
    # Each view as a cumulative rotation target from the current heading: counter-clockwise
    # in [0, 2 pi) or clockwise in (-2 pi, 0].
    ccw = [(normalize_angle(h - yaw) % two_pi, h, g) for h, g in views]
    as_ccw = lambda v: (v[0], v[1], v[2])
    as_cw = lambda v: (v[0] - two_pi if v[0] > 0 else 0.0, v[1], v[2])
    left = sorted((as_ccw(v) for v in ccw if v[0] <= math.pi), key=lambda v: v[0])
    right = sorted((as_cw(v) for v in ccw if v[0] > math.pi), key=lambda v: -v[0])
    candidates = [sorted(map(as_ccw, ccw), key=lambda v: v[0]),
                  sorted(map(as_cw, ccw), key=lambda v: -v[0]),
                  left + right, right + left]

    def cost(order):
        previous, total = 0.0, 0.0
        for target, _, _ in order:
            total, previous = total + abs(target - previous), target
        return total

    plan, previous = [], 0.0
    for target, heading, gain in min(candidates, key=cost):
        plan.append((heading, target - previous, gain))
        previous = target
    return plan


def rotation_problem(costmap, footprint, pose, turn_rad, *, margin_m=0.05, lethal=100,
                     step_rad=math.radians(5.0)):
    """Why turning in place by ``turn_rad`` from ``pose`` is not shown safe on the Nav2
    costmap, or None. Every sampled footprint along the turn must keep ``margin_m`` from
    lethal cells and lie on known cells (cells under the current footprint excepted)."""
    x, y, yaw = pose
    radius = max(math.hypot(px, py) for px, py in footprint) + margin_m + costmap.resolution
    r0, c0 = costmap.cell_of(x - radius, y - radius)
    r1, c1 = costmap.cell_of(x + radius, y + radius)
    h, w = costmap.shape
    if r0 < 0 or c0 < 0 or r1 >= h or c1 >= w:
        return 'turn area leaves the costmap'
    rows, cols = np.mgrid[r0:r1 + 1, c0:c1 + 1]
    value = costmap.data[rows, cols].ravel()
    cx = costmap.origin_x + (cols.ravel() + 0.5) * costmap.resolution
    cy = costmap.origin_y + (rows.ravel() + 0.5) * costmap.resolution
    lethal_cells = value >= lethal
    unknown_cells = value < 0
    if unknown_cells.any():
        here = points_to_polygon_distance(cx[unknown_cells], cy[unknown_cells],
                                          rotated_footprint_vertices(footprint, x, y, yaw))
        keep = np.flatnonzero(unknown_cells)[here > 0.0]
        unknown_cells = np.zeros_like(unknown_cells)
        unknown_cells[keep] = True
    steps = max(1, int(math.ceil(abs(turn_rad) / step_rad)))
    for k in range(steps + 1):
        polygon = rotated_footprint_vertices(footprint, x, y, yaw + turn_rad * k / steps)
        if lethal_cells.any():
            gap = points_to_polygon_distance(cx[lethal_cells], cy[lethal_cells], polygon)
            if gap.min() < margin_m:
                return (f'footprint within {margin_m:.2f} m of an obstacle at '
                        f'{math.degrees(turn_rad * k / steps):+.0f} deg of the turn')
        if unknown_cells.any():
            gap = points_to_polygon_distance(cx[unknown_cells], cy[unknown_cells], polygon)
            if (gap <= 0.0).any():
                return f'footprint over unknown space at {math.degrees(turn_rad * k / steps):+.0f} deg of the turn'
    return None


def camera_frontier_map(awareness, costmap, grid, seen, free, unknown, obstacle):
    """The explorer's costmap awareness recast for the camera, once LiDAR frontiers are gone:
    ``free`` where the costmap is free and the SLAM map cell was seen by a usable frame,
    ``unknown`` where it is free but never seen, ``obstacle`` everywhere else (occupied,
    unmapped, off the SLAM map). Its frontiers (``navigator.get_frontier_clusters``) are the
    edges of the camera's coverage inside known free space."""
    h, w = awareness.shape
    rows, cols = np.mgrid[0:h, 0:w]
    x = costmap.origin_x + (cols + 0.5) * costmap.resolution
    y = costmap.origin_y + (rows + 0.5) * costmap.resolution
    r, c = grid.cell_of(x, y)
    gh, gw = grid.shape
    inside = (r >= 0) & (r < gh) & (c >= 0) & (c < gw)
    rr, cc = np.where(inside, r, 0), np.where(inside, c, 0)
    value = np.where(inside, grid.data[rr, cc], -1)
    seen_here = inside & seen[rr, cc]
    known_free = inside & (value >= 0) & (value < 65) & (awareness == free)
    out = np.full((h, w), obstacle, dtype=awareness.dtype)
    out[known_free & seen_here] = free
    out[known_free & ~seen_here] = unknown
    return out
