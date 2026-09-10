import numpy as np
from collections import deque
from ridgeback_autonomy.frontier_explorer.params import UNKNOWN, FREE


def frontier_mask(robot_map):
    """Boolean mask of frontier cells: FREE cells 4-adjacent to an UNKNOWN cell.

    Vectorized replacement for per-cell is_frontier_point() — the node calls
    frontier detection every planner tick on the full global costmap, so this
    must not be a Python loop.
    """
    free = robot_map == FREE
    unknown = robot_map == UNKNOWN

    unknown_adjacent = np.zeros_like(free)
    unknown_adjacent[:-1, :] |= unknown[1:, :]   # neighbor below (y+1)
    unknown_adjacent[1:, :] |= unknown[:-1, :]   # neighbor above (y-1)
    unknown_adjacent[:, :-1] |= unknown[:, 1:]   # neighbor right (x+1)
    unknown_adjacent[:, 1:] |= unknown[:, :-1]   # neighbor left (x-1)

    return free & unknown_adjacent


def is_frontier_point(x, y, robot_map):
    """Checks if a point is a valid frontier (A FREE cell next to an UNKNOWN cell)."""
    if y < 0 or y >= robot_map.shape[0] or x < 0 or x >= robot_map.shape[1]:
        return False
    if robot_map[y, x] != FREE:
        return False
    # Check the 4 direct neighbors (Up, Down, Left, Right)
    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < robot_map.shape[1] and 0 <= ny < robot_map.shape[0]:
            if robot_map[ny, nx] == UNKNOWN:
                return True
    return False


def get_frontier_clusters(robot_map):
    """Group adjacent frontier points into clusters (8-connectivity flood fill).

    Returns a list of clusters, each a list of (x, y) tuples. Cluster seeds are
    interior cells only, but a cluster may grow onto the map edge — same
    semantics as the original per-cell implementation.
    """
    mask = frontier_mask(robot_map)

    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return []
    # BFS over just the frontier cells (usually a few hundred, not the grid)
    frontier_cells = set(zip(xs.tolist(), ys.tolist()))

    h, w = robot_map.shape
    visited = set()
    clusters = []

    for x, y in sorted(frontier_cells, key=lambda p: (p[1], p[0])):
        if (x, y) in visited:
            continue
        if not (1 <= x < w - 1 and 1 <= y < h - 1):
            continue  # seeds stay interior; growth below may reach the edge
        cluster = []
        queue = deque([(x, y)])
        visited.add((x, y))
        while queue:
            cx, cy = queue.popleft()
            cluster.append((cx, cy))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (cx + dx, cy + dy)
                    if n in frontier_cells and n not in visited:
                        visited.add(n)
                        queue.append(n)
        clusters.append(cluster)
    return clusters
