from __future__ import annotations

import numpy as np

from ridgeback_autonomy.frontier_explorer.navigator import (
    frontier_mask, get_frontier_clusters, is_frontier_point,
)
from ridgeback_autonomy.frontier_explorer.params import UNKNOWN, OBSTACLE, FREE


def _grid(rows):
    """Build an int8 grid from single-char rows: '.'=FREE '?'=UNKNOWN '#'=OBSTACLE."""
    lookup = {'.': FREE, '?': UNKNOWN, '#': OBSTACLE}
    return np.array([[lookup[c] for c in row] for row in rows], dtype=np.int8)


def _reference_clusters(robot_map):
    """Original per-cell flood-fill implementation, kept as the oracle."""
    from collections import deque
    visited = np.zeros_like(robot_map, dtype=bool)
    clusters = []
    for y in range(1, robot_map.shape[0] - 1):
        for x in range(1, robot_map.shape[1] - 1):
            if not visited[y, x] and is_frontier_point(x, y, robot_map):
                new_cluster = []
                queue = deque([(x, y)])
                visited[y, x] = True
                while queue:
                    curr_x, curr_y = queue.popleft()
                    new_cluster.append((curr_x, curr_y))
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            nx, ny = curr_x + dx, curr_y + dy
                            if (0 <= nx < robot_map.shape[1]
                                    and 0 <= ny < robot_map.shape[0]
                                    and not visited[ny, nx]
                                    and is_frontier_point(nx, ny, robot_map)):
                                visited[ny, nx] = True
                                queue.append((nx, ny))
                clusters.append(new_cluster)
    return clusters


def _as_cluster_sets(clusters):
    return {frozenset(c) for c in clusters}


def test_frontier_mask_free_next_to_unknown() -> None:
    grid = _grid([
        '?????',
        '?...?',
        '?.#.?',
        '?...?',
        '?????',
    ])
    mask = frontier_mask(grid)
    # every FREE cell borders the unknown ring except none — all are frontiers
    ys, xs = np.nonzero(mask)
    cells = set(zip(xs.tolist(), ys.tolist()))
    assert (2, 2) not in cells  # obstacle is not a frontier
    assert (1, 1) in cells and (3, 3) in cells
    for x, y in cells:
        assert grid[y, x] == FREE


def test_frontier_mask_interior_free_not_frontier() -> None:
    grid = _grid([
        '?????',
        '?...?',
        '?...?',
        '?...?',
        '?????',
    ])
    mask = frontier_mask(grid)
    assert not mask[2, 2]  # centre cell touches only FREE cells
    assert mask[1, 1] and mask[1, 3] and mask[3, 1]


def test_no_frontiers_returns_empty() -> None:
    assert get_frontier_clusters(_grid(['...', '...', '...'])) == []
    assert get_frontier_clusters(_grid(['???', '???', '???'])) == []
    assert get_frontier_clusters(_grid(['###', '#.#', '###'])) == []


def test_two_separated_clusters() -> None:
    # unknown on the left and right edges, obstacle wall down the middle
    grid = _grid([
        '??.#.??',
        '??.#.??',
        '??.#.??',
        '??.#.??',
        '??.#.??',
    ])
    clusters = get_frontier_clusters(grid)
    assert len(clusters) == 2
    sizes = sorted(len(c) for c in clusters)
    assert sizes == [5, 5]


def test_diagonal_cells_join_one_cluster() -> None:
    # two frontier cells touching only diagonally must merge (8-connectivity)
    grid = _grid([
        '?????',
        '?.#??',
        '?#.??',
        '?????',
        '?????',
    ])
    clusters = get_frontier_clusters(grid)
    assert len(clusters) == 1
    assert frozenset(clusters[0]) == frozenset({(1, 1), (2, 2)})


def test_matches_reference_implementation_on_random_grids() -> None:
    rng = np.random.default_rng(42)
    for _ in range(20):
        grid = rng.choice(
            np.array([UNKNOWN, FREE, OBSTACLE], dtype=np.int8),
            size=(24, 31), p=[0.35, 0.5, 0.15])
        got = _as_cluster_sets(get_frontier_clusters(grid))
        expected = _as_cluster_sets(_reference_clusters(grid))
        assert got == expected


def test_cluster_grows_onto_edge_from_interior_seed() -> None:
    # frontier line reaching the map edge: edge cells belong to the cluster
    # (seeded from the interior), matching the original semantics
    grid = _grid([
        '?.???',
        '?.???',
        '?.???',
        '?????',
    ])
    clusters = get_frontier_clusters(grid)
    assert len(clusters) == 1
    assert frozenset(clusters[0]) == frozenset({(1, 0), (1, 1), (1, 2)})
