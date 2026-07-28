import itertools
import math

import pytest

from replanning_engine.grid_astar import OccupancyGrid, find_path


def _empty_grid(width: int, height: int, cell_size_m: float = 1.0) -> OccupancyGrid:
    return OccupancyGrid(width=width, height=height, cell_size_m=cell_size_m, blocked=frozenset())


def _grid_with_blocked(width: int, height: int, blocked: set, cell_size_m: float = 1.0) -> OccupancyGrid:
    return OccupancyGrid(width=width, height=height, cell_size_m=cell_size_m, blocked=frozenset(blocked))


def _path_cost(path: list, cell_size_m: float) -> float:
    cost = 0.0
    for (x1, y1), (x2, y2) in zip(path, path[1:]):
        cost += cell_size_m * (math.sqrt(2.0) if x1 != x2 and y1 != y2 else 1.0)
    return cost


def _brute_force_shortest_cost(grid: OccupancyGrid, start, goal) -> float | None:
    """Dijkstra via networkx-free BFS-with-priority over the same 8-connected
    graph (independent reimplementation, not calling find_path), for
    correctness cross-checking on small grids."""
    import heapq as hq

    dist = {start: 0.0}
    pq = [(0.0, start)]
    neighbors_deltas = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    while pq:
        d, cur = hq.heappop(pq)
        if cur == goal:
            return d
        if d > dist.get(cur, math.inf):
            continue
        cx, cy = cur
        for dx, dy in neighbors_deltas:
            nb = (cx + dx, cy + dy)
            if not grid.in_bounds(nb) or grid.is_blocked(nb):
                continue
            if dx != 0 and dy != 0 and (grid.is_blocked((cx + dx, cy)) or grid.is_blocked((cx, cy + dy))):
                continue
            step = grid.cell_size_m * (math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0)
            nd = d + step
            if nd < dist.get(nb, math.inf):
                dist[nb] = nd
                hq.heappush(pq, (nd, nb))
    return dist.get(goal)


def test_straight_line_on_empty_grid():
    grid = _empty_grid(10, 10)
    path = find_path(grid, (0, 0), (9, 0))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (9, 0)
    # no obstacles -> optimal path is the direct orthogonal line, cost == 9
    assert _path_cost(path, grid.cell_size_m) == pytest.approx(9.0)


def test_diagonal_shortcut_used_when_available():
    grid = _empty_grid(10, 10)
    path = find_path(grid, (0, 0), (5, 5))
    cost = _path_cost(path, grid.cell_size_m)
    # diagonal distance should be used (5*sqrt(2)), not the orthogonal
    # zig-zag/L-shape distance (10)
    assert cost == pytest.approx(5 * math.sqrt(2.0))


def test_start_equals_goal():
    grid = _empty_grid(5, 5)
    path = find_path(grid, (2, 2), (2, 2))
    assert path == [(2, 2)]


def test_blocked_start_or_goal_returns_none():
    grid = _grid_with_blocked(5, 5, {(0, 0)})
    assert find_path(grid, (0, 0), (4, 4)) is None
    grid2 = _grid_with_blocked(5, 5, {(4, 4)})
    assert find_path(grid2, (0, 0), (4, 4)) is None


def test_out_of_bounds_raises():
    grid = _empty_grid(5, 5)
    with pytest.raises(ValueError):
        find_path(grid, (-1, 0), (4, 4))
    with pytest.raises(ValueError):
        find_path(grid, (0, 0), (5, 5))


def test_no_path_when_goal_fully_enclosed():
    # wall of blocked cells surrounding (5,5) on all 8 sides
    blocked = {(x, y) for x in range(4, 7) for y in range(4, 7) if (x, y) != (5, 5)}
    grid = _grid_with_blocked(10, 10, blocked)
    assert find_path(grid, (0, 0), (5, 5)) is None


def test_path_detours_around_a_wall():
    # vertical wall from y=0..7 at x=5, leaving a gap at y=8,9 -- path from
    # left side to right side must detour through the gap.
    blocked = {(5, y) for y in range(0, 8)}
    grid = _grid_with_blocked(10, 10, blocked)
    path = find_path(grid, (0, 0), (9, 0))
    assert path is not None
    assert all(cell not in grid.blocked for cell in path)
    # must pass through the gap (x=5, y in {8,9}) to cross the wall
    assert any(x == 5 and y >= 8 for x, y in path)


def test_corner_cutting_prevented():
    # Two blocked cells touching only at a corner: (2,1) and (1,2), leaving a
    # diagonal "gap" between (1,1) and (2,2). Placed away from the grid
    # boundary (unlike a corner of the grid itself) so a detour actually
    # exists. A naive 8-connected A* without corner-cutting prevention would
    # cut straight through that gap; ours must route around it instead.
    blocked = {(2, 1), (1, 2)}
    grid = _grid_with_blocked(6, 6, blocked)
    path = find_path(grid, (1, 1), (2, 2))
    assert path is not None
    # the direct corner-cut (length 2: [(1,1),(2,2)]) must NOT be the result
    assert path != [(1, 1), (2, 2)]
    assert all(cell not in grid.blocked for cell in path)
    for (x1, y1), (x2, y2) in zip(path, path[1:]):
        if x1 != x2 and y1 != y2:
            # every diagonal step actually taken must have at least one of
            # its two orthogonal "corner" cells free
            assert not (grid.is_blocked((x2, y1)) and grid.is_blocked((x1, y2)))


def test_matches_independent_dijkstra_cost_on_random_small_grids():
    import random

    rng = random.Random(42)
    for trial in range(15):
        width, height = 8, 8
        blocked = {
            (x, y)
            for x, y in itertools.product(range(width), range(height))
            if rng.random() < 0.25
        }
        start, goal = (0, 0), (width - 1, height - 1)
        blocked.discard(start)
        blocked.discard(goal)
        grid = _grid_with_blocked(width, height, blocked, cell_size_m=2.5)

        expected_cost = _brute_force_shortest_cost(grid, start, goal)
        path = find_path(grid, start, goal)

        if expected_cost is None:
            assert path is None, f"trial {trial}: expected no path, A* found one"
        else:
            assert path is not None, f"trial {trial}: expected a path, A* found none"
            assert _path_cost(path, grid.cell_size_m) == pytest.approx(expected_cost), f"trial {trial}: cost mismatch"


def test_path_never_passes_through_blocked_cells_on_random_grids():
    import random

    rng = random.Random(7)
    for _ in range(15):
        width, height = 10, 10
        blocked = {
            (x, y)
            for x, y in itertools.product(range(width), range(height))
            if rng.random() < 0.3
        }
        start, goal = (0, 0), (width - 1, height - 1)
        blocked.discard(start)
        blocked.discard(goal)
        grid = _grid_with_blocked(width, height, blocked)

        path = find_path(grid, start, goal)
        if path is not None:
            assert all(cell not in grid.blocked for cell in path)
            # path must be contiguous 8-connected steps
            for (x1, y1), (x2, y2) in zip(path, path[1:]):
                assert abs(x1 - x2) <= 1 and abs(y1 - y2) <= 1
