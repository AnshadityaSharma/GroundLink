"""Generic occupancy-grid A* search.

No domain knowledge of drones, PX4, or geography here -- grid in, path out.
Domain-specific grid construction (from a no-fly zone + waypoints, in local
projected meters) lives in reroute.py. Kept generic and dependency-free so
it's trivially unit-testable with synthetic grids.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

Cell = tuple[int, int]

_NEIGHBORS: list[Cell] = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


@dataclass(frozen=True)
class OccupancyGrid:
    width: int
    height: int
    cell_size_m: float
    blocked: frozenset[Cell]

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_blocked(self, cell: Cell) -> bool:
        return cell in self.blocked


def _heuristic(a: Cell, b: Cell, cell_size_m: float) -> float:
    """Euclidean distance -- admissible and consistent for this cost model
    (diagonal step cost = sqrt(2) * cell_size, orthogonal = cell_size), so
    plain A* is optimal here; no need for a weighted/inadmissible variant."""
    return math.hypot(a[0] - b[0], a[1] - b[1]) * cell_size_m


def find_path(grid: OccupancyGrid, start: Cell, goal: Cell) -> list[Cell] | None:
    """Standard A*, 8-connected, with corner-cutting prevention: a diagonal
    move is disallowed if both orthogonal cells adjacent to it are blocked,
    so the path can't slip through the gap between two blocked cells that
    meet only at a corner.

    Returns None if start/goal is itself blocked or no path exists.
    """
    if not grid.in_bounds(start) or not grid.in_bounds(goal):
        raise ValueError(f"start {start} / goal {goal} out of grid bounds ({grid.width}x{grid.height})")
    if grid.is_blocked(start) or grid.is_blocked(goal):
        return None
    if start == goal:
        return [start]

    counter = 0  # tie-breaker so heapq never compares Cell tuples directly
    open_heap: list[tuple[float, int, Cell]] = [(0.0, counter, start)]
    came_from: dict[Cell, Cell] = {}
    g_score: dict[Cell, float] = {start: 0.0}
    closed: set[Cell] = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return _reconstruct_path(came_from, current)
        closed.add(current)

        cx, cy = current
        for dx, dy in _NEIGHBORS:
            neighbor = (cx + dx, cy + dy)
            if not grid.in_bounds(neighbor) or grid.is_blocked(neighbor) or neighbor in closed:
                continue
            if dx != 0 and dy != 0 and (grid.is_blocked((cx + dx, cy)) or grid.is_blocked((cx, cy + dy))):
                continue  # corner-cutting prevention

            step_cost = grid.cell_size_m * (math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0)
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                counter += 1
                heapq.heappush(open_heap, (tentative_g + _heuristic(neighbor, goal, grid.cell_size_m), counter, neighbor))

    return None


def _reconstruct_path(came_from: dict[Cell, Cell], current: Cell) -> list[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
