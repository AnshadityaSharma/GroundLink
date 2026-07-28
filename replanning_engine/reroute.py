"""No-fly-zone reroute orchestration.

Pure Python -- no MAVSDK import anywhere in this file. Consumes/produces
mission_planner.Waypoint objects only, so it's fully unit-testable without
SITL running. See replanning_engine/DESIGN.md for the full write-up of the
approach (coarse occupancy grid + A*, chosen for explainability over a
visibility graph or anything more exotic).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from firmware_link.telemetry import Position
from mission_planner.geo import latlon_to_local_xy, local_xy_to_latlon
from mission_planner.waypoint import Waypoint, WaypointKind
from replanning_engine.grid_astar import Cell, OccupancyGrid, find_path
from replanning_engine.no_fly_zone import NoFlyZone, blocks_remaining_path

_MAX_GRID_CELLS = 40_000
_DEFAULT_CELL_SIZE_M = 10.0
_DEFAULT_PADDING_M = 50.0


@dataclass(frozen=True)
class RerouteResult:
    # Replacement for remaining_waypoints[:span_end+1] -- i.e. this PLUS
    # remaining_waypoints[span_end+1:] is the full new remaining mission.
    # Only populated when succeeded=True.
    new_leading_waypoints: list[Waypoint]
    span_end_index: int | None  # index into the ORIGINAL remaining_waypoints list
    succeeded: bool
    reason: str


def reroute_around_no_fly_zones(
    zones: list[NoFlyZone],
    current_position: Position,
    remaining_waypoints: list[Waypoint],
    safety_margin_m: float,
) -> RerouteResult:
    """Reroutes the minimal blocked span of remaining_waypoints around the
    given zones, leaving everything after the reconnection point untouched
    ("preserve as much of the original mission as possible", per context.md).

    On failure (start/goal inside a zone, or no safe path found), the caller
    should fall back to the battery-critical RTL response -- see DESIGN.md's
    "edge cases" section. This function never raises for that case; it
    returns succeeded=False with a `reason`.
    """
    if not remaining_waypoints or not zones:
        return RerouteResult(new_leading_waypoints=list(remaining_waypoints), span_end_index=None, succeeded=True, reason="nothing_to_reroute")

    blocked_indices: set[int] = set()
    for zone in zones:
        blocked_indices.update(blocks_remaining_path(zone, current_position, remaining_waypoints, safety_margin_m))

    if not blocked_indices:
        return RerouteResult(new_leading_waypoints=list(remaining_waypoints), span_end_index=None, succeeded=True, reason="no_intersection")

    span_end = max(blocked_indices)
    goal_wp = remaining_waypoints[span_end]
    altitude_m = goal_wp.relative_altitude_m

    origin_lat = current_position.latitude_deg
    origin_lon = current_position.longitude_deg
    start_xy = (0.0, 0.0)
    goal_xy = latlon_to_local_xy(goal_wp.latitude_deg, goal_wp.longitude_deg, origin_lat, origin_lon)

    blocked_region = _union_buffered_zones(zones, origin_lat, origin_lon, safety_margin_m)

    grid, to_cell, to_xy = _build_grid(start_xy, goal_xy, blocked_region)
    start_cell = to_cell(start_xy)
    goal_cell = to_cell(goal_xy)

    if grid.is_blocked(start_cell) or grid.is_blocked(goal_cell):
        return RerouteResult(new_leading_waypoints=[], span_end_index=None, succeeded=False, reason="start_or_goal_inside_zone")

    cell_path = find_path(grid, start_cell, goal_cell)
    if cell_path is None:
        return RerouteResult(new_leading_waypoints=[], span_end_index=None, succeeded=False, reason="no_safe_path_found")

    xy_path = [to_xy(c) for c in cell_path]
    simplified_xy = _simplify(xy_path, tolerance_m=grid.cell_size_m / 2.0)

    # drop both endpoints: the first duplicates current_position (not itself
    # a waypoint to fly to -- we're already there), the last duplicates
    # goal_wp (which remaining_waypoints[span_end:] already supplies).
    detour_xy = simplified_xy[1:-1]

    detour_waypoints = []
    for x, y in detour_xy:
        lat, lon = local_xy_to_latlon(x, y, origin_lat, origin_lon)
        detour_waypoints.append(
            Waypoint(
                latitude_deg=lat,
                longitude_deg=lon,
                relative_altitude_m=altitude_m,
                kind=WaypointKind.NAV,  # NEVER TAKEOFF here -- vehicle is already airborne (D8 lesson)
                label="reroute",
            )
        )

    return RerouteResult(new_leading_waypoints=detour_waypoints, span_end_index=span_end, succeeded=True, reason="rerouted")


def _union_buffered_zones(zones: list[NoFlyZone], origin_lat: float, origin_lon: float, safety_margin_m: float) -> Polygon:
    polygons = []
    for zone in zones:
        pts = [latlon_to_local_xy(lat, lon, origin_lat, origin_lon) for lat, lon in zone.boundary_latlon]
        poly = Polygon(pts)
        if not poly.is_valid:
            poly = poly.buffer(0)
        polygons.append(poly.buffer(safety_margin_m))
    return unary_union(polygons)


def _build_grid(start_xy: tuple[float, float], goal_xy: tuple[float, float], blocked_region: Polygon):
    zminx, zminy, zmaxx, zmaxy = blocked_region.bounds
    minx = min(start_xy[0], goal_xy[0], zminx) - _DEFAULT_PADDING_M
    maxx = max(start_xy[0], goal_xy[0], zmaxx) + _DEFAULT_PADDING_M
    miny = min(start_xy[1], goal_xy[1], zminy) - _DEFAULT_PADDING_M
    maxy = max(start_xy[1], goal_xy[1], zmaxy) + _DEFAULT_PADDING_M

    width_m = maxx - minx
    height_m = maxy - miny

    cell_size = _DEFAULT_CELL_SIZE_M
    est_cells = max(1.0, width_m / cell_size) * max(1.0, height_m / cell_size)
    if est_cells > _MAX_GRID_CELLS:
        cell_size *= math.sqrt(est_cells / _MAX_GRID_CELLS)

    grid_width = max(1, math.ceil(width_m / cell_size))
    grid_height = max(1, math.ceil(height_m / cell_size))

    blocked_cells = set()
    for gx in range(grid_width):
        for gy in range(grid_height):
            cx = minx + (gx + 0.5) * cell_size
            cy = miny + (gy + 0.5) * cell_size
            if blocked_region.contains(Point(cx, cy)):
                blocked_cells.add((gx, gy))

    grid = OccupancyGrid(width=grid_width, height=grid_height, cell_size_m=cell_size, blocked=frozenset(blocked_cells))

    def to_cell(xy: tuple[float, float]) -> Cell:
        x, y = xy
        gx = min(max(int((x - minx) / cell_size), 0), grid_width - 1)
        gy = min(max(int((y - miny) / cell_size), 0), grid_height - 1)
        return (gx, gy)

    def to_xy(cell: Cell) -> tuple[float, float]:
        gx, gy = cell
        return (minx + (gx + 0.5) * cell_size, miny + (gy + 0.5) * cell_size)

    return grid, to_cell, to_xy


def _perpendicular_distance_m(point: tuple[float, float], line_start: tuple[float, float], line_end: tuple[float, float]) -> float:
    x0, y0 = point
    x1, y1 = line_start
    x2, y2 = line_end
    if (x1, y1) == (x2, y2):
        return math.hypot(x0 - x1, y0 - y1)
    numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denominator = math.hypot(y2 - y1, x2 - x1)
    return numerator / denominator


def _simplify(points: list[tuple[float, float]], tolerance_m: float) -> list[tuple[float, float]]:
    """Greedy collinearity reduction: an intermediate point is dropped if it
    deviates from the line (last kept point -> next point) by less than
    tolerance_m. Turns a staircase-y grid path into a handful of waypoints.
    Not a full Douglas-Peucker -- deliberately simple and explainable, per
    DESIGN.md's "explainable in a report" requirement.
    """
    if len(points) <= 2:
        return list(points)

    simplified = [points[0]]
    anchor_index = 0
    for i in range(1, len(points) - 1):
        deviation = _perpendicular_distance_m(points[i], points[anchor_index], points[i + 1])
        if deviation > tolerance_m:
            simplified.append(points[i])
            anchor_index = i
    simplified.append(points[-1])
    return simplified
