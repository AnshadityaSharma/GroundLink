"""Lawnmower (boustrophedon) grid-coverage mission generation.

Given a boundary polygon (lat/lon) and a line spacing, produces a Mission
whose waypoints sweep the polygon's bounding area in parallel passes,
alternating direction each row, clipped to the polygon itself.
"""

from __future__ import annotations

import math

from shapely.geometry import LineString, Polygon

from mission_planner.geo import latlon_to_local_xy as _latlon_to_local_xy
from mission_planner.geo import local_xy_to_latlon as _local_xy_to_latlon
from mission_planner.waypoint import Mission, Waypoint, WaypointKind


def generate_lawnmower_mission(
    boundary_latlon: list[tuple[float, float]],
    spacing_m: float,
    altitude_m: float,
    speed_m_s: float | None = None,
    heading_deg: float = 0.0,
    mission_name: str = "grid_coverage",
) -> Mission:
    """Generate a lawnmower-pattern coverage mission over a boundary polygon.

    Args:
        boundary_latlon: polygon vertices as (lat_deg, lon_deg), in order.
            Does not need to be explicitly closed (first point repeated).
        spacing_m: distance between adjacent sweep lines, in meters.
        altitude_m: relative altitude for all waypoints.
        speed_m_s: optional cruise speed override for waypoints.
        heading_deg: sweep-line direction, degrees clockwise from north.
            0 = sweep lines run north-south (rows stacked east-west).
        mission_name: label stored on the returned Mission.

    Raises:
        ValueError: if boundary has fewer than 3 vertices or spacing <= 0.
    """
    if len(boundary_latlon) < 3:
        raise ValueError("boundary must have at least 3 vertices")
    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive")

    origin_lat, origin_lon = boundary_latlon[0]
    local_pts = [_latlon_to_local_xy(lat, lon, origin_lat, origin_lon) for lat, lon in boundary_latlon]
    polygon = Polygon(local_pts)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)  # repair minor self-touching issues

    minx, miny, maxx, maxy = polygon.bounds

    # Rotate the polygon so sweep lines are axis-aligned (simplifies clipping),
    # sweep in the rotated frame, then rotate waypoints back.
    theta = math.radians(heading_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    def to_rot(x, y):
        return x * cos_t + y * sin_t, -x * sin_t + y * cos_t

    def from_rot(x, y):
        return x * cos_t - y * sin_t, x * sin_t + y * cos_t

    rot_pts = [to_rot(x, y) for x, y in local_pts]
    rot_polygon = Polygon(rot_pts)
    r_minx, r_miny, r_maxx, r_maxy = rot_polygon.bounds

    span_x = r_maxx - r_minx
    num_lines = max(1, math.ceil(span_x / spacing_m) + 1)

    waypoints: list[Waypoint] = []
    going_up = True
    for i in range(num_lines):
        line_x = r_minx + i * spacing_m
        if line_x > r_maxx:
            break
        sweep_line = LineString([(line_x, r_miny - 1.0), (line_x, r_maxy + 1.0)])
        clipped = sweep_line.intersection(rot_polygon)
        if clipped.is_empty:
            continue

        segments = []
        if clipped.geom_type == "LineString":
            segments = [clipped]
        elif clipped.geom_type == "MultiLineString":
            segments = list(clipped.geoms)
        else:
            continue

        # order segments along the sweep direction, respecting alternating pass direction
        segments.sort(key=lambda s: s.coords[0][1], reverse=not going_up)

        for seg in segments:
            coords = list(seg.coords)
            if not going_up:
                coords = list(reversed(coords))
            for x_r, y_r in (coords[0], coords[-1]):
                x_local, y_local = from_rot(x_r, y_r)
                lat, lon = _local_xy_to_latlon(x_local, y_local, origin_lat, origin_lon)
                waypoints.append(
                    Waypoint(
                        latitude_deg=lat,
                        longitude_deg=lon,
                        relative_altitude_m=altitude_m,
                        kind=WaypointKind.NAV,
                        speed_m_s=speed_m_s,
                        label=f"row{i}",
                    )
                )
        going_up = not going_up

    return Mission(waypoints=waypoints, name=mission_name, metadata={"spacing_m": spacing_m, "heading_deg": heading_deg})
