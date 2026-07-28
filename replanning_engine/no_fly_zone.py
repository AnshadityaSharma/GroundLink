"""No-fly-zone representation and path-intersection detection.

Detecting whether a newly-announced no-fly zone affects the current mission
is deliberately NOT modeled as a constraint_monitor-style threshold check
against a single telemetry snapshot -- it needs the *planned remaining
path*, which a TelemetrySnapshot doesn't carry. See replanning_engine/DESIGN.md
for why this lives here instead of in constraint_monitor.

All geometry work happens in local projected meters (via mission_planner.geo),
not raw lat/lon degrees -- buffering a polygon by a metric safety margin is
meaningless in degree-space (see DESIGN.md's note on this).
"""

from __future__ import annotations

from dataclasses import dataclass

from shapely.geometry import LineString, Polygon

from firmware_link.telemetry import Position
from mission_planner.geo import latlon_to_local_xy
from mission_planner.waypoint import Waypoint


@dataclass(frozen=True)
class NoFlyZone:
    boundary_latlon: list[tuple[float, float]]
    label: str
    activated_at_unix_s: float


def blocks_remaining_path(
    zone: NoFlyZone,
    current_position: Position,
    remaining_waypoints: list[Waypoint],
    safety_margin_m: float,
) -> list[int]:
    """Return indices into remaining_waypoints whose incoming leg intersects
    the (safety-margin-buffered) zone.

    Index i means the segment from (current_position if i==0 else
    remaining_waypoints[i-1]) to remaining_waypoints[i] is blocked. Empty
    list => the zone doesn't affect the current plan (no replan needed).
    """
    if len(zone.boundary_latlon) < 3:
        raise ValueError("no-fly zone boundary must have at least 3 vertices")
    if not remaining_waypoints:
        return []

    origin_lat = current_position.latitude_deg
    origin_lon = current_position.longitude_deg

    zone_xy = [latlon_to_local_xy(lat, lon, origin_lat, origin_lon) for lat, lon in zone.boundary_latlon]
    zone_polygon = Polygon(zone_xy)
    if not zone_polygon.is_valid:
        zone_polygon = zone_polygon.buffer(0)
    buffered_zone = zone_polygon.buffer(safety_margin_m)

    points_xy = [(0.0, 0.0)] + [
        latlon_to_local_xy(wp.latitude_deg, wp.longitude_deg, origin_lat, origin_lon) for wp in remaining_waypoints
    ]

    blocked_indices = []
    for i in range(len(remaining_waypoints)):
        segment = LineString([points_xy[i], points_xy[i + 1]])
        if segment.intersects(buffered_zone):
            blocked_indices.append(i)
    return blocked_indices
