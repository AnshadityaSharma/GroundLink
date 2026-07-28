import math

import pytest
from shapely.geometry import LineString, Polygon

from firmware_link.telemetry import Position
from mission_planner.geo import latlon_to_local_xy
from mission_planner.waypoint import Waypoint, WaypointKind
from replanning_engine.no_fly_zone import NoFlyZone
from replanning_engine.reroute import reroute_around_no_fly_zones

_R = 6371000.0
_ORIGIN_LAT, _ORIGIN_LON = 47.397742, 8.545594


def _offset(lat0: float, lon0: float, dx_m: float, dy_m: float) -> tuple[float, float]:
    d_lat = (dy_m / _R) * (180.0 / math.pi)
    d_lon = (dx_m / (_R * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
    return lat0 + d_lat, lon0 + d_lon


def _position(dx_m: float, dy_m: float) -> Position:
    lat, lon = _offset(_ORIGIN_LAT, _ORIGIN_LON, dx_m, dy_m)
    return Position(latitude_deg=lat, longitude_deg=lon, absolute_altitude_m=488.0, relative_altitude_m=15.0)


def _waypoint(dx_m: float, dy_m: float, alt: float = 15.0) -> Waypoint:
    lat, lon = _offset(_ORIGIN_LAT, _ORIGIN_LON, dx_m, dy_m)
    return Waypoint(latitude_deg=lat, longitude_deg=lon, relative_altitude_m=alt)


def _rect_zone(x0: float, y0: float, x1: float, y1: float, label: str = "zone") -> NoFlyZone:
    boundary = [_offset(_ORIGIN_LAT, _ORIGIN_LON, x, y) for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]]
    return NoFlyZone(boundary_latlon=boundary, label=label, activated_at_unix_s=1000.0)


def _to_local(position_or_waypoint) -> tuple[float, float]:
    return latlon_to_local_xy(position_or_waypoint.latitude_deg, position_or_waypoint.longitude_deg, _ORIGIN_LAT, _ORIGIN_LON)


def test_no_zones_is_a_noop():
    current = _position(0, 0)
    waypoints = [_waypoint(100, 0)]
    result = reroute_around_no_fly_zones([], current, waypoints, safety_margin_m=5.0)
    assert result.succeeded
    assert result.reason == "nothing_to_reroute"
    assert result.new_leading_waypoints == waypoints


def test_zone_not_intersecting_path_is_a_noop():
    current = _position(0, 0)
    waypoints = [_waypoint(100, 0)]
    zone = _rect_zone(50, 500, 60, 510)  # far off the path
    result = reroute_around_no_fly_zones([zone], current, waypoints, safety_margin_m=5.0)
    assert result.succeeded
    assert result.reason == "no_intersection"
    assert result.new_leading_waypoints == waypoints
    assert result.span_end_index is None


def test_reroutes_around_a_blocking_zone():
    current = _position(0, 0)
    waypoints = [_waypoint(200, 0)]
    # zone straddles the direct path, roughly midway
    zone = _rect_zone(80, -20, 120, 20)

    result = reroute_around_no_fly_zones([zone], current, waypoints, safety_margin_m=5.0)

    assert result.succeeded
    assert result.reason == "rerouted"
    assert result.span_end_index == 0
    assert len(result.new_leading_waypoints) > 0

    # every detour waypoint must be plain NAV, never TAKEOFF (D8 lesson --
    # this mission is mid-flight, a spurious takeoff item would repeat a
    # real, already-diagnosed bug)
    assert all(wp.kind == WaypointKind.NAV for wp in result.new_leading_waypoints)

    # the full detour path (current -> detour... -> original goal) must not
    # cross the buffered zone
    zone_poly = Polygon([_to_local_from_latlon(lat, lon) for lat, lon in zone.boundary_latlon]).buffer(5.0)
    full_path_xy = [_to_local(current)] + [_to_local(wp) for wp in result.new_leading_waypoints] + [_to_local(waypoints[0])]
    for a, b in zip(full_path_xy, full_path_xy[1:]):
        segment = LineString([a, b])
        assert not segment.intersects(zone_poly), f"detour segment {a}->{b} crosses the no-fly zone"


def _to_local_from_latlon(lat, lon):
    return latlon_to_local_xy(lat, lon, _ORIGIN_LAT, _ORIGIN_LON)


def test_reroute_waypoint_count_is_reasonable_not_one_per_grid_cell():
    current = _position(0, 0)
    waypoints = [_waypoint(300, 0)]
    zone = _rect_zone(100, -30, 200, 30)

    result = reroute_around_no_fly_zones([zone], current, waypoints, safety_margin_m=5.0)

    assert result.succeeded
    # a 300m detour around a ~130m-wide obstacle should need a handful of
    # waypoints, not dozens (which is what an unsimplified per-cell path
    # would produce at the default 10m grid resolution)
    assert len(result.new_leading_waypoints) <= 6


def test_preserves_untouched_waypoints_after_the_blocked_span():
    current = _position(0, 0)
    untouched_1 = _waypoint(300, 0)
    untouched_2 = _waypoint(400, 50)
    waypoints = [_waypoint(100, 0), untouched_1, untouched_2]
    zone = _rect_zone(30, -20, 70, 20)  # blocks only the first leg

    result = reroute_around_no_fly_zones([zone], current, waypoints, safety_margin_m=5.0)

    assert result.succeeded
    assert result.span_end_index == 0
    # everything after span_end must be byte-for-byte untouched
    remaining_after_splice = waypoints[result.span_end_index + 1 :]
    assert remaining_after_splice == [untouched_1, untouched_2]


def test_altitude_preserved_on_detour_waypoints():
    current = _position(0, 0)
    waypoints = [_waypoint(200, 0, alt=42.0)]
    zone = _rect_zone(80, -20, 120, 20)

    result = reroute_around_no_fly_zones([zone], current, waypoints, safety_margin_m=5.0)

    assert result.succeeded
    assert all(wp.relative_altitude_m == 42.0 for wp in result.new_leading_waypoints)


def test_multiple_overlapping_zones_are_unioned():
    current = _position(0, 0)
    waypoints = [_waypoint(300, 0)]
    zone_a = _rect_zone(80, -30, 150, 30, label="a")
    zone_b = _rect_zone(140, -30, 220, 30, label="b")  # overlaps zone_a

    result = reroute_around_no_fly_zones([zone_a, zone_b], current, waypoints, safety_margin_m=5.0)

    assert result.succeeded
    assert result.reason == "rerouted"
    combined = Polygon(
        [_to_local_from_latlon(lat, lon) for lat, lon in zone_a.boundary_latlon]
    ).buffer(5.0).union(
        Polygon([_to_local_from_latlon(lat, lon) for lat, lon in zone_b.boundary_latlon]).buffer(5.0)
    )
    full_path_xy = [_to_local(current)] + [_to_local(wp) for wp in result.new_leading_waypoints] + [_to_local(waypoints[0])]
    for a, b in zip(full_path_xy, full_path_xy[1:]):
        assert not LineString([a, b]).intersects(combined)


def test_goal_inside_zone_fails_cleanly():
    current = _position(0, 0)
    waypoints = [_waypoint(100, 0)]
    zone = _rect_zone(80, -30, 120, 30)  # fully encloses the goal at (100,0)

    result = reroute_around_no_fly_zones([zone], current, waypoints, safety_margin_m=5.0)

    assert not result.succeeded
    assert result.reason == "start_or_goal_inside_zone"


def test_fully_enclosed_goal_with_no_gap_reports_no_safe_path():
    # A hollow rectangular frame (union of 4 thick wall segments) fully
    # enclosing the goal, with the start well outside. The goal cell itself
    # is NOT inside any single zone polygon (it's in the hollow interior),
    # so this exercises the A*-finds-nothing path distinctly from the
    # start_or_goal_inside_zone short-circuit above.
    current = _position(0, 0)
    goal = _waypoint(100, 0)
    waypoints = [goal]

    wall_thickness = 20.0  # thick relative to the 10m default grid cell so
                            # rasterization can't leave an accidental gap
    interior_half = 10.0
    cx, cy = 100.0, 0.0

    south = _rect_zone(cx - interior_half - wall_thickness, cy - interior_half - wall_thickness, cx + interior_half + wall_thickness, cy - interior_half, "south")
    north = _rect_zone(cx - interior_half - wall_thickness, cy + interior_half, cx + interior_half + wall_thickness, cy + interior_half + wall_thickness, "north")
    west = _rect_zone(cx - interior_half - wall_thickness, cy - interior_half, cx - interior_half, cy + interior_half, "west")
    east = _rect_zone(cx + interior_half, cy - interior_half, cx + interior_half + wall_thickness, cy + interior_half, "east")

    result = reroute_around_no_fly_zones([south, north, west, east], current, waypoints, safety_margin_m=1.0)

    assert not result.succeeded
    assert result.reason == "no_safe_path_found"


def test_empty_remaining_waypoints_is_a_noop():
    current = _position(0, 0)
    zone = _rect_zone(50, -10, 60, 10)
    result = reroute_around_no_fly_zones([zone], current, [], safety_margin_m=5.0)
    assert result.succeeded
    assert result.reason == "nothing_to_reroute"
    assert result.new_leading_waypoints == []
