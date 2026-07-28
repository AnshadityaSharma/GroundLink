import math

import pytest
from shapely.geometry import Point, Polygon

from mission_planner.grid_coverage import generate_lawnmower_mission

_R = 6371000.0


def _latlon_offset(lat0: float, lon0: float, dx_m: float, dy_m: float) -> tuple[float, float]:
    d_lat = (dy_m / _R) * (180.0 / math.pi)
    d_lon = (dx_m / (_R * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
    return lat0 + d_lat, lon0 + d_lon


def _latlon_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    d_lat = lat - lat0
    d_lon = lon - lon0
    y = d_lat * (math.pi / 180.0) * _R
    x = d_lon * (math.pi / 180.0) * _R * math.cos(math.radians(lat0))
    return x, y


_ORIGIN_LAT, _ORIGIN_LON = 47.0, 8.0


def _square_boundary(side_m: float) -> list[tuple[float, float]]:
    return [
        _latlon_offset(_ORIGIN_LAT, _ORIGIN_LON, 0, 0),
        _latlon_offset(_ORIGIN_LAT, _ORIGIN_LON, side_m, 0),
        _latlon_offset(_ORIGIN_LAT, _ORIGIN_LON, side_m, side_m),
        _latlon_offset(_ORIGIN_LAT, _ORIGIN_LON, 0, side_m),
    ]


def test_rejects_degenerate_boundary():
    with pytest.raises(ValueError):
        generate_lawnmower_mission([(0, 0), (1, 1)], spacing_m=10, altitude_m=10)


def test_rejects_nonpositive_spacing():
    with pytest.raises(ValueError):
        generate_lawnmower_mission(_square_boundary(100), spacing_m=0, altitude_m=10)


def test_produces_waypoints_for_simple_square():
    mission = generate_lawnmower_mission(_square_boundary(100), spacing_m=20, altitude_m=15)
    assert len(mission.waypoints) > 0
    assert all(wp.relative_altitude_m == 15 for wp in mission.waypoints)


def test_reasonable_waypoint_count():
    """For a 100m square with 20m spacing we expect ~6 sweep lines, 2 waypoints
    each (convex shape => one segment per line) -- allow slack for shapely
    edge-of-polygon clipping behavior, but it must stay in a sane range."""
    mission = generate_lawnmower_mission(_square_boundary(100), spacing_m=20, altitude_m=15)
    expected_lines = math.ceil(100 / 20) + 1
    # each line contributes at least 2 waypoints (entry/exit), rarely more
    # for a convex polygon; allow +/- 2 lines of slack for boundary clipping.
    assert 2 * (expected_lines - 2) <= len(mission.waypoints) <= 2 * (expected_lines + 2)


def test_no_missed_area_gap_never_exceeds_spacing():
    """Every point inside the polygon must be within spacing_m of some sweep
    line, i.e. consecutive sweep-line x-positions (in local meters) must
    never be farther apart than spacing_m."""
    side = 150.0
    spacing = 25.0
    mission = generate_lawnmower_mission(_square_boundary(side), spacing_m=spacing, altitude_m=10)

    xs = sorted(
        {round(_latlon_to_xy(wp.latitude_deg, wp.longitude_deg, _ORIGIN_LAT, _ORIGIN_LON)[0], 3) for wp in mission.waypoints}
    )
    assert len(xs) >= 2

    # first/last line must be within spacing of the polygon's edges (0 and side)
    assert xs[0] <= spacing + 1e-6
    assert xs[-1] >= side - spacing - 1e-6

    for a, b in zip(xs, xs[1:]):
        assert b - a <= spacing + 1e-6, f"gap {b - a} exceeds spacing {spacing}"


def test_all_waypoints_within_or_on_boundary():
    """Waypoints shouldn't land far outside the requested boundary polygon."""
    side = 120.0
    boundary = _square_boundary(side)
    polygon = Polygon([(lon, lat) for lat, lon in boundary])
    # small buffer to tolerate floating point / projection edge effects
    buffered = polygon.buffer(1e-6)

    mission = generate_lawnmower_mission(boundary, spacing_m=15, altitude_m=10)
    for wp in mission.waypoints:
        pt = Point(wp.longitude_deg, wp.latitude_deg)
        assert buffered.contains(pt) or buffered.touches(pt) or polygon.distance(pt) < 1.0


def test_alternating_pass_direction():
    """Lawnmower pattern should alternate sweep direction row to row so
    consecutive rows connect end-to-start (no long transit hops)."""
    mission = generate_lawnmower_mission(_square_boundary(100), spacing_m=25, altitude_m=10)
    rows: dict[str, list] = {}
    for wp in mission.waypoints:
        rows.setdefault(wp.label, []).append(wp)

    row_labels = sorted(rows.keys(), key=lambda label: int(label.replace("row", "")))
    assert len(row_labels) >= 2

    # y-coordinate of each row's first waypoint should alternate in sign of
    # (end - start) direction between consecutive rows
    directions = []
    for label in row_labels:
        pts = rows[label]
        if len(pts) < 2:
            continue
        _, y_start = _latlon_to_xy(pts[0].latitude_deg, pts[0].longitude_deg, _ORIGIN_LAT, _ORIGIN_LON)
        _, y_end = _latlon_to_xy(pts[-1].latitude_deg, pts[-1].longitude_deg, _ORIGIN_LAT, _ORIGIN_LON)
        directions.append(y_end - y_start)

    assert len(directions) >= 2
    for a, b in zip(directions, directions[1:]):
        assert (a > 0) != (b > 0), "consecutive rows should sweep in alternating directions"
