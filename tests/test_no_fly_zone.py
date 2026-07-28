import math

import pytest

from firmware_link.telemetry import Position
from mission_planner.waypoint import Waypoint
from replanning_engine.no_fly_zone import NoFlyZone, blocks_remaining_path

_R = 6371000.0
_ORIGIN_LAT, _ORIGIN_LON = 47.397742, 8.545594


def _offset(lat0: float, lon0: float, dx_m: float, dy_m: float) -> tuple[float, float]:
    d_lat = (dy_m / _R) * (180.0 / math.pi)
    d_lon = (dx_m / (_R * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
    return lat0 + d_lat, lon0 + d_lon


def _position(dx_m: float, dy_m: float) -> Position:
    lat, lon = _offset(_ORIGIN_LAT, _ORIGIN_LON, dx_m, dy_m)
    return Position(latitude_deg=lat, longitude_deg=lon, absolute_altitude_m=488.0, relative_altitude_m=15.0)


def _waypoint(dx_m: float, dy_m: float) -> Waypoint:
    lat, lon = _offset(_ORIGIN_LAT, _ORIGIN_LON, dx_m, dy_m)
    return Waypoint(latitude_deg=lat, longitude_deg=lon, relative_altitude_m=15.0)


def _square_zone(center_dx: float, center_dy: float, half_side: float, label: str = "zone") -> NoFlyZone:
    corners_xy = [
        (center_dx - half_side, center_dy - half_side),
        (center_dx + half_side, center_dy - half_side),
        (center_dx + half_side, center_dy + half_side),
        (center_dx - half_side, center_dy + half_side),
    ]
    boundary = [_offset(_ORIGIN_LAT, _ORIGIN_LON, x, y) for x, y in corners_xy]
    return NoFlyZone(boundary_latlon=boundary, label=label, activated_at_unix_s=1000.0)


def test_rejects_degenerate_zone_boundary():
    zone = NoFlyZone(boundary_latlon=[(0, 0), (1, 1)], label="bad", activated_at_unix_s=0.0)
    with pytest.raises(ValueError):
        blocks_remaining_path(zone, _position(0, 0), [_waypoint(100, 0)], safety_margin_m=5.0)


def test_empty_remaining_waypoints_returns_empty():
    zone = _square_zone(50, 0, 10)
    assert blocks_remaining_path(zone, _position(0, 0), [], safety_margin_m=5.0) == []


def test_zone_far_from_path_blocks_nothing():
    # straight path along +x axis; zone is far off to the side
    current = _position(0, 0)
    waypoints = [_waypoint(100, 0), _waypoint(200, 0)]
    zone = _square_zone(50, 500, 10)  # 500m off the path
    assert blocks_remaining_path(zone, current, waypoints, safety_margin_m=5.0) == []


def test_zone_directly_on_first_leg_blocks_index_0():
    current = _position(0, 0)
    waypoints = [_waypoint(100, 0), _waypoint(200, 0)]
    zone = _square_zone(50, 0, 10)  # sits right on the current->wp0 leg
    blocked = blocks_remaining_path(zone, current, waypoints, safety_margin_m=5.0)
    assert blocked == [0]


def test_zone_on_second_leg_blocks_only_index_1():
    current = _position(0, 0)
    waypoints = [_waypoint(100, 0), _waypoint(200, 0)]
    zone = _square_zone(150, 0, 10)  # sits on the wp0->wp1 leg
    blocked = blocks_remaining_path(zone, current, waypoints, safety_margin_m=5.0)
    assert blocked == [1]


def test_zone_blocking_both_legs():
    # a wide zone straddling both legs
    current = _position(0, 0)
    waypoints = [_waypoint(100, 0), _waypoint(200, 0)]
    zone = _square_zone(150, 0, 80)
    blocked = blocks_remaining_path(zone, current, waypoints, safety_margin_m=5.0)
    assert blocked == [0, 1]


def test_safety_margin_extends_detection_beyond_raw_polygon():
    # zone polygon itself doesn't touch the path, but is within safety_margin_m
    current = _position(0, 0)
    waypoints = [_waypoint(100, 0)]
    zone = _square_zone(50, 20, 5)  # nearest edge ~15m off the path (20-5)
    assert blocks_remaining_path(zone, current, waypoints, safety_margin_m=1.0) == []
    assert blocks_remaining_path(zone, current, waypoints, safety_margin_m=20.0) == [0]


def test_path_that_does_not_pass_near_zone_at_all():
    current = _position(0, 0)
    waypoints = [_waypoint(0, 100)]  # path goes north, zone is to the east
    zone = _square_zone(200, 0, 10)
    assert blocks_remaining_path(zone, current, waypoints, safety_margin_m=5.0) == []
