import math

import pytest
from shapely.geometry import LineString, Polygon

from mission_planner.geo import latlon_to_local_xy
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from sim.failure_injection.scenarios import (
    BatteryDrainScenario,
    GpsDegradationScenario,
    make_no_fly_zone_scenario,
)

_R = 6371000.0
_ORIGIN_LAT, _ORIGIN_LON = 47.397742, 8.545594


def _offset(lat0, lon0, dx_m, dy_m):
    d_lat = (dy_m / _R) * (180.0 / math.pi)
    d_lon = (dx_m / (_R * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
    return lat0 + d_lat, lon0 + d_lon


def _wp(dx, dy, kind=WaypointKind.NAV):
    lat, lon = _offset(_ORIGIN_LAT, _ORIGIN_LON, dx, dy)
    return Waypoint(latitude_deg=lat, longitude_deg=lon, relative_altitude_m=15.0, kind=kind)


def _to_local(lat, lon):
    return latlon_to_local_xy(lat, lon, _ORIGIN_LAT, _ORIGIN_LON)


def test_generated_zone_intersects_the_targeted_leg():
    mission = Mission(
        waypoints=[
            _wp(0, 100, kind=WaypointKind.TAKEOFF),
            _wp(0, 200),
            _wp(0, 300),
        ]
    )
    scenario = make_no_fly_zone_scenario(mission, _ORIGIN_LAT, _ORIGIN_LON, trigger_after_waypoint_index=1, width_m=60.0)

    assert scenario.trigger_after_waypoint_index == 1
    a, b = mission.waypoints[1], mission.waypoints[2]
    segment = LineString([_to_local(a.latitude_deg, a.longitude_deg), _to_local(b.latitude_deg, b.longitude_deg)])
    zone_polygon = Polygon([_to_local(lat, lon) for lat, lon in scenario.zone.boundary_latlon])

    assert segment.intersects(zone_polygon)


def test_generated_zone_does_not_swallow_unrelated_legs():
    # a zone straddling leg 0->1 shouldn't also cover a leg far away (2->3)
    mission = Mission(
        waypoints=[
            _wp(0, 100, kind=WaypointKind.TAKEOFF),
            _wp(0, 200),
            _wp(0, 1000),
            _wp(0, 1100),
        ]
    )
    scenario = make_no_fly_zone_scenario(mission, _ORIGIN_LAT, _ORIGIN_LON, trigger_after_waypoint_index=0, width_m=60.0)

    c, d = mission.waypoints[2], mission.waypoints[3]
    far_segment = LineString([_to_local(c.latitude_deg, c.longitude_deg), _to_local(d.latitude_deg, d.longitude_deg)])
    zone_polygon = Polygon([_to_local(lat, lon) for lat, lon in scenario.zone.boundary_latlon])

    assert not far_segment.intersects(zone_polygon)


def test_rejects_trigger_index_leaving_no_waypoints_after():
    mission = Mission(waypoints=[_wp(0, 100, kind=WaypointKind.TAKEOFF), _wp(0, 200)])
    with pytest.raises(ValueError):
        make_no_fly_zone_scenario(mission, _ORIGIN_LAT, _ORIGIN_LON, trigger_after_waypoint_index=1)


def test_battery_drain_scenario_defaults():
    s = BatteryDrainScenario(target_percent=12.0)
    assert s.target_percent == 12.0
    assert s.drain_interval_s == 60.0


def test_gps_degradation_scenario_holds_satellite_count():
    s = GpsDegradationScenario(simulated_num_satellites=3)
    assert s.simulated_num_satellites == 3
