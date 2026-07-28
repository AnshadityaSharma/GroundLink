"""Tests for replanning_engine.engine's ORCHESTRATION logic: does it call
the right GroundLinkVehicle methods in the right order, and does it build
correct ReplanEvents? This is NOT a test of whether the underlying MAVSDK
calls actually work against real PX4 -- that's the live-SITL verification
pass (see DESIGN.md / decisions.md). FakeVehicle below is a hand-written
stand-in implementing GroundLinkVehicle's own method surface, not a mock of
MAVSDK internals -- per DESIGN.md's stated testing philosophy.
"""

import math

import pytest

from firmware_link.telemetry import GpsFixType, Position
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from replanning_engine.battery_response import BatteryResponseThresholds
from replanning_engine.engine import EngineConfig, ReplanningEngine
from replanning_engine.gps_response import GpsResponseThresholds
from replanning_engine.no_fly_zone import NoFlyZone

_R = 6371000.0
_ORIGIN_LAT, _ORIGIN_LON = 47.397742, 8.545594


def _offset(lat0, lon0, dx_m, dy_m):
    d_lat = (dy_m / _R) * (180.0 / math.pi)
    d_lon = (dx_m / (_R * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
    return lat0 + d_lat, lon0 + d_lon


def _position(dx_m=0.0, dy_m=0.0) -> Position:
    lat, lon = _offset(_ORIGIN_LAT, _ORIGIN_LON, dx_m, dy_m)
    return Position(latitude_deg=lat, longitude_deg=lon, absolute_altitude_m=488.0, relative_altitude_m=15.0)


def _waypoint(dx_m, dy_m, alt=15.0, kind=WaypointKind.NAV) -> Waypoint:
    lat, lon = _offset(_ORIGIN_LAT, _ORIGIN_LON, dx_m, dy_m)
    return Waypoint(latitude_deg=lat, longitude_deg=lon, relative_altitude_m=alt, kind=kind)


def _rect_zone(x0, y0, x1, y1, label="zone") -> NoFlyZone:
    boundary = [_offset(_ORIGIN_LAT, _ORIGIN_LON, x, y) for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]]
    return NoFlyZone(boundary_latlon=boundary, label=label, activated_at_unix_s=1000.0)


class FakeVehicle:
    """Hand-written stand-in for GroundLinkVehicle's async method surface.
    Records every call for assertion; streams are pre-scripted so
    _wait_until_settled() can be driven deterministically in tests."""

    def __init__(self, flight_modes=("HOLD",), ground_speeds=(0.0,)):
        self.calls: list[tuple[str, tuple]] = []
        self._flight_modes = flight_modes
        self._ground_speeds = ground_speeds

    def _record(self, name, *args):
        self.calls.append((name, args))

    async def pause_mission(self):
        self._record("pause_mission")

    async def clear_mission(self):
        self._record("clear_mission")

    async def hold(self):
        self._record("hold")

    async def return_to_launch(self):
        self._record("return_to_launch")

    async def land(self):
        self._record("land")

    async def set_speed(self, speed_m_s):
        self._record("set_speed", speed_m_s)

    async def upload_mission(self, mission):
        self._record("upload_mission", mission)

    async def start_mission(self):
        self._record("start_mission")

    async def resume_mission_from(self, index):
        self._record("resume_mission_from", index)

    async def flight_mode_stream(self):
        for mode in self._flight_modes:
            yield mode

    async def ground_speed_stream(self):
        for speed in self._ground_speeds:
            yield speed


def _make_mission(waypoints):
    return Mission(waypoints=waypoints, name="test_mission")


# -- No-fly-zone trigger ------------------------------------------------------


@pytest.mark.asyncio
async def test_no_fly_zone_noop_when_not_intersecting():
    vehicle = FakeVehicle()
    engine = ReplanningEngine(vehicle, EngineConfig(no_fly_zone_safety_margin_m=5.0))
    engine.set_active_mission(_make_mission([_waypoint(100, 0)]))

    zone = _rect_zone(50, 500, 60, 510)  # far off the path
    event = await engine.handle_no_fly_zone(zone, _position(0, 0))

    assert event.outcome == "no_action"
    assert vehicle.calls == []  # no MAVSDK interaction for a non-blocking zone


@pytest.mark.asyncio
async def test_no_fly_zone_triggers_full_handoff_when_blocking():
    # settle after 3 consecutive HOLD+near-zero-speed samples (default
    # required_consecutive=3), then one extra sample so the merged stream
    # has enough entries to satisfy both mode and speed before the count
    vehicle = FakeVehicle(flight_modes=("HOLD", "HOLD", "HOLD", "HOLD"), ground_speeds=(0.0, 0.0, 0.0, 0.0))
    engine = ReplanningEngine(vehicle, EngineConfig(no_fly_zone_safety_margin_m=5.0, settle_required_consecutive=2))
    engine.set_active_mission(_make_mission([_waypoint(200, 0)]))

    zone = _rect_zone(80, -20, 120, 20)
    event = await engine.handle_no_fly_zone(zone, _position(0, 0))

    assert event.outcome == "rerouted"
    call_names = [name for name, _ in vehicle.calls]
    assert call_names == ["pause_mission", "clear_mission", "upload_mission", "start_mission"]

    uploaded_mission = vehicle.calls[2][1][0]
    assert all(wp.kind == WaypointKind.NAV for wp in uploaded_mission.waypoints)


@pytest.mark.asyncio
async def test_no_fly_zone_falls_back_to_rtl_when_no_safe_reroute():
    vehicle = FakeVehicle()
    engine = ReplanningEngine(vehicle, EngineConfig(no_fly_zone_safety_margin_m=5.0))
    # zone fully encloses the only remaining waypoint -> reroute fails
    engine.set_active_mission(_make_mission([_waypoint(100, 0)]))
    zone = _rect_zone(80, -30, 120, 30)

    event = await engine.handle_no_fly_zone(zone, _position(0, 0))

    assert event.outcome == "rtl_fallback"
    assert [name for name, _ in vehicle.calls] == ["return_to_launch"]


# -- Battery-critical trigger --------------------------------------------------


@pytest.mark.asyncio
async def test_battery_continue_takes_no_action():
    vehicle = FakeVehicle()
    engine = ReplanningEngine(vehicle)
    engine.set_active_mission(_make_mission([_waypoint(100, 0)]))

    event = await engine.handle_battery_critical(80.0)

    assert event.outcome == "no_action"
    assert vehicle.calls == []


@pytest.mark.asyncio
async def test_battery_rtl_between_thresholds():
    vehicle = FakeVehicle()
    engine = ReplanningEngine(vehicle, EngineConfig(battery_thresholds=BatteryResponseThresholds(rtl_below_percent=20, land_immediately_below_percent=8)))
    engine.set_active_mission(_make_mission([_waypoint(100, 0)]))

    event = await engine.handle_battery_critical(15.0)

    assert event.outcome == "rtl"
    assert [name for name, _ in vehicle.calls] == ["return_to_launch"]
    assert event.new_remaining_waypoints == []


@pytest.mark.asyncio
async def test_battery_land_immediately_below_lower_threshold():
    vehicle = FakeVehicle()
    engine = ReplanningEngine(vehicle, EngineConfig(battery_thresholds=BatteryResponseThresholds(rtl_below_percent=20, land_immediately_below_percent=8)))
    engine.set_active_mission(_make_mission([_waypoint(100, 0)]))

    event = await engine.handle_battery_critical(5.0)

    assert event.outcome == "land_immediately"
    assert [name for name, _ in vehicle.calls] == ["land"]


# -- GPS-degraded trigger -------------------------------------------------------


@pytest.mark.asyncio
async def test_gps_continue_normal_takes_no_action():
    vehicle = FakeVehicle()
    engine = ReplanningEngine(vehicle)
    engine.set_active_mission(_make_mission([_waypoint(100, 0)]))

    event = await engine.handle_gps_degraded(GpsFixType.FIX_3D, 1.0, nominal_speed_m_s=5.0)

    assert event.outcome == "no_action"
    assert vehicle.calls == []


@pytest.mark.asyncio
async def test_gps_slow_down_sets_reduced_speed():
    vehicle = FakeVehicle()
    thresholds = GpsResponseThresholds(min_fix_type_to_continue=GpsFixType.FIX_3D, max_hdop_for_normal_speed=2.5, slow_down_speed_fraction=0.5)
    engine = ReplanningEngine(vehicle, EngineConfig(gps_thresholds=thresholds))
    engine.set_active_mission(_make_mission([_waypoint(100, 0)]))

    event = await engine.handle_gps_degraded(GpsFixType.FIX_3D, 3.5, nominal_speed_m_s=10.0)

    assert event.outcome == "slowed_down"
    assert vehicle.calls == [("set_speed", (5.0,))]


@pytest.mark.asyncio
async def test_gps_hold_on_degraded_fix():
    vehicle = FakeVehicle()
    engine = ReplanningEngine(vehicle)
    engine.set_active_mission(_make_mission([_waypoint(100, 0)]))

    event = await engine.handle_gps_degraded(GpsFixType.NO_FIX, 1.0, nominal_speed_m_s=5.0)

    assert event.outcome == "hold"
    assert [name for name, _ in vehicle.calls] == ["hold"]


@pytest.mark.asyncio
async def test_resume_after_gps_recovery_calls_start_mission():
    vehicle = FakeVehicle()
    engine = ReplanningEngine(vehicle)
    engine.set_active_mission(_make_mission([_waypoint(100, 0)]))

    event = await engine.resume_after_gps_recovery()

    assert event.outcome == "resumed"
    assert [name for name, _ in vehicle.calls] == ["start_mission"]


# -- Remaining-waypoint tracking -------------------------------------------------


def test_remaining_waypoints_before_any_mission_set():
    vehicle = FakeVehicle()
    engine = ReplanningEngine(vehicle)
    assert engine.remaining_waypoints() == []


def test_remaining_waypoints_reflects_current_index():
    vehicle = FakeVehicle()
    engine = ReplanningEngine(vehicle)
    wps = [_waypoint(100, 0), _waypoint(200, 0), _waypoint(300, 0)]
    engine.set_active_mission(_make_mission(wps))
    engine._current_index = 1
    assert engine.remaining_waypoints() == wps[1:]


# -- Settle-timeout safety net --------------------------------------------------


@pytest.mark.asyncio
async def test_handoff_times_out_if_never_settles():
    # vehicle never reports HOLD -- stays in MISSION mode forever (stream
    # exhausts after a few samples, simulating "never settles")
    vehicle = FakeVehicle(flight_modes=("MISSION", "MISSION"), ground_speeds=(5.0, 5.0))
    engine = ReplanningEngine(vehicle, EngineConfig(settle_timeout_s=0.2, settle_required_consecutive=2))
    engine.set_active_mission(_make_mission([_waypoint(200, 0)]))

    zone = _rect_zone(80, -20, 120, 20)
    with pytest.raises(TimeoutError):
        await engine.handle_no_fly_zone(zone, _position(0, 0))
