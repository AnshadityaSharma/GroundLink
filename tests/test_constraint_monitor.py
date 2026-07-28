import math
from pathlib import Path

import pytest

from constraint_monitor.events import Severity, ViolationEvent, ViolationKind
from constraint_monitor.log_replay import load_snapshots_jsonl
from constraint_monitor.monitor import ConstraintMonitor, Thresholds
from firmware_link.telemetry import BatteryState, GpsFixType, GpsState, Position, TelemetrySnapshot

_FIXTURE = Path(__file__).parent / "fixtures" / "sample_telemetry.jsonl"

_R = 6371000.0
_ORIGIN_LAT, _ORIGIN_LON = 47.397742, 8.545594


def _latlon_offset(lat0, lon0, dx_m, dy_m):
    d_lat = (dy_m / _R) * (180.0 / math.pi)
    d_lon = (dx_m / (_R * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
    return lat0 + d_lat, lon0 + d_lon


def _geofence_box(side_m: float) -> list[tuple[float, float]]:
    half = side_m / 2
    return [
        _latlon_offset(_ORIGIN_LAT, _ORIGIN_LON, -half, -half),
        _latlon_offset(_ORIGIN_LAT, _ORIGIN_LON, half, -half),
        _latlon_offset(_ORIGIN_LAT, _ORIGIN_LON, half, half),
        _latlon_offset(_ORIGIN_LAT, _ORIGIN_LON, -half, half),
    ]


def _snapshot(**overrides) -> TelemetrySnapshot:
    defaults = dict(
        timestamp_unix_s=100.0,
        battery=BatteryState(voltage_v=16.5, remaining_percent=80.0),
        gps=GpsState(fix_type=GpsFixType.FIX_3D, num_satellites=12, hdop=1.0),
        position=Position(
            latitude_deg=_ORIGIN_LAT, longitude_deg=_ORIGIN_LON, absolute_altitude_m=488.0, relative_altitude_m=15.0
        ),
        attitude=None,
    )
    defaults.update(overrides)
    return TelemetrySnapshot(**defaults)


def test_thresholds_reject_inverted_battery_levels():
    with pytest.raises(ValueError):
        Thresholds(battery_warning_percent=10, battery_critical_percent=20)


def test_nominal_snapshot_produces_no_violations():
    monitor = ConstraintMonitor(Thresholds())
    events = monitor.check(_snapshot())
    assert events == []


def test_battery_warning():
    monitor = ConstraintMonitor(Thresholds(battery_warning_percent=30, battery_critical_percent=15))
    events = monitor.check(_snapshot(battery=BatteryState(voltage_v=15.0, remaining_percent=25.0)))
    assert len(events) == 1
    assert events[0].kind == ViolationKind.BATTERY_LOW
    assert events[0].severity == Severity.WARNING
    assert events[0].details["remaining_percent"] == 25.0


def test_battery_critical_takes_precedence_over_warning():
    monitor = ConstraintMonitor(Thresholds(battery_warning_percent=30, battery_critical_percent=15))
    events = monitor.check(_snapshot(battery=BatteryState(voltage_v=13.9, remaining_percent=8.0)))
    kinds = [e.kind for e in events]
    assert ViolationKind.BATTERY_CRITICAL in kinds
    assert ViolationKind.BATTERY_LOW not in kinds
    assert all(e.severity == Severity.CRITICAL for e in events if e.kind == ViolationKind.BATTERY_CRITICAL)


def test_gps_fix_degraded():
    monitor = ConstraintMonitor(Thresholds(min_gps_fix_type=GpsFixType.FIX_3D))
    events = monitor.check(_snapshot(gps=GpsState(fix_type=GpsFixType.FIX_2D, num_satellites=5, hdop=1.0)))
    kinds = [e.kind for e in events]
    assert ViolationKind.GPS_FIX_DEGRADED in kinds


def test_gps_hdop_high():
    monitor = ConstraintMonitor(Thresholds(max_hdop=2.0))
    events = monitor.check(_snapshot(gps=GpsState(fix_type=GpsFixType.FIX_3D, num_satellites=10, hdop=3.5)))
    kinds = [e.kind for e in events]
    assert ViolationKind.GPS_HDOP_HIGH in kinds


def test_geofence_breach_detected_outside_boundary():
    geofence = _geofence_box(side_m=100)
    monitor = ConstraintMonitor(Thresholds(geofence_latlon=geofence))
    far_position = Position(
        latitude_deg=_ORIGIN_LAT + 0.01,  # ~1.1km north, well outside a 100m box
        longitude_deg=_ORIGIN_LON,
        absolute_altitude_m=488.0,
        relative_altitude_m=15.0,
    )
    events = monitor.check(_snapshot(position=far_position))
    kinds = [e.kind for e in events]
    assert ViolationKind.GEOFENCE_BREACH in kinds
    assert events[[e.kind for e in events].index(ViolationKind.GEOFENCE_BREACH)].severity == Severity.CRITICAL


def test_geofence_no_breach_when_inside_boundary():
    geofence = _geofence_box(side_m=200)
    monitor = ConstraintMonitor(Thresholds(geofence_latlon=geofence))
    events = monitor.check(_snapshot())  # exactly at origin, well inside a 200m box
    assert ViolationKind.GEOFENCE_BREACH not in [e.kind for e in events]


def test_events_are_structured_not_strings():
    monitor = ConstraintMonitor(Thresholds(battery_warning_percent=30, battery_critical_percent=15))
    events = monitor.check(_snapshot(battery=BatteryState(voltage_v=13.5, remaining_percent=5.0)))
    assert all(isinstance(e, ViolationEvent) for e in events)
    d = events[0].to_dict()
    assert set(d.keys()) == {"timestamp_unix_s", "kind", "severity", "message", "details"}
    assert isinstance(d["kind"], str)  # enum serialized to plain value, log/dashboard friendly


def test_replay_from_logged_telemetry_detects_expected_violations():
    """End-to-end: load a replayed flight log and confirm the monitor flags
    the battery drain / GPS degradation / geofence excursion baked into the
    fixture (see tests/fixtures/sample_telemetry.jsonl)."""
    geofence = _geofence_box(side_m=300)
    monitor = ConstraintMonitor(
        Thresholds(
            battery_warning_percent=30,
            battery_critical_percent=15,
            min_gps_fix_type=GpsFixType.FIX_3D,
            max_hdop=2.5,
            geofence_latlon=geofence,
        )
    )

    snapshots = list(load_snapshots_jsonl(_FIXTURE))
    assert len(snapshots) == 5

    all_events = list(monitor.replay(snapshots))
    kinds_seen = {e.kind for e in all_events}

    assert ViolationKind.BATTERY_LOW in kinds_seen
    assert ViolationKind.BATTERY_CRITICAL in kinds_seen
    assert ViolationKind.GPS_HDOP_HIGH in kinds_seen
    assert ViolationKind.GPS_FIX_DEGRADED in kinds_seen
    assert ViolationKind.GEOFENCE_BREACH in kinds_seen

    # events must stay time-ordered since replanning_engine will consume this stream in order
    timestamps = [e.timestamp_unix_s for e in all_events]
    assert timestamps == sorted(timestamps)
