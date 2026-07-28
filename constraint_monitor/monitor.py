"""Threshold-based constraint checking against telemetry.

ConstraintMonitor.check() is a pure function of (thresholds, snapshot) -> events,
so it works identically whether the snapshot came from a live MAVSDK stream or
a replayed log file — the two entry points below (watch / replay) are thin
adapters over the same core logic.
"""

from __future__ import annotations

from collections.abc import AsyncIterator, Iterable, Iterator

from shapely.geometry import Point, Polygon

from constraint_monitor.events import Severity, ViolationEvent, ViolationKind
from firmware_link.telemetry import GpsFixType, TelemetrySnapshot


class Thresholds:
    def __init__(
        self,
        battery_warning_percent: float = 30.0,
        battery_critical_percent: float = 15.0,
        min_gps_fix_type: GpsFixType = GpsFixType.FIX_3D,
        max_hdop: float = 2.5,
        geofence_latlon: list[tuple[float, float]] | None = None,
    ):
        if battery_critical_percent >= battery_warning_percent:
            raise ValueError("battery_critical_percent must be lower than battery_warning_percent")
        self.battery_warning_percent = battery_warning_percent
        self.battery_critical_percent = battery_critical_percent
        self.min_gps_fix_type = min_gps_fix_type
        self.max_hdop = max_hdop
        self.geofence_polygon: Polygon | None = (
            Polygon([(lon, lat) for lat, lon in geofence_latlon]) if geofence_latlon else None
        )


class ConstraintMonitor:
    def __init__(self, thresholds: Thresholds):
        self.thresholds = thresholds

    def check(self, snapshot: TelemetrySnapshot) -> list[ViolationEvent]:
        events: list[ViolationEvent] = []
        t = snapshot.timestamp_unix_s

        if snapshot.battery is not None:
            pct = snapshot.battery.remaining_percent
            if pct <= self.thresholds.battery_critical_percent:
                events.append(
                    ViolationEvent(
                        timestamp_unix_s=t,
                        kind=ViolationKind.BATTERY_CRITICAL,
                        severity=Severity.CRITICAL,
                        message=f"Battery critical: {pct:.1f}% remaining (threshold {self.thresholds.battery_critical_percent}%)",
                        details={"remaining_percent": pct},
                    )
                )
            elif pct <= self.thresholds.battery_warning_percent:
                events.append(
                    ViolationEvent(
                        timestamp_unix_s=t,
                        kind=ViolationKind.BATTERY_LOW,
                        severity=Severity.WARNING,
                        message=f"Battery low: {pct:.1f}% remaining (threshold {self.thresholds.battery_warning_percent}%)",
                        details={"remaining_percent": pct},
                    )
                )

        if snapshot.gps is not None:
            gps = snapshot.gps
            if gps.fix_type < self.thresholds.min_gps_fix_type:
                events.append(
                    ViolationEvent(
                        timestamp_unix_s=t,
                        kind=ViolationKind.GPS_FIX_DEGRADED,
                        severity=Severity.CRITICAL,
                        message=f"GPS fix degraded: {gps.fix_type.name} (need >= {self.thresholds.min_gps_fix_type.name})",
                        details={"fix_type": gps.fix_type.name, "num_satellites": gps.num_satellites},
                    )
                )
            if gps.hdop > self.thresholds.max_hdop:
                events.append(
                    ViolationEvent(
                        timestamp_unix_s=t,
                        kind=ViolationKind.GPS_HDOP_HIGH,
                        severity=Severity.WARNING,
                        message=f"GPS HDOP high: {gps.hdop:.2f} (threshold {self.thresholds.max_hdop})",
                        details={"hdop": gps.hdop},
                    )
                )

        if snapshot.position is not None and self.thresholds.geofence_polygon is not None:
            pos = snapshot.position
            point = Point(pos.longitude_deg, pos.latitude_deg)
            if not self.thresholds.geofence_polygon.contains(point):
                events.append(
                    ViolationEvent(
                        timestamp_unix_s=t,
                        kind=ViolationKind.GEOFENCE_BREACH,
                        severity=Severity.CRITICAL,
                        message=f"Geofence breach at ({pos.latitude_deg:.6f}, {pos.longitude_deg:.6f})",
                        details={"latitude_deg": pos.latitude_deg, "longitude_deg": pos.longitude_deg},
                    )
                )

        return events

    def replay(self, snapshots: Iterable[TelemetrySnapshot]) -> Iterator[ViolationEvent]:
        """Run checks over a replayed (e.g. logged) sequence of snapshots."""
        for snapshot in snapshots:
            yield from self.check(snapshot)

    async def watch(self, telemetry_stream: AsyncIterator[TelemetrySnapshot]) -> AsyncIterator[ViolationEvent]:
        """Run checks over a live async telemetry stream."""
        async for snapshot in telemetry_stream:
            for event in self.check(snapshot):
                yield event
