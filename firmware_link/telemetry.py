"""Vehicle-agnostic telemetry types.

Everything outside firmware_link (constraint_monitor, replanning_engine,
dashboard) depends only on these dataclasses, never on MAVSDK types directly.
That's the seam that lets the MAVLink layer change (or get swapped for a
replay source in tests) without touching planning/monitoring code.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class GpsFixType(IntEnum):
    NO_GPS = 0
    NO_FIX = 1
    FIX_2D = 2
    FIX_3D = 3
    DGPS = 4
    RTK_FLOAT = 5
    RTK_FIXED = 6


@dataclass(frozen=True)
class BatteryState:
    voltage_v: float
    remaining_percent: float  # 0-100


@dataclass(frozen=True)
class GpsState:
    fix_type: GpsFixType
    num_satellites: int
    hdop: float


@dataclass(frozen=True)
class Position:
    latitude_deg: float
    longitude_deg: float
    absolute_altitude_m: float
    relative_altitude_m: float


@dataclass(frozen=True)
class Attitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class TelemetrySnapshot:
    """One point-in-time telemetry sample, timestamped at capture."""

    timestamp_unix_s: float
    battery: BatteryState | None
    gps: GpsState | None
    position: Position | None
    attitude: Attitude | None
