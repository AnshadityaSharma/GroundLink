"""Firmware-agnostic mission representation.

This is the shared vocabulary between mission_planner, replanning_engine,
and firmware_link. Nothing in here knows about MAVLink/MAVSDK — the
firmware_link layer is responsible for translating a Mission into whatever
the flight stack's upload API wants.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto


class WaypointKind(Enum):
    NAV = auto()  # normal fly-to waypoint
    TAKEOFF = auto()
    LAND = auto()
    RTL = auto()  # return-to-launch


@dataclass(frozen=True)
class Waypoint:
    latitude_deg: float
    longitude_deg: float
    relative_altitude_m: float
    kind: WaypointKind = WaypointKind.NAV
    acceptance_radius_m: float = 5.0
    speed_m_s: float | None = None  # None = use vehicle default
    priority: int = 0  # higher = more valuable; used by replanning_engine later
    label: str | None = None  # human-readable id, e.g. "row3_col5"


@dataclass
class Mission:
    waypoints: list[Waypoint]
    name: str = "unnamed_mission"
    metadata: dict = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.waypoints)

    def __iter__(self):
        return iter(self.waypoints)
