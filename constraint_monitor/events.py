"""Structured violation events emitted by ConstraintMonitor.

Deliberately not print statements — replanning_engine (and the dashboard's
event log) consume these as data, so they need a stable shape.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class ViolationKind(Enum):
    BATTERY_LOW = "battery_low"
    BATTERY_CRITICAL = "battery_critical"
    GPS_FIX_DEGRADED = "gps_fix_degraded"
    GPS_HDOP_HIGH = "gps_hdop_high"
    GEOFENCE_BREACH = "geofence_breach"


class Severity(Enum):
    WARNING = "warning"
    CRITICAL = "critical"


@dataclass(frozen=True)
class ViolationEvent:
    timestamp_unix_s: float
    kind: ViolationKind
    severity: Severity
    message: str
    # Raw value(s) that triggered the violation, e.g. {"remaining_percent": 12.3}
    # kept as a dict rather than typed fields since each ViolationKind carries
    # different data — replanning_engine pattern-matches on `kind`.
    details: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "timestamp_unix_s": self.timestamp_unix_s,
            "kind": self.kind.value,
            "severity": self.severity.value,
            "message": self.message,
            "details": self.details,
        }
