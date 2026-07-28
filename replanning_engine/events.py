"""Structured replanning-decision log entries.

This is the "log of replanning events with the reason for each decision"
context.md asks the dashboard to show -- designed alongside the decision
logic itself so engine.py emits it from the start rather than bolting
logging on after the fact. Mirrors constraint_monitor.events.ViolationEvent's
shape/conventions deliberately, for consistency across the codebase.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from mission_planner.waypoint import Waypoint


class ReplanTrigger(Enum):
    NO_FLY_ZONE = "no_fly_zone"
    BATTERY_CRITICAL = "battery_critical"
    GPS_DEGRADED = "gps_degraded"


@dataclass(frozen=True)
class ReplanEvent:
    timestamp_unix_s: float
    trigger: ReplanTrigger
    reason: str
    outcome: str  # e.g. "rerouted" | "rtl" | "hold" | "no_safe_reroute_found"
    old_remaining_waypoints: list[Waypoint] = field(default_factory=list)
    new_remaining_waypoints: list[Waypoint] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "timestamp_unix_s": self.timestamp_unix_s,
            "trigger": self.trigger.value,
            "reason": self.reason,
            "outcome": self.outcome,
            "old_remaining_waypoint_count": len(self.old_remaining_waypoints),
            "new_remaining_waypoint_count": len(self.new_remaining_waypoints),
        }
