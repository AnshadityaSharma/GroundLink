"""Battery-critical response.

Deliberately NOT full pathfinding, per project direction -- this trigger
exists to make the evaluation section complete (baseline vs. adaptive across
multiple scenario types), not to be architecturally impressive. Two-tier
threshold decision using PX4-native actions; the "shortest safe return path"
context.md asks for is PX4's own RTL flight-mode logic, not anything
computed here.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class BatteryAction(Enum):
    CONTINUE = "continue"
    RETURN_TO_LAUNCH = "return_to_launch"
    LAND_IMMEDIATELY = "land_immediately"


@dataclass(frozen=True)
class BatteryResponseThresholds:
    rtl_below_percent: float = 20.0
    land_immediately_below_percent: float = 8.0

    def __post_init__(self):
        if self.land_immediately_below_percent >= self.rtl_below_percent:
            raise ValueError("land_immediately_below_percent must be lower than rtl_below_percent")


def decide_battery_response(remaining_percent: float, thresholds: BatteryResponseThresholds) -> BatteryAction:
    """Pure decision function: battery% -> action. Below
    land_immediately_below_percent, RTL itself might not complete in time,
    so land right here rather than searching for a better landing spot
    (context.md's "nearest safe landing point", simplified per the agreed
    fidelity level for this trigger)."""
    if remaining_percent <= thresholds.land_immediately_below_percent:
        return BatteryAction.LAND_IMMEDIATELY
    if remaining_percent <= thresholds.rtl_below_percent:
        return BatteryAction.RETURN_TO_LAUNCH
    return BatteryAction.CONTINUE
