"""GPS-degraded response.

Deliberately NOT full pathfinding, per project direction. Two verified
MAVSDK levers only: slower (action.set_current_speed) or stopped
(action.hold). There is no MAVSDK Action method to command a switch into
ALTCTL/POSCTL/MANUAL as an autonomous "conservative navigation mode" --
those PX4 modes expect continuous RC/manual input, which an unpiloted
mission doesn't have. This directly narrows a phrase in context.md
("switches to a more conservative navigation mode") to what's actually
achievable -- see replanning_engine/DESIGN.md's "Verified against MAVSDK"
section for the introspection that established this.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from firmware_link.telemetry import GpsFixType


class GpsAction(Enum):
    CONTINUE_NORMAL = "continue_normal"
    SLOW_DOWN = "slow_down"
    HOLD = "hold"


@dataclass(frozen=True)
class GpsResponseThresholds:
    min_fix_type_to_continue: GpsFixType = GpsFixType.FIX_3D
    max_hdop_for_normal_speed: float = 2.5
    slow_down_speed_fraction: float = 0.5


def decide_gps_response(fix_type: GpsFixType, hdop: float, thresholds: GpsResponseThresholds) -> GpsAction:
    """Pure decision function: (fix_type, hdop) -> action.

    Below min_fix_type_to_continue, the position estimate itself is
    untrustworthy, not just imprecise -- hold in place rather than keep
    navigating via waypoints. Above that fix quality but with high HDOP,
    slow down rather than stop, since the position is still usable, just
    less precise.
    """
    if fix_type < thresholds.min_fix_type_to_continue:
        return GpsAction.HOLD
    if hdop > thresholds.max_hdop_for_normal_speed:
        return GpsAction.SLOW_DOWN
    return GpsAction.CONTINUE_NORMAL
