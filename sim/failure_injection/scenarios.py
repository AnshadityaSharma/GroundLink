"""Concrete failure-injection scenario definitions for GroundLink's
evaluation (context.md: "baseline RTL-on-failure vs. adaptive replanning
under identical injected failures"). Grounded in real PX4 SITL fault
parameters and real MAVSDK calls -- verified by reading the actual
PX4-Autopilot source, not guessed. Each apply_*() function is a thin async
call using firmware_link.GroundLinkVehicle, the same pattern as everything
else in this codebase.

STATUS: definitions and apply_*() functions are written and import-checked,
but NOT yet run against live SITL -- that's this module's own next
verification step, same discipline as everything else (measured batch, not
one anecdotal run). Do not treat "the parameter exists and the call is
correctly formed" as equivalent to "confirmed this actually degrades the
vehicle's behavior as intended" until that's done.
"""

from __future__ import annotations

from dataclasses import dataclass

from firmware_link.mavsdk_client import GroundLinkVehicle
from mission_planner.geo import latlon_to_local_xy, local_xy_to_latlon
from mission_planner.waypoint import Mission
from replanning_engine.no_fly_zone import NoFlyZone


@dataclass(frozen=True)
class NoFlyZoneScenario:
    """A no-fly zone that "appears" partway through a mission.

    Applied at the application level -- feeding the NoFlyZone into
    ReplanningEngine.handle_no_fly_zone() once the vehicle has passed
    trigger_after_waypoint_index -- not via a PX4 SITL parameter. There's
    no PX4-native concept of a geofence appearing mid-flight; this is
    GroundLink's own input, same as a real operator/dashboard announcement
    would be.
    """

    zone: NoFlyZone
    trigger_after_waypoint_index: int


@dataclass(frozen=True)
class BatteryDrainScenario:
    """Injects low battery via PX4's real SITL battery simulation
    (src/modules/simulation/battery_simulator/battery_simulator_params.c,
    confirmed by reading it directly):

      SIM_BAT_MIN_PCT floors/sets the simulated battery percentage
        directly -- used by apply_battery_drain_scenario() below for an
        immediate, deterministic "battery is now at X%" injection, rather
        than waiting out a real-time drain.
      SIM_BAT_DRAIN is the drain INTERVAL in seconds (PX4 default 60;
        lower = faster drain) -- exposed here for a gradual-drain scenario
        variant; apply_battery_drain_scenario() only uses target_percent
        unless drain_interval_s is explicitly set to something other than
        the PX4 default.
    """

    target_percent: float
    drain_interval_s: float = 60.0


@dataclass(frozen=True)
class GpsDegradationScenario:
    """Injects degraded GPS via PX4's real SITL GPS sim parameter
    (src/modules/simulation/sensor_gps_sim/parameters.c, confirmed by
    reading it directly):

      SIM_GPS_USED sets the simulated number of satellites used (PX4
      default 10). Dropping it low enough degrades fix type/HDOP through
      PX4's own EKF2 estimator, not something GroundLink fakes at the
      telemetry level -- the degradation constraint_monitor observes is
      the real consequence of a real (simulated) sensor input change.
    """

    simulated_num_satellites: int


def make_no_fly_zone_scenario(
    mission: Mission,
    origin_lat_deg: float,
    origin_lon_deg: float,
    trigger_after_waypoint_index: int,
    width_m: float = 80.0,
    label: str = "injected_no_fly_zone",
) -> NoFlyZoneScenario:
    """Build a NoFlyZoneScenario whose zone actually straddles the given
    mission's path between trigger_after_waypoint_index and the next
    waypoint -- a hardcoded/global zone polygon wouldn't reliably intersect
    an arbitrary mission's route, so this generates one relative to it.

    origin_lat_deg/origin_lon_deg should be the vehicle's home/launch
    position -- the same projection origin used elsewhere in this codebase
    (mission_planner.geo), so the zone lines up correctly regardless of
    where in the world the mission actually is.
    """
    if trigger_after_waypoint_index + 1 >= len(mission.waypoints):
        raise ValueError("trigger_after_waypoint_index must leave at least one waypoint after it")

    a = mission.waypoints[trigger_after_waypoint_index]
    b = mission.waypoints[trigger_after_waypoint_index + 1]
    ax, ay = latlon_to_local_xy(a.latitude_deg, a.longitude_deg, origin_lat_deg, origin_lon_deg)
    bx, by = latlon_to_local_xy(b.latitude_deg, b.longitude_deg, origin_lat_deg, origin_lon_deg)

    mid_x, mid_y = (ax + bx) / 2.0, (ay + by) / 2.0
    dx, dy = (bx - ax), (by - ay)
    length = max(1e-6, (dx**2 + dy**2) ** 0.5)
    # perpendicular unit vector, to build a box straddling the direct path
    perp_x, perp_y = -dy / length, dx / length
    half_w = width_m / 2.0

    corners_xy = [
        (mid_x - dx * 0.3 + perp_x * half_w, mid_y - dy * 0.3 + perp_y * half_w),
        (mid_x + dx * 0.3 + perp_x * half_w, mid_y + dy * 0.3 + perp_y * half_w),
        (mid_x + dx * 0.3 - perp_x * half_w, mid_y + dy * 0.3 - perp_y * half_w),
        (mid_x - dx * 0.3 - perp_x * half_w, mid_y - dy * 0.3 - perp_y * half_w),
    ]
    boundary_latlon = [local_xy_to_latlon(x, y, origin_lat_deg, origin_lon_deg) for x, y in corners_xy]

    zone = NoFlyZone(boundary_latlon=boundary_latlon, label=label, activated_at_unix_s=0.0)
    return NoFlyZoneScenario(zone=zone, trigger_after_waypoint_index=trigger_after_waypoint_index)


async def apply_battery_drain_scenario(vehicle: GroundLinkVehicle, scenario: BatteryDrainScenario) -> None:
    await vehicle.set_param_float("SIM_BAT_MIN_PCT", scenario.target_percent)
    if scenario.drain_interval_s != 60.0:
        await vehicle.set_param_float("SIM_BAT_DRAIN", scenario.drain_interval_s)


async def apply_gps_degradation_scenario(vehicle: GroundLinkVehicle, scenario: GpsDegradationScenario) -> None:
    await vehicle.set_param_int("SIM_GPS_USED", scenario.simulated_num_satellites)


# Concrete scenarios for the evaluation section (context.md: "at least 2-3
# distinct failure scenarios"). Thresholds chosen to reliably cross
# constraint_monitor's default Thresholds (constraint_monitor/monitor.py):
# BATTERY_DRAIN_CRITICAL's 12% is below the default battery_critical_percent
# (15%); GPS_DEGRADATION_LOW_SATS's 3 satellites should push PX4 below a 3D
# fix (typically needs 4+), tripping GPS_FIX_DEGRADED, not just GPS_HDOP_HIGH.
BATTERY_DRAIN_CRITICAL = BatteryDrainScenario(target_percent=12.0)
GPS_DEGRADATION_LOW_SATS = GpsDegradationScenario(simulated_num_satellites=3)
