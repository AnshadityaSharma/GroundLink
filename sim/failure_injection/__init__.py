from sim.failure_injection.scenarios import (
    BATTERY_DRAIN_CRITICAL,
    GPS_DEGRADATION_LOW_SATS,
    BatteryDrainScenario,
    GpsDegradationScenario,
    NoFlyZoneScenario,
    apply_battery_drain_scenario,
    apply_gps_degradation_scenario,
    make_no_fly_zone_scenario,
)

__all__ = [
    "BATTERY_DRAIN_CRITICAL",
    "GPS_DEGRADATION_LOW_SATS",
    "BatteryDrainScenario",
    "GpsDegradationScenario",
    "NoFlyZoneScenario",
    "apply_battery_drain_scenario",
    "apply_gps_degradation_scenario",
    "make_no_fly_zone_scenario",
]
