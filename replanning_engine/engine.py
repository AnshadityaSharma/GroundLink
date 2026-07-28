"""MAVSDK-facing replanning handoff: pause -> confirm settled -> clear ->
upload -> resume.

This is the ONLY file in replanning_engine that imports firmware_link (and,
transitively, touches MAVSDK) -- everything else in the package is pure
Python (see DESIGN.md).

STATUS (see decisions.md D11/D14 for the full account): orchestration logic
is unit-tested with a hand-written vehicle stand-in (tests/test_engine.py).
Run against live SITL, a real bug was found and fixed (MAVSDK's
start_mission() can report success while PX4's internal mode-switch is
silently rejected -- _start_mission_and_confirm_resumed below now actively
verifies and retries). CONFIRMED RELIABLE via a 5-trial measured batch:
5/5 PASS, each reaching the final waypoint around 27s with 233-240 distinct
positions (real continuous flight), consistent across trials (D14). An
earlier batch measurement showed 0/5 with silent hangs; that was diagnosed
as a test-harness artifact (SITL backgrounded in the same script/session as
the Python client), NOT a flaw in this module -- see D14 for the full
diagnosis. Any new SITL trial script for this module must launch SITL as
an independent process invocation, never backgrounded inside the same
script/session that runs the MAVSDK client.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field

from firmware_link.mavsdk_client import GroundLinkVehicle
from firmware_link.telemetry import GpsFixType, Position
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from replanning_engine.battery_response import BatteryAction, BatteryResponseThresholds, decide_battery_response
from replanning_engine.events import ReplanEvent, ReplanTrigger
from replanning_engine.gps_response import GpsAction, GpsResponseThresholds, decide_gps_response
from replanning_engine.no_fly_zone import NoFlyZone
from replanning_engine.reroute import reroute_around_no_fly_zones


@dataclass
class EngineConfig:
    no_fly_zone_safety_margin_m: float = 5.0  # caller should pass the
    # mission's own acceptance_radius_m by default, per DESIGN.md's answer
    # to open question 3 -- not hardcoded here since it's mission-specific.
    battery_thresholds: BatteryResponseThresholds = field(default_factory=BatteryResponseThresholds)
    gps_thresholds: GpsResponseThresholds = field(default_factory=GpsResponseThresholds)
    settle_timeout_s: float = 15.0
    settle_speed_epsilon_m_s: float = 0.3
    settle_required_consecutive: int = 3


class ReplanningEngine:
    """Owns the currently-active Mission and executes replans against a
    GroundLinkVehicle. Does not itself watch ConstraintMonitor's event
    stream or NoFlyZone announcements -- callers (engine.py's future
    consumer, e.g. a demo script or the dashboard backend) drive it by
    calling handle_*() when a violation/announcement fires."""

    def __init__(self, vehicle: GroundLinkVehicle, config: EngineConfig | None = None):
        self.vehicle = vehicle
        self.config = config or EngineConfig()
        self.current_mission: Mission | None = None
        self._current_index: int = 0

    def set_active_mission(self, mission: Mission) -> None:
        """Call once, right after the initial ground-start mission is
        uploaded -- this is the source of truth remaining-waypoint tracking
        is built from (see DESIGN.md: PX4 doesn't hand back our own
        Waypoint metadata, only raw MissionItems)."""
        self.current_mission = mission
        self._current_index = 0

    async def track_mission_progress(self) -> None:
        """Run as a background task (asyncio.create_task) alongside
        telemetry/constraint monitoring, to keep remaining-waypoint tracking
        in sync with what PX4 reports."""
        async for current, _total in self.vehicle.mission_progress_stream():
            self._current_index = current

    def remaining_waypoints(self) -> list[Waypoint]:
        if self.current_mission is None:
            return []
        return self.current_mission.waypoints[self._current_index :]

    # -- Trigger 1: no-fly zone --------------------------------------------

    async def handle_no_fly_zone(self, zone: NoFlyZone, current_position: Position) -> ReplanEvent:
        remaining = self.remaining_waypoints()
        result = reroute_around_no_fly_zones([zone], current_position, remaining, self.config.no_fly_zone_safety_margin_m)

        if result.reason in ("nothing_to_reroute", "no_intersection"):
            return self._event(
                ReplanTrigger.NO_FLY_ZONE,
                f"zone '{zone.label}' does not intersect the remaining path",
                "no_action",
                remaining,
                remaining,
            )

        if not result.succeeded:
            # no safe reroute exists -- fall back to RTL rather than getting
            # stuck (DESIGN.md's edge-case handling)
            await self.vehicle.return_to_launch()
            return self._event(
                ReplanTrigger.NO_FLY_ZONE,
                f"zone '{zone.label}' blocks remaining path, no safe reroute found ({result.reason})",
                "rtl_fallback",
                remaining,
                [],
            )

        new_remaining = result.new_leading_waypoints + remaining[result.span_end_index + 1 :]
        await self._execute_handoff(new_remaining)
        return self._event(
            ReplanTrigger.NO_FLY_ZONE,
            f"zone '{zone.label}' blocks remaining path, rerouted around it",
            "rerouted",
            remaining,
            new_remaining,
        )

    # -- Trigger 2: battery-critical (simple, not pathfinding) --------------

    async def handle_battery_critical(self, remaining_percent: float) -> ReplanEvent:
        action = decide_battery_response(remaining_percent, self.config.battery_thresholds)
        remaining = self.remaining_waypoints()

        if action == BatteryAction.CONTINUE:
            return self._event(
                ReplanTrigger.BATTERY_CRITICAL,
                f"battery {remaining_percent:.1f}% above thresholds",
                "no_action",
                remaining,
                remaining,
            )

        if action == BatteryAction.RETURN_TO_LAUNCH:
            await self.vehicle.return_to_launch()
            outcome = "rtl"
        else:
            await self.vehicle.land()
            outcome = "land_immediately"

        return self._event(ReplanTrigger.BATTERY_CRITICAL, f"battery {remaining_percent:.1f}%", outcome, remaining, [])

    # -- Trigger 3: GPS-degraded (simple, not pathfinding) -------------------

    async def handle_gps_degraded(self, fix_type: GpsFixType, hdop: float, nominal_speed_m_s: float) -> ReplanEvent:
        action = decide_gps_response(fix_type, hdop, self.config.gps_thresholds)
        remaining = self.remaining_waypoints()
        reason = f"gps fix={fix_type.name} hdop={hdop:.1f}"

        if action == GpsAction.CONTINUE_NORMAL:
            return self._event(ReplanTrigger.GPS_DEGRADED, reason + " nominal", "no_action", remaining, remaining)

        if action == GpsAction.SLOW_DOWN:
            await self.vehicle.set_speed(nominal_speed_m_s * self.config.gps_thresholds.slow_down_speed_fraction)
            outcome = "slowed_down"
        else:
            await self.vehicle.hold()
            outcome = "hold"

        return self._event(ReplanTrigger.GPS_DEGRADED, reason, outcome, remaining, remaining)

    async def resume_after_gps_recovery(self) -> ReplanEvent:
        """Called once GPS quality recovers above thresholds after a HOLD.
        The mission was never cleared for a plain GPS-degraded HOLD (unlike
        the no-fly-zone reroute path), so resuming just needs the vehicle
        back in MISSION mode -- exactly HOW to best do that (start_mission()
        again vs. set_current_mission_item()) is one of DESIGN.md's open
        empirical questions, left to be settled during SITL verification."""
        await self.vehicle.start_mission()
        remaining = self.remaining_waypoints()
        return self._event(ReplanTrigger.GPS_DEGRADED, "gps recovered, resuming mission", "resumed", remaining, remaining)

    # -- Shared handoff mechanics --------------------------------------------

    async def _execute_handoff(self, new_remaining_waypoints: list[Waypoint]) -> None:
        """pause -> confirm settled -> clear -> upload -> resume."""
        assert all(
            wp.kind != WaypointKind.TAKEOFF for wp in new_remaining_waypoints
        ), "replanned mid-flight waypoints must never carry WaypointKind.TAKEOFF (D8 lesson)"

        await self.vehicle.pause_mission()
        await self._wait_until_settled()
        await self.vehicle.clear_mission()

        new_mission = Mission(waypoints=new_remaining_waypoints, name="replan")
        await self.vehicle.upload_mission(new_mission)
        await self._start_mission_and_confirm_resumed()

        self.current_mission = new_mission
        self._current_index = 0

    async def _start_mission_and_confirm_resumed(self, max_attempts: int = 5) -> None:
        """Call start_mission() and ACTIVELY VERIFY the vehicle actually
        enters MISSION mode, retrying the call if it doesn't.

        Found via a live-SITL trial that hung indefinitely, diagnosed with
        the same .ulg-log technique that cracked D10: MAVSDK's
        start_mission() call itself reports success (it tracks the
        MAV_CMD_MISSION_START ack), but PX4 internally follows that with its
        own DO_SET_MODE attempt to actually switch into MISSION mode -- and
        that second, internal step came back MAV_RESULT_TEMPORARILY_REJECTED
        in the .ulg (vehicle_command_ack result=1), invisible to MAVSDK's
        Python-level success/failure reporting. start_mission()'s own
        retry-on-MissionError (mavsdk_client.py) doesn't help here because
        no exception is ever raised -- the call looks like it succeeded.
        So this polls the actual FlightMode telemetry after each attempt
        instead of trusting the call's return."""
        for attempt in range(max_attempts):
            await self.vehicle.start_mission()
            if await self._wait_for_mission_mode(timeout_s=3.0):
                return
        raise TimeoutError(f"Vehicle never entered MISSION mode after {max_attempts} start_mission() attempts")

    async def _wait_for_mission_mode(self, timeout_s: float) -> bool:
        try:
            async with asyncio.timeout(timeout_s):
                async for mode in self.vehicle.flight_mode_stream():
                    if mode == "MISSION":
                        return True
        except TimeoutError:
            pass
        return False

    async def _wait_until_settled(self) -> None:
        """Block until flight_mode == HOLD and ground speed is near zero,
        for several consecutive samples -- same debounce pattern as
        wait_ready_to_arm (decisions.md D9): don't act on one noisy sample.
        """
        state: dict[str, object] = {"mode": None, "speed": None}
        queue: asyncio.Queue = asyncio.Queue()

        async def pump_mode():
            async for mode in self.vehicle.flight_mode_stream():
                await queue.put(("mode", mode))

        async def pump_speed():
            async for speed in self.vehicle.ground_speed_stream():
                await queue.put(("speed", speed))

        tasks = [asyncio.create_task(pump_mode()), asyncio.create_task(pump_speed())]
        start = time.monotonic()
        consecutive_ok = 0
        try:
            while True:
                remaining_budget = self.config.settle_timeout_s - (time.monotonic() - start)
                if remaining_budget <= 0:
                    raise TimeoutError("Vehicle did not settle into HOLD before replanning timeout")
                channel, value = await asyncio.wait_for(queue.get(), timeout=remaining_budget)
                state[channel] = value

                settled = (
                    state["mode"] == "HOLD"
                    and state["speed"] is not None
                    and state["speed"] < self.config.settle_speed_epsilon_m_s
                )
                consecutive_ok = consecutive_ok + 1 if settled else 0
                if consecutive_ok >= self.config.settle_required_consecutive:
                    return
        finally:
            for t in tasks:
                t.cancel()

    def _event(self, trigger: ReplanTrigger, reason: str, outcome: str, old: list[Waypoint], new: list[Waypoint]) -> ReplanEvent:
        return ReplanEvent(
            timestamp_unix_s=time.time(),
            trigger=trigger,
            reason=reason,
            outcome=outcome,
            old_remaining_waypoints=old,
            new_remaining_waypoints=new,
        )
