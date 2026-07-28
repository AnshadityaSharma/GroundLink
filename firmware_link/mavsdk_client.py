"""MAVSDK-backed implementation of the firmware link.

This is the ONLY module in the codebase that imports `mavsdk`. Everything
else (mission_planner, constraint_monitor, replanning_engine, dashboard)
talks in terms of mission_planner.Mission and firmware_link.telemetry types.
Swapping MAVSDK for something else later means changing this file only.

NOTE: parameter ordering/field names below (MissionItem, GpsInfo, RawGps,
Battery, Position) were checked against the actually-installed mavsdk
package (3.17.2, via runtime introspection of its type signatures in the
WSL venv) rather than assumed from memory/docs — this caught two real bugs
(remaining_percent is already 0-100 not 0-1; FixType needs `.value`, `int()`
raises on it) and one gap (hdop lives on raw_gps(), not gps_info()). See
decisions.md D5.

Run end-to-end against real PX4 SITL (v1.17.0, Gazebo Harmonic, WSL2),
40+ times via a scripted trial harness: connect/arm/upload/start_mission/
telemetry streaming AND full mission completion (vehicle climbs and flies
through all waypoints) are now reliable -- 8/8 in the final confirmation
batch, altitudes 14.94-14.96m every time, plus a clean re-verification
through the real sim/launch_sitl.sh + connectivity_check.py.

Getting here took three real, independent bugs, all found via systematic
testing rather than guessing -- see decisions.md D8/D9/D10 for the full
account:
  1. Missing WaypointKind.TAKEOFF on the first waypoint (D8) -- raw MAVSDK
     mission uploads don't auto-insert takeoff the way QGroundControl does.
  2. Weak arm-readiness gate (D9) -- is_global_position_ok/is_home_position_ok
     could read true before PX4's own is_armable check agreed, causing
     COMMAND_DENIED on arm(). Fixed below via debounced is_armable + retry.
  3. THE actual cause of the "mission completes instantly without flying"
     symptom (D10): PX4's on-disk `dataman` mission store persists across
     process restarts and was never cleared between SITL runs, so a fresh
     process would immediately execute stale leftover mission state.
     Confirmed via PX4's own .ulg log, not MAVSDK-side speculation -- an
     earlier hypothesis blamed WSL2 scheduler timing; that was WRONG and is
     explicitly superseded in decisions.md D9's note. The real fix lives in
     sim/launch_sitl.sh (deletes dataman before every launch), not in this
     file -- nothing in mavsdk_client.py needed to change for #3.
"""

from __future__ import annotations

import asyncio
import time
from collections.abc import AsyncIterator

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.mission import MissionItem, MissionPlan

from firmware_link.telemetry import (
    Attitude,
    BatteryState,
    GpsFixType,
    GpsState,
    Position,
    TelemetrySnapshot,
)
from mission_planner.waypoint import Mission, WaypointKind

_VEHICLE_ACTION_BY_KIND = {
    WaypointKind.NAV: MissionItem.VehicleAction.NONE,
    WaypointKind.TAKEOFF: MissionItem.VehicleAction.TAKEOFF,
    WaypointKind.LAND: MissionItem.VehicleAction.LAND,
    # RTL isn't a MissionItem vehicle_action in MAVSDK -- it's a top-level
    # command (action.return_to_launch()), not a mission-item property.
    # A WaypointKind.RTL waypoint degrades to a plain NAV point for now;
    # revisit when replanning_engine needs to trigger RTL mid-mission.
    WaypointKind.RTL: MissionItem.VehicleAction.NONE,
}


def _mission_to_plan(mission: Mission, default_speed_m_s: float = 5.0) -> MissionPlan:
    """Convert a firmware-agnostic Mission into a MAVSDK MissionPlan.

    IMPORTANT: unlike QGroundControl (which auto-inserts a takeoff item),
    raw MAVSDK mission uploads do NOT auto-takeoff. Verified against real
    PX4 SITL: a mission with no TAKEOFF vehicle_action reports
    mission_progress as immediately complete (3/3) while the vehicle never
    leaves the ground -- horizontal-only acceptance-radius checks are
    satisfied trivially without ever climbing. Callers building a
    ground-start mission MUST mark their first waypoint
    kind=WaypointKind.TAKEOFF (see connectivity_check.py). Logged as D8.
    """
    items = [
        MissionItem(
            wp.latitude_deg,
            wp.longitude_deg,
            wp.relative_altitude_m,
            wp.speed_m_s if wp.speed_m_s is not None else default_speed_m_s,
            True,  # is_fly_through
            float("nan"),  # gimbal_pitch_deg
            float("nan"),  # gimbal_yaw_deg
            MissionItem.CameraAction.NONE,
            float("nan"),  # loiter_time_s
            float("nan"),  # camera_photo_interval_s
            wp.acceptance_radius_m,
            float("nan"),  # yaw_deg
            float("nan"),  # camera_photo_distance_m
            _VEHICLE_ACTION_BY_KIND[wp.kind],
        )
        for wp in mission.waypoints
    ]
    return MissionPlan(items)


class GroundLinkVehicle:
    """Thin async wrapper over mavsdk.System for GroundLink's needs."""

    def __init__(self, system_address: str = "udpin://0.0.0.0:14540"):
        self._system_address = system_address
        self.drone = System()

    async def connect(self, timeout_s: float = 30.0) -> None:
        await self.drone.connect(system_address=self._system_address)

        start = time.monotonic()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                return
            if time.monotonic() - start > timeout_s:
                raise TimeoutError(f"No PX4 SITL connection on {self._system_address} after {timeout_s}s")

    async def wait_ready_to_arm(self, timeout_s: float = 60.0, required_consecutive: int = 3) -> None:
        """Wait for PX4's own is_armable signal, debounced.

        A single instantaneous read of is_global_position_ok/is_home_position_ok
        (the original check here) was found -- via a systematic N-trial pass-rate
        test, not a one-off -- to go true before PX4's own arming-check logic
        agrees the vehicle can actually arm: real runs against SITL got
        COMMAND_DENIED on arm() despite this check having already returned.
        is_armable is PX4's own authoritative "can arm right now" signal, and
        requiring it to hold for several consecutive samples (not just one)
        guards against a single-sample flicker. See decisions.md D9.
        """
        start = time.monotonic()
        consecutive_ok = 0
        async for health in self.drone.telemetry.health():
            ready = (
                health.is_global_position_ok
                and health.is_home_position_ok
                and health.is_local_position_ok
                and health.is_armable
            )
            consecutive_ok = consecutive_ok + 1 if ready else 0
            if consecutive_ok >= required_consecutive:
                return
            if time.monotonic() - start > timeout_s:
                raise TimeoutError("Vehicle did not become ready-to-arm (health checks) in time")

    async def arm(self, retries: int = 5, retry_delay_s: float = 1.0) -> None:
        """Retries on ActionError since PX4 can transiently deny arming for a
        moment even after is_armable reads true -- see wait_ready_to_arm."""
        last_error: ActionError | None = None
        for attempt in range(retries):
            try:
                await self.drone.action.arm()
                return
            except ActionError as e:
                last_error = e
                if attempt < retries - 1:
                    await asyncio.sleep(retry_delay_s)
        assert last_error is not None
        raise last_error

    async def upload_mission(self, mission: Mission) -> None:
        plan = _mission_to_plan(mission)
        await self.drone.mission.upload_mission(plan)

    async def start_mission(self) -> None:
        await self.drone.mission.start_mission()

    async def telemetry_stream(self) -> AsyncIterator[TelemetrySnapshot]:
        """Merge battery/gps/position/attitude into combined snapshots.

        MAVSDK exposes each telemetry channel as its own independent async
        generator with its own update rate. We keep the latest value of each
        and emit a snapshot every time ANY channel updates, so consumers see
        near-real-time data without needing to know about MAVSDK's channel
        model.
        """
        latest_battery: BatteryState | None = None
        latest_position: Position | None = None
        latest_attitude: Attitude | None = None

        # GpsState merges two independent MAVSDK channels (gps_info: fix
        # type/sat count, raw_gps: hdop) so it's tracked as separate parts
        # and rebuilt whenever either channel updates.
        latest_fix_type: GpsFixType | None = None
        latest_num_satellites: int | None = None
        latest_hdop: float | None = None

        def _current_gps_state() -> GpsState | None:
            if latest_fix_type is None or latest_num_satellites is None:
                return None
            return GpsState(
                fix_type=latest_fix_type,
                num_satellites=latest_num_satellites,
                hdop=latest_hdop if latest_hdop is not None else float("nan"),
            )

        queue: asyncio.Queue = asyncio.Queue()

        async def pump_battery():
            async for b in self.drone.telemetry.battery():
                await queue.put(("battery", BatteryState(voltage_v=b.voltage_v, remaining_percent=b.remaining_percent)))

        async def pump_gps_info():
            async for g in self.drone.telemetry.gps_info():
                await queue.put(("gps_info", (GpsFixType(g.fix_type.value), g.num_satellites)))

        async def pump_raw_gps():
            async for g in self.drone.telemetry.raw_gps():
                await queue.put(("raw_gps_hdop", g.hdop))

        async def pump_position():
            async for p in self.drone.telemetry.position():
                await queue.put(
                    (
                        "position",
                        Position(
                            latitude_deg=p.latitude_deg,
                            longitude_deg=p.longitude_deg,
                            absolute_altitude_m=p.absolute_altitude_m,
                            relative_altitude_m=p.relative_altitude_m,
                        ),
                    )
                )

        async def pump_attitude():
            async for a in self.drone.telemetry.attitude_euler():
                await queue.put(("attitude", Attitude(roll_deg=a.roll_deg, pitch_deg=a.pitch_deg, yaw_deg=a.yaw_deg)))

        tasks = [
            asyncio.create_task(pump_battery()),
            asyncio.create_task(pump_gps_info()),
            asyncio.create_task(pump_raw_gps()),
            asyncio.create_task(pump_position()),
            asyncio.create_task(pump_attitude()),
        ]

        try:
            while True:
                channel, value = await queue.get()
                if channel == "battery":
                    latest_battery = value
                elif channel == "gps_info":
                    latest_fix_type, latest_num_satellites = value
                elif channel == "raw_gps_hdop":
                    latest_hdop = value
                elif channel == "position":
                    latest_position = value
                elif channel == "attitude":
                    latest_attitude = value

                yield TelemetrySnapshot(
                    timestamp_unix_s=time.time(),
                    battery=latest_battery,
                    gps=_current_gps_state(),
                    position=latest_position,
                    attitude=latest_attitude,
                )
        finally:
            for task in tasks:
                task.cancel()
