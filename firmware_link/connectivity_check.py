"""Step 2 milestone: does the plumbing work?

Connects to a running PX4 SITL instance, arms it, uploads a trivial
3-waypoint mission, starts it, and streams telemetry to stdout.

Run against a REAL PX4 SITL instance (see README.md / sim/launch_sitl.sh) —
this is intentionally not mocked, per project convention.

Usage (from inside the WSL venv):
    python -m firmware_link.connectivity_check
"""

from __future__ import annotations

import asyncio
import sys
import time

from firmware_link.mavsdk_client import GroundLinkVehicle
from mission_planner.waypoint import Mission, Waypoint, WaypointKind

# PX4 SITL default home position (jMAVSim/Gazebo default), offset slightly
# for a trivial 3-waypoint mission. Not a real-world site.
_HOME_LAT = 47.397742
_HOME_LON = 8.545594

_TEST_MISSION = Mission(
    name="connectivity_check",
    waypoints=[
        # First waypoint MUST be kind=TAKEOFF -- raw MAVSDK mission uploads
        # don't auto-insert a takeoff item the way QGroundControl does. See
        # firmware_link/mavsdk_client.py's _mission_to_plan docstring and
        # decisions.md D8.
        Waypoint(latitude_deg=_HOME_LAT + 0.0003, longitude_deg=_HOME_LON, relative_altitude_m=15.0, kind=WaypointKind.TAKEOFF),
        Waypoint(latitude_deg=_HOME_LAT + 0.0003, longitude_deg=_HOME_LON + 0.0003, relative_altitude_m=15.0),
        Waypoint(latitude_deg=_HOME_LAT, longitude_deg=_HOME_LON + 0.0003, relative_altitude_m=15.0),
    ],
)


async def main() -> int:
    vehicle = GroundLinkVehicle(system_address="udpin://0.0.0.0:14540")

    print("[connectivity_check] connecting to PX4 SITL on udp://:14540 ...")
    await vehicle.connect(timeout_s=30.0)
    print("[connectivity_check] connected.")

    print("[connectivity_check] waiting for GPS/home position lock (SITL-simulated GPS)...")
    await vehicle.wait_ready_to_arm(timeout_s=60.0)
    print("[connectivity_check] ready to arm.")

    print(f"[connectivity_check] uploading {len(_TEST_MISSION)}-waypoint test mission...")
    await vehicle.upload_mission(_TEST_MISSION)
    print("[connectivity_check] mission uploaded.")

    print("[connectivity_check] arming...")
    await vehicle.arm()
    print("[connectivity_check] armed.")

    print("[connectivity_check] starting mission...")
    await vehicle.start_mission()
    print("[connectivity_check] mission started. Streaming telemetry (Ctrl+C to stop):\n")

    # telemetry_stream() yields a new merged snapshot on every channel update
    # (attitude alone updates ~250Hz) -- print at most once/second so a human
    # can actually read this, rather than flooding stdout with near-duplicate
    # lines. The full unthrottled rate is exactly what constraint_monitor
    # consumes in Step 4; this throttle is presentation-only, here.
    run_start = time.monotonic()
    last_print = 0.0
    run_duration_s = 45.0

    async for snapshot in vehicle.telemetry_stream():
        now = time.monotonic()
        if now - last_print >= 1.0:
            bat = f"{snapshot.battery.remaining_percent:.1f}%" if snapshot.battery else "?"
            gps = f"{snapshot.gps.fix_type.name} ({snapshot.gps.num_satellites} sats)" if snapshot.gps else "?"
            pos = (
                f"lat={snapshot.position.latitude_deg:.6f} lon={snapshot.position.longitude_deg:.6f} "
                f"alt_rel={snapshot.position.relative_altitude_m:.1f}m"
                if snapshot.position
                else "?"
            )
            print(f"[t={snapshot.timestamp_unix_s:.1f}] battery={bat} gps={gps} pos={pos}")
            last_print = now

        if now - run_start >= run_duration_s:
            print(f"\n[connectivity_check] ran for {run_duration_s:.0f}s, stopping.")
            break

    return 0


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        sys.exit(0)
