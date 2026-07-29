#!/usr/bin/env python3
"""Live-SITL check: the REAL end-to-end no-fly-zone failure-injection path.

Unlike battery/GPS, this scenario deliberately does NOT go through
ConstraintMonitor -- sim/failure_injection/scenarios.py's own design
docstring is explicit that a mid-flight no-fly-zone is an application-level
announcement (there's no PX4-native concept of a geofence appearing during
a mission), the same kind of input a real operator/dashboard action would
produce. So the real trigger path here is:

    make_no_fly_zone_scenario() builds a zone straddling a specific leg of
    the ACTUAL uploaded mission
        -> application-level progress watch (mission_progress_stream, the
           same stream engine.track_mission_progress() consumes) detects
           the vehicle has left the waypoint just before the blocked leg
        -> replanning_engine.handle_no_fly_zone() fed the real zone and the
           REAL current position read off live telemetry, not a hardcoded
           position
        -> firmware_link executes the reroute handoff (pause/clear/upload/
           resume, D11/D14's already-proven mechanics)

This test's own job is to confirm the vehicle actually avoids entering the
zone polygon during the affected leg (checked geometrically against every
recorded position, not just trusted from the outcome string) and that the
mission still completes.
"""
import asyncio, json, pathlib, sys, time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from shapely.geometry import Point, Polygon

from firmware_link.mavsdk_client import GroundLinkVehicle
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from replanning_engine.engine import ReplanningEngine, EngineConfig
from sim.failure_injection.scenarios import make_no_fly_zone_scenario

CRUISE_ALT, SPEED = 15.0, 5.0
LEG = 0.0018          # ~200m/leg
TRIGGER_AFTER_WP_INDEX = 1   # zone straddles mission.waypoints[1] -> [2]
DETECT_TIMEOUT_S = 60.0
MISSION_TIMEOUT_S = 180.0

state = {'lat': None, 'lon': None, 'alt': 0.0, 'prog': 0}
positions = []   # (t, lat, lon) -- every position sample, for zone-entry check
T0 = {'v': 0.0}

async def pump_pos(v):
    async for p in v.drone.telemetry.position():
        state['lat'], state['lon'], state['alt'] = p.latitude_deg, p.longitude_deg, p.relative_altitude_m
        positions.append((time.monotonic() - T0['v'], p.latitude_deg, p.longitude_deg))

async def main():
    out = {'ok': False}
    v = GroundLinkVehicle()
    await v.connect(timeout_s=60)
    await v.wait_ready_to_arm(timeout_s=90)

    async for h in v.drone.telemetry.home():
        home = (h.latitude_deg, h.longitude_deg); break
    out['home'] = [round(home[0], 6), round(home[1], 6)]

    mission = Mission(name='nfz_scenario', waypoints=[
        Waypoint(home[0], home[1], CRUISE_ALT, kind=WaypointKind.TAKEOFF, speed_m_s=SPEED),
        Waypoint(home[0] + 1 * LEG, home[1], CRUISE_ALT, speed_m_s=SPEED),
        Waypoint(home[0] + 2 * LEG, home[1], CRUISE_ALT, speed_m_s=SPEED),
        Waypoint(home[0] + 3 * LEG, home[1], CRUISE_ALT, speed_m_s=SPEED),
    ])
    await v.upload_mission(mission)

    # Build the zone against the REAL uploaded mission -- it straddles the
    # leg from waypoints[1] to waypoints[2].
    scenario = make_no_fly_zone_scenario(
        mission, home[0], home[1], trigger_after_waypoint_index=TRIGGER_AFTER_WP_INDEX, width_m=80.0,
    )
    zone_polygon = Polygon([(lon, lat) for lat, lon in scenario.zone.boundary_latlon])
    out['zone_boundary'] = [(round(a, 6), round(b, 6)) for a, b in scenario.zone.boundary_latlon]

    await v.arm()
    T0['v'] = time.monotonic()
    tasks = [asyncio.create_task(pump_pos(v))]
    try:
        await v.start_mission()
        engine = ReplanningEngine(v, EngineConfig())
        engine.set_active_mission(mission)

        # Application-level detection: watch the REAL mission_progress
        # stream (the same one engine.track_mission_progress() consumes)
        # for the vehicle having left waypoint[TRIGGER_AFTER_WP_INDEX],
        # i.e. now heading toward the blocked leg -- exactly the condition
        # scenario.trigger_after_waypoint_index describes.
        target_index = TRIGGER_AFTER_WP_INDEX + 1
        t_start = time.monotonic()
        async with asyncio.timeout(DETECT_TIMEOUT_S):
            async for current, _total in v.mission_progress_stream():
                state['prog'] = current
                if current >= target_index:
                    break
        out['t_to_announce_s'] = round(time.monotonic() - t_start, 2)
        out['progress_at_announce'] = state['prog']
        t_announce_rel = time.monotonic() - T0['v']

        # Feed the REAL zone and REAL current position -- not a hardcoded
        # test position -- exactly as the real evaluation loop would.
        current_pos = (await anext(v.drone.telemetry.position().__aiter__()))
        from firmware_link.telemetry import Position
        real_position = Position(
            latitude_deg=current_pos.latitude_deg, longitude_deg=current_pos.longitude_deg,
            absolute_altitude_m=current_pos.absolute_altitude_m, relative_altitude_m=current_pos.relative_altitude_m,
        )
        out['position_at_announce'] = [round(real_position.latitude_deg, 6), round(real_position.longitude_deg, 6)]

        ev = await engine.handle_no_fly_zone(scenario.zone, real_position)
        out['event_outcome'] = ev.outcome
        out['new_remaining_count'] = len(ev.new_remaining_waypoints)

        # Confirm the mission completes.
        t_f = time.monotonic()
        async with asyncio.timeout(MISSION_TIMEOUT_S):
            async for current, total in v.mission_progress_stream():
                if current >= total - 1:
                    break
        out['t_to_mission_end_s'] = round(time.monotonic() - t_f, 2)

        # Give the final leg a moment to actually finish, then check every
        # recorded position from the moment the zone was announced onward
        # against the zone polygon -- geometric proof of avoidance, not
        # just trusting the 'rerouted' string.
        await asyncio.sleep(5)
        positions_after_announce = [(t, la, lo) for (t, la, lo) in positions if t >= t_announce_rel]
        entries = [(t, la, lo) for (t, la, lo) in positions_after_announce if zone_polygon.contains(Point(lo, la))]
        out['positions_checked'] = len(positions_after_announce)
        out['zone_entries'] = len(entries)
        out['zone_never_entered'] = len(entries) == 0

        out['ok'] = (ev.outcome == 'rerouted' and out['zone_never_entered'] and out['positions_checked'] > 20)
    except Exception as e:
        out['error'] = f'{type(e).__name__}: {e}'
    finally:
        for t_ in tasks:
            t_.cancel()
    print('RESULT ' + json.dumps(out))

asyncio.run(main())
