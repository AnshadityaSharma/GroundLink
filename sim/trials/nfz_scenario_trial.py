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
        -> firmware_link executes the reroute handoff (adaptive) or RTL
           (baseline)

evaluation/DESIGN.md open question 3: this uses a real lawnmower/coverage-
grid mission (mission_planner.grid_coverage.generate_lawnmower_mission(),
already built and unit-tested) rather than D21's simple 4-waypoint
point-to-point mission, so "coverage %" means something concrete -- the
no-fly zone straddles one of the sweep's actual rows. Completion/coverage %
is measured geometrically against the original mission's swept waypoints
(same method as the battery/GPS scenario scripts), independent of both
adaptive's internal replan bookkeeping and baseline's RTL making
mission_progress meaningless.
"""
import argparse
import asyncio
import json
import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from shapely.geometry import Point, Polygon

from firmware_link.mavsdk_client import GroundLinkVehicle
from firmware_link.telemetry import Position
from mission_planner.grid_coverage import generate_lawnmower_mission
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from replanning_engine.engine import ReplanningEngine, EngineConfig
from sim.failure_injection.scenarios import make_no_fly_zone_scenario

CRUISE_ALT, SPEED = 15.0, 5.0
# Small coverage rectangle north of home: ~220m (E-W) x 180m (N-S), 90m row
# spacing -> 3 rows, 6 sweep waypoints (picked empirically for a total path
# length of ~720m / ~144s cruise, the same order of magnitude as the other
# scenario trials -- see decisions.md D22 for the sizing check).
BOUNDARY_OFFSETS_M = [(-110, 40), (110, 40), (110, 220), (-110, 220)]
ROW_SPACING_M = 90.0
TRIGGER_AFTER_WP_INDEX = 3   # zone straddles waypoints[3]->[4], one of the sweep rows
DETECT_TIMEOUT_S = 150.0    # this mission is longer than D21's 4-waypoint one -- climb (~17s)
# + reaching item 4 partway through a ~720m/~144s sweep is comfortably under 120s in
# practice, but was measured, not assumed, after an initial 60s timeout (decisions.md D22)
MISSION_TIMEOUT_S = 240.0
SAFE_CONFIRM_TIMEOUT_S = 20.0
POST_CONFIRM_WATCH_S = 15.0

state = {'lat': None, 'lon': None, 'alt': 0.0, 'mode': None, 'prog': 0}
positions = []   # (t, lat, lon) -- every position sample, for zone-entry and completion checks
T0 = {'v': 0.0}


def dist_m(a, b, c, d):
    R = 6371000.0
    p1, p2 = math.radians(a), math.radians(c)
    dp, dl = math.radians(c - a), math.radians(d - b)
    x = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(x))


def offset_latlon(lat, lon, dx_m, dy_m):
    R = 6371000.0
    dlat = (dy_m / R) * (180.0 / math.pi)
    dlon = (dx_m / (R * math.cos(math.radians(lat)))) * (180.0 / math.pi)
    return lat + dlat, lon + dlon


async def pump_pos(v):
    async for p in v.drone.telemetry.position():
        state['lat'], state['lon'], state['alt'] = p.latitude_deg, p.longitude_deg, p.relative_altitude_m
        positions.append((time.monotonic() - T0['v'], p.latitude_deg, p.longitude_deg))


async def pump_mode(v):
    async for m in v.flight_mode_stream():
        state['mode'] = m


def completion_percent(mission: Mission) -> dict:
    """Fraction of the original coverage mission's sweep waypoints (non-
    takeoff) the vehicle came within acceptance_radius_m of -- the same
    method used in the battery/GPS scenario scripts, and the practical
    stand-in for true swept-area coverage per DESIGN.md section 2."""
    targets = [wp for wp in mission.waypoints if wp.kind != WaypointKind.TAKEOFF]
    reached = 0
    for wp in targets:
        hit = any(dist_m(wp.latitude_deg, wp.longitude_deg, la, lo) <= wp.acceptance_radius_m for (_t, la, lo) in positions)
        reached += 1 if hit else 0
    return {'reached': reached, 'total': len(targets), 'percent': round(100.0 * reached / len(targets), 1) if targets else 0.0}


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', action='store_true')
    args = parser.parse_args()

    out = {'ok': False, 'condition': 'baseline' if args.baseline else 'adaptive'}
    v = GroundLinkVehicle()
    await v.connect(timeout_s=60)
    await v.wait_ready_to_arm(timeout_s=90)

    async for h in v.drone.telemetry.home():
        home = (h.latitude_deg, h.longitude_deg); break
    out['home'] = [round(home[0], 6), round(home[1], 6)]

    boundary = [offset_latlon(home[0], home[1], dx, dy) for dx, dy in BOUNDARY_OFFSETS_M]
    sweep = generate_lawnmower_mission(boundary, spacing_m=ROW_SPACING_M, altitude_m=CRUISE_ALT, speed_m_s=SPEED)
    mission = Mission(name='nfz_scenario_coverage', waypoints=[
        Waypoint(home[0], home[1], CRUISE_ALT, kind=WaypointKind.TAKEOFF, speed_m_s=SPEED),
    ] + list(sweep.waypoints))
    await v.upload_mission(mission)
    out['n_sweep_waypoints'] = len(sweep.waypoints)

    # Build the zone against the REAL uploaded mission -- it straddles one
    # of the actual sweep rows.
    scenario = make_no_fly_zone_scenario(
        mission, home[0], home[1], trigger_after_waypoint_index=TRIGGER_AFTER_WP_INDEX, width_m=60.0,
    )
    zone_polygon = Polygon([(lon, lat) for lat, lon in scenario.zone.boundary_latlon])
    out['zone_boundary'] = [(round(a, 6), round(b, 6)) for a, b in scenario.zone.boundary_latlon]

    await v.arm()
    T0['v'] = time.monotonic()
    tasks = [asyncio.create_task(f) for f in (pump_pos(v), pump_mode(v))]
    try:
        await v.start_mission()
        engine = ReplanningEngine(v, EngineConfig(adaptive_replanning_enabled=not args.baseline))
        engine.set_active_mission(mission)

        # Application-level detection: watch the REAL mission_progress
        # stream (the same one engine.track_mission_progress() consumes)
        # for the vehicle having left waypoint[TRIGGER_AFTER_WP_INDEX],
        # i.e. now heading toward the blocked leg -- identical between
        # conditions, per evaluation/DESIGN.md's "detection unaffected"
        # principle.
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
        real_position = Position(
            latitude_deg=current_pos.latitude_deg, longitude_deg=current_pos.longitude_deg,
            absolute_altitude_m=current_pos.absolute_altitude_m, relative_altitude_m=current_pos.relative_altitude_m,
        )
        out['position_at_announce'] = [round(real_position.latitude_deg, 6), round(real_position.longitude_deg, 6)]

        ev = await engine.handle_no_fly_zone(scenario.zone, real_position)
        out['event_outcome'] = ev.outcome
        out['new_remaining_count'] = len(ev.new_remaining_waypoints)
        t_cmd = time.monotonic()

        if ev.outcome == 'rerouted':
            # Confirm the mission completes. mission_progress.current
            # reaching total-1 means the vehicle has just STARTED heading
            # to the final item, not reached it -- an earlier version broke
            # here and only slept 5s afterward, which undercounted
            # completion (decisions.md D22: measured 4/6, not the full 6/6,
            # because the final ~180m leg hadn't finished). Wait for the
            # final item's own position directly instead of trusting the
            # progress index alone.
            t_f = time.monotonic()
            async with asyncio.timeout(MISSION_TIMEOUT_S):
                async for current, total in v.mission_progress_stream():
                    if current >= total - 1:
                        break
            final_wp = mission.waypoints[-1]
            async with asyncio.timeout(MISSION_TIMEOUT_S):
                while dist_m(final_wp.latitude_deg, final_wp.longitude_deg, state['lat'], state['lon']) > final_wp.acceptance_radius_m:
                    await asyncio.sleep(0.3)
            out['t_to_mission_end_s'] = round(time.monotonic() - t_f, 2)
            await asyncio.sleep(3)
            out['safe_confirmed'] = True  # reroute completion is itself the 'safe' outcome here
        else:
            # baseline_rtl -- watch for RTL mode + distance-to-home
            # shrinking, same pattern as the battery/GPS scenario scripts.
            d0 = dist_m(home[0], home[1], state['lat'], state['lon']) if state['lat'] is not None else None
            while time.monotonic() - t_cmd < SAFE_CONFIRM_TIMEOUT_S and state['mode'] != 'RETURN_TO_LAUNCH':
                await asyncio.sleep(0.2)
            out['reached_response_mode'] = state['mode'] == 'RETURN_TO_LAUNCH'
            await asyncio.sleep(POST_CONFIRM_WATCH_S)
            d1 = dist_m(home[0], home[1], state['lat'], state['lon']) if state['lat'] is not None else None
            out['dist_at_response_m'] = round(d0, 1) if d0 is not None else None
            out['dist_after_watch_m'] = round(d1, 1) if d1 is not None else None
            out['returning'] = (d0 is not None and d1 is not None and d1 < d0 - 5.0)
            out['safe_confirmed'] = out['reached_response_mode'] and out['returning']

        # Geometric proof of avoidance, not just trusting the outcome
        # string -- every recorded position from the moment of announcement
        # onward, checked against the zone polygon.
        positions_after_announce = [(t, la, lo) for (t, la, lo) in positions if t >= t_announce_rel]
        entries = [(t, la, lo) for (t, la, lo) in positions_after_announce if zone_polygon.contains(Point(lo, la))]
        out['positions_checked'] = len(positions_after_announce)
        out['zone_entries'] = len(entries)
        out['zone_never_entered'] = len(entries) == 0

        comp = completion_percent(mission)
        out['completion'] = comp

        out['ok'] = out['zone_never_entered'] and (
            (ev.outcome == 'rerouted' and not args.baseline) or (ev.outcome == 'baseline_rtl' and args.baseline)
        )
    except Exception as e:
        out['error'] = f'{type(e).__name__}: {e}'
    finally:
        for t_ in tasks:
            t_.cancel()
    print('RESULT ' + json.dumps(out))

asyncio.run(main())
