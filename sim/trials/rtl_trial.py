#!/usr/bin/env python3
"""Live-SITL check: does handle_battery_critical()'s RTL path actually
complete a full return-and-land, or does it only switch flight mode?

D13 verified only that FlightMode reached RETURN_TO_LAUNCH within 15s, and
explicitly flagged the rest as unverified. This watches the whole sequence
through to disarm:

    mode -> RETURN_TO_LAUNCH, then vehicle transits back toward home
    (distance-to-home must actually shrink), descends (rel_alt -> ~0),
    and finally DISARMS on its own.

Note PX4's RTL climbs to RTL_RETURN_ALT (60m by default) before transiting,
so this legitimately takes a couple of minutes -- that duration is why D13
stopped at the mode switch.
"""
import asyncio, json, math, pathlib, sys, time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from firmware_link.mavsdk_client import GroundLinkVehicle
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from replanning_engine.engine import ReplanningEngine, EngineConfig

CRUISE_ALT = 15.0
SPEED = 5.0
OUTBOUND_DEG_LAT = 0.0018      # ~200m out, far enough that 'returned home' is meaningful
MIN_DIST_BEFORE_RTL_M = 100.0
RTL_TIMEOUT_S = 300.0
BATTERY_PCT_FOR_RTL = 15.0     # below rtl_below_percent=20, above land_immediately=8

state = {'lat': None, 'lon': None, 'alt': 0.0, 'mode': None, 'armed': True}
trace = []
T0 = {'v': 0.0}

def dist_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))

async def pump_pos(v):
    async for p in v.drone.telemetry.position():
        state['lat'], state['lon'], state['alt'] = p.latitude_deg, p.longitude_deg, p.relative_altitude_m

async def pump_mode(v):
    async for m in v.flight_mode_stream():
        state['mode'] = m

async def pump_armed(v):
    async for a in v.drone.telemetry.armed():
        state['armed'] = a

async def pump_trace(home):
    while True:
        await asyncio.sleep(2.0)
        if state['lat'] is not None:
            trace.append((round(time.monotonic() - T0['v'], 1),
                          round(dist_m(home[0], home[1], state['lat'], state['lon']), 1),
                          round(state['alt'], 1), state['mode'], state['armed']))

async def main():
    out = {'ok': False}
    v = GroundLinkVehicle()
    await v.connect(timeout_s=60)
    await v.wait_ready_to_arm(timeout_s=90)

    async for h in v.drone.telemetry.home():
        home = (h.latitude_deg, h.longitude_deg)
        break
    out['home'] = [round(home[0], 6), round(home[1], 6)]

    mission = Mission(name='rtl_check', waypoints=[
        Waypoint(home[0], home[1], CRUISE_ALT, kind=WaypointKind.TAKEOFF, speed_m_s=SPEED),
        Waypoint(home[0] + OUTBOUND_DEG_LAT, home[1], CRUISE_ALT, speed_m_s=SPEED),
    ])
    await v.upload_mission(mission)
    await v.arm()

    T0['v'] = time.monotonic()
    tasks = [asyncio.create_task(f) for f in
             (pump_pos(v), pump_mode(v), pump_armed(v), pump_trace(home))]
    try:
        await v.start_mission()
        engine = ReplanningEngine(v, EngineConfig())
        engine.set_active_mission(mission)

        # fly out until genuinely away from home, so 'came back' means something
        t = time.monotonic()
        while time.monotonic() - t < 180:
            await asyncio.sleep(0.5)
            if state['lat'] is None:
                continue
            d = dist_m(home[0], home[1], state['lat'], state['lon'])
            if d >= MIN_DIST_BEFORE_RTL_M and state['alt'] > CRUISE_ALT - 2:
                break
        else:
            raise TimeoutError('never got far enough from home to test RTL')

        out['dist_at_rtl_m'] = round(d, 1)
        out['alt_at_rtl_m'] = round(state['alt'], 1)

        ev = await engine.handle_battery_critical(BATTERY_PCT_FOR_RTL)
        out['event_outcome'] = ev.outcome
        t_cmd = time.monotonic()

        # --- phase 1: mode actually switches
        while time.monotonic() - t_cmd < 20:
            if state['mode'] == 'RETURN_TO_LAUNCH':
                break
            await asyncio.sleep(0.2)
        out['reached_rtl_mode'] = state['mode'] == 'RETURN_TO_LAUNCH'
        out['t_to_rtl_mode_s'] = round(time.monotonic() - t_cmd, 1)

        # --- phase 2: actually returns, descends, and disarms
        max_dist = 0.0
        while time.monotonic() - t_cmd < RTL_TIMEOUT_S:
            await asyncio.sleep(0.5)
            max_dist = max(max_dist, dist_m(home[0], home[1], state['lat'], state['lon']))
            if not state['armed']:
                break
        out['disarmed'] = not state['armed']
        out['t_to_disarm_s'] = round(time.monotonic() - t_cmd, 1)
        out['max_dist_during_rtl_m'] = round(max_dist, 1)
        out['final_dist_to_home_m'] = round(dist_m(home[0], home[1], state['lat'], state['lon']), 1)
        out['final_alt_m'] = round(state['alt'], 2)

        out['ok'] = (ev.outcome == 'rtl'
                     and out['reached_rtl_mode']
                     and out['disarmed']
                     and out['final_dist_to_home_m'] < 10.0
                     and out['final_alt_m'] < 1.0)
    except Exception as e:
        out['error'] = f'{type(e).__name__}: {e}'
    finally:
        for t_ in tasks:
            t_.cancel()
    out['trace'] = trace
    print('RESULT ' + json.dumps(out))

asyncio.run(main())
