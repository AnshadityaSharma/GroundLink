#!/usr/bin/env python3
"""Live-SITL check: ReplanningEngine.resume_after_gps_recovery().

Completely untested before this. Two things are being established:

1. Does it work at all -- after a GPS-degraded HOLD, does calling it actually
   put the vehicle back into MISSION mode and get it flying again?
2. DESIGN.md's open empirical question: does a plain start_mission() RESUME
   from the current mission item, or RESTART the mission from item 0? The
   answer decides whether resume_after_gps_recovery() is correct as written
   or needs set_current_mission_item() first (mavsdk_client.resume_mission_from).

Live risk being probed deliberately: resume_after_gps_recovery() calls
vehicle.start_mission() DIRECTLY, bypassing engine._start_mission_and_confirm_resumed().
That wrapper exists because PX4 can silently reject the internal mode switch
while MAVSDK reports success (D11). So this test never trusts the call's
return -- it polls FlightMode and real position afterwards.
"""
import asyncio, json, math, pathlib, sys, time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from firmware_link.mavsdk_client import GroundLinkVehicle
from firmware_link.telemetry import GpsFixType
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from replanning_engine.engine import ReplanningEngine, EngineConfig

CRUISE_ALT, SPEED = 15.0, 5.0
LEG = 0.0014                 # ~155m per leg -> ~30s each, so waypoints tick over
HOLD_AT_PROGRESS = 2         # trigger the HOLD while heading to item 2
RESUME_CONFIRM_TIMEOUT_S = 20.0

state = {'lat': None, 'lon': None, 'alt': 0.0, 'mode': None, 'speed': 0.0, 'prog': 0}
trace, positions = [], []
T0 = {'v': 0.0}

def dist_m(a, b, c, d):
    R = 6371000.0
    p1, p2 = math.radians(a), math.radians(c)
    dp, dl = math.radians(c - a), math.radians(d - b)
    x = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R*math.asin(math.sqrt(x))

async def pump_pos(v):
    async for p in v.drone.telemetry.position():
        state['lat'], state['lon'], state['alt'] = p.latitude_deg, p.longitude_deg, p.relative_altitude_m
        positions.append((time.monotonic(), p.latitude_deg, p.longitude_deg))

async def pump_mode(v):
    async for m in v.flight_mode_stream():
        state['mode'] = m

async def pump_speed(v):
    async for s in v.ground_speed_stream():
        state['speed'] = s

async def pump_prog(v):
    async for cur, _t in v.mission_progress_stream():
        state['prog'] = cur

async def pump_trace():
    while True:
        await asyncio.sleep(1.0)
        trace.append((round(time.monotonic()-T0['v'],1), round(state['speed'],2),
                      round(state['alt'],1), state['mode'], state['prog']))

def travelled_since(t_mark):
    pts = [(la, lo) for (t, la, lo) in positions if t >= t_mark]
    return sum(dist_m(pts[i-1][0], pts[i-1][1], pts[i][0], pts[i][1]) for i in range(1, len(pts))) if len(pts) > 1 else 0.0

async def main():
    out = {'ok': False}
    v = GroundLinkVehicle()
    await v.connect(timeout_s=60)
    await v.wait_ready_to_arm(timeout_s=90)

    async for h in v.drone.telemetry.home():
        home = (h.latitude_deg, h.longitude_deg); break

    mission = Mission(name='gps_resume', waypoints=[
        Waypoint(home[0], home[1], CRUISE_ALT, kind=WaypointKind.TAKEOFF, speed_m_s=SPEED),
        Waypoint(home[0] + LEG,     home[1], CRUISE_ALT, speed_m_s=SPEED),
        Waypoint(home[0] + 2 * LEG, home[1], CRUISE_ALT, speed_m_s=SPEED),
        Waypoint(home[0] + 3 * LEG, home[1], CRUISE_ALT, speed_m_s=SPEED),
    ])
    await v.upload_mission(mission)
    await v.arm()

    T0['v'] = time.monotonic()
    tasks = [asyncio.create_task(f) for f in
             (pump_pos(v), pump_mode(v), pump_speed(v), pump_prog(v), pump_trace())]
    try:
        await v.start_mission()
        engine = ReplanningEngine(v, EngineConfig())
        engine.set_active_mission(mission)
        asyncio.create_task(engine.track_mission_progress())

        # fly until we're partway through the mission, so 'resume' has a
        # meaningful place to resume FROM
        t = time.monotonic()
        while time.monotonic() - t < 150:
            if state['prog'] >= HOLD_AT_PROGRESS and state['alt'] > CRUISE_ALT - 2:
                break
            await asyncio.sleep(0.3)
        else:
            raise TimeoutError('mission never progressed far enough to test resume')

        out['progress_at_hold'] = state['prog']

        # --- degrade GPS -> HOLD
        ev_hold = await engine.handle_gps_degraded(GpsFixType.NO_FIX, 99.0, SPEED)
        out['hold_outcome'] = ev_hold.outcome
        t_h = time.monotonic()
        while time.monotonic() - t_h < 15 and state['mode'] != 'HOLD':
            await asyncio.sleep(0.2)
        out['reached_hold_mode'] = state['mode'] == 'HOLD'
        await asyncio.sleep(6)                       # let it actually stop
        out['speed_in_hold'] = round(state['speed'], 3)
        out['stopped_in_hold'] = state['speed'] < 0.6

        # --- resume: the call under test
        t_mark = time.monotonic()
        ev = await engine.resume_after_gps_recovery()
        out['resume_outcome'] = ev.outcome
        t_r = time.monotonic()
        while time.monotonic() - t_r < RESUME_CONFIRM_TIMEOUT_S and state['mode'] != 'MISSION':
            await asyncio.sleep(0.2)
        out['reached_mission_mode'] = state['mode'] == 'MISSION'
        out['t_to_mission_mode_s'] = round(time.monotonic() - t_r, 2)
        out['progress_after_resume'] = state['prog']

        # --- did it actually fly again, and did it resume or restart?
        # Stronger evidence than 'it moved': the mission must actually run
        # to completion FROM the resumed point. A restart-from-zero would
        # also 'move', so reaching the final item is what distinguishes them.
        t_f = time.monotonic()
        while time.monotonic() - t_f < 120 and state['prog'] < 3:
            await asyncio.sleep(0.5)
        out['reached_final_waypoint'] = state['prog'] >= 3
        out['t_to_final_wp_s'] = round(time.monotonic() - t_f, 1)
        out['min_progress_after_resume'] = min(p for (_a,_b,_c,_d,p) in trace if _a >= round(t_mark - T0['v'], 1)) if trace else -1
        out['travelled_after_resume_m'] = round(travelled_since(t_mark), 1)
        out['moving_again'] = out['travelled_after_resume_m'] > 20.0
        out['progress_at_end'] = state['prog']
        # restart would drive progress back toward 0/1; resume keeps it >= hold index
        out['restarted_from_zero'] = out['progress_after_resume'] < out['progress_at_hold']

        out['ok'] = (ev_hold.outcome == 'hold' and out['reached_hold_mode']
                     and out['stopped_in_hold'] and ev.outcome == 'resumed'
                     and out['reached_mission_mode'] and out['moving_again']
                     and out['reached_final_waypoint'] and not out['restarted_from_zero'])
    except Exception as e:
        out['error'] = f'{type(e).__name__}: {e}'
    finally:
        for t_ in tasks:
            t_.cancel()
    out['trace'] = trace
    print('RESULT ' + json.dumps(out))

asyncio.run(main())
