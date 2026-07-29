#!/usr/bin/env python3
"""Live-SITL check: does ReplanningEngine.handle_gps_degraded()'s SLOW_DOWN
path (action.set_current_speed) actually reduce the vehicle's real ground speed?

Measurement discipline is the whole point of this script:

  * speed_before is never recorded until the vehicle is CONFIRMED in steady
    cruise -- past climb-out (horizontal speed is ~0 for the first ~17s while
    it climbs) and holding a stable ground speed.
  * Waypoints are built relative to the vehicle's ACTUAL home position read
    from telemetry, not hardcoded coordinates. An earlier version assumed the
    PX4 SITL default home; the legs came out far shorter than intended and the
    vehicle began decelerating into waypoint 1 in the middle of the 'before'
    window, making speed_before meaningless (3.8 m/s +/- 1.4).
  * The before-window is validated AFTER the fact: if the vehicle was not
    genuinely steady across the whole window (stdev too high, or mission
    progress advanced -- i.e. it hit a waypoint and turned), the sample is
    rejected and retried rather than reported.
"""
import asyncio, json, pathlib, statistics, sys, time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from firmware_link.mavsdk_client import GroundLinkVehicle
from firmware_link.telemetry import GpsFixType
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from replanning_engine.engine import ReplanningEngine, EngineConfig

CRUISE_ALT = 15.0
NOMINAL_SPEED = 5.0
TARGET_FRACTION = 0.5        # GpsResponseThresholds.slow_down_speed_fraction
LEG_DEG_LAT = 0.0180         # ~2km per leg: must outlast climb-out + both
                             # measurement windows, since PX4 re-applies the
                             # mission item speed at each waypoint transition
STEADY_WINDOW_S = 5.0
STEADY_STDEV_MAX = 0.25
MEASURE_WINDOW_S = 6.0
MEASURE_STDEV_MAX = 0.30     # post-hoc validity gate on each window
SETTLE_AFTER_CMD_S = 12.0

samples, trace = [], []
alt = {'v': 0.0}
prog = {'cur': 0}
T0 = {'v': 0.0}

async def pump_speed(v):
    async for s in v.ground_speed_stream():
        samples.append((time.monotonic(), s))

async def pump_alt(v):
    async for p in v.drone.telemetry.position():
        alt['v'] = p.relative_altitude_m

async def pump_prog(v):
    async for cur, _tot in v.mission_progress_stream():
        prog['cur'] = cur

async def pump_trace():
    while True:
        await asyncio.sleep(0.5)
        w = window(time.monotonic() - 0.5, time.monotonic())
        if w:
            trace.append((round(time.monotonic() - T0['v'], 1),
                          round(statistics.mean(w), 2), round(alt['v'], 1), prog['cur']))

def window(t0, t1):
    return [s for (t, s) in samples if t0 <= t <= t1]

async def wait_for_steady_cruise(timeout_s=180.0):
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        await asyncio.sleep(0.5)
        if alt['v'] < CRUISE_ALT - 1.5:
            continue
        now = time.monotonic()
        w = window(now - STEADY_WINDOW_S, now)
        if len(w) < 10:
            continue
        if statistics.mean(w) > 1.0 and statistics.pstdev(w) < STEADY_STDEV_MAX:
            return {'mean': round(statistics.mean(w), 3), 'stdev': round(statistics.pstdev(w), 3),
                    'alt': round(alt['v'], 2), 't_to_steady_s': round(now - start, 2)}
    raise TimeoutError('vehicle never reached steady cruise')

async def measure(label, attempts=3):
    """Average ground speed over a window, rejecting the window outright if
    the vehicle wasn't steady across all of it."""
    for i in range(attempts):
        p0, t0 = prog['cur'], time.monotonic()
        await asyncio.sleep(MEASURE_WINDOW_S)
        w = window(t0, time.monotonic())
        mean, sd = statistics.mean(w), statistics.pstdev(w)
        if sd < MEASURE_STDEV_MAX and prog['cur'] == p0:
            return {'mean': round(mean, 3), 'stdev': round(sd, 3), 'n': len(w),
                    'alt': round(alt['v'], 2), 'attempt': i + 1}
    raise RuntimeError(f'{label}: never got a steady window (last stdev={sd:.3f}, '
                       f'progress {p0}->{prog["cur"]})')

async def main():
    out = {'ok': False}
    v = GroundLinkVehicle()
    await v.connect(timeout_s=60)
    await v.wait_ready_to_arm(timeout_s=90)

    # Read the REAL home position rather than assuming the SITL default.
    async for h in v.drone.telemetry.home():
        home_lat, home_lon = h.latitude_deg, h.longitude_deg
        break
    out['home'] = [round(home_lat, 6), round(home_lon, 6)]

    mission = Mission(name='speed_check', waypoints=[
        Waypoint(home_lat, home_lon, CRUISE_ALT, kind=WaypointKind.TAKEOFF, speed_m_s=NOMINAL_SPEED),
        Waypoint(home_lat + LEG_DEG_LAT, home_lon, CRUISE_ALT, speed_m_s=NOMINAL_SPEED),
        Waypoint(home_lat + 2 * LEG_DEG_LAT, home_lon, CRUISE_ALT, speed_m_s=NOMINAL_SPEED),
    ])
    await v.upload_mission(mission)
    await v.arm()

    T0['v'] = time.monotonic()
    tasks = [asyncio.create_task(f) for f in
             (pump_speed(v), pump_alt(v), pump_prog(v), pump_trace())]
    try:
        await v.start_mission()
        engine = ReplanningEngine(v, EngineConfig())
        engine.set_active_mission(mission)

        out['steady_cruise'] = await wait_for_steady_cruise()

        before = await measure('speed_before')
        out['before'] = before

        ev = await engine.handle_gps_degraded(GpsFixType.FIX_3D, 3.5, NOMINAL_SPEED)
        out['event_outcome'] = ev.outcome
        await asyncio.sleep(SETTLE_AFTER_CMD_S)

        after = await measure('speed_after')
        out['after'] = after

        expected = NOMINAL_SPEED * TARGET_FRACTION
        out['expected_after'] = expected
        out['ratio'] = round(after['mean'] / before['mean'], 3)
        out['abs_err_vs_expected'] = round(abs(after['mean'] - expected), 3)
        out['ok'] = (ev.outcome == 'slowed_down'
                     and after['mean'] < before['mean'] - 0.75
                     and out['abs_err_vs_expected'] < 0.75)
    except Exception as e:
        out['error'] = f'{type(e).__name__}: {e}'
    finally:
        for t in tasks:
            t.cancel()
    out['trace'] = trace
    print('RESULT ' + json.dumps(out))

asyncio.run(main())
