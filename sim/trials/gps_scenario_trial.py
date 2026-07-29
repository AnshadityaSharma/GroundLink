#!/usr/bin/env python3
"""Live-SITL check: the REAL end-to-end GPS-degradation failure-injection
path -- the actual trigger chain the evaluation will use, not
handle_gps_degraded() called manually (that was D12):

    sim/failure_injection.apply_gps_degradation_scenario()
        -> PX4 SITL flips the simulated GPS fix instantly (D19: confirmed by
           reading SensorGpsSim.cpp -- a hard threshold at 4 satellites, no
           EKF2 involved, no gradual transition)
        -> constraint_monitor.ConstraintMonitor.watch() observes real telemetry
        -> first GPS_FIX_DEGRADED ViolationEvent
        -> replanning_engine.handle_gps_degraded() fed the REAL fix_type/hdop
           read off the live telemetry stream at detection time, not a
           hardcoded test value
        -> firmware_link executes HOLD (fix_type below FIX_3D -> engine
           chooses HOLD, not SLOW_DOWN -- see gps_response.py)

Fault is injected only after the vehicle is confirmed in steady cruise
(same climb-out gate as speed_trial.py, D15) so this is a genuine mid-flight
failure, not one injected during takeoff.
"""
import asyncio, json, pathlib, statistics, sys, time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from constraint_monitor.events import ViolationKind
from constraint_monitor.monitor import ConstraintMonitor, Thresholds
from firmware_link.mavsdk_client import GroundLinkVehicle
from firmware_link.telemetry import GpsFixType
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from replanning_engine.engine import ReplanningEngine, EngineConfig
from sim.failure_injection.scenarios import GPS_DEGRADATION_LOW_SATS, apply_gps_degradation_scenario

CRUISE_ALT, SPEED = 15.0, 5.0
LEG = 0.0035
N_LEGS = 3
STEADY_WINDOW_S = 5.0
STEADY_STDEV_MAX = 0.25
DETECT_TIMEOUT_S = 20.0     # D19: this flip is near-instant, unlike battery
HOLD_CONFIRM_TIMEOUT_S = 15.0

speed_samples = []
state = {'alt': 0.0, 'mode': None, 'speed': 0.0, 'gps_fix': None, 'gps_hdop': None}
T0 = {'v': 0.0}

async def pump_speed(v):
    async for s in v.ground_speed_stream():
        state['speed'] = s
        speed_samples.append((time.monotonic(), s))

async def pump_alt(v):
    async for p in v.drone.telemetry.position():
        state['alt'] = p.relative_altitude_m

async def pump_mode(v):
    async for m in v.flight_mode_stream():
        state['mode'] = m

def window(t0, t1):
    return [s for (t, s) in speed_samples if t0 <= t <= t1]

async def wait_for_steady_cruise(timeout_s=120.0):
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        await asyncio.sleep(0.5)
        if state['alt'] < CRUISE_ALT - 1.5:
            continue
        now = time.monotonic()
        w = window(now - STEADY_WINDOW_S, now)
        if len(w) < 10:
            continue
        if statistics.mean(w) > 1.0 and statistics.pstdev(w) < STEADY_STDEV_MAX:
            return round(now - start, 2)
    raise TimeoutError('vehicle never reached steady cruise')

async def main():
    out = {'ok': False}
    v = GroundLinkVehicle()
    await v.connect(timeout_s=60)
    await v.wait_ready_to_arm(timeout_s=90)

    async for h in v.drone.telemetry.home():
        home = (h.latitude_deg, h.longitude_deg); break
    out['home'] = [round(home[0], 6), round(home[1], 6)]

    mission = Mission(name='gps_scenario', waypoints=[
        Waypoint(home[0], home[1], CRUISE_ALT, kind=WaypointKind.TAKEOFF, speed_m_s=SPEED),
    ] + [
        Waypoint(home[0] + (i+1) * LEG, home[1], CRUISE_ALT, speed_m_s=SPEED) for i in range(N_LEGS)
    ])
    await v.upload_mission(mission)
    await v.arm()

    T0['v'] = time.monotonic()
    tasks = [asyncio.create_task(f) for f in (pump_speed(v), pump_alt(v), pump_mode(v))]
    try:
        await v.start_mission()
        engine = ReplanningEngine(v, EngineConfig())
        engine.set_active_mission(mission)
        asyncio.create_task(engine.track_mission_progress())

        out['t_to_steady_cruise_s'] = await wait_for_steady_cruise()
        out['alt_at_injection'] = round(state['alt'], 2)

        await apply_gps_degradation_scenario(v, GPS_DEGRADATION_LOW_SATS)
        out['scenario'] = {'simulated_num_satellites': GPS_DEGRADATION_LOW_SATS.simulated_num_satellites}
        t_inject = time.monotonic()

        monitor = ConstraintMonitor(Thresholds())  # default min_gps_fix_type=FIX_3D

        # This IS the real trigger path: constraint_monitor.watch() over the
        # actual live telemetry stream. Keep the last raw fix_type/hdop from
        # the very same snapshots so the engine call below uses real,
        # concurrently-observed values, not values reconstructed after the
        # fact from the (fix_type-only) violation event.
        detected = None
        last_snapshot = None
        async with asyncio.timeout(DETECT_TIMEOUT_S):
            async for snapshot in v.telemetry_stream():
                last_snapshot = snapshot
                for event in monitor.check(snapshot):
                    if event.kind == ViolationKind.GPS_FIX_DEGRADED:
                        detected = event
                        break
                if detected is not None:
                    break
        if detected is None:
            raise RuntimeError('constraint_monitor never reported GPS_FIX_DEGRADED')

        out['t_to_detect_s'] = round(time.monotonic() - t_inject, 2)
        out['detected_fix_type'] = detected.details['fix_type']
        out['detected_num_satellites'] = detected.details['num_satellites']

        gps = last_snapshot.gps
        out['live_fix_type'] = gps.fix_type.name
        out['live_hdop'] = gps.hdop

        # Feed the REAL, concurrently-observed fix_type/hdop into the
        # engine -- exactly as the real evaluation loop would.
        ev = await engine.handle_gps_degraded(gps.fix_type, gps.hdop, SPEED)
        out['event_outcome'] = ev.outcome
        t_cmd = time.monotonic()

        while time.monotonic() - t_cmd < HOLD_CONFIRM_TIMEOUT_S and state['mode'] != 'HOLD':
            await asyncio.sleep(0.2)
        out['reached_hold_mode'] = state['mode'] == 'HOLD'
        out['t_to_hold_mode_s'] = round(time.monotonic() - t_cmd, 2)

        await asyncio.sleep(6)  # let it actually stop
        out['speed_in_hold'] = round(state['speed'], 3)
        out['stopped_in_hold'] = state['speed'] < 0.6

        out['ok'] = (ev.outcome == 'hold' and out['reached_hold_mode'] and out['stopped_in_hold']
                     and gps.fix_type < GpsFixType.FIX_3D)
    except Exception as e:
        out['error'] = f'{type(e).__name__}: {e}'
    finally:
        for t_ in tasks:
            t_.cancel()
    print('RESULT ' + json.dumps(out))

asyncio.run(main())
