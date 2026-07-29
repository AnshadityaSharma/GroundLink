#!/usr/bin/env python3
"""Live-SITL check: the REAL end-to-end battery-critical failure-injection
path -- not handle_battery_critical() called manually (that was D13/D16),
but the actual trigger chain the evaluation will use:

    sim/failure_injection.apply_battery_drain_scenario()
        -> real PX4 battery drain over time
        -> constraint_monitor.ConstraintMonitor.watch() observes real telemetry
        -> first BATTERY_CRITICAL ViolationEvent
        -> replanning_engine.handle_battery_critical() fed the REAL percent
           from that event, not a hardcoded test value
        -> firmware_link executes RTL

D19 (decisions.md) found SIM_BAT_MIN_PCT is a FLOOR reached through real-time
drain, not an instant set -- by reading BatterySimulator.cpp directly. Battery
starts at exactly 100% the instant the vehicle arms and drains linearly over
SIM_BAT_DRAIN seconds (default 60), floored at target_percent. So with this
module's BATTERY_DRAIN_CRITICAL (target 12%), constraint_monitor should see
the battery cross the 15% CRITICAL threshold at roughly
(100-15)/100*60 = 51s after arming -- this script does not assume that
number, it measures it.
"""
import asyncio, json, math, pathlib, sys, time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from constraint_monitor.events import ViolationKind
from constraint_monitor.monitor import ConstraintMonitor, Thresholds
from firmware_link.mavsdk_client import GroundLinkVehicle
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from replanning_engine.engine import ReplanningEngine, EngineConfig
from sim.failure_injection.scenarios import BATTERY_DRAIN_CRITICAL, apply_battery_drain_scenario

CRUISE_ALT, SPEED = 15.0, 5.0
LEG = 0.0035          # ~390m/leg at this latitude -> ~78s cruise per leg
N_LEGS = 3
DETECT_TIMEOUT_S = 100.0
RTL_CONFIRM_TIMEOUT_S = 20.0
POST_RTL_WATCH_S = 15.0   # just confirm it's genuinely returning, not the full landing (that's D16's job)

state = {'lat': None, 'lon': None, 'alt': 0.0, 'mode': None}
trace = []
T0 = {'v': 0.0}
ARM_T = {'v': None}

def dist_m(a, b, c, d):
    R = 6371000.0
    p1, p2 = math.radians(a), math.radians(c)
    dp, dl = math.radians(c - a), math.radians(d - b)
    x = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R*math.asin(math.sqrt(x))

async def pump_pos(v):
    async for p in v.drone.telemetry.position():
        state['lat'], state['lon'], state['alt'] = p.latitude_deg, p.longitude_deg, p.relative_altitude_m

async def pump_mode(v):
    async for m in v.flight_mode_stream():
        state['mode'] = m

async def pump_trace(home):
    while True:
        await asyncio.sleep(2.0)
        if state['lat'] is not None:
            d = dist_m(home[0], home[1], state['lat'], state['lon'])
            trace.append((round(time.monotonic()-T0['v'],1), round(d,1), round(state['alt'],1), state['mode']))

async def main():
    out = {'ok': False}
    v = GroundLinkVehicle()
    await v.connect(timeout_s=60)
    await v.wait_ready_to_arm(timeout_s=90)

    async for h in v.drone.telemetry.home():
        home = (h.latitude_deg, h.longitude_deg); break
    out['home'] = [round(home[0], 6), round(home[1], 6)]

    mission = Mission(name='battery_scenario', waypoints=[
        Waypoint(home[0], home[1], CRUISE_ALT, kind=WaypointKind.TAKEOFF, speed_m_s=SPEED),
    ] + [
        Waypoint(home[0] + (i+1) * LEG, home[1], CRUISE_ALT, speed_m_s=SPEED) for i in range(N_LEGS)
    ])
    await v.upload_mission(mission)

    # Apply the fault BEFORE arming: the drain clock starts at arm
    # regardless of when the param was set (BatterySimulator.cpp forces
    # 100% while disarmed), so this just needs to happen sometime before.
    await apply_battery_drain_scenario(v, BATTERY_DRAIN_CRITICAL)
    out['scenario'] = {'target_percent': BATTERY_DRAIN_CRITICAL.target_percent}

    await v.arm()
    ARM_T['v'] = time.monotonic()
    T0['v'] = ARM_T['v']

    tasks = [asyncio.create_task(f) for f in (pump_pos(v), pump_mode(v), pump_trace(home))]
    try:
        await v.start_mission()
        engine = ReplanningEngine(v, EngineConfig())
        engine.set_active_mission(mission)
        asyncio.create_task(engine.track_mission_progress())

        monitor = ConstraintMonitor(Thresholds())  # defaults: critical=15%

        # This IS the real trigger path: constraint_monitor.watch() over the
        # actual live telemetry stream, not a manually-constructed event.
        detected = None
        async with asyncio.timeout(DETECT_TIMEOUT_S):
            async for event in monitor.watch(v.telemetry_stream()):
                if event.kind == ViolationKind.BATTERY_CRITICAL:
                    detected = event
                    break
        if detected is None:
            raise RuntimeError('constraint_monitor never reported BATTERY_CRITICAL')

        out['detected_percent'] = detected.details['remaining_percent']
        out['t_to_detect_s'] = round(time.monotonic() - ARM_T['v'], 1)

        # Feed the REAL detected value into the engine -- not a hardcoded
        # test percent -- exactly as the real evaluation loop would.
        ev = await engine.handle_battery_critical(detected.details['remaining_percent'])
        out['event_outcome'] = ev.outcome
        t_cmd = time.monotonic()

        while time.monotonic() - t_cmd < RTL_CONFIRM_TIMEOUT_S and state['mode'] != 'RETURN_TO_LAUNCH':
            await asyncio.sleep(0.2)
        out['reached_rtl_mode'] = state['mode'] == 'RETURN_TO_LAUNCH'
        out['t_to_rtl_mode_s'] = round(time.monotonic() - t_cmd, 1)

        d0 = dist_m(home[0], home[1], state['lat'], state['lon'])
        await asyncio.sleep(POST_RTL_WATCH_S)
        d1 = dist_m(home[0], home[1], state['lat'], state['lon'])
        out['dist_at_rtl_confirm_m'] = round(d0, 1)
        out['dist_after_watch_m'] = round(d1, 1)
        out['returning'] = d1 < d0 - 5.0   # genuinely closing distance, not noise

        out['ok'] = (out['event_outcome'] == 'rtl' and out['reached_rtl_mode'] and out['returning']
                     and 8.0 < out['detected_percent'] <= 15.0)
    except Exception as e:
        out['error'] = f'{type(e).__name__}: {e}'
    finally:
        for t_ in tasks:
            t_.cancel()
    out['trace'] = trace
    print('RESULT ' + json.dumps(out))

asyncio.run(main())
