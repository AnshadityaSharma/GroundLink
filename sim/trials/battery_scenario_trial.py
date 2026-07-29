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
        -> firmware_link executes RTL, LAND, or (baseline) always RTL

D19 (decisions.md) found SIM_BAT_MIN_PCT is a FLOOR reached through real-time
drain, not an instant set -- by reading BatterySimulator.cpp directly.

evaluation/DESIGN.md: this script now also drives the baseline-vs-adaptive
comparison. --baseline sets EngineConfig.adaptive_replanning_enabled=False,
which (per engine.py) collapses the response to a plain RTL regardless of
severity. --severity moderate|severe picks BATTERY_DRAIN_CRITICAL (12%
floor -- both conditions land on RTL, since 12% sits between the 8%/20%
tiers either way) or BATTERY_DRAIN_SEVERE (5% floor via a fast drain, so
first detection is already below the 8% land-immediately tier -- adaptive
diverges to LAND_IMMEDIATELY here while baseline still always RTLs).

Records, regardless of condition/outcome:
- completion %: measured geometrically against the ORIGINAL mission's
  non-takeoff waypoints (position trace vs. each waypoint's own
  acceptance_radius_m), independent of internal mission_progress bookkeeping
  -- robust to baseline's RTL making mission_progress meaningless.
- time-to-safe-recovery: detection timestamp -> actual disarm (D16's proven
  full-completion check), for both LAND and RTL responses. An earlier
  version measured "mode confirmed + a fixed 15s watch window" instead,
  which made every trial land near the same ~16s regardless of response
  type -- masking exactly the LAND-vs-RTL speed divergence the severe
  battery case exists to demonstrate. Fixed in D22.
"""
import argparse
import asyncio
import json
import math
import pathlib
import statistics
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from constraint_monitor.events import ViolationKind
from constraint_monitor.monitor import ConstraintMonitor, Thresholds
from firmware_link.mavsdk_client import GroundLinkVehicle
from mission_planner.waypoint import Mission, Waypoint, WaypointKind
from replanning_engine.engine import ReplanningEngine, EngineConfig
from sim.failure_injection.scenarios import (
    BATTERY_DRAIN_CRITICAL,
    BATTERY_DRAIN_SEVERE,
    apply_battery_drain_scenario,
)

CRUISE_ALT, SPEED = 15.0, 5.0
LEG = 0.0035          # ~390m/leg at this latitude -> ~78s cruise per leg
N_LEGS = 3
DETECT_TIMEOUT_S = 100.0
SAFE_CONFIRM_TIMEOUT_S = 30.0
# D22 correction: time-to-safe-recovery was originally "mode confirmed +
# a fixed 15s watch", which meant every trial landed near the same ~16s
# number regardless of whether the response was LAND (fast) or a full RTL
# (climb+transit+land, ~60s per D16) -- masking exactly the divergence the
# severe battery case exists to measure. Now waits for actual disarm
# (D16's proven full-completion check) for both response types.
DISARM_TIMEOUT_S = 150.0
STEADY_WINDOW_S = 5.0
STEADY_STDEV_MAX = 0.25

state = {'lat': None, 'lon': None, 'alt': 0.0, 'mode': None, 'speed': 0.0, 'armed': True}
speed_samples = []   # (t_rel, speed) -- for the severe case's steady-cruise gate
positions = []   # (t_rel, lat, lon) -- every position sample, for completion-% check
trace = []
T0 = {'v': 0.0}
ARM_T = {'v': None}


def dist_m(a, b, c, d):
    R = 6371000.0
    p1, p2 = math.radians(a), math.radians(c)
    dp, dl = math.radians(c - a), math.radians(d - b)
    x = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(x))


async def pump_pos(v):
    async for p in v.drone.telemetry.position():
        state['lat'], state['lon'], state['alt'] = p.latitude_deg, p.longitude_deg, p.relative_altitude_m
        positions.append((time.monotonic() - T0['v'], p.latitude_deg, p.longitude_deg))


async def pump_mode(v):
    async for m in v.flight_mode_stream():
        state['mode'] = m


async def pump_armed(v):
    async for a in v.drone.telemetry.armed():
        state['armed'] = a


async def pump_speed(v):
    async for s in v.ground_speed_stream():
        state['speed'] = s
        speed_samples.append((time.monotonic() - T0['v'], s))


def _speed_window(t0, t1):
    return [s for (t, s) in speed_samples if t0 <= t <= t1]


async def wait_for_steady_cruise(timeout_s=120.0):
    """Same gate as speed_trial.py/gps_scenario_trial.py: don't call this
    'mid-flight' until climb-out is genuinely over and speed has settled."""
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        await asyncio.sleep(0.5)
        if state['alt'] < CRUISE_ALT - 1.5:
            continue
        now_rel = time.monotonic() - T0['v']
        w = _speed_window(now_rel - STEADY_WINDOW_S, now_rel)
        if len(w) < 10:
            continue
        if statistics.mean(w) > 1.0 and statistics.pstdev(w) < STEADY_STDEV_MAX:
            return round(time.monotonic() - start, 2)
    raise TimeoutError('vehicle never reached steady cruise')


async def pump_trace(home):
    while True:
        await asyncio.sleep(2.0)
        if state['lat'] is not None:
            d = dist_m(home[0], home[1], state['lat'], state['lon'])
            trace.append((round(time.monotonic() - T0['v'], 1), round(d, 1), round(state['alt'], 1), state['mode']))


def completion_percent(mission: Mission) -> dict:
    """Fraction of the ORIGINAL mission's non-takeoff waypoints the vehicle
    came within acceptance_radius_m of, at any point -- independent of
    mission_progress bookkeeping, which baseline's RTL makes meaningless."""
    targets = [wp for wp in mission.waypoints if wp.kind != WaypointKind.TAKEOFF]
    reached = 0
    for wp in targets:
        hit = any(dist_m(wp.latitude_deg, wp.longitude_deg, la, lo) <= wp.acceptance_radius_m for (_t, la, lo) in positions)
        reached += 1 if hit else 0
    return {'reached': reached, 'total': len(targets), 'percent': round(100.0 * reached / len(targets), 1) if targets else 0.0}


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', action='store_true')
    parser.add_argument('--severity', choices=['moderate', 'severe'], default='moderate')
    args = parser.parse_args()

    scenario = BATTERY_DRAIN_SEVERE if args.severity == 'severe' else BATTERY_DRAIN_CRITICAL

    out = {'ok': False, 'condition': 'baseline' if args.baseline else 'adaptive', 'severity': args.severity}
    v = GroundLinkVehicle()
    await v.connect(timeout_s=60)
    await v.wait_ready_to_arm(timeout_s=90)

    async for h in v.drone.telemetry.home():
        home = (h.latitude_deg, h.longitude_deg); break
    out['home'] = [round(home[0], 6), round(home[1], 6)]

    mission = Mission(name='battery_scenario', waypoints=[
        Waypoint(home[0], home[1], CRUISE_ALT, kind=WaypointKind.TAKEOFF, speed_m_s=SPEED),
    ] + [
        Waypoint(home[0] + (i + 1) * LEG, home[1], CRUISE_ALT, speed_m_s=SPEED) for i in range(N_LEGS)
    ])
    await v.upload_mission(mission)

    if args.severity == 'moderate':
        # Apply BEFORE arming: the drain clock starts at arm regardless of
        # when the param was set (BatterySimulator.cpp forces 100% while
        # disarmed), so this just needs to happen sometime before. Matches
        # D20's already-verified timing (~52-54s to detection).
        await apply_battery_drain_scenario(v, scenario)
    out['scenario'] = {'target_percent': scenario.target_percent, 'drain_interval_s': scenario.drain_interval_s}

    await v.arm()
    ARM_T['v'] = time.monotonic()
    T0['v'] = ARM_T['v']

    tasks = [asyncio.create_task(f) for f in (pump_pos(v), pump_mode(v), pump_speed(v), pump_armed(v), pump_trace(home))]
    try:
        await v.start_mission()
        engine = ReplanningEngine(v, EngineConfig(adaptive_replanning_enabled=not args.baseline))
        engine.set_active_mission(mission)
        asyncio.create_task(engine.track_mission_progress())

        if args.severity == 'severe':
            # Apply only after steady cruise -- see scenarios.py's comment
            # on BATTERY_DRAIN_SEVERE for why: applying this fast a drain
            # before arm can crash the battery while still climbing out,
            # before there's any real distance/altitude to recover from.
            out['t_to_steady_cruise_s'] = await wait_for_steady_cruise()
            out['alt_at_injection'] = round(state['alt'], 2)
            await apply_battery_drain_scenario(v, scenario)

            # scenarios.py's drain_interval_s=1.5 was meant to make the
            # 100%->5% traversal fast enough that ConstraintMonitor's FIRST
            # <=15% crossing would already be below the 8% land-immediately
            # tier. Empirically (this trial, twice) it wasn't reliable --
            # PX4's battery telemetry publishes often enough that samples at
            # 12% and 10% (both still in the RTL-choosing 8-20% band) were
            # caught instead of skipping straight past them. Rather than
            # keep gambling on sample-rate timing, wait deterministically
            # for the fault to actually finish taking effect -- poll real
            # telemetry directly until the reported percentage has reached
            # the target floor -- before starting to watch for the
            # violation. This still goes through the same real
            # ConstraintMonitor.check() logic below; it just doesn't depend
            # on catching an exact, unreliable millisecond of crossing.
            async for b in v.drone.telemetry.battery():
                if b.remaining_percent <= scenario.target_percent + 1.0:
                    out['battery_at_floor_confirmed'] = round(b.remaining_percent, 1)
                    break

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
        t_detect = time.monotonic()
        out['t_to_detect_s'] = round(t_detect - ARM_T['v'], 1)

        # Feed the REAL detected value into the engine -- not a hardcoded
        # test percent -- exactly as the real evaluation loop would.
        ev = await engine.handle_battery_critical(detected.details['remaining_percent'])
        out['event_outcome'] = ev.outcome
        t_cmd = time.monotonic()

        if ev.outcome == 'land_immediately':
            while time.monotonic() - t_cmd < SAFE_CONFIRM_TIMEOUT_S and state['mode'] != 'LAND':
                await asyncio.sleep(0.2)
            out['reached_response_mode'] = state['mode'] == 'LAND'
        else:
            # 'rtl' or 'baseline_rtl' -- both physically call return_to_launch()
            while time.monotonic() - t_cmd < SAFE_CONFIRM_TIMEOUT_S and state['mode'] != 'RETURN_TO_LAUNCH':
                await asyncio.sleep(0.2)
            out['reached_response_mode'] = state['mode'] == 'RETURN_TO_LAUNCH'

        # Real completion, not a fixed watch window (D22): wait for the
        # vehicle to actually disarm on its own, same check D16 used to
        # prove full RTL return-and-land. Works for LAND too -- PX4
        # auto-disarms after touchdown either way.
        t_disarm_start = time.monotonic()
        while time.monotonic() - t_disarm_start < DISARM_TIMEOUT_S and state['armed']:
            await asyncio.sleep(0.3)
        out['disarmed'] = not state['armed']
        out['final_alt_m'] = round(state['alt'], 2)
        out['final_dist_to_home_m'] = round(dist_m(home[0], home[1], state['lat'], state['lon']), 1) if state['lat'] is not None else None
        out['safe_confirmed'] = out['reached_response_mode'] and out['disarmed']

        out['t_to_safe_recovery_s'] = round(time.monotonic() - t_detect, 1) if out['safe_confirmed'] else None

        comp = completion_percent(mission)
        out['completion'] = comp

        out['ok'] = out['safe_confirmed'] and (
            (ev.outcome == 'baseline_rtl' and args.baseline)
            or (not args.baseline and ev.outcome in ('rtl', 'land_immediately'))
        )
    except Exception as e:
        out['error'] = f'{type(e).__name__}: {e}'
    finally:
        for t_ in tasks:
            t_.cancel()
    out['trace'] = trace
    print('RESULT ' + json.dumps(out))

asyncio.run(main())
