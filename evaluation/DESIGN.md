# evaluation — Design

**Status**: design only, no implementation yet. Written for review before any code is written, same convention as `replanning_engine/DESIGN.md` (see `context.md`). Nothing below has been run. Open questions that need a decision before implementation are collected at the end, not scattered — read those before the rest if you're short on time.

## What this is answering

`context.md`'s own novelty claim: *"Evaluation directly compares mission completion / area coverage between (a) baseline RTL-on-failure behavior and (b) GroundLink's adaptive replanning, under identical injected failure scenarios in SITL."* Everything verified so far (`decisions.md` D12-D21) proves the adaptive side works. This proves it's *better than the thing it's replacing*, under the same injected conditions — the actual evaluation-section deliverable.

---

## 1. The baseline toggle

**One config flag on the existing `EngineConfig`, not a second engine or a parallel codepath**, per direction:

```python
@dataclass
class EngineConfig:
    ...
    adaptive_replanning_enabled: bool = True
```

**What changes when it's `False`**: nothing about *detection*. `ConstraintMonitor`'s thresholds, the no-fly-zone application-level announcement, and the moment each `handle_*()` is called all stay identical between conditions — that's what makes "identical injected failure scenarios" (context.md's phrase) actually true, rather than two differently-timed experiments. What changes is only the *response*: instead of computing a reroute, a slow-down, a hold, or a severity-appropriate RTL/land split, every trigger that would have produced anything other than "no action needed" collapses to one thing — `vehicle.return_to_launch()`. This is deliberately the plainest possible reading of "baseline RTL-on-failure" from context.md's own problem statement (*"most systems fall back to a crude failsafe (Return-to-Launch) that abandons the mission entirely"*) — one uniform response, no severity tiers, no pathfinding.

Concretely, each `handle_*()` gets a guard near its top, after the pure decision function has run:

```python
if action_would_do_something and not self.config.adaptive_replanning_enabled:
    await self.vehicle.return_to_launch()
    return self._event(trigger, reason, "baseline_rtl", remaining, [])
```

`"baseline_rtl"` is a distinct outcome string from `"rtl"`/`"rerouted"`/`"hold"`/`"slowed_down"`/`"land_immediately"` so aggregation can tell which condition produced a result without needing a separate log field. The three pure decision functions (`decide_battery_response`, `decide_gps_response`, `reroute_around_no_fly_zones`) are untouched — baseline still calls them (so "would this even have triggered a response" stays identical), it just discards the *result* of the decision and substitutes RTL. This is additive to `engine.py` only; `battery_response.py`, `gps_response.py`, `reroute.py`, `no_fly_zone.py` don't change at all.

**Reusing the existing trial scripts, not writing three new ones**: `battery_scenario_trial.py` / `gps_scenario_trial.py` / `nfz_scenario_trial.py` already build `ReplanningEngine(vehicle, EngineConfig())`. They gain one CLI arg (`--baseline`, default off) that flips that one field, and their pass/fail assertions get loosened to accept either `"rtl"`/`"rerouted"`/etc. (adaptive) or `"baseline_rtl"` (baseline) as a valid outcome — the scripts' job shifts from "assert the adaptive-specific behavior happened" to "record the metrics below, whichever condition is active." No new trial infrastructure.

---

## 2. Metrics, per scenario

All three need **mission completion** as a shared metric (context.md asks for it directly). Two scenarios also get a scenario-specific metric.

### Mission completion % — measured against the *original* mission, independent of internal replan bookkeeping

Naively reading `mission_progress()`'s index after a reroute doesn't work as a completion metric: `_execute_handoff` uploads a *new* mission (detour + untouched tail) and resets `_current_index` to 0 against it, so the index loses its mapping back to the original waypoint list. Baseline has the opposite problem — after RTL, `mission_progress` stops meaning anything at all (the vehicle is flying home, not through the mission).

So completion is measured **geometrically and independently of both**, the same way D21's zone-avoidance check was done — off the recorded position trace, not off internal state:

> **Completion % = (count of original-mission waypoints whose location the vehicle came within `acceptance_radius_m` of, at any point during the trial) / (total original waypoints).**

This works identically for both conditions with no special-casing: adaptive's detour waypoints don't count (they're not in the original mission — correctly, since the metric is "how much of the *planned* mission got done"), but original waypoints reached *after* a detour still count, because it's a pure position check against the original list, not a progress-index read. Baseline naturally scores low on this (it heads home instead of continuing), which is the entire point of the comparison.

### Time-to-safe-recovery — battery and GPS scenarios

> **Time-to-safe-recovery = (timestamp `ConstraintMonitor` first reports the violation) → (timestamp the vehicle is confirmed in a safe state).**

"Safe state" per scenario:
- **Battery**: `FlightMode` confirmed `RETURN_TO_LAUNCH` or `LAND` (whichever the response chose) *and* independently confirmed actually acting on it — distance-to-home shrinking for RTL (same check as D20), descent for LAND. Reuses D16/D20's already-proven confirmation logic; no new detection code.
- **GPS**: `FlightMode` confirmed `HOLD` *and* ground speed confirmed near-zero (adaptive) — or, for baseline, whatever `RETURN_TO_LAUNCH` produces under degraded GPS, which is explicitly **not assumed to work** (see open question 2 below).

### Coverage % — no-fly-zone scenario only

Context.md asks for "area/coverage completed" specifically for this scenario. Genuine swept-*area* coverage (actual polygon area covered, not just waypoint count) isn't something any existing module computes — `mission_planner/grid_coverage.py` generates lawnmower waypoints but doesn't track covered area. Building that is real new geometry work. Proposed for now, flagged as open question 3 below: **coverage % = mission-completion % (as defined above) computed against a lawnmower-generated survey mission instead of the simple 4-waypoint point-to-point mission D21 used** — i.e., same metric, applied to a mission shape where "how many of the planned sweep points got covered" is a meaningful proxy for area coverage, using `mission_planner.grid_coverage.generate_lawnmower_mission()` (already built, already tested) with the no-fly zone carved out of part of the swept area.

---

## 3. Sample size and scenarios under test

Matching the 5-trial discipline already established for every other measurement in this project (D14-D21):

| scenario | conditions | trials each | total |
|---|---|---|---|
| battery-critical (12%, existing case) | adaptive, baseline | 5 | 10 |
| GPS-degradation (existing case) | adaptive, baseline | 5 | 10 |
| no-fly-zone (lawnmower mission, if approved — see open question 3) | adaptive, baseline | 5 | 10 |

**30 trials total**, no isolated-run-first step this time — the isolated-then-batch discipline earlier in this project was for *verifying a piece of code works at all*; this phase is re-running already-verified mechanics under a config toggle to collect comparison numbers, so going straight to 5-trial batches per condition is the same rigor without the redundant first step. Each trial is a fresh SITL instance launched as an independent process invocation (D14), with `dataman` and `parameters.bson` reset before every launch (D19). Rough time budget: each trial is ~60-150s of flight plus ~30-40s SITL launch/reset overhead, so 30 trials is roughly 90-120 minutes of wall-clock trial execution, run sequentially (SITL trials can't be parallelized — one SITL instance at a time, per the whole project's established constraint).

This does **not** include a possible fourth battery sub-case (severe/<8%) — see open question 1, which would add 10 more trials (~30-40 more minutes) if approved.

---

## 4. Aggregation and presentation

A results table, not raw logs, per direction. Each trial script already prints one `RESULT {json}` line to stdout (established convention since D15) and results get appended to a per-scenario/condition file under `~/replan_trial_results/` (also already established). New: a small `evaluation/aggregate_results.py` that reads those files and produces one markdown table per scenario:

```
| scenario | condition | n | completion % (mean ± sd) | time-to-safe-recovery (mean ± sd) |
|---|---|---|---|---|
| battery-critical | adaptive | 5 | ... | ... |
| battery-critical | baseline | 5 | ... | ... |
```

Output goes into `evaluation/RESULTS.md` (generated, not hand-written — regenerating it re-runs the same aggregation script over whatever's in `~/replan_trial_results/`, so it's reproducible from the raw JSON rather than transcribed by hand). This is the one new piece of code beyond the `EngineConfig` flag and the trial-script CLI arg — a pure aggregation script, no SITL/MAVSDK involved, easy to unit-test against fixture JSON if useful.

---

## Open questions — need a decision before implementation

1. **Should the battery scenario include a second, more severe sub-case?** At 12% (the existing D20 case), *both* conditions independently land on RTL (12% is between `land_immediately_below_percent=8` and `rtl_below_percent=20` either way) — so time-to-safe-recovery would likely come out near-identical between conditions, and completion % would be the only real differentiator. A second case below 8% (e.g. 5%) would make adaptive choose `LAND_IMMEDIATELY` (fast, no transit) while baseline still blindly attempts a full RTL (climb + transit + land, ~60s per D16) — directly demonstrating context.md's own stated concern that "RTL itself might not complete in time." Recommend adding it; it costs ~10 more trials (~30-40 min) and is the more interesting result. Your call.

2. **What should baseline actually do when the failure *is* GPS degradation?** Baseline's uniform response is `return_to_launch()` — but RTL is itself a GPS-dependent maneuver, and the injected failure in this scenario is GPS going to `NO_FIX` (D19/D20/D21: confirmed instant, not gradual). It's genuinely unknown, not assumed, whether PX4 will actually execute a normal RTL transit under `NO_FIX` in SITL (it may reject the command, coast on a stale/dead-reckoned local-position estimate for a few seconds, or behave some other way) — this needs to be *observed and reported honestly*, including "baseline's RTL attempt itself failed/never returned" as a legitimate, meaningful result if that's what happens, not treated as a test bug to fix. Flagging so it isn't silently assumed to mirror the battery-scenario RTL behavior. No decision needed here, just confirming this is to be measured, not predicted.

3. **Should the no-fly-zone comparison use a lawnmower/coverage-grid mission instead of D21's simple 4-waypoint mission?** "Coverage" as a concept only really means something for an area-sweeping mission, not a point-to-point one — reusing `generate_lawnmower_mission()` (already built and tested, no new pathfinding code) would produce a more meaningful coverage number, at the cost of building one new mission-construction helper for this scenario's trial script (small, reuses existing tested code) and picking sensible boundary/spacing values so a mid-mission zone genuinely blocks part of the sweep. Recommend yes. If you'd rather keep the existing 4-waypoint mission for consistency with D21's already-verified numbers, completion % alone (metric in section 2) still works fine as the reported result, just without a distinct "coverage" framing.

4. **Is 5 trials/condition the right size, or should this be trimmed given it's 30 (or 40) total trials?** Flagging the time cost plainly since this is now a report deliverable, not a quick reverification — recommend keeping 5 for the same statistical reasons as every prior 5-trial batch in this project, but if wall-clock time is a real constraint, 3/condition (18-24 trials total) is a defensible fallback consistent with the "3 is enough, not a full investigation" latitude already used once this session (D18's re-verification).
