# replanning_engine — Design

**Status**: design only, no implementation yet. Written for review before any code is written, per project convention (see `context.md` and the Step 4 report). Everything under "Verified against MAVSDK" below was checked against the actually-installed `mavsdk==3.17.2` package in this environment, the same discipline that caught three real bugs during `firmware_link` development (decisions.md D5, D8, D9) — nothing here is assumed from memory or docs.

## Scope and fidelity (as directed, not re-litigated)

All three trigger types get built, at different depths:

1. **No-fly-zone → full reroute.** This is the core deliverable. Grid-based A*, real engineering effort.
2. **Battery-critical → simple safe return.** No pathfinding — a two-tier decision using PX4-native actions.
3. **GPS-degraded → simple mode/speed downgrade.** No pathfinding — a two-tier decision, deliberately not gold-plated.

## Module layout and the decoupling boundary

Per `context.md`'s convention ("keep the replanning logic modular and testable in isolation from the MAVLink/firmware layer") and the pattern already established between `mission_planner` and `firmware_link`: the actual *decision logic* (should we reroute, what's the new path) is pure Python operating on `mission_planner.Waypoint`/`Mission` objects and plain geometry — no MAVSDK import anywhere in it, unit-testable with no SITL running. A separate, thin layer translates decisions into MAVSDK calls.

```
replanning_engine/
  events.py           ReplanEvent — structured log entry (reason, trigger, old/new path,
                       timestamp), the dashboard's future "replanning decisions" feed.
  no_fly_zone.py       NoFlyZone dataclass + path-intersection detection (pure).
  grid_astar.py         Generic occupancy-grid A* (pure, reusable, no domain knowledge
                       of drones or PX4 -- just "grid in, path out").
  reroute.py           No-fly-zone reroute orchestration: builds the grid from a zone +
                       remaining waypoints, runs A*, simplifies, splices back into the
                       mission (pure -- consumes/produces Waypoint/Mission only).
  battery_response.py  Battery-critical decision logic (pure).
  gps_response.py       GPS-degraded decision logic (pure).
  engine.py            Orchestrator: wires ConstraintMonitor's ViolationEvent stream to
                       the handlers above, then to firmware_link execution. This is the
                       only file in the package that touches GroundLinkVehicle.
```

`engine.py` is the seam. Everything above it is testable with plain dataclasses; everything at or below it needs a `GroundLinkVehicle` (real or, for unit tests, a hand-written stand-in implementing the same small interface — not a mock of MAVSDK internals, a substitute for our own wrapper).

---

## Trigger 1: No-fly-zone → full reroute

### How a zone is represented and announced

A `NoFlyZone` is a polygon in lat/lon (mirrors `constraint_monitor.Thresholds.geofence_latlon` — same shapely-backed representation, consistent with the rest of the codebase) plus a label and the timestamp it became active:

```python
@dataclass(frozen=True)
class NoFlyZone:
    boundary_latlon: list[tuple[float, float]]
    label: str
    activated_at_unix_s: float
```

Zones are **announced events**, not telemetry thresholds. This is a deliberate departure from how `constraint_monitor` works today (`ConstraintMonitor.check()` is a pure function of *one* telemetry snapshot). A no-fly zone appearing mid-flight isn't a property of the vehicle's current state — it's new information about the world that has to be checked against the *planned path*, which the vehicle doesn't carry in a `TelemetrySnapshot`. So detection for this trigger lives in `replanning_engine`, not `constraint_monitor`: `constraint_monitor` keeps doing what it does (threshold checks against live telemetry), and a new `NoFlyZone` announcement is a second, independent input to `engine.py` (in the SITL evaluation, this will come from `sim/failure_injection`'s scenario config; on the dashboard side, later, from an operator drawing a polygon).

### Detection: does the zone actually block anything?

On every new `NoFlyZone` announcement, check whether it intersects the vehicle's **remaining path** — not just current position:

```python
def blocks_remaining_path(zone: NoFlyZone, current_position: Position, remaining_waypoints: list[Waypoint]) -> list[int]:
    """Returns indices (into remaining_waypoints) of the leg endpoints whose
    incoming segment intersects the zone. Empty list = zone doesn't affect
    the current plan, no replan needed."""
```

Build the ordered point sequence `[current_position] + remaining_waypoints`, buffer the zone polygon by a safety margin (default: the mission's waypoint `acceptance_radius_m`, since that's already the tolerance the mission is flying to — reuse it rather than invent a second unrelated constant), and test each consecutive segment against the buffered polygon with shapely's `LineString.intersects()`. If nothing intersects, this is a no-op (the zone exists but doesn't matter *yet* — it might if the mission changes later, so it stays tracked, but no replan fires now).

### Identifying "remaining waypoints"

`mission.mission_progress()` (verified present on the installed `mavsdk.mission.Mission` — see "Verified against MAVSDK" below) streams `(current, total)`: the index of the mission item PX4 is currently flying to. `engine.py` keeps the `Mission` object it last uploaded and maps `current` back to that object's `waypoints` list — "remaining waypoints" = `mission.waypoints[current:]`. This requires `engine.py` to track its own "currently active mission" state; there's no way to ask PX4 for the original `Waypoint` objects back (only raw `MissionItem`s via `download_mission()`, which loses the `WaypointKind`/`priority`/`label` metadata our own objects carry) — so the source of truth for "what mission is flying right now" is our own in-memory record, kept in sync by only ever mutating it through `engine.py`'s upload path.

### Pathfinding: grid-based A*

Deliberately the simplest thing that's still correct and reroutes around a polygon: a coarse occupancy grid + textbook A*, not a visibility graph, not RRT, not anything that needs its own subsection to explain in the report.

**1. Grid construction.** Bounding box = the convex hull of `{current_position} ∪ blocked-segment endpoints ∪ zone.boundary_latlon`, padded by a fixed margin (default 50m, generous enough that A* has room to route around the zone's corners without hugging the bounding box edge). Project to local ENU meters using the same equirectangular approximation already in `mission_planner/grid_coverage.py` (`_latlon_to_local_xy`/`_local_xy_to_latlon` — these get promoted to a shared `mission_planner/geo.py` module so both `grid_coverage.py` and `replanning_engine` import the same projection code instead of duplicating it; this is a small refactor of existing tested code, not new risk). Cell size defaults to 10m, but is auto-coarsened if the bounding box would produce more than ~40,000 cells (a 200×200 grid), so a pathological huge zone/waypoint spread can't blow up runtime — this keeps the "coarse" promise even at the edges.

**2. Marking blocked cells.** A cell is blocked if its center point falls inside the buffered zone polygon (same buffer distance as the detection check above, for consistency between "does this block the path" and "where exactly is blocked").

**3. Search.** 8-connected grid (diagonal moves allowed — otherwise paths look artificially blocky and burn more waypoints than necessary). Edge cost = Euclidean distance between cell centers (1× cell size for orthogonal moves, √2× for diagonal). Heuristic = Euclidean distance from cell to goal cell, which is admissible and consistent for this cost model (never overestimates true remaining cost), so standard A* is optimal here — no need to justify a weighted/inadmissible variant.

**4. Simplification.** Raw A* output is one waypoint per grid cell, which is far more than needed. Collapse consecutive collinear-ish points: walk the path, drop any point where the deviation from the straight line between its neighbors is under a tolerance (default: half the grid cell size), which turns a staircase-y grid path into a handful of waypoints that hug the zone's silhouette. This keeps the replanned mission's waypoint count sane for the mission upload and for the dashboard's map view.

**5. Splicing back in.** The detour's start point is `current_position` (or the last un-rerouted waypoint, if the vehicle hasn't reached the blocked segment yet), its end point is the first remaining waypoint *after* the blocked span. The new `Mission` = `[detour waypoints...] + mission.waypoints[end_index:]` — everything after the reconnection point is untouched, which is the "preserve as much of the original mission as possible" requirement from `context.md`.

**6. Edge cases:**
- **No path exists** (zone + bounding box padding fully encloses start or goal): fall back to the battery-critical response's RTL path (abandon the survey, go home) rather than failing silently. Logged as a `ReplanEvent` with `outcome="no_safe_reroute_found"`.
- **Multiple overlapping zones**: union them into one blocked region before rasterizing (shapely `unary_union`) — the grid doesn't care how many source polygons contributed to a blocked cell.
- **Zone announced but vehicle already inside it**: out of scope for the reroute algorithm itself (A* routing *out* of a cell already marked blocked doesn't make sense) — this degrades to the same "no safe reroute, fall back to RTL/hold" path, flagged separately so it's not silently misattributed to "A* found no path" in the log.

---

## Trigger 2: Battery-critical → simple safe return (not full pathfinding, deliberately)

Two-tier, both using PX4-native actions rather than anything we compute:

1. **`ViolationKind.BATTERY_CRITICAL` fires, RTL still safe** (above some `battery_min_for_rtl_percent` threshold, conservatively above whatever `BATTERY_CRITICAL` itself is set to): call `action.return_to_launch()`. PX4 computes the actual return trajectory (climb to RTL altitude, transit home, land) — this already *is* "the shortest safe return path" that `context.md` asks for; there's no value in re-deriving it in our own A*/straight-line code when PX4's own flight-mode logic does it natively and is what's actually flying the vehicle.
2. **Battery below that second, lower threshold** (RTL itself might not complete in time): call `action.land()` — land immediately at current position rather than attempting the trip home. This is the "nearest safe landing point if RTL is no longer feasible" case from `context.md`, simplified to "land right here" rather than searching for a better landing spot — appropriate for the "simple, not gold-plated" fidelity level.

No grid, no A*, no waypoint computation — this entire trigger is a threshold check plus one of two single MAVSDK action calls.

---

## Trigger 3: GPS-degraded → simple mode/speed downgrade

### Verified against MAVSDK (checked, not assumed)

Introspected `mavsdk.action.Action` and `mavsdk.telemetry.FlightMode` on the installed `mavsdk==3.17.2`:

- `action.set_current_speed(speed_m_s)` — **real, confirmed.** "Set current speed... during a mission, reposition, and similar. Ephemeral, not stored on the drone." Exactly the "reduce speed" response `context.md` asks for.
- `action.hold()` — **real, confirmed.** Switches to PX4's HOLD/Loiter mode: "stop and maintain its current GPS position and altitude." (Same underlying mode as `mission.pause_mission()` — the docstring for `pause_mission()` explicitly says pausing "puts the vehicle into HOLD mode.") Note this still *requires* a GPS position to hold — it is not a GPS-independent mode.
- **What does NOT exist**: there is no MAVSDK `Action` method to command a switch into `ALTCTL`/`POSCTL`/`MANUAL`/`STABILIZED` (visible as `FlightMode` telemetry values, but not reachable via a dedicated action call). Those modes exist in PX4 for RC-piloted flight and expect continuous manual stick input for horizontal (POSCTL) or full attitude (MANUAL/STABILIZED/ACRO/RATTITUDE) control — there's no autonomous, no-RC-input way to "fly" in them, so they aren't a meaningful "more conservative navigation mode" option for an unpiloted mission. This directly contradicts a phrase in `context.md` ("switches to a more conservative navigation mode") if read as "a different PX4 flight mode that still navigates" — the honest, verified answer is that the only two levers MAVSDK actually gives us for autonomous GPS-degraded response are *slower* (`set_current_speed`) and *stopped* (`hold`), not *differently-navigating*.

### Response, two-tier

1. **`ViolationKind.GPS_HDOP_HIGH`** (still 3D fix, just imprecise): `action.set_current_speed()` to a conservative fraction of the mission's normal cruise speed (default: half), continue the mission. Speed is restored once HDOP recovers below threshold for some debounce window (mirroring the debounce pattern already used for `wait_ready_to_arm`'s `is_armable` check, decisions.md D9 — same lesson: don't act on a single noisy sample).
2. **`ViolationKind.GPS_FIX_DEGRADED`** (below 3D fix): `action.hold()` — stop navigating via waypoints entirely, since the position estimate itself is untrustworthy, not just imprecise. If fix quality recovers to 3D within a timeout, resume the mission from its current index (`mission.set_current_mission_item()` — verified present — or simply `start_mission()` again, need to check empirically which resumes cleanly, see "Needs SITL verification" below). If it doesn't recover in time, escalate to the battery-critical response's RTL path — continuing to fly blind is not something this system should keep waiting on indefinitely.

---

## Mission handoff mechanics (shared by all three triggers)

Sequence: **pause → confirm settled → clear → upload → resume.**

```
1. mission.pause_mission()        -- vehicle enters HOLD
2. wait until: flight_mode == HOLD AND ground speed < some small epsilon
               (debounced -- don't act on one noisy sample, same lesson as D9)
3. mission.clear_mission()        -- remove the old plan from the vehicle
4. mission.upload_mission(new_plan)
5. mission.start_mission()        -- resume flying, now on the new plan
```

Step 2 is a deliberate addition beyond the bare minimum, informed directly by this project's own bug history: D8/D9/D10 all came from *assuming* a state transition had completed (ready-to-arm, a clean SITL restart) when it hadn't. Racing `clear_mission()` against a vehicle that hasn't actually stopped yet is exactly the kind of assumption that's already burned real time twice on this project — so the design bakes in an explicit "wait and confirm" step rather than trusting `pause_mission()`'s return to mean "fully settled."

**Critical nuance, learned from D8**: a replanned mission uploaded *mid-flight* must **not** include a `WaypointKind.TAKEOFF` item on its first waypoint. `firmware_link/mavsdk_client.py`'s `_mission_to_plan()` currently doesn't care what kind the first item is (it just maps whatever `WaypointKind` it's given) — `reroute.py`/`battery_response.py`/`gps_response.py` must all be careful to build replan waypoints as plain `WaypointKind.NAV`, never `TAKEOFF`, since the vehicle is already airborne. Getting this wrong would repeat exactly the D8 failure mode (PX4 either ignoring a spurious takeoff item mid-flight, or worse, doing something undefined with it) — flagging this explicitly so it isn't rediscovered the hard way.

### New `GroundLinkVehicle` methods needed (thin wrappers, same pattern as existing ones)

```python
async def pause_mission(self) -> None            # mission.pause_mission()
async def clear_mission(self) -> None             # mission.clear_mission()
async def hold(self) -> None                       # action.hold()
async def return_to_launch(self) -> None            # action.return_to_launch()
async def land(self) -> None                         # action.land()
async def set_speed(self, speed_m_s: float) -> None    # action.set_current_speed()
async def mission_progress_stream(self) -> AsyncIterator[tuple[int, int]]  # mission.mission_progress()
```

All of these map directly to the introspected, real methods listed above — no new capability is being assumed here that wasn't checked.

### Needs SITL verification before being trusted — this is not optional

Everything verified so far (`firmware_link`, decisions.md D8-D10) tested exactly one flow: ground-start, arm, upload once, fly once. The replan handoff sequence above has **never been run against live SITL**. Specific things that need empirical confirmation, not assumption, before `replanning_engine` is trusted for the evaluation:

1. Does `mission.pause_mission()` reliably produce `HOLD` mode from `MISSION` mode mid-flight (vs. only from ground)?
2. Does `mission.clear_mission()` followed immediately by `mission.upload_mission()` behave cleanly, or does it hit some PX4-side race/rejection akin to the `dataman` issue (D10) — i.e., does clearing a mission actually clear it immediately, or is there an async settling period before a new upload is safe?
3. Does `mission.start_mission()` on a freshly-uploaded *replan* correctly start from item 0 of the *new* plan, given the vehicle is already airborne and not at the new mission's nominal "start"? (The original ground-start missions never had this ambiguity — first item was always where the vehicle was.)
4. For the GPS-recovery-resume case: does `mission.set_current_mission_item()` actually resume execution, or does it just change the reported index without the vehicle acting on it (its own docstring hints at MAVLink-mission-loop-counter edge cases that might not apply cleanly here)? — **SETTLED (decisions.md D17): the question turned out to be moot.** A plain `mission.start_mission()` after a GPS-degraded HOLD *resumes from the current mission item* rather than restarting from item 0, so `set_current_mission_item()` is not needed at all on this path. Measured 5/5 against live SITL: progress sat at item 2 when the HOLD fired and stayed at item 2 through the resume (never dipping at any sample), then ran forward to the final waypoint in ~30s. `resume_after_gps_recovery()` is correct as written.

Given the project's track record — three real bugs, each one initially invisible from the MAVLink/telemetry side and only found by checking PX4's actual behavior directly — the plan is: build the pure-Python pieces (`no_fly_zone.py`, `grid_astar.py`, `reroute.py`, `battery_response.py`, `gps_response.py`) and unit test them fully first (no SITL needed, these are plain geometry/dataclasses), then build `engine.py`'s MAVSDK-facing handoff and immediately run it against live SITL with the same systematic, measured-pass-rate discipline used for D9/D10 — not a single anecdotal success — before calling any of this "verified."

---

## Data model additions

```python
# replanning_engine/events.py
@dataclass(frozen=True)
class ReplanEvent:
    timestamp_unix_s: float
    trigger: ReplanTrigger              # NO_FLY_ZONE | BATTERY_CRITICAL | GPS_DEGRADED
    reason: str                          # human-readable, for the dashboard log
    old_remaining_waypoints: list[Waypoint]
    new_remaining_waypoints: list[Waypoint]
    outcome: str                          # "rerouted" | "rtl" | "hold" | "no_safe_reroute_found" | ...
```

This is the "log of replanning events with the reason for each decision" `context.md` asks the dashboard to show — designed now so `engine.py` emits it from the start rather than bolting logging on after the fact.

## What's testable without SITL vs. what isn't

**Fully unit-testable now, no SITL**: `no_fly_zone.blocks_remaining_path()`, `grid_astar` (feed it a synthetic occupancy grid, assert the path avoids blocked cells and matches Dijkstra/brute-force on small grids), `reroute.py`'s splicing logic, `battery_response.py`'s and `gps_response.py`'s pure decision functions (given a `Thresholds`-like config and a violation, assert which action gets chosen) — same testing discipline as `mission_planner`/`constraint_monitor`'s existing 17 tests.

**Needs live SITL, cannot be responsibly claimed working from unit tests alone**: everything in `engine.py`'s MAVSDK-facing handoff (the four numbered questions above). This split is the actual point of writing this design doc before code — the parts that are genuinely just geometry and decision logic can be built and trusted immediately; the parts that touch PX4's live state machine get the same "verify against real SITL, measure a real pass rate, don't declare victory on one good run" treatment that D9/D10 established as necessary.

## Open questions for review

1. ~~**Resume semantics on GPS recovery** — `set_current_mission_item()` vs. re-`start_mission()` on the same uploaded plan: worth a quick SITL check before committing to the design, or fine to decide during implementation once both are tried?~~ **Answered by SITL check (decisions.md D17): re-`start_mission()` alone, which resumes from the current item. 5/5.**
2. **RTL altitude** — PX4's RTL climbs to a configured altitude before transiting home (`action.get/set_return_to_launch_altitude()` also exists in the verified method list, not detailed above since the battery-critical trigger doesn't need to touch it — PX4's default is used). Worth exposing as a GroundLink-level setting, or leave it as a PX4 parameter untouched by this system?
3. **No-fly-zone safety margin** — currently proposed as reusing the mission's `acceptance_radius_m`. Reasonable default, or should it be its own explicit parameter (e.g. tied to expected GPS error under degraded conditions)?
4. **Failure-injection format** — `sim/failure_injection/` is still just a README describing intent. Once this design is approved, the evaluation section (`context.md`: "baseline RTL-on-failure vs. adaptive replanning under identical injected failures") needs concrete scenario configs (when a no-fly zone appears, what battery drain rate, etc.) — happy to draft that alongside `engine.py`, or should it wait until the reroute logic itself is built and tested?
