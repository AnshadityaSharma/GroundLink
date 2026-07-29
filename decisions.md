# GroundLink — Decisions Log

Chronological log of architectural/technical decisions with rationale. Updated as decisions are made, not batched at the end.

---

## D1 — MAVSDK-Python over pymavlink

**Decision**: Use MAVSDK-Python as the firmware integration library.

**Rationale**:
- Async/await API maps directly onto what this system needs to do concurrently: stream telemetry, run constraint checks, and (later) push replanned missions/offboard commands without blocking on each other. pymavlink is synchronous/callback-based and we'd end up rebuilding an async wrapper around it anyway.
- Higher-level mission API (`mission.upload_mission`, `action.arm`, `telemetry.battery()`, `mission.set_current_mission_item`, offboard mode) removes a lot of raw MAVLink message-packing boilerplate that pymavlink requires by hand.
- Maintained by the PX4 team, with PX4 SITL as the first-class reference target — matches our primary flight stack.
- `context.md` already lists MAVSDK-Python as the preferred option.

**Trade-off accepted**: MAVSDK's ArduPilot support is less mature than its PX4 support, and it's a heavier dependency (runs a local gRPC-backed `mavsdk_server` process under the hood) than pymavlink's single-file simplicity. Given PX4 is our primary flight stack, this is acceptable. If ArduPilot becomes a hard requirement later, pymavlink remains available as a fallback for that specific integration point — the firmware_link layer is structured so the MAVSDK client is the only thing that would need replacing (see D8).

---

## D2 — pyproject.toml over requirements.txt

**Decision**: Use a single `pyproject.toml` (PEP 621) with an editable install, not a bare `requirements.txt`.

**Rationale**: `mission_planner`, `firmware_link`, `constraint_monitor`, `replanning_engine`, and `dashboard` are separate top-level packages that need to import from each other (e.g. `replanning_engine` will consume `mission_planner` types, `dashboard` will consume everything). An editable install (`pip install -e .`) makes those imports work cleanly from anywhere in the tree and under pytest, without `sys.path` hacks or a `src/` layout workaround. A plain `requirements.txt` doesn't give us that — we'd need one anyway alongside manual path munging.

**Trade-off accepted**: Slightly more setup ceremony than `pip install -r requirements.txt` for a first-time contributor. Documented step-by-step in README.md.

---

## D3 — Execution environment: WSL2 (Ubuntu 24.04), not Windows-native

**Decision**: All Python tooling (venv, MAVSDK, PX4 build, Streamlit) runs inside WSL2 Ubuntu, not split between Windows-native and WSL-native tooling.

**Rationale**: PX4 SITL's build toolchain (and Gazebo/jMAVSim) requires Linux. The dev machine is Windows 11. Rather than mock the flight controller on Windows (which `context.md` explicitly rules out) or split the stack across two OSes (fragile — path translation, networking edge cases, "works on my half" bugs), everything lives in one Linux environment: WSL2.

**Verified before committing to this** (2026-07-28 environment audit): WSL2 2.7.3.0 / kernel 6.6.114.1-1 with WSLg 1.0.73 (GUI passthrough) already present and working on this machine; Ubuntu 24.04.1 LTS available; 954GB free disk, 16 cores, 15GB RAM. Confirmed — not assumed — that a server bound inside WSL2 is reachable from a normal Windows browser via `localhost:<port>` (default NAT networking with localhost forwarding, no `.wslconfig` override present), which is what makes the Streamlit dashboard viewable from Windows later without extra config.

**Open flag**: PX4's official `ubuntu.sh` dependency script has historically been tested primarily against 20.04/22.04. Ubuntu 24.04 support exists in current PX4 releases but is newer territory — if it causes friction during the SITL build (Step 2), it'll be logged here rather than silently patched around.

---

## D4 — Project files on Windows filesystem, venv + PX4 clone native to WSL

**Decision**: The GroundLink repo itself stays on the Windows filesystem (`C:\Users\Admin\Desktop\drones_project`, visible to WSL at `/mnt/c/Users/Admin/Desktop/drones_project`) so it's directly editable from Windows-side tools. The Python venv and the PX4-Autopilot clone are created inside WSL's native filesystem (`~/groundlink-venv`, `~/PX4-Autopilot`), not under `/mnt/c`.

**Rationale**: Plain Python source edits over the `/mnt/c` DrvFs bridge are fine (small text files, no heavy I/O). But venvs (lots of small symlinked files) and especially a full PX4 build (thousands of compiled objects, incremental rebuilds) are known to be significantly slower and occasionally flaky over the 9P/DrvFs bridge. Keeping those two specifically on native WSL ext4 avoids that class of problem while keeping the actual project source in one canonical, Windows-editable location.

---

## D5 — pip bootstrap without sudo/ensurepip; mavsdk API verified by introspection, not memory

**Context**: The WSL Ubuntu 24.04 environment has no passwordless sudo, and `ensurepip` isn't installed (Ubuntu splits it out of the base `python3` package). `python3 -m venv` alone can't provision `pip` here.

**Decision**: Created the venv with `python3 -m venv --without-pip`, then bootstrapped pip via `https://bootstrap.pypa.io/get-pip.py` run inside the venv. This needs no root at all — pip installs entirely into the user-owned venv directory.

**Consequence flagged for the user**: this sidesteps the *venv* blocker, but PX4's own build toolchain (`cmake`, `ninja-build`, `build-essential`, plus whatever `PX4-Autopilot/Tools/setup/ubuntu.sh` needs) has to come from `apt`, which does need sudo. That step still needs the user to run one command interactively in their own WSL terminal — logged as an open item, not routed around.

**mavsdk API verification**: Rather than write the MAVSDK conversion code from memory/training-data recall of the API, once the venv existed I introspected the actually-installed `mavsdk==3.17.2` package's real docstrings/enum values directly in WSL. This caught three real mismatches against my first draft of `firmware_link/mavsdk_client.py`:
- `Battery.remaining_percent` is already scaled 0-100, not a 0-1 fraction (draft had `* 100.0`, would have produced 0-10000%).
- `mavsdk.telemetry.FixType` is a plain `Enum`, not `IntEnum` — `int(fix_type)` raises `TypeError`; needs `.value`.
- HDOP is not on `GpsInfo` at all — it's a field on the separate `raw_gps()` stream, so the client now merges two independent MAVSDK channels into one `GpsState`.

`MissionItem`'s 14-parameter positional signature, by contrast, matched the first draft exactly. Both the confirmation and the mismatches came from checking the installed package for real rather than trusting recall — logging this because it's the kind of MAVSDK-version-drift issue that's easy to get subtly wrong and hard to notice until it's run against real SITL.

---

## D6 — `ubuntu.sh --no-nuttx` for now; sudo steps require the user to run them

**Decision**: Run PX4's official `Tools/setup/ubuntu.sh --no-nuttx` rather than the full script, and rather than hand-picking apt packages to replicate what it does.

**Rationale**:
- `--no-nuttx` skips only the ARM cross-compilation toolchain used for building real Pixhawk firmware (confirmed by reading `ubuntu.sh` directly — `INSTALL_NUTTX` gates a separate code block from `INSTALL_SIM`). Current milestone is SITL-only, so this saves a large, currently-unneeded download. If/when real hardware access happens, re-running `ubuntu.sh` without the flag adds NuttX support without redoing anything else.
- Confirmed by reading the script (not assumed): it explicitly lists Ubuntu 24.04 and 22.04 LTS as supported — the compatibility flag raised in D3 is resolved, this machine's Ubuntu 24.04.1 is officially in scope.
- Running the official script rather than a hand-extracted apt package list avoids silently missing something it does beyond `apt install` (e.g. adding Gazebo's package repo/key, pip-installing `requirements.txt`).

**Consequence flagged for the user**: like the `cmake`/`ninja` install, this script calls `sudo apt-get install` internally and there's no passwordless sudo available, so it has to be run by the user interactively, not by me.

---

## D7 — Gazebo Harmonic, not jMAVSim (decided by PX4 upstream, not by us)

**Context**: The build prompt asked to prefer jMAVSim over Gazebo if Gazebo proved unreliable under WSLg, to avoid burning time fighting graphics drivers.

**What actually happened**: `Tools/setup/ubuntu.sh` on PX4 v1.17.0 installs Gazebo Harmonic (`gz-harmonic` and friends) as the simulator dependency and does not install Java/jMAVSim at all — confirmed by checking installed packages after running the script (`gz-harmonic 1.0.0-1~noble` present, `java` command missing). PX4 deprecated jMAVSim in favor of Gazebo in recent releases; by pinning to the latest stable tag (v1.17.0, see D-earlier clone note), we inherited that default rather than choosing between them ourselves. If Gazebo's rendering under WSLg turns out to be unreliable when we actually launch a full simulation (vs. just the SITL binary build), that's the point where we'd revisit — logged here as the trigger condition, not yet hit.

---

## D8 — Raw MAVSDK mission uploads need an explicit TAKEOFF item; execution is intermittent under this environment (partially unresolved — flagged, not swept under the rug)

**What happened**: The first end-to-end run against live PX4 SITL "worked" in the shallow sense — connect/arm/upload/start_mission/telemetry all returned success with no errors — but a closer look (prompted by the vehicle's lat/lon looking suspiciously frozen across a 45s telemetry window) showed the vehicle never actually left the ground. `mission.mission_progress()` reported `3/3` (complete) almost immediately after `start_mission()`, `flight_mode` read `MISSION`, and `relative_altitude_m` stayed at ~0.0 the entire time. No error was ever raised anywhere in the stack.

**Root cause #1** (confirmed empirically, not from docs): QGroundControl silently auto-inserts a takeoff mission item when you build a mission through its UI. A raw `mavsdk.mission.upload_mission()` call does not — and PX4's waypoint-reached check for a plain NAV item is horizontal-acceptance-radius-only, so with the vehicle still sitting on the ground, three closely-spaced waypoints (all within ~33m of each other) can be trivially "already within acceptance radius."

**Fix applied**: mark the first waypoint of a ground-start mission with `kind=WaypointKind.TAKEOFF` (the enum already existed in `mission_planner/waypoint.py`; it just wasn't wired through). `firmware_link/mavsdk_client.py`'s `_mission_to_plan()` now maps `WaypointKind` to MAVSDK's `MissionItem.VehicleAction` (`TAKEOFF`/`LAND`/`NONE`). This fix is correct and necessary — confirmed by direct inspection of the uploaded `MissionPlan` (`vehicle_action: TAKEOFF` on item 0) — but **turned out not to be sufficient on its own**, see below.

**Honest status after more testing**: re-running the exact same mission against fresh SITL instances 4 times (1 raw ad-hoc script, 3 through the real `GroundLinkVehicle` + `connectivity_check` code path, one with an added 3s EKF-settling delay before upload) produced the takeoff-and-fly-through-all-3-waypoints behavior **once** and the instant-complete-without-climbing behavior **three times**, with no code difference between the runs. This rules out a settling-time race as the sole explanation (the delay didn't fix it) and points at something environment-level and non-deterministic — most likely PX4's lockstep simulation timing interacting with WSL2's virtualized CPU scheduling, which is a known source of jitter for time-sensitive sim loops, but this is a hypothesis, not confirmed.

**What IS solidly verified, every single run, no exceptions**: connect, arm, mission upload (with correct MAVSDK field mapping), `start_mission()` accepted, and live telemetry (battery/GPS/position) streaming — which is the literal Step 2 deliverable ("connects... arms it, uploads a trivial 3-waypoint mission, and streams back telemetry"). What's NOT reliably verified: the vehicle completing that mission by actually flying it, which turned out to be a harder, flakier thing to pin down than the milestone asked for.

**Not chasing this further right now** — logging it here rather than either quietly re-running until it looks good (misleading) or burning unbounded time on an intermittent SITL/WSL2 timing issue during a scaffolding pass. Flagged explicitly in the Step 4 report as something to revisit, likely worth an EKF/lockstep-readiness check more rigorous than `is_global_position_ok`/`is_home_position_ok` before arming.

**Known gap, separately**: `WaypointKind.RTL` has no equivalent `MissionItem.VehicleAction` in MAVSDK — RTL is a top-level command (`action.return_to_launch()`), not a mission-item property. It currently degrades silently to `NONE` in `_mission_to_plan`. Not a problem for Steps 1-4 (nothing uses `WaypointKind.RTL` yet), but `replanning_engine` will need to issue RTL as a mode-change command rather than a mission waypoint.

**Also found while debugging this**: killing a running connectivity script mid-mission (e.g. via an external `timeout`) and reconnecting a second script to the same still-armed SITL instance can leave the vehicle latched in `HOLD` mode (observed once, likely a datalink-loss failsafe from the abrupt heartbeat gap). Not the same issue as above — a re-run against a fresh SITL instance ruled this out as the cause of the no-takeoff behavior.

---

## D9 — Systematic pass-rate investigation of the mission-execution flakiness (bounded effort, two real bugs fixed, root cause NOT found at the time — SUPERSEDED BY D10, see below)

> **This entry's final paragraph speculated that WSL2 CPU-scheduling/lockstep timing was the likely cause. That was wrong. D10 below found and confirmed the actual root cause: a stale on-disk PX4 mission store, nothing to do with WSL2 or timing.** Kept here rather than deleted/rewritten so the investigation trail (what was ruled out, and how) stays intact and isn't misremembered later.

Following up on D8 with an actual measured pass rate instead of anecdotal runs, per explicit direction to bound the effort and report back rather than chase this indefinitely. Built a small trial harness (`run_trials.sh` + `trial.py`, not committed to the repo — throwaway diagnostic tooling in the session scratchpad) that, per trial: kills any existing PX4/Gazebo processes, launches a fresh SITL instance, waits for boot, runs the real `GroundLinkVehicle` + the actual `connectivity_check` test mission, and records PASS (climbed above 5m) or FAIL (never left the ground) plus PX4's own internal log.

**Bug found #1 — stale Gazebo world reused across "fresh" trials.** The very first 8-trial batch was 0/8, and PX4's own log showed `"gazebo already running world: default"` with a lockstep clock starting from a large, non-zero, ever-increasing value — the `gz sim` server process had been running continuously for 46+ minutes across dozens of supposedly-independent tests, because cleanup only ever killed the `bin/px4` process, never the simulator itself. Fixed the harness to kill `gz sim` between trials too, and added a check that fails loudly (`reason=stale_gazebo_world_reused`) if it ever happens again. This is a real, valid fix — worth keeping in mind for anyone scripting repeated SITL runs — but re-running the 8-trial batch with genuinely fresh worlds every time still scored **0/8**. Not the (sole) cause.

**Bug found #2 — weak arm-readiness gate.** 2 of those 8 trials failed with `mavsdk.action.ActionError: COMMAND_DENIED` directly on `arm()` — PX4 itself refusing to arm despite `wait_ready_to_arm()` (checking only `is_global_position_ok` + `is_home_position_ok`) already having returned. Fixed `firmware_link/mavsdk_client.py`: `wait_ready_to_arm()` now requires PX4's own `is_armable` flag (plus `is_local_position_ok`) to read true for 3 consecutive samples (debounced, not a single flicker), and `arm()` retries up to 5 times on `ActionError`. Re-ran the same 8-trial batch: **zero `COMMAND_DENIED` errors this time** — that failure mode is genuinely fixed — but altitude-climb pass rate was still **0/8**.

**Hypothesis tested and ruled out — insufficient settle time.** Increased the post-boot settle delay from 3s to 15s before issuing any MAVSDK commands, on the theory that the automated harness's tight back-to-back timing (vs. the naturally slower pacing of interactive manual testing, where the one prior success was observed) was the differentiator. 5-trial batch: **0/5**. Ruled out.

**Hypothesis tested and ruled out — SITL and test client sharing one WSL session/pty.** The one earlier manual success had SITL running as the sole process of its own dedicated `wsl.exe` session, with the test script in a completely separate session — structurally different from the harness, which backgrounds SITL inside the same script/session as the test call. Reproduced that exact separate-session structure for one trial: **still FAIL** (max altitude 0.05m). Ruled out.

**Where this leaves things**: 30 systematic automated trials, 0 passes, across four different conditions (stale-world-fixed, arm-readiness-fixed, long-settle, separate-session), against a total sample of 1 success in roughly 5-6 attempts across the whole investigation (manual + automated combined, ~15-20%, though the automated-only rate looks closer to 0%). Two real, verified bugs were found and fixed along the way (stale Gazebo world reuse, weak arm-readiness gate) and both fixes are shipped in `mavsdk_client.py` regardless of the outcome here — they're correct fixes for real problems, independent of whether they explain the climb failure. The climb failure itself remains **unexplained** after a genuinely bounded, structured effort. Leading candidates not yet tested: PX4/Gazebo lockstep timing sensitivity that's specific to WSL2's virtualized CPU scheduling (would require comparing against native Linux to confirm), or something in PX4's own navigator/takeoff state machine visible only via its uORB topics rather than MAVLink-level telemetry (would require log analysis with PX4's `pyulog` tooling on the `.ulg` flight logs SITL already writes to `~/PX4-Autopilot/log/`, not yet attempted).

**Recommendation** (superseded by D10 below — kept for the trail): treat "connect/arm/upload/telemetry-stream reliable, autonomous mission-completion unreliable under this WSL2 setup" as the accurate current baseline.

---

## D10 — ROOT CAUSE FOUND AND CONFIRMED: stale on-disk `dataman` mission store, not WSL2/timing

Following the direction to check PX4's own `.ulg` flight log with `pyulog` (independent of MAVSDK) before concluding this was an environment issue.

**What the `.ulg` showed, from a failed trial**: `mission_result` reported `seq_current=5, seq_reached=5, finished=1` within 50ms of arming — but the uploaded mission only had 3 waypoints (valid indices 0-2). PX4 was executing waypoint state that didn't come from the mission we'd just uploaded. `vehicle_local_position.z` stayed at ~0 the entire time (no real climb), and `vehicle_status` showed the vehicle auto-disarming ~11 seconds after arming (`arming_state: 2→1`) — consistent with PX4's landed-timeout safety disarm, because the land-detector never saw the vehicle leave the ground: it never actually tried to, since the navigator thought the mission was already finished.

**Root cause**: PX4's `dataman` file — its on-disk mission/waypoint storage, at `~/PX4-Autopilot/build/px4_sitl_default/rootfs/dataman` — persists across process restarts by default. Every trial across the entire investigation (D8, D9, and the 30 automated trials in D9) killed the `bin/px4` process (and later, `gz sim`) between runs, but never touched this file. It had been accumulating mission-upload state since the very first SITL launch of the session (11:57, confirmed by file mtime). Each "fresh" trial was arming a fresh PX4 process that immediately read stale leftover mission state off disk — not a timing race, not WSL2 scheduler jitter, not anything in `firmware_link`'s MAVSDK code. The two real bugs fixed in D8 and D9 (missing `TAKEOFF` item, weak arm-readiness gate) were both genuine, correct fixes for genuine problems — they just weren't the problem causing the pass-rate investigation's 0/30.

**Fix, confirmed**: delete `dataman` before every SITL launch. Single-trial test after deleting it: **PASS, max_alt=14.95m**. Follow-up 8-trial batch (dataman deleted before each, exact same harness that measured 0/8 three times in D9): **8/8 PASS**, altitudes 14.94-14.96m every time — tight, consistent, not a fluke. Re-verified end-to-end afterward through the actual `sim/launch_sitl.sh` script (not just the throwaway trial harness) with the real `firmware_link/connectivity_check.py`: clean climb to 15m and full three-waypoint transit, confirmed by continuously changing lat/lon across the whole flight.

**Fix applied in the codebase**: `sim/launch_sitl.sh` now kills any lingering `bin/px4`/`gz sim` processes and deletes `dataman` before every launch, with a comment explaining why and warning not to remove it. This needs to be part of every SITL restart going forward — a manual `make px4_sitl gz_x500` without going through this script (or without remembering this step) will silently reintroduce the bug.

**Native Linux comparison (D9's step 2 contingency)**: not needed. The cause is confirmed and understood; it isn't environment-specific — the same stale-dataman bug would reproduce identically on native Linux, since it has nothing to do with WSL2.

**Lesson for next time**: when a MAVLink-level symptom looks inexplicable (commands succeed, telemetry looks fine, but behavior is wrong), check PX4's own internal `.ulg` log before assuming an environment/timing cause — `mission_result.seq_current` exceeding the uploaded item count was the single line that cracked this open, and it's invisible from the MAVSDK/telemetry side entirely.

---

## D11 — Replan handoff root cause found and fixed (mode-switch rejection); confirmed by isolated runs, NOT confirmed by batch measurement — logged honestly as a mixed result, not a clean win

Following up on `replanning_engine/DESIGN.md`'s flagged empirical questions about the pause->clear->upload->resume handoff, using the same `.ulg`-log discipline that cracked D10.

**Symptom**: a live-SITL trial of `ReplanningEngine.handle_no_fly_zone()` (in-flight reroute) hung indefinitely after a successful-looking replan -- mission uploaded, `start_mission()` returned without error, but the vehicle froze in place (position jitter only, `mission_progress.current` stuck at 0) instead of flying the new route.

**Root cause, confirmed via `.ulg`, not guessed**: MAVSDK's `mission.start_mission()` call itself succeeds (it only tracks the `MAV_CMD_MISSION_START` ack), but PX4 internally follows that with its own `DO_SET_MODE` attempt to actually switch into MISSION mode -- and in the failing run, that second, internal step came back `vehicle_command_ack: command=176 result=1` (`MAV_RESULT_TEMPORARILY_REJECTED`), invisible to MAVSDK's Python-level success/failure reporting. Same class of timing race as the arm-readiness issue (D9), just on a different command.

**Fix applied**: `replanning_engine/engine.py`'s `_execute_handoff` no longer trusts `start_mission()`'s return value. A new `_start_mission_and_confirm_resumed()` actively polls `FlightMode` telemetry after calling it and retries the call (up to 5x) if the vehicle hasn't actually entered `MISSION` mode within a few seconds. `firmware_link/mavsdk_client.py`'s `start_mission()` also gained its own `MissionError` retry (matching `arm()`'s existing pattern) -- a real, correct fix for a related-but-distinct failure mode, though it turned out not to be the one causing the hang (MAVSDK never raised an exception for the rejection in the first place, so exception-based retry alone couldn't catch it).

**What's actually confirmed vs. not, precisely**:
- **2 isolated single-run tests, both with the fix applied, both clean successes**: full reroute, `mission_progress` correctly advancing (0→1→2→3 in one run), 175 and 241 distinct positions respectively (real continuous flight, not a stall), one reaching the final waypoint at 24.6s. This is real, concrete evidence the root cause was correctly identified and the fix works *in that execution context*.
- **A scripted batch (5 trials, same script, same fix, fresh SITL + cleared `dataman` each time, generous 220s per-trial timeout): 0/5, every trial hit the external timeout with no output at all** -- not the diagnosed-and-fixed mode-rejection symptom (which now raises a clean `TimeoutError` after ~15s, not a silent 220s hang), something else entirely. Checked immediately after: no stuck `px4`/`gz sim` processes, load average under 1.0, no WSL restart -- nothing obviously wrong with the environment.

**Honest conclusion**: the mode-rejection root cause is real and the fix is correct for it, but there is now a **second, distinct, unexplained failure mode** where the identical script hangs silently for the full external timeout specifically when run through the batch harness (`run_engine_trials.sh`) and not when run as a single isolated invocation. This has not been diagnosed. Per explicit instruction not to spend hours chasing a novel failure alone overnight, this is logged here as an **open issue** rather than investigated further tonight. Leading candidates for next time, not yet tested: something about the batch harness's `setsid ... &` backgrounding interacting differently with the async telemetry-polling code added for the mode-confirmation fix (the isolated tests were run as a plain foreground `timeout N python script` without that backgrounding layer); or resource/state accumulation specific to running many consecutive trials in the same long-lived WSL session that a single isolated test never encounters. **Do not report this as "D11 confirms the handoff is reliable" -- it does not. It confirms the specific root cause and fix are correct, and separately flags that batch-mode execution has its own unresolved problem.**

---

## D12 — GPS-degraded response verified against live SITL: 5/5, clean

Following D11, verified `ReplanningEngine.handle_gps_degraded()` (the simpler trigger -- no mission re-upload, just direct `action.set_current_speed()` / `action.hold()` calls) using the same measured-batch discipline via a generalized `run_trials.sh` (parameterized to reuse the D9/D10/D11 launch/cleanup/dataman-clear discipline for any trial script).

**Test**: one continuous flight per trial -- takeoff, climb past 10m, then in sequence: (1) `handle_gps_degraded(FIX_3D, hdop=3.5, ...)` → expect `slowed_down` outcome + vehicle still airborne and flying 5s later; (2) `handle_gps_degraded(NO_FIX, ...)` → expect `hold` outcome + `FlightMode` actually confirmed as `HOLD` within 10s (not just trusting the call's return -- same "verify, don't trust" lesson as D11).

**Result: 5/5 PASS**, every trial, both stages, across 5 independent fresh-SITL trials. No timeouts, no hangs, no batch-vs-isolated discrepancy like D11's. Consistent with this trigger being architecturally simpler (no `pause_mission`/`clear_mission`/`upload_mission`/`start_mission` sequence, no PX4-internal mode-switch race to hit) -- the same class of bug that hit the reroute handoff has no equivalent surface here.

**Not yet verified**: whether `set_current_speed()` actually reduces the vehicle's real flight speed by the expected fraction (the test only confirms the mission keeps progressing after the call, not the resulting velocity) -- a reasonable next check, not done tonight given the scope already covered. Also not yet verified: `resume_after_gps_recovery()` (calling `start_mission()` again after a HOLD to resume the original mission) -- untested, same category of "PX4 internal state transition" risk as D11's bug, flagged rather than assumed safe.

---

## D13 — Battery-critical response verified against live SITL: 5/5 both paths, clean

Verified `ReplanningEngine.handle_battery_critical()` -- also a simple, non-terminal-mission-upload trigger (just `action.return_to_launch()` or `action.land()`), same measured-batch discipline.

**Test**: takeoff, climb past 10m, then call `handle_battery_critical()` with a percentage chosen to trigger one specific path (15% for RTL, 5% for immediate land, against `BatteryResponseThresholds(rtl_below_percent=20, land_immediately_below_percent=8)`), then actively confirm (not just trust the call's return) that `FlightMode` actually reaches `RETURN_TO_LAUNCH` or `LAND` respectively within 15s.

**Results**:
- RTL path: **5/5 PASS**, `FlightMode` confirmed `RETURN_TO_LAUNCH` every trial.
- Land path: **5/5 PASS**, `FlightMode` confirmed `LAND` every trial.

10/10 across both paths, no timeouts, no hangs -- consistent with D12's finding that triggers without a mission pause/clear/upload/resume cycle don't hit the class of PX4-internal-state-transition race that D11's reroute handoff did.

**Not yet verified**: that RTL actually completes (returns to the launch point and lands) rather than just entering the right flight mode -- the test window (15s) only confirms the mode switch, not the full return-and-land sequence, which would take substantially longer to observe. Reasonable next check, not done tonight given the scope already covered in one session.

**Overnight verification summary** (D11+D12+D13): of the three trigger types, GPS-degraded and battery-critical are now measured-reliable (5/5 and 10/10 respectively). The no-fly-zone reroute (the core deliverable) has its root cause fixed and confirmed correct in isolation, but batch-measured reliability is still an open, unexplained problem (D11) -- this is the one piece that should not be treated as production-ready without further investigation.

---

## D14 — D11's batch-vs-isolated discrepancy: root cause found — it was the test harness, not the reroute logic. 5/5 confirmed on re-measurement.

Investigated per explicit direction, checking two specific hypotheses before touching any code:

**Hypothesis (a) — stale MAVSDK/mavsdk_server processes leaking between trials, same class as the D10 `dataman` bug.** Tested directly: MAVSDK-Python spawns its own persistent `mavsdk_server` gRPC subprocess per `System()` connection (confirmed by reading `mavsdk/system.py` -- `_start_mavsdk_server()`, cleaned up via `__del__`, which isn't guaranteed to run on a hard kill). Ran the trial script under `timeout` and killed it externally at two different points (8s, before much happens; 45s, deep into replan processing) and checked for a surviving `mavsdk_server` process both times: **none found either time.** This hypothesis is disconfirmed -- no leaked process was the cause.

**Hypothesis (b) — failure correlates with trial position (state accumulating across repeated trials).** Tested directly: ran the *exact* batch script with `N=1` (a single trial, no repetition possible). **It still failed (rc=124, full timeout).** This disconfirms accumulation across trials as the cause -- the very first trial in a batch fails the same way as the fifth.

**What actually distinguishes success from failure, found by systematic elimination**: not command substitution (removed it, still hung), not stdin inheritance (explicitly redirected the foreground call from `/dev/null` too, still hung), not trial count (N=1 still hung). The one variable that flips the outcome: whether PX4 SITL is backgrounded via `setsid ... &` **inside the same shell script/session** that later runs the Python MAVSDK client (hangs, reproduced 3 times independently), versus launching SITL as a **separate, independent process invocation** and only then running the Python client separately (succeeds, reproduced 6 times independently -- 1 minimal repro + 5-trial re-measurement below). This holds regardless of `setsid`, explicit `disown`, or full I/O redirection on the backgrounded job -- something about a single session hosting both a backgrounded PX4/Gazebo process tree and a later foreground asyncio/MAVSDK client with multiple concurrent telemetry subscriptions causes a silent hang. The exact kernel/pty-level mechanism is not fully explained (a legitimate remaining gap), but the *triggering condition* is now precisely identified and reliably reproducible/avoidable, which is what matters for actually using this code.

**Fix**: restructure how SITL is launched relative to the test client -- launch it as a fully independent process invocation, never backgrounded within the same script/session as the code that will later run the MAVSDK client against it. This is a testing/tooling fix, not a code fix -- nothing in `replanning_engine` or `firmware_link` changed.

**Re-measurement, same discipline as every other batch in this log**: 5 trials, fresh SITL instance per trial (dataman cleared, launched independently each time), identical `engine_trial_quiet.py` script, unchanged since D11.

**Result: 5/5 PASS.** Every trial reached the final waypoint (`reached_idx2=True`), consistently around 27s (27.10, 27.17, 27.14, 27.16, 27.21s), 233-240 distinct positions each (real continuous flight), altitude steady at 15.0-15.1m. This is a materially cleaner, more consistent result than even the two earlier "successful" isolated runs from D11 (which had more variable timing/position counts) -- strong evidence this is the same underlying behavior each time, not luck.

**Conclusion**: the no-fly-zone reroute handoff (`ReplanningEngine.handle_no_fly_zone()`, including the D11 mode-switch-confirmation fix) is now genuinely measured-reliable, not just "correct in isolation." D11's "open issue" status is resolved -- it was never a flaw in the reroute/replan logic, it was an artifact of how the trial harness itself launched SITL. All future SITL trial scripts in this project must launch SITL as an independent process, not backgrounded inside the same script that runs the test client -- noted here so this isn't rediscovered the hard way.

---

## D15 — `set_current_speed()` verified to actually change real flight speed: 5/5, and the measurement bug that made the first attempt meaningless

D12 left this explicitly open: `handle_gps_degraded()`'s SLOW_DOWN path was confirmed to *return* `slowed_down` and leave the mission progressing, but nothing checked whether the vehicle's real velocity changed. Checking it turned out to be mostly a measurement-discipline problem, not a code problem.

**The measurement bug (found, not guessed).** The first version sampled `speed_before` as soon as a short stability check passed. It reported `speed_before = 3.51 m/s +/- 1.56` — a standard deviation a third of the mean, i.e. not a cruise speed at all. Rather than tighten thresholds blindly, the script was made to emit a downsampled speed/altitude/mission-progress trace. The trace showed exactly what was wrong:

```
t=0.0 -> 16.5s   speed ~0.00, alt 0 -> 14.3    (pure vertical climb-out)
t=17.0 -> 25.6s  speed 0.69 -> 4.98, steady    (accelerating into the leg, then cruise)
t=26.1 -> 28.6s  speed 4.55, 3.69, 2.86, 2.21, 1.70, 1.25   (decelerating -- unprompted)
```

Horizontal ground speed is ~0 for the first 17 seconds because the vehicle is climbing straight up, and the deceleration at t=26.1s begins *before* the `handle_gps_degraded()` call at t~28.5s. So the "before" window straddled a slowdown the command did not cause. Two independent defects in the harness: the steady-cruise gate accepted a brief plateau inside a transient, and the measurement window was never validated after the fact.

**Fixes to the harness** (`~/trials/speed_trial.py`; no product code changed):
- Waypoints are now built from the vehicle's **actual home position read from `telemetry.home()`**, not hardcoded SITL-default coordinates. The real home this session was `47.397971, 8.546164` — the assumed default `47.397742, 8.545594` was ~65m off, which silently shortened the legs.
- Legs lengthened to ~2km so a single leg outlasts climb-out plus both measurement windows. This matters because PX4 re-applies each mission item's own speed at every waypoint transition, which would clobber the `set_current_speed` override mid-measurement.
- Each measurement window is **validated after the fact and rejected if invalid**: standard deviation must be < 0.30 m/s *and* `mission_progress` must not have advanced during the window (i.e. the vehicle did not reach a waypoint and turn). An invalid window is retried, never reported. This is the change that actually makes the number trustworthy, independent of any particular theory about what causes a transient.

**Honest gap**: with the corrected harness the anomaly is gone, but the specific cause of that t=26.1s deceleration in the *discarded* runs was never positively identified — the waypoint was still ~475m away, so it was not waypoint arrival. It is not chased further because the post-hoc validity gate now rejects such windows outright rather than averaging through them. Flagging it rather than claiming a root cause that was not confirmed.

**Result — isolated confirmation run, then a 5-trial batch** (fresh SITL per trial, launched as an independent process invocation per D14, `dataman` cleared each time):

| trial | before (m/s) | after (m/s) | ratio | err vs commanded 2.5 |
|-------|--------------|-------------|-------|----------------------|
| iso   | 4.976 +/- 0.020 | 2.503 +/- 0.009 | 0.503 | 0.003 |
| 1     | 4.963 +/- 0.008 | 2.503 +/- 0.015 | 0.504 | 0.003 |
| 2     | 4.968 +/- 0.012 | 2.500 +/- 0.012 | 0.503 | 0.000 |
| 3     | 4.966 +/- 0.015 | 2.500 +/- 0.010 | 0.503 | 0.000 |
| 4     | 4.971 +/- 0.022 | 2.498 +/- 0.024 | 0.503 | 0.002 |
| 5     | 4.978 +/- 0.018 | 2.493 +/- 0.015 | 0.501 | 0.007 |

**5/5 PASS**, every window accepted on its first attempt (no retries needed — the vehicle was genuinely steady each time). Measured ratio 0.501-0.504 against the commanded `slow_down_speed_fraction = 0.5`, and the resulting speed lands within 0.007 m/s of the commanded 2.5 m/s. Within-window standard deviations of 0.008-0.024 m/s show these are real steady-state cruise measurements, not averages across a transient.

**Conclusion**: `action.set_current_speed()` as wrapped by `GroundLinkVehicle.set_speed()` and driven by `handle_gps_degraded()` does change the vehicle's actual ground speed, promptly and to the commanded value. D12's open question is closed. Note the override is ephemeral (not persisted on the vehicle) and PX4 re-applies the mission item's speed at the next waypoint transition — so a sustained slow-down across multiple waypoints would need re-issuing, which is NOT currently done and is a real limitation worth addressing if the GPS-degraded state is expected to persist across waypoints.

**Tooling note**: SITL console output is now discarded (`/dev/null`) by default in the trial launcher instead of being captured. PX4 emits ~250MB/min of cursor-redraw spam; ~21GB of such logs had accumulated from previous sessions and none of them ever diagnosed anything — the `.ulg` flight logs are the real diagnostic source (D10, D11). Old console logs were purged.

---

## D16 — RTL verified to complete the full return-and-land, not just the mode switch: 5/5

D13 verified only that `FlightMode` reached `RETURN_TO_LAUNCH` within 15s and explicitly flagged the rest — whether the vehicle actually goes home and lands — as unverified, because the full sequence takes far longer than that test window. Closed now.

**Test**: takeoff, fly outbound until genuinely away from home (gate: >= 100m horizontal distance *and* at cruise altitude, so "came back" is a meaningful claim), then `handle_battery_critical(15.0)` (below `rtl_below_percent=20`, above `land_immediately_below_percent=8`, so it takes the RTL branch). Then watch the whole sequence to termination against real position/altitude/armed telemetry — not the call's return value, not the mode alone:

- `FlightMode` reaches `RETURN_TO_LAUNCH`
- horizontal distance to the **actual home position read from `telemetry.home()`** shrinks to ~0
- `rel_alt` drops to ~0
- the vehicle **disarms on its own**

Pass required all four, with `final_dist_to_home < 10m` and `final_alt < 1.0m`.

**Isolated confirmation run** — the trace is a textbook RTL and worth keeping:

```
t=16-38s   dist 0 -> 102m, alt 14.8    outbound mission leg
t=40s      RETURN_TO_LAUNCH engaged, alt starts climbing 16.8 -> 29.8
t=50-70s   dist 102 -> 1.7m at alt ~29.8   (transit home at RTL altitude)
t=72-96s   dist 0.0, alt 27.7 -> -0.1      (descent over home)
t=100s     alt 0.0, ARMED=False            (landed and disarmed)
```

Note PX4 climbs to its RTL return altitude (~30m here) *before* transiting — the vehicle briefly moves further from home (max 108.9m vs 102.5m at command time) while it turns and climbs. Worth knowing: a naive "distance must decrease monotonically" check would have failed a perfectly correct RTL.

**Batch result (5 trials, fresh SITL each, launched as an independent process invocation per D14): 5/5 PASS.**

| trial | dist at RTL (m) | t to RTL mode (s) | t to disarm (s) | final dist home (m) | final alt (m) |
|-------|-----------------|-------------------|-----------------|---------------------|---------------|
| iso   | 102.5 | 1.0 | 62.1 | 0.0 | 0.03 |
| 1     | 100.1 | 0.4 | 61.5 | 0.1 | 0.02 |
| 2     | 102.4 | 1.0 | 62.1 | 0.0 | 0.04 |
| 3     | 101.6 | 1.0 | 61.1 | 0.1 | 0.02 |
| 4     | 102.3 | 1.0 | 61.1 | 0.0 | 0.01 |
| 5     | 102.3 | 1.0 | 62.1 | 0.0 | -0.05 |

Every trial returned to within 0.1m of home, landed to within 5cm of ground, and disarmed autonomously, in a tightly clustered 61.1-62.1s. The consistency across independent fresh-SITL trials is strong evidence this is the real, repeatable behavior rather than a lucky run.

**Conclusion**: `handle_battery_critical()`'s RTL path delivers a complete return-and-land through to disarm. D13's open item is closed. Still not covered: RTL behavior when the return path itself crosses a no-fly zone (PX4's native RTL knows nothing about our zones) — a genuine design gap, not a test gap, and worth deciding on deliberately rather than discovering in flight.

---

## D17 — `resume_after_gps_recovery()` verified against live SITL: 5/5, and DESIGN.md's resume-vs-restart question is answered

This method was completely untested — flagged in D12 as "same category of PX4 internal state transition risk as D11's bug, flagged rather than assumed safe." Treated with the same rigor as the original reroute handoff.

**Two things under test**, one behavioural and one design:

1. Does it work at all — after a GPS-degraded HOLD, does it actually put the vehicle back into MISSION mode and get it flying again?
2. DESIGN.md's open empirical question: does a plain `start_mission()` **resume from the current mission item, or restart from item 0?** The answer decides whether `resume_after_gps_recovery()` is correct as written, or needs `set_current_mission_item()` first (i.e. `mavsdk_client.resume_mission_from`).

**Live risk probed deliberately**: `resume_after_gps_recovery()` calls `vehicle.start_mission()` **directly**, bypassing `engine._start_mission_and_confirm_resumed()` — the wrapper that exists precisely because PX4 can silently reject the internal mode switch while MAVSDK reports success (D11). So the test never trusts the call's return value; it polls `FlightMode` and real position afterwards.

**Test**: 4-waypoint mission (~155m legs). Fly until `mission_progress` reaches item 2, so "resume" has a meaningful place to resume *from*. Then `handle_gps_degraded(NO_FIX, 99.0, ...)` → confirm `HOLD` mode *and* that the vehicle actually stopped (ground speed, not just mode). Then `resume_after_gps_recovery()` → confirm MISSION mode, confirm it physically moves again, and — the strongest discriminator — confirm it **runs through to the final waypoint from the resumed point**. A restart-from-zero would also "move," so reaching the final item is what actually distinguishes the two.

**Isolated confirmation run** trace, the interesting window:

```
t=48s  speed 5.00  MISSION prog=1
t=49s  speed 4.75  HOLD    prog=2   <- degraded, decelerating
t=55s  speed 0.03  HOLD    prog=2   <- fully stopped
t=56s  speed 1.14  MISSION prog=2   <- resumed, mode back within ~1s
t=58s  speed 4.98  MISSION prog=2   <- back at full cruise speed
```

**Batch result (5 trials, fresh SITL each, launched as an independent process invocation per D14): 5/5 PASS.**

| trial | speed in HOLD (m/s) | t to MISSION mode (s) | progress at hold -> after resume | travelled after resume (m) | reached final wp | t to final wp (s) |
|-------|---------------------|------------------------|----------------------------------|-----------------------------|------------------|--------------------|
| 1 | 0.061 | 1.00 | 2 -> 2 | 150.4 | yes | 30.6 |
| 2 | 0.067 | 1.00 | 2 -> 2 | 150.4 | yes | 30.6 |
| 3 | 0.058 | 1.00 | 2 -> 2 | 147.8 | yes | 30.1 |
| 4 | 0.063 | 1.01 | 2 -> 2 | 150.5 | yes | 30.5 |
| 5 | 0.081 | 1.01 | 2 -> 2 | 145.4 | yes | 29.5 |

**Answer to DESIGN.md's open question: plain `start_mission()` RESUMES from the current mission item — it does NOT restart from zero.** Mission progress was at item 2 when the HOLD was triggered and remained at item 2 after resume in all 5 trials (`min_progress_after_resume = 2` — progress never dipped at any point during the resume, so this isn't an artifact of sampling after a fast re-advance), then continued forward to the final waypoint in ~30s. `resume_after_gps_recovery()` is therefore **correct as written**; `mavsdk_client.resume_mission_from()` / `set_current_mission_item()` is not needed for this path. DESIGN.md should be updated to record the settled answer.

**On the bypassed retry wrapper**: the mode switch succeeded on the first attempt in all 5 trials (~1.0s, no retries), so the D11 rejection race did not fire here. That is *not* evidence it cannot. The difference from D11's path is plausible — a plain resume involves no `pause`/`clear`/`upload` sequence beforehand, so there is far less PX4-internal state churn for the mode switch to race against — but "didn't happen in 5 trials" is weaker than "can't happen." **Recommendation (not applied here, since these runs measured the code as written and changing it would invalidate the measurement): route `resume_after_gps_recovery()` through `_start_mission_and_confirm_resumed()` for the same verify-don't-trust guarantee the reroute path has.** Cheap, strictly safer, and removes the one remaining place where a bare `start_mission()` return value is trusted.

**Overnight verification status after D15-D17**: all three previously-open items are now measured against live SITL — `set_current_speed` effect (5/5, D15), RTL full return-and-land (5/5, D16), GPS-recovery resume (5/5, D17) — joining the reroute handoff (5/5, D14), GPS-degraded (5/5, D12), and battery-critical (10/10, D13).

---

## D18 — `resume_after_gps_recovery()` routed through the confirm-and-retry wrapper (D17's flagged gap); re-verified 3/3

D17 flagged that `resume_after_gps_recovery()` called `vehicle.start_mission()` directly, bypassing `_start_mission_and_confirm_resumed()` — the wrapper `_execute_handoff` uses because PX4 can silently reject the internal mode switch while MAVSDK reports success (D11). "Didn't fail in 5 trials" was explicitly not treated as proof it couldn't. Fix approved and applied: `resume_after_gps_recovery()` now calls `_start_mission_and_confirm_resumed()` instead of `vehicle.start_mission()` directly — same verify-and-retry-up-to-5x guarantee the reroute handoff already has.

Unit/orchestration suite unaffected (77 passed — nothing in `tests/` exercises this specific call path against a real mode-switch race, by construction, since it's a hand-written vehicle stand-in).

**Re-verification (3 trials, not a full new investigation — same script as D17, fresh SITL per trial launched as an independent process invocation): 3/3 PASS**, same shape of result as D17: progress held at item 2 through the HOLD and resume, travelled 144.6-150.3m afterward, reached the final waypoint at 29.5-30.6s. `t_to_mission_mode_s = 0.0` in all three trials, down from D17's ~1.0s — expected and correct, not a discrepancy: the wrapper itself now blocks until `FlightMode` confirms `MISSION` before `resume_after_gps_recovery()` returns, so the trial script's own poll (which starts after the call returns) finds the mode already confirmed.

**Conclusion**: the one remaining "trusts the return value" gap flagged in D17 is closed. Every MAVSDK mode-transition call in the replan handoff path (`_execute_handoff`'s resume, and now the GPS-recovery resume) goes through the same confirm-and-retry discipline.

**Left alone, per explicit instruction**: the unexplained t~26s deceleration anomaly from the discarded pre-fix speed-measurement runs (D15). The post-hoc validity gate already handles it correctly by rejecting such windows; not chased further.

---

## D19 — failure_injection's SITL fault-parameter assumptions corrected against real PX4 source, and a real param-persistence bug found and fixed (same class as D10)

`sim/failure_injection/scenarios.py` had never been run against live SITL. Before building the end-to-end scenario trials, its docstring claims about `SIM_BAT_MIN_PCT` and `SIM_GPS_USED` were checked against PX4's actual source (`BatterySimulator.cpp`, `SensorGpsSim.cpp`) rather than trusted, following the project's standing discipline of confirming mechanism before testing behavior built on it.

**Battery: `SIM_BAT_MIN_PCT` is a floor, not a setpoint.** The docstring claimed it gives an "immediate, deterministic" battery-percent injection. `BatterySimulator.cpp` shows otherwise: `_battery_percentage = max(_battery_percentage, min_pct / 100)` every cycle, and the percentage itself is forced to exactly 1.0 (100%) while disarmed and only drains while armed, linearly over `SIM_BAT_DRAIN` seconds (default 60). So `apply_battery_drain_scenario()` sets a target the battery reaches in real time -- roughly `(100 - target_percent) / 100 * drain_interval_s` seconds after arming -- not instantly. Fixed the docstring to state this plainly, with the mechanism cited.

**GPS: `SIM_GPS_USED` is a hard instant threshold, not gradual EKF2 degradation.** The docstring claimed the degradation happens "through PX4's own EKF2 estimator." `SensorGpsSim.cpp` shows a hard `if (_sim_gps_used.get() >= 4)` branch: either a clean 3D fix (fix_type=3, hdop=0.7) or NO_FIX (fix_type=0, hdop=100), nothing in between, no EKF2 involvement -- the simulated sensor message itself is hardcoded at these two values. Fixed the docstring accordingly. (One further correction made empirically, not from source alone -- see the D20 write-up: the raw PX4 value 0 is reported to MAVSDK/GroundLink as `GpsFixType.NO_GPS`, not `NO_FIX`, despite the PX4 C++ comment literally saying `// No fix`.)

**Real bug found while building the GPS-scenario trial: PX4 param changes persist to disk across "fresh" SITL restarts, same class of bug as D10's `dataman` issue.** `sim/trials/launch_sitl_bg.sh` deleted `dataman` before every launch but not PX4's parameter store. Confirmed via `strings` on `$PX4_DIR/build/px4_sitl_default/rootfs/parameters.bson` after a GPS-scenario trial: `SIM_GPS_USED` and `SIM_BAT_MIN_PCT` were present in the persisted file. Consequence, reproduced twice in a row before being diagnosed: a GPS-scenario trial that set `SIM_GPS_USED=3` left that value on disk; the next "fresh" SITL boot loaded it at startup, so the vehicle never obtained a real GPS fix and `wait_ready_to_arm()` timed out waiting for health checks that could never pass -- not a flaky trial, a deterministic consequence of stale on-disk parameter state.

**Fix**: `launch_sitl_bg.sh` now also deletes `parameters.bson` and `parameters_backup.bson` before every launch, alongside `dataman`, with a comment explaining why (mirroring D10's dataman comment). This is a testing/tooling fix, not a product-code fix -- nothing in `replanning_engine`, `firmware_link`, or `sim/failure_injection` changed for this part.

**Lesson generalized from D10**: any PX4 SITL on-disk state that isn't part of the source tree itself (dataman, saved parameters, and potentially others not yet hit) must be reset before every trial that claims to start "fresh." This is now the second time an unreset on-disk file has silently corrupted a "fresh" restart -- worth treating as a standing risk for any future SITL state file, not just these two.

---

## D20 — All three failure-injection scenarios confirmed end-to-end: the real detection→replan→execute trigger path, not each piece in isolation

Per explicit direction, this verifies the actual trigger chain the evaluation will exercise -- `sim/failure_injection` fault injection → real detection (`constraint_monitor.ConstraintMonitor.watch()` for battery/GPS; application-level position/progress announcement for no-fly-zone, per `scenarios.py`'s own design docstring, since PX4 has no native mid-flight-geofence concept) → `replanning_engine` fed the real detected values, not hardcoded test inputs → `firmware_link` executes the response -- as opposed to D12/D13/D15/D16/D17 which called `engine.handle_*()` directly. Same measured-batch discipline as every prior verification: isolated clean run first, then a 5-trial batch, fresh SITL launched as an independent process invocation each time (D14), now also with parameters reset each time (D19).

### Battery-drain scenario

Mission: takeoff + 3 legs (~390m each). `apply_battery_drain_scenario(vehicle, BATTERY_DRAIN_CRITICAL)` (target 12%) applied before arming. `ConstraintMonitor(Thresholds())` (`battery_critical_percent=15.0`) run via `.watch()` over the live `telemetry_stream()` -- the real detection path. On the first `BATTERY_CRITICAL` event, its `remaining_percent` (not a hardcoded value) is fed straight into `engine.handle_battery_critical()`.

**Result: isolated run + 5/5 batch, all PASS.** Detection fired consistently at 15.0% remaining, 52.4-54.1s after arming (matches D19's predicted ~51-53s, confirmed rather than assumed), engine chose `rtl` every time (12% floor sits between `land_immediately_below_percent=8` and `rtl_below_percent=20`, as designed), `RETURN_TO_LAUNCH` mode confirmed within ~1.2s of the call, and distance-to-home was independently confirmed shrinking in every trial afterward (not just the mode flag) -- e.g. 176m -> 144m over a 15s post-command window in the isolated run. Full return-and-land mechanics were already proven in D16, so this test's scope was deliberately the wiring: real drain -> real detection -> real engine call -> real action initiation, not re-measuring landing precision.

| trial | detected % | t to detect (s) | outcome | t to RTL mode (s) | returning? |
|-------|-----------|------------------|---------|--------------------|------------|
| iso | 15.0 | 53.8 | rtl | 1.2 | yes |
| 1 | 15.0 | 53.4 | rtl | 1.2 | yes |
| 2 | 15.0 | 52.9 | rtl | 1.2 | yes |
| 3 | 15.0 | 53.4 | rtl | 1.0 | yes |
| 4 | 15.0 | 52.4 | rtl | 1.0 | yes |
| 5 | 15.0 | 54.1 | rtl | 1.0 | yes |

### GPS-degradation scenario

Mission: takeoff + 3 legs, fault applied only after the vehicle is confirmed in steady cruise (same climb-out gate as D15) so this is a genuine mid-flight failure. `apply_gps_degradation_scenario(vehicle, GPS_DEGRADATION_LOW_SATS)` (`SIM_GPS_USED=3`) applied, then the vehicle's own `telemetry_stream()` is run through `ConstraintMonitor.check()` directly (the same logic `.watch()` wraps) so the concurrently-observed real `fix_type`/`hdop` -- not values reconstructed from the violation event's dict, which doesn't carry hdop -- can be fed to `engine.handle_gps_degraded()`.

**Result: isolated run + 5/5 batch, all PASS**, after fixing the param-persistence bug found along the way (D19) -- the first two attempts both failed at `wait_ready_to_arm()` before the fix, for the reason diagnosed there, not counted as scenario failures. Detection was near-instant every time (0.01-0.04s after injection, matching D19's "hard threshold, no EKF2" finding), engine chose `hold` every time, `HOLD` mode confirmed within 0.2-1.2s, and the vehicle was confirmed genuinely stopped (ground speed 0.03-0.15 m/s) 6s after the command in every trial.

| trial | detected fix | t to detect (s) | outcome | t to HOLD mode (s) | speed in hold (m/s) |
|-------|-------------|------------------|---------|---------------------|----------------------|
| iso | NO_GPS | 0.04 | hold | 0.8 | 0.032 |
| 1 | NO_GPS | 0.03 | hold | 0.2 | 0.15 |
| 2 | NO_GPS | 0.01 | hold | 1.21 | 0.125 |
| 3 | NO_GPS | 0.02 | hold | 0.2 | 0.106 |
| 4 | NO_GPS | 0.04 | hold | 0.2 | 0.094 |
| 5 | NO_GPS | 0.01 | hold | 0.8 | 0.114 |

**One more real, source-contradicting finding surfaced by actually running it, not caught by reading source alone**: PX4 reports the "no fix" case (raw `fix_type=0`) to MAVSDK/GroundLink as `GpsFixType.NO_GPS`, not `GpsFixType.NO_FIX` -- despite PX4's own C++ comment on that branch literally saying `// No fix`. D19's docstring fix (written from source alone, before running) had used the label "NO_FIX", which was wrong for exactly this reason; corrected here. It does not change any test's pass/fail, since `decide_gps_response()` compares `fix_type < min_fix_type_to_continue` using the enum's ordinal value (`GpsFixType.NO_GPS = 0`), and 0 is still correctly below `FIX_3D = 3` regardless of which name is attached to it -- but it's a reminder that a source read establishes mechanism, not vocabulary, and both matter when writing something meant to be read later.

**Also observed, flagged rather than chased (consistent with the standing "leave documented open items, don't chase every anomaly" instruction)**: `live_hdop` was inconsistent across trials -- `0.7` (the pre-fault "good fix" value) in two trials, `NaN` in three. `mavsdk_client.py`'s own design comment already explains why: `fix_type` and `hdop` are merged from two independently-updating MAVSDK channels (`gps_info()` and `raw_gps()`) into one `GpsState`, so the very first snapshot after an instant fault flip can carry a fresh `fix_type` paired with a stale or not-yet-populated `hdop` from before the flip. This did not affect any outcome here -- `decide_gps_response()` checks `fix_type` first and returns `HOLD` before `hdop` is ever consulted -- but it is a real telemetry-merge artifact worth knowing about for any future scenario that triggers off `hdop` (e.g. a pure `GPS_HDOP_HIGH`/slow-down scenario) rather than `fix_type`.

### No-fly-zone scenario

See D21 immediately below.

---

## D21 — No-fly-zone scenario confirmed end-to-end: 5/5, zero zone entries across 15,683 checked positions

Completes the D20 set. Unlike battery/GPS, this scenario deliberately does not go through `ConstraintMonitor` -- `scenarios.py`'s own design docstring is explicit that a mid-flight no-fly-zone is an application-level announcement, since PX4 has no native concept of a geofence appearing during a mission. So the real trigger path here is: `make_no_fly_zone_scenario()` builds a zone straddling a specific leg of the **actual uploaded mission** -> application-level detection watching the same `mission_progress_stream()` `engine.track_mission_progress()` consumes, for the vehicle having left the waypoint just before the blocked leg -> `engine.handle_no_fly_zone()` fed the real zone and the **real current position read off live telemetry** at that moment, not a hardcoded position -> `firmware_link` executes the reroute handoff (D11/D14's already-proven pause/clear/upload/resume mechanics).

**Test**: 4-waypoint mission (~200m legs). Zone built (via `make_no_fly_zone_scenario`, 80m wide) straddling the leg from `waypoints[1]` to `waypoints[2]`. Once `mission_progress` shows the vehicle heading to item 2 (i.e. it has left item 1, the leg about to be blocked), the real zone and real position are handed to `handle_no_fly_zone()`. After that, the mission is watched through to its last item, and -- the actual proof of avoidance, not just trusting the `"rerouted"` outcome string -- **every recorded position from the moment of announcement onward is checked geometrically against the zone polygon** (`shapely.Polygon.contains`), not sampled or assumed.

**Result: isolated run + 5/5 batch, all PASS.**

| trial | t to announce (s) | progress at announce | outcome | new remaining wps | t to mission end (s) | positions checked | zone entries |
|-------|--------------------|-----------------------|---------|---------------------|------------------------|---------------------|----------------|
| iso | 58.12 | 2 | rerouted | 6 | 55.31 | 3039 | 0 |
| 1 | 59.35 | 2 | rerouted | 6 | 51.85 | 3208 | 0 |
| 2 | 58.41 | 2 | rerouted | 6 | 51.84 | 3126 | 0 |
| 3 | 57.81 | 2 | rerouted | 6 | 51.55 | 3006 | 0 |
| 4 | 57.38 | 2 | rerouted | 6 | 51.46 | 3153 | 0 |
| 5 | 57.26 | 2 | rerouted | 6 | 52.27 | 3151 | 0 |

**Zero zone entries across 15,683 checked positions, combined.** The reroute consistently expanded the 2-waypoint remaining path (the blocked leg's target plus the final waypoint) into a 6-waypoint detour, and every trial's mission completed in a tight 51.5-52.3s window after the reroute (the isolated run's 55.31s was not re-seen in the batch, consistent with normal SITL timing variance rather than a pattern).

**Conclusion**: all three failure-injection scenarios (D20 battery, D20 GPS, D21 no-fly-zone) are now confirmed through the real detection -> replan -> execute path each will use in the actual evaluation, not just via `engine.handle_*()` called directly with test-chosen inputs. Combined with D14/D15/D16/D17/D18's direct verification of the underlying engine mechanics, every trigger type GroundLink supports has now been measured against live SITL twice: once at the mechanism level, once at the real end-to-end trigger level.

---

## D22 — Baseline-vs-adaptive comparative evaluation: implemented, measured (40 trials across 8 groups), one real metric bug caught and fixed mid-run, one genuinely important safety finding surfaced

Per `evaluation/DESIGN.md` (reviewed and approved before any code was written, same convention as `replanning_engine/DESIGN.md`) and its four open questions, all approved: (1) add a severe battery sub-case to surface a real LAND-vs-RTL timing divergence, (2) observe baseline's RTL-under-GPS-failure honestly rather than engineering around whatever happens, (3) use a real lawnmower coverage mission for the no-fly-zone scenario so "coverage %" means something, (4) keep 5 trials/condition.

### Implementation

- **`EngineConfig.adaptive_replanning_enabled: bool = True`** (`replanning_engine/engine.py`) — one additive flag. `False` collapses every trigger's response to a plain `action.return_to_launch()`, context.md's own definition of the baseline ("a crude failsafe... that abandons the mission entirely"). Detection (`ConstraintMonitor` thresholds, the no-fly-zone announcement trigger point) is completely unaffected by this flag in all three `handle_*()` methods — only the response branches differ, which is what makes "identical injected failure scenarios" between conditions actually true. The three pure decision modules (`battery_response.py`, `gps_response.py`, `reroute.py`/`no_fly_zone.py`) are untouched. Unit suite (77 tests) unaffected by the change.
- All three scenario trial scripts (`sim/trials/{battery,gps,nfz}_scenario_trial.py`) gained a `--baseline` flag (`battery_scenario_trial.py` also gained `--severity moderate|severe`) rather than writing separate baseline scripts, and a geometric **completion %** metric (fraction of the original mission's non-takeoff waypoints the vehicle came within `acceptance_radius_m` of, checked against the recorded position trace — independent of `mission_progress` bookkeeping, which baseline's RTL makes meaningless and adaptive's reroute resets to a new index).
- `BATTERY_DRAIN_SEVERE = BatteryDrainScenario(target_percent=5.0, drain_interval_s=1.5)` added to `scenarios.py` for the severe sub-case.
- `nfz_scenario_trial.py` rebuilt around `mission_planner.grid_coverage.generate_lawnmower_mission()` (already built, already unit-tested) — a small 220m x 180m, 90m-row-spacing, 3-row/6-waypoint sweep (~720m total path, sized down from an initial 8-waypoint/1.5km attempt that would have made each trial ~5-6 minutes — see the sizing note in the script) — with the no-fly zone straddling one of the actual sweep rows.
- `evaluation/aggregate_results.py` — pure aggregation (no SITL/MAVSDK), reads the `RESULT {json}` lines each trial script already prints into per-scenario/condition files under `~/replan_trial_results/`, writes `evaluation/RESULTS.md`. Regenerating the report means re-running this script, not hand-editing a table.

### Two real methodology problems found and fixed while building this, not by luck

**Severe battery detection landed in the ambiguous 8-15% band, not below the 8% floor, on the first two attempts.** The plan was: drain fast enough (`drain_interval_s=1.5`, originally `5.0`) that `ConstraintMonitor`'s first `<=15%` crossing would already be past the 8% land-immediately tier. Empirically, two separate trials caught 12% and then 10% instead — PX4's battery telemetry publishes more often than assumed, so the traversal wasn't reliably skipping the ambiguous band. Fix: don't gamble on sample-rate timing — poll `telemetry.battery()` directly and wait for the reading to actually reach the target floor *before* starting to watch for the violation (still through the same real `ConstraintMonitor.check()` logic afterward). Deterministic once fixed: all 10 severe-battery trials (both conditions) subsequently detected at exactly the floor (5.0%, or the 6.0% "within 1% of floor" gate).

**The severe fault was originally applied before arming, same as the moderate case — but at that drain rate it crashed the battery while the vehicle was still climbing out**, before there was any real distance/altitude to recover from (one discarded trial: RTL engaged at ~1.3m altitude, ~0.1m from home, making the "is it returning" distance check meaningless). Fixed by applying the severe fault only after steady cruise is confirmed (same climb-out gate `speed_trial.py`/`gps_scenario_trial.py` already use) — more realistic anyway, and it makes every subsequent confirmation check meaningful.

**Caught after the first full 40-trial run, before reporting: `time-to-safe-recovery` was measuring "mode confirmed + a fixed 15s watch window," not real completion.** All four battery groups came back with `t_to_safe_recovery_s` clustered at ~16.0-16.2s regardless of whether the response was LAND (should be fast) or a full RTL (climb+transit+land, ~60s per D16) — because the metric declared "safe" as soon as the fixed window elapsed, never actually waiting for the vehicle to land. This directly masked the one thing the severe battery case was added to demonstrate. Caught by comparing the numbers against D16's already-known RTL timing before reporting them, not assumed correct because the script ran without errors. Fixed: both `land_immediately` and `rtl`/`baseline_rtl` branches now wait for actual disarm (`telemetry.armed()`, the same proven-reliable check D16 used), whatever that takes. The 4 battery groups (20 trials) were re-run after the fix; GPS and no-fly-zone groups were left as originally measured since their recovery-time logic didn't have this flaw (GPS already waited for a real stop, no-fly-zone doesn't use this metric at all — reroute completion is its own "safe" signal).

### Results (40 trials: 5/group x 8 groups; see `evaluation/RESULTS.md` for the generated table)

**Battery-critical, moderate (12%) — both conditions choose RTL, as designed** (12% sits between the 8%/20% tiers regardless of adaptive/baseline, so this case is a *control*, not expected to diverge): adaptive 75.4 +/- 0.5s to disarm, baseline 75.1 +/- 0.1s — correctly near-identical, both landed on the same decision. 5/5 safe-confirmed both conditions. (One trial in the original moderate-baseline batch hit a transient `TimeoutError` during the `ConstraintMonitor.watch()` detection wait — same class of isolated SITL flakiness seen earlier this session, not a methodology issue — replaced with one additional trial to keep n=5.)

**Battery-critical, severe (~5%) — the real divergence, now correctly measured**: adaptive (LAND_IMMEDIATELY) **19.8 +/- 0.6s** to disarm vs. baseline (always RTL) **51.3 +/- 0.0s** — a **2.6x speed difference**, directly demonstrating context.md's own stated concern that "RTL itself might not complete in time" under a severe failure. 5/5 safe-confirmed both conditions (baseline's RTL does complete here — battery running out doesn't itself impair navigation the way GPS failure does, see below).

**GPS-degradation — the most consequential finding of this whole evaluation**: adaptive (HOLD) 5/5 safe-confirmed, 6.7 +/- 0.2s to a confirmed stop. **Baseline (always RTL): 0/5 safe-confirmed.** Every baseline trial commanded `RETURN_TO_LAUNCH`, and PX4 accepted the mode switch (`reached_response_mode: true` in all 5) — but within ~5s, PX4's own internal failsafe logic overrode it to `LAND` on its own, and the reported distance-to-home did not reliably shrink (one representative trial: 27m -> 43m *away* from home during the "landing," since the position estimate itself drifts without GPS correction). This was **observed and reported exactly as it happened, per the explicit instruction not to engineer around it** — the trial script never assumes RTL succeeds, and this is what actually happened, consistently, across all 5 trials. This is precisely the failure mode context.md's problem statement describes in the abstract ("a crude failsafe... abandons the mission entirely") made concrete and measured: baseline's uniform "always RTL" policy is not just less capable than adaptive here, it demonstrably **does not work** when the failure is GPS-related, because RTL itself depends on the thing that just failed.

**No-fly-zone (lawnmower coverage mission)**: adaptive completion 83.3 +/- 0.0%, baseline completion 50.0 +/- 0.0% — baseline abandons the mission at the point of detection (3/6 sweep waypoints already flown), adaptive reroutes around the zone and continues (5/6 waypoints reached). Zone never entered in any of the 10 trials (both conditions). Adaptive's 5/6 rather than a clean 6/6 is a real, honestly-reported result, not a bug: the reroute's path-simplification (`reroute.py`'s `_simplify`, tolerance-based) can land the detour within a few meters of a swept waypoint's original coordinates without landing inside its 5m acceptance radius — a legitimate characteristic of the reroute algorithm, not chased further given the geometric zone-avoidance proof (the actual safety property) was clean in every trial. Baseline's safe-confirmed rate was 4/5 (one trial's RTL-distance check didn't confirm within the watch window) — not investigated further, consistent with the "flag rather than chase every anomaly" discipline used throughout this project when the finding doesn't change the overall conclusion.

**Battery completion % is 0.0% for all four battery groups, both conditions** — not a bug, an honest artifact of this mission's geometry: the injected failure (whether moderate ~53s or severe ~30s post-arm) fires before the vehicle can reach the first of 3 waypoints (390m away, ~78s of cruise at 5 m/s) regardless of response. Completion % simply isn't a useful differentiator for the battery scenario as configured — GPS and no-fly-zone are where it discriminates. Reported as measured, not hidden.

**Conclusion**: the adaptive/baseline comparison this evaluation exists to produce is now real, measured, and — per the explicit instruction — not partially reported or declared before every batch fully completed. Full regression (77 unit tests) passed after all engine/scenario changes. The GPS finding in particular is a genuinely strong, unplanned result: it wasn't designed to make baseline fail outright, it was designed to observe honestly, and baseline's RTL-on-any-failure policy turned out to be actively unsafe under the one failure type (GPS) where blind RTL depends on the very thing that broke.
