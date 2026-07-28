# GroundLink

**Real-Time Adaptive Drone Mission Planner** — a ground control system that plans drone missions on top of real flight-controller firmware (PX4) and *replans them live* when flight conditions change, instead of just aborting to Return-to-Launch.

Most mission planners upload a route and fly it blind. If the battery drains faster than expected, GPS degrades, or a no-fly zone appears mid-flight, they fall back to a crude failsafe that abandons the mission — even when a smarter, partial-completion path was possible. GroundLink treats replanning as a first-class capability: it monitors live telemetry, detects constraint violations (low battery, degraded GPS, geofence/no-fly-zone breaches) in real time, and automatically reroutes the remaining mission instead of aborting it.

Built on **PX4** (via **MAVSDK-Python**), validated in **SITL** (Software-In-The-Loop simulation, using PX4 + Gazebo Harmonic), with the same code path intended to work on real hardware.
> See [context.md](context.md) for the full problem statement, and [decisions.md](decisions.md) for a complete, dated log of every architectural decision and every bug found along the way — including three real PX4/MAVSDK bugs found and fixed during development (not hidden, not glossed over).

---

## Table of contents

- [What's built right now](#whats-built-right-now)
- [Architecture](#architecture)
- [Getting started](#getting-started)
  - [Linux (native)](#linux-native)
  - [Windows](#windows)
  - [WSL2 on Windows](#wsl2-on-windows-the-fully-tested-path)
  - [macOS](#macos)
- [Running the system](#running-the-system)
- [Project structure](#project-structure)
- [Documentation](#documentation)

---

## What's built right now

| Component | Status |
|---|---|
| `mission_planner` — waypoint model + lawnmower coverage-grid generator | ✅ Built, tested (7 tests) |
| `constraint_monitor` — battery/GPS/geofence threshold checks, structured violation events | ✅ Built, tested (10 tests) |
| `firmware_link` — MAVSDK integration: connect/arm/upload/telemetry | ✅ Built, verified against real PX4 SITL (8/8 trials, no flakiness — see [decisions.md D10](decisions.md)) |
| `replanning_engine` — battery-critical return, GPS-degraded downgrade | ✅ Built, verified against real PX4 SITL (10/10 and 5/5 trials — see [decisions.md D12](decisions.md)/[D13](decisions.md)) |
| `replanning_engine` — no-fly-zone reroute (grid A*) | 🟡 Built, root-cause bug found+fixed and confirmed correct in 2 isolated SITL runs, but batch-measured reliability still unresolved (0/5 — open issue, see [decisions.md D11](decisions.md)) |
| `sim/failure_injection` — battery/GPS/no-fly-zone scenario configs | ✅ Built (real PX4 SITL params), unit-tested; live-SITL injection itself not yet verified |
| `dashboard` — live Streamlit ground station | ⬜ Not started |

See [replanning_engine/DESIGN.md](replanning_engine/DESIGN.md) for the design and decisions.md D8-D13 for the full, honest verification history — including the parts that are confirmed vs. still open.

Every claim of "verified" or "tested" in this repo means it was actually run against a real PX4 SITL instance, not mocked — that's a deliberate project convention (see `context.md`). Where something hasn't been tested, this README says so explicitly.

## Architecture

```
mission_planner/      Firmware-agnostic mission definitions (Waypoint, Mission)
                       + lawnmower coverage-grid generation.
        |
        v
firmware_link/         The ONLY package that imports MAVSDK. Converts Mission ->
                       PX4 mission uploads, streams telemetry back as
                       vehicle-agnostic TelemetrySnapshot objects.
        |
        v
constraint_monitor/    Pure function: (Thresholds, TelemetrySnapshot) -> list of
                       structured ViolationEvent (battery/GPS/geofence). Works
                       against a live stream or a replayed log file.
        |
        v
replanning_engine/     [in design] Consumes ViolationEvents, recomputes the
                       remaining mission (reroute around no-fly zones, safe
                       return on critical battery, conservative mode on GPS
                       degradation), hands the new plan back to firmware_link.
        |
        v
dashboard/             [not started] Streamlit UI: live map, telemetry, replan log.
```

The layering is deliberate: `replanning_engine` depends only on `mission_planner` and `constraint_monitor` types, never on MAVSDK directly — the MAVLink layer could be swapped (e.g. for pymavlink, or ArduPilot) without touching planning/replanning logic.

## Getting started

**PX4 SITL requires Linux.** The table below tells you which path to take depending on your OS.

| Your OS | Path |
|---|---|
| Linux (Ubuntu 22.04/24.04) | Follow [Linux (native)](#linux-native) directly |
| Windows | Use WSL2 — see [Windows](#windows) then [WSL2 on Windows](#wsl2-on-windows-the-fully-tested-path) |
| macOS | See [macOS](#macos) (PX4's own Homebrew-based setup — not tested by this team) |

### Linux (native)

Everything below has been verified on **WSL2 Ubuntu 24.04**, which — for this purpose — *is* Ubuntu 24.04: same kernel-level userspace, same `apt` packages, same PX4 build. The steps are identical on bare-metal/native Ubuntu 22.04 or 24.04; you just don't need a "enter WSL" step. Skip straight to [Running the system](#running-the-system).

If you're on a different Linux distro, PX4's `Tools/setup/ubuntu.sh` is Debian/Ubuntu-specific — you'll need your distro's equivalent build dependencies (`build-essential`-equivalent, `cmake`, `ninja-build`) and PX4's own instructions for other distros.

### Windows

Native Windows cannot build or run PX4 SITL (Gazebo + PX4's toolchain need Linux). Use **WSL2** — it is not optional for this project on Windows.

1. If you don't already have it: `wsl --install` in PowerShell (as Administrator), then restart. This installs WSL2 with an Ubuntu distro by default.
2. Once you have a WSL2 Ubuntu distro, everything runs *inside* it — see [WSL2 on Windows](#wsl2-on-windows-the-fully-tested-path) below.
3. The project's source code stays on the Windows filesystem (e.g. `C:\Users\you\GroundLink`, visible from WSL at `/mnt/c/Users/you/GroundLink`) so you can edit it with normal Windows tools (VS Code, etc.) — only the Python venv and the PX4 build itself need to live on WSL's native Linux filesystem, for performance and reliability (see [decisions.md D4](decisions.md)).
4. A Streamlit dashboard (once built) running inside WSL2 on `localhost:8501` is reachable from a normal Windows browser with no extra configuration — WSL2's default networking forwards `localhost` automatically. Confirmed by direct test, not assumed (see decisions.md D3).

### WSL2 on Windows (the fully tested path)

This exact sequence is what's been run, repeatedly, against real SITL. Open a WSL2 Ubuntu terminal (`wsl -d Ubuntu` from PowerShell, or just open the "Ubuntu" app) and run everything below inside it.

**1. Build tools (needs sudo — asks once):**

```bash
sudo apt update && sudo apt install -y build-essential cmake ninja-build
```

**2. Python virtual environment** (native WSL filesystem, not `/mnt/c` — venvs over the Windows/WSL filesystem bridge are slow and occasionally flaky):

```bash
python3 -m venv --without-pip ~/groundlink-venv
source ~/groundlink-venv/bin/activate
curl -sS https://bootstrap.pypa.io/get-pip.py | python
```

(`ensurepip`/system `pip3` isn't reliably present out of the box on Ubuntu 24.04, and installing it needs sudo too — this route avoids needing sudo for the Python side at all. See [decisions.md D5](decisions.md).)

**3. Install GroundLink:**

```bash
cd /mnt/c/Users/<you>/path/to/GroundLink   # wherever you cloned this repo
pip install -e '.[dev]'
```

**4. PX4 SITL** (one-time clone + build; pinned to v1.17.0, which is the current stable release with Gazebo Harmonic as its SITL simulator — see [decisions.md D7](decisions.md) for why not jMAVSim):

```bash
cd ~
git clone --branch v1.17.0 --depth 1 --recursive --shallow-submodules \
  https://github.com/PX4/PX4-Autopilot.git

cd ~/PX4-Autopilot
Tools/setup/ubuntu.sh --no-nuttx   # needs sudo once; --no-nuttx skips the ARM
                                    # cross-toolchain for real hardware, which
                                    # isn't needed for SITL-only work (D6)

source ~/groundlink-venv/bin/activate
pip install -r Tools/setup/requirements.txt   # PX4's own Python build deps,
                                                # into the SAME venv (D5)

make px4_sitl_default   # first build compiles the whole flight stack,
                         # several minutes
```

Now jump to [Running the system](#running-the-system) below.

### macOS

**Not tested by this team** — no Mac was available during development. These steps are transcribed directly from PX4's own official `Tools/setup/macos.sh` (Homebrew-based), which is PX4's documented, maintained path for macOS. If something doesn't match reality on your machine, PX4's own docs/issues are the authority, not this README.

```bash
git clone --branch v1.17.0 --depth 1 --recursive --shallow-submodules \
  https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
Tools/setup/macos.sh --sim-tools   # installs Homebrew (if missing), the
                                    # px4-dev toolchain formula, and px4-sim
                                    # (Gazebo, via PX4's own Homebrew tap)

python3 -m venv ~/groundlink-venv
source ~/groundlink-venv/bin/activate
pip install --upgrade pip
pip install -r Tools/setup/requirements.txt

make px4_sitl_default
```

Then clone GroundLink itself and `pip install -e '.[dev]'` into the same venv, as in the WSL2 steps above. There is no WSL-style networking caveat on macOS/native Linux — everything runs in one OS.

## Running the system

**Launch PX4 SITL** (from inside `~/PX4-Autopilot`, or point `PX4_DIR` at wherever you cloned it):

```bash
cd /path/to/GroundLink
bash sim/launch_sitl.sh
```

This script does more than a bare `make px4_sitl gz_x500` — it also kills any leftover PX4/Gazebo processes and clears PX4's on-disk mission store (`dataman`) before every launch. Both of those are **required**, not optional: a stale `dataman` file left over from a previous SITL session was a real, confirmed bug that silently corrupted every mission upload until it was found and fixed — see [decisions.md D10](decisions.md) for the full story. Don't launch SITL any other way without replicating what this script does.

Leave that running in one terminal. In a second WSL2/Linux/macOS terminal, with the venv active:

```bash
source ~/groundlink-venv/bin/activate
cd /path/to/GroundLink

# Run the test suite (17 tests, no SITL needed):
pytest -v

# Connect to the running SITL instance, arm, upload a 3-waypoint mission,
# and stream live telemetry to stdout:
python -m firmware_link.connectivity_check
```

`connectivity_check.py` is the "does the plumbing actually work" smoke test — verified repeatedly against real SITL: the vehicle connects, arms, uploads and flies a real 3-waypoint mission (climbing to ~15m and transiting all waypoints), and streams battery/GPS/position telemetry the whole time.

## Project structure

```
GroundLink/
  mission_planner/      Waypoint + Mission dataclasses, lawnmower grid generator
  firmware_link/         MAVSDK integration layer (the only package that imports mavsdk)
  constraint_monitor/    Battery/GPS/geofence threshold checks -> structured events
  replanning_engine/     Core novelty: live mission reroute on constraint violation
                         (in design -- see replanning_engine/DESIGN.md)
  dashboard/             Streamlit ground station UI (not started)
  sim/                   SITL launch script, failure-injection configs (planned)
  tests/                 pytest suite + fixtures (e.g. a sample telemetry replay log)
  context.md             Full problem statement, objectives, tech stack
  decisions.md           Dated log of every architectural decision and bug found
```

- **`mission_planner/`** — firmware-agnostic mission representation (`Waypoint`, `Mission`) and `generate_lawnmower_mission()`, a boustrophedon coverage-grid generator over a boundary polygon.
- **`firmware_link/`** — the only place MAVSDK is imported. Defines `TelemetrySnapshot` and friends (vehicle-agnostic dataclasses the rest of the codebase depends on instead of MAVSDK types directly) and `GroundLinkVehicle`, a thin async wrapper for connect/arm/upload-mission/stream-telemetry.
- **`constraint_monitor/`** — `ConstraintMonitor.check()` is a pure function of `(Thresholds, TelemetrySnapshot) -> list[ViolationEvent]`. Works identically against a live MAVSDK stream (`.watch()`) or a replayed JSONL log (`.replay()` / `log_replay.load_snapshots_jsonl()`). Violations are structured `ViolationEvent` objects (kind/severity/message/details), not print statements.
- **`replanning_engine/`** — not implemented yet; see [replanning_engine/DESIGN.md](replanning_engine/DESIGN.md) for the approach.
- **`sim/launch_sitl.sh`** — the correct way to start SITL (see [Running the system](#running-the-system) above for why it's not just a bare `make` command).
- **`tests/`** — pytest suite; `tests/fixtures/` holds replay data.

## Documentation

- [context.md](context.md) — full problem statement, objectives, system architecture, tech stack, and team conventions.
- [decisions.md](decisions.md) — the complete decision log, in date order, including the ones that were later found to be wrong and explicitly corrected (see D9 vs D10) rather than silently rewritten. This is the most detailed technical record in the repo.
- [replanning_engine/DESIGN.md](replanning_engine/DESIGN.md) — design for the core novelty piece (written before implementation, per project convention: review the approach before the code exists).
