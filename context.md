# GroundLink — Project Context

## Team
Team 11 — Anshaditya Sharma (23BRS1204), Animesh Ojha (23BRS1332), Balajee Jivesh (23BRS1237)
Course: Autonomous Drones

## Title
**GroundLink: Real-Time Adaptive Drone Mission Planner**

## Problem Statement
Most drone mission-planning tools operate on a "plan once, execute blindly" model — a waypoint mission is uploaded to the flight controller and the drone flies it regardless of what happens mid-flight. Real-world conditions rarely cooperate: battery drains faster than estimated, GPS accuracy degrades in certain zones, obstacles or no-fly zones appear that weren't known at planning time. When this happens, most systems fall back to a crude failsafe (Return-to-Launch) that abandons the mission entirely, even when a smarter, partial-completion path was possible.

GroundLink addresses this gap: a ground control system that plans missions on top of a real flight-controller firmware (PX4/ArduPilot) and dynamically replans them in real time when flight conditions change — instead of just aborting.

## Objectives
1. Build a mission planner that generates waypoint/survey-grid missions and uploads them to a PX4/ArduPilot flight stack over MAVLink.
2. Continuously monitor live telemetry (battery, GPS quality, position, attitude) during flight.
3. Detect constraint violations in real time — low battery, GPS degradation, geofence breach, injected obstacle/no-fly zone.
4. Automatically replan the remaining mission — e.g., re-route around a no-fly zone, shorten the survey path and prioritize unfinished high-value waypoints, or compute a safe return path — rather than defaulting to blind RTL.
5. Provide a live ground-station dashboard showing the map, telemetry, and replanning decisions as they happen.
6. Validate the full pipeline in SITL (Software-In-The-Loop) simulation, with the same code path usable on real hardware if available.

## System Architecture

### 1. Mission Planning Layer
Defines missions as waypoint sequences or coverage/survey grids over a target area. Supports manual waypoint entry and auto-generated grid coverage for a given boundary.

### 2. Firmware Integration Layer
Communicates with PX4/ArduPilot (via MAVSDK-Python or pymavlink) over MAVLink to upload missions, command mode changes, and stream telemetry back — battery voltage/percentage, GPS fix type and HDOP, position, velocity, attitude.

### 3. Constraint Monitor
Runs alongside telemetry streaming, checking live values against safety thresholds (battery %, GPS fix quality, geofence boundary, injected obstacle zones). Flags violations as they occur.

### 4. Replanning Engine (core novelty)
On a triggered violation, recomputes the remaining mission instead of default failsafe behavior:
- Battery-critical → computes shortest safe return path, or nearest safe landing point if RTL is no longer feasible.
- GPS degraded → reduces speed / switches to a more conservative navigation mode.
- No-fly zone / obstacle appears → re-routes remaining waypoints around it, preserving as much of the original mission as possible.
- Reprioritizes remaining waypoints by value if not all can be completed.

### 5. Ground Station Dashboard
Live map with drone position and planned/replanned path overlay, telemetry readout, and a log of replanning events with the reason for each decision — this is the primary demo and evaluation artifact.

## Novelty
Unlike a standard mission planner (upload-and-forget) or a pure failsafe system (abort-only), GroundLink treats replanning as a first-class capability — the system reasons about how to still accomplish as much of the mission as possible under a real-time constraint, rather than treating any anomaly as mission-ending. Evaluation directly compares mission completion / area coverage between (a) baseline RTL-on-failure behavior and (b) GroundLink's adaptive replanning, under identical injected failure scenarios in SITL.

## Tech Stack
- **Flight stack**: PX4 (SITL via jMAVSim or Gazebo) — real Pixhawk-class hardware later if available
- **Comms**: MAVLink via MAVSDK-Python (preferred) or pymavlink
- **Replanning logic**: Python — graph/grid-based path re-routing around obstacles, threshold-based constraint checks
- **Dashboard**: Streamlit (fast to build, consistent with team's other project dashboards) or lightweight React + Leaflet if time allows
- **Simulation**: PX4 SITL with injected failure scenarios (simulated GPS degradation, battery drain, geofence/no-fly zone)

## Deliverables
- Working SITL demo: mission execution with live replanning under at least 2–3 distinct failure scenarios
- Ground station dashboard showing real-time telemetry and replanning decisions
- Comparative evaluation: baseline vs. adaptive replanning (mission completion %, time-to-safe-recovery)
- Technical report + presentation (architecture diagram, decisions log, evaluation section)

## Development Environment
- Host OS: **Windows 11**. PX4 SITL does not run natively on Windows — it requires Linux.
- Use **WSL2 (Ubuntu)** for all PX4 SITL work and the Python backend (mission planner, firmware link, constraint monitor, replanning engine).
- Do not assume WSL is already installed/configured with the right Ubuntu version, PX4 build deps, or a working PX4 clone — verify each of these explicitly before relying on them, and report what's missing rather than silently working around it.
- The dashboard (Streamlit) can run inside WSL too (accessible from Windows browser via localhost) — keep everything in one environment rather than splitting Windows-native and WSL-native pieces, to avoid networking/config headaches between the two.
- If WSL is not installed or is an incompatible version, stop and tell me exactly what needs installing rather than attempting a partial workaround.

## Conventions to Follow (match Ansh's existing project style)
- Maintain a `decisions.md` file logging key architectural/technical decisions with rationale, similar to the `bb` project's D29–D30 pattern.
- Keep the replanning logic modular and testable in isolation from the MAVLink/firmware layer (mirrors the reactive/deliberative separation used in the VLM Rover project).
- Prefer real integration (actual PX4 SITL + MAVLink) over mocked/simulated stand-ins wherever feasible — this team consistently favors real hardware/firmware integration over pure-simulation shortcuts.
- Never overstate simulation results as real-flight results in documentation — be precise about what was SITL-tested vs. hardware-tested.