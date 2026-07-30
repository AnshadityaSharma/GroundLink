# How to run GroundLink

Three ways to see this project, from zero effort to fully custom. Start with #1 — it's all most people will need.

---

## 1. View the presentation dashboard (do this first — no install, no setup)

This is the easiest way to see what GroundLink does. It shows real flights that were already recorded from this project's actual test runs.

**Steps:**

1. Open the `dashboard` folder in this repository.
2. Double-click `index.html`.
3. It opens in your normal web browser (Chrome, Edge, Firefox, Safari — any of them work).

That's it. No Python, no simulator, no internet connection required — the file works completely offline and contains everything it needs already embedded inside it.

**What you'll see:** three failure scenarios (a no-fly zone appearing mid-flight, a battery running critically low, and GPS signal being lost), each showing a real recorded drone flight on a map, with a side-by-side comparison of GroundLink's response versus a "dumb" baseline system that only knows how to abort and return home.

**What you can do on the page:**
- Switch between the three scenarios using the tabs near the top.
- Press play to watch the flight happen, or drag the scrubber bar to jump to any moment.
- Speed the playback up (1×, 2×, 4×, 8×).
- Turn the planned route, flown path, restricted zone, or range rings on/off.
- Read the big number at the top of each scenario — that's the headline result (e.g. "83.3% of the survey completed" vs. baseline's "50.0%").

This is a **replay of real recorded flights**, not a live simulation — nothing on this page is invented or simulated live. If you want to watch a flight happen live instead of replayed, see section 2 below.

---

## 2. Run a real, live simulation

This actually boots a real flight-controller simulator (the same software real drones run) and flies a virtual drone through one of GroundLink's failure scenarios live, in real time. This is how the recordings shown in the dashboard were originally produced.

This requires installing some software first, so it's more effort than section 1 — only do this if you want to watch it happen live rather than watch the replay.

### Prerequisites (one-time setup)

The simulator only runs on Linux. If you're on Windows, you'll need **WSL2 with Ubuntu** (Windows' built-in way of running Linux alongside Windows). You'll need, all installed once beforehand:

- **WSL2 + Ubuntu**, if on Windows (`wsl --install` in PowerShell, then restart)
- A **Python environment** with this project installed into it
- **PX4** (the flight-controller simulator software) cloned and built — this is the biggest one-time step, and can take several minutes to compile

Full step-by-step install instructions for all of this are in [`README.md`](README.md#getting-started) — this file assumes that one-time setup is already done and focuses on how to actually *run* something afterward.

### Exact commands to run one scenario

These are the **exact, tested commands for this machine** — copy-paste them one at a time into a fresh WSL2 Ubuntu terminal, starting from `~` with nothing else active. Every line below was actually executed (not guessed) to confirm it works: the repo path, the venv name, the SITL launch, and a full live scenario run (GPS, both adaptive and `--baseline`) were all run start-to-finish and produced a real `RESULT` line.

**Step 1 — go to the repo root:**

```bash
cd /mnt/c/Users/Admin/Desktop/drones_project
```

That's the WSL-side path to this specific checkout (Windows path `C:\Users\Admin\Desktop\drones_project`, exposed under WSL's `/mnt/c/...` mount).

**Step 2 — activate the Python environment:**

```bash
source ~/groundlink-venv/bin/activate
```

Confirmed the venv exists at `~/groundlink-venv` and has this project installed editable (`pip list` shows `groundlink 0.1.0 ... /mnt/c/Users/Admin/Desktop/drones_project`).

**Step 3 — start the simulator:**

```bash
bash sim/trials/launch_sitl_bg.sh
```

Confirmed present at `sim/trials/launch_sitl_bg.sh` (nothing moved). Takes ~20–30s, prints `SITL_UP`, then returns control to your prompt — no second window needed. If it ever prints `SITL_FAILED_TO_START` instead, something's wrong with the PX4 build at `~/PX4-Autopilot` (confirmed present and built on this machine at `~/PX4-Autopilot/build/px4_sitl_default/bin/px4`).

**Step 4 — run a scenario:**

```bash
python sim/trials/gps_scenario_trial.py
```

You'll see live progress printed to the screen, ending in a line starting with `RESULT` that summarizes what happened (detection time, what GroundLink decided to do, and whether it worked). Actual tested output looked like:

```
RESULT {"ok": true, "condition": "adaptive", ..., "event_outcome": "hold", ..., "safe_confirmed": true, ...}
```

There are three scenario scripts you can run this way — swap the last command for whichever one you want to watch:

| Script | What you'll watch happen |
|---|---|
| `sim/trials/battery_scenario_trial.py` | Drone flies out, its battery is drained to a critical level mid-flight, and it responds (returns home, or lands immediately if severe) |
| `sim/trials/gps_scenario_trial.py` | Drone flies out, GPS signal is cut mid-flight, and it holds its position rather than trying to navigate blind |
| `sim/trials/nfz_scenario_trial.py` | Drone flies a survey pattern, a restricted zone appears mid-survey, and it reroutes around it instead of abandoning the job |

**Step 5 — clean up before running another scenario (or when you're done):**

```bash
pkill -9 -f "bin/px4"; pkill -9 -f "gz sim"; pkill -9 -f "mavsdk_server"
```

This is the same cleanup `launch_sitl_bg.sh` itself runs before every fresh launch (see the script). **You must run this — or just re-run Step 3, which does it for you — before starting another scenario in the same session**, per D19: PX4 persists parameter changes to disk and will silently carry a fault (like a forced GPS or battery failure) into the next "fresh" boot otherwise. To go again: repeat from Step 3.

### How long each one takes

These are honest numbers, measured from dozens of real runs during development (see `decisions.md` for the full data) — not estimates.

| Scenario | Roughly how long, start to finish |
|---|---|
| Simulator boot (before any scenario) | ~20–30 seconds |
| Battery critical — moderate drain | ~2–2.5 minutes (failure detected ~53s after takeoff, then ~75s more to a fully confirmed safe landing) |
| Battery critical — severe drain | ~1.5 minutes (detected similarly fast, but only ~20s more to land immediately, versus baseline's ~51s if it insists on a full return trip anyway) |
| GPS degraded | ~1–1.5 minutes (the failure itself is detected in under 1 second — most of the time is climb-out and confirming the drone actually stopped safely) |
| No-fly zone | ~2.5–3 minutes (zone appears ~58s into the survey, reroute completes ~52s after that) |

---

## 3. Run with custom parameters (no code editing required)

Each of the three scenario scripts accepts command-line flags so you can choose different conditions without opening any code. The flag tables below are the exact, live-tested `--help` output for each script on this machine (not transcribed from memory). `battery_scenario_trial.py --severity severe --baseline --export <path>` and `gps_scenario_trial.py --baseline` were both run start-to-finish and produced real results (the battery run exported 4047 samples to the JSON file).

### Battery-critical scenario — `sim/trials/battery_scenario_trial.py`

| Flag | What it does |
|---|---|
| `--baseline` | Run the "dumb" baseline response (always return-to-launch) instead of GroundLink's adaptive response — use this to compare the two |
| `--severity moderate` or `--severity severe` | *(default: `moderate`)* `moderate` drains the battery to 12% — both adaptive and baseline choose the same response here, a fair head-to-head. `severe` drains it fast to 5% — GroundLink lands immediately instead of gambling on a full trip home, while baseline still attempts the full return regardless |
| `--export path/to/file.json` | Also save the full flight recording to a file — this is exactly how the recordings shown in the dashboard were produced |

Example commands:

```bash
# Default: adaptive response, moderate battery drain
python sim/trials/battery_scenario_trial.py

# Baseline response under a severe battery drain
python sim/trials/battery_scenario_trial.py --baseline --severity severe

# Adaptive response, severe drain, and save the flight recording
python sim/trials/battery_scenario_trial.py --severity severe --export my_flight.json
```

### GPS-degraded scenario — `sim/trials/gps_scenario_trial.py`

| Flag | What it does |
|---|---|
| `--baseline` | Run the baseline response (attempt return-to-launch, even though GPS — which RTL itself needs — is the thing that just failed) instead of GroundLink's adaptive response (hold position) |
| `--export path/to/file.json` | Save the full flight recording to a file |

*(This scenario has only one variant — GPS fix lost mid-flight — so there's no severity flag.)*

Example commands:

```bash
# Adaptive response (the default)
python sim/trials/gps_scenario_trial.py

# Baseline response, for comparison
python sim/trials/gps_scenario_trial.py --baseline
```

### No-fly-zone scenario — `sim/trials/nfz_scenario_trial.py`

| Flag | What it does |
|---|---|
| `--baseline` | Run the baseline response (abandon the survey and return home) instead of GroundLink's adaptive response (reroute around the zone and keep going) |
| `--export path/to/file.json` | Save the full flight recording to a file |

*(This scenario also has only one variant — a restricted zone appearing mid-survey — so there's no severity flag.)*

Example commands:

```bash
# Adaptive response (the default)
python sim/trials/nfz_scenario_trial.py

# Baseline response, for comparison
python sim/trials/nfz_scenario_trial.py --baseline
```

### Going further

The exact battery percentages, drain speeds, and GPS satellite thresholds behind `--severity` and the scenarios above are defined in `sim/failure_injection/scenarios.py` (e.g. the "severe" battery drain targets 5%, "moderate" targets 12%). Changing those specific numbers means editing that file directly — everything above this point is available without touching any code.

---

## A note on order

Read this file top to bottom: **section 1 needs nothing installed and shows the real result in seconds. Sections 2 and 3 are for anyone who wants to see the simulator run live instead of watching the recording.** Most people — including anyone just reviewing the outcome — only need section 1.
