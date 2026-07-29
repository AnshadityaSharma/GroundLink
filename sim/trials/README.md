# SITL trial scripts

One-off-looking but repeatedly-needed scripts that measure `replanning_engine`
behaviour against a live PX4 SITL instance. They live here, in version control,
because previous sessions kept them outside the repo and they were lost —
each investigation then re-derived the same harness from scratch.

These are **measurement tools, not tests**. `pytest` never runs them; they need
a real SITL instance and take minutes each. The unit/orchestration suite under
`tests/` is the thing CI-style runs.

## The one rule that matters

**Launch SITL as its own independent process invocation. Never background it
inside the same shell script/session that later runs the MAVSDK client.**

Doing so causes a silent, total hang — the client produces no output and runs
until an external timeout. This cost a full investigation to pin down; see
[decisions.md D14](../../decisions.md). It is not a flaw in the engine code, and
no amount of `setsid`/`disown`/redirection on the backgrounded job avoids it.

So: run `launch_sitl_bg.sh` on its own, let it return, then run the trial script
as a *separate* invocation.

```bash
bash sim/trials/launch_sitl_bg.sh     # returns once SITL is up, then exits
```

```bash
python sim/trials/speed_trial.py      # separate invocation
```

Invoke the launcher as `bash <path>`, not `./<path>`. When the repo lives on a
Windows drive and is reached from WSL through `/mnt/c` (DrvFs), executing the
script directly fails with a confusing `C:/Program: No such file or directory`
even though the shebang and line endings are correct. Going through `bash`
sidesteps it.

`launch_sitl_bg.sh` also deletes PX4's `dataman` file before every launch
([D10](../../decisions.md)) and kills lingering `px4`/`gz sim`/`mavsdk_server`
processes. Its console output goes to `/dev/null` by default — PX4 emits
~250MB/min of cursor-redraw spam and it has never diagnosed anything; the `.ulg`
flight logs are the real diagnostic source. Pass a path as `$1` only when
debugging the boot sequence itself.

## The scripts

| Script | What it measures | Result |
|---|---|---|
| `speed_trial.py` | Does `set_current_speed()` actually change real ground speed? Before/after cruise measurement. | 5/5, ratio 0.501-0.504 vs commanded 0.5 ([D15](../../decisions.md)) |
| `rtl_trial.py` | Does RTL complete the full return-and-land through to autonomous disarm, not just the mode switch? | 5/5, home within 0.1m, disarm in ~61s ([D16](../../decisions.md)) |
| `gps_resume_trial.py` | Does `resume_after_gps_recovery()` work, and does it resume or restart the mission? | 5/5, resumes from the current item ([D17](../../decisions.md)) |

## Measurement discipline these encode

Learned the hard way, and the reason the scripts are more careful than they
first look:

- **Never trust a call's return value.** MAVSDK can report success while PX4
  silently rejects the internal mode switch ([D11](../../decisions.md)). Always
  poll real telemetry — `FlightMode`, position, `armed` — afterwards.
- **Read the vehicle's actual home position** from `telemetry.home()`; do not
  hardcode SITL defaults. The real home was ~65m off the assumed default, which
  silently shortened mission legs ([D15](../../decisions.md)).
- **Validate a measurement window after the fact, and reject it if invalid.**
  `speed_trial.py` throws out any window whose variance is too high or during
  which the vehicle reached a waypoint and turned, then retries. Horizontal
  speed is ~0 for the first ~17s of any flight because the vehicle is climbing
  straight up — sampling "cruise speed" before that is meaningless.
- **Run a batch, not one lucky flight.** A single success proved nothing in
  D11; the fresh-SITL-per-trial batch is what distinguishes a real fix from a
  coincidence.
