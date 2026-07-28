# Failure injection configs

Not implemented yet — this is consumed by `replanning_engine`, which is out
of scope for the Step 1-4 skeleton (see `decisions.md` and the project
README's "Step 4 report" section for why).

Planned scope once `replanning_engine` is reviewed and built: SITL-side
failure scenarios (simulated battery drain rate override, GPS noise/dropout
injection via PX4's `param set` SIM_GPS_* params, and geofence/no-fly-zone
polygons introduced mid-mission) that `constraint_monitor` should detect and
`replanning_engine` should react to. Each scenario will get its own config
file here (YAML/JSON, TBD) so the evaluation in the project deliverables
(baseline RTL vs. adaptive replanning, under identical injected failures)
is reproducible.
