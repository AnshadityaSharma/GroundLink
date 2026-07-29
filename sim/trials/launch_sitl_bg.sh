#!/usr/bin/env bash
# Launch PX4 SITL fully detached, then EXIT immediately.
#
# Per decisions.md D14: SITL must be launched as an independent process
# invocation, NEVER backgrounded inside the same shell session that later
# runs the MAVSDK client -- doing so causes a silent hang. This script is
# therefore only ever invoked on its own, and returns as soon as SITL is up.
#
# Output goes to /dev/null by default: PX4's console emits ~250MB/min of
# cursor-redraw spam even with stdin closed, and it has never once been the
# thing that diagnosed a bug here -- the .ulg flight logs are the real
# diagnostic source (see D10/D11). Pass a path as $1 only when actively
# debugging the boot sequence itself.
set -uo pipefail
LOG="${1:-/dev/null}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

pkill -9 -f 'bin/px4' 2>/dev/null || true
pkill -9 -f 'gz sim' 2>/dev/null || true
pkill -9 -f 'mavsdk_server' 2>/dev/null || true
sleep 2
rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/dataman"   # D10

cd "$PX4_DIR"
setsid nohup env HEADLESS=1 make px4_sitl gz_x500 </dev/null >"$LOG" 2>&1 &
disown

# Just confirm the process comes up and stays up. Actual arm-readiness is
# the client's job (wait_ready_to_arm), so don't duplicate it here.
sleep 25
if pgrep -f 'bin/px4' >/dev/null; then echo SITL_UP; exit 0; fi
echo SITL_FAILED_TO_START; exit 1
