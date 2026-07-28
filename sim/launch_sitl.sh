#!/usr/bin/env bash
# Launch PX4 SITL (Gazebo Harmonic, headless) for GroundLink.
# Run from inside WSL2 Ubuntu. Assumes PX4-Autopilot is already cloned +
# built at ~/PX4-Autopilot (see README.md "PX4 SITL" section for the
# one-time clone/setup/build steps -- this script only launches).
#
# HEADLESS=1 avoids depending on WSLg's GPU rendering (we only need the
# MAVLink link, not visualization). Stdin is closed because PX4's
# interactive console spams stdout with cursor-redraw escape codes when
# there's no real TTY attached -- both gotchas were hit for real while
# setting this up, see decisions.md.
#
# The dataman file (PX4's on-disk mission/waypoint store) is deleted before
# every launch. It persists across process restarts by default, and a
# stale mission left over from a previous SITL session will silently
# corrupt the next one -- the vehicle arms and reports the mission
# "finished" instantly without ever taking off, because the navigator is
# executing leftover waypoint state, not what you just uploaded. This was
# a real, confirmed root cause (not a guess) -- see decisions.md D10. Do
# not remove this line; without it the bug comes back on the next restart.
#
# Any previous PX4/Gazebo processes are also killed first -- leaving the
# Gazebo server running across "restarts" (only killing the px4 binary)
# was a second real bug found during the same investigation.

set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

if [[ ! -d "$PX4_DIR" ]]; then
    echo "PX4-Autopilot not found at $PX4_DIR -- clone it first (see README.md)." >&2
    exit 1
fi

pkill -9 -f 'bin/px4' 2>/dev/null || true
pkill -9 -f 'gz sim' 2>/dev/null || true
sleep 1

rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/dataman"

cd "$PX4_DIR"
exec env HEADLESS=1 make px4_sitl gz_x500 < /dev/null
