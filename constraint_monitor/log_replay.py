"""Load a JSONL telemetry log into TelemetrySnapshot objects for replay.

Log format: one JSON object per line, matching TelemetrySnapshot's shape.
Any of "battery" / "gps" / "position" / "attitude" may be null/omitted for
a given line if that channel didn't update at that tick.

    {"timestamp_unix_s": 1234.5, "battery": {"voltage_v": 15.8, "remaining_percent": 42.0},
     "gps": {"fix_type": 3, "num_satellites": 11, "hdop": 1.1},
     "position": {"latitude_deg": 47.3977, "longitude_deg": 8.5456,
                   "absolute_altitude_m": 488.0, "relative_altitude_m": 15.0},
     "attitude": null}
"""

from __future__ import annotations

import json
from collections.abc import Iterator
from pathlib import Path

from firmware_link.telemetry import (
    Attitude,
    BatteryState,
    GpsFixType,
    GpsState,
    Position,
    TelemetrySnapshot,
)


def _parse_battery(d: dict | None) -> BatteryState | None:
    return BatteryState(**d) if d else None


def _parse_gps(d: dict | None) -> GpsState | None:
    if not d:
        return None
    return GpsState(fix_type=GpsFixType(d["fix_type"]), num_satellites=d["num_satellites"], hdop=d["hdop"])


def _parse_position(d: dict | None) -> Position | None:
    return Position(**d) if d else None


def _parse_attitude(d: dict | None) -> Attitude | None:
    return Attitude(**d) if d else None


def load_snapshots_jsonl(path: str | Path) -> Iterator[TelemetrySnapshot]:
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            yield TelemetrySnapshot(
                timestamp_unix_s=row["timestamp_unix_s"],
                battery=_parse_battery(row.get("battery")),
                gps=_parse_gps(row.get("gps")),
                position=_parse_position(row.get("position")),
                attitude=_parse_attitude(row.get("attitude")),
            )
