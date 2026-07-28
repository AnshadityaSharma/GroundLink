"""Shared lat/lon <-> local-ENU-meters projection.

Equirectangular approximation, accurate enough for mission-local extents
(survey grids, no-fly-zone reroutes) up to a few km from the origin. Not
suitable for long-range navigation -- everything in this codebase operates
at single-mission scale, so that's an accepted, deliberate limitation.

Promoted out of mission_planner.grid_coverage so replanning_engine can share
the exact same projection code rather than reimplementing it (see
replanning_engine/DESIGN.md).
"""

from __future__ import annotations

import math

_EARTH_RADIUS_M = 6371000.0


def local_xy_to_latlon(x_m: float, y_m: float, origin_lat_deg: float, origin_lon_deg: float) -> tuple[float, float]:
    d_lat = (y_m / _EARTH_RADIUS_M) * (180.0 / math.pi)
    d_lon = (x_m / (_EARTH_RADIUS_M * math.cos(math.radians(origin_lat_deg)))) * (180.0 / math.pi)
    return origin_lat_deg + d_lat, origin_lon_deg + d_lon


def latlon_to_local_xy(lat_deg: float, lon_deg: float, origin_lat_deg: float, origin_lon_deg: float) -> tuple[float, float]:
    d_lat = lat_deg - origin_lat_deg
    d_lon = lon_deg - origin_lon_deg
    y_m = d_lat * (math.pi / 180.0) * _EARTH_RADIUS_M
    x_m = d_lon * (math.pi / 180.0) * _EARTH_RADIUS_M * math.cos(math.radians(origin_lat_deg))
    return x_m, y_m
