#!/usr/bin/env python3
"""Consolidate the three raw scenario exports (dashboard/data/*.json,
produced by sim/trials/*_scenario_trial.py --export) plus
evaluation/RESULTS.md's comparison numbers into one dashboard/data/flight_data.json
the presentation dashboard loads, then renders dashboard/index.html from
dashboard/index.html.template by substituting that JSON in.

index.html is a build artifact -- edit index.html.template and re-run this
script. Doing both steps here is deliberate: the dashboard has to stay one
self-contained offline file, so the data is inlined rather than fetched,
and if the substitution lived outside this script the template and the
built file could silently drift apart.

Downsamples each scenario's position series to a manageable size for a
smooth browser animation (real telemetry ticks faster than the vehicle
visibly moves) -- keeps the first/last sample and one sample per ~0.6s
bucket, plus always keeps the samples immediately bracketing the replan
trigger so the marker lands precisely. Rounds coordinates to 6 decimal
places (~11cm) and altitude/speed to 1 decimal. Nothing is fabricated or
smoothed -- this is real recorded telemetry from the exported runs,
thinned for file size only.
"""
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA_DIR = HERE / 'data'
RESULTS_MD = HERE.parent / 'evaluation' / 'RESULTS.md'
OUT = DATA_DIR / 'flight_data.json'
TEMPLATE = HERE / 'index.html.template'
INDEX_HTML = HERE / 'index.html'
PLACEHOLDER = '__FLIGHT_DATA__'

BUCKET_S = 0.6

def downsample(series, trigger_t):
    if not series:
        return series
    out = []
    last_bucket = None
    for i, pt in enumerate(series):
        bucket = int(pt['t'] / BUCKET_S)
        near_trigger = trigger_t is not None and abs(pt['t'] - trigger_t) < 0.3
        if bucket != last_bucket or i == 0 or i == len(series) - 1 or near_trigger:
            out.append({
                't': round(pt['t'], 2),
                'lat': round(pt['lat'], 6),
                'lon': round(pt['lon'], 6),
                'alt': round(pt.get('alt', 0.0), 1),
                'mode': pt.get('mode'),
            })
            last_bucket = bucket
    return out


def parse_results_md(path):
    """Pull the per-scenario comparison tables out of RESULTS.md into a
    plain dict -- this is the same generated file evaluation/aggregate_results.py
    produces, not re-derived or guessed here."""
    text = path.read_text()
    groups = {}
    current_scenario = None
    for line in text.splitlines():
        h = re.match(r'^## (.+)$', line)
        if h:
            current_scenario = h.group(1).strip()
            groups[current_scenario] = {}
            continue
        row = re.match(r'^\| (adaptive|baseline) \| (\d+) \| ([^|]+) \| ([^|]+) \| ([^|]+) \| ([^|]+) \|$', line)
        if row and current_scenario:
            cond, n, completion, recovery, safe_rate, zone_rate = [g.strip() for g in row.groups()]
            groups[current_scenario][cond] = {
                'n': int(n), 'completion': completion, 'recovery_time': recovery,
                'safe_rate': safe_rate, 'zone_never_entered_rate': zone_rate,
            }
    return groups


def render_index(flight_data_json):
    """Inline the flight data into the template to produce index.html.

    newline='' on both ends so the template's own line endings survive
    verbatim instead of being rewritten to CRLF on Windows -- otherwise
    every build on a Windows checkout shows up as a whole-file diff.
    """
    with TEMPLATE.open(encoding='utf-8', newline='') as fh:
        template = fh.read()
    count = template.count(PLACEHOLDER)
    if count != 1:
        raise SystemExit(
            f'expected exactly one {PLACEHOLDER} in {TEMPLATE.name}, found {count}'
        )
    html = template.replace(PLACEHOLDER, flight_data_json)
    with INDEX_HTML.open('w', encoding='utf-8', newline='') as fh:
        fh.write(html)
    return html


def main():
    scenarios = {}
    for name, filename in (
        ('no_fly_zone', 'no_fly_zone.json'),
        ('battery_critical', 'battery_critical.json'),
        ('gps_degraded', 'gps_degraded.json'),
    ):
        raw = json.loads((DATA_DIR / filename).read_text())
        trigger_t = raw.get('t_replan_trigger')
        raw['series'] = downsample(raw['series'], trigger_t)
        scenarios[name] = raw
        print(f'{name}: {len(raw["series"])} samples after downsampling')

    comparison = parse_results_md(RESULTS_MD)

    flight_data_json = json.dumps({'scenarios': scenarios, 'comparison': comparison})
    OUT.write_text(flight_data_json)
    print(f'wrote {OUT} ({OUT.stat().st_size} bytes)')

    render_index(flight_data_json)
    print(f'wrote {INDEX_HTML} ({INDEX_HTML.stat().st_size} bytes)')


if __name__ == '__main__':
    main()
