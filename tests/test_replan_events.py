from mission_planner.waypoint import Waypoint
from replanning_engine.events import ReplanEvent, ReplanTrigger


def test_replan_event_to_dict_is_json_friendly():
    old_wps = [Waypoint(latitude_deg=1.0, longitude_deg=2.0, relative_altitude_m=15.0)]
    new_wps = [
        Waypoint(latitude_deg=1.1, longitude_deg=2.1, relative_altitude_m=15.0),
        Waypoint(latitude_deg=1.2, longitude_deg=2.2, relative_altitude_m=15.0),
    ]
    event = ReplanEvent(
        timestamp_unix_s=1234.5,
        trigger=ReplanTrigger.NO_FLY_ZONE,
        reason="zone 'restricted_area_1' blocks leg 0",
        outcome="rerouted",
        old_remaining_waypoints=old_wps,
        new_remaining_waypoints=new_wps,
    )
    d = event.to_dict()
    assert d["trigger"] == "no_fly_zone"
    assert d["outcome"] == "rerouted"
    assert d["old_remaining_waypoint_count"] == 1
    assert d["new_remaining_waypoint_count"] == 2
    assert isinstance(d["reason"], str)


def test_replan_event_defaults_to_empty_waypoint_lists():
    event = ReplanEvent(
        timestamp_unix_s=0.0,
        trigger=ReplanTrigger.BATTERY_CRITICAL,
        reason="battery at 5%",
        outcome="land_immediately",
    )
    assert event.old_remaining_waypoints == []
    assert event.new_remaining_waypoints == []
