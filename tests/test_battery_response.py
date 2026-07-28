import pytest

from replanning_engine.battery_response import BatteryAction, BatteryResponseThresholds, decide_battery_response


def test_rejects_inverted_thresholds():
    with pytest.raises(ValueError):
        BatteryResponseThresholds(rtl_below_percent=10.0, land_immediately_below_percent=20.0)


def test_continue_above_rtl_threshold():
    t = BatteryResponseThresholds(rtl_below_percent=20.0, land_immediately_below_percent=8.0)
    assert decide_battery_response(50.0, t) == BatteryAction.CONTINUE
    assert decide_battery_response(20.1, t) == BatteryAction.CONTINUE


def test_rtl_between_thresholds():
    t = BatteryResponseThresholds(rtl_below_percent=20.0, land_immediately_below_percent=8.0)
    assert decide_battery_response(20.0, t) == BatteryAction.RETURN_TO_LAUNCH
    assert decide_battery_response(15.0, t) == BatteryAction.RETURN_TO_LAUNCH
    assert decide_battery_response(8.1, t) == BatteryAction.RETURN_TO_LAUNCH


def test_land_immediately_below_lower_threshold():
    t = BatteryResponseThresholds(rtl_below_percent=20.0, land_immediately_below_percent=8.0)
    assert decide_battery_response(8.0, t) == BatteryAction.LAND_IMMEDIATELY
    assert decide_battery_response(0.0, t) == BatteryAction.LAND_IMMEDIATELY


def test_default_thresholds_are_usable():
    t = BatteryResponseThresholds()
    assert decide_battery_response(100.0, t) == BatteryAction.CONTINUE
