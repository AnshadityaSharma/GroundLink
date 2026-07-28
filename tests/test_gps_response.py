from firmware_link.telemetry import GpsFixType
from replanning_engine.gps_response import GpsAction, GpsResponseThresholds, decide_gps_response


def test_continue_normal_with_good_fix_and_low_hdop():
    t = GpsResponseThresholds(min_fix_type_to_continue=GpsFixType.FIX_3D, max_hdop_for_normal_speed=2.5)
    assert decide_gps_response(GpsFixType.FIX_3D, 1.0, t) == GpsAction.CONTINUE_NORMAL


def test_slow_down_with_good_fix_but_high_hdop():
    t = GpsResponseThresholds(min_fix_type_to_continue=GpsFixType.FIX_3D, max_hdop_for_normal_speed=2.5)
    assert decide_gps_response(GpsFixType.FIX_3D, 3.5, t) == GpsAction.SLOW_DOWN


def test_hold_when_fix_degraded_below_minimum():
    t = GpsResponseThresholds(min_fix_type_to_continue=GpsFixType.FIX_3D, max_hdop_for_normal_speed=2.5)
    assert decide_gps_response(GpsFixType.FIX_2D, 1.0, t) == GpsAction.HOLD
    assert decide_gps_response(GpsFixType.NO_FIX, 1.0, t) == GpsAction.HOLD
    assert decide_gps_response(GpsFixType.NO_GPS, 99.0, t) == GpsAction.HOLD


def test_fix_degraded_takes_precedence_over_hdop():
    # a degraded fix should HOLD even if HDOP happens to read low (untrustworthy)
    t = GpsResponseThresholds(min_fix_type_to_continue=GpsFixType.FIX_3D, max_hdop_for_normal_speed=2.5)
    assert decide_gps_response(GpsFixType.FIX_2D, 0.5, t) == GpsAction.HOLD


def test_rtk_fix_types_treated_as_better_than_3d():
    t = GpsResponseThresholds(min_fix_type_to_continue=GpsFixType.FIX_3D, max_hdop_for_normal_speed=2.5)
    assert decide_gps_response(GpsFixType.RTK_FIXED, 0.5, t) == GpsAction.CONTINUE_NORMAL


def test_default_thresholds_are_usable():
    t = GpsResponseThresholds()
    assert decide_gps_response(GpsFixType.FIX_3D, 1.0, t) == GpsAction.CONTINUE_NORMAL
