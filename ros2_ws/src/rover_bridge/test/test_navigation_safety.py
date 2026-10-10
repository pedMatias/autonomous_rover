import math

import pytest

from rover_bridge.navigation_safety import NavigationSafety


def ready():
    gate = NavigationSafety(enabled=True)
    gate.command = (0.45, 0.3)
    gate.command_at = 10.0
    gate.scan_at = 10.0
    gate.scan_stamp = 1000.0
    gate.scan_valid = True
    gate.odom_at = 10.0
    gate.odom_stamp = 1000.0
    return gate


def test_default_disabled_and_forward_arc_allowed():
    assert NavigationSafety().result(10, 1000, True) == ((0.0, 0.0), "motion_disabled")
    assert ready().result(10.1, 1000.1, True) == ((0.45, 0.3), "ready")


@pytest.mark.parametrize("command", [
    (0.1, 0), (0, 1), (-0.45, 0), (0.45, 2),
    (0.51, 0), (0.35, 0.1), (math.nan, 0), (0.45, math.inf),
])
def test_unsafe_commands_are_stopped_never_clamped_up(command):
    gate = ready()
    gate.command = command
    assert gate.result(10.1, 1000.1, True)[0] == (0.0, 0.0)


def test_straight_minimum_speed_and_zero_allowed():
    gate = ready()
    gate.command = (0.35, 0.0)
    assert gate.result(10.1, 1000.1, True) == ((0.35, 0.0), "ready")
    gate.command = (0.0, 0.0)
    assert gate.result(10.1, 1000.1, True) == ((0.0, 0.0), "stopped")


@pytest.mark.parametrize("attribute,value,reason", [
    ("command_at", 9.7, "command_timeout"),
    ("scan_at", 9.0, "scan_unavailable"),
    ("scan_stamp", 999.0, "scan_unavailable"),
    ("scan_stamp", 1001.0, "scan_unavailable"),
    ("scan_valid", False, "scan_unavailable"),
    ("odom_at", 9.0, "odometry_unavailable"),
    ("odom_stamp", 999.0, "odometry_unavailable"),
    ("odom_stamp", math.nan, "odometry_unavailable"),
])
def test_freshness_and_sensor_failures(attribute, value, reason):
    gate = ready()
    setattr(gate, attribute, value)
    assert gate.result(10.1, 1000.1, True) == ((0.0, 0.0), reason)


def test_localization_or_tf_failure_stops():
    assert ready().result(10.1, 1000.1, False) == (
        (0.0, 0.0), "transform_unavailable"
    )


def test_stop_after_scan_loss_even_with_new_commands():
    gate = ready()
    gate.command_at = 10.6
    assert gate.result(10.6, 1000.6, True)[1] == "scan_unavailable"


def test_invalid_limits_rejected():
    with pytest.raises(ValueError):
        NavigationSafety(min_wheel_speed=0.6)
    with pytest.raises(ValueError):
        NavigationSafety(scan_timeout=math.nan)
