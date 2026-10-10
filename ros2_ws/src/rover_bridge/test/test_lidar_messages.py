import math
import time

import pytest

pytest.importorskip("rclpy", reason="Run this ROS message test in ROS2 Jazzy")

from rover_bridge.lidar import build_scan
from rover_lidar.protocol import Scan


def test_laser_scan_metadata_and_invalid_ranges():
    scan = Scan(0, 1710000000.25, 0.1, 0.05, 12.0,
                (0, 1000) + (2000,) * 358, (100,) * 360)
    message = build_scan(scan, "laser_frame")
    assert message.header.stamp.sec == 1710000000
    assert message.header.stamp.nanosec == 250000000
    assert message.header.frame_id == "laser_frame"
    assert message.angle_min == pytest.approx(-math.pi)
    assert message.angle_max == pytest.approx(math.pi - 2 * math.pi / 360)
    assert message.angle_increment == pytest.approx(2 * math.pi / 360)
    assert message.time_increment == 0
    assert message.scan_time == pytest.approx(0.1)
    assert math.isnan(message.ranges[0])
    assert message.ranges[1] == 1.0
    assert len(message.intensities) == 360


def test_ros_gate_defaults_disabled_and_rejects_invalid_scan():
    import rclpy
    from rover_bridge.navigation_gate import NavigationGate

    rclpy.init()
    node = NavigationGate()
    try:
        assert not node.safety.enabled
        scan = Scan(0, time.time() - 0.1, 0.1, 0.05, 12.0,
                    (0,) * 360, (0,) * 360)
        node._scan(build_scan(scan, "laser_frame"))
        assert not node.safety.scan_valid
        assert not node._tf_valid()
    finally:
        node.destroy_node()
        rclpy.shutdown()
