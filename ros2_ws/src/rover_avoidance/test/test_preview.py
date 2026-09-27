import json
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped

from rover_avoidance.node import RoverAvoidance


def test_preview_without_odometry_never_publishes_commands_and_rechecks_goal():
    rclpy.init(args=["--ros-args", "-p", "preview_only:=true"])
    node = RoverAvoidance()
    try:
        assert node._cmd_pub is None
        assert all(publisher.topic_name != "/cmd_vel" for publisher in node.publishers)
        assert node._goal_sub.topic_name == "/avoidance/preview_goal_relative"

        goal = PointStamped()
        goal.header.frame_id = "base_link"
        goal.point.x = 2.0
        node._on_goal(goal)
        assert node._goal_odom is not None

        wall = np.array(
            [[x, y, 0.2] for x in (0.25, 0.5, 0.8) for y in np.linspace(-1.0, 1.0, 21)],
            dtype=np.float32,
        )
        node._obstacle_points = lambda: (wall, np.empty((0, 3), dtype=np.float32), 0.8)
        node._tick()
        assert node._state.startswith("PREVIEW_BLOCKED:")
        assert node._goal_odom is not None

        node._obstacle_points = lambda: (
            np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.float32), 0.8,
        )
        node._tick()
        assert node._state.startswith("PREVIEW_CLEAR:")
        assert node._goal_odom is not None

        node._tick()
        assert node._state.startswith("PREVIEW_CLEAR:")
        assert node._goal_odom is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_idle_preview_shows_points_in_the_planner_obstacle_band():
    rclpy.init(args=["--ros-args", "-p", "preview_only:=true"])
    node = RoverAvoidance()
    try:
        points = np.array([[0.5, 0.0, 0.2], [0.5, 0.0, 0.0], [2.0, 0.0, 0.2]], dtype=np.float32)
        node._obstacle_points = lambda: (points, np.empty((0, 3), dtype=np.float32), 0.8)
        frames = []
        node._publish_debug = lambda raw, considered, *rest: frames.append((considered, rest[-1]))
        node._tick()
        np.testing.assert_allclose(frames[0][0], [[0.5, 0.0, 0.2]])
        assert frames[0][1] == "IDLE"
        node._state = "PREVIEW_FAULT: invalid depth"
        node._tick()
        assert frames[1][1] == "PREVIEW_FAULT"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_preview_goal_subscription_accepts_goal_without_odometry():
    rclpy.init(args=["--ros-args", "-p", "preview_only:=true"])
    node = RoverAvoidance()
    sender = rclpy.create_node("preview_goal_test_sender")
    try:
        publisher = sender.create_publisher(PointStamped, "/avoidance/preview_goal_relative", 10)
        goal = PointStamped()
        goal.header.frame_id = "base_link"
        goal.point.x = 2.0
        for _ in range(20):
            publisher.publish(goal)
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._goal_odom is not None:
                break
        assert node._goal_odom == (2.0, 0.0)
        assert node._state.startswith("PREVIEW:")
        assert all(pub.topic_name != "/cmd_vel" for pub in node.publishers)
    finally:
        sender.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_preview_search_marks_partial_path_and_stops_when_too_close():
    rclpy.init(args=["--ros-args", "-p", "preview_only:=true"])
    node = RoverAvoidance()
    try:
        markers = []
        summaries = []
        node._debug_markers_pub = SimpleNamespace(publish=markers.append)
        node._debug_summary_pub = SimpleNamespace(publish=summaries.append)
        goal = PointStamped()
        goal.header.frame_id = "base_link"
        goal.point.x = 2.0
        node._on_goal(goal)

        wall = np.array([[0.8, y, 0.2] for y in np.linspace(-1.5, 1.5, 61)], dtype=np.float32)
        node._obstacle_points = lambda: (wall, np.empty((0, 3), dtype=np.float32), 0.8)
        node._tick()
        assert node._state.startswith("PREVIEW_SEARCH:")
        assert node._cmd_pub is None
        assert {marker.ns for marker in markers[-1].markers} >= {"search_prefix", "search_endpoint"}
        step = json.loads(summaries[-1].data)["search_step"]
        assert 0.3 < step["distance"] <= 1.0
        assert step["stop_margin_m"] == 0.3

        wall[:, 0] = 0.4
        node._tick()
        assert node._state.startswith("PREVIEW_BLOCKED:")
        assert json.loads(summaries[-1].data)["search_step"] is None
        assert not {"search_prefix", "search_endpoint"} & {marker.ns for marker in markers[-1].markers}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_preview_search_without_debug_publishing():
    rclpy.init(args=["--ros-args", "-p", "preview_only:=true", "-p", "debug_enabled:=false"])
    node = RoverAvoidance()
    try:
        goal = PointStamped()
        goal.header.frame_id = "base_link"
        goal.point.x = 2.0
        node._on_goal(goal)
        wall = np.array([[0.8, y, 0.2] for y in np.linspace(-1.5, 1.5, 61)], dtype=np.float32)
        node._obstacle_points = lambda: (wall, np.empty((0, 3), dtype=np.float32), 0.8)
        node._tick()
        assert node._state.startswith("PREVIEW_SEARCH:")
        assert node._cmd_pub is None
    finally:
        node.destroy_node()
        rclpy.shutdown()