"""Experimental short relative-goal navigation with simple depth avoidance."""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .planner import (
    choose_heading,
    choose_search_step,
    command_for_heading,
    decode_depth_message,
    depth_to_base_points,
    filtered_yaw,
    obstacle_points_in_view,
    relative_goal_to_odom,
)


def yaw_from_quaternion(orientation):
    """Return planar yaw from a ROS quaternion."""
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RoverAvoidance(Node):
    def __init__(self):
        super().__init__("rover_avoidance")
        defaults = {
            "goal_topic": "/avoidance/goal_relative",
            "odom_topic": "/odom",
            "depth_topic": "/camera/depth/image_raw/compressedDepth",
            "camera_info_topic": "/oak/rgb/camera_info",
            "cmd_vel_topic": "/cmd_vel",
            "status_topic": "/avoidance/status",
            "base_frame": "base_link",
            "tick_rate_hz": 10.0,
            "max_linear_speed_mps": 0.35,
            "max_angular_speed_rps": 0.65,
            "min_align_rate_rps": 0.5,
            "wheel_separation_m": 0.172,
            "max_wheel_speed_mps": 0.5,
            "min_wheel_speed_mps": 0.05,
            "heading_gain": 1.2,
            "align_tolerance_rad": 0.12,
            "align_release_ratio": 0.5,
            "yaw_filter_time_constant_s": 0.3,
            "goal_tolerance_m": 0.15,
            "obstacle_clearance_m": 0.18,
            "obstacle_min_height_m": 0.04,
            "obstacle_max_height_m": 0.75,
            "obstacle_lookahead_m": 1.2,
            "depth_min_m": 0.2,
            "depth_max_m": 3.0,
            "depth_decimation": 8,
            "min_valid_depth_fraction": 0.08,
            "missing_depth_blocks_paths": False,
            "sensor_timeout_s": 0.6,
            "debug_enabled": False,
            "preview_only": False,
            "preview_search_max_step_m": 1.0,
            "preview_search_stop_margin_m": 0.3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._p = {name: self.get_parameter(name).value for name in defaults}
        self._preview_only = bool(self._p["preview_only"])
        if self._preview_only:
            self._p["debug_enabled"] = True
        if float(self._p["tick_rate_hz"]) <= 0.0:
            raise ValueError("tick_rate_hz must be positive")
        if int(self._p["depth_decimation"]) < 1:
            raise ValueError("depth_decimation must be positive")

        self._odom = None
        self._odom_received_at = None
        self._depth = None
        self._depth_received_at = None
        self._camera_info = None
        self._goal_odom = None
        self._aligning = False
        self._yaw = None
        self._state = "IDLE"
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._odom_sub = self.create_subscription(
            Odometry, str(self._p["odom_topic"]), self._on_odom, 10
        )
        self._depth_sub = self.create_subscription(
            CompressedImage,
            str(self._p["depth_topic"]),
            self._on_depth,
            qos_profile_sensor_data,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            str(self._p["camera_info_topic"]),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self._goal_sub = self.create_subscription(
            PointStamped,
            "/avoidance/preview_goal_relative" if self._preview_only else str(self._p["goal_topic"]),
            self._on_goal, 10,
        )
        self._cmd_pub = None if self._preview_only else self.create_publisher(
            Twist, str(self._p["cmd_vel_topic"]), 10
        )
        self._status_pub = self.create_publisher(
            String, "/avoidance/preview_status" if self._preview_only else str(self._p["status_topic"]), 10
        )
        if self._p["debug_enabled"]:
            self._debug_points_pub = self.create_publisher(PointCloud2, "/avoidance/debug/points", 10)
            self._debug_considered_pub = self.create_publisher(PointCloud2, "/avoidance/debug/considered", 10)
            self._debug_markers_pub = self.create_publisher(MarkerArray, "/avoidance/debug/paths", 10)
            self._debug_summary_pub = self.create_publisher(String, "/avoidance/debug/summary", 10)
        self._timer = self.create_timer(
            1.0 / float(self._p["tick_rate_hz"]), self._tick
        )
        if self._preview_only:
            self.get_logger().info("Preview only: no /cmd_vel publisher; send goals to /avoidance/preview_goal_relative")
        else:
            self.get_logger().warning(
                "Experimental autonomy publishes /cmd_vel while a goal is active; "
                "do not run teleop at the same time."
            )
        self._set_state("IDLE", "waiting for a relative goal")

    def _on_odom(self, message):
        self._odom = message
        now = time.monotonic()
        dt = 0.0 if self._odom_received_at is None else now - self._odom_received_at
        self._odom_received_at = now
        raw_yaw = yaw_from_quaternion(message.pose.pose.orientation)
        try:
            self._yaw = filtered_yaw(
                self._yaw, raw_yaw, dt, float(self._p["yaw_filter_time_constant_s"])
            )
        except ValueError:
            self._yaw = raw_yaw

    def _on_depth(self, message):
        self._depth = message
        self._depth_received_at = time.monotonic()

    def _on_camera_info(self, message):
        self._camera_info = message

    def _on_goal(self, message):
        if message.header.frame_id and message.header.frame_id != str(self._p["base_frame"]):
            self._stop("FAULT", "goal frame must be base_link")
            return
        if not self._preview_only and (self._odom is None or not self._sensor_fresh(self._odom_received_at)):
            self._stop("FAULT", "fresh odometry required to accept goal")
            return
        x, y = float(message.point.x), float(message.point.y)
        if not math.isfinite(x) or not math.isfinite(y) or x <= 0.05:
            self._stop("FAULT", "goal must be finite and more than 0.05 m forward")
            return
        if self._preview_only:
            self._goal_odom = (x, y)
        else:
            pose = self._odom.pose.pose
            yaw = self._yaw if self._yaw is not None else yaw_from_quaternion(pose.orientation)
            self._goal_odom = relative_goal_to_odom(
                x, y, pose.position.x, pose.position.y, yaw
            )
        self._aligning = False
        frame = "base_link" if self._preview_only else "odom"
        self._set_state("PREVIEW" if self._preview_only else "DRIVE", f"goal accepted at ({self._goal_odom[0]:.2f}, {self._goal_odom[1]:.2f}) {frame}")

    def _sensor_fresh(self, received_at):
        return received_at is not None and time.monotonic() - received_at <= float(
            self._p["sensor_timeout_s"]
        )

    def _obstacle_points(self):
        depth_message = self._depth
        camera_info = self._camera_info
        if depth_message is None or camera_info is None:
            raise ValueError("waiting for depth and camera calibration")
        if not self._sensor_fresh(self._depth_received_at):
            raise ValueError("depth data is stale")
        if depth_message.header.frame_id != camera_info.header.frame_id:
            raise ValueError("depth and camera calibration frames do not match")
        depth = decode_depth_message(depth_message)
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self._p["base_frame"]),
                depth_message.header.frame_id,
                rclpy.time.Time(),
            ).transform
            translation = (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            )
            rotation = (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
        except TransformException as exc:
            raise ValueError(f"camera-to-base transform unavailable: {exc}") from exc
        points, unknown_points, valid_fraction = depth_to_base_points(
            depth,
            camera_info,
            translation,
            rotation,
            int(self._p["depth_decimation"]),
            float(self._p["depth_min_m"]),
            float(self._p["depth_max_m"]),
            float(self._p["obstacle_lookahead_m"]),
            bool(self._p["missing_depth_blocks_paths"]),
        )
        if valid_fraction < float(self._p["min_valid_depth_fraction"]):
            raise ValueError(f"too little valid depth ({valid_fraction:.1%})")
        return points, unknown_points, valid_fraction

    def _tick(self):
        if self._goal_odom is None:
            if self._state.startswith("BLOCKED:"):
                return
            if self._p["debug_enabled"]:
                try:
                    points, unknown_points, valid_fraction = self._obstacle_points()
                    considered = obstacle_points_in_view(
                        points,
                        float(self._p["obstacle_clearance_m"]),
                        float(self._p["obstacle_min_height_m"]),
                        float(self._p["obstacle_max_height_m"]),
                        float(self._p["obstacle_lookahead_m"]),
                    )
                    self._publish_debug(
                        points, considered, [], None, valid_fraction, unknown_points,
                        self._state.split(":", 1)[0],
                    )
                except (ValueError, TypeError, cv2.error):
                    pass
            return
        if not self._preview_only and (self._odom is None or not self._sensor_fresh(self._odom_received_at)):
            self._preview_fault_or_stop("odometry missing or stale")
            return
        try:
            points, unknown_points, valid_fraction = self._obstacle_points()
        except (ValueError, TypeError, cv2.error) as exc:
            self._preview_fault_or_stop(str(exc))
            return

        if self._preview_only:
            dx, dy = self._goal_odom
            yaw = 0.0
        else:
            pose = self._odom.pose.pose
            yaw = self._yaw if self._yaw is not None else yaw_from_quaternion(pose.orientation)
            dx = self._goal_odom[0] - pose.position.x
            dy = self._goal_odom[1] - pose.position.y
        distance = math.hypot(dx, dy)
        if distance <= float(self._p["goal_tolerance_m"]):
            if self._preview_only:
                self._set_state("PREVIEW", f"goal within tolerance ({distance:.2f} m remaining)")
            else:
                self._stop("GOAL_REACHED", f"goal reached ({distance:.2f} m remaining)")
            return

        goal_heading = normalize_angle(math.atan2(dy, dx) - yaw)
        steering = {
            "max_linear_speed": float(self._p["max_linear_speed_mps"]),
            "max_angular_speed": float(self._p["max_angular_speed_rps"]),
            "heading_gain": float(self._p["heading_gain"]),
            "align_tolerance": float(self._p["align_tolerance_rad"]),
            "release_ratio": float(self._p["align_release_ratio"]),
            "was_aligning": self._aligning,
            "min_angular_speed": float(self._p["min_align_rate_rps"]),
            "wheel_separation_m": float(self._p["wheel_separation_m"]),
            "max_wheel_speed_mps": float(self._p["max_wheel_speed_mps"]),
            "min_wheel_speed_mps": float(self._p["min_wheel_speed_mps"]),
        }
        candidates = [] if self._p["debug_enabled"] or self._preview_only else None
        considered = [] if self._p["debug_enabled"] or self._preview_only else None
        heading = choose_heading(
            goal_heading,
            distance,
            points,
            rover_half_width=float(self._p["obstacle_clearance_m"]),
            obstacle_min_height=float(self._p["obstacle_min_height_m"]),
            obstacle_max_height=float(self._p["obstacle_max_height_m"]),
            lookahead_m=float(self._p["obstacle_lookahead_m"]),
            unknown_points=unknown_points,
            steering=steering,
            candidates_debug=candidates,
            considered_points=considered,
        )
        search_step = None
        if self._preview_only and heading is None and not len(unknown_points):
            search_step = choose_search_step(
                candidates, considered, float(self._p["obstacle_clearance_m"]),
                float(self._p["preview_search_max_step_m"]),
                float(self._p["preview_search_stop_margin_m"]),
            )
        if self._p["debug_enabled"]:
            state = (
                ("PREVIEW_CLEAR" if heading is not None else "PREVIEW_SEARCH" if search_step else "PREVIEW_BLOCKED")
                if self._preview_only else "BLOCKED" if heading is None else "DRIVE"
            )
            self._publish_debug(points, considered, candidates, heading, valid_fraction, unknown_points, state, search_step)
        if heading is None:
            if self._preview_only:
                if search_step is None:
                    self._set_state("PREVIEW_BLOCKED", "no full path or short safe search segment")
                else:
                    self._set_state("PREVIEW_SEARCH", f"possible {search_step['distance']:.2f} m search segment; no motion commanded")
                return
            nearest = min(
                (
                    point for point in points
                    if point[0] > 0.0
                    and float(self._p["obstacle_min_height_m"]) <= point[2] <= float(self._p["obstacle_max_height_m"])
                ),
                key=lambda point: math.hypot(point[0], point[1]),
                default=None,
            )
            detail = (
                f"nearest measured x={nearest[0]:.2f} y={nearest[1]:.2f} m"
                if nearest is not None else "no measured obstacle"
            )
            self._stop("BLOCKED", f"no clear local path toward goal ({detail}, unknown={len(unknown_points)})")
            return

        linear, angular = command_for_heading(heading, **steering)
        self._aligning = angular != 0.0
        if self._preview_only:
            self._set_state("PREVIEW_CLEAR", f"candidate {heading:.2f} rad, hypothetical vx={linear:.2f} wz={angular:.2f}")
            return
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._cmd_pub.publish(command)
        self._set_state("DRIVE", "curving toward clear path" if angular != 0.0 else "moving toward goal")

    def _stop(self, state, reason):
        # Publish zero once and drop the goal; bridge watchdog handles node loss.
        if self._cmd_pub is not None:
            self._cmd_pub.publish(Twist())
        self._goal_odom = None
        self._aligning = False
        if self._p["debug_enabled"] and state != "BLOCKED":
            self._debug_markers_pub.publish(self._debug_clear_markers())
        self._set_state(state, reason)

    def _preview_fault_or_stop(self, reason):
        if self._preview_only:
            self._debug_markers_pub.publish(self._debug_clear_markers())
            self._set_state("PREVIEW_FAULT", reason)
        else:
            self._stop("FAULT", reason)

    def _debug_clear_markers(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        return MarkerArray(markers=[marker])

    def _publish_debug(self, points, considered, candidates, selected, valid_fraction, unknown_points, state, search_step=None):
        stamp = self.get_clock().now().to_msg()
        frame = str(self._p["base_frame"])
        for publisher, values in ((self._debug_points_pub, points), (self._debug_considered_pub, considered)):
            cloud = PointCloud2()
            cloud.header.stamp = stamp
            cloud.header.frame_id = frame
            cloud.height = 1
            cloud.width = len(values)
            cloud.fields = [
                PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                for index, name in enumerate(("x", "y", "z"))
            ]
            cloud.point_step = 12
            cloud.row_step = 12 * cloud.width
            cloud.data = np.asarray(values, dtype="<f4").reshape(-1, 3).tobytes()
            cloud.is_dense = True
            publisher.publish(cloud)

        markers = self._debug_clear_markers()
        for index, candidate in enumerate(candidates):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame
            marker.ns = "candidates"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.025
            marker.color.a = 1.0
            marker.color.r = 1.0 if not candidate["clear"] else 0.0
            marker.color.g = 1.0 if candidate["clear"] else 0.15
            marker.color.b = 1.0 if candidate["clear"] and candidate["heading"] == selected else 0.0
            marker.points = [Point(x=x, y=y, z=0.06) for x, y in candidate["path"]]
            markers.markers.append(marker)
            if candidate["blocked_point"] is not None:
                blocker = Marker()
                blocker.header = marker.header
                blocker.ns = "first_blockers"
                blocker.id = index
                blocker.type = Marker.SPHERE
                blocker.action = Marker.ADD
                blocker.pose.orientation.w = 1.0
                blocker.pose.position.x, blocker.pose.position.y, blocker.pose.position.z = candidate["blocked_point"]
                blocker.scale.x = blocker.scale.y = blocker.scale.z = 0.05
                blocker.color.r = blocker.color.a = 1.0
                markers.markers.append(blocker)
        if search_step is not None:
            prefix = Marker()
            prefix.header.stamp = stamp
            prefix.header.frame_id = frame
            prefix.ns = "search_prefix"
            prefix.type = Marker.LINE_STRIP
            prefix.action = Marker.ADD
            prefix.scale.x = 0.05
            prefix.color.g = prefix.color.a = 1.0
            prefix.points = [Point(x=x, y=y, z=0.08) for x, y in search_step["path"]]
            markers.markers.append(prefix)
            endpoint = Marker()
            endpoint.header = prefix.header
            endpoint.ns = "search_endpoint"
            endpoint.type = Marker.SPHERE
            endpoint.action = Marker.ADD
            endpoint.pose.orientation.w = 1.0
            endpoint.pose.position.x, endpoint.pose.position.y = search_step["point"]
            endpoint.pose.position.z = 0.08
            endpoint.scale.x = endpoint.scale.y = endpoint.scale.z = 0.11
            endpoint.color.g = endpoint.color.a = 1.0
            markers.markers.append(endpoint)
        label = Marker()
        label.header.stamp = stamp
        label.header.frame_id = frame
        label.ns = "summary"
        label.id = 0
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.orientation.w = 1.0
        label.pose.position.z = 0.85
        label.scale.z = 0.13
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = (
            f"{state} | depth {len(points)} | obstacles {len(considered)} | "
            f"valid {valid_fraction:.0%} | clearance {float(self._p['obstacle_clearance_m']) + 0.025:.3f} m"
        )
        markers.markers.append(label)
        self._debug_markers_pub.publish(markers)

        summary = String()
        summary.data = json.dumps({
            "frame": frame,
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
            "state": state,
            "valid_fraction": round(valid_fraction, 3),
            "sampled_depth_points": len(points),
            "considered_obstacle_points": len(considered),
            "unknown_points": len(unknown_points),
            "selected_heading_rad": selected,
            "search_step": None if search_step is None else {
                "heading": search_step["heading"],
                "distance": search_step["distance"],
                "point": search_step["point"],
                "stop_margin_m": float(self._p["preview_search_stop_margin_m"]),
            },
            "candidates": [
                {key: candidate[key] for key in ("heading", "linear", "angular", "clear", "blocked_point")}
                for candidate in candidates
            ],
        })
        self._debug_summary_pub.publish(summary)

    def _set_state(self, state, reason):
        text = f"{state}: {reason}"
        if text == self._state:
            return
        self._state = text
        status = String()
        status.data = text
        self._status_pub.publish(status)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = RoverAvoidance()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if node._cmd_pub is not None and rclpy.ok():
            node._cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()