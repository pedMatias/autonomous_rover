"""The only Nav2-to-motor command route; disabled unless explicitly armed."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .navigation_safety import NavigationSafety
from .node import default_calibration_file, load_calibration


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1_000_000_000


class NavigationGate(Node):
    def __init__(self):
        super().__init__("rover_navigation_gate")
        defaults = {
            "motion_enabled": False, "cmd_topic": "/nav2/cmd_vel",
            "output_topic": "/cmd_vel_safe", "scan_topic": "/scan",
            "odom_topic": "/odom", "base_frame": "base_link", "map_frame": "map",
            "odom_frame": "odom",
            "command_timeout_s": 0.3, "sensor_timeout_s": 0.5,
            "min_wheel_speed_mps": 0.35, "max_wheel_speed_mps": 0.5,
            "calibration_file": default_calibration_file(),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        calibration = load_calibration(str(self.get_parameter("calibration_file").value))
        self.safety = NavigationSafety(
            enabled=bool(self.get_parameter("motion_enabled").value),
            timeout=float(self.get_parameter("command_timeout_s").value),
            scan_timeout=float(self.get_parameter("sensor_timeout_s").value),
            track_width=float(calibration["track_width_m"]),
            min_wheel_speed=float(self.get_parameter("min_wheel_speed_mps").value),
            max_wheel_speed=float(self.get_parameter("max_wheel_speed_mps").value),
        )
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.scan_frame = None
        self.scan_time = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 1
        )
        self.health = self.create_publisher(String, "/rover/navigation/status", 10)
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_topic").value), self._command, 1
        )
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value),
            self._scan, qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom, 1
        )
        self.last_reason = None
        self.timer = self.create_timer(0.05, self._tick)
        self.health_timer = self.create_timer(1.0, self._health)
        self.get_logger().warning(
            "Navigation command gate armed" if self.safety.enabled
            else "Navigation command gate DISABLED: only zero motor commands"
        )

    def _command(self, message):
        unsupported = (message.linear.y, message.linear.z,
                       message.angular.x, message.angular.y)
        self.safety.command = (
            (float(message.linear.x), float(message.angular.z))
            if all(value == 0.0 for value in unsupported) else (math.nan, math.nan)
        )
        self.safety.command_at = time.monotonic()

    def _scan(self, message):
        self.safety.scan_at = time.monotonic()
        self.safety.scan_stamp = stamp_seconds(message.header.stamp)
        count = len(message.ranges)
        if self.safety.scan_stamp <= 0:
            self.safety.scan_valid = False
            self.scan_time = None
            self.get_logger().warning("Rejected scan with nonpositive timestamp")
            return
        self.safety.scan_valid = (
            count >= 90
            and bool(message.header.frame_id)
            and math.isfinite(message.range_min)
            and math.isfinite(message.range_max)
            and 0 < message.range_min < message.range_max
            and all(math.isfinite(v) for v in (
                message.angle_min, message.angle_max, message.angle_increment,
                message.scan_time,
            ))
            and message.angle_increment > 0
            and 0.02 <= message.scan_time <= 0.5
            and abs(count * message.angle_increment - 2 * math.pi) < 0.02
            and abs(message.angle_max - message.angle_min
                    - (count - 1) * message.angle_increment) < 0.02
            and sum(
                math.isfinite(v) and message.range_min <= v <= message.range_max
                for v in message.ranges
            ) >= count * 0.5
        )
        self.scan_frame = message.header.frame_id
        self.scan_time = Time.from_msg(message.header.stamp)

    def _odom(self, message):
        if (
            message.child_frame_id != self.base_frame
            or message.header.frame_id != self.odom_frame
            or not all(math.isfinite(v) for v in (
                message.pose.pose.position.x, message.pose.pose.position.y,
                message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                message.pose.pose.orientation.z, message.pose.pose.orientation.w,
                message.twist.twist.linear.x, message.twist.twist.angular.z,
            ))
        ):
            self.safety.odom_at = None
            return
        self.safety.odom_at = time.monotonic()
        self.safety.odom_stamp = stamp_seconds(message.header.stamp)

    def _tf_valid(self):
        if self.scan_time is None or not self.scan_frame:
            return False
        try:
            self.buffer.lookup_transform(self.map_frame, self.scan_frame, self.scan_time)
            transform = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
            age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds(transform.header.stamp)
            return -0.1 <= age <= self.safety.scan_timeout
        except TransformException:
            return False

    def _tick(self):
        velocity, reason = self.safety.result(
            time.monotonic(), self.get_clock().now().nanoseconds / 1e9, self._tf_valid()
        )
        message = Twist()
        message.linear.x, message.angular.z = velocity
        self.publisher.publish(message)
        if reason != self.last_reason:
            self.get_logger().warning(f"Navigation gate: {reason}")
            self.last_reason = reason
            self._health()

    def _health(self):
        status = String()
        status.data = json.dumps({
            "motion_enabled": self.safety.enabled, "reason": self.last_reason,
        })
        self.health.publish(status)

    def destroy_node(self):
        self.publisher.publish(Twist())
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NavigationGate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
