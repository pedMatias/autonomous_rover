"""Independent network-to-LaserScan adapter with bounded latest-scan delivery."""

import json
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from rover_lidar.protocol import read_scan


def build_scan(scan, frame_id):
    message = LaserScan()
    seconds = math.floor(scan.stamp)
    carry, nanoseconds = divmod(round((scan.stamp - seconds) * 1_000_000_000),
                               1_000_000_000)
    message.header.stamp.sec = seconds + carry
    message.header.stamp.nanosec = nanoseconds
    message.header.frame_id = frame_id
    message.angle_min = -math.pi
    message.angle_increment = 2 * math.pi / len(scan.ranges_mm)
    message.angle_max = message.angle_min + (len(scan.ranges_mm) - 1) * message.angle_increment
    # Re-binned angles are not acquisition order; don't invent per-ray timestamps.
    message.time_increment = 0.0
    message.scan_time = scan.scan_time
    message.range_min = scan.range_min
    message.range_max = scan.range_max
    message.ranges = scan.ranges_metres()
    message.intensities = [float(value) for value in scan.intensities]
    return message


class LidarBridge(Node):
    def __init__(self):
        super().__init__("rover_lidar_bridge")
        defaults = {
            "pi_host": "192.168.0.12", "port": 8771, "scan_topic": "/scan",
            "frame_id": "laser_frame", "max_age_s": 0.5,
            "future_tolerance_s": 0.05, "reconnect_delay_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.host = str(self.get_parameter("pi_host").value)
        self.port = int(self.get_parameter("port").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.max_age = float(self.get_parameter("max_age_s").value)
        self.future_tolerance = float(self.get_parameter("future_tolerance_s").value)
        self.reconnect_delay = float(self.get_parameter("reconnect_delay_s").value)
        if not self.host or not 1 <= self.port <= 65535 or not self.frame_id:
            raise ValueError("lidar host, port and frame_id must be valid")
        if (
            any(not math.isfinite(v) for v in
                (self.max_age, self.future_tolerance, self.reconnect_delay))
            or not 0 < self.max_age <= 1 or not 0 <= self.future_tolerance <= 0.1
            or self.reconnect_delay <= 0
        ):
            raise ValueError("invalid lidar timing parameters")
        if self.get_parameter("use_sim_time").value:
            raise ValueError("live lidar reception requires use_sim_time=false")
        self.publisher = self.create_publisher(
            LaserScan, str(self.get_parameter("scan_topic").value), qos_profile_sensor_data
        )
        self.health = self.create_publisher(String, "/rover/lidar/status", 10)
        self.lock = threading.Lock()
        self.latest = None
        self.connected = False
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()
        self.timer = self.create_timer(0.02, self._publish)
        self.health_timer = self.create_timer(1.0, self._health)
        self.last_stamp = None

    def _receive(self):
        while not self.stop.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=0.5) as client:
                    client.settimeout(0.5)
                    previous = None
                    previous_stamp = None
                    with self.lock:
                        self.connected = True
                    self.get_logger().info(f"Lidar connected to {self.host}:{self.port}")
                    while not self.stop.is_set():
                        scan = read_scan(client)
                        age = time.time() - scan.stamp
                        if not -self.future_tolerance <= age <= self.max_age:
                            raise ValueError(f"stale/future lidar acquisition stamp (age {age:.3f}s)")
                        if previous is not None and (
                            scan.sequence <= previous or scan.stamp <= previous_stamp
                        ):
                            raise ValueError("lidar sequence/timestamp did not advance")
                        previous, previous_stamp = scan.sequence, scan.stamp
                        with self.lock:
                            self.latest = scan
            except (OSError, ValueError, ConnectionError) as exc:
                if not self.stop.is_set():
                    self.get_logger().warning(f"Lidar unavailable: {exc}")
            finally:
                with self.lock:
                    self.connected = False
                    self.latest = None
            self.stop.wait(self.reconnect_delay)

    def _publish(self):
        with self.lock:
            scan = self.latest
            self.latest = None
        if scan is not None:
            if -self.future_tolerance <= time.time() - scan.stamp <= self.max_age:
                self.publisher.publish(build_scan(scan, self.frame_id))
                self.last_stamp = scan.stamp
            else:
                self.get_logger().warning("Discarding scan that expired before ROS publication")

    def _health(self):
        with self.lock:
            connected = self.connected
        age = None if self.last_stamp is None else time.time() - self.last_stamp
        message = String()
        message.data = json.dumps({
            "connected": connected,
            "fresh": connected and age is not None and 0 <= age <= self.max_age,
            "scan_age_s": age,
        }, allow_nan=False)
        self.health.publish(message)

    def destroy_node(self):
        self.stop.set()
        self.thread.join(timeout=2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
