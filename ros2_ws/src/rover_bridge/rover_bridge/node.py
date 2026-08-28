"""ROS2 network bridge for the production Raspberry Pi rover services."""

import json
import logging
import math
import socket
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, CameraInfo, Image
from std_msgs.msg import String

from .protocol import encode_cmd, read_rgbd_packet


class RoverBridge(Node):
    def __init__(self):
        super().__init__("rover_bridge")
        self.declare_parameter("pi_host", "192.168.50.5")
        self.declare_parameter("command_port", 8765)
        self.declare_parameter("telemetry_port", 8766)
        self.declare_parameter("camera_port", 8770)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("socket_timeout_s", 1.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("telemetry_topic", "/rover/telemetry")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame_id", "camera_color_optical_frame")

        self._pi_host = str(self.get_parameter("pi_host").value)
        self._command_port = int(self.get_parameter("command_port").value)
        self._telemetry_port = int(self.get_parameter("telemetry_port").value)
        self._camera_port = int(self.get_parameter("camera_port").value)
        self._command_period = 1.0 / float(self.get_parameter("command_rate_hz").value)
        self._reconnect_delay = float(self.get_parameter("reconnect_delay_s").value)
        self._socket_timeout = float(self.get_parameter("socket_timeout_s").value)
        self._camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self._stop_event = threading.Event()

        self._twist_lock = threading.Lock()
        self._latest_twist = (0.0, 0.0)
        self._command_seq = 0

        self._telemetry_pub = self.create_publisher(
            String, str(self.get_parameter("telemetry_topic").value), 10
        )
        self._battery_pub = self.create_publisher(
            BatteryState, str(self.get_parameter("battery_topic").value), 10
        )
        self._rgb_pub = self.create_publisher(
            Image, str(self.get_parameter("rgb_topic").value), 10
        )
        self._depth_pub = self.create_publisher(
            Image, str(self.get_parameter("depth_topic").value), 10
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), 10
        )
        self._cmd_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._twist_callback,
            10,
        )

        self._threads = [
            threading.Thread(target=self._command_loop, name="rover-command", daemon=True),
            threading.Thread(target=self._telemetry_loop, name="rover-telemetry", daemon=True),
            threading.Thread(target=self._camera_loop, name="rover-camera", daemon=True),
        ]
        for thread in self._threads:
            thread.start()
        self.get_logger().info(
            "Bridging rover at %s (commands %s, telemetry %s, camera %s)",
            self._pi_host,
            self._command_port,
            self._telemetry_port,
            self._camera_port,
        )

    def _twist_callback(self, message):
        vx = float(message.linear.x)
        wz = float(message.angular.z)
        if not math.isfinite(vx) or not math.isfinite(wz):
            self.get_logger().warning("Ignoring non-finite /cmd_vel message")
            return
        with self._twist_lock:
            self._latest_twist = (vx, wz)

    def _next_command(self):
        with self._twist_lock:
            vx, wz = self._latest_twist
        seq = self._command_seq
        self._command_seq += 1
        return encode_cmd(seq, vx, wz)

    def _connect(self, port):
        return socket.create_connection(
            (self._pi_host, port), timeout=self._socket_timeout
        )

    def _command_loop(self):
        while not self._stop_event.is_set():
            client = None
            try:
                client = self._connect(self._command_port)
                client.settimeout(self._socket_timeout)
                self.get_logger().info("Connected to rover command port %s", self._command_port)
                while not self._stop_event.is_set():
                    client.sendall(self._next_command())
                    self._stop_event.wait(self._command_period)
            except (OSError, ValueError) as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warning("Rover command connection lost: %s", exc)
            finally:
                if client is not None:
                    try:
                        client.sendall(encode_cmd(self._command_seq, 0.0, 0.0))
                    except OSError:
                        pass
                    try:
                        client.close()
                    except OSError:
                        pass
            self._stop_event.wait(self._reconnect_delay)

    def _telemetry_loop(self):
        while not self._stop_event.is_set():
            client = None
            try:
                client = self._connect(self._telemetry_port)
                client.settimeout(self._socket_timeout)
                self.get_logger().info(
                    "Connected to rover telemetry port %s", self._telemetry_port
                )
                buffer = b""
                while not self._stop_event.is_set():
                    try:
                        chunk = client.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("telemetry socket closed")
                    buffer += chunk
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        if line.strip():
                            self._handle_telemetry_line(line)
            except (OSError, ValueError) as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warning("Rover telemetry connection lost: %s", exc)
            finally:
                if client is not None:
                    try:
                        client.close()
                    except OSError:
                        pass
            self._stop_event.wait(self._reconnect_delay)

    def _handle_telemetry_line(self, line):
        try:
            payload = json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.get_logger().warning("Invalid rover telemetry: %s", exc)
            return
        if not isinstance(payload, dict) or payload.get("type") != "telemetry":
            self.get_logger().warning("Ignoring unexpected rover telemetry payload")
            return

        raw = String()
        raw.data = json.dumps(payload, separators=(",", ":"))
        self._telemetry_pub.publish(raw)

        voltage = payload.get("voltage_v")
        if isinstance(voltage, (int, float)) and math.isfinite(voltage):
            battery = BatteryState()
            battery.header.stamp = self._stamp(payload.get("ts"))
            battery.voltage = float(voltage)
            battery.present = True
            self._battery_pub.publish(battery)

    def _camera_loop(self):
        while not self._stop_event.is_set():
            client = None
            try:
                client = self._connect(self._camera_port)
                client.settimeout(self._socket_timeout)
                self.get_logger().info("Connected to rover camera port %s", self._camera_port)
                while not self._stop_event.is_set():
                    header, rgb_bytes, depth_bytes = read_rgbd_packet(client)
                    self._publish_rgbd(header, rgb_bytes, depth_bytes)
            except (ConnectionError, OSError, ValueError) as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warning("Rover camera connection lost: %s", exc)
            finally:
                if client is not None:
                    try:
                        client.close()
                    except OSError:
                        pass
            self._stop_event.wait(self._reconnect_delay)

    def _publish_rgbd(self, header, rgb_bytes, depth_bytes):
        rgb = cv2.imdecode(np.frombuffer(rgb_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        depth = cv2.imdecode(
            np.frombuffer(depth_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED
        )
        if rgb is None or depth is None:
            raise ValueError("could not decode rover RGB-D packet")
        if depth.ndim != 2:
            raise ValueError("rover depth image is not single-channel")
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        rgb = np.ascontiguousarray(rgb)
        depth = np.ascontiguousarray(depth)
        stamp = self._stamp(header.get("ts"))

        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self._camera_frame_id
        rgb_msg.height, rgb_msg.width = rgb.shape[:2]
        rgb_msg.encoding = "bgr8"
        rgb_msg.is_bigendian = 0
        rgb_msg.step = rgb_msg.width * 3
        rgb_msg.data = rgb.tobytes()

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self._camera_frame_id
        depth_msg.height, depth_msg.width = depth.shape[:2]
        depth_msg.encoding = "16UC1"
        depth_msg.is_bigendian = 0
        depth_msg.step = depth_msg.width * 2
        depth_msg.data = depth.tobytes()

        info_msg = self._camera_info_message(header.get("camera_info"), stamp)
        self._rgb_pub.publish(rgb_msg)
        self._depth_pub.publish(depth_msg)
        self._camera_info_pub.publish(info_msg)

    def _camera_info_message(self, info, stamp):
        if not isinstance(info, dict):
            raise ValueError("camera_info is missing from rover camera packet")
        k = [float(value) for value in info.get("k", [])]
        if len(k) != 9:
            raise ValueError("camera_info.k must contain nine values")
        d = [float(value) for value in info.get("d", [])]
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = self._camera_frame_id
        message.width = int(info.get("width", 0))
        message.height = int(info.get("height", 0))
        message.distortion_model = str(info.get("distortion_model", "rational_polynomial"))
        message.d = d
        message.k = k
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
        return message

    def _stamp(self, timestamp):
        if not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            return self.get_clock().now().to_msg()
        seconds = int(timestamp)
        nanoseconds = int(round((timestamp - seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        stamp = self.get_clock().now().to_msg()
        stamp.sec = seconds
        stamp.nanosec = nanoseconds
        return stamp

    def destroy_node(self):
        self._stop_event.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
