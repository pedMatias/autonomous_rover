"""ROS2 network bridge for the production Raspberry Pi rover services."""

import json
import math
import os
import socket
import threading
import time
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .odometry import OdometryIntegrator
from .protocol import encode_cmd, read_rgbd_packet


def default_calibration_file():
    configured_path = os.environ.get("ROVER_CALIBRATION_FILE")
    if configured_path:
        return configured_path
    source_path = Path(__file__).resolve().parents[4] / "config" / "calibration.yaml"
    if source_path.exists():
        return str(source_path)
    try:
        return os.path.join(
            get_package_share_directory("rover_bridge"),
            "config",
            "calibration.yaml",
        )
    except LookupError:
        pass
    return ""


def load_calibration(path):
    if not path:
        raise ValueError(
            "calibration_file is empty; set it to the canonical config/calibration.yaml"
        )
    try:
        with open(path, "r") as calibration_stream:
            calibration = yaml.safe_load(calibration_stream) or {}
    except OSError as exc:
        raise ValueError(f"could not read calibration_file {path}: {exc}") from exc
    if not isinstance(calibration, dict):
        raise ValueError(f"calibration_file {path} must contain a YAML mapping")

    required = (
        "telemetry_distance_unit",
        "meters_per_left_unit",
        "meters_per_right_unit",
        "track_width_m",
        "left_encoder_sign",
        "right_encoder_sign",
    )
    missing = [key for key in required if key not in calibration]
    if missing:
        raise ValueError(
            f"calibration_file {path} is missing: {', '.join(missing)}"
        )
    return calibration


def build_color_compressed(stamp, frame_id, rgb_jpeg):
    message = CompressedImage()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.format = "jpeg"
    message.data = rgb_jpeg
    return message


def build_depth_compressed(stamp, frame_id, depth_png):
    message = CompressedImage()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.format = "16UC1; compressedDepth png"
    message.data = depth_png
    return message


def fresh_twist(twist, last_update_time, now, timeout):
    if last_update_time is None or now - last_update_time > timeout:
        return 0.0, 0.0
    return twist


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
        self.declare_parameter("camera_socket_timeout_s", 10.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("telemetry_topic", "/rover/telemetry")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("calibration_file", default_calibration_file())
        self.declare_parameter("max_distance_delta_m", 2.0)
        self.declare_parameter("odom_pose_covariance", 0.05)
        self.declare_parameter("odom_twist_covariance", 0.1)
        self.declare_parameter("frame_id", "oakd_rgb_optical_frame")

        self._pi_host = str(self.get_parameter("pi_host").value)
        self._command_port = int(self.get_parameter("command_port").value)
        self._telemetry_port = int(self.get_parameter("telemetry_port").value)
        self._camera_port = int(self.get_parameter("camera_port").value)
        self._command_period = 1.0 / float(self.get_parameter("command_rate_hz").value)
        self._reconnect_delay = float(self.get_parameter("reconnect_delay_s").value)
        self._socket_timeout = float(self.get_parameter("socket_timeout_s").value)
        self._camera_socket_timeout = float(
            self.get_parameter("camera_socket_timeout_s").value
        )
        self._cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout_s").value)
        if not math.isfinite(self._cmd_vel_timeout) or self._cmd_vel_timeout <= 0.0:
            raise ValueError("cmd_vel_timeout_s must be finite and positive")
        self._camera_frame_id = str(self.get_parameter("frame_id").value)
        self._odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self._base_frame_id = str(self.get_parameter("base_frame_id").value)
        calibration_file = str(self.get_parameter("calibration_file").value)
        calibration = load_calibration(calibration_file)
        self._odom = OdometryIntegrator(
            left_distance_scale=float(calibration["meters_per_left_unit"]),
            right_distance_scale=float(calibration["meters_per_right_unit"]),
            track_width_m=float(calibration["track_width_m"]),
            left_encoder_sign=int(calibration["left_encoder_sign"]),
            right_encoder_sign=int(calibration["right_encoder_sign"]),
            max_distance_delta_m=float(
                self.get_parameter("max_distance_delta_m").value
            ),
        )
        self.get_logger().info(
            f"Using calibration file {calibration_file}: "
            f"left={calibration['meters_per_left_unit']} m/{calibration['telemetry_distance_unit']}, "
            f"right={calibration['meters_per_right_unit']} m/{calibration['telemetry_distance_unit']}, "
            f"track={calibration['track_width_m']}"
        )
        self._odom_pose_covariance = float(
            self.get_parameter("odom_pose_covariance").value
        )
        self._odom_twist_covariance = float(
            self.get_parameter("odom_twist_covariance").value
        )
        if not math.isfinite(self._odom_pose_covariance) or self._odom_pose_covariance <= 0.0:
            raise ValueError("odom_pose_covariance must be finite and positive")
        if not math.isfinite(self._odom_twist_covariance) or self._odom_twist_covariance <= 0.0:
            raise ValueError("odom_twist_covariance must be finite and positive")
        self._stop_event = threading.Event()

        self._camera_packet_condition = threading.Condition()
        self._camera_packet = None
        self._missing_pi_stamp_logged = False

        self._stats_lock = threading.Lock()
        self._rx_count = 0
        self._pub_count = 0
        self._dropped_frames = 0
        self._queue_drops = 0
        self._last_sequence = None
        self._last_stats_time = time.monotonic()
        self._last_stats_rx = 0
        self._last_stats_pub = 0

        self._twist_lock = threading.Lock()
        self._latest_twist = (0.0, 0.0)
        self._last_twist_time = None
        self._command_seq = 0

        self._telemetry_pub = self.create_publisher(
            String, str(self.get_parameter("telemetry_topic").value), 10
        )
        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._battery_pub = self.create_publisher(
            BatteryState, str(self.get_parameter("battery_topic").value), 10
        )
        compressed_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._color_compressed_pub = self.create_publisher(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            compressed_qos,
        )
        self._depth_compressed_pub = self.create_publisher(
            CompressedImage,
            "/camera/depth/image_raw/compressedDepth",
            compressed_qos,
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            qos_profile_sensor_data,
        )
        self._cmd_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._twist_callback,
            10,
        )
        self._stats_timer = self.create_timer(5.0, self._log_camera_stats)

        self._threads = [
            threading.Thread(target=self._command_loop, name="rover-command", daemon=True),
            threading.Thread(target=self._telemetry_loop, name="rover-telemetry", daemon=True),
            threading.Thread(
                target=self._camera_reader_loop, name="rover-camera-reader", daemon=True
            ),
            threading.Thread(
                target=self._camera_publisher_loop,
                name="rover-camera-publisher",
                daemon=True,
            ),
        ]
        for thread in self._threads:
            thread.start()
        self.get_logger().info(
            f"Bridging rover at {self._pi_host} "
            f"(commands {self._command_port}, telemetry {self._telemetry_port}, "
            f"camera {self._camera_port})"
        )

    def _twist_callback(self, message):
        vx = float(message.linear.x)
        wz = float(message.angular.z)
        if not math.isfinite(vx) or not math.isfinite(wz):
            self.get_logger().warning("Ignoring non-finite /cmd_vel message")
            return
        with self._twist_lock:
            self._latest_twist = (vx, wz)
            self._last_twist_time = time.monotonic()

    def _next_command(self):
        with self._twist_lock:
            twist = self._latest_twist
            last_twist_time = self._last_twist_time
        vx, wz = fresh_twist(
            twist,
            last_twist_time,
            time.monotonic(),
            self._cmd_vel_timeout,
        )
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
                self.get_logger().info(
                    f"Connected to rover command port {self._command_port}"
                )
                while not self._stop_event.is_set():
                    client.sendall(self._next_command())
                    self._stop_event.wait(self._command_period)
            except (OSError, ValueError) as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warning(f"Rover command connection lost: {exc}")
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
                    f"Connected to rover telemetry port {self._telemetry_port}"
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
                    self.get_logger().warning(
                        f"Rover telemetry connection lost: {exc}"
                    )
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
            self.get_logger().warning(f"Invalid rover telemetry: {exc}")
            return
        if not isinstance(payload, dict) or payload.get("type") != "telemetry":
            self.get_logger().warning("Ignoring unexpected rover telemetry payload")
            return

        raw = String()
        raw.data = json.dumps(payload, separators=(",", ":"))
        self._telemetry_pub.publish(raw)

        self._publish_odometry(payload)

        voltage = payload.get("voltage_v")
        if isinstance(voltage, (int, float)) and math.isfinite(voltage):
            battery = BatteryState()
            battery.header.stamp = self._stamp(payload.get("ts"))
            battery.voltage = float(voltage)
            battery.present = True
            self._battery_pub.publish(battery)

    def _publish_odometry(self, payload):
        data = payload.get("data")
        if not isinstance(data, dict):
            return
        left_distance_unit = data.get("odl")
        right_distance_unit = data.get("odr")
        timestamp = payload.get("ts")
        if not all(isinstance(value, (int, float)) for value in (left_distance_unit, right_distance_unit, timestamp)):
            return
        if not all(math.isfinite(float(value)) for value in (left_distance_unit, right_distance_unit, timestamp)):
            self.get_logger().warning("Ignoring non-finite wheel-distance telemetry")
            return
        try:
            state = self._odom.update(left_distance_unit, right_distance_unit, timestamp)
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring invalid wheel-distance telemetry: {exc}")
            return
        if self._odom.last_update_reset:
            self.get_logger().warning(
                "Resetting odometry baseline after encoder reset or timestamp jump"
            )
        stamp = self._stamp(timestamp)
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self._odom_frame_id
        message.child_frame_id = self._base_frame_id
        message.pose.pose.position.x = state.x
        message.pose.pose.position.y = state.y
        message.pose.pose.orientation.z = math.sin(state.theta / 2.0)
        message.pose.pose.orientation.w = math.cos(state.theta / 2.0)
        message.twist.twist.linear.x = state.linear_velocity
        message.twist.twist.angular.z = state.angular_velocity
        message.pose.covariance[0] = self._odom_pose_covariance
        message.pose.covariance[7] = self._odom_pose_covariance
        message.pose.covariance[35] = self._odom_pose_covariance
        message.twist.covariance[0] = self._odom_twist_covariance
        message.twist.covariance[7] = self._odom_twist_covariance
        message.twist.covariance[35] = self._odom_twist_covariance
        self._odom_pub.publish(message)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame_id
        transform.child_frame_id = self._base_frame_id
        transform.transform.translation.x = state.x
        transform.transform.translation.y = state.y
        transform.transform.rotation.z = message.pose.pose.orientation.z
        transform.transform.rotation.w = message.pose.pose.orientation.w
        self._tf_broadcaster.sendTransform(transform)

    def _reset_odometry_baseline(self):
        self._odom.reset()

    def _camera_reader_loop(self):
        while not self._stop_event.is_set():
            client = None
            try:
                client = self._connect(self._camera_port)
                client.settimeout(self._camera_socket_timeout)
                self.get_logger().info(
                    f"Connected to rover camera port {self._camera_port}"
                )
                while not self._stop_event.is_set():
                    packet = read_rgbd_packet(client)
                    self._record_received(packet[0])
                    with self._camera_packet_condition:
                        if self._camera_packet is not None:
                            with self._stats_lock:
                                self._queue_drops += 1
                        self._camera_packet = packet
                        self._camera_packet_condition.notify()
            except (ConnectionError, OSError, ValueError) as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warning(f"Rover camera connection lost: {exc}")
            finally:
                if client is not None:
                    try:
                        client.close()
                    except OSError:
                        pass
            self._stop_event.wait(self._reconnect_delay)

    def _camera_publisher_loop(self):
        while not self._stop_event.is_set():
            with self._camera_packet_condition:
                while self._camera_packet is None and not self._stop_event.is_set():
                    self._camera_packet_condition.wait(timeout=0.5)
                if self._stop_event.is_set():
                    return
                packet = self._camera_packet
                self._camera_packet = None
            try:
                self._publish_rgbd(*packet)
            except (ValueError, TypeError) as exc:
                self.get_logger().warning(f"Could not publish rover RGB-D packet: {exc}")

    def _record_received(self, header):
        with self._stats_lock:
            self._rx_count += 1
            sequence = header.get("seq")
            if isinstance(sequence, int):
                if (
                    self._last_sequence is not None
                    and sequence > self._last_sequence + 1
                ):
                    self._dropped_frames += sequence - self._last_sequence - 1
                self._last_sequence = sequence

    def _publish_rgbd(self, header, rgb_bytes, depth_bytes):
        stamp = self._stamp_from(header)
        self._color_compressed_pub.publish(
            build_color_compressed(stamp, self._camera_frame_id, rgb_bytes)
        )
        self._depth_compressed_pub.publish(
            build_depth_compressed(stamp, self._camera_frame_id, depth_bytes)
        )

        info_msg = self._camera_info_message(header.get("camera_info"), stamp)
        self._camera_info_pub.publish(info_msg)
        with self._stats_lock:
            self._pub_count += 1

    def _log_camera_stats(self):
        now = time.monotonic()
        with self._stats_lock:
            elapsed = max(now - self._last_stats_time, 1e-9)
            rx_fps = (self._rx_count - self._last_stats_rx) / elapsed
            pub_fps = (self._pub_count - self._last_stats_pub) / elapsed
            dropped_frames = self._dropped_frames
            queue_drops = self._queue_drops
            self._last_stats_time = now
            self._last_stats_rx = self._rx_count
            self._last_stats_pub = self._pub_count
        self.get_logger().info(
            "Camera stats: "
            f"rx_fps={rx_fps:.2f}, pub_fps={pub_fps:.2f}, "
            f"dropped_frames={dropped_frames}, queue_drops={queue_drops}"
        )

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

    def _stamp_from(self, header):
        timestamp = header.get("stamp")
        if timestamp is None:
            timestamp = header.get("ts")
        if timestamp is None and not self._missing_pi_stamp_logged:
            self.get_logger().warning(
                "Rover camera packet has no Pi timestamp; using WSL2 arrival time"
            )
            self._missing_pi_stamp_logged = True
        return self._stamp(timestamp)

    def destroy_node(self):
        self._stop_event.set()
        with self._camera_packet_condition:
            self._camera_packet_condition.notify_all()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
