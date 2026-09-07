"""Headless rover hardware agent for the ROS2 computer.

The Raspberry Pi owns the ESP32 UART. A remote ROS2 computer sends newline-
delimited JSON velocity commands and receives newline-delimited telemetry.
The agent has no ROS2 dependency, so it can run on the factory Pi image.

Command protocol (client -> command_port)::

    {"type":"cmd_vel","seq":12,"vx":0.2,"wz":0.0}

Telemetry protocol (telemetry_port -> client)::

    {"type":"telemetry","seq":42,"ts":1710000000.123,
     "data":{...},"cmd_echo":{"left_mps":0.2,"right_mps":0.2,"seq":12}}
"""
import argparse
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
import socket
import threading
import time

import yaml

from base_ctrl import BaseController


REPO_ROOT = os.path.dirname(os.path.realpath(__file__))
DEFAULT_AGENT_CONFIG = os.path.join(REPO_ROOT, "config", "rover_agent.yaml")
DEFAULT_CALIBRATION = os.path.join(REPO_ROOT, "config", "calibration.yaml")


def load_yaml(path):
    with open(path, "r") as fh:
        return yaml.safe_load(fh) or {}


def setup_logging(log_path):
    log_dir = os.path.dirname(log_path)
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)
    logger = logging.getLogger("rover_agent")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    file_handler = RotatingFileHandler(log_path, maxBytes=5 * 1024 * 1024, backupCount=5)
    file_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(file_handler)
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(console_handler)
    return logger


def numeric(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def twist_to_wheels(vx, wz, wheel_separation, max_wheel_speed):
    left = vx - (wz * wheel_separation / 2.0)
    right = vx + (wz * wheel_separation / 2.0)
    peak = max(abs(left), abs(right))
    if peak > max_wheel_speed:
        scale = max_wheel_speed / peak
        left *= scale
        right *= scale
    return left, right


def limit_wheel_speeds(previous, target, elapsed_s, max_acceleration_mps2):
    """Limit a normal wheel command's change over the elapsed interval."""
    if max_acceleration_mps2 <= 0.0:
        return target
    maximum_delta = max(0.0, elapsed_s) * max_acceleration_mps2
    limited = []
    for previous_speed, target_speed in zip(previous, target):
        delta = max(
            -maximum_delta,
            min(maximum_delta, target_speed - previous_speed),
        )
        limited.append(previous_speed + delta)
    return tuple(limited)


class RoverAgent:
    def __init__(self, agent_config, calibration_config, logger, base=None):
        self.config = agent_config
        self.calibration = calibration_config
        self.log = logger
        self.base = base or BaseController(
            agent_config.get("uart_device", "/dev/ttyAMA0"),
            int(agent_config.get("uart_baud", 115200)),
        )
        self.stop_event = threading.Event()
        self.command_lock = threading.Lock()
        self.telemetry_client = None
        self.telemetry_lock = threading.Lock()
        self.command_client = None
        self.last_command_time = time.monotonic()
        self.last_command_seq = None
        self.last_wheels = (0.0, 0.0)
        self.last_wheel_update_time = time.monotonic()
        self.last_voltage = None
        self.telemetry_seq = 0
        self.threads = []

    @property
    def command_timeout(self):
        return float(self.config.get("command_timeout_s", 0.5))

    def start(self):
        self.base.base_json_ctrl({"T": 142, "cmd": 50})
        self.base.base_json_ctrl({"T": 131, "cmd": 1})
        self.base.base_json_ctrl({"T": 143, "cmd": 0})
        self.threads = [
            threading.Thread(target=self._feedback_loop, name="uart-reader", daemon=True),
            threading.Thread(target=self._watchdog_loop, name="watchdog", daemon=True),
            threading.Thread(target=self._command_server, name="command-server", daemon=True),
            threading.Thread(target=self._telemetry_server, name="telemetry-server", daemon=True),
        ]
        for thread in self.threads:
            thread.start()
        self.log.info(
            "Listening for commands on %s:%s and telemetry on %s:%s",
            self.config.get("bind_host", "0.0.0.0"), self.config.get("command_port", 8765),
            self.config.get("bind_host", "0.0.0.0"), self.config.get("telemetry_port", 8766),
        )

    def stop(self):
        self.stop_event.set()
        self._send_wheels(0.0, 0.0, "shutdown")
        with self.telemetry_lock:
            client = self.telemetry_client
            self.telemetry_client = None
        if client is not None:
            try:
                client.close()
            except OSError:
                pass
        if self.command_client is not None:
            try:
                self.command_client.close()
            except OSError:
                pass

    def _send_wheels(self, left, right, reason="command", command_seq=None):
        with self.command_lock:
            self.base.base_speed_ctrl(left, right)
            self.last_wheels = (left, right)
            self.last_wheel_update_time = time.monotonic()
            if reason == "command":
                self.last_command_seq = command_seq
        if reason != "command":
            self.log.info("wheel command L=%.3f R=%.3f (%s)", left, right, reason)

    def _feedback_loop(self):
        while not self.stop_event.is_set():
            data = self.base.feedback_data()
            timestamp = time.time()
            if not isinstance(data, dict):
                continue
            if numeric(data.get("v")):
                raw_voltage = float(data["v"])
                self.last_voltage = raw_voltage / 100.0 if raw_voltage > 100 else raw_voltage
            self._publish_telemetry({
                "ts": timestamp,
                "data": data,
                "voltage_v": self.last_voltage,
            })

    def _watchdog_loop(self):
        interval = min(0.05, self.command_timeout / 4.0)
        while not self.stop_event.wait(interval):
            with self.command_lock:
                expired = time.monotonic() - self.last_command_time > self.command_timeout
                moving = self.last_wheels != (0.0, 0.0)
            if expired and moving:
                self._send_wheels(0.0, 0.0, "watchdog")

    def _command_server(self):
        host = self.config.get("bind_host", "0.0.0.0")
        port = int(self.config.get("command_port", 8765))
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, port))
            server.listen(1)
            server.settimeout(1.0)
            while not self.stop_event.is_set():
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                self.log.info("Command client connected: %s", address)
                self.command_client = client
                client.settimeout(1.0)
                try:
                    self._read_commands(client)
                finally:
                    self.command_client = None
                    try:
                        client.close()
                    except OSError:
                        pass
                    self._send_wheels(0.0, 0.0, "command_disconnect")
                    self.log.info("Command client disconnected: %s", address)

    def _read_commands(self, client):
        buffer = b""
        while not self.stop_event.is_set():
            try:
                chunk = client.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                return
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    command = json.loads(line.decode("utf-8"))
                    self._handle_command(command)
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                    self.log.warning("Invalid command: %s", exc)

    def _handle_command(self, command):
        if command.get("type") != "cmd_vel":
            raise ValueError("command type must be cmd_vel")
        vx, wz = command.get("vx"), command.get("wz")
        if not numeric(vx) or not numeric(wz):
            raise ValueError("vx and wz must be finite numbers")
        low_voltage = self.calibration.get("low_battery_stop_v")
        if self.last_voltage is not None and low_voltage is not None and self.last_voltage < float(low_voltage):
            self._send_wheels(0.0, 0.0, "low_battery")
            return
        left, right = twist_to_wheels(
            float(vx), float(wz),
            float(self.config.get("wheel_separation_m", 0.314)),
            float(self.config.get("max_wheel_speed_mps", 0.5)),
        )
        with self.command_lock:
            previous_wheels = self.last_wheels
            elapsed_s = time.monotonic() - self.last_wheel_update_time
        left, right = limit_wheel_speeds(
            previous_wheels,
            (left, right),
            elapsed_s,
            float(self.config.get("max_wheel_accel_mps2", 1.0)),
        )
        self._send_wheels(left, right, command_seq=command.get("seq"))
        with self.command_lock:
            self.last_command_time = time.monotonic()

    def _telemetry_server(self):
        host = self.config.get("bind_host", "0.0.0.0")
        port = int(self.config.get("telemetry_port", 8766))
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, port))
            server.listen(1)
            server.settimeout(1.0)
            while not self.stop_event.is_set():
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                client.settimeout(0.2)
                with self.telemetry_lock:
                    previous_client = self.telemetry_client
                    self.telemetry_client = client
                if previous_client is not None:
                    try:
                        previous_client.close()
                    except OSError:
                        pass
                self.log.info("Telemetry client connected: %s", address)

    def _publish_telemetry(self, payload):
        with self.command_lock:
            left_mps, right_mps = self.last_wheels
            command_seq = self.last_command_seq
        cmd_echo = {
            "left_mps": left_mps,
            "right_mps": right_mps,
            "seq": command_seq,
        }
        with self.telemetry_lock:
            self.telemetry_seq += 1
            message = {
                "type": "telemetry",
                "seq": self.telemetry_seq,
                "ts": payload["ts"],
                "data": payload["data"],
                "voltage_v": payload["voltage_v"],
                "cmd_echo": cmd_echo,
            }
            encoded = (
                json.dumps(message, separators=(",", ":")) + "\n"
            ).encode("utf-8")
            client = self.telemetry_client
            if client is None:
                return
            try:
                client.sendall(encoded)
            except (OSError, socket.timeout):
                if self.telemetry_client is client:
                    self.telemetry_client = None
                try:
                    client.close()
                except OSError:
                    pass


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--agent-config", default=DEFAULT_AGENT_CONFIG)
    parser.add_argument("--calibration", default=DEFAULT_CALIBRATION)
    parser.add_argument("--log", default=os.path.expanduser("~/rover/logs/rover-agent.log"))
    return parser.parse_args()


def main():
    args = parse_args()
    logger = setup_logging(args.log)
    agent = RoverAgent(load_yaml(args.agent_config), load_yaml(args.calibration), logger)
    try:
        agent.start()
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger.info("Stopping on Ctrl+C")
    finally:
        agent.stop()


if __name__ == "__main__":
    main()