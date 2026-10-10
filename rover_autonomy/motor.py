"""Mock output by default; exclusive UART and bounded commands when explicitly enabled."""

import fcntl
import json
import logging
import math
import shutil
import subprocess
import termios
import threading
import time


LOG = logging.getLogger(__name__)


class MockMotor:
    motion_enabled = False

    def __init__(self, event, clock=time.monotonic, min_speed=0.35, max_speed=0.5):
        self.event, self.clock = event, clock
        self.min_speed, self.max_speed = min_speed, max_speed
        self.lock = threading.Lock()
        self.wheels = (0.0, 0.0)
        self.distances = [0.0, 0.0]

    def submit(self, left, right, sequence):
        if not all(math.isfinite(v) for v in (left, right)):
            raise ValueError("wheel speeds must be finite")
        if (left, right) != (0.0, 0.0) and (
            min(left, right) < self.min_speed or max(left, right) > self.max_speed
            or (left + right) / 2 < 0.35
        ):
            raise ValueError("wheel commands violate the forward stall or speed limits")
        with self.lock:
            self.wheels = left, right
        self.event("command_queued", {"command_seq": sequence, "left": left, "right": right})
        self.event("command_written", {"command_seq": sequence, "left": left, "right": right,
                                       "mock": True, "acknowledged": False})

    def sample(self, dt, calibration):
        with self.lock:
            for i, scale in enumerate(("meters_per_left_unit", "meters_per_right_unit")):
                self.distances[i] += self.wheels[i] * dt / calibration[scale]
            return {"T": 1001, "odl": self.distances[0], "odr": self.distances[1], "v": 1240}

    def status(self):
        with self.lock:
            return {"mode": "mock", "written_wheels": self.wheels, "error": None}

    def start(self, feedback, fault):
        pass

    def close(self):
        self.submit(0.0, 0.0, None)


class SerialMotor:
    motion_enabled = True

    def __init__(self, config, event):
        self.config, self.event = config, event
        self.condition = threading.Condition()
        self.closed = threading.Event()
        self.stop_written = threading.Event()
        self.pending = None
        self.written = (0.0, 0.0)
        self.error = None
        self.last_submit = time.monotonic()
        self.ser = None
        self.threads = []
        self.timeout = float(config["command_timeout_s"])
        self.max_speed = float(config["max_wheel_speed_mps"])
        self.min_speed = 0.35
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("command_timeout_s must be finite and positive")
        if not math.isfinite(self.max_speed) or self.max_speed < 0.35:
            raise ValueError("max_wheel_speed_mps must be finite and at least 0.35")

    def start(self, feedback, fault):
        import serial

        device = self.config["uart_device"]
        if shutil.which("fuser") is None:
            raise RuntimeError("fuser is required to check existing UART owners")
        owners = subprocess.run(["fuser", device], capture_output=True, text=True, check=False)
        if owners.returncode == 0:
            owner_ids = (owners.stdout + owners.stderr).strip()
            raise RuntimeError(f"UART already owned by PID(s): {owner_ids}")
        if owners.returncode != 1:
            raise RuntimeError(f"UART ownership check failed: {owners.stderr.strip()}")
        self.ser = serial.Serial(device, int(self.config["uart_baud"]), timeout=0.1,
                                 write_timeout=0.2, exclusive=True)
        try:
            fcntl.ioctl(self.ser.fileno(), termios.TIOCEXCL)
            for data in ({"T": 1, "L": 0, "R": 0}, {"T": 142, "cmd": 50},
                         {"T": 131, "cmd": 1}, {"T": 143, "cmd": 0}):
                self._write(data)
            self.threads = [
                threading.Thread(target=self._writer, args=(fault,), name="autonomy-uart-write", daemon=True),
                threading.Thread(target=self._reader, args=(feedback, fault),
                                 name="autonomy-uart-read", daemon=True),
            ]
            for thread in self.threads:
                thread.start()
        except (OSError, ValueError, RuntimeError):
            self.ser.close()
            raise

    def submit(self, left, right, sequence):
        if not all(math.isfinite(v) for v in (left, right)):
            raise ValueError("wheel speeds must be finite")
        if (left, right) != (0.0, 0.0) and (
            min(left, right) < self.min_speed or max(left, right) > self.max_speed
            or (left + right) / 2 < 0.35
        ):
            raise ValueError("each wheel must stay at or above 0.35 m/s and within the forward speed limit")
        with self.condition:
            self.pending = (left, right, sequence, time.monotonic())
            self.last_submit = time.monotonic()
            if left == right == 0:
                self.stop_written.clear()
            self.event("command_queued", {"command_seq": sequence, "left": left, "right": right})
            self.condition.notify()

    def _write(self, data):
        encoded = (json.dumps(data) + "\n").encode()
        if self.ser.write(encoded) != len(encoded):
            raise OSError("incomplete UART write")

    def _writer(self, fault):
        try:
            while not self.closed.is_set():
                with self.condition:
                    if self.pending is None:
                        self.condition.wait(timeout=0.05)
                    item, self.pending = self.pending, None
                    now = time.monotonic()
                    if item is None and self.written != (0.0, 0.0) and now - self.last_submit > self.timeout:
                        item = (0.0, 0.0, None, now)
                        LOG.error("UART command watchdog expired")
                    if item is None:
                        continue
                    left, right, sequence, submitted = item
                    if now - submitted > self.timeout:
                        left = right = 0.0
                        LOG.error("Expired queued command replaced with stop")
                    started = time.monotonic()
                    self._write({"T": 1, "L": left, "R": right})
                    self.written = (left, right)
                    if left == right == 0:
                        self.stop_written.set()
                self.event("command_written", {
                    "command_seq": sequence, "left": left, "right": right,
                    "queue_ms": (started - submitted) * 1000,
                    "write_ms": (time.monotonic() - started) * 1000, "acknowledged": False,
                })
        except (OSError, ValueError) as exc:
            self.error = str(exc)
            LOG.exception("UART writer failed")
            fault(f"UART writer failed: {exc}")

    def _reader(self, feedback, fault):
        buffer = b""
        try:
            while not self.closed.is_set():
                chunk = self.ser.read(min(8192, max(1, self.ser.in_waiting)))
                if not chunk:
                    continue
                buffer += chunk
                if len(buffer) > 16384:
                    raise ValueError("UART feedback exceeded bounded line buffer")
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    try:
                        data = json.loads(line.decode())
                    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                        LOG.warning("Invalid UART feedback: %s", exc)
                        continue
                    if not isinstance(data, dict):
                        LOG.warning("UART feedback is not an object")
                        continue
                    self.event("raw_feedback", {"data": data})
                    if all(key in data for key in ("odl", "odr", "v")):
                        feedback(data)
        except (OSError, ValueError) as exc:
            self.error = str(exc)
            LOG.exception("UART reader failed")
            fault(f"UART reader failed: {exc}")

    def status(self):
        with self.condition:
            return {"mode": "UART", "written_wheels": self.written, "error": self.error,
                    "pending": self.pending is not None}

    def close(self):
        if self.ser is None:
            return
        self.submit(0.0, 0.0, None)
        if not self.stop_written.wait(timeout=1):
            LOG.error("Shutdown stop write was not confirmed")
        self.closed.set()
        with self.condition:
            self.condition.notify_all()
        for thread in self.threads:
            thread.join(timeout=1)
        self.ser.close()
