"""Camera acquisition and subscriber-paced visualization; no hardware control."""

from collections import deque
import io
import json
import logging
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
import psutil

from rover_avoidance.planner import quaternion_rotate

from .core import (
    CameraFrame, PreviewEngine, PreviewSettings, Snapshot,
    validate_goal, validate_planner,
)


LOG = logging.getLogger(__name__)


class LiveSource:
    name = "live OAK-D"
    recorded_decision = None

    def __init__(self, settings: PreviewSettings):
        self.settings = settings
        self.camera = None

    def start(self) -> None:
        from recorder.oak_camera import OakCamera

        self.camera = OakCamera(
            rgb_size=(self.settings.camera_width, self.settings.camera_height),
            fps=self.settings.camera_fps, latest_only=True,
        )
        self.camera.start()

    def read(self, timeout: float) -> CameraFrame | None:
        result = self.camera.read(timeout)
        if result is None:
            return None
        timestamp, rgb, depth = result
        return CameraFrame(
            rgb, depth, self.camera.get_camera_info(), timestamp,
            time.monotonic(), self.name,
        )

    def stop(self) -> None:
        if self.camera is not None:
            self.camera.stop()


class StaticSource:
    def __init__(self, rgb: np.ndarray, depth: np.ndarray, calibration: dict,
                 name: str, captured_at: float | None = None, refresh_timestamp: bool = True):
        self.rgb, self.depth, self.calibration = rgb, depth, calibration
        self.name, self.captured_at = name, captured_at
        self.refresh_timestamp = refresh_timestamp
        self.recorded_decision = None

    def start(self) -> None:
        pass

    def read(self, timeout: float) -> CameraFrame:
        return CameraFrame(
            self.rgb, self.depth, self.calibration,
            time.time() if self.captured_at is None and self.refresh_timestamp else self.captured_at,
            time.monotonic(), self.name,
        )

    def stop(self) -> None:
        pass


def demo_source(scene: str) -> StaticSource:
    depth = np.full((240, 320), 2000, dtype=np.uint16)
    if scene == "wall":
        depth[:] = 400
    elif scene == "empty":
        depth[:] = 0
    rgb = np.zeros((240, 320, 3), dtype=np.uint8)
    rgb[:] = (65, 45, 25)
    cv2.putText(rgb, f"SYNTHETIC: {scene}", (15, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return StaticSource(rgb, depth, {
        "width": 320, "height": 240,
        "k": [280.0, 0.0, 160.0, 0.0, 280.0, 120.0, 0.0, 0.0, 1.0],
    }, f"synthetic demo ({scene}); not live")


def replay_source(path: Path) -> tuple:
    with np.load(path, allow_pickle=False) as recording:
        rgb, depth, k = recording["rgb"], recording["depth"], recording["k"]
        metadata = json.loads(str(recording["metadata"].item())) if "metadata" in recording else None
    if depth.ndim != 2:
        raise ValueError("recorded depth must be a two-dimensional uint16 array")
    height, width = depth.shape
    if width > 1280 or height > 720 or width < 1 or height < 1:
        raise ValueError("recorded frame dimensions must be within 1280x720")
    calibration = {"width": width, "height": height, "k": k.reshape(-1).tolist()}
    captured_at = None
    settings, planner, goal = None, None, None
    if metadata is not None:
        if metadata["schema"] != 1:
            raise ValueError("unsupported snapshot recording schema")
        settings = PreviewSettings(**metadata["config"]["preview"])
        planner = validate_planner(metadata["config"]["planner"])
        goal = None if metadata["decision"]["goal"] is None else validate_goal(*metadata["decision"]["goal"])
        captured_at = metadata["decision"]["captured_at"]
    source = StaticSource(
        rgb, depth, calibration, f"recorded replay ({path.name}); not live", captured_at,
        refresh_timestamp=False,
    )
    source.recorded_decision = None if metadata is None else metadata["decision"]
    PreviewEngine._validate_frame(source.read(0.0))
    return source, settings, planner, goal


def recording_bytes(snapshot: Snapshot, engine: PreviewEngine) -> bytes:
    if snapshot.frame is None:
        raise ValueError("this failure has no camera frame to record")
    stream = io.BytesIO()
    np.savez_compressed(
        stream, rgb=snapshot.frame.rgb, depth=snapshot.frame.depth,
        k=np.asarray(snapshot.frame.calibration["k"]),
        metadata=json.dumps({
            "schema": 1, "config": engine.config(), "decision": snapshot.data,
        }, allow_nan=False),
    )
    return stream.getvalue()


def render_jpeg(snapshot: Snapshot, engine: PreviewEngine) -> bytes | None:
    if snapshot.frame is None:
        return None
    frame, settings, p = snapshot.frame, engine.settings, engine.planner
    height, width = frame.depth.shape
    scale = min(1.0, settings.display_width / width)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    rgb = cv2.resize(frame.rgb, size, interpolation=cv2.INTER_AREA)
    depth_small = cv2.resize(frame.depth, size, interpolation=cv2.INTER_NEAREST)
    depth_color = cv2.applyColorMap(
        np.clip(depth_small.astype(np.float32) / (p["depth_max_m"] * 1000) * 255, 0, 255).astype(np.uint8),
        cv2.COLORMAP_TURBO,
    )
    depth_color[(depth_small < p["depth_min_m"] * 1000) | (depth_small > p["depth_max_m"] * 1000)] = 0
    points = np.asarray(snapshot.data.get("obstacle_points", []), dtype=float)
    if points.size:
        rotation = settings.camera_rotation_xyzw
        optical = quaternion_rotate(
            points - settings.camera_translation_m,
            (-rotation[0], -rotation[1], -rotation[2], rotation[3]),
        )
        k = frame.calibration["k"]
        for x, y, z in optical:
            if z <= 0:
                continue
            u, v = round((k[0] * x / z + k[2]) * scale), round((k[4] * y / z + k[5]) * scale)
            if 0 <= u < size[0] and 0 <= v < size[1]:
                cv2.circle(rgb, (u, v), 2, (0, 0, 255), -1)
    image = np.concatenate((rgb, depth_color), axis=1)
    cv2.putText(image, "RGB: red = considered points", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(image, "Depth: black = unmeasured/out of range", (size[0] + 8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, settings.jpeg_quality])
    if not ok:
        raise ValueError("preview JPEG encoding failed")
    return encoded.tobytes()


class PreviewRuntime:
    def __init__(self, engine: PreviewEngine, source):
        self.engine, self.source = engine, source
        self.stop_event = threading.Event()
        self.capture_thread = threading.Thread(target=self._capture, name="preview-camera", daemon=True)
        self.display_thread = threading.Thread(target=self._display, name="preview-display", daemon=True)
        self._view_lock = threading.Lock()
        self._frozen_render_lock = threading.Lock()
        self._viewer_at = 0.0
        self._views = deque(maxlen=3)
        self._frozen_view = None
        self._health = {}
        self._frame_count = 0
        self._encode_count = 0
        self._last_dequeue = None
        self._encoding_error = None

    def start(self) -> None:
        self.capture_thread.start()
        self.display_thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        for thread in (self.capture_thread, self.display_thread):
            if thread.ident is not None:
                thread.join(timeout=5.0)
                if thread.is_alive():
                    LOG.error("%s did not stop within five seconds", thread.name)

    def _capture(self) -> None:
        try:
            self.source.start()
            if isinstance(self.source, LiveSource):
                until = time.monotonic() + self.engine.settings.warmup_s
                while time.monotonic() < until and not self.stop_event.is_set():
                    self.source.read(0.1)
            last_received = time.monotonic()
            while not self.stop_event.is_set():
                started = time.monotonic()
                frame = self.source.read(0.1)
                if frame is not None:
                    last_received = time.monotonic()
                    self._last_dequeue = frame.dequeued_at
                    self._frame_count += 1
                    self.engine.evaluate(frame)
                elif time.monotonic() - last_received > self.engine.planner["sensor_timeout_s"]:
                    if self.engine.latest().data["state"] != "FAULT":
                        self.engine.fault("camera timeout: no new RGB-D frame")
                self.stop_event.wait(max(0.0, 1 / self.engine.settings.processing_hz - (time.monotonic() - started)))
        except (ImportError, RuntimeError, OSError, ValueError, KeyError, cv2.error) as exc:
            LOG.exception("Camera preview stopped")
            self.engine.fault(f"camera acquisition failed: {exc}")
        finally:
            if sys.exc_info()[0] is not None:
                LOG.error("Unexpected camera thread failure", exc_info=True)
                self.engine.fault("unexpected camera thread failure; see service log")
            self.source.stop()

    def _display(self) -> None:
        process = psutil.Process()
        process.cpu_percent()
        previous_time, previous_frames = time.monotonic(), 0
        try:
            while not self.stop_event.is_set():
                started = time.monotonic()
                if started - previous_time >= 1.0:
                    health = {
                        "process_cpu_percent": process.cpu_percent(),
                        "rss_mb": process.memory_info().rss / (1024 * 1024),
                        "input_fps": (self._frame_count - previous_frames) / (started - previous_time),
                        "encoded_frames": self._encode_count,
                        "battery_v": None, "wheel_feedback": "unavailable: camera-only preview",
                    }
                    temperature = Path("/sys/class/thermal/thermal_zone0/temp")
                    if temperature.exists():
                        health["cpu_temperature_c"] = float(temperature.read_text()) / 1000
                    else:
                        health["cpu_temperature_c"] = None
                    with self._view_lock:
                        self._health = health
                    previous_time, previous_frames = started, self._frame_count
                with self._view_lock:
                    active = started - self._viewer_at < 5.0
                    last_seq = self._views[-1][0].data["seq"] if self._views else -1
                snapshot = self.engine.latest()
                if active and snapshot.data["seq"] != last_seq:
                    try:
                        jpeg = render_jpeg(snapshot, self.engine)
                    except (ValueError, cv2.error) as exc:
                        LOG.exception("Could not visualize decision %s", snapshot.data["seq"])
                        with self._view_lock:
                            self._views.append((snapshot, None))
                            self._encoding_error = str(exc)
                        self.stop_event.wait(1 / self.engine.settings.display_hz)
                        continue
                    with self._view_lock:
                        self._views.append((snapshot, jpeg))
                        self._encoding_error = None
                    if jpeg is not None:
                        self._encode_count += 1
                self.stop_event.wait(max(0.0, 1 / self.engine.settings.display_hz - (time.monotonic() - started)))
        except (OSError, ValueError, cv2.error, psutil.Error) as exc:
            LOG.exception("Visualization stopped")
            with self._view_lock:
                self._encoding_error = str(exc)
        finally:
            if sys.exc_info()[0] is not None:
                LOG.error("Unexpected visualization thread failure", exc_info=True)
                with self._view_lock:
                    self._encoding_error = "unexpected visualization failure; see service log"

    def view(self, frozen: bool = False) -> Snapshot | None:
        if frozen:
            snapshot = self.engine.frozen()
            if snapshot is None:
                return None
            with self._frozen_render_lock:
                with self._view_lock:
                    cached = self._frozen_view
                if cached is None or cached[0].data["seq"] != snapshot.data["seq"]:
                    try:
                        jpeg = render_jpeg(snapshot, self.engine)
                    except (ValueError, cv2.error) as exc:
                        LOG.exception("Could not visualize frozen decision %s", snapshot.data["seq"])
                        jpeg = None
                        with self._view_lock:
                            self._encoding_error = str(exc)
                    with self._view_lock:
                        self._frozen_view = (snapshot, jpeg)
            return snapshot
        with self._view_lock:
            self._viewer_at = time.monotonic()
            return self._views[-1][0] if self._views else self.engine.latest()

    def jpeg(self, sequence: int, frozen: bool = False) -> bytes | None:
        with self._view_lock:
            views = [self._frozen_view] if frozen else self._views
            for view in views:
                if view is not None and view[0].data["seq"] == sequence:
                    return view[1]
        return None

    def health(self) -> dict:
        with self._view_lock:
            result = dict(self._health)
            result["visualization_error"] = self._encoding_error
        result.update(
            camera_thread_alive=self.capture_thread.is_alive(),
            display_thread_alive=self.display_thread.is_alive(),
            input_dequeue_age_s=None if self._last_dequeue is None else time.monotonic() - self._last_dequeue,
            input_frames=self._frame_count,
        )
        return result
