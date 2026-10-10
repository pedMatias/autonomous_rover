"""Independent acquisition, control and debug-render workers."""

from collections import deque
import logging
import threading
import time

import cv2
import psutil

from rover_preview.core import CameraFrame
from rover_preview.runtime import render_jpeg


LOG = logging.getLogger(__name__)


class ProfiledSource:
    name = "live OAK-D"

    def __init__(self, settings):
        self.settings = settings
        self.camera = None
        self.metadata = {}
        self.previous_sequence = None

    def start(self):
        from recorder.oak_camera import OakCamera

        self.camera = OakCamera(
            rgb_size=(self.settings.camera_width, self.settings.camera_height),
            fps=self.settings.camera_fps, latest_only=True, profile=True,
        )
        self.camera.start()
        self.camera.warmup(self.settings.warmup_s)

    def read(self, timeout):
        result = self.camera.read(timeout)
        if result is None:
            return None
        wall, rgb, depth = result
        self.metadata = dict(self.camera.last_metadata)
        sequence = self.metadata["depth_seq"]
        if self.previous_sequence is not None and sequence <= self.previous_sequence:
            raise ValueError("camera sequence did not advance")
        self.previous_sequence = sequence
        return CameraFrame(rgb, depth, self.camera.get_camera_info(), wall,
                           self.metadata["dequeued_at"], self.name,
                           self.metadata["depth_seq"], self.metadata)

    def stop(self):
        if self.camera is not None:
            self.camera.stop()


class Runtime:
    def __init__(self, controller, source, calibration):
        self.controller, self.source, self.calibration = controller, source, calibration
        self.stop_event = threading.Event()
        self.frame_lock = threading.Lock()
        self.latest = None
        self.frames = 0
        self.overwrites = 0
        self.sequence_gaps = 0
        self.device_previous = None
        self.last_camera_metadata = {}
        self.views = deque(maxlen=15)
        self.view_lock = threading.Lock()
        self.viewer_at = 0.0
        self.metrics = {}
        self.threads = []

    def start(self):
        self.controller.motor.start(self.controller.feedback, self.controller.fault)
        self.threads = [
            threading.Thread(target=self._capture, name="autonomy-camera", daemon=True),
            threading.Thread(target=self._control, name="autonomy-control", daemon=True),
            threading.Thread(target=self._render, name="autonomy-render", daemon=True),
        ]
        for thread in self.threads:
            thread.start()

    def _capture(self):
        try:
            self.source.start()
            while not self.stop_event.is_set():
                started = time.monotonic()
                frame = self.source.read(0.1)
                if frame is not None:
                    self.frames += 1
                    metadata = getattr(self.source, "metadata", {})
                    self.last_camera_metadata = dict(metadata)
                    sequence = metadata.get("depth_seq")
                    if sequence is not None and self.device_previous is not None:
                        self.sequence_gaps += max(0, sequence - self.device_previous - 1)
                    self.device_previous = sequence
                    self.controller.records.submit("frame", {
                        "frame_seq": self.frames, "camera": metadata,
                        "capture_ms": (time.monotonic() - started) * 1000,
                    }, frame)
                    with self.frame_lock:
                        if self.latest is not None:
                            self.overwrites += 1
                        self.latest = self.frames, frame
                self.stop_event.wait(max(0, 1 / self.controller.engine.settings.camera_fps
                                         - (time.monotonic() - started)))
        except (OSError, RuntimeError, ValueError, KeyError, ImportError, cv2.error) as exc:
            LOG.exception("Acquisition stopped")
            self.controller.fault(f"camera failed: {exc}")
        finally:
            self.source.stop()

    def _control(self):
        previous_feedback = time.monotonic()
        previous_tick = 0.0
        try:
            while not self.stop_event.wait(0.02):
                now = time.monotonic()
                if not self.controller.motor.motion_enabled:
                    data = self.controller.motor.sample(now - previous_feedback, self.calibration)
                    self.controller.feedback(data)
                    previous_feedback = now
                self.controller.watchdog()
                if now - previous_tick < 1 / self.controller.limits.control_hz:
                    continue
                previous_tick = now
                with self.frame_lock:
                    item, self.latest = self.latest, None
                if item is not None:
                    self.controller.evaluate(item[1], item[0])
        except (OSError, RuntimeError, ValueError, TypeError) as exc:
            LOG.exception("Control worker stopped")
            self.controller.fault(f"control failed: {exc}")

    def _render(self):
        process = psutil.Process()
        process.cpu_percent()
        previous_time, previous_frames = time.monotonic(), 0
        try:
            while not self.stop_event.wait(1 / self.controller.engine.settings.display_hz):
                now = time.monotonic()
                if now - previous_time >= 1:
                    self.metrics = {
                        "cpu_percent": process.cpu_percent(), "rss_mb": process.memory_info().rss / 1024 ** 2,
                        "input_hz": (self.frames - previous_frames) / (now - previous_time),
                        "control_overwrites": self.overwrites, "upstream_sequence_gaps": self.sequence_gaps,
                    }
                    previous_time, previous_frames = now, self.frames
                if now - self.viewer_at > 5:
                    continue
                with self.controller.lock:
                    snapshot = self.controller.last_snapshot
                started = time.monotonic()
                jpeg = render_jpeg(snapshot, self.controller.engine)
                if jpeg is not None:
                    with self.view_lock:
                        if not self.views or self.views[-1][0].data["seq"] != snapshot.data["seq"]:
                            self.views.append((snapshot, jpeg))
                    self.metrics["render_ms"] = (time.monotonic() - started) * 1000
        except (OSError, ValueError, TypeError, cv2.error) as exc:
            LOG.exception("GCS rendering failed")
            self.metrics["render_error"] = str(exc)

    def view(self):
        self.viewer_at = time.monotonic()
        with self.view_lock:
            return None if not self.views else self.views[-1][0]

    def jpeg(self, sequence):
        with self.view_lock:
            return next((jpeg for snapshot, jpeg in self.views if snapshot.data["seq"] == sequence), None)

    def status(self):
        return {**self.controller.status(), "profile": dict(self.metrics),
                "camera_input": dict(self.last_camera_metadata),
                "workers": {thread.name: thread.is_alive() for thread in self.threads}}

    def close(self):
        self.stop_event.set()
        self.controller.stop("process shutdown")
        self.controller.motor.close()
        for thread in self.threads:
            thread.join(timeout=10)
            if thread.is_alive():
                LOG.error("%s did not stop", thread.name)
        self.controller.records.submit("run_summary", self.status())
        self.controller.records.close()
