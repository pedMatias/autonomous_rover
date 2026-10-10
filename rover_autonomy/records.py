"""Bounded background lossless recording and explicit incomplete-run reporting."""

import json
import logging
from pathlib import Path
import queue
import shutil
import threading
import time
import uuid

import numpy as np


LOG = logging.getLogger(__name__)
EVENT_FLUSH_INTERVAL_S = 1.0


class RunLog:
    def __init__(self, directory: Path, config: dict, capacity: int, reserve_bytes: int):
        self.run_id = uuid.uuid4().hex
        self.directory = directory / self.run_id
        self.directory.mkdir(parents=True, exist_ok=False)
        self.config = config
        self.reserve_bytes = reserve_bytes
        self.queue = queue.Queue(maxsize=capacity)
        self.lock = threading.Lock()
        self.closing = threading.Event()
        self.thread = threading.Thread(target=self._write, name="autonomy-log", daemon=True)
        self.dropped = 0
        self.first_gap = None
        self.last_gap = None
        self.error = None
        self.written = 0
        self.last_write_ms = None
        self.complete = True
        self.gaps = []
        self.gap_ranges_truncated = False
        self.thread.start()

    def submit(self, kind: str, payload: dict, frame=None) -> None:
        event = {"kind": kind, "run_id": self.run_id, "mono_s": time.monotonic(),
                 "wall_s": time.time(), **payload}
        with self.lock:
            if self.closing.is_set() or self.error is not None:
                self._gap(event)
                return
            try:
                self.queue.put_nowait((event, frame))
            except queue.Full:
                self._gap(event)

    def _gap(self, event: dict) -> None:
        if self.dropped == 0:
            LOG.error("Run recording incomplete: queue full, closed or unavailable")
        self.complete = False
        self.dropped += 1
        identity = {key: event.get(key) for key in ("kind", "frame_seq", "command_seq", "mono_s")}
        if self.first_gap is None:
            self.first_gap = identity
        self.last_gap = identity
        sequence = event.get("frame_seq")
        if type(sequence) is int:
            if self.gaps and self.gaps[-1]["kind"] == event["kind"] and self.gaps[-1]["last"] + 1 == sequence:
                self.gaps[-1]["last"] = sequence
            elif len(self.gaps) < 64:
                self.gaps.append({"kind": event["kind"], "first": sequence, "last": sequence})
            else:
                self.gap_ranges_truncated = True

    def status(self) -> dict:
        with self.lock:
            return {"run_id": self.run_id, "complete": self.complete, "dropped_events": self.dropped,
                    "first_gap": self.first_gap, "last_gap": self.last_gap, "error": self.error,
                    "written_events": self.written, "queue_size": self.queue.qsize(),
                    "gap_ranges": [dict(gap) for gap in self.gaps],
                    "gap_ranges_truncated": self.gap_ranges_truncated,
                    "last_write_ms": self.last_write_ms}

    def close(self) -> None:
        self.closing.set()
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            with self.lock:
                self.complete = False
            LOG.error("Recording is still draining; manifest is not final")

    def _write(self) -> None:
        current_event = None
        try:
            (self.directory / "config.json").write_text(json.dumps(self.config, allow_nan=False))
            with (self.directory / "events.jsonl").open("w") as stream:
                while not self.closing.is_set() or not self.queue.empty():
                    try:
                        event, frame = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    current_event = event
                    started = time.monotonic()
                    required = 65536 if frame is None else frame.rgb.nbytes + frame.depth.nbytes + 65536
                    if shutil.disk_usage(self.directory).free <= self.reserve_bytes + required:
                        raise OSError("recording disk reserve reached")
                    if frame is not None:
                        filename = f"frame-{event['frame_seq']}.npz"
                        np.savez(self.directory / filename, rgb=frame.rgb, depth=frame.depth,
                                 metadata=json.dumps({
                                     "calibration": frame.calibration, "captured_at": frame.captured_at,
                                     "dequeued_at": frame.dequeued_at, "source": frame.source,
                                     "source_sequence": frame.source_sequence,
                                     "camera_metadata": frame.camera_metadata or {},
                                 }, allow_nan=False))
                        event["file"] = filename
                    stream.write(json.dumps(event, allow_nan=False) + "\n")
                    if time.monotonic() - getattr(self, "_last_flush", 0.0) >= EVENT_FLUSH_INTERVAL_S:
                        stream.flush()
                        self._last_flush = time.monotonic()
                    with self.lock:
                        self.written += 1
                        self.last_write_ms = (time.monotonic() - started) * 1000
                    current_event = None
                stream.flush()
        except (OSError, ValueError, TypeError) as exc:
            LOG.exception("Run recording failed")
            with self.lock:
                self.error = str(exc)
                self.complete = False
                if current_event is not None:
                    self._gap(current_event)
                while not self.queue.empty():
                    queued, _ = self.queue.get_nowait()
                    self._gap(queued)
        finally:
            try:
                (self.directory / "manifest.json").write_text(
                    json.dumps(self.status(), allow_nan=False))
            except OSError:
                LOG.exception("Could not persist run manifest")
