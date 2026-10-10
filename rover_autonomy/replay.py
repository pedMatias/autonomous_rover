"""Sequence replay of run inputs/goals through mocked motors, never live hardware."""

from collections import Counter
import json
import logging
from pathlib import Path

import numpy as np

from rover_preview.core import CameraFrame, PreviewEngine, PreviewSettings
from .controller import Controller, Limits
from .motor import MockMotor


LOG = logging.getLogger(__name__)


class ReplayRecords:
    def __init__(self):
        self.counts = Counter()

    def submit(self, kind, payload, frame=None):
        self.counts[kind] += 1

    def status(self):
        return {"mode": "replay", "events": dict(self.counts)}


def replay_run(directory: Path):
    config = json.loads((directory / "config.json").read_text())
    now = [0.0]
    records = ReplayRecords()
    motor = MockMotor(lambda kind, payload: records.submit(kind, payload))
    controller = Controller(PreviewEngine(PreviewSettings(**config["preview"]), config["planner"]),
                            config["calibration"], motor, records, Limits(**config["limits"]),
                            clock=lambda: now[0])
    decisions, mismatches, input_errors, examples = 0, 0, 0, []
    with (directory / "events.jsonl").open() as stream:
        for line in stream:
            event = json.loads(line)
            now[0] = event["mono_s"]
            kind = event["kind"]
            if kind == "heartbeat":
                controller.heartbeat()
            elif kind == "feedback":
                controller.feedback(event["data"], event["received_at"])
            elif kind == "goal_accepted":
                try:
                    controller.accept_goal(*event["relative_goal"], event["goal_id"], controller.epoch)
                except ValueError as exc:
                    LOG.error("Cannot reconstruct recorded goal %s: %s", event["goal_id"], exc)
                    input_errors += 1
                    if len(examples) < 20:
                        examples.append({"goal_id": event["goal_id"], "error": str(exc)})
            elif kind == "stop_requested":
                controller.stop(event["reason"])
            elif kind == "decision":
                path = directory / f"frame-{event['frame_seq']}.npz"
                if not path.exists():
                    mismatches += 1
                    if len(examples) < 20:
                        examples.append({"frame_seq": event["frame_seq"], "error": "recorded frame missing"})
                    continue
                with np.load(path, allow_pickle=False) as frame_file:
                    metadata = json.loads(str(frame_file["metadata"].item()))
                    frame = CameraFrame(frame_file["rgb"], frame_file["depth"], metadata["calibration"],
                                        metadata["captured_at"], metadata["dequeued_at"], "run replay",
                                        metadata.get("source_sequence"), metadata.get("camera_metadata"))
                controller.watchdog()
                snapshot = controller.evaluate(frame, event["frame_seq"])
                decisions += 1
                same = (snapshot.data["state"], snapshot.data["selected_heading_rad"]) == (
                    event["state"], event["selected_heading_rad"])
                if not same:
                    mismatches += 1
                    if len(examples) < 20:
                        examples.append({"frame_seq": event["frame_seq"], "recorded": event["state"],
                                         "replayed": snapshot.data["state"]})
    controller.stop("replay finished")
    manifest_path = directory / "manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
    else:
        LOG.error("Run manifest missing: replaying an interrupted/unfinalized recording")
        manifest = {"complete": False, "error": "run manifest missing"}
    return {"decisions": decisions, "mismatches": mismatches, "examples": examples,
            "input_errors": input_errors, "motor_mode": "mock", "manifest": manifest}
