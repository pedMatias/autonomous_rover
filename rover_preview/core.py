"""Raw-depth preview and bounded, frame-consistent decision snapshots."""

from dataclasses import asdict, dataclass, fields
import logging
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import yaml

from rover_avoidance.planner import (
    choose_heading,
    choose_search_step,
    command_for_heading,
    depth_to_base_points,
    obstacle_points_in_view,
)


LOG = logging.getLogger(__name__)
ROOT = Path(__file__).resolve().parent.parent


def finite_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


@dataclass(frozen=True)
class PreviewSettings:
    camera_translation_m: tuple = (0.0, 0.0, 0.12)
    camera_rotation_xyzw: tuple = (-0.5, 0.5, -0.5, 0.5)
    camera_mount_verified: bool = False
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: float = 10.0
    warmup_s: float = 8.0
    processing_hz: float = 10.0
    display_hz: float = 3.0
    display_width: int = 480
    jpeg_quality: int = 70
    max_debug_points: int = 600

    def __post_init__(self):
        for name, length in (("camera_translation_m", 3), ("camera_rotation_xyzw", 4)):
            values = getattr(self, name)
            if not isinstance(values, (list, tuple)) or len(values) != length:
                raise ValueError(f"{name} must contain {length} numbers")
            values = tuple(finite_number(value, name) for value in values)
            object.__setattr__(self, name, values)
        if np.linalg.norm(self.camera_rotation_xyzw) < 1e-12:
            raise ValueError("camera quaternion must not be zero")
        if not isinstance(self.camera_mount_verified, bool):
            raise ValueError("camera_mount_verified must be a boolean")
        for name, low, high in (
            ("camera_width", 64, 1280), ("camera_height", 64, 720),
            ("display_width", 160, 640), ("jpeg_quality", 20, 90),
            ("max_debug_points", 10, 2000),
        ):
            value = getattr(self, name)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"{name} must be an integer in [{low}, {high}]")
        if self.camera_width % 16:
            raise ValueError("camera_width must be a multiple of 16 for aligned depth")
        for name, low, high in (
            ("camera_fps", 1.0, 30.0), ("processing_hz", 1.0, 20.0),
            ("display_hz", 0.5, 5.0), ("warmup_s", 0.0, 30.0),
        ):
            if not low <= finite_number(getattr(self, name), name) <= high:
                raise ValueError(f"{name} must be in [{low}, {high}]")


PLANNER_KEYS = (
    "max_linear_speed_mps", "max_angular_speed_rps", "min_align_rate_rps",
    "wheel_separation_m", "max_wheel_speed_mps", "min_wheel_speed_mps",
    "heading_gain", "align_tolerance_rad", "align_release_ratio",
    "goal_tolerance_m", "obstacle_clearance_m", "obstacle_min_height_m",
    "obstacle_max_height_m", "obstacle_lookahead_m", "depth_min_m", "depth_max_m",
    "depth_decimation", "min_valid_depth_fraction", "missing_depth_blocks_paths",
    "sensor_timeout_s", "preview_search_max_step_m", "preview_search_stop_margin_m",
)


def validate_planner(values: dict) -> dict:
    result = {key: values[key] for key in PLANNER_KEYS}
    if not isinstance(result["missing_depth_blocks_paths"], bool):
        raise ValueError("missing_depth_blocks_paths must be a boolean")
    if type(result["depth_decimation"]) is not int or not 4 <= result["depth_decimation"] <= 32:
        raise ValueError("preview depth_decimation must be an integer in [4, 32]")
    for key in PLANNER_KEYS:
        if key in ("missing_depth_blocks_paths", "depth_decimation"):
            continue
        result[key] = finite_number(result[key], key)
        if result[key] < 0.0:
            raise ValueError(f"{key} must not be negative")
    if not 0.35 <= result["max_linear_speed_mps"] < result["max_wheel_speed_mps"] <= 1.3:
        raise ValueError("hypothetical forward speed must preserve the 0.35 m/s stall floor")
    if not 0.0 < result["depth_min_m"] < result["depth_max_m"] <= 10.0:
        raise ValueError("depth limits must satisfy 0 < min < max <= 10 m")
    if not result["obstacle_min_height_m"] < result["obstacle_max_height_m"]:
        raise ValueError("obstacle height limits must increase")
    if not 0.0 < result["min_valid_depth_fraction"] <= 1.0:
        raise ValueError("min_valid_depth_fraction must be in (0, 1]")
    for key in ("obstacle_clearance_m", "obstacle_lookahead_m", "sensor_timeout_s",
                "preview_search_max_step_m", "preview_search_stop_margin_m"):
        if not 0.0 < result[key] <= 5.0:
            raise ValueError(f"{key} must be in (0, 5]")
    command_for_heading(0.5, **steering_settings(result))
    return result


def steering_settings(planner: dict) -> dict:
    return {
        "max_linear_speed": planner["max_linear_speed_mps"],
        "max_angular_speed": planner["max_angular_speed_rps"],
        "min_angular_speed": planner["min_align_rate_rps"],
        "wheel_separation_m": planner["wheel_separation_m"],
        "max_wheel_speed_mps": planner["max_wheel_speed_mps"],
        "min_wheel_speed_mps": planner["min_wheel_speed_mps"],
        "heading_gain": planner["heading_gain"],
        "align_tolerance": planner["align_tolerance_rad"],
        "release_ratio": planner["align_release_ratio"],
    }


def load_settings(path: Path) -> tuple:
    with path.open() as stream:
        values = yaml.safe_load(stream)
    if not isinstance(values, dict):
        raise ValueError("preview configuration must be a mapping")
    planner_path = (path.parent / values.pop("planner_config")).resolve()
    with planner_path.open() as stream:
        planner = yaml.safe_load(stream)["rover_avoidance"]["ros__parameters"]
    allowed = {field.name for field in fields(PreviewSettings)}
    if set(values) - allowed:
        raise ValueError(f"unknown preview settings: {sorted(set(values) - allowed)}")
    return PreviewSettings(**values), validate_planner(planner)


def validate_goal(x: object, y: object) -> tuple:
    x, y = finite_number(x, "x"), finite_number(y, "y")
    if x <= 0.05 or math.hypot(x, y) > 10.0:
        raise ValueError("goal must be more than 0.05 m forward and within 10 m")
    return x, y


@dataclass(frozen=True)
class CameraFrame:
    rgb: np.ndarray
    depth: np.ndarray
    calibration: dict
    captured_at: float | None
    dequeued_at: float
    source: str
    source_sequence: int | None = None
    camera_metadata: dict | None = None


@dataclass(frozen=True)
class Snapshot:
    data: dict
    frame: CameraFrame | None


def bounded_points(points: np.ndarray, limit: int) -> list:
    if not len(points):
        return []
    stride = max(1, math.ceil(len(points) / limit))
    return np.round(points[::stride], 4).tolist()


class PreviewEngine:
    def __init__(self, settings: PreviewSettings, planner: dict):
        self.settings = settings
        self.planner = validate_planner(planner)
        self._lock = threading.Lock()
        self._goal = None
        self._goal_revision = 0
        self._sequence = 0
        self._latest = Snapshot({
            "seq": 0, "state": "STARTING", "reason": "waiting for camera",
            "preview_only": True, "motor_interface": "absent", "goal": None,
            "goal_revision": 0, "candidates": [], "source": "not started",
        }, None)
        self._frozen = None

    def set_goal(self, goal: tuple | None) -> int:
        validated = None if goal is None else validate_goal(*goal)
        with self._lock:
            self._goal = validated
            self._goal_revision += 1
            return self._goal_revision

    def latest(self) -> Snapshot:
        with self._lock:
            return self._latest

    def frozen(self) -> Snapshot | None:
        with self._lock:
            return self._frozen

    def config(self) -> dict:
        return {"preview": asdict(self.settings), "planner": self.planner}

    def _publish(self, data: dict, frame: CameraFrame | None, revision: int) -> Snapshot | None:
        with self._lock:
            if revision != self._goal_revision:
                return None
            previous = self._latest.data["state"]
            self._sequence += 1
            data.update(seq=self._sequence, goal_revision=revision,
                        preview_only=True, motor_interface="absent")
            snapshot = Snapshot(data, frame)
            self._latest = snapshot
            if data["state"] in ("PREVIEW_BLOCKED", "PREVIEW_SEARCH", "FAULT") and (
                data["state"] != previous or self._frozen is None
                or revision != self._frozen.data["goal_revision"]
            ):
                self._frozen = snapshot
            if previous != data["state"]:
                log = LOG.error if data["state"] == "FAULT" else LOG.info
                log("%s: %s", data["state"], data["reason"])
            return snapshot

    def fault(self, reason: str, frame: CameraFrame | None = None,
              revision: int | None = None, goal: tuple | None = None) -> Snapshot | None:
        with self._lock:
            if revision is None:
                revision, goal = self._goal_revision, self._goal
            frame = frame or self._latest.frame
        LOG.error("Preview fault: %s", reason)
        return self._publish({
            "state": "FAULT", "reason": reason, "goal": goal,
            "source": frame.source if frame else "unavailable",
            "captured_at": frame.captured_at if frame else None,
            "candidates": [], "selected_heading_rad": None, "search_step": None,
            "sampled_points": [], "obstacle_points": [],
        }, frame, revision)

    def evaluate(self, frame: CameraFrame) -> Snapshot | None:
        started = time.monotonic()
        with self._lock:
            goal, revision = self._goal, self._goal_revision
        p = self.planner
        try:
            if started - frame.dequeued_at > p["sensor_timeout_s"]:
                raise ValueError("depth input is stale since host dequeue")
            self._validate_frame(frame)
            info = SimpleNamespace(**frame.calibration)
            points, unknown, fraction = depth_to_base_points(
                frame.depth, info, self.settings.camera_translation_m,
                self.settings.camera_rotation_xyzw,
                decimation=p["depth_decimation"], min_depth_m=p["depth_min_m"],
                max_depth_m=p["depth_max_m"], unknown_range_m=p["obstacle_lookahead_m"],
                missing_depth_blocks_paths=p["missing_depth_blocks_paths"],
            )
            considered = obstacle_points_in_view(
                points, p["obstacle_clearance_m"], p["obstacle_min_height_m"],
                p["obstacle_max_height_m"], p["obstacle_lookahead_m"],
            )
            candidates, selected, search = [], None, None
            state, reason = "IDLE", "waiting for a hypothetical relative goal"
            if fraction < p["min_valid_depth_fraction"]:
                state, reason = "FAULT", f"too little valid depth ({fraction:.1%})"
            elif goal is not None:
                distance = math.hypot(*goal)
                if distance <= p["goal_tolerance_m"]:
                    state, reason = "PREVIEW_AT_GOAL", "hypothetical goal is within tolerance"
                else:
                    selected = choose_heading(
                        math.atan2(goal[1], goal[0]), distance, points,
                        rover_half_width=p["obstacle_clearance_m"],
                        obstacle_min_height=p["obstacle_min_height_m"],
                        obstacle_max_height=p["obstacle_max_height_m"],
                        lookahead_m=p["obstacle_lookahead_m"], unknown_points=unknown,
                        steering=steering_settings(p), candidates_debug=candidates,
                    )
                    if selected is not None:
                        state, reason = "PREVIEW_CLEAR", "a full local candidate path has no measured collision"
                    else:
                        filtered_unknown = obstacle_points_in_view(
                            unknown, p["obstacle_clearance_m"], p["obstacle_min_height_m"],
                            p["obstacle_max_height_m"], p["obstacle_lookahead_m"],
                        )
                        search = choose_search_step(
                            candidates, np.concatenate((considered, filtered_unknown)),
                            p["obstacle_clearance_m"], p["preview_search_max_step_m"],
                            p["preview_search_stop_margin_m"],
                        )
                        state = "PREVIEW_SEARCH" if search else "PREVIEW_BLOCKED"
                        reason = (
                            "all full paths blocked; a hypothetical short prefix exists"
                            if search else "all candidate footprints blocked; no usable short prefix"
                        )
            ranges = np.hypot(considered[:, 0], considered[:, 1])
            nearest = considered[int(np.argmin(ranges))].tolist() if len(ranges) else None
            sampled = frame.depth[::p["depth_decimation"], ::p["depth_decimation"]]
            valid = (sampled >= p["depth_min_m"] * 1000) & (sampled <= p["depth_max_m"] * 1000)
            data = {
                "state": state, "reason": reason, "source": frame.source, "goal": goal,
                "captured_at": frame.captured_at,
                "dequeued_at": frame.dequeued_at,
                "frame_size": [frame.depth.shape[1], frame.depth.shape[0]],
                "valid_fraction": fraction,
                "unmeasured_fraction": float(1 - np.count_nonzero(valid) / sampled.size),
                "sampled_depth_points": len(points), "considered_obstacle_points": len(considered),
                "unknown_points": len(unknown),
                "sampled_points": bounded_points(points, self.settings.max_debug_points),
                "obstacle_points": bounded_points(considered, self.settings.max_debug_points),
                "nearest_obstacle_point": nearest,
                "nearest_obstacle_range_m": float(np.min(ranges)) if len(ranges) else None,
                "selected_heading_rad": selected, "candidates": candidates,
                "search_step": search, "collision_radius_m": p["obstacle_clearance_m"] + 0.025,
                "missing_depth_blocks_paths": p["missing_depth_blocks_paths"],
                "camera_mount_verified": self.settings.camera_mount_verified,
                "processing_ms": (time.monotonic() - started) * 1000,
            }
            return self._publish(data, frame, revision)
        except (ValueError, TypeError, AttributeError, KeyError) as exc:
            return self.fault(str(exc), frame, revision, goal)

    @staticmethod
    def _validate_frame(frame: CameraFrame) -> None:
        if frame.depth.ndim != 2 or frame.depth.dtype != np.uint16:
            raise ValueError("depth must be a uint16 image in millimetres")
        if not all(0 < dimension <= maximum for dimension, maximum in zip(frame.depth.shape, (720, 1280))):
            raise ValueError("frame dimensions must be within 1280x720 and nonempty")
        if frame.rgb.dtype != np.uint8 or frame.rgb.shape != (*frame.depth.shape, 3):
            raise ValueError("RGB must be uint8 BGR and aligned to the depth image")
        if (frame.captured_at is not None and not np.isfinite(frame.captured_at)) or not np.isfinite(frame.dequeued_at):
            raise ValueError("frame timestamps must be finite")
        height, width = frame.depth.shape
        if (frame.calibration["width"], frame.calibration["height"]) != (width, height):
            raise ValueError("calibration dimensions must match aligned RGB/depth")
        k = np.asarray(frame.calibration["k"], dtype=float)
        if k.shape != (9,) or not np.all(np.isfinite(k)) or k[0] <= 0 or k[4] <= 0:
            raise ValueError("calibration K must contain nine finite values with positive focal lengths")
