"""Goal tracking and lightweight supervision using the existing geometry planner."""

from collections import OrderedDict
from dataclasses import dataclass, fields
import logging
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import yaml

from recorder.odometry import OdometryIntegrator
from rover_avoidance.planner import (
    choose_heading, command_for_heading, depth_to_base_points, normalize_angle,
    obstacle_points_in_view, relative_goal_to_odom,
)
from rover_preview.core import (
    PreviewEngine, Snapshot, bounded_points, finite_number, load_settings, steering_settings,
    validate_planner,
)


LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class Limits:
    input_timeout_s: float = 0.6
    heartbeat_timeout_s: float = 2.0
    control_hz: float = 10.0
    stop_speed_mps: float = 0.02
    stopped_samples: int = 3
    progress_timeout_s: float = 15.0
    progress_distance_m: float = 0.1
    forward_speed_mps: float = 0.4
    min_wheel_speed_mps: float = 0.35
    log_queue_size: int = 64
    disk_reserve_bytes: int = 1024 ** 3

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            finite_number(value, field.name)
            if value <= 0:
                raise ValueError(f"{field.name} must be positive")
        for key in ("stopped_samples", "log_queue_size", "disk_reserve_bytes"):
            if type(getattr(self, key)) is not int:
                raise ValueError(f"{key} must be an integer")
        if not 0.35 <= self.min_wheel_speed_mps < self.forward_speed_mps:
            raise ValueError("wheel speed floor must be at least 0.35 and below forward speed")


def load_config(path: Path):
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("autonomy configuration must be a mapping")
    config = dict(config)
    settings, planner = load_settings(path.parent / config.pop("preview_config"))
    with (path.parent / config.pop("calibration_config")).open() as stream:
        calibration = yaml.safe_load(stream)
    with (path.parent / config.pop("agent_config")).open() as stream:
        agent = yaml.safe_load(stream)
    planner["max_linear_speed_mps"] = config.get("forward_speed_mps", 0.4)
    planner["min_wheel_speed_mps"] = config.get("min_wheel_speed_mps", 0.35)
    planner = validate_planner(planner)
    allowed = {field.name for field in fields(Limits)}
    if set(config) - allowed:
        raise ValueError(f"unknown autonomy settings: {sorted(set(config) - allowed)}")
    return settings, planner, calibration, agent, Limits(**config)


class Controller:
    def __init__(self, engine, calibration, motor, records, limits=Limits(), clock=time.monotonic):
        self.engine, self.motor, self.records = engine, motor, records
        self.limits, self.clock = limits, clock
        self.odom = OdometryIntegrator(
            left_distance_scale=calibration["meters_per_left_unit"],
            right_distance_scale=calibration["meters_per_right_unit"],
            track_width_m=calibration["track_width_m"],
        )
        self.low_voltage = finite_number(calibration["low_battery_stop_v"], "low_battery_stop_v")
        self.lock = threading.RLock()
        self.pose = None
        self.telemetry = None
        self.feedback_at = None
        self.frame_at = None
        self.heartbeat_at = None
        self.heartbeat_timeout_s = limits.heartbeat_timeout_s
        self.heartbeat_watchdog_enabled = True
        self.frame_seq = 0
        self.command_seq = 0
        self.epoch = 0
        self.goal = None
        self.relative_goal = None
        self.goal_id = None
        self.state = "STARTING"
        self.reason = "waiting for depth and feedback"
        self.last_snapshot = Snapshot({"seq": 0, "candidates": [], "state": self.state}, None)
        self.request_cache = OrderedDict()
        self.last_distance = None
        self.progress_at = None
        self.bypass_side = 0
        self.stopped_count = 0
        self.fatal_error = None
        self.last_wheels = (0.0, 0.0)

    def _event(self, kind, **payload):
        self.records.submit(kind, payload)

    def _state(self, state, reason):
        if (state, reason) != (self.state, self.reason):
            LOG.info("%s: %s", state, reason)
            self._event("state", state=state, reason=reason, goal_id=self.goal_id)
        self.state, self.reason = state, reason

    def _command(self, left, right):
        self.command_seq += 1
        self.last_wheels = (left, right)
        self._event("command_requested", command_seq=self.command_seq, frame_seq=self.frame_seq,
                    goal_id=self.goal_id, left=left, right=right)
        self.motor.submit(left, right, self.command_seq)

    def stop(self, reason="operator stop", state="STOPPED"):
        with self.lock:
            self.goal = None
            self.epoch += 1
            self._command(0.0, 0.0)
            self._state(state, reason)

    def fault(self, reason):
        with self.lock:
            self.fatal_error = reason
            self.stop(reason, "FAULT")

    def heartbeat(self):
        with self.lock:
            self.heartbeat_at = self.clock()
            self._event("heartbeat")

    def configure_heartbeat(self, enabled, timeout_s):
        if type(enabled) is not bool:
            raise ValueError("enabled must be a boolean")
        if enabled:
            timeout_s = finite_number(timeout_s, "timeout_s")
            if not 1.0 <= timeout_s <= 60.0:
                raise ValueError("timeout_s must be between 1 and 60 seconds")
        elif timeout_s is not None:
            raise ValueError("timeout_s must be null when the heartbeat watchdog is disabled")
        with self.lock:
            self.heartbeat_watchdog_enabled = enabled
            self.heartbeat_timeout_s = timeout_s
            self._event("heartbeat_watchdog_configured", enabled=enabled, timeout_s=timeout_s)
            if self.goal is not None:
                reason = self._not_ready(self.clock())
                if reason:
                    self.stop(reason, "FAULT")
            return {"enabled": enabled, "timeout_s": timeout_s}

    def feedback(self, data, received_at=None):
        with self.lock:
            now = self.clock() if received_at is None else received_at
            try:
                if not isinstance(data, dict):
                    raise ValueError("feedback must be an object")
                odl = finite_number(data["odl"], "odl")
                odr = finite_number(data["odr"], "odr")
                voltage = finite_number(data["v"], "v")
                voltage = voltage / 100 if voltage > 100 else voltage
                pose = self.odom.update(odl, odr, now)
                if self.odom.last_update_reset:
                    raise ValueError("odometry continuity lost")
                self.pose, self.telemetry, self.feedback_at = pose, {**data, "voltage_v": voltage}, now
                self._event("feedback", data=self.telemetry, pose=pose, received_at=now)
                if voltage < self.low_voltage:
                    self.stop("low battery", "FAULT")
                if self.state == "STOPPING":
                    vx, wz = pose[3:]
                    b = self.engine.planner["wheel_separation_m"]
                    stopped = max(abs(vx - wz * b / 2), abs(vx + wz * b / 2)) <= self.limits.stop_speed_mps
                    self.stopped_count = self.stopped_count + 1 if stopped else 0
                    if self.stopped_count >= self.limits.stopped_samples:
                        distance = math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])
                        self.goal = None
                        self.epoch += 1
                        if distance <= self.engine.planner["goal_tolerance_m"]:
                            self._state("GOAL_REACHED", "goal tolerance reached; wheel odometry confirms stopped")
                        else:
                            self._state("BLOCKED", "stopped outside goal tolerance; no creep recovery")
            except (ValueError, KeyError, TypeError) as exc:
                LOG.error("Invalid motor feedback: %s", exc)
                self.feedback_at = None
                self.stop(str(exc), "FAULT")

    def _not_ready(self, now):
        if self.fatal_error:
            return self.fatal_error
        if self.frame_at is None or now - self.frame_at > self.limits.input_timeout_s:
            return "depth missing or stale"
        if self.feedback_at is None or now - self.feedback_at > self.limits.input_timeout_s:
            return "odometry missing or stale"
        if self.telemetry["voltage_v"] < self.low_voltage:
            return "low battery"
        if self.heartbeat_watchdog_enabled and (
            self.heartbeat_at is None or now - self.heartbeat_at > self.heartbeat_timeout_s
        ):
            return "GCS heartbeat expired"
        return None

    def accept_goal(self, x, y, request_id, epoch):
        x, y = finite_number(x, "x"), finite_number(y, "y")
        if x <= 0.05 or not math.isfinite(math.hypot(x, y)):
            raise ValueError("goal must be finite and more than 0.05 m forward")
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
            raise ValueError("request_id must be a string of 1-64 characters")
        if type(epoch) is not int:
            raise ValueError("epoch must be an integer")
        with self.lock:
            if request_id in self.request_cache:
                previous = self.request_cache[request_id]
                if previous["relative_goal"] != [x, y]:
                    raise ValueError("request_id already used for a different goal")
                return previous
            if epoch != self.epoch:
                raise ValueError("stale goal epoch; refresh status")
            if self.goal is not None or self.state == "STOPPING":
                raise ValueError("stop/cancel the active goal first")
            reason = self._not_ready(self.clock())
            if reason:
                raise ValueError(reason)
            vx, wz = self.pose[3:]
            if abs(vx) + abs(wz) * self.engine.planner["wheel_separation_m"] / 2 > self.limits.stop_speed_mps:
                raise ValueError("wait for stopped wheel feedback before starting a new goal")
            self.goal = relative_goal_to_odom(x, y, *self.pose[:3])
            self.relative_goal = (x, y)
            self.goal_id = request_id
            self.epoch += 1
            self.last_distance = math.hypot(x, y)
            self.progress_at = self.clock()
            self.bypass_side = 0
            result = {"goal_id": request_id, "relative_goal": [x, y], "odom_goal": self.goal,
                      "epoch": self.epoch, "motion_enabled": self.motor.motion_enabled}
            self.request_cache[request_id] = result
            if len(self.request_cache) > 128:
                self.request_cache.popitem(last=False)
            self._state("DRIVING", "goal accepted")
            self._event("goal_accepted", **result)
            return result

    def watchdog(self):
        with self.lock:
            if self.goal is not None:
                reason = self._not_ready(self.clock())
                if reason:
                    self.stop(reason, "FAULT")

    def evaluate(self, frame, frame_seq):
        started = self.clock()
        with self.lock:
            self.frame_seq = frame_seq
            try:
                PreviewEngine._validate_frame(frame)
                if started - frame.dequeued_at > self.limits.input_timeout_s:
                    raise ValueError("depth input stale at evaluation")
                p = self.engine.planner
                points, unknown, fraction = depth_to_base_points(
                    frame.depth, SimpleNamespace(**frame.calibration),
                    self.engine.settings.camera_translation_m, self.engine.settings.camera_rotation_xyzw,
                    decimation=p["depth_decimation"], min_depth_m=p["depth_min_m"],
                    max_depth_m=p["depth_max_m"], unknown_range_m=p["obstacle_lookahead_m"],
                    missing_depth_blocks_paths=p["missing_depth_blocks_paths"],
                )
                if fraction < p["min_valid_depth_fraction"]:
                    raise ValueError(f"too little valid depth ({fraction:.1%})")
                self.frame_at = frame.dequeued_at
                considered = obstacle_points_in_view(points, p["obstacle_clearance_m"],
                                                      p["obstacle_min_height_m"], p["obstacle_max_height_m"],
                                                      p["obstacle_lookahead_m"])
                candidates, selected, remaining = [], None, None
                if self.state == "STARTING":
                    self._state("IDLE", "waiting for a goal")
                if self.goal is not None and self.state == "DRIVING":
                    reason = self._not_ready(started)
                    if reason:
                        self.stop(reason, "FAULT")
                    else:
                        dx, dy = self.goal[0] - self.pose[0], self.goal[1] - self.pose[1]
                        remaining = math.hypot(dx, dy)
                        heading = normalize_angle(math.atan2(dy, dx) - self.pose[2])
                        if remaining <= p["goal_tolerance_m"]:
                            self.stopped_count = 0
                            self._command(0.0, 0.0)
                            self._state("STOPPING", "inside goal tolerance; awaiting stopped feedback")
                        elif abs(heading) > math.pi / 2:
                            self.stop("goal behind rover; no reverse/pivot recovery", "BLOCKED")
                        elif started - self.progress_at > self.limits.progress_timeout_s:
                            self.stop("no goal progress within configured timeout", "BLOCKED")
                        else:
                            selected = choose_heading(
                                heading, remaining, points, rover_half_width=p["obstacle_clearance_m"],
                                obstacle_min_height=p["obstacle_min_height_m"],
                                obstacle_max_height=p["obstacle_max_height_m"], lookahead_m=p["obstacle_lookahead_m"],
                                unknown_points=unknown, steering=steering_settings(p), candidates_debug=candidates,
                            )
                            clear = [candidate for candidate in candidates if candidate["clear"]]
                            if selected is None:
                                self.stop("all local candidate paths blocked", "BLOCKED")
                            else:
                                preferred = [c for c in clear if (c["heading"] - heading) * self.bypass_side > 0]
                                if preferred and abs(selected - heading) > 0.1:
                                    selected = min(preferred, key=lambda c: abs(c["heading"] - heading))["heading"]
                                if abs(selected - heading) > 0.1 and not self.bypass_side:
                                    self.bypass_side = 1 if selected > heading else -1
                                linear, angular = command_for_heading(selected, **steering_settings(p))
                                half_track = p["wheel_separation_m"] / 2
                                left, right = linear - angular * half_track, linear + angular * half_track
                                reason = self._not_ready(self.clock())
                                if reason:
                                    self.stop(reason, "FAULT")
                                    selected = None
                                elif min(left, right) < self.limits.min_wheel_speed_mps:
                                    self.stop("planner proposed a sub-stall wheel command", "FAULT")
                                    selected = None
                                else:
                                    self._command(left, right)
                            if remaining < self.last_distance - self.limits.progress_distance_m:
                                self.last_distance, self.progress_at = remaining, started
                data = {
                    "seq": frame_seq, "frame_seq": frame_seq, "state": self.state, "reason": self.reason,
                    "sensor_sequence": frame.source_sequence,
                    "camera_metadata": frame.camera_metadata or {},
                    "goal_id": self.goal_id, "odom_goal": self.goal, "remaining_m": remaining,
                    "relative_goal": self.relative_goal,
                    "selected_heading_rad": selected, "candidates": candidates,
                    "valid_fraction": fraction, "unknown_points": len(unknown),
                    "unmeasured_fraction": float(1 - np.count_nonzero(
                        (frame.depth >= p["depth_min_m"] * 1000)
                        & (frame.depth <= p["depth_max_m"] * 1000)) / frame.depth.size),
                    "missing_depth_blocks_paths": p["missing_depth_blocks_paths"],
                    "sampled_points": bounded_points(points, self.engine.settings.max_debug_points),
                    "obstacle_points": bounded_points(considered, self.engine.settings.max_debug_points),
                    "processing_ms": (self.clock() - started) * 1000,
                    "camera_mount_verified": self.engine.settings.camera_mount_verified,
                }
                self.last_snapshot = Snapshot(data, frame)
            except (ValueError, TypeError, KeyError, AttributeError) as exc:
                LOG.error("Depth evaluation failed: %s", exc)
                self.frame_at = None
                self.stop(str(exc), "FAULT")
                self.last_snapshot = Snapshot({"seq": frame_seq, "frame_seq": frame_seq,
                                               "state": self.state, "reason": self.reason,
                                               "candidates": [], "selected_heading_rad": None}, frame)
            self.records.submit("decision", self.last_snapshot.data)
            return self.last_snapshot

    def status(self):
        with self.lock:
            now = self.clock()
            return {"state": self.state, "reason": self.reason, "epoch": self.epoch,
                    "goal_id": self.goal_id, "odom_goal": self.goal, "pose": self.pose,
                    "relative_goal": self.relative_goal,
                    "remaining_m": None if self.goal is None or self.pose is None else math.hypot(
                        self.goal[0] - self.pose[0], self.goal[1] - self.pose[1]),
                    "telemetry": self.telemetry, "requested_wheels": self.last_wheels,
                    "frame_seq": self.frame_seq, "command_seq": self.command_seq,
                    "frame_age_s": None if self.frame_at is None else now - self.frame_at,
                    "feedback_age_s": None if self.feedback_at is None else now - self.feedback_at,
                    "heartbeat_age_s": None if self.heartbeat_at is None else now - self.heartbeat_at,
                    "heartbeat_watchdog_enabled": self.heartbeat_watchdog_enabled,
                    "heartbeat_timeout_s": self.heartbeat_timeout_s,
                    "motion_enabled": self.motor.motion_enabled,
                    "camera_mount_verified": self.engine.settings.camera_mount_verified,
                    "motor": self.motor.status(), "logging": self.records.status()}
