from dataclasses import replace
from decimal import Decimal
from html.parser import HTMLParser
import io
import json
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import pytest

from rover_preview.core import (
    ROOT, PreviewEngine, PreviewSettings, load_settings, validate_goal, validate_planner,
)
from rover_preview.runtime import (
    PreviewRuntime, StaticSource, demo_source, recording_bytes, render_jpeg, replay_source,
)
from rover_preview.server import create_app


@pytest.fixture
def engine():
    settings, planner = load_settings(ROOT / "config" / "rover_preview.yaml")
    return PreviewEngine(settings, planner)


def wait_for(predicate, timeout=3.0):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(0.01)
    pytest.fail("preview condition was not reached within the timeout")


def test_idle_exposes_obstacles_without_a_goal(engine):
    snapshot = engine.evaluate(demo_source("wall").read(0))
    assert snapshot.data["state"] == "IDLE"
    assert snapshot.data["considered_obstacle_points"] > 0
    assert snapshot.data["candidates"] == []
    assert snapshot.data["preview_only"] is True
    assert snapshot.data["motor_interface"] == "absent"


def test_goal_clear_blocked_and_recovery_use_existing_planner(engine):
    engine.set_goal((3.0, 1.0))
    clear = engine.evaluate(demo_source("clear").read(0))
    assert clear.data["state"] == "PREVIEW_CLEAR"
    assert len(clear.data["candidates"]) == 7
    selected = [
        candidate for candidate in clear.data["candidates"]
        if candidate["clear"] and candidate["heading"] == clear.data["selected_heading_rad"]
    ]
    assert selected
    for candidate in clear.data["candidates"]:
        assert candidate["linear"] >= 0.35
        assert candidate["linear"] - abs(candidate["angular"]) * engine.planner["wheel_separation_m"] / 2 > 0
    blocked = engine.evaluate(demo_source("wall").read(0))
    assert blocked.data["state"] == "PREVIEW_BLOCKED"
    assert all(not candidate["clear"] and candidate["blocked_point"] for candidate in blocked.data["candidates"])
    assert engine.frozen() is blocked
    again = engine.evaluate(demo_source("wall").read(0))
    assert again.data["seq"] > blocked.data["seq"]
    assert engine.frozen() is blocked
    assert engine.evaluate(demo_source("clear").read(0)).data["state"] == "PREVIEW_CLEAR"
    assert engine.frozen() is blocked
    engine.set_goal(None)
    assert engine.evaluate(demo_source("clear").read(0)).data["state"] == "IDLE"


def test_preview_search_is_a_prefix_with_no_commands(engine):
    engine.set_goal((3.0, 0.0))
    source = demo_source("wall")
    source.depth = np.full_like(source.depth, 900)
    snapshot = engine.evaluate(source.read(0))
    assert snapshot.data["state"] == "PREVIEW_SEARCH"
    step = snapshot.data["search_step"]
    assert 0.15 <= step["distance"] <= engine.planner["preview_search_max_step_m"]
    assert step["path"][-1] == step["point"]
    assert snapshot.data["selected_heading_rad"] is None


@pytest.mark.parametrize("x,y", [
    (float("nan"), 0), (float("inf"), 0), (True, 0), (0.05, 0),
    (-1, 0), (1, "left"), (10, 1), (None, 0),
])
def test_invalid_goals_are_explicitly_rejected(x, y):
    with pytest.raises(ValueError):
        validate_goal(x, y)


@pytest.mark.parametrize("change,reason", [
    ({"depth": np.zeros((240, 320), dtype=np.uint16)}, "too little valid depth"),
    ({"depth": np.ones((240, 320), dtype=np.float32)}, "uint16"),
    ({"rgb": np.zeros((120, 160, 3), dtype=np.uint8)}, "aligned"),
    ({"calibration": {"width": 1, "height": 1, "k": [1] * 9}}, "dimensions"),
    ({"calibration": {"width": 320, "height": 240, "k": [float("nan")] * 9}}, "calibration"),
    ({"calibration": {}}, "width"),
    ({"dequeued_at": time.monotonic() - 60}, "stale"),
])
def test_bad_sensor_input_exposes_fault_and_never_proposes_a_path(engine, change, reason):
    engine.set_goal((3.0, 0.0))
    frame = replace(demo_source("clear").read(0), **change)
    snapshot = engine.evaluate(frame)
    assert snapshot.data["state"] == "FAULT"
    assert reason in snapshot.data["reason"]
    assert snapshot.data["selected_heading_rad"] is None
    assert snapshot.data["candidates"] == []
    assert engine.frozen() is snapshot


def test_camera_to_base_projection_and_distance_are_not_object_counts(engine):
    snapshot = engine.evaluate(demo_source("wall").read(0))
    nearest = snapshot.data["nearest_obstacle_point"]
    assert nearest[0] == pytest.approx(.4)
    assert .04 <= nearest[2] <= .75
    assert snapshot.data["nearest_obstacle_range_m"] == pytest.approx(np.hypot(*nearest[:2]))
    assert "object_count" not in snapshot.data
    assert snapshot.data["camera_mount_verified"] is False


def test_debug_output_is_bounded_and_json_serializable(engine):
    settings = replace(engine.settings, max_debug_points=100)
    engine = PreviewEngine(settings, engine.planner)
    frame = demo_source("wall").read(0)
    engine.set_goal((3.0, 1.0))
    snapshot = engine.evaluate(frame)
    assert len(snapshot.data["sampled_points"]) <= 100
    assert len(snapshot.data["obstacle_points"]) <= 100
    assert len(json.dumps(snapshot.data, allow_nan=False).encode()) < 75000
    jpeg = render_jpeg(snapshot, engine)
    decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (240, 640, 3)


def test_old_goal_decision_and_fault_cannot_overwrite_new_goal(engine, monkeypatch):
    import rover_preview.core as core

    engine.set_goal((3.0, 0.0))
    original = core.depth_to_base_points

    def change_goal(*args, **kwargs):
        engine.set_goal((2.0, 1.0))
        return original(*args, **kwargs)

    monkeypatch.setattr(core, "depth_to_base_points", change_goal)
    assert engine.evaluate(demo_source("clear").read(0)) is None
    assert engine.latest().data["seq"] == 0

    def change_goal_and_fail(*args, **kwargs):
        engine.set_goal((2.0, -1.0))
        raise ValueError("old input")

    monkeypatch.setattr(core, "depth_to_base_points", change_goal_and_fail)
    assert engine.evaluate(demo_source("clear").read(0)) is None
    assert engine.latest().data["seq"] == 0


def test_snapshot_bundle_restores_exact_goal_calibration_and_settings(engine, tmp_path):
    engine.set_goal((3.0, 1.0))
    original = engine.evaluate(demo_source("wall").read(0))
    recording = tmp_path / "failure.npz"
    recording.write_bytes(recording_bytes(original, engine))
    source, settings, planner, goal = replay_source(recording)
    restored = PreviewEngine(settings, planner)
    restored.set_goal(goal)
    replayed = restored.evaluate(source.read(0))
    for key in ("state", "goal", "candidates", "nearest_obstacle_point",
                "selected_heading_rad", "valid_fraction", "collision_radius_m"):
        assert replayed.data[key] == original.data[key]
    assert "not live" in replayed.data["source"]
    assert replayed.data["captured_at"] == original.data["captured_at"]
    np.testing.assert_array_equal(replayed.frame.depth, original.frame.depth)
    app = create_app(PreviewRuntime(restored, source)).test_client()
    assert app.get("/api/config").json["recorded_decision"]["state"] == original.data["state"]


def test_plain_recorded_frame_needs_no_depthai_or_embedded_config(tmp_path):
    source = demo_source("clear")
    path = tmp_path / "frame.npz"
    np.savez(path, rgb=source.rgb, depth=source.depth, k=source.calibration["k"])
    replay, settings, planner, goal = replay_source(path)
    assert settings is planner is goal is None
    assert replay.read(0).depth.shape == (240, 320)
    assert replay.read(0).captured_at is None


def test_runtime_drops_old_visualization_and_does_not_encode_without_viewers(engine):
    runtime = PreviewRuntime(engine, demo_source("clear"))
    runtime.start()
    try:
        wait_for(lambda: runtime.health()["input_frames"] >= 4)
        assert runtime._encode_count == 0
        runtime.view()
        wait_for(lambda: len(runtime._views) > 0)
        for _ in range(6):
            runtime.view()
            time.sleep(.2)
        assert len(runtime._views) <= 3
        assert runtime._encode_count < runtime.health()["input_frames"]
        snapshot = runtime.view()
        assert runtime.jpeg(snapshot.data["seq"]) is not None
    finally:
        runtime.stop()
    assert not runtime.capture_thread.is_alive()
    assert not runtime.display_thread.is_alive()


def test_slow_renderer_never_holds_up_camera_planning(engine, monkeypatch):
    import rover_preview.runtime as module

    rendering, release = threading.Event(), threading.Event()
    original = module.render_jpeg

    def slow(snapshot, engine):
        rendering.set()
        assert release.wait(3)
        return original(snapshot, engine)

    monkeypatch.setattr(module, "render_jpeg", slow)
    runtime = PreviewRuntime(engine, demo_source("clear"))
    runtime.view()
    runtime.start()
    try:
        assert rendering.wait(2)
        previous = runtime.health()["input_frames"]
        wait_for(lambda: runtime.health()["input_frames"] >= previous + 3)
    finally:
        release.set()
        runtime.stop()


def test_timeout_camera_failure_and_recovery_are_visible(engine):
    class Source(StaticSource):
        available = True
        failed = False

        def read(self, timeout):
            if self.failed:
                raise RuntimeError("camera unplugged")
            return super().read(timeout) if self.available else None

    base = demo_source("clear")
    source = Source(base.rgb, base.depth, base.calibration, "test")
    runtime = PreviewRuntime(engine, source)
    runtime.start()
    try:
        wait_for(lambda: engine.latest().data["state"] == "IDLE")
        source.available = False
        wait_for(lambda: engine.latest().data["state"] == "FAULT")
        assert "timeout" in engine.latest().data["reason"]
        source.available = True
        wait_for(lambda: engine.latest().data["state"] == "IDLE")
        source.failed = True
        wait_for(lambda: not runtime.capture_thread.is_alive())
        assert "camera unplugged" in engine.latest().data["reason"]
        assert runtime.display_thread.is_alive()
    finally:
        runtime.stop()


@pytest.fixture
def client(engine):
    runtime = PreviewRuntime(engine, demo_source("wall"))
    app = create_app(runtime)
    app.testing = True
    return app.test_client(), runtime


def post_goal(client, value, **kwargs):
    return client.post("/api/goal", json=value, headers={"X-Rover-Preview": "1"}, **kwargs)


def test_http_dashboard_goal_api_and_no_motor_surface(client):
    browser, runtime = client
    assert browser.get("/").status_code == 200
    assert b"NO MOTOR INTERFACE" in browser.get("/").data
    assert browser.get("/static/dashboard.js").status_code == 200
    response = post_goal(browser, {"x": 3, "y": 1})
    assert response.status_code == 202
    assert response.json["preview_only"] is True
    runtime.engine.evaluate(runtime.source.read(0))
    assert browser.get("/api/snapshot").json["state"] == "PREVIEW_BLOCKED"
    assert browser.post("/send_command").status_code in (403, 404)
    assert browser.post("/api/cmd_vel", headers={"X-Rover-Preview": "1"}).status_code == 404
    assert browser.get("/api/config").json["preview"]["display_hz"] == 3
    assert browser.post("/api/goal/cancel", headers={"X-Rover-Preview": "1"}).status_code == 202
    assert runtime.engine.evaluate(runtime.source.read(0)).data["state"] == "IDLE"


def test_dashboard_default_goal_passes_native_number_input_constraints(client):
    class Inputs(HTMLParser):
        controls = []

        def handle_starttag(self, tag, attrs):
            values = dict(attrs)
            if tag == "input" and values.get("type") == "number":
                self.controls.append(values)

    browser, runtime = client
    parser = Inputs()
    parser.feed(browser.get("/").data.decode())
    assert len(parser.controls) == 2
    for control in parser.controls:
        value, lower, upper, step = (
            Decimal(control[name]) for name in ("value", "min", "max", "step")
        )
        assert lower <= value <= upper
        assert (value - lower) % step == 0


@pytest.mark.parametrize("value", [
    {"x": True, "y": 0}, {"x": float("nan"), "y": 0}, {"x": 3}, {"x": 0, "y": 0},
    {"x": 3, "y": 0, "vx": .4}, [3, 0], None,
])
def test_http_invalid_input_reports_error_without_acceptance(client, value):
    browser, runtime = client
    response = post_goal(browser, value)
    assert response.status_code in (400, 415)
    assert "error" in response.json
    assert runtime.engine._goal_revision == 0


def test_http_origin_payload_and_frame_sequence_guards(client):
    browser, runtime = client
    assert browser.post("/api/goal", json={"x": 3, "y": 0}).status_code == 403
    assert browser.post("/api/goal", json={"x": 3, "y": 0}, headers={
        "X-Rover-Preview": "1", "Origin": "http://untrusted.example",
    }).status_code == 403
    assert browser.post("/api/goal", data="{broken", content_type="application/json",
                        headers={"X-Rover-Preview": "1"}).status_code == 400
    assert browser.post("/api/goal", data="x" * 2000, content_type="application/json",
                        headers={"X-Rover-Preview": "1"}).status_code == 413
    assert browser.get("/api/frame.jpg").status_code == 400
    assert browser.get("/api/frame.jpg?seq=1234").status_code == 409
    assert browser.get("/api/frozen").status_code == 404
    assert browser.get("/api/frozen/download").status_code == 404


def test_http_frozen_image_and_replay_bundle_match_decision(client):
    browser, runtime = client
    post_goal(browser, {"x": 3, "y": 0})
    runtime.engine.evaluate(runtime.source.read(0))
    frozen = browser.get("/api/frozen")
    sequence = frozen.json["seq"]
    jpeg = browser.get(f"/api/frozen/frame.jpg?seq={sequence}")
    assert jpeg.status_code == 200
    assert jpeg.headers["X-Preview-Sequence"] == str(sequence)
    assert jpeg.mimetype == "image/jpeg"
    assert browser.get("/api/frozen/download?seq=999").status_code == 409
    download = browser.get(f"/api/frozen/download?seq={sequence}")
    assert download.status_code == 200
    with np.load(io.BytesIO(download.data), allow_pickle=False) as bundle:
        metadata = json.loads(str(bundle["metadata"].item()))
        expected = {key: value for key, value in frozen.json.items()
                    if key not in ("image_available", "has_camera_frame")}
        assert metadata["decision"] == expected
        np.testing.assert_array_equal(bundle["depth"], runtime.engine.frozen().frame.depth)


@pytest.mark.parametrize("changes", [
    {"camera_width": 641}, {"camera_rotation_xyzw": [0, 0, 0, 0]},
    {"display_hz": 100}, {"max_debug_points": True}, {"processing_hz": float("nan")},
])
def test_invalid_preview_configuration_is_rejected(changes):
    with pytest.raises(ValueError):
        PreviewSettings(**changes)


def test_invalid_planner_configuration_is_rejected(engine):
    for changes in (
        {"max_linear_speed_mps": .1}, {"depth_decimation": 1},
        {"missing_depth_blocks_paths": "false"}, {"depth_max_m": float("inf")},
        {"sensor_timeout_s": 0}, {"min_valid_depth_fraction": 0},
    ):
        with pytest.raises(ValueError):
            validate_planner({**engine.planner, **changes})


def test_preview_imports_neither_ros_nor_uart_packages():
    code = """
import importlib.abc
import sys
class NoHardware(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'rclpy', 'serial', 'base_ctrl', 'rover_agent', 'depthai'}:
            raise AssertionError('Forbidden preview import: ' + fullname)
sys.meta_path.insert(0, NoHardware())
from rover_preview.core import ROOT, load_settings, PreviewEngine
from rover_preview.runtime import demo_source, PreviewRuntime
from rover_preview.server import create_app
settings, planner = load_settings(ROOT / 'config' / 'rover_preview.yaml')
engine = PreviewEngine(settings, planner)
engine.set_goal((3, 0))
assert engine.evaluate(demo_source('wall').read(0)).data['state'] == 'PREVIEW_BLOCKED'
create_app(PreviewRuntime(engine, demo_source('clear')))
"""
    subprocess.run([sys.executable, "-c", code], check=True, cwd=ROOT, timeout=10)


@pytest.mark.parametrize("valid_count,expected_state", [(73, "FAULT"), (74, "PREVIEW_CLEAR")])
def test_minimum_valid_depth_threshold_is_checked_exactly(engine, valid_count, expected_state):
    source = demo_source("empty")
    sampled = source.depth[::8, ::8]
    roi = sampled[sampled.shape[0] // 4:, :]
    roi.flat[:valid_count] = 2000
    engine.set_goal((3, 0))
    snapshot = engine.evaluate(source.read(0))
    assert snapshot.data["valid_fraction"] == pytest.approx(valid_count / 920)
    assert snapshot.data["state"] == expected_state


def test_visualization_failure_is_reported_and_recovers_without_stopping_planner(engine, monkeypatch):
    import rover_preview.runtime as module

    failing = threading.Event()
    failing.set()
    original = module.render_jpeg

    def render(snapshot, engine):
        if failing.is_set():
            raise ValueError("bad visualization input")
        return original(snapshot, engine)

    monkeypatch.setattr(module, "render_jpeg", render)
    runtime = PreviewRuntime(engine, demo_source("clear"))
    runtime.view()
    runtime.start()
    try:
        wait_for(lambda: runtime.health()["visualization_error"] is not None)
        assert runtime.capture_thread.is_alive()
        assert runtime.display_thread.is_alive()
        failing.clear()
        wait_for(lambda: runtime.health()["visualization_error"] is None)
        wait_for(lambda: runtime._encode_count > 0)
    finally:
        runtime.stop()
