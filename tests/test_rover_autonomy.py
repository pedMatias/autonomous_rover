from dataclasses import asdict, replace
import json
from pathlib import Path
import threading
import time
import zipfile
from types import SimpleNamespace

import numpy as np
import pytest

from rover_autonomy.controller import Controller, Limits, load_config
from rover_autonomy.motor import MockMotor, SerialMotor
from rover_autonomy.records import RunLog
from rover_autonomy.replay import ReplayRecords, replay_run
from rover_autonomy.runtime import Runtime
from rover_autonomy.server import create_app
from rover_preview.core import ROOT, PreviewEngine
from rover_preview.runtime import demo_source


class MemoryRecords:
    def __init__(self):
        self.events = []

    def submit(self, kind, payload, frame=None):
        self.events.append((kind, payload, frame))

    def status(self):
        return {"complete": True}


@pytest.fixture
def rig():
    settings, planner, calibration, agent, limits = load_config(ROOT / "config/rover_autonomy.yaml")
    clock = [time.monotonic()]
    records = MemoryRecords()
    motor = MockMotor(lambda kind, payload: records.submit(kind, payload))
    controller = Controller(PreviewEngine(settings, planner), calibration, motor, records, limits,
                            clock=lambda: clock[0])
    controller.feedback({"odl": 0, "odr": 0, "v": 1240})
    controller.heartbeat()
    frame = replace(demo_source("clear").read(0), dequeued_at=clock[0])
    controller.evaluate(frame, 1)
    return SimpleNamespace(controller=controller, motor=motor, clock=clock, records=records,
                           frame=frame, calibration=calibration, agent=agent)


def goal(rig, x=1.0, y=0.0, request_id="goal"):
    return rig.controller.accept_goal(x, y, request_id, rig.controller.epoch)


def test_goal_coordinates_and_idempotent_requests(rig):
    rig.controller.pose = (2.0, 3.0, np.pi / 2, 0.0, 0.0)
    result = goal(rig, 10, 1)
    assert result["odom_goal"] == pytest.approx((1, 13))
    assert rig.controller.accept_goal(10, 1, "goal", 0) is result
    with pytest.raises(ValueError, match="different"):
        rig.controller.accept_goal(10, 2, "goal", 0)
    with pytest.raises(ValueError, match="active"):
        goal(rig, request_id="new")
    rig.controller.stop()
    with pytest.raises(ValueError, match="stale"):
        rig.controller.accept_goal(1, 0, "other", 0)


def test_clear_goal_uses_forward_commands_and_records_decisions(rig):
    goal(rig)
    result = rig.controller.evaluate(rig.frame, 2)
    assert result.data["state"] == "DRIVING"
    assert len(result.data["candidates"]) == 7
    assert rig.motor.wheels == pytest.approx((.4, .4))
    assert any(kind == "decision" for kind, _, _ in rig.records.events)


@pytest.mark.parametrize("cause", ["depth", "feedback", "heartbeat"])
def test_watchdogs_stop_and_do_not_resume_after_heartbeat(rig, cause):
    goal(rig)
    rig.controller.evaluate(rig.frame, 2)
    rig.clock[0] += 3
    rig.controller.frame_at = rig.clock[0]
    rig.controller.feedback_at = rig.clock[0]
    now = rig.clock[0]
    rig.controller.frame_at = now if cause != "depth" else now - 3
    rig.controller.feedback_at = now if cause != "feedback" else now - 3
    rig.controller.heartbeat_at = now if cause != "heartbeat" else now - 3
    rig.controller.watchdog()
    assert rig.motor.wheels == (0, 0)
    assert rig.controller.goal is None
    rig.controller.heartbeat()
    assert rig.controller.state == "FAULT"
    assert rig.controller.goal is None


def test_gcs_heartbeat_watchdog_can_be_configured_or_explicitly_disabled(rig):
    goal(rig)
    rig.controller.evaluate(rig.frame, 2)
    rig.clock[0] += 3
    rig.controller.frame_at = rig.clock[0]
    rig.controller.feedback_at = rig.clock[0]
    rig.controller.configure_heartbeat(False, None)
    rig.controller.watchdog()
    assert rig.controller.goal is not None
    assert rig.controller.status()["heartbeat_watchdog_enabled"] is False
    with pytest.raises(ValueError, match="between 1 and 60"):
        rig.controller.configure_heartbeat(True, 0.5)
    rig.controller.configure_heartbeat(True, 1.0)
    assert rig.controller.goal is None
    assert rig.controller.state == "FAULT"
    assert rig.controller.reason == "GCS heartbeat expired"


@pytest.mark.parametrize("scene,state", [("empty", "FAULT"), ("wall", "BLOCKED")])
def test_missing_or_blocked_input_never_uses_search_prefix(rig, scene, state):
    goal(rig)
    result = rig.controller.evaluate(replace(demo_source(scene).read(0), dequeued_at=rig.clock[0]), 2)
    assert result.data["state"] == state
    assert rig.motor.wheels == (0, 0)
    assert result.data["selected_heading_rad"] is None


def test_completion_waits_for_stopped_feedback(rig):
    goal(rig)
    rig.controller.pose = (.9, 0, 0, .35, 0)
    rig.controller.odom.x = .9
    result = rig.controller.evaluate(rig.frame, 2)
    assert result.data["state"] == "STOPPING"
    assert rig.motor.wheels == (0, 0)
    for _ in range(3):
        rig.clock[0] += .02
        rig.controller.feedback({"odl": 0, "odr": 0, "v": 1240})
    assert rig.controller.state == "GOAL_REACHED"


def test_low_voltage_and_odometry_reset_stop(rig):
    goal(rig)
    rig.clock[0] += .02
    rig.controller.feedback({"odl": 0, "odr": 0, "v": 900})
    assert rig.controller.state == "FAULT"
    assert rig.motor.wheels == (0, 0)
    rig.clock[0] += 1
    rig.controller.feedback({"odl": 0, "odr": 0, "v": 1240})
    assert "continuity" in rig.controller.reason


def test_cancel_during_planning_cannot_restart_motion(rig, monkeypatch):
    import rover_autonomy.controller as module
    original = module.choose_heading
    entered, release = threading.Event(), threading.Event()
    def paused(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return original(*args, **kwargs)
    monkeypatch.setattr(module, "choose_heading", paused)
    goal(rig)
    planning = threading.Thread(target=rig.controller.evaluate, args=(rig.frame, 2))
    planning.start()
    assert entered.wait(2)
    stopping = threading.Thread(target=rig.controller.stop)
    stopping.start()
    release.set()
    planning.join(2)
    stopping.join(2)
    assert rig.motor.wheels == (0, 0)
    assert rig.controller.goal is None


def test_serial_handoff_is_latest_only_and_stop_supersedes(rig):
    motor = SerialMotor(rig.agent, lambda kind, payload: None)
    motor.submit(.35, .35, 1)
    motor.submit(.4, .35, 2)
    assert motor.pending[:3] == (.4, .35, 2)
    motor.submit(0, 0, 3)
    assert motor.pending[:3] == (0, 0, 3)
    with pytest.raises(ValueError, match="forward"):
        motor.submit(.05, .05, 4)
    with pytest.raises(ValueError, match="forward"):
        motor.submit(-.35, .35, 5)
    with pytest.raises(ValueError, match="0.35"):
        motor.submit(.34, .46, 6)


def test_serial_writer_purges_old_motion_and_expires_commands(rig):
    writes = []
    motor = SerialMotor({**rig.agent, "command_timeout_s": .05}, lambda kind, payload: None)
    motor.ser = SimpleNamespace(write=lambda data: writes.append(json.loads(data)) or len(data))
    motor.submit(.35, .35, 1)
    motor.submit(0, 0, 2)
    worker = threading.Thread(target=motor._writer, args=(lambda reason: pytest.fail(reason),), daemon=True)
    worker.start()
    try:
        assert motor.stop_written.wait(1)
        assert writes == [{"T": 1, "L": 0, "R": 0}]
        motor.submit(.35, .35, 3)
        until = time.monotonic() + 1
        while len(writes) < 3 and time.monotonic() < until:
            time.sleep(.01)
        assert writes[-2:] == [{"T": 1, "L": .35, "R": .35}, {"T": 1, "L": 0, "R": 0}]
    finally:
        motor.closed.set()
        with motor.condition:
            motor.condition.notify_all()
        worker.join(1)


def test_serial_startup_ownership_and_initial_zero_are_mocked(rig, monkeypatch):
    import sys
    import rover_autonomy.motor as motor_module

    writes = []
    class FakeSerial:
        in_waiting = 0

        def __init__(self, *args, **kwargs):
            assert kwargs["exclusive"] is True
            self.closed = False

        def fileno(self):
            return 123

        def write(self, payload):
            writes.append(json.loads(payload))
            return len(payload)

        def read(self, size):
            time.sleep(.01)
            return b""

        def close(self):
            self.closed = True

    monkeypatch.setattr(motor_module.subprocess, "run",
                        lambda *args, **kwargs: SimpleNamespace(returncode=1, stdout="", stderr=""))
    monkeypatch.setattr(motor_module.fcntl, "ioctl", lambda *args: 0)
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=FakeSerial))
    motor = SerialMotor(rig.agent, lambda kind, payload: None)
    motor.start(lambda data: None, lambda reason: pytest.fail(reason))
    try:
        assert writes[:4] == [
            {"T": 1, "L": 0, "R": 0}, {"T": 142, "cmd": 50},
            {"T": 131, "cmd": 1}, {"T": 143, "cmd": 0},
        ]
    finally:
        motor.close()
    assert writes[-1] == {"T": 1, "L": 0, "R": 0}


def test_serial_refuses_existing_uart_owner_before_opening(rig, monkeypatch):
    import sys
    import rover_autonomy.motor as motor_module
    from subprocess import CalledProcessError

    opened = []
    monkeypatch.setattr(motor_module.subprocess, "run",
                        lambda *args, **kwargs: SimpleNamespace(returncode=0, stdout="", stderr="123"))
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=lambda *args, **kwargs: opened.append(True)))
    motor = SerialMotor(rig.agent, lambda kind, payload: None)
    with pytest.raises(RuntimeError, match="123"):
        motor.start(lambda data: None, lambda reason: None)
    assert opened == []


def test_narrow_obstacle_selects_a_consistent_bypass_side(rig):
    goal(rig, 3, 0)
    depth = rig.frame.depth.copy()
    depth[70:140, 145:175] = 800
    frame = replace(rig.frame, depth=depth)
    result = rig.controller.evaluate(frame, 2)
    assert result.data["state"] == "DRIVING"
    assert abs(result.data["selected_heading_rad"]) > .1
    side = rig.controller.bypass_side
    assert side in (-1, 1)
    again = rig.controller.evaluate(frame, 3)
    assert again.data["selected_heading_rad"] * side > 0
    assert min(rig.motor.wheels) >= .35
    assert max(rig.motor.wheels) <= .5


def test_normal_launch_uses_live_hardware_and_demo_is_explicit():
    from rover_autonomy.__main__ import parse_args

    actual = parse_args([])
    assert actual.demo is None
    assert actual.mock_motors is False
    camera_test = parse_args(["--mock-motors"])
    assert camera_test.demo is None
    assert camera_test.mock_motors is True
    demo = parse_args(["--demo", "clear"])
    assert demo.demo == "clear"
    assert demo.mock_motors is True


def test_autonomy_tuning_keeps_every_wheel_above_stall_floor(rig):
    assert rig.controller.engine.planner["max_linear_speed_mps"] == .4
    assert rig.controller.engine.planner["min_wheel_speed_mps"] == .35
    goal(rig, 1, .8)
    frame = replace(rig.frame, source_sequence=77,
                    camera_metadata={"rgb_seq": 77, "depth_seq": 77, "rgb_depth_skew_ms": 0.0})
    result = rig.controller.evaluate(frame, 2)
    assert result.data["state"] == "DRIVING"
    assert .35 <= min(rig.motor.wheels) <= max(rig.motor.wheels) <= .5
    assert result.data["sensor_sequence"] == 77
    assert result.data["camera_metadata"]["depth_seq"] == 77


def test_gcs_requests_sequence_guard_and_origin(rig):
    runtime = Runtime(rig.controller, demo_source("clear"), rig.calibration)
    runtime.views.append((rig.controller.last_snapshot, b"jpeg"))
    client = create_app(runtime).test_client()
    headers = {"X-Rover-Autonomy": "1"}
    assert client.post("/api/goal", json={}).status_code == 403
    assert client.post("/api/heartbeat", headers=headers).status_code == 200
    configured = client.post("/api/heartbeat/configure", headers=headers,
                             json={"enabled": True, "timeout_s": 4.5})
    assert configured.status_code == 200
    assert configured.json == {"enabled": True, "timeout_s": 4.5}
    assert client.post("/api/heartbeat/configure", headers=headers,
                       json={"enabled": True, "timeout_s": 0.1}).status_code == 400
    disabled = client.post("/api/heartbeat/configure", headers=headers,
                           json={"enabled": False, "timeout_s": None})
    assert disabled.status_code == 200
    assert disabled.json == {"enabled": False, "timeout_s": None}
    accepted = client.post("/api/goal", headers=headers,
                           json={"x": 1, "y": 0, "request_id": "gcs", "epoch": 0})
    assert accepted.status_code == 202
    assert accepted.json["motion_enabled"] is False
    assert client.get("/api/frame.jpg?seq=1").headers["X-Rover-Sequence"] == "1"
    assert client.get("/api/frame.jpg?seq=0").status_code == 409
    assert client.post("/api/stop", headers=headers).status_code == 202
    assert rig.motor.wheels == (0, 0)
    assert client.post("/api/heartbeat", headers={**headers, "Origin": "http://evil.invalid"}).status_code == 403
    assert client.post("/api/goal", data="x" * 3000, content_type="application/json",
                       headers=headers).status_code == 413


def test_log_lossless_arrays_and_disk_failure_are_explicit(rig, tmp_path):
    log = RunLog(tmp_path, {}, 8, 1)
    log.submit("frame", {"frame_seq": 1}, rig.frame)
    log.submit("decision", {"frame_seq": 1, "selected_heading_rad": 0})
    log.close()
    assert log.status()["complete"]
    with np.load(log.directory / "frame-1.npz", allow_pickle=False) as stored:
        assert np.array_equal(stored["depth"], rig.frame.depth)
        assert np.array_equal(stored["rgb"], rig.frame.rgb)
    with zipfile.ZipFile(log.directory / "frame-1.npz") as archive:
        assert all(entry.compress_type == zipfile.ZIP_STORED for entry in archive.infolist())
    manifest = json.loads((log.directory / "manifest.json").read_text())
    assert manifest["written_events"] == 2
    failed = RunLog(tmp_path, {}, 8, 10 ** 30)
    failed.submit("frame", {"frame_seq": 2}, rig.frame)
    failed.close()
    assert failed.status()["complete"] is False
    assert "reserve" in failed.status()["error"]
    assert failed.status()["dropped_events"] == 1


def test_recorded_sequence_replays_with_mock_motors(tmp_path):
    settings, planner, calibration, agent, limits = load_config(ROOT / "config/rover_autonomy.yaml")
    clock = [time.monotonic()]
    log = RunLog(tmp_path, {"preview": asdict(settings), "planner": planner,
                           "calibration": calibration, "limits": asdict(limits)}, 64, 1)
    motor = MockMotor(lambda kind, payload: log.submit(kind, payload))
    controller = Controller(PreviewEngine(settings, planner), calibration, motor, log, limits,
                            clock=lambda: clock[0])
    frame = replace(demo_source("clear").read(0), dequeued_at=clock[0])
    log.submit("frame", {"frame_seq": 1}, frame)
    controller.feedback({"odl": 0, "odr": 0, "v": 1240})
    controller.heartbeat()
    controller.evaluate(frame, 1)
    controller.accept_goal(1, 0, "replay-goal", 0)
    log.submit("frame", {"frame_seq": 2}, frame)
    controller.evaluate(frame, 2)
    log.close()
    replay = replay_run(log.directory)
    assert replay["motor_mode"] == "mock"
    assert replay["decisions"] == 2
    assert replay["mismatches"] == 0
    assert replay["input_errors"] == 0
    (log.directory / "manifest.json").unlink()
    interrupted = replay_run(log.directory)
    assert interrupted["manifest"]["complete"] is False
    assert "missing" in interrupted["manifest"]["error"]


def test_runtime_mock_workers_and_bounded_view_cache(rig):
    rig.controller.clock = time.monotonic
    runtime = Runtime(rig.controller, demo_source("clear"), rig.calibration)
    runtime.start()
    try:
        runtime.view()
        until = time.monotonic() + 3
        while not runtime.views and time.monotonic() < until:
            time.sleep(.02)
        assert runtime.views
        assert all(thread.is_alive() for thread in runtime.threads)
        assert runtime.views.maxlen == 15
        assert runtime.status()["motion_enabled"] is False
    finally:
        rig.records.close = lambda: None
        runtime.close()


def test_stopped_outside_tolerance_is_not_reported_reached(rig):
    goal(rig)
    rig.controller.pose = (.9, 0, 0, .35, 0)
    rig.controller.odom.x = 1.3
    rig.controller.evaluate(rig.frame, 2)
    for _ in range(3):
        rig.clock[0] += .02
        rig.controller.feedback({"odl": 0, "odr": 0, "v": 1240})
    assert rig.controller.state == "BLOCKED"
    assert rig.motor.wheels == (0, 0)


def test_gcs_mock_goal_completes_and_run_replays(tmp_path):
    import urllib.request
    from werkzeug.serving import make_server

    settings, planner, calibration, _, limits = load_config(ROOT / "config/rover_autonomy.yaml")
    log = RunLog(tmp_path, {"preview": asdict(settings), "planner": planner,
                           "calibration": calibration, "limits": asdict(limits)}, 64, 1)
    motor = MockMotor(lambda kind, payload: log.submit(kind, payload))
    controller = Controller(PreviewEngine(settings, planner), calibration, motor, log, limits)
    runtime = Runtime(controller, demo_source("clear"), calibration)
    server = make_server("127.0.0.1", 0, create_app(runtime), threaded=True)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    runtime.start()
    base = f"http://127.0.0.1:{server.server_port}"
    runtime.view()

    def request(route, payload=None):
        headers = {"Content-Type": "application/json", "X-Rover-Autonomy": "1"}
        data = None if payload is None else json.dumps(payload).encode()
        with urllib.request.urlopen(urllib.request.Request(base + route, data=data, headers=headers), timeout=2) as response:
            return json.loads(response.read())

    try:
        until = time.monotonic() + 5
        while not runtime.views and time.monotonic() < until:
            time.sleep(.02)
        assert runtime.views
        request("/api/heartbeat", {})
        status = request("/api/status")
        accepted = request("/api/goal", {"x": 1, "y": 0, "request_id": "integrated", "epoch": status["epoch"]})
        assert accepted["motion_enabled"] is False
        until = time.monotonic() + 5
        while controller.state not in ("GOAL_REACHED", "BLOCKED", "FAULT") and time.monotonic() < until:
            request("/api/heartbeat", {})
            request("/api/view")
            time.sleep(.1)
        assert controller.state == "GOAL_REACHED", controller.reason
        assert controller.pose[0] == pytest.approx(1, abs=planner["goal_tolerance_m"])
    finally:
        server.shutdown()
        worker.join(2)
        server.server_close()
        runtime.close()
    assert log.status()["complete"]
    replay = replay_run(log.directory)
    assert replay["decisions"] > 5
    assert replay["mismatches"] == 0


@pytest.mark.parametrize("change", [{"control_hz": 0}, {"heartbeat_timeout_s": True}, {"log_queue_size": 1.5}])
def test_invalid_limits_rejected(change):
    with pytest.raises(ValueError):
        Limits(**change)
