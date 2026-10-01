import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest


@pytest.fixture
def oak_module(monkeypatch):
    import sys

    pipeline = MagicMock()
    calibration = pipeline.getCalibrationData.return_value
    calibration.getCameraIntrinsics.return_value = [[100, 0, 160], [0, 100, 120], [0, 0, 1]]
    calibration.getDistortionCoefficients.return_value = [0] * 8
    sdk = SimpleNamespace(
        Pipeline=lambda: pipeline,
        node=SimpleNamespace(
            Camera=object(), StereoDepth=SimpleNamespace(
                PresetMode=SimpleNamespace(ROBOTICS=object()),
            ), Sync=object(),
        ),
        CameraBoardSocket=SimpleNamespace(CAM_A=0, CAM_B=1, CAM_C=2),
    )
    monkeypatch.setitem(sys.modules, "depthai", sdk)
    path = Path(__file__).resolve().parents[1] / "recorder" / "oak_camera.py"
    spec = importlib.util.spec_from_file_location("preview_test_oak", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, pipeline


@pytest.mark.parametrize("latest_only,options", [(False, {}), (True, {"maxSize": 1, "blocking": False})])
def test_bounded_preview_queue_does_not_change_default_recording_queue(oak_module, latest_only, options):
    module, pipeline = oak_module
    camera = module.OakCamera(rgb_size=(320, 240), latest_only=latest_only)
    camera.start()
    pipeline.create.return_value.out.createOutputQueue.assert_called_once_with(**options)
    assert camera.get_camera_info()["width"] == 320
    pipeline.start.assert_called_once()
    camera.stop()
    pipeline.stop.assert_called_once()


def test_camera_read_retains_existing_timestamp_and_array_contract(oak_module, monkeypatch):
    module, pipeline = oak_module
    camera = module.OakCamera()
    camera.sync_queue = MagicMock()
    camera.sync_queue.has.return_value = True
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    depth = np.ones((2, 3), dtype=np.uint16)
    group = {
        "rgb": SimpleNamespace(getCvFrame=lambda: rgb),
        "depth": SimpleNamespace(getFrame=lambda: depth),
    }
    camera.sync_queue.get.return_value = group
    monkeypatch.setattr(module.time, "time", lambda: 123.0)
    timestamp, actual_rgb, actual_depth = camera.read()
    assert timestamp == 123.0
    assert actual_rgb is rgb
    assert actual_depth is depth


def test_empty_camera_queue_yields_instead_of_busy_spinning(oak_module, monkeypatch):
    module, pipeline = oak_module
    camera = module.OakCamera()
    camera.sync_queue = MagicMock()
    camera.sync_queue.has.return_value = False
    ticks = iter([0.0, 0.0, 0.01, 0.02])
    monkeypatch.setattr(module.time, "monotonic", lambda: next(ticks))
    sleep = MagicMock()
    monkeypatch.setattr(module.time, "sleep", sleep)
    assert camera.read(timeout=.015) is None
    assert sleep.call_count == 2
    sleep.assert_called_with(.005)


def test_camera_warmup_also_yields_when_no_frames_arrive(oak_module, monkeypatch):
    module, pipeline = oak_module
    camera = module.OakCamera()
    camera.sync_queue = MagicMock()
    camera.sync_queue.has.return_value = False
    ticks = iter([0.0, 0.0, 0.1, 0.2])
    monkeypatch.setattr(module.time, "monotonic", lambda: next(ticks))
    sleep = MagicMock()
    monkeypatch.setattr(module.time, "sleep", sleep)
    camera.warmup(seconds=.15)
    assert sleep.call_count == 2
