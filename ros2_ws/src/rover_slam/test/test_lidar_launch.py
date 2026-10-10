import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch", reason="Run launch checks in ROS2 Jazzy")
pytest.importorskip("launch_ros", reason="Run launch checks in ROS2 Jazzy")

from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[1]


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "launch" / f"{name}.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("name", [
    "lidar", "live_bringup", "gcs", "lidar_mapping", "lidar_navigation",
])
def test_launch_description_constructs(name):
    assert load_launch(name).generate_launch_description().entities


@pytest.mark.parametrize("enabled,verified,valid", [
    ("false", "false", True), ("true", "false", False),
    ("true", "true", True), ("yes", "false", False),
])
def test_navigation_arming_is_explicit(enabled, verified, valid):
    context = LaunchContext()
    context.launch_configurations.update(
        motion_enabled=enabled, hardware_verified=verified
    )
    validate = load_launch("lidar_navigation").validate_arming
    if valid:
        assert validate(context) == []
    else:
        with pytest.raises(ValueError):
            validate(context)


def test_mounting_transform_cannot_be_missing_or_nonfinite():
    context = LaunchContext()
    context.launch_configurations.update({
        f"laser_{axis}": "0"
        for axis in ("x", "y", "z", "roll", "pitch", "yaw")
    })
    validate = load_launch("lidar").validate_mount
    assert validate(context) == []
    context.launch_configurations["laser_z"] = ""
    with pytest.raises(ValueError):
        validate(context)
    context.launch_configurations["laser_z"] = "nan"
    with pytest.raises(ValueError):
        validate(context)
