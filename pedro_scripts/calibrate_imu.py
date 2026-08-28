"""Capture stationary ESP32 gyro means and store them as calibration bias."""
import argparse
import os
import sys
import time

import yaml

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.dirname(THIS_DIR)
CALIB_FILE = os.path.join(REPO_ROOT, "config", "calibration.yaml")
sys.path.insert(0, REPO_ROOT)

from base_ctrl import BaseController  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()

    base = BaseController("/dev/ttyAMA0", 115200)
    base.base_json_ctrl({"T": 142, "cmd": 50})
    base.base_json_ctrl({"T": 131, "cmd": 1})
    base.base_json_ctrl({"T": 143, "cmd": 0})
    samples = []
    print(f"Leave the rover completely still for {args.duration:.1f} seconds")
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        data = base.feedback_data()
        if isinstance(data, dict) and all(key in data for key in ("gx", "gy", "gz")):
            samples.append({key: float(data[key]) for key in ("gx", "gy", "gz")})
    if not samples:
        raise RuntimeError("No gyro samples received")

    bias = {key: sum(sample[key] for sample in samples) / len(samples) for key in ("gx", "gy", "gz")}
    with open(CALIB_FILE, "r") as fh:
        calibration = yaml.safe_load(fh) or {}
    for axis in ("x", "y", "z"):
        calibration[f"gyro_bias_{axis}"] = bias[f"g{axis}"]
    calibration.pop("gyro_bias", None)
    with open(CALIB_FILE, "w") as fh:
        yaml.safe_dump(calibration, fh, sort_keys=False)
    print(f"Captured {len(samples)} samples")
    print(f"Gyro bias: {bias}")
    print(f"Saved to {CALIB_FILE}")


if __name__ == "__main__":
    main()