"""Verify the ESP32 UART feedback stream, with an explicit optional drive test."""
import argparse
import json
import os
import sys
import time

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.dirname(THIS_DIR)
sys.path.insert(0, REPO_ROOT)

from base_ctrl import BaseController  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive", action="store_true", help="Drive forward briefly before stopping")
    parser.add_argument("--speed", type=float, default=0.39)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--log", default=os.path.expanduser("~/rover/logs/esp32-link.jsonl"))
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    base = BaseController("/dev/ttyAMA0", 115200)
    base.base_json_ctrl({"T": 142, "cmd": 50})
    base.base_json_ctrl({"T": 131, "cmd": 1})
    base.base_json_ctrl({"T": 143, "cmd": 0})

    frames = 0
    started = time.monotonic()
    try:
        if args.drive:
            print(f"Driving at {args.speed:.3f} m/s for {args.duration:.1f} s")
            base.base_speed_ctrl(args.speed, args.speed)
            drive_until = time.monotonic() + args.duration
        else:
            print("Read-only link check; no motor command will be sent")
            drive_until = time.monotonic()

        with open(args.log, "a") as log_file:
            while time.monotonic() - started < args.duration + 2.0:
                if args.drive and time.monotonic() >= drive_until:
                    base.base_speed_ctrl(0.0, 0.0)
                    args.drive = False
                data = base.feedback_data()
                if not isinstance(data, dict):
                    continue
                if "odl" in data and "odr" in data:
                    frames += 1
                log_file.write(json.dumps({"ts": time.time(), "data": data}) + "\n")
                log_file.flush()
                print(json.dumps(data, separators=(",", ":")))
                if not args.drive and time.monotonic() - started >= args.duration:
                    break
    finally:
        base.base_speed_ctrl(0.0, 0.0)

    elapsed = max(time.monotonic() - started, 0.001)
    print(f"Feedback frames with encoder data: {frames} ({frames / elapsed:.1f} Hz)")
    print(f"Saved raw feedback to {args.log}")


if __name__ == "__main__":
    main()