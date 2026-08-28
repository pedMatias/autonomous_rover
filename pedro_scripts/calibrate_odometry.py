"""
Wheel/encoder odometry calibration for the UGV Rover, run ON THE RASPBERRY PI.

Derives the constants needed for dead-reckoning odometry integration:
    - meters_per_tick_left/right: linear distance per encoder count
  - track_width_m:   effective distance between the two wheel contact points

Motion types used here:
    1. Straight-line driving:  L == R (same speed, same direction)
    2. Diagonal-turn arcs:     L, R same sign, different magnitude

Room constraint: keep every run within ~2 meters of clear space (3m room).

Run with all other UART owners stopped first:
    sudo systemctl stop rover-camera.service rover-agent.service
    pkill -9 -f app.py
    /home/ws/ugv_rpi/ugv-env/bin/python pedro_scripts/calibrate_odometry.py straight
    /home/ws/ugv_rpi/ugv-env/bin/python pedro_scripts/calibrate_odometry.py arc

Usage:
    calibrate_odometry.py straight [--speed 0.39] [--duration 4.0]
    calibrate_odometry.py arc      [--left 0.06] [--right 0.39] [--duration 4.0]
    calibrate_odometry.py summary
"""
import argparse
import math
import os
import sys
import time

import yaml

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.dirname(THIS_DIR)
CALIB_FILE = os.path.join(REPO_ROOT, "config", "calibration.yaml")

sys.path.insert(0, REPO_ROOT)
from base_ctrl import BaseController  # noqa: E402


def load_calib():
    with open(CALIB_FILE, "r") as fh:
        data = yaml.safe_load(fh) or {}
    data.setdefault("meters_per_tick_left", data.get("meters_per_tick"))
    data.setdefault("meters_per_tick_right", data.get("meters_per_tick"))
    data.setdefault("track_width_m", None)
    data.setdefault("straight_trials", [])
    data.setdefault("arc_trials", [])
    return data


def save_calib(data):
    data["last_updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(CALIB_FILE, "w") as fh:
        yaml.safe_dump(data, fh, sort_keys=False)


def safe_drive(base, left, right):
    """Use straight or diagonal commands during controlled calibration runs."""
    if left * right < 0:
        raise ValueError(
            f"REFUSING unsafe command L={left}, R={right}: opposite-sign wheel "
            f"speeds are excluded from this calibration procedure."
        )
    base.base_speed_ctrl(left, right)


def wait_for_feedback(base, timeout=2.0):
    """Poll feedback_data() until odl/odr are present, or timeout."""
    start = time.time()
    data = None
    while time.time() - start < timeout:
        d = base.feedback_data()
        if d and "odl" in d and "odr" in d:
            data = d
            break
        time.sleep(0.02)
    return data


def make_base():
    base = BaseController("/dev/ttyAMA0", 115200)
    # Enable periodic feedback flow + reasonable interval, matching app.py's boot sequence.
    base.base_json_ctrl({"T": 142, "cmd": 50})
    base.base_json_ctrl({"T": 131, "cmd": 1})
    time.sleep(0.3)
    return base


def run_straight(args):
    print(f"=== Straight-line calibration trial: speed={args.speed} m/s, duration={args.duration}s ===")
    est_distance = args.speed * args.duration
    print(f"Estimated distance to travel: ~{est_distance:.2f} m")
    if est_distance > 2.0:
        print(f"WARNING: estimated distance {est_distance:.2f} m exceeds the 2 m budget for this room. Reduce --speed or --duration.")
        return
    input("Position the rover at your marked START point, clear path ahead, then press Enter to start...")

    base = make_base()
    try:
        start_data = wait_for_feedback(base)
        if not start_data:
            print("ERROR: no feedback received before starting. Aborting (is app.py still running? pkill -9 -f app.py).")
            return
        odl_start, odr_start = start_data["odl"], start_data["odr"]
        print(f"Start odl={odl_start} odr={odr_start}")

        t0 = time.time()
        while time.time() - t0 < args.duration:
            safe_drive(base, args.speed, args.speed)
            time.sleep(0.05)
    finally:
        safe_drive(base, 0, 0)

    time.sleep(0.3)
    end_data = wait_for_feedback(base)
    if not end_data:
        print("ERROR: no feedback received after stopping. Trial discarded.")
        return
    odl_end, odr_end = end_data["odl"], end_data["odr"]
    odl_delta = odl_end - odl_start
    odr_delta = odr_end - odr_start
    print(f"End odl={odl_end} odr={odr_end}")
    print(f"Delta ticks: odl={odl_delta}  odr={odr_delta}")

    measured = input("Measure the ACTUAL distance traveled with a tape measure (meters), e.g. 1.85: ").strip()
    try:
        measured_m = float(measured)
    except ValueError:
        print("Invalid number, trial discarded.")
        return

    if odl_delta <= 0 or odr_delta <= 0:
        print("ERROR: non-positive tick delta, trial discarded (did the rover actually move?).")
        return
    m_per_tick_left = measured_m / odl_delta
    m_per_tick_right = measured_m / odr_delta

    calib = load_calib()
    calib["straight_trials"].append({
        "speed": args.speed,
        "duration": args.duration,
        "odl_delta": odl_delta,
        "odr_delta": odr_delta,
        "measured_m": measured_m,
        "meters_per_tick_left": m_per_tick_left,
        "meters_per_tick_right": m_per_tick_right,
    })
    trials = calib["straight_trials"]
    calib["meters_per_tick_left"] = sum(t["meters_per_tick_left"] for t in trials) / len(trials)
    calib["meters_per_tick_right"] = sum(t["meters_per_tick_right"] for t in trials) / len(trials)
    save_calib(calib)

    print(f"Trial meters_per_tick_left = {m_per_tick_left:.6f}")
    print(f"Trial meters_per_tick_right = {m_per_tick_right:.6f}")
    print(
        f"Averaged scales over {len(trials)} trial(s): "
        f"left={calib['meters_per_tick_left']:.6f}, "
        f"right={calib['meters_per_tick_right']:.6f}"
    )
    print("Saved to", CALIB_FILE)


def run_arc(args):
    print(f"=== Diagonal-turn arc calibration trial: L={args.left} m/s, R={args.right} m/s, duration={args.duration}s ===")
    if args.left * args.right < 0:
        print("REFUSING: L and R must have the same sign (diagonal turn only, no in-place spins).")
        return
    est_distance = max(abs(args.left), abs(args.right)) * args.duration
    if est_distance > 2.0:
        print(f"WARNING: estimated max wheel travel {est_distance:.2f} m exceeds the 2 m budget for this room. Reduce speeds/duration.")
        return

    calib = load_calib()
    m_per_tick_left = calib.get("meters_per_tick_left", calib.get("meters_per_tick"))
    m_per_tick_right = calib.get("meters_per_tick_right", calib.get("meters_per_tick"))
    if not m_per_tick_left or not m_per_tick_right:
        print("ERROR: run the 'straight' calibration first (wheel scales are not set yet).")
        return

    input("Mark the rover's current heading (e.g. against a wall/floor line), clear path ahead, then press Enter to start...")

    base = make_base()
    try:
        start_data = wait_for_feedback(base)
        if not start_data:
            print("ERROR: no feedback received before starting. Aborting.")
            return
        odl_start, odr_start = start_data["odl"], start_data["odr"]

        t0 = time.time()
        while time.time() - t0 < args.duration:
            safe_drive(base, args.left, args.right)
            time.sleep(0.05)
    finally:
        safe_drive(base, 0, 0)

    time.sleep(0.3)
    end_data = wait_for_feedback(base)
    if not end_data:
        print("ERROR: no feedback received after stopping. Trial discarded.")
        return
    odl_end, odr_end = end_data["odl"], end_data["odr"]
    odl_delta = odl_end - odl_start
    odr_delta = odr_end - odr_start
    print(f"Delta ticks: odl={odl_delta}  odr={odr_delta}")

    angle_str = input(
        "Measure the ACTUAL heading change (degrees, positive = turned toward the faster wheel's opposite side), e.g. 35: "
    ).strip()
    try:
        angle_deg = float(angle_str)
    except ValueError:
        print("Invalid number, trial discarded.")
        return
    if angle_deg == 0:
        print("ERROR: zero heading change, cannot derive track width, trial discarded.")
        return

    theta_rad = math.radians(angle_deg)
    left_dist = odl_delta * m_per_tick_left
    right_dist = odr_delta * m_per_tick_right
    track_width = abs(right_dist - left_dist) / abs(theta_rad)

    calib["arc_trials"].append({
        "left_speed": args.left,
        "right_speed": args.right,
        "duration": args.duration,
        "odl_delta": odl_delta,
        "odr_delta": odr_delta,
        "angle_deg": angle_deg,
        "track_width_m": track_width,
    })
    trials = calib["arc_trials"]
    calib["track_width_m"] = sum(t["track_width_m"] for t in trials) / len(trials)
    save_calib(calib)

    print(f"Trial track_width_m = {track_width:.4f}")
    print(f"Averaged track_width_m over {len(trials)} trial(s) = {calib['track_width_m']:.4f}")
    print("Saved to", CALIB_FILE)


def run_summary(args):
    calib = load_calib()
    print(f"meters_per_tick_left = {calib.get('meters_per_tick_left')}")
    print(f"meters_per_tick_right = {calib.get('meters_per_tick_right')}")
    print(f"track_width_m   = {calib.get('track_width_m')}")
    print(f"straight_trials = {len(calib.get('straight_trials', []))}")
    print(f"arc_trials      = {len(calib.get('arc_trials', []))}")
    print(f"last_updated    = {calib.get('last_updated')}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)

    p_straight = sub.add_parser("straight", help="Straight-line meters_per_tick calibration")
    p_straight.add_argument("--speed", type=float, default=0.39, help="m/s, both wheels equal (factory startup default 0.39)")
    p_straight.add_argument("--duration", type=float, default=4.0, help="seconds to drive (default 4.0)")

    p_arc = sub.add_parser("arc", help="Diagonal-turn track_width calibration")
    p_arc.add_argument("--left", type=float, default=0.06, help="left wheel m/s (factory startup default 0.06)")
    p_arc.add_argument("--right", type=float, default=0.39, help="right wheel m/s (factory startup default 0.39)")
    p_arc.add_argument("--duration", type=float, default=4.0, help="seconds to drive (default 4.0)")

    sub.add_parser("summary", help="Print current calibration values")

    args = parser.parse_args()
    if args.mode == "straight":
        run_straight(args)
    elif args.mode == "arc":
        run_arc(args)
    elif args.mode == "summary":
        run_summary(args)


if __name__ == "__main__":
    main()
