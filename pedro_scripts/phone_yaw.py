#!/usr/bin/env python3
"""Heading change from a phone gyroscope recording (phyphox "Gyroscope" CSV).

Companion to pedro_scripts/calibrate_odometry.py: the `arc` subcommand drives
the rover and then asks for the ACTUAL heading change in degrees. This script
turns a phone gyroscope recording into that number.

Workflow:
    1. Mount the phone flat (screen up) and rigid on the rover chassis.
    2. Record 10-30 s stationary in phyphox, export CSV.
    3. Run `./ugv-env/bin/python pedro_scripts/calibrate_odometry.py arc ...`
       while recording gyro on the phone, export one CSV per run.
    4. ./ugv-env/bin/python pedro_scripts/phone_yaw.py bias stationary.csv
    5. ./ugv-env/bin/python pedro_scripts/phone_yaw.py yaw \
           --bias stationary.csv arc1.csv arc2.csv arc3.csv
    6. Type the printed degrees into the `arc` prompt.

Sign convention matches the arc prompt ("positive = turned toward the faster
wheel's opposite side"): phone flat and screen-up, a left turn (faster right
wheel, e.g. arc --left 0.06 --right 0.39) integrates to a POSITIVE angle.

The bias measured here is the PHONE's and only de-biases phone recordings.
The rover's own gyro_bias_x/y/z in config/calibration.yaml come from the
ESP32 IMU via pedro_scripts/calibrate_imu.py - do not mix them.
"""
import argparse
import csv
import math
import sys


def load_gyro_csv(path):
    """Return (t, gz) in seconds and rad/s from a phyphox/Sensor Logger CSV."""
    with open(path, newline="") as fh:
        rows = [r for r in csv.reader(fh) if r]
    header = [c.strip().lower() for c in rows[0]]
    t_col = next((i for i, c in enumerate(header) if "time" in c), None)
    if t_col is None:
        sys.exit(f"error: {path}: no time column in {rows[0]}")
    z_col = next((i for i, c in enumerate(header) if "gyro" in c and "z" in c), None)
    if z_col is None:
        z_col = next((i for i, c in enumerate(header) if c == "z" or c.startswith("z ")), None)
    if z_col is None:
        sys.exit(f"error: {path}: no gyroscope z column in {rows[0]}")
    t, gz = [], []
    for row in rows[1:]:
        try:
            t.append(float(row[t_col]))
            gz.append(float(row[z_col]))
        except (ValueError, IndexError):
            continue
    if len(t) < 10:
        sys.exit(f"error: {path}: only {len(t)} samples parsed")
    if t[0] > t[-1]:
        t.reverse()
        gz.reverse()
    return t, gz


def mean(xs):
    return sum(xs) / len(xs)


def std(xs):
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / max(len(xs) - 1, 1))


def integrate_yaw(t, gz, bias):
    """Trapezoid integration of bias-corrected gyro_z over the whole clip."""
    return sum(
        0.5 * (gz[i] + gz[i - 1] - 2.0 * bias) * (t[i] - t[i - 1])
        for i in range(1, len(t))
    )


def cmd_bias(args):
    t, gz = load_gyro_csv(args.csv)
    print(f"file: {args.csv}  ({len(t)} samples, {t[-1] - t[0]:.1f} s)")
    print(f"phone gyro_z bias: {mean(gz):+.6f} rad/s  (std {std(gz):.6f})")
    print("pass this file via `yaw --bias` to de-bias arc recordings")


def cmd_yaw(args):
    bias = None
    if args.bias:
        bt, bgz = load_gyro_csv(args.bias)
        bias = mean(bgz)
        print(f"bias from {args.bias}: {bias:+.6f} rad/s "
              f"(std {std(bgz):.6f}, {bt[-1] - bt[0]:.1f} s)\n")

    angles = []
    for path in args.csv:
        t, gz = load_gyro_csv(path)
        b = bias
        if b is None:
            head = [g for g, ti in zip(gz, t) if ti - t[0] <= args.bias_secs]
            b = mean(head) if len(head) >= 5 else mean(gz)
        deg = math.degrees(integrate_yaw(t, gz, b))
        peak = max(abs(g - b) for g in gz)
        angles.append(deg)
        print(f"{path}: {deg:+.2f} deg  ({t[-1] - t[0]:.1f} s, "
              f"{len(t)} samples, peak |gz| {peak:.2f} rad/s)")
        if abs(deg) < 1.0:
            print("  warning: near-zero heading - "
                  "recording may have missed the maneuver")

    print()
    if len(angles) > 1:
        m, s = mean(angles), std(angles)
        print(f"mean heading change: {m:+.2f} deg (std {s:.2f}, n={len(angles)})")
        print(f"-> type {m:+.2f} at the calibrate_odometry.py arc prompt")
    else:
        print(f"-> type {angles[0]:+.2f} at the calibrate_odometry.py arc prompt")
    print("(batch only same-direction arcs; opposite arcs read as negative)")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    p_bias = sub.add_parser("bias", help="Stationary phone gyro_z bias (rad/s)")
    p_bias.add_argument("csv", help="phyphox gyroscope CSV, phone mounted and still")
    p_bias.set_defaults(fn=cmd_bias)

    p_yaw = sub.add_parser("yaw", help="Heading change (deg) for arc trial CSVs")
    p_yaw.add_argument("csv", nargs="+",
                       help="phyphox gyroscope CSV(s), one per arc run, same turn direction")
    p_yaw.add_argument("--bias",
                       help="stationary CSV for bias; default: first --bias-secs of each file")
    p_yaw.add_argument("--bias-secs", type=float, default=2.0,
                       help="seconds of still lead-in used as bias when no --bias file (default 2.0)")
    p_yaw.set_defaults(fn=cmd_yaw)

    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
