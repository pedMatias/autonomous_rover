"""
Synchronized RGB-D + wheel-odometry recorder for the UGV Rover -> ROS2 bag2.

This is the Pi-side data-collection stage in the project plan in README.md.
Ties together:
  - recorder/oak_camera.py    OAK-D Lite RGB + aligned-depth capture
  - recorder/base_poller.py   ESP32 encoder feedback -> odometry integration
  - recorder/odometry.py      differential-drive dead-reckoning
  - recorder/bag_writer.py    rosbags-based bag2 writer

All bag writes happen on a single dedicated thread (a Queue feeds it) since
rosbags' Writer is not documented as thread-safe - the camera-read loop and
the base-feedback poller thread both just enqueue messages.

DRIVING: app.py (the localhost:5000 web GCS) and this recorder cannot run at
the same time - both need exclusive access to the same UART (/dev/ttyAMA0),
and sharing it would corrupt commands/feedback on both ends. So this script
owns the UART itself and provides a minimal keyboard teleop (see
teleop_thread_fn below) reusing the SAME BaseController connection used for
reading feedback.

The recorder uses only straight or diagonal wheel commands during data
collection. This keeps the recording procedure within the calibrated,
lower-load motion envelope; the production agent still supports the factory
in-place turn convention.

Teleop controls while recording (terminal must be interactive/a TTY):
    w = forward   s = backward   a = diagonal-left   d = diagonal-right
    space = stop  q = quit (stops driving, ends recording, saves bag)
    Auto-stops the rover if no key is pressed for 0.5s (safety watchdog).

Run with app.py STOPPED first (it holds the UART port):
    pkill -9 -f app.py
    /home/ws/ugv_rpi/ugv-env/bin/python recorder/record.py --out recordings/run1

Ctrl+C (or 'q') to stop cleanly (flushes and closes the bag).
"""
import argparse
import os
import queue
import select
import shutil
import subprocess
import sys
import termios
import threading
import time
import tty

import yaml

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.dirname(THIS_DIR)
CALIB_FILE = os.path.join(REPO_ROOT, "config", "calibration.yaml")

sys.path.insert(0, REPO_ROOT)
from base_ctrl import BaseController  # noqa: E402

from recorder.oak_camera import OakCamera  # noqa: E402
from recorder.odometry import OdometryIntegrator  # noqa: E402
from recorder.base_poller import BasePoller  # noqa: E402
from recorder.bag_writer import BagWriter  # noqa: E402


def check_uart_owners():
    owners = (("app.py", "factory app"), ("rover_agent.py", "production agent"))
    try:
        for pattern, description in owners:
            out = subprocess.run(["pgrep", "-af", pattern], capture_output=True, text=True)
            if out.stdout.strip():
                print(f"ERROR: {description} appears to be running and owns /dev/ttyAMA0.")
                print("Stop it first: sudo systemctl stop rover-camera.service rover-agent.service")
                print("or stop the factory process with: pkill -9 -f app.py")
                sys.exit(1)
    except FileNotFoundError:
        pass


def load_calibration():
    if not os.path.exists(CALIB_FILE):
        print(f"ERROR: no calibration file found at {CALIB_FILE}.")
        print("Run pedro_scripts/calibrate_odometry.py first (straight + arc modes).")
        sys.exit(1)
    with open(CALIB_FILE, "r") as fh:
        calib = yaml.safe_load(fh) or {}
    m_per_tick_left = calib.get("meters_per_tick_left", calib.get("meters_per_tick"))
    m_per_tick_right = calib.get("meters_per_tick_right", calib.get("meters_per_tick"))
    track_width = calib.get("track_width_m")
    if not m_per_tick_left or not m_per_tick_right or not track_width:
        print("ERROR: calibration file lacks left/right meters_per_tick or track_width_m.")
        sys.exit(1)
    return m_per_tick_left, m_per_tick_right, track_width


def dir_size_bytes(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            total += os.path.getsize(os.path.join(root, name))
    return total


def safe_drive(base, left, right):
    # Same guard as pedro_scripts/calibrate_odometry.py:safe_drive() - kept as
    # a local copy since that file is a standalone CLI script, not a module.
    # Refuses any opposite-sign (skid/spin) L/R pair, which can crash the rover.
    if left * right < 0:
        raise ValueError(
            f"REFUSING unsafe command L={left}, R={right}: opposite-sign wheel "
            f"speeds are an in-place/skid turn and can crash the rover."
        )
    base.base_speed_ctrl(left, right)


# key -> (left, right) given (drive_speed, turn_ratio)
TELEOP_KEYS = {
    "w": lambda speed, ratio: (speed, speed),
    "s": lambda speed, ratio: (-speed, -speed),
    "a": lambda speed, ratio: (speed * ratio, speed),
    "d": lambda speed, ratio: (speed, speed * ratio),
    " ": lambda speed, ratio: (0.0, 0.0),
}

TELEOP_STOP_TIMEOUT = 0.5  # auto-stop the rover if no key pressed for this long


def teleop_thread_fn(base, drive_speed, turn_ratio, stop_event, quit_event):
    # Reads single keypresses from stdin (cbreak mode) and drives the rover.
    # Runs on its own thread but sends commands through the SAME BaseController
    # connection the rest of the recorder uses for reading feedback - a serial
    # connection can send + receive independently, this is safe (app.py and
    # calibrate_odometry.py both do the same). Auto-stops the rover if no key
    # is pressed for TELEOP_STOP_TIMEOUT seconds, as a safety watchdog.
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_cmd = (0.0, 0.0)
    last_key_time = time.time()
    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                ch = sys.stdin.read(1)
                if ch == "q":
                    quit_event.set()
                    break
                cmd_fn = TELEOP_KEYS.get(ch)
                if cmd_fn is not None:
                    last_key_time = time.time()
                    left, right = cmd_fn(drive_speed, turn_ratio)
                    safe_drive(base, left, right)
                    last_cmd = (left, right)
            elif last_cmd != (0.0, 0.0) and (time.time() - last_key_time) > TELEOP_STOP_TIMEOUT:
                safe_drive(base, 0.0, 0.0)
                last_cmd = (0.0, 0.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        safe_drive(base, 0.0, 0.0)


def writer_thread_fn(bag_path, msg_queue, stop_event, counters):
    # The rosbags Writer wraps a sqlite3 connection, which sqlite3 forbids
    # using from a different thread than the one that created it - so the
    # BagWriter must be constructed, opened, written to, AND closed here,
    # all on this one thread. Do not pass an already-open BagWriter in.
    bag_writer = BagWriter(bag_path)
    bag_writer.open()
    try:
        while not (stop_event.is_set() and msg_queue.empty()):
            try:
                item = msg_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            kind, host_ts, payload = item
            if kind == "rgb":
                bag_writer.write_rgb(host_ts, payload)
                counters["frames"] += 1
            elif kind == "depth":
                bag_writer.write_depth(host_ts, payload)
            elif kind == "camera_info":
                bag_writer.write_camera_info(host_ts, payload)
            elif kind == "odom":
                x, y, theta, vx, vth = payload
                bag_writer.write_odometry(host_ts, x, y, theta, vx, vth)
                bag_writer.write_tf(host_ts, x, y, theta)
                counters["last_pose"] = (x, y, theta)
    finally:
        bag_writer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, help="Output bag directory (must not already exist)")
    parser.add_argument("--fps", type=float, default=10.0, help="Camera fps (default 10.0)")
    parser.add_argument("--rgb-width", type=int, default=640)
    parser.add_argument("--rgb-height", type=int, default=480)
    parser.add_argument("--warmup", type=float, default=8.0, help="Seconds of camera warmup before recording (default 8.0)")
    parser.add_argument("--duration", type=float, default=None, help="Optional recording duration in seconds (default: run until Ctrl+C)")
    parser.add_argument("--drive-speed", type=float, default=0.39, help="Teleop forward/backward speed in m/s (factory startup default 0.39)")
    parser.add_argument("--turn-ratio", type=float, default=0.153846, help="Teleop diagonal-turn inner-wheel ratio, 0-1 (factory startup default 0.2/1.3)")
    parser.add_argument("--no-teleop", action="store_true", help="Disable the built-in keyboard teleop (e.g. if driving via some other safe means)")
    args = parser.parse_args()

    if os.path.exists(args.out):
        print(f"ERROR: output path {args.out} already exists, refusing to overwrite.")
        sys.exit(1)

    check_uart_owners()
    m_per_tick_left, m_per_tick_right, track_width = load_calibration()
    print(
        f"Using calibration: left={m_per_tick_left:.6f}, "
        f"right={m_per_tick_right:.6f}, track_width_m={track_width:.4f}"
    )

    odometry = OdometryIntegrator(
        meters_per_tick_left=m_per_tick_left,
        meters_per_tick_right=m_per_tick_right,
        track_width_m=track_width,
    )

    print("Connecting to base controller...")
    base = BaseController("/dev/ttyAMA0", 115200)

    print("Starting OAK-D camera pipeline...")
    camera = OakCamera(rgb_size=(args.rgb_width, args.rgb_height), fps=args.fps)
    camera.start()
    print(f"Warming up camera for {args.warmup}s (depth density needs time to stabilize)...")
    camera.warmup(args.warmup)
    camera_info = camera.get_camera_info()

    print(f"Recording to bag at {args.out} ...")

    msg_queue = queue.Queue()
    stop_event = threading.Event()
    counters = {"frames": 0, "last_pose": (0.0, 0.0, 0.0)}

    writer_thread = threading.Thread(
        target=writer_thread_fn, args=(args.out, msg_queue, stop_event, counters), daemon=True
    )
    writer_thread.start()

    def on_odom_sample(host_ts, _raw_data, pose):
        msg_queue.put(("odom", host_ts, pose))

    poller = BasePoller(base, odometry, on_odom_sample)
    poller.enable_feedback_stream()
    poller.start()

    quit_event = threading.Event()
    teleop_thread = None
    if not args.no_teleop and sys.stdin.isatty():
        teleop_thread = threading.Thread(
            target=teleop_thread_fn,
            args=(base, args.drive_speed, args.turn_ratio, stop_event, quit_event),
            daemon=True,
        )
        teleop_thread.start()
        print("Teleop: w=forward s=backward a=diag-left d=diag-right space=stop q=quit")
        print(f"Teleop speed={args.drive_speed}m/s turn_ratio={args.turn_ratio} (auto-stops after {TELEOP_STOP_TIMEOUT}s idle)")
    else:
        print("Teleop disabled (no TTY or --no-teleop) - drive by some other safe means.")

    print("Recording started. Press Ctrl+C (or 'q') to stop.")
    start_time = time.time()
    last_status = 0
    frame_count = 0
    try:
        while True:
            if args.duration and (time.time() - start_time) > args.duration:
                print("Duration reached, stopping.")
                break
            if quit_event.is_set():
                print("Quit requested, stopping.")
                break

            result = camera.read(timeout=1.0)
            if result is None:
                continue
            host_ts, rgb_frame, depth_frame = result
            msg_queue.put(("rgb", host_ts, rgb_frame))
            msg_queue.put(("depth", host_ts, depth_frame))
            msg_queue.put(("camera_info", host_ts, camera_info))
            frame_count += 1

            now = time.time()
            if now - last_status > 1.0:
                elapsed = now - start_time
                fps = frame_count / elapsed if elapsed > 0.5 else 0.0
                x, y, theta = counters["last_pose"]
                size_mb = dir_size_bytes(args.out) / (1024 * 1024) if os.path.exists(args.out) else 0.0
                free_gb = shutil.disk_usage(REPO_ROOT).free / (1024 ** 3)
                print(
                    f"[{elapsed:6.1f}s] fps={fps:4.1f}  pose=(x={x:+.2f} y={y:+.2f} th={theta:+.2f}rad)  "
                    f"bag_size={size_mb:6.1f}MB  disk_free={free_gb:5.1f}GB  frames_queued={msg_queue.qsize()}"
                )
                last_status = now
    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C)...")
    finally:
        poller.stop()
        camera.stop()
        stop_event.set()
        safe_drive(base, 0.0, 0.0)
        if teleop_thread is not None:
            teleop_thread.join(timeout=2.0)
        writer_thread.join(timeout=10.0)
        print(f"Done. Recorded {frame_count} frame pairs to {args.out}")


if __name__ == "__main__":
    main()
