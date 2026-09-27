"""Keyboard teleop for the rover, running inside the ROS2 graph.

Reads ``w`` / ``a`` / ``s`` / ``d`` from an interactive terminal and publishes
``geometry_msgs/msg/Twist`` to ``/cmd_vel``, which ``rover_bridge`` forwards to
the Pi as the ``vx`` / ``wz`` command protocol.

Motion envelope
---------------
``rover_agent.twist_to_wheels()`` converts a Twist to wheel speeds as::

    left  = vx - (wz * wheel_separation / 2.0)
    right = vx + (wz * wheel_separation / 2.0)

so ``wz != 0`` with ``vx == 0`` is an opposite-sign pair, i.e. an in-place skid
turn, while ``wz != 0`` with ``vx > 0`` is a same-sign pair, i.e. a diagonal
arc. This node only ever produces same-sign pairs: the wheel pair is chosen
first (reusing the audited ``recorder/record.py`` teleop convention), then
inverted back into a Twist. Holding ``a`` or ``d`` without ``w`` / ``s`` creeps
forward at ``turn_speed_mps`` instead of spinning in place, so an in-place turn
is unreachable by construction. A final guard refuses to publish any pair that
still somehow comes out opposite-sign.

Controls (lower-case keys; the terminal has no key-release events)
------------------------------------------------------------------
``w`` forward, ``s`` reverse, ``a`` diagonal left, ``d`` diagonal right,
``w``+``a`` / ``w``+``d`` / ``s``+``a`` / ``s``+``d`` combined, ``space`` or
``x`` stop, ``q`` or ``Ctrl+C`` quit (always zeroing ``/cmd_vel`` first).

A key counts as held while it was seen within ``key_window_s``; the terminal's
key-repeat keeps a held key alive, and a released key simply ages out. Motion
therefore stops by itself if the keyboard loop stalls, and ``rover_bridge``'s
``cmd_vel_timeout_s`` (0.5 s) is the second safety net.

Run it in its own terminal, never piped or under ``ros2 launch``::

    ros2 run rover_bridge rover_teleop
    ros2 run rover_bridge rover_teleop --dry-run
    ros2 run rover_bridge rover_teleop --ros-args -p drive_speed_mps:=0.20
"""
from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import threading
import time
import tty
from typing import NamedTuple

FORWARD_KEY = "w"
REVERSE_KEY = "s"
LEFT_KEY = "a"
RIGHT_KEY = "d"
DRIVE_KEYS = frozenset({FORWARD_KEY, REVERSE_KEY, LEFT_KEY, RIGHT_KEY})
STOP_KEYS = frozenset({" ", "x"})
QUIT_KEYS = frozenset({"q", "\x03"})

ESCAPE_INTRO = "\x1b"
ESCAPE_CONTINUATION = frozenset("[0123456789;")

# Escape sequences (arrow keys and friends) end with a letter, so they must be
# consumed before lower-casing anything: an up arrow is "\x1b[A".
NO_STOP_REASON = "stop"


class ParamSpec(NamedTuple):
    """One tunable, shared by the CLI flags, the ROS parameters, and --dry-run."""

    name: str
    flag: str
    default: object
    caster: type
    help: str


PARAM_SPECS = (
    ParamSpec(
        "cmd_vel_topic", "--cmd-vel-topic", "/cmd_vel", str,
        "Twist topic published by this node.",
    ),
    ParamSpec(
        "drive_speed_mps", "--drive-speed", 0.39, float,
        "Full-wheel speed for w/s in m/s (factory startup default 0.39).",
    ),
    ParamSpec(
        "turn_speed_mps", "--turn-speed", 0.20, float,
        "Forward creep used when a/d is held alone in m/s; keeps every a/d "
        "command a same-sign diagonal instead of an in-place spin.",
    ),
    ParamSpec(
        "turn_ratio", "--turn-ratio", 0.153846, float,
        "Inner-wheel ratio for a/d, strictly between 0 and 1 (1 = straight). "
        "Radius = (1 + ratio) / (2 * (1 - ratio)) * track width.",
    ),
    ParamSpec(
        "key_window_s", "--key-window", 0.7, float,
        "Seconds after which an unrepeated key stops counting as held. Keep it "
        "above the OS key-repeat delay.",
    ),
    ParamSpec(
        "publish_rate_hz", "--rate", 20.0, float,
        "cmd_vel publish rate in Hz; rover_bridge zeroes commands after 0.5 s.",
    ),
    ParamSpec(
        "wheel_separation_m", "--wheel-separation-m", 0.172, float,
        "Track width used to invert the Pi's twist_to_wheels conversion.",
    ),
    ParamSpec(
        "max_wheel_speed_mps", "--max-wheel-speed-mps", 0.5, float,
        "Per-wheel speed cap; a larger peak is scaled down, as on the Pi.",
    ),
)


class WheelCommand(NamedTuple):
    """A resolved teleop command in both wheel and Twist form."""

    left: float
    right: float
    vx: float
    wz: float
    safe: bool
    reason: str


def _finite_positive(name: str, value: float) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def validate_motion_parameters(
    drive_speed_mps: float,
    turn_speed_mps: float,
    turn_ratio: float,
    key_window_s: float,
    publish_rate_hz: float,
    wheel_separation_m: float,
    max_wheel_speed_mps: float,
) -> None:
    _finite_positive("drive_speed_mps", drive_speed_mps)
    _finite_positive("turn_speed_mps", turn_speed_mps)
    _finite_positive("key_window_s", key_window_s)
    _finite_positive("publish_rate_hz", publish_rate_hz)
    _finite_positive("wheel_separation_m", wheel_separation_m)
    _finite_positive("max_wheel_speed_mps", max_wheel_speed_mps)
    if not math.isfinite(turn_ratio) or not 0.0 < turn_ratio < 1.0:
        raise ValueError("turn_ratio must be finite and strictly between 0 and 1")


def active_keys(last_seen: dict[str, float], now: float, window_s: float) -> frozenset[str]:
    """Return keys whose terminal repeat has arrived within the hold window."""
    if not math.isfinite(now) or not math.isfinite(window_s) or window_s <= 0.0:
        return frozenset()
    return frozenset(
        key for key, timestamp in last_seen.items()
        if key in DRIVE_KEYS and math.isfinite(timestamp) and 0.0 <= now - timestamp <= window_s
    )


def wheels_to_twist(left: float, right: float, wheel_separation_m: float) -> tuple[float, float]:
    """Invert the Pi's differential-drive conversion exactly."""
    _finite_positive("wheel_separation_m", wheel_separation_m)
    if not math.isfinite(left) or not math.isfinite(right):
        raise ValueError("wheel speeds must be finite")
    return (left + right) / 2.0, (right - left) / wheel_separation_m


def _scale_wheels(left: float, right: float, max_wheel_speed_mps: float) -> tuple[float, float]:
    peak = max(abs(left), abs(right))
    if peak > max_wheel_speed_mps:
        scale = max_wheel_speed_mps / peak
        return left * scale, right * scale
    return left, right


def resolve_wheel_command(
    keys: set[str] | frozenset[str],
    drive_speed_mps: float = 0.39,
    turn_speed_mps: float = 0.20,
    turn_ratio: float = 0.153846,
    wheel_separation_m: float = 0.172,
    max_wheel_speed_mps: float = 0.5,
) -> WheelCommand:
    """Resolve held keys into a safe same-sign wheel command and Twist.

    The wheel command is selected before converting to Twist so the safety
    property is explicit and testable. ``a``/``d`` alone use positive
    ``turn_speed_mps``; they never create a zero-linear-velocity spin.
    """
    validate_motion_parameters(
        drive_speed_mps,
        turn_speed_mps,
        turn_ratio,
        1.0,
        1.0,
        wheel_separation_m,
        max_wheel_speed_mps,
    )
    keys = frozenset(keys) & DRIVE_KEYS
    if FORWARD_KEY in keys and REVERSE_KEY in keys:
        return WheelCommand(0.0, 0.0, 0.0, 0.0, True, "conflicting direction keys")
    if LEFT_KEY in keys and RIGHT_KEY in keys:
        return WheelCommand(0.0, 0.0, 0.0, 0.0, True, "conflicting turn keys")
    if not keys:
        return WheelCommand(0.0, 0.0, 0.0, 0.0, True, NO_STOP_REASON)

    reverse = REVERSE_KEY in keys
    direction = -1.0 if reverse else 1.0
    speed = drive_speed_mps if FORWARD_KEY in keys or reverse else turn_speed_mps
    speed *= direction
    left_turn = LEFT_KEY in keys
    right_turn = RIGHT_KEY in keys
    left = speed
    right = speed
    if left_turn:
        if direction > 0.0:
            left *= turn_ratio
        else:
            right *= turn_ratio
    elif right_turn:
        if direction > 0.0:
            right *= turn_ratio
        else:
            left *= turn_ratio

    left, right = _scale_wheels(left, right, max_wheel_speed_mps)
    if left * right < -1e-12:
        return WheelCommand(0.0, 0.0, 0.0, 0.0, False, "opposite-sign wheel command refused")
    vx, wz = wheels_to_twist(left, right, wheel_separation_m)
    return WheelCommand(left, right, vx, wz, True, "diagonal" if left_turn or right_turn else "straight")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Safe w/a/s/d diagonal keyboard teleop -> ROS2 /cmd_vel"
    )
    for spec in PARAM_SPECS:
        parser.add_argument(spec.flag, dest=spec.name, type=spec.caster, default=spec.default, help=spec.help)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print representative commands and exit without ROS2 or a TTY",
    )
    return parser


def parse_cli(argv=None):
    """Parse local flags and leave ROS2's ``--ros-args`` for rclpy."""
    return _parser().parse_known_args(argv)


def _format_command(label: str, command: WheelCommand) -> str:
    return (
        f"{label:8s} L={command.left:+.3f} R={command.right:+.3f} "
        f"vx={command.vx:+.3f} wz={command.wz:+.3f} ({command.reason})"
    )


def run_dry_run(args) -> int:
    validate_motion_parameters(
        args.drive_speed_mps,
        args.turn_speed_mps,
        args.turn_ratio,
        args.key_window_s,
        args.publish_rate_hz,
        args.wheel_separation_m,
        args.max_wheel_speed_mps,
    )
    samples = (
        ("none", frozenset()),
        ("w", frozenset({FORWARD_KEY})),
        ("s", frozenset({REVERSE_KEY})),
        ("a", frozenset({LEFT_KEY})),
        ("d", frozenset({RIGHT_KEY})),
        ("w+a", frozenset({FORWARD_KEY, LEFT_KEY})),
        ("w+d", frozenset({FORWARD_KEY, RIGHT_KEY})),
        ("s+a", frozenset({REVERSE_KEY, LEFT_KEY})),
        ("s+d", frozenset({REVERSE_KEY, RIGHT_KEY})),
    )
    print("Keyboard teleop dry run (opposite-sign/in-place turns are refused):")
    for label, keys in samples:
        print(_format_command(label, resolve_wheel_command(
            keys,
            args.drive_speed_mps,
            args.turn_speed_mps,
            args.turn_ratio,
            args.wheel_separation_m,
            args.max_wheel_speed_mps,
        )))
    return 0

class KeyboardTeleopNode:
    """ROS2 wrapper around the pure keyboard command resolver."""

    def __init__(self, rclpy, Node, Twist, args):
        self._rclpy = rclpy
        self._node = Node("rover_keyboard_teleop")
        self._Twist = Twist
        self._stop_event = threading.Event()
        self._quit_event = threading.Event()
        self._input_thread = None
        self._last_seen: dict[str, float] = {}
        self._key_lock = threading.Lock()

        for spec in PARAM_SPECS:
            self._node.declare_parameter(spec.name, getattr(args, spec.name))
        self._topic_name = str(self._node.get_parameter("cmd_vel_topic").value)
        self._drive_speed = float(self._node.get_parameter("drive_speed_mps").value)
        self._turn_speed = float(self._node.get_parameter("turn_speed_mps").value)
        self._turn_ratio = float(self._node.get_parameter("turn_ratio").value)
        self._key_window = float(self._node.get_parameter("key_window_s").value)
        self._rate = float(self._node.get_parameter("publish_rate_hz").value)
        self._wheel_separation = float(self._node.get_parameter("wheel_separation_m").value)
        self._max_wheel_speed = float(self._node.get_parameter("max_wheel_speed_mps").value)
        validate_motion_parameters(
            self._drive_speed, self._turn_speed, self._turn_ratio,
            self._key_window, self._rate, self._wheel_separation,
            self._max_wheel_speed,
        )
        self._publisher = self._node.create_publisher(Twist, self._topic_name, 10)
        self._timer = self._node.create_timer(1.0 / self._rate, self._publish_current)
        self._node.get_logger().info(
            f"Keyboard teleop on {self._topic_name}: w/s={self._drive_speed:.3f} m/s, "
            f"turn creep={self._turn_speed:.3f} m/s, ratio={self._turn_ratio:.3f}, "
            f"key window={self._key_window:.2f}s"
        )

    @property
    def node(self):
        return self._node

    @property
    def quit_requested(self) -> bool:
        return self._quit_event.is_set()

    def _twist_message(self, command: WheelCommand):
        message = self._Twist()
        message.linear.x = float(command.vx)
        message.linear.y = 0.0
        message.linear.z = 0.0
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = float(command.wz)
        return message

    def _current_command(self) -> WheelCommand:
        with self._key_lock:
            keys = active_keys(dict(self._last_seen), time.monotonic(), self._key_window)
        return resolve_wheel_command(
            keys, self._drive_speed, self._turn_speed, self._turn_ratio,
            self._wheel_separation, self._max_wheel_speed,
        )

    def _publish_current(self):
        if self.quit_requested:
            command = WheelCommand(0.0, 0.0, 0.0, 0.0, True, NO_STOP_REASON)
        else:
            command = self._current_command()
        if not command.safe:
            self._node.get_logger().error(command.reason)
            command = WheelCommand(0.0, 0.0, 0.0, 0.0, False, command.reason)
        self._publisher.publish(self._twist_message(command))

    def _record_key(self, key: str) -> None:
        with self._key_lock:
            if key in STOP_KEYS:
                self._last_seen.clear()
            elif key in DRIVE_KEYS:
                self._last_seen[key] = time.monotonic()

    def _input_loop(self):
        if not sys.stdin.isatty():
            self._node.get_logger().error("Keyboard teleop requires an interactive TTY")
            self._quit_event.set()
            return
        fd = sys.stdin.fileno()
        try:
            old_settings = termios.tcgetattr(fd)
        except termios.error as exc:
            self._node.get_logger().error(f"Cannot configure terminal: {exc}")
            self._quit_event.set()
            return
        try:
            tty.setcbreak(fd)
            self._node.get_logger().info(
                "Controls: w forward, s reverse, a/d diagonal, w+a/w+d combine, "
                "space or x stop, q quit"
            )
            while not self._stop_event.is_set():
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not ready:
                    continue
                key = sys.stdin.read(1)
                if key == ESCAPE_INTRO:
                    while select.select([sys.stdin], [], [], 0.001)[0]:
                        sys.stdin.read(1)
                    continue
                key = key.lower()
                if key in QUIT_KEYS:
                    self._quit_event.set()
                    break
                self._record_key(key)
        except (OSError, termios.error) as exc:
            self._node.get_logger().error(f"Keyboard input failed: {exc}")
            self._quit_event.set()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self._stop_event.set()

    def start_input(self):
        self._input_thread = threading.Thread(
            target=self._input_loop, name="rover-keyboard", daemon=True
        )
        self._input_thread.start()

    def stop(self):
        self._stop_event.set()
        self._quit_event.set()
        if self._input_thread is not None:
            self._input_thread.join(timeout=1.0)
        zero = WheelCommand(0.0, 0.0, 0.0, 0.0, True, NO_STOP_REASON)
        self._publisher.publish(self._twist_message(zero))
        self._rclpy.spin_once(self._node, timeout_sec=0.05)


def main(args=None) -> int:
    cli_args, _ = parse_cli(args)
    if cli_args.dry_run:
        return run_dry_run(cli_args)
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
    except ImportError as exc:
        print(f"[!] ROS2 Python dependencies are unavailable: {exc}", file=sys.stderr)
        return 1

    rclpy.init()
    teleop = KeyboardTeleopNode(rclpy, Node, Twist, cli_args)
    teleop.start_input()
    try:
        while rclpy.ok() and not teleop.quit_requested:
            rclpy.spin_once(teleop.node, timeout_sec=0.1)
        return 0
    except KeyboardInterrupt:
        return 0
    except (ValueError, RuntimeError) as exc:
        print(f"[!] Keyboard teleop failed: {exc}", file=sys.stderr)
        return 1
    finally:
        teleop.stop()
        teleop.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

