"""ROS-independent, fail-closed checks for autonomous velocity commands."""

import math


class NavigationSafety:
    def __init__(self, enabled=False, timeout=0.3, scan_timeout=0.5,
                 track_width=0.172, min_wheel_speed=0.35, max_wheel_speed=0.5):
        values = (timeout, scan_timeout, track_width, min_wheel_speed, max_wheel_speed)
        if any(not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError("safety limits must be finite and positive")
        if min_wheel_speed >= max_wheel_speed:
            raise ValueError("minimum wheel speed must be below maximum")
        self.enabled = enabled
        self.timeout = timeout
        self.scan_timeout = scan_timeout
        self.track_width = track_width
        self.min_wheel_speed = min_wheel_speed
        self.max_wheel_speed = max_wheel_speed
        self.command = (0.0, 0.0)
        self.command_at = None
        self.scan_at = None
        self.scan_stamp = None
        self.scan_valid = False
        self.odom_at = None
        self.odom_stamp = None

    def result(self, monotonic_now, wall_now, tf_valid):
        if not self.enabled:
            return (0.0, 0.0), "motion_disabled"
        if self.command_at is None or not 0 <= monotonic_now - self.command_at <= self.timeout:
            return (0.0, 0.0), "command_timeout"
        if (
            not self.scan_valid or self.scan_at is None or self.scan_stamp is None
            or not 0 <= monotonic_now - self.scan_at <= self.scan_timeout
            or not -0.05 <= wall_now - self.scan_stamp <= self.scan_timeout
        ):
            return (0.0, 0.0), "scan_unavailable"
        if (
            self.odom_at is None or self.odom_stamp is None
            or not 0 <= monotonic_now - self.odom_at <= self.scan_timeout
            or not -0.05 <= wall_now - self.odom_stamp <= self.scan_timeout
        ):
            return (0.0, 0.0), "odometry_unavailable"
        if not tf_valid:
            return (0.0, 0.0), "transform_unavailable"
        vx, wz = self.command
        if not all(math.isfinite(v) for v in self.command):
            return (0.0, 0.0), "invalid_command"
        if vx == 0.0 and wz == 0.0:
            return (0.0, 0.0), "stopped"
        left = vx - wz * self.track_width / 2
        right = vx + wz * self.track_width / 2
        if vx < self.min_wheel_speed or not (
            self.min_wheel_speed <= left <= self.max_wheel_speed
            and self.min_wheel_speed <= right <= self.max_wheel_speed
        ):
            return (0.0, 0.0), "unsafe_wheel_speed"
        return (vx, wz), "ready"
