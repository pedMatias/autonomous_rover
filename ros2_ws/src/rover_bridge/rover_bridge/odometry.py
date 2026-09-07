"""Bounded differential-drive odometry for the live ROS2 bridge."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OdometryState:
    x: float
    y: float
    theta: float
    linear_velocity: float
    angular_velocity: float


class OdometryIntegrator:
    """Integrate cumulative wheel distances into a planar pose.

    The first sample establishes the encoder baseline. A non-positive time
    step or an implausibly large encoder jump establishes a new baseline and
    reports zero velocity instead of creating a pose spike after reconnects
    or counter resets.
    """

    def __init__(
        self,
        left_distance_scale=1.0,
        right_distance_scale=1.0,
        track_width_m=0.172,
        left_encoder_sign=1,
        right_encoder_sign=1,
        max_distance_delta_m=2.0,
        min_dt_s=0.005,
        max_dt_s=0.5,
        max_linear_velocity_mps=1.0,
        max_angular_velocity_rps=5.0,
    ):
        values = (
            left_distance_scale,
            right_distance_scale,
            track_width_m,
            max_distance_delta_m,
            min_dt_s,
            max_dt_s,
            max_linear_velocity_mps,
            max_angular_velocity_rps,
        )
        if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in values):
            raise ValueError("odometry calibration values must be finite and positive")
        if int(left_encoder_sign) not in (-1, 1) or int(right_encoder_sign) not in (-1, 1):
            raise ValueError("encoder signs must be -1 or 1")

        self.left_distance_scale = float(left_distance_scale)
        self.right_distance_scale = float(right_distance_scale)
        self.track_width_m = float(track_width_m)
        self.left_encoder_sign = int(left_encoder_sign)
        self.right_encoder_sign = int(right_encoder_sign)
        self.max_distance_delta_m = float(max_distance_delta_m)
        self.min_dt_s = float(min_dt_s)
        self.max_dt_s = float(max_dt_s)
        self.max_linear_velocity_mps = float(max_linear_velocity_mps)
        self.max_angular_velocity_rps = float(max_angular_velocity_rps)
        if self.min_dt_s >= self.max_dt_s:
            raise ValueError("min_dt_s must be less than max_dt_s")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self._previous_left = None
        self._previous_right = None
        self._previous_timestamp = None
        self.last_update_reset = False

    @property
    def initialized(self):
        return self._previous_left is not None

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.reset_baseline()

    def reset_baseline(self):
        """Discard encoder history without changing the integrated pose."""
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self._previous_left = None
        self._previous_right = None
        self._previous_timestamp = None
        self.last_update_reset = True

    def update(self, left_distance, right_distance, timestamp):
        left_distance = float(left_distance) * self.left_encoder_sign * self.left_distance_scale
        right_distance = float(right_distance) * self.right_encoder_sign * self.right_distance_scale
        timestamp = float(timestamp)
        if not all(math.isfinite(value) for value in (left_distance, right_distance, timestamp)):
            raise ValueError("wheel distances and timestamp must be finite")

        self.last_update_reset = False
        if self._previous_left is None:
            self._set_baseline(left_distance, right_distance, timestamp)
            return self.state()

        left_delta = left_distance - self._previous_left
        right_delta = right_distance - self._previous_right
        dt = timestamp - self._previous_timestamp
        if (
            dt < self.min_dt_s
            or dt > self.max_dt_s
            or abs(left_delta) > self.max_distance_delta_m
            or abs(right_delta) > self.max_distance_delta_m
        ):
            self._set_baseline(left_distance, right_distance, timestamp)
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.last_update_reset = True
            return self.state()

        center_distance = (left_delta + right_delta) / 2.0
        heading_delta = (right_delta - left_delta) / self.track_width_m
        linear_velocity = center_distance / dt
        angular_velocity = heading_delta / dt
        if (
            abs(linear_velocity) > self.max_linear_velocity_mps
            or abs(angular_velocity) > self.max_angular_velocity_rps
        ):
            self._set_baseline(left_distance, right_distance, timestamp)
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.last_update_reset = True
            return self.state()
        midpoint_heading = self.theta + heading_delta / 2.0
        self.x += center_distance * math.cos(midpoint_heading)
        self.y += center_distance * math.sin(midpoint_heading)
        self.theta = math.atan2(
            math.sin(self.theta + heading_delta),
            math.cos(self.theta + heading_delta),
        )
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self._set_baseline(left_distance, right_distance, timestamp)
        return self.state()

    def state(self):
        return OdometryState(
            self.x,
            self.y,
            self.theta,
            self.linear_velocity,
            self.angular_velocity,
        )

    def _set_baseline(self, left_distance, right_distance, timestamp):
        self._previous_left = left_distance
        self._previous_right = right_distance
        self._previous_timestamp = timestamp