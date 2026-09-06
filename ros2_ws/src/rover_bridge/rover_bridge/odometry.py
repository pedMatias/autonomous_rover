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
    ):
        values = (
            left_distance_scale,
            right_distance_scale,
            track_width_m,
            max_distance_delta_m,
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
            dt <= 0.0
            or abs(left_delta) > self.max_distance_delta_m
            or abs(right_delta) > self.max_distance_delta_m
        ):
            self._set_baseline(left_distance, right_distance, timestamp)
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.last_update_reset = True
            return self.state()

        center_distance = (left_distance + right_distance) / 2.0
        heading_delta = (right_distance - left_distance) / self.track_width_m
        midpoint_heading = self.theta + heading_delta / 2.0
        self.x += center_distance * math.cos(midpoint_heading)
        self.y += center_distance * math.sin(midpoint_heading)
        self.theta = math.atan2(
            math.sin(self.theta + heading_delta),
            math.cos(self.theta + heading_delta),
        )
        self.linear_velocity = center_distance / dt
        self.angular_velocity = heading_delta / dt
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