"""
Differential-drive wheel odometry integrator for the UGV Rover.

Consumes cumulative T:1001 wheel-distance units (odl, odr) from the ESP32
feedback and integrates them into a 2D pose (x, y, theta). The active
calibration supplies the meters represented by one reported unit on each
side and the effective track width.

Known limitation (accepted in the project plan in README.md): encoder counts
can closely match between wheels while the rover still drifts off a straight
line in the real world (wheel diameter/alignment/slip asymmetry not visible
to the encoders). This integrator will NOT correct for that - it is expected
to accumulate heading drift over time, which is exactly what RTAB-Map's
visual loop closure is meant to correct downstream. Do not over-engineer
fixes here.
"""
import math


class OdometryIntegrator:
    def __init__(self, left_distance_scale=1.0, right_distance_scale=1.0,
                 track_width_m=None, max_distance_delta_m=2.0,
                 min_dt_s=0.005, max_dt_s=0.5,
                 max_linear_velocity_mps=1.0, max_angular_velocity_rps=5.0):
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
        if any(
            value is None
            or not math.isfinite(float(value))
            or float(value) <= 0.0
            for value in values
        ):
            raise ValueError(
                "odometry calibration and guard values must be finite and positive"
            )
        if min_dt_s >= max_dt_s:
            raise ValueError("min_dt_s must be less than max_dt_s")
        self.left_distance_scale = float(left_distance_scale)
        self.right_distance_scale = float(right_distance_scale)
        self.track_width_m = float(track_width_m)
        self.max_distance_delta_m = float(max_distance_delta_m)
        self.min_dt_s = float(min_dt_s)
        self.max_dt_s = float(max_dt_s)
        self.max_linear_velocity_mps = float(max_linear_velocity_mps)
        self.max_angular_velocity_rps = float(max_angular_velocity_rps)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self._prev_odl = None
        self._prev_odr = None
        self._prev_ts = None

        self.vx = 0.0
        self.vth = 0.0
        self.last_update_reset = False

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta
        self.reset_baseline()

    def reset_baseline(self):
        """Discard encoder history without changing the integrated pose."""
        self._prev_odl = None
        self._prev_odr = None
        self._prev_ts = None
        self.vx = 0.0
        self.vth = 0.0
        self.last_update_reset = True

    def update(self, odl, odr, ts):
        """Feed a new (odl, odr, host_timestamp) sample. Returns (x, y, theta, vx, vth).

        The first call only establishes the baseline (no motion is assumed
        between "power on" and "first sample"), since odl/odr are cumulative
        counters of unknown absolute origin.
        """
        odl = float(odl)
        odr = float(odr)
        ts = float(ts)
        if not all(math.isfinite(value) for value in (odl, odr, ts)):
            raise ValueError("wheel distances and timestamp must be finite")

        self.last_update_reset = False
        if self._prev_odl is None:
            self._prev_odl = odl
            self._prev_odr = odr
            self._prev_ts = ts
            return self.x, self.y, self.theta, self.vx, self.vth

        dl_units = odl - self._prev_odl
        dr_units = odr - self._prev_odr
        dt = ts - self._prev_ts

        dl = dl_units * self.left_distance_scale
        dr = dr_units * self.right_distance_scale
        if (
            dt < self.min_dt_s
            or dt > self.max_dt_s
            or abs(dl) > self.max_distance_delta_m
            or abs(dr) > self.max_distance_delta_m
        ):
            self._prev_odl = odl
            self._prev_odr = odr
            self._prev_ts = ts
            self.vx = 0.0
            self.vth = 0.0
            self.last_update_reset = True
            return self.x, self.y, self.theta, self.vx, self.vth

        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.track_width_m
        vx = dc / dt
        vth = dtheta / dt
        if (
            abs(vx) > self.max_linear_velocity_mps
            or abs(vth) > self.max_angular_velocity_rps
        ):
            self._prev_odl = odl
            self._prev_odr = odr
            self._prev_ts = ts
            self.vx = 0.0
            self.vth = 0.0
            self.last_update_reset = True
            return self.x, self.y, self.theta, self.vx, self.vth

        # Integrate using the mid-point heading for better accuracy over the step.
        mid_theta = self.theta + dtheta / 2.0
        self.x += dc * math.cos(mid_theta)
        self.y += dc * math.sin(mid_theta)
        self.theta = math.atan2(math.sin(self.theta + dtheta), math.cos(self.theta + dtheta))

        self.vx = vx
        self.vth = vth

        self._prev_odl = odl
        self._prev_odr = odr
        self._prev_ts = ts

        return self.x, self.y, self.theta, self.vx, self.vth
