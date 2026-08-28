"""
Differential-drive wheel odometry integrator for the UGV Rover.

Consumes cumulative encoder tick counts (odl, odr from the ESP32's T:1001
feedback) and integrates them into a 2D pose (x, y, theta) using the
calibration constants derived by pedro_scripts/calibrate_odometry.py
(meters_per_tick_left/right, track_width_m).

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
    def __init__(self, meters_per_tick=None, track_width_m=None,
                 meters_per_tick_left=None, meters_per_tick_right=None):
        if meters_per_tick_left is None:
            meters_per_tick_left = meters_per_tick
        if meters_per_tick_right is None:
            meters_per_tick_right = meters_per_tick
        if not meters_per_tick_left or not meters_per_tick_right or not track_width_m:
            raise ValueError("left/right meters_per_tick and track_width_m must be calibrated")
        self.meters_per_tick_left = meters_per_tick_left
        self.meters_per_tick_right = meters_per_tick_right
        self.track_width_m = track_width_m

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self._prev_odl = None
        self._prev_odr = None
        self._prev_ts = None

        self.vx = 0.0
        self.vth = 0.0

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta
        self._prev_odl = None
        self._prev_odr = None
        self._prev_ts = None
        self.vx = 0.0
        self.vth = 0.0

    def update(self, odl, odr, ts):
        """Feed a new (odl, odr, host_timestamp) sample. Returns (x, y, theta, vx, vth).

        The first call only establishes the baseline (no motion is assumed
        between "power on" and "first sample"), since odl/odr are cumulative
        counters of unknown absolute origin.
        """
        if self._prev_odl is None:
            self._prev_odl = odl
            self._prev_odr = odr
            self._prev_ts = ts
            return self.x, self.y, self.theta, self.vx, self.vth

        dl_ticks = odl - self._prev_odl
        dr_ticks = odr - self._prev_odr
        dt = ts - self._prev_ts

        dl = dl_ticks * self.meters_per_tick_left
        dr = dr_ticks * self.meters_per_tick_right
        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.track_width_m

        # Integrate using the mid-point heading for better accuracy over the step.
        mid_theta = self.theta + dtheta / 2.0
        self.x += dc * math.cos(mid_theta)
        self.y += dc * math.sin(mid_theta)
        self.theta = math.atan2(math.sin(self.theta + dtheta), math.cos(self.theta + dtheta))

        if dt > 0:
            self.vx = dc / dt
            self.vth = dtheta / dt

        self._prev_odl = odl
        self._prev_odr = odr
        self._prev_ts = ts

        return self.x, self.y, self.theta, self.vx, self.vth
