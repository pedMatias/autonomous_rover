# Rover low-speed stall limit (do not creep)

Operator observation (2026-09-25, on the Waveshare UGV Rover 6WD):

> Very low movement or rotation commands -- whether to move slowly or to rotate
> in place -- create very high current spikes, because the wheels cannot
> overcome floor friction. The motors stay in a continuous stall effort until
> the rover browns out and shuts down. Deal with the rover at higher speed,
> for both movement and rotation. **0.35 m/s is the minimum forward speed.**

Treat this as a hardware constraint, not a tuning preference.

## Why this matters

A commanded wheel speed below the stall band does not produce slow motion; it
produces a stalled motor:

- PWM duty is too low to break static friction, so the wheel does not turn.
- The controller keeps the duty applied, so the motor draws locked-rotor
  current continuously instead of only during acceleration.
- Sustained stall current collapses the battery rail, which is consistent with
  the observed brownout: the Pi leaves the network entirely (no ARP entry) and
  the camera stream stops.

Measured support on this rover (wheels on the floor, straight-line commands):

| Commanded `vx` | Expected travel in 1.4 s | Measured travel | Ratio |
| --- | --- | --- | --- |
| **0.02 m/s** | 0.028 m | 0.009 m | **0.33** |
| 0.05 m/s | 0.070 m | 0.063 m | 0.89 |
| 0.08 m/s | 0.112 m | 0.100 m | 0.89 |
| 0.12 m/s | 0.169 m | 0.145 m | 0.86 |
| 0.20 m/s | 0.281 m | 0.225 m | 0.80 |

The 0.02 m/s case moved a third of the commanded distance: the wheels were in
the stall band. The higher rows match the command and reflect the Pi's
`max_wheel_accel_mps2` ramp at the start of each step.

Shutdown evidence from the same session: with `wz` commands between 0.10 and
0.65 rad/s the odometry reported **0.000 rad** of rotation, and minutes later
the bridge logged `rx_fps=0.00` / `Rover camera connection lost` and the Pi
became unreachable with no ARP entry.

The electrical claim (stall current collapsing the rail) is the operator's
diagnosis and is consistent with the measurements above; verify it with
`/battery_state` voltage while a wheel is commanded just below the stall band.
The remainder of this note does not depend on that verification.

## Practical rules

- **Never command forward motion below 0.35 m/s.** `rover_avoidance` uses
  `max_linear_speed_mps: 0.35` for exactly this reason. Do not lower it to make
  the prototype feel gentle; slow is what stalls the motors.
- **Do not rotate in place.** The first autonomous pivot coincided with a
  shutdown requiring a manual power cycle. `rover_avoidance` now turns on
  forward arcs; its wheel limits keep both sides moving in the same direction.
  `min_align_rate_rps` now sets the minimum angular rate *while driving
  forward*, not a minimum in-place rotation rate.
- **Keep `teleop` speeds above the band as well.** `rover_teleop`'s defaults
  (0.39 m/s drive, 0.20 m/s turn creep) are above it; do not dial them down to
  a crawl for "safety".
- **Short and decisive beats long and tentative.** A brief 0.35 m/s move plus a
  stop is electrically kinder than a long 0.05 m/s creep.
- **The physical power switch stays the immediate stop.** Higher speeds reduce
  stall, but they also mean the rover covers more ground: at 0.35 m/s the
  bridge's `cmd_vel_timeout_s` of 0.5 s still stops it within ~0.18 m.

## Still to measure on hardware

These checks need a powered rover on an open floor with someone ready on the
switch:

1. **Forward arcs.** With a short goal, compare requested wheel speeds and
  measured heading change. Stop if either side stalls; do not use an in-place
  `wz` sweep.
2. **Forward stall band.** Confirm the 0.35 m/s floor for straight motion, and
   check whether a loaded rover (full battery, on carpet) needs a higher value.

Do not run these with a partially discharged battery: the earlier shutdown
happened at the end of a long session, and a low rail makes the stall band look
wider than it is.
