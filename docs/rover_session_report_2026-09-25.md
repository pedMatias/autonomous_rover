# Rover autonomy debugging session -- 2026-09-25

Report on the first live tests of `rover_avoidance` on the Waveshare UGV Rover
(6WD, Raspberry Pi 5, OAK-D Lite), the measurements taken, what was found, what
was changed, and what is still open.

## Executive summary

The rover initially never moved at all, then moved only in jerks, and finally
browned out and shut down. Four separate causes were found and three are fixed:

| # | Symptom | Cause | Status |
| --- | --- | --- | --- |
| 1 | No reaction to a goal at all | Single-shot `ros2 topic pub --once` exits ~0.1 s after publishing; the goal never reached the node's callback | Diagnosed; use `--times 3 --keep-alive 3` |
| 2 | `BLOCKED: no clear local path` on a visibly clear path | Every zero/beyond-range depth pixel was turned into "unknown space" points that vetoed all corridors, because the camera height lies inside the obstacle height band | **Fixed** (`missing_depth_blocks_paths: false`) |
| 3 | Jerky, stall-then-jump motion | Low-speed commands sit in the wheel stall band; plus the wheel-derived yaw random-walks in 3.33 degree quantisation steps, and the steering had a single chatter-prone threshold | **Partly fixed**; rotation stall band still to measure |
| 4 | Rover shut down mid-session | Sub-stall commands stall the motors against floor friction and pull locked-rotor current until the rail collapses | **Documented**; forward speed floor raised to 0.35 m/s |

The planner itself turned out to be healthy: over 45 consecutive live frames it
chose the straight-ahead path every time with zero measured points in the
forward corridor. The problems were in message delivery, depth policy, command
magnitudes and odometry quality -- not in the corridor logic.

## System under test

```
PC (ROS 2 Jazzy)                                   Raspberry Pi 5           ESP32
-----------------                                  --------------           -----
rover_avoidance  --/cmd_vel-->  rover_bridge  --->  rover_agent  -------->   motors
     ^                              |                    |
     |--/camera/depth/.../compressedDepth ---------------+ camera_stream (8770)
     |--/oak/rgb/camera_info ----------------------------+
     |--/odom, /tf (20 Hz) <-------- telemetry (8766) ---+
```

- `rover_bridge` forwards the latest `/cmd_vel` to the Pi at 20 Hz and zeroes
  commands if none arrive within `cmd_vel_timeout_s` (0.5 s).
- `rover_agent` on the Pi converts Twist to per-wheel speeds, applies a
  `max_wheel_accel_mps2` of 1.0 ramp, and has its own 0.5 s watchdog.
- `rover_avoidance` accepts a base_link-relative `PointStamped`, freezes it in
  `odom`, picks one of seven straight local corridors from depth, and drives.
- Every terminal needs `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
  `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`.

## Timeline of attempts

| Time | Attempt | Result |
| --- | --- | --- |
| 12:28-12:33 | First bringup + three `ros2 topic pub --once` goals | Node stayed `IDLE`; no `DRIVE`/`FAULT` in its log |
| 12:39-12:44 | Restart, graph checks, `ros2 topic echo /avoidance/status` | Avoidance subscription present and correct; status echo empty (volatile topic, published before the echo subscribed) |
| 12:44-12:58 | Bridge died (battery), 349 `connection lost` entries in the bridge log | Pi unreachable; not a software fault |
| 13:00-13:10 | `--once` goal x=10.0 | Node stayed `IDLE` again |
| 13:14-13:20 | `--times 3 --keep-alive 3` goal x=0.0, then x=10.0 | x=0.0 gave `FAULT: goal must be finite and more than 0.05 m forward` (delivery proven); x=10.0 gave `DRIVE: goal accepted` then `BLOCKED: no clear local path` |
| 13:37-13:42 | Live depth analysis, no motion | Depth decodes correctly; 35% zero pixels; all seven corridors vetoed by "unknown" |
| 14:20-14:35 | Policy A implemented and validated | 45/45 live frames chose heading 0.00; straight corridor had 0 measured points |
| 14:38-14:47 | User re-ran a 10 m goal | Rover moved while alternating `aligning`/`moving`, ended `FAULT: odometry missing or stale` |
| 15:12-15:16 | Command-path measurement with wheels idle | 19 Hz telemetry and odometry, max gap 74 ms, Pi echo at 20.3 cmd/s, 12.28 V |
| 15:16-15:20 | 0.10 m/s step response, then a 0.5 m avoidance goal | Echo at +37 ms, full speed +87 ms, no watchdog stops; goal run: 48 turn / 12 forward samples, 0.157 m travel |
| 15:22-15:24 | Planner stability probe (no motion) | 45/45 frames heading 0.00; TF confirmed `(0, 0, 0.12)` |
| 15:29 | Straight speed sweep 0.02-0.20 m/s | 0.02 m/s delivered only 33% of expected travel |
| 15:35 | Rotation sweep 0.10-0.65 rad/s | Rover lost power mid-sweep; results confounded (see F6) |
| 15:40-16:00 | Documentation, config changes, tests | `docs/rover_low_speed_stall.md` added; 42 tests pass |

## Findings

Each finding lists the evidence and whether it is confirmed, disproved, or still
a hypothesis.

### F1. Depth was decoded correctly -- "the rover cannot see" was disproved

The compressed depth payload is a 12-byte `compressedDepth` header followed by a
PNG (`00 00 00 00 00 00 00 00 00 00 00 00 89 50 4E 47`), decoded with the node's
own functions to a 480x640 `uint16` image whose values are physically sensible
(far wall ~3100 mm at the top rows, near floor ~390 mm at the bottom rows).

Frame composition while looking at a clear indoor path:

| Pixels | Share | Meaning |
| --- | --- | --- |
| zero (no stereo return) | 35.5% | 93% of the top rows (textureless ceiling), 89% of the bottom rows (textureless floor) |
| farther than `depth_max_m` (3 m) | 6.5% | the sensor saw nothing within range |
| valid 0.2-3.0 m | 54.4% | usable measurements |

**Status: confirmed.** The readings are not wrong; how they were interpreted was.

### F2. `BLOCKED` on a clear path -- the missing-depth policy (fixed)

`depth_to_base_points` converted every zero *and* every beyond-range pixel into
"unknown" points sampled every 0.1 m along the ray from 0.1 m to 1.2 m, and
`choose_heading` rejected a corridor if any one of them fell inside the obstacle
height band `z in [0.04, 0.75]`. The camera sits at `z = 0.12 m`, *inside* that
band, so nearly every unknown ray produced a hit at its very first sample.

Measured on one live clear-path frame:

| Policy | Forward-corridor measured obstacles | Forward-corridor unknown hits | Verdict |
| --- | --- | --- | --- |
| shipped behaviour (zero + beyond = unknown, 1.2 m) | 0 | 7,981 | `BLOCKED` |
| beyond-range treated as free | 0 | 4,843 | `BLOCKED` |
| beyond-range free, unknown sampled only to 0.5 m | 0 | 3,190 | `BLOCKED` |

No threshold tweak fixes this: the dropouts are structural (stereo cannot match
textureless surfaces). The same frame did contain one real obstacle correctly
located at about `x = 0.69 m, y = -0.44 m` (to the right), which blocked only the
right-turn corridors -- the measured-point logic was working.

**Fix:** `missing_depth_blocks_paths` (default `false`): only measured returns
block a corridor; the conservative rule is still available for comparison.
**Status: fixed and validated** (45/45 live frames then chose heading 0.00 with
the straight corridor clear).

### F3. Goals were published but never delivered (`--once` is not enough)

`ros2 topic pub --once` waits for a matching subscription, publishes once, and
keeps the publisher alive for only 0.1 s (`ros2topic/verb/pub.py`, `keep_alive`
default 0.1). Three separate `--once` attempts produced no callback at all: the
node logged only `IDLE`, and a processed goal would always log `DRIVE` or
`FAULT` immediately.

The same failure was reproduced with a custom probe that published without first
waiting for the subscription match. After adding the match wait, the identical
goal reached the node every time. `--times 3 --keep-alive 3` also worked
consistently and produced the decisive test (`x=0.0` -> `FAULT`, proving the
callback ran).

**Status: confirmed.** For scripted tests, wait for
`publisher.get_subscription_count() > 0` (or use the CLI's `--times`), and note
that a goal is a *one-shot* command: repeating it re-accepts a new relative
target, which is what the three `--times 3` 10 m messages did.

### F4. The ROS-to-Pi command path is healthy

Measured with the wheels idle:

- `/rover/telemetry` 19.0 Hz, `/odom` 19.1 Hz, max gap 74 ms, **zero** gaps
  above 0.1 s over 6.7 s.
- The Pi's `cmd_echo.seq` advanced at **20.3 commands/s** -- the bridge's 20 Hz
  stream arrives continuously.
- Battery 12.28 V at the time (stop threshold is 9.9 V).

Step response after commanding 0.10 m/s:

| Event | Delay after publish |
| --- | --- |
| Pi echo shows first nonzero wheel speed (0.049 m/s) | +37 ms |
| Pi echo reaches the full 0.100 m/s | +87 ms |
| Wheel-integrated travel returns to rest after the stop | +230 ms |

The intermediate 0.049 m/s reading is the Pi's 1.0 m/s^2 acceleration ramp, not
jitter. Throughout a 2 s constant command the echo stayed at a steady
`L = R = 0.100` with no watchdog zeroes, and the bridge logged no telemetry or
command connection losses during the earlier 10 m run.

**Status: confirmed.** There is no significant ROS-to-Pi lag and no timeout storm
while the link is up. "Too many timeouts on the Pi" was not reproduced.

### F5. Odometry quality is the weakest signal in the loop

With a steady 0.100 m/s command the published `/odom` twist alternated between
`0.000`, `0.094` and `0.195 m/s`, while position integrated smoothly at about
0.095 m/s. The cause is resolution: wheel distance arrives in 1 cm units
(`odl`/`odr`) at 20 Hz, so one count is `0.01 / 0.05 = 0.2 m/s` of apparent
velocity.

The heading is worse for control purposes. During a nominally straight creep the
estimated yaw only ever took two values, `3.33 deg` and `6.66 deg`:

- one count of left/right mismatch is `0.01 / 0.172 = 0.0581 rad = 3.33 deg`
- the align tolerance is `0.12 rad = 6.9 deg`, i.e. only about two counts

So the heading estimate random-walks across the steering tolerance within about
half a second of straight driving, and a proportional controller keeps reacting
to quantisation noise. This, together with the stall band (F6), is why the
`aligning`/`moving` states churned every few ticks in the 10 m run.

**Status: confirmed (quantisation).** Mitigations shipped: a yaw low-pass filter
(`yaw_filter_time_constant_s: 0.3`) and hysteresis on the steering threshold.
A better long-term source would be the gyro already present in telemetry
(`gz`, with a bias in `config/calibration.yaml`), which is smooth and does not
suffer from count quantisation -- not implemented because its units and drift
have not been measured.

### F6. Low-speed commands stall the motors (hardware limit)

Measured straight-line travel against command:

| Commanded `vx` | Expected travel in 1.4 s | Measured | Ratio |
| --- | --- | --- | --- |
| **0.02 m/s** | 0.028 m | 0.009 m | **0.33** |
| 0.05 m/s | 0.070 m | 0.063 m | 0.89 |
| 0.08 m/s | 0.112 m | 0.100 m | 0.89 |
| 0.12 m/s | 0.169 m | 0.145 m | 0.86 |
| 0.20 m/s | 0.281 m | 0.225 m | 0.80 |

Below roughly 0.03 m/s the wheels do not reliably turn: the duty cycle cannot
beat static friction, so the motor sits in a stall instead of creeping. This is
why jerky motion is reported at low speed and smooth motion above ~0.35 m/s, and
why the factory app is smooth when driven fast.

The dangerous consequence observed in this session is a **brownout**: sustained
stall current collapses the battery rail and the rover shuts down (the Pi left
the network entirely -- no ARP entry -- and the camera stream stopped). The same
session had already lost the rover once at ~12:44 for the same reason.

The rotation sweep (`wz` 0.10, 0.20, 0.30, 0.45, 0.65 rad/s) reported 0.000 rad
of rotation for every value except one, but that sweep overlapped the power
loss, so **those rotation numbers are invalid and must be re-measured**.

Linking the two: the align command is `1.2 * error` rad/s, so an error of
0.15 rad commands only 0.18 rad/s, i.e. `+/-0.015 m/s` per wheel -- inside the
stall band. The rover therefore often *asks* to move without moving, which reads
as a stall followed by a jump, and is electrically the worst case.

**Status: confirmed for straight motion and for the brownout mechanism (as the
operator's diagnosis, consistent with the measurements). Rotation value: open.**

## Changes shipped

### `rover_avoidance`

| File | Change |
| --- | --- |
| `planner.py` | `depth_to_base_points(..., missing_depth_blocks_paths=False)`; new `normalize_angle` and `filtered_yaw`; `command_for_heading(..., release_ratio, was_aligning, min_angular_speed)` |
| `node.py` | New params `missing_depth_blocks_paths`, `align_tolerance_rad`, `align_release_ratio`, `yaw_filter_time_constant_s`, `min_align_rate_rps`; `max_linear_speed_mps` 0.12 -> 0.35; yaw filtered in `_on_odom` and used for both goal freezing and steering; `_aligning` hysteresis state |
| `config/avoidance.yaml` | Matching values with comments explaining the stall limit and the quantisation workarounds |

### `rover_slam`

`camera_z` default `0.0 -> 0.12` in `live_bringup.launch.py`,
`offline_mapping.launch.py` and `config/frames.yaml`, from the measured camera
height (17 cm above the floor, `base_link` 5 cm above the floor). Verified live:
TF now reports `(0.0, 0.0, 0.12)`.

### Documentation

- `docs/rover_low_speed_stall.md` (**new**): the hardware constraint, its
  evidence, practical rules, and the measurements still owed.
- `docs/rover_session_report_2026-09-25.md` (**new**): this report.
- `docs/rover_avoidance_first_test.md`: never-creep warning; camera-value note;
  missing-depth policy.
- `docs/record_new_dataset.md`: camera transform consistency and the
  `camera_z:=0.0` override for older bags.
- `rover_avoidance/README.md`, `rover_slam/README.md`,
  `rover_bridge/README.md`, `AGENTS.md`: cross-references to the stall limit
  and the new parameters.

### Tests

`rover_avoidance/test/test_planner.py` grew from 12 to 22 tests, covering: depth
holes not being obstacles by default, holes still blocking in conservative mode,
beyond-range returns, a measured obstacle inside an otherwise dropped-out frame,
steering hysteresis, the yaw filter (including wrap-around and parameter
validation), and the minimum align rate. `colcon test` reports **42 tests, 0
failures**.

## Open items

1. **Measure the rotation stall band** on a powered rover: sweep `wz` (for
   example 0.2, 0.4, 0.6, 0.8 rad/s for 1-2 s each, stopping between steps,
   in place) and set `min_align_rate_rps` from the result. The current 0.5 rad/s
   is provisional and unverified.
2. **Confirm the 0.35 m/s forward floor** with a loaded rover on the target
   floor; raise it if the wheels still stall.
3. **Re-run the 0.5 m goal test** with `max_linear_speed_mps: 0.35`, the yaw
   filter and the align floor, and compare the turn/forward sample ratio against
   the 48/12 baseline measured at 0.12 m/s. Note that 0.35 m/s covers 1.2 m of
   lookahead in 3.4 s and the bridge still stops within ~0.18 m after a 0.5 s
   command gap.
4. **Explain `FAULT: odometry missing or stale`** seen at the end of the 10 m
   run. It was not reproduced while the link was healthy (odometry had no gaps
   above 0.1 s), so a link dropout is the leading explanation, not the planner.
5. **Consider the gyro** (`gz` in telemetry, bias in `config/calibration.yaml`)
   as the heading source instead of wheel counts, after measuring its units and
   drift.
6. **Fetch the Pi-side agent log.** SSH currently fails
   (`Permission denied (publickey,password)` for `pi@192.168.0.12`); key-based
   access would give the `watchdog` / `wheel command` lines that confirm or rule
   out Pi-side command timeouts directly.
7. **Known limitation of the depth policy:** with `missing_depth_blocks_paths:
   false`, a textureless or dark obstacle can return nothing and will not block
   a path. Compensate with the speed floor, the lookahead and the physical power
   switch, or enable the conservative mode for demos.

## Safety notes

- The physical power switch is the immediate stop; treat `/cmd_vel` as advisory.
- Check battery voltage before and during runs. 12.28 V was healthy; the earlier
  brownouts happened on a rail that had sagged.
- Do not run long unattended goals. The 10 m goal was the longest and the
  heaviest run of the session.
- Never creep. Stopping is safer than commanding a speed the wheels cannot turn.
- Bringup must be restarted after code or config changes: Python modules are
  loaded into the running process, so a rebuild alone does not update a live
  node.

## Method notes and caveats

- Measurements came from `/odom`, `/rover/telemetry` (the Pi's `cmd_echo`),
  `/cmd_vel`, `/avoidance/status`, `/camera/depth/image_raw/compressedDepth`,
  `/oak/rgb/camera_info` and the bridge/launch logs in `~/.ros/log`.
- The wheel-distance unit is 1 cm, so all odometry-derived smoothness claims are
  limited by that resolution; "did it move" is trustworthy, "how smoothly" is
  not.
- Terminal output capture in the tooling was unreliable (empty results, killed
  processes, one background process that appeared isolated from DDS discovery).
  Every load-bearing measurement was therefore repeated and written to a file
  before being trusted, and results that arrived while processes were being
  killed were discarded -- the rotation sweep in particular.
- The electrical explanation for the brownout is the operator's diagnosis. It is
  consistent with the measured stall band, but current was not measured; doing
  so needs a supply-side measurement or the Pi's own voltage telemetry during a
  deliberate stall test.
