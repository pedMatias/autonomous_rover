# Pi-local autonomy prototype

The standalone Python application combines local OAK-D depth, the existing
candidate-arc planner, encoder goal tracking, a GCS and detailed run recording.
It is a supervised experimental alternative to the remote ROS motor gateway,
not a general route planner.

## Run on the rover

Install dependencies on the Pi and launch the real system:

```bash
cd /home/ws/ugv_rpi
ugv-env/bin/python -m pip install -r requirements-autonomy.txt
ugv-env/bin/python -m rover_autonomy
```

With no mode flags, this opens the live OAK-D and the real ESP32 motor UART.
An accepted goal starts physical motion. This is not a demo/dry run. Use a
supervised test area, keep the physical stop accessible, and ensure no other
camera or UART owner is active.

Stop the services or manual processes that currently own the camera or UART
before launching. On installations using these units, stop
`rover-agent.service`, `rover-camera.service`, and `rover-preview.service`.
Check which are installed/running first; this application does not manage
existing services. It refuses an occupied UART rather than taking it over.
Another OAK-D owner will cause camera startup to fault; no goal will be accepted.

The GCS binds **only to `127.0.0.1:5002`**. Use SSH forwarding from the operator
computer:

```bash
ssh -L 5002:127.0.0.1:5002 <user>@<pi>
```

Open `http://127.0.0.1:5002`. Verify it reports **LIVE MOTOR OUTPUT ENABLED**
and check the camera-mount warning, depth view, telemetry/battery and logging
status before submitting a goal. A goal is a high-level request and starts the
onboard controller immediately when readiness checks pass.

Every launch creates a uniquely named run directory under
`recordings/autonomy/`. `--port` and `--logs` select another port or output
directory.

## Explicit non-driving modes

- `--mock-motors`: use the real OAK-D with simulated wheel feedback/commands.
  This is a stationary perception/debug mode, not odometry validation.
- `--demo clear`, `--demo wall` or `--demo empty`: use synthetic images and
  mock motors, with no physical camera or UART access.
- `--replay-run recordings/autonomy/<run-id>`: offline replay with mock
  output; it does not start the GCS or touch hardware.

The real-motor mode uses `fuser` (from OS package `psmisc`) to reject an active
UART owner and requests exclusive serial access. No LAN bind option is exposed.

For supervised experiments, keep the physical switch accessible, start on a
clear floor and then try one stationary obstacle with room to pass. Never
creep or pivot:
the [0.35 m/s minimum forward speed](rover_low_speed_stall.md) is a hardware
constraint. Autonomy tuning commands at least 0.35 m/s to **each** wheel and
0.4 m/s nominal forward speed. The ROS planner configuration is unchanged.
Turning behavior still needs supervised observation on this rover.

The controller sends direct bounded forward wheel commands, not the former
agent's low-speed acceleration ramp. Observe startup response and battery
behavior in supervised tests. The Pi watchdog cannot survive
process death; firmware crash-stop behavior remains unverified. Do not mistake
the GCS stop button for the physical immediate stop.

## Goals, stopping and scope

Goals are forward relative offsets in metres (`x` forward, `y` left). Unlike
the preview's 10 m radial limit, `(10, 1)` is accepted when inputs are ready.
The accepted target is fixed in encoder odometry.

Only one goal is active. Cancel/stop before replacing it; a new goal requires
fresh depth, feedback, battery information, heartbeat and stopped wheel feedback.
Requests carry an ID and controller epoch; retries of an accepted request do
not start a second goal, and stale epochs are rejected.

The controller stops inside the configured odometry goal tolerance, then waits
for several stopped-feedback samples before reporting `GOAL_REACHED`. This
does not guarantee physical endpoint accuracy or account for all braking drift.
Inspect actual stopping travel and drift from experiments.

Forward-only local arcs may pass around one simple obstacle. The controller
returns `BLOCKED` for obstructed paths, a behind-rover target or lack of progress.
It does not back up, pivot, execute the preview's hypothetical search prefixes,
or navigate U-shaped obstacles/dead ends.

Freshness, battery, validity and heartbeat checks are lightweight. By default,
host depth/feedback timeout is 0.6 s and operator heartbeat timeout is 2 s.
The GCS can set the heartbeat timeout from 1 to 60 seconds or explicitly disable
the heartbeat-loss stop for the current process. Disabling it means a lost GCS
or SSH connection will not stop an active goal; the dashboard requires an
explicit browser confirmation and displays a persistent warning. These are
configurable prototype values, not verified stopping guarantees. Heartbeat
expiry terminates the goal; reconnecting requires a new goal.
The browser sends heartbeats independently of rendering, including while the
debug view is frozen. Each browser request is bounded to 3 seconds and the page
shows consecutive failures, last successful browser round-trip time, and the
Pi's last-received age. This browser request bound is distinct from the
configurable controller stop timeout. A sleeping/throttled tab or broken SSH
tunnel can still cause a stop when the watchdog is enabled.

## GCS observability

The page prominently shows controller state, stop reason and remaining goal
distance alongside heartbeat delivery health. It also shows sequence-matched RGB/aligned depth, the local frame ID and OAK-D
sensor sequence/timing metadata, considered obstacle points,
candidate paths, the selected path, rejection reasons, goal/odometry, requested
and written commands, feedback, battery/faults, timing and recording health.
Black depth pixels and validity metrics expose unmeasured/out-of-range input.
The current broad coverage threshold does not prove that a selected corridor
was fully observed; missing returns are not evidence of free space.

Rendering is a separate lower-rate worker with a 15-entry hard cache bound.
The browser does not wait for health before requesting an image. On image
expiry it fetches a fresh view and retries its matching image once. Repeated
failure is visibly marked stale; images are never paired with another decision.
Freeze retains a debug view while live controller status and heartbeat continue.

## Logs and post-run replay

Each run contains:

- `config.json`: camera/planner/control settings, calibration, Git revision and
  real/mock motor mode.
- `events.jsonl`: frame references, device/host timing, heartbeats, accepted/
  rejected goals, state changes, telemetry/odometry, planner candidates/reasons,
  requested/queued/written commands, HTTP handler times and run summary.
- `frame-<sequence>.npz`: lossless BGR8 RGB, uint16 depth and camera metadata
  for each acquired pair successfully recorded.
- `manifest.json`: completion status, dropped events, bounded frame-gap ranges,
  writer errors and write counts.

This lean format reuses the existing NumPy preview serialization rather than
adding a ROS bag dependency to the controller. It is not a ROS bag.

Capture fans inputs into a bounded recording queue and a separate latest-frame
control handoff. Disk I/O never executes in control. Frame pairs are stored as
uncompressed NPZ to reduce Pi CPU and writer backlog; the default queue has 64
entries (a worst-case all-image queue at 640x480 holds about 98 MB of raw
arrays). JSONL events are flushed once per second instead of on every event.
The writer preserves a 1 GiB disk reserve. Raw 10 Hz RGB/depth is approximately
55 GB/hour; measure actual storage load.

Logging failure does not stop an otherwise valid drive. GCS shows incomplete
recording and drop counts; a final manifest is best effort on disk failure.
Manifest completeness concerns host events, not proof of zero sensor drops.
Camera sequence gaps and control overwrites are separate run-summary counters.
Gap-range detail is bounded and explicitly marked truncated if exhausted.

Replay never opens the camera/UART:

```bash
python -m rover_autonomy --replay-run recordings/autonomy/<run-id>
```

It processes recorded frame sequences, goals, heartbeats and feedback through
mock output and reports decision-state/heading mismatches. Asynchronous timing
and incomplete inputs can prevent exact reproduction; this is a debugging
comparison, not a deterministic simulation of firmware or physical motion.

## Profiling and tests

Camera profiling preserves RGB/depth device sequences/timestamps, skew,
host dequeue and conversion durations. The installed DepthAI Sync API receives
the configured threshold in this profiled path. Device timestamps are explicitly
not host monotonic timestamps; host freshness is not exposure age.

GCS shows planner/render/write timing, input rate, CPU/RSS, upstream sequence
gaps and control overwrites. HTTP events report Pi handler duration, not
end-to-end browser latency. Compare no viewers, one viewer and recording load.
The provisional 10 Hz / 100 ms reference is not a safety certification.

```bash
python -m pip install -r requirements-autonomy-test.txt
python -m pytest -q tests/test_rover_autonomy.py tests/test_autonomy_dashboard.py \
  tests/test_oak_camera_preview.py tests/test_rover_agent.py tests/test_odometry.py
```

No test in this set intentionally opens the camera or real motor UART.
Use mocked checks and short supervised experiments to drive the next change;
defer semantic ML, global routing, visual localization and language rewrites
until observations justify them.
