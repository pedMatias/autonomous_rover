# Pi-local no-motion preview

This milestone reads aligned raw RGB/depth from the OAK-D, evaluates the
existing avoidance planner locally, and serves an observability dashboard on
port **5001**. It does **not** navigate, estimate progress, open the motor UART,
connect to the motor agent, import ROS2, or transmit wheel commands. Goals and
speeds are hypothetical. Keep the rover stationary.

Other software can still drive independently. Stop teleop and live avoidance
before using this preview; the dashboard is not an emergency stop.

## Install

Run from the repository root on the Pi, using the existing Python environment:

```bash
cd /home/ws/ugv_rpi
ugv-env/bin/python -m pip install -r requirements-preview-pi.txt
```

The small requirements set excludes the factory app's MediaPipe, audio,
Picamera2, and neural-network stack. It installs the existing `rover_avoidance`
Python package in editable mode, but the preview only imports its pure planner.
ROS messages and compressed-depth decoding are not used.

Stop the camera streamer before opening the OAK-D:

```bash
sudo systemctl stop rover-camera.service
ugv-env/bin/python -m rover_preview --host 0.0.0.0
```

Open `http://<pi-ip>:5001` from the same trusted LAN. There is no cloud dependency.
This is a development dashboard using Werkzeug's threaded server, not an
internet-facing deployment. There is no authentication; do not expose the port
to an untrusted network. Goal writes require JSON and a same-origin preview
header. These are not a replacement for authentication.

The preview needs sole ownership of the OAK-D: do not run a recorder or another
camera consumer alongside it. It does not compete with the motor agent for the
UART, but that agent can still accept commands from other clients.

Camera startup performs an eight-second warmup. A camera/import failure stays
visible as `FAULT`; it never silently substitutes synthetic images.

## Dashboard

- **Camera:** RGB and coloured depth from exactly the displayed decision.
  Red RGB dots are the measured points used for obstacle rejection. Black depth
  means unmeasured or outside the configured range, not confirmed free space.
- **Top-down:** sampled points, considered obstacles, candidate centreline
  paths, their first blocking points, and the chosen path. Hover a red point for
  its `(X, Y, Z)` and horizontal range from the base origin. These are point
  measurements, not recognised/grouped objects.
- **Decisions:** explicit status/reason, valid-depth coverage, missing-depth
  policy, hypothetical linear/angular speeds, and an optional short search
  prefix. Green does not guarantee an obstacle-free real-world route.
- **Health:** process CPU/RSS, input rate, temperature where available, worker
  liveness, visualization errors, and time since host dequeue. Dequeue age is
  **not** a measurement of camera-exposure latency. Battery and encoder feedback
  are deliberately unavailable in this camera-only milestone.
- **Failures:** retains one snapshot at the start of each blocked/search/fault
  episode (or a new goal revision). Viewing it does not pause camera acquisition.
  Live preview continues evaluating the current goal on new frames.

Relative goals use positive X forward, positive Y left, and must be within 10 m
and more than 0.05 m forward. A goal remains relative to the stationary rover;
there is no odometry to freeze it in a world frame. The local planner can stop
at dead ends and does not implement global routing or autonomous recovery.

## Settings and resource bounds

[`config/rover_preview.yaml`](../config/rover_preview.yaml) references the
[existing planner tuning](../ros2_ws/src/rover_avoidance/config/avoidance.yaml).
The configured optical-to-base quaternion and camera height match the defaults
in `live_bringup.launch.py`; they are **not a measured camera mount**. The UI
warns until `camera_mount_verified` is explicitly set after checking the real
mount. Camera axes are optical right/down/forward; base axes are
forward/left/up. Do not substitute an identity rotation.

Default acquisition/planning is capped at 10 Hz, visualization at 3 Hz, and each
displayed point set at 600 points. DepthAI's preview output queue holds only the
latest frame and does not block. The network carries a thumbnail JPEG and
bounded decision JSON, not full-resolution depth or ROS messages.

There is no per-viewer capture/encode loop: a visualization worker publishes
cached JPEGs independently of planning. It keeps three recent display frames
to match image requests by sequence, plus one cached failure image. After five
seconds without live-view requests it stops live JPEG encoding. A stale image
sequence receives HTTP 409 rather than an image from a different decision.

Planner geometry and intrinsics are checked explicitly; calibration dimensions
must match aligned depth. Sparse depth and missing camera frames produce faults.
The existing measured-return/missing-depth policy is preserved, including its
limitations for textureless, dark, or otherwise unmeasured obstacles.

Hypothetical forward speeds preserve the documented 0.35 m/s stall floor and
forward-arc steering. This preview is **not** approval to enable real movement.
See [the hardware stall constraints](rover_low_speed_stall.md).

## Synthetic input and exact failure replay

On a development machine, install only the non-camera requirements:

```bash
python3 -m venv --system-site-packages .preview-env
.preview-env/bin/python -m pip install -r requirements-preview.txt
.preview-env/bin/python -m rover_preview --demo clear
# Other cases: --demo wall, --demo empty
```

The default host is `127.0.0.1`. Synthetic and recorded sources are prominently
labelled **not live** and never import DepthAI.

Choose **View last failure**, then **Download failure for replay**. The `.npz`
bundle contains exact RGB/depth arrays, intrinsics, decision, goal, and effective
settings; it uses no pickled objects. Replay it with:

```bash
.preview-env/bin/python -m rover_preview --replay preview-failure-12.npz
```

Replay loops the recorded frame, restores embedded tuning/transform and the
hypothetical goal, and preserves its original capture timestamp. It is not a
live camera-freshness test. Passing `--config path.yaml` intentionally overrides
the recorded settings for comparison.
The original decision/reason is also shown in a separate dashboard panel:
recomputing geometry from one frame cannot reproduce a camera disconnect or a
timing failure, even though their recorded reason and input remain available.

A plain `.npz` with `rgb` (uint8 BGR, H x W x 3), `depth` (uint16 millimetres,
H x W), and `k` (nine intrinsics) also works, using the normal config and no
initial goal. A fault with no camera frame cannot be exported.
When a plain frame has no recorded capture timestamp, the dashboard shows it as
unavailable rather than inventing a new capture time.

## Optional Pi service

Adjust the user/paths if the Pi checkout differs, then install explicitly:

```bash
sudo cp systemd/rover-preview.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start rover-preview.service
journalctl -u rover-preview.service -f
```

The service conflicts with `rover-camera.service` and masks the configured motor
UART paths. It is not enabled automatically. Camera failure leaves the dashboard
alive for inspection; restart the service after repairing the camera connection.

To return to ROS camera streaming:

```bash
sudo systemctl stop rover-preview.service
sudo systemctl start rover-camera.service
```

## Validation

From the repository root with the preview requirements installed:

```bash
python -m pytest -q tests/test_rover_preview.py \
  tests/test_oak_camera_preview.py \
  ros2_ws/src/rover_avoidance/test/test_planner.py \
  tests/test_camera_stream_protocol.py
```

Before any driving milestone, validate on the actual Pi: mount/intrinsics,
depth quality, processing latency/frame age under load, CPU/RSS with and without
viewers, and physical obstacle visibility. Development-machine tests cannot
certify Pi performance, camera behavior, stopping distance, or collision safety.
