# rover_bridge

`rover_bridge` is the ROS2-side network adapter for the rover. The Raspberry
Pi runs the hardware services and owns the ESP32 UART; this package runs on
the ROS2 computer and translates the Pi TCP protocols into ROS2 commands,
telemetry, compressed RGB-D topics, odometry, and TF.

```text
ROS2 computer                         Raspberry Pi
------------                          ------------
/cmd_vel  --------------------------> TCP :8765 -> rover_agent.py -> UART
/rover/telemetry <------------------- TCP :8766 <- rover_agent.py
compressed RGB-D <------------------ TCP :8770 <- camera_stream.py
```

The bridge does not require ROS2 on the Pi and does not decode camera images
before publishing them. The Pi camera connection is a single-client service;
do not run a raw camera measurement and this node at the same time.

## Requirements

- ROS2 Jazzy (or a compatible ROS2 installation) on the host.
- A reachable Pi running `rover_agent.py` and `camera_stream.py` or their
  systemd services.
- Python ROS2 dependencies declared in [package.xml](package.xml), including
  `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `tf2_ros`,
  PyYAML, NumPy, and OpenCV.
- The canonical calibration file from the repository:
  [config/calibration.yaml](../../../config/calibration.yaml).

The Pi services use these default TCP ports:

| Port | Direction | Service | Payload |
| --- | --- | --- | --- |
| `8765` | host to Pi | command server | newline-delimited JSON velocity commands |
| `8766` | Pi to host | telemetry server | newline-delimited JSON telemetry |
| `8770` | Pi to host | camera server | length-prefixed RGB-D packets |
| `8771` | Pi to host | lidar server | versioned binary 360-bin scans; ROS bridge publishes `/scan` |

## Build and run

From the ROS2 workspace:

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rover_bridge
source install/setup.bash

ros2 run rover_bridge rover_bridge \
  --ros-args \
  --params-file install/rover_bridge/share/rover_bridge/config/rover_bridge.yaml
```

Override the Pi address without editing the YAML file:

```bash
ros2 run rover_bridge rover_bridge \
  --ros-args -p pi_host:=192.168.50.5
```

The checked-in [config/rover_bridge.yaml](config/rover_bridge.yaml) currently
uses `192.168.0.12`. Change it for a fixed deployment or override `pi_host`
at launch time.

## ROS2 interface

### Subscription

| Topic | Type | Behavior |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Uses `linear.x` and `angular.z`; other components are ignored. |

The topic name is configurable with `cmd_vel_topic`. The latest finite
command is sent to the Pi at `command_rate_hz` (20 Hz by default). If no new
command arrives within `cmd_vel_timeout_s` (0.5 seconds), the bridge sends a
zero command. It also sends a zero command when its command socket is being
closed.

Commands below roughly 0.05 m/s per wheel do not produce slow motion on this
rover: the motors stall against floor friction, draw locked-rotor current and
can brown the rover out. Keep requested speeds above that band -- minimum
forward speed is 0.35 m/s. See `docs/rover_low_speed_stall.md`.

### Publications

The optional `rover_lidar_bridge` executable independently connects to Pi TCP
8771 and publishes `/scan` (`sensor_msgs/msg/LaserScan`, sensor-data QoS) plus
`/rover/lidar/status` (JSON `std_msgs/msg/String`). It uses acquisition stamps,
rejects stale/future packets and retains only the latest scan. The
`rover_navigation_gate` executable routes Nav2 commands to `/cmd_vel_safe`;
motion is disabled by default. See the
[lidar integration and testing guide](../../../docs/lidar_ros2_nav2.md).

| Topic | Type | Notes |
| --- | --- | --- |
| `/rover/telemetry` | `std_msgs/msg/String` | Compact JSON copy of each valid telemetry frame. |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Voltage from `voltage_v`, when finite. |
| `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Pi JPEG bytes forwarded unchanged; format `jpeg`. |
| `/camera/depth/image_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | Pi compressed-depth PNG forwarded unchanged; format `16UC1; compressedDepth png`. |
| `/oak/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration attached to each RGB-D packet. |
| `/odom` | `nav_msgs/msg/Odometry` | Bounded planar differential-drive odometry. |
| `/tf` | TF | Broadcasts `odom` to `base_link` by default. |

The compressed image topic names are currently fixed in `node.py`; the
telemetry, battery, camera-info, and odometry topic names are parameters.
Compressed image publishers use reliable QoS. CameraInfo uses the ROS sensor
data QoS profile.


## Keyboard teleop (`rover_teleop`)

A terminal-based `w`/`a`/`s`/`d` teleop that publishes `/cmd_vel`. It resolves
keys into **same-sign wheel speeds** (diagonal arcs) and never produces an
in-place skid turn.

```bash
ros2 run rover_bridge rover_teleop
```

| Keys | Result |
| --- | --- |
| `w` | forward straight |
| `s` | reverse straight |
| `a` | gentle forward-left diagonal arc |
| `d` | gentle forward-right diagonal arc |
| `w`+`a` | tighter forward-left arc |
| `w`+`d` | tighter forward-right arc |
| `s`+`a` | reverse-left arc |
| `s`+`d` | reverse-right arc |
| `space` / `x` | immediate stop |
| `q` | quit (also sends a zero Twist) |

Parameters (CLI or ROS):

```
--drive-speed 0.39      # full-wheel speed for w/s (m/s)
--turn-speed 0.20       # forward creep used when a/d is held alone (m/s)
--turn-ratio 0.153846   # inner/outer wheel ratio (0–1); higher = wider arc
--key-window 0.7        # hold window for detecting simultaneous keys (s)
--rate 20               # publish rate (Hz)
```

Dry run (no ROS, prints the motion table):

```bash
ros2 run rover_bridge rover_teleop --dry-run
```

### Why no in-place spins?

The Pi's `twist_to_wheels` conversion turns `vx=0, wz≠0` into opposite-sign
wheel speeds (left negative, right positive or vice versa). That mode caused
issues in the past. This node instead resolves every command into a **same-sign
wheel pair** first, then inverts back to a Twist. Holding `a`/`d` without `w`
or `s` creeps forward at `--turn-speed` instead of spinning in place, so an
in-place turn is unreachable by construction. A final guard also refuses to
publish any pair that comes out opposite-sign.


## Pi protocols

### Commands on TCP 8765

Commands are newline-delimited UTF-8 JSON:

```json
{"type":"cmd_vel","seq":12,"vx":0.2,"wz":0.0}
```

`seq` must be an integer and `vx` and `wz` must be finite numbers. The Pi
agent converts the command to wheel speeds, applies its own wheel limits and
acceleration limit, and stops on its watchdog if commands stop arriving.

### Telemetry on TCP 8766

Each line is a JSON object with the following shape:

```json
{
  "type": "telemetry",
  "seq": 42,
  "ts": 1710000000.123,
  "data": {"odl": 100, "odr": 100},
  "voltage_v": 7.8,
  "cmd_echo": {"left_mps": 0.2, "right_mps": 0.2, "seq": 12}
}
```

The bridge accepts telemetry only when `type` is `telemetry`. It publishes
the original JSON, checks that `seq` values are contiguous, and discards one
odometry interval after a gap, duplicate, reorder, or invalid sequence.
`cmd_echo` confirms the command placed on the Pi UART write queue; it is not a
hardware acknowledgement from the ESP32.

### Camera packets on TCP 8770

Each packet is:

1. A four-byte unsigned big-endian JSON header length.
2. A JSON header whose `type` is `rgbd` and which contains `rgb_bytes`,
   `depth_bytes`, timestamps, sequence data, and `camera_info`.
3. The RGB payload, followed by the depth payload.

The parser in [protocol.py](rover_bridge/protocol.py) limits headers to 1 MiB
and the combined payload to 64 MiB. It reads complete packets, validates
sizes, and raises on truncated or malformed input. `depth_png_bytes()` accepts
both a bare PNG and a ROS compressedDepth payload with its 12-byte config
header.

The reader and publisher use a latest-frame slot rather than an unbounded
queue. If publication falls behind, older packets are replaced and counted as
queue drops. Every five seconds the node logs receive FPS, publish FPS,
sequence drops, and queue drops.

## Odometry behavior

[odometry.py](rover_bridge/odometry.py) integrates cumulative left and right
encoder distances using a differential-drive model:

- The first sample establishes the encoder and timestamp baseline.
- Encoder units are multiplied by the calibrated left and right metre scales
  and encoder signs.
- Equal wheel motion advances `x`; wheel differences change yaw using the
  calibrated track width.
- A non-positive, too-short, or too-long time step resets the baseline.
- A wheel-distance jump over `max_distance_delta_m` resets the baseline.
- A calculated velocity over either velocity limit resets the baseline.
- A reset preserves the current pose and publishes zero velocity for that
  sample, avoiding reconnect and counter-reset pose spikes.

The live odometry is a provisional motion prior, not navigation-grade ground
truth. Visual registration and loop closure should correct its drift.

The calibration YAML must contain:

```yaml
telemetry_distance_unit: cm
meters_per_left_unit: 0.01
meters_per_right_unit: 0.01
track_width_m: 0.206
left_encoder_sign: 1
right_encoder_sign: 1
```

The bridge requires all six keys. Set `ROVER_CALIBRATION_FILE` or pass the
`calibration_file` ROS parameter when the canonical repository path is not
available. The odometry frames and covariance values are configurable.

## Parameters

The complete default parameter set is in
[config/rover_bridge.yaml](config/rover_bridge.yaml):

| Parameter | Default | Purpose |
| --- | --- | --- |
| `pi_host` | `192.168.0.12` | Pi address in the checked-in profile. |
| `command_port` / `telemetry_port` / `camera_port` | `8765` / `8766` / `8770` | Pi TCP endpoints. |
| `command_rate_hz` | `20.0` | Command resend rate. |
| `reconnect_delay_s` | `1.0` | Delay before reconnecting a failed socket. |
| `socket_timeout_s` | `1.0` | Command and telemetry socket timeout. |
| `camera_socket_timeout_s` | `10.0` | Camera socket timeout for larger RGB-D packets. |
| `cmd_vel_topic` / `cmd_vel_timeout_s` | `/cmd_vel` / `0.5` | Input topic and stale-command stop time. |
| `telemetry_topic` / `battery_topic` | `/rover/telemetry` / `/battery_state` | Low-rate telemetry outputs. |
| `camera_info_topic` | `/oak/rgb/camera_info` | Per-frame camera calibration output. |
| `odom_topic` | `/odom` | Odometry output. |
| `odom_frame_id` / `base_frame_id` | `odom` / `base_link` | Odometry and TF frame names. |
| `calibration_file` | auto-detected | Encoder scales, signs, and track width. |
| `max_distance_delta_m` | `2.0` | Maximum accepted wheel delta per sample. |
| `min_odom_dt_s` / `max_odom_dt_s` | `0.005` / `0.5` | Accepted telemetry time-step range. |
| `max_linear_velocity_mps` / `max_angular_velocity_rps` | `1.0` / `5.0` | Odometry spike limits. |
| `odom_pose_covariance` / `odom_twist_covariance` | `0.05` / `0.1` | Diagonal covariance values for x, y, yaw and their velocities. |
| `frame_id` | `oak_rgb_camera_optical_frame` | Camera image and CameraInfo frame. |

## Tests

The package has focused tests for command freshness and telemetry sequence
handling, compressed message construction, odometry integration and reset
behavior, and the TCP protocol parser:

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select rover_bridge
colcon test-result --verbose
```

The package files are [node.py](rover_bridge/node.py),
[protocol.py](rover_bridge/protocol.py), [odometry.py](rover_bridge/odometry.py),
and [setup.py](setup.py). The console entry point is
`rover_bridge = rover_bridge.node:main`.

## Operational cautions

- Only one process may own the Pi UART. Do not run the factory app,
  calibration tools, standalone recorder, and production agent together.
- The Pi camera service accepts one TCP client. Stop the bridge before a raw
  port-8770 measurement.
- Confirm telemetry and camera connections before sending motion commands.
- Send an explicit zero `/cmd_vel` before stopping a command publisher.
- A killed bridge should cause the Pi command socket to close and the Pi
  watchdog to stop the wheels; verify this behavior on the real rover before
  autonomous use.