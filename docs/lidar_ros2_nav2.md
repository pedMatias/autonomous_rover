# Lidar sharing, ROS2 mapping and gated Nav2 testing

## Architecture and limits

The Pi runs `python -m rover_lidar`: serial acquisition and complete scan
assembly, then uncompressed binary TCP on port **8771**. WSL2 runs
`rover_lidar_bridge`, which publishes **`/scan` (`sensor_msgs/msg/LaserScan`)**.
The camera remains on TCP 8770 and keeps publishing compressed RGB/depth.
The lidar process never opens the motor UART or camera.

**D500 / STL-19P status: serial packets and scan assembly were validated from
the directly attached CP2102 USB-UART adapter.** A receive-only capture on
2026-10-10 produced thousands of 47-byte `54 2c` packets accepted by this
reader with zero CRC failures, then assembled complete 360-bin scans at about
10 Hz with valid returns. This empirically confirms compatibility of the
observed D500 stream with this LD06/LD19-style `--protocol ld06` decoder; it
does not validate every D500 firmware revision, the rover's onboard
audio-board UART path, scan orientation/mounting, or moving-map quality.

Use the stable `/dev/serial/by-id/` path for the directly attached CP2102
adapter (observed as `usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`),
not the onboard QinHeng `/dev/ttyACM0` path: the latter enumerated but returned
zero bytes in the same test. The D500 data rate is supported by USB 2.0; a
USB 3.x port also works, and port color is not a reliable data-path test.
Proceed with stationary ROS `/scan` validation before mapping or navigation.

The source is the sensor's serial stream. Timestamps are estimates of
acquisition time from Pi reception and packet angular speed, not an exact
hardware-clock synchronization. Scans are re-binned to 360 angular slots,
with the nearest return retained; missing, zero, low-confidence and
out-of-range returns become `NaN`, never invented free space. Partial startup
revolutions are discarded. Coverage below 75% or durations outside
0.02-0.5 seconds are rejected. The ROS scan uses `time_increment=0` because
angular bin order is not acquisition order. There is no per-ray motion
deskew; verify moving-map quality before relying on navigation.

Each scan has a bounded JSON header and little-endian uint16 ranges plus
uint8 confidence values. A 360-bin packet is about 1.3 KB, or about 12.7 KB/s
at 10 Hz. A live eight-second CP2102 capture measured about 19.5 KB/s raw
UART traffic. Neither link needs compression. Only the latest assembled
scan is retained; slow consumers disconnect rather than accumulating an
application queue. Receivers reject old/future acquisition stamps and
reconnect.

The TCP endpoint is unauthenticated: use only a trusted rover LAN, restrict
the firewall to your ROS host, and do not expose it to the Internet.

## 1. Confirm hardware and ownership (Pi; no driving)

Identify the model from the label/manufacturer documentation, and inspect USB:

```bash
ls -l /dev/serial/by-id/
lsusb
```

For Waveshare D500/STL-19P, the official [datasheet](https://files.waveshare.com/wiki/D500-LiDAR-Kit/LDROBOT_STL-19P_Datasheet_EN_v1.0(1).pdf)
specifies 230400 baud, 8N1, no flow control, one-way UART transmission and
automatic data output after the motor stabilizes. It specifies TX on pin 1,
PWM on pin 2, ground on pin 3 and +5 V on pin 4. **When external motor-speed
control is not used, its datasheet says PWM must be grounded. However, the
current [Waveshare D500 wiki](https://www.waveshare.com/wiki/D500_LiDAR_Kit)
says a floating/high-resistance PWM input selects internal 10 Hz control.
These manufacturer sources conflict. Verify the Rover board schematic or get
written Waveshare guidance before changing the PWM connection; do not add or
remove a jumper based on either statement alone. Also verify the actual
connector orientation and whether the board already controls PWM. Waveshare's
[UGV Rover product page](https://www.waveshare.com/ugv-rover.htm) describes
the audio board's lidar UART-to-USB path.

Confirm packet protocol, clockwise/counterclockwise angle direction, valid
range limits, and the physical forward reference. `clockwise` means
increasing sensor angles go clockwise viewed from above; ROS angles increase
counterclockwise about positive Z. The driver does not apply the legacy
hard-coded 180-degree offset: mounting yaw belongs in TF.

Leave `base_config.use_lidar: false` in [config.yaml](../config.yaml).
The production motor agent explicitly disables the legacy lidar and extra-sensor readers,
so it can coexist with this service even if the factory setting changes.
Do not run another lidar reader simultaneously. The service uses an exclusive
serial lock. The existing single-owner motor/camera rules still apply.

Install the small Pi dependency set in the existing environment:

```bash
cd /home/ws/ugv_rpi
ugv-env/bin/python -m pip install -r requirements-lidar.txt
```

For the D500 direct CP2102 connection validated above, run manually:

```bash
ugv-env/bin/python -m rover_lidar \
  --protocol ld06 \
  --device /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --direction clockwise \
  --range-min 0.05 --range-max 12.0
```

The CP2102 path is specific to the adapter observed during the live test; check
`/dev/serial/by-id/` and substitute its actual stable path if it differs. The
onboard audio-board serial adapter did not produce data in that test. The
example limits follow the D500's documented 0.03-12 m range, with a 0.05 m
minimum chosen for the receiver. Do not change connector wiring or PWM based
on port color or this software test.
Expected logs: port opened and stream listening on port 8771. CRC warnings,
coverage warnings or repeated reconnects are failures to investigate, not
successful operation. No valid packet for two seconds triggers reconnection.
Check permissions (`dialout`) if opening the device fails.

The verified CP2102 settings are in the Pi's local, git-ignored
`config/lidar.env`. Install and start the publisher:

```bash
sudo install -m 644 systemd/rover-lidar.service /etc/systemd/system/rover-lidar.service
sudo systemctl daemon-reload
sudo systemctl enable --now rover-lidar.service
systemctl --no-pager status rover-lidar.service
journalctl -u rover-lidar.service -f
```

The service listens on TCP `0.0.0.0:8771`; connect to the Pi's LAN address
from WSL2. Keep this unauthenticated endpoint on a trusted LAN. It is
independent of `rover-agent.service` and `rover-camera.service`; enabling it
does not enable driving.

## 2. Prepare ROS2 Jazzy (WSL2)

Install the navigation dependencies on WSL2, not on the Pi:

```bash
sudo apt update
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-rviz2
```

In **every** ROS terminal, use your checkout's path:

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
colcon build --symlink-install --packages-select rover_bridge rover_slam
source install/setup.bash
```

Synchronize Pi and PC clocks using their normal time service. Verify the
offset is comfortably below **50 ms** (preferably below 10 ms); the bridge
rejects scans more than 50 ms in the future or 500 ms old. Do not remove the
freshness check to hide a clock mismatch. WSL's clock can drift after host
sleep; recheck after resume. `use_sim_time` must be false for live acquisition.

## 3. Stationary `/scan` and RViz test

Measure the lidar pose relative to `base_link`: X forward, Y left, Z up,
distances in metres and roll/pitch/yaw in radians. Set shell variables to
those measured values, for example `LASER_X`, `LASER_Y`, `LASER_Z`,
`LASER_ROLL`, `LASER_PITCH`, `LASER_YAW`. No mounting values are guessed by
the launch: all six must be supplied.

For **lidar only**, without the motor or camera bridge:

```bash
ros2 launch rover_slam lidar.launch.py pi_host:=192.168.0.12 \
  laser_x:="$LASER_X" laser_y:="$LASER_Y" laser_z:="$LASER_Z" \
  laser_roll:="$LASER_ROLL" laser_pitch:="$LASER_PITCH" laser_yaw:="$LASER_YAW"
```

In another initialized ROS terminal:

```bash
ros2 topic info /scan --verbose
ros2 topic hz /scan --qos-reliability best_effort
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /rover/lidar/status
ros2 run tf2_ros tf2_echo base_link laser_frame
rviz2
```

In RViz set Fixed Frame to `base_link`, add **LaserScan `/scan`**, and set
its reliability to **Best Effort**. Check a measured target straight ahead,
then targets to the left and right. Verify distances in metres, orientation,
mounting offset, stable scan rate, valid returns, and low scan age. Do not
proceed if mirrored or rotated. Very dark/reflective surfaces may have missing
returns; missing returns are not evidence of free space.

Stop lidar-only launch before starting combined bringup:

```bash
ros2 launch rover_slam live_bringup.launch.py pi_host:=192.168.0.12 \
  enable_lidar:=true \
  laser_x:="$LASER_X" laser_y:="$LASER_Y" laser_z:="$LASER_Z" \
  laser_roll:="$LASER_ROLL" laser_pitch:="$LASER_PITCH" laser_yaw:="$LASER_YAW"
```

The existing Pi motor and camera services should be running for this stage;
do not run the factory app or local autonomy simultaneously.
Verify `/scan`, `/odom`, camera RGB/depth and `/oak/rgb/camera_info` coexist.
The GCS launch accepts the same `enable_lidar` and mounting arguments;
this change does not add a lidar plot to its web UI.

## 4. Record and build a 2D map

With combined bringup running, start mapping in another terminal:

```bash
ros2 launch rover_slam lidar_mapping.launch.py
ros2 lifecycle get /slam_toolbox
```

Expected state: `active`. In RViz use Fixed Frame `map`, add Map `/map`,
LaserScan `/scan`, and TF. Mapping publishes `map -> odom`; the rover bridge
publishes `odom -> base_link`; lidar bringup publishes `base_link -> laser_frame`.
Do not run AMCL or RTAB-Map's map transform publisher concurrently.

For manual mapping use the existing keyboard teleop in its own ROS terminal:

```bash
ros2 run rover_bridge rover_teleop
```

Follow [the stall constraints](rover_low_speed_stall.md): never creep, never
rotate in place, keep the area clear and the physical switch accessible.
The navigation command gate is not part of this manual-mapping launch.

Record to a **new** bag path from the repository root:

```bash
ros2 bag record -o recordings/lidar_room_01 --topics \
  /scan /odom /tf /tf_static /cmd_vel /rover/telemetry /battery_state \
  /camera/color/image_raw/compressed \
  /camera/depth/image_raw/compressedDepth /oak/rgb/camera_info
```

Stop the rover, stop recording, and save the occupancy map before stopping SLAM:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/lidar_room_01"
```

Run map saving from the repository root, not `ros2_ws`, to use the intended
`maps/` directory. Map YAML/image files are occupancy maps, not RTAB-Map databases.

For RViz replay, stop live bringup and mapping, play the bag with `--clock`
and set RViz `use_sim_time=true`. Do not start the live lidar receiver or
motor bridge during replay. This bag includes the mapper's `map -> odom` TF:
do not replay it into a second mapper without first filtering that transform,
or there will be two map-transform authorities. The mapping launch supports
`use_sim_time:=true` for appropriately prepared offline sensor/odometry bags.

## 5. Nav2 dry test: motion disabled

Stop mapping, combined bringup, GCS and teleop first. This launch starts its
own motor/camera bridge, lidar receiver, static transforms, AMCL, Nav2 and
command gate. Its bridge listens **only on `/cmd_vel_safe`**.

```bash
ros2 launch rover_slam lidar_navigation.launch.py \
  map:=/ABSOLUTE/PATH/maps/lidar_room_01.yaml pi_host:=192.168.0.12 \
  laser_x:="$LASER_X" laser_y:="$LASER_Y" laser_z:="$LASER_Z" \
  laser_roll:="$LASER_ROLL" laser_pitch:="$LASER_PITCH" laser_yaw:="$LASER_YAW"
```

Default `motion_enabled=false` means the gate sends **zero commands only**.
In RViz select Fixed Frame `map`, add the map, scan, global/local costmaps
and planned path. Set the rover's initial pose using **2D Pose Estimate**.
Add a Navigation2 panel or use **Nav2 Goal** to submit a goal.

Inspect:

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 topic info /cmd_vel_safe --verbose
ros2 topic echo /cmd_vel_safe
ros2 topic echo /nav2/cmd_vel
ros2 topic echo /rover/navigation/status
ros2 run tf2_ros tf2_echo map base_link
```

The gate must be the only publisher on `/cmd_vel_safe`, and the bridge must
subscribe there. Nav2 may calculate nonzero commands, but `/cmd_vel_safe`
must remain zero while disabled. A non-driving goal can eventually fail the
progress checker; that is expected.

The supplied configuration is a starting point, not a hardware certification:
measure footprint before arming (both costmaps use a conservative `robot_radius`
placeholder of 0.4 m), verify sensor limits, track width and odometry,
and test costmap obstacles at known positions. Edit a copy of
[nav2_lidar.yaml](../ros2_ws/src/rover_slam/config/nav2_lidar.yaml), passed with
`params_file:=/absolute/path/to/your.yaml`, if required.

The controller uses forward arcs only, no reverse, no rotate-to-heading, and
no spin/backup recoveries. Final goal yaw is deliberately ignored.
Unsafe low-speed or tight-turn commands are **stopped**, not boosted upward.
A goal/path that needs a tighter turn than the wheel limits allow will fail;
change the path/controller tuning or reconsider the hardware rather than
lowering the safety limits. There is no velocity smoother that would
deliberately generate intermediate creep commands. The Pi's existing motor
acceleration limit still applies: verify its real startup behavior before
approving autonomous operation.

## 6. Supervised motion and failure-stop tests

Only after stationary and dry tests pass, with a charged battery, open floor,
no bystanders and someone at the physical switch, restart navigation with:

```bash
ros2 launch rover_slam lidar_navigation.launch.py \
  map:=/ABSOLUTE/PATH/maps/lidar_room_01.yaml pi_host:=192.168.0.12 \
  laser_x:="$LASER_X" laser_y:="$LASER_Y" laser_z:="$LASER_Z" \
  laser_roll:="$LASER_ROLL" laser_pitch:="$LASER_PITCH" laser_yaw:="$LASER_YAW" \
  motion_enabled:=true hardware_verified:=true
```

The extra confirmation is required by the launch. It is an operator
acknowledgement, not an automatic hardware verification. Choose a short,
straight-ahead goal first. The gate requires fresh scans with at least 50%
valid ranges, fresh finite odometry and valid map/laser transforms.
Both wheels must be forward and within 0.35-0.50 m/s.

Before increasing scope, measure each failure mode while ready to stop power:

| Fault | Expected result |
| --- | --- |
| Stop the Pi lidar service | `/scan` stops; gate outputs zero within 0.5 s of the last accepted scan plus its 50 ms timer period. |
| Disconnect lidar USB | Same stop behavior; explicit Pi error logs and reconnection attempts. |
| Stop Nav2's command publisher | Gate outputs zero after its 0.3 s command timeout plus timer period. |
| Lose odometry/localization transforms | Gate outputs zero; status reports odometry/transform unavailable. |
| Lose PC-to-Pi command connection or kill the gate/bridge | Existing bridge/Pi command watchdog stops motors; verify actual stopping distance. |
| Add an obstacle in a controlled test | Obstacle appears in costmaps and controller stops/replans before contact. |

Use `ros2 topic echo /cmd_vel_safe` and actual wheel motion to verify stops;
zero ROS commands alone do not prove mechanical stopping. Sensor freshness
is not a certified emergency stop. At 0.45 m/s, a 0.55 s detection window is
about 0.25 m of travel **before braking**. Allow adequate margins.
Do not disable these gates to make an unachievable goal move.

Record `/nav2/cmd_vel`, `/cmd_vel_safe`, `/rover/navigation/status` and
`/rover/lidar/status` along with the sensor topics during navigation tests.

## Automated validation

From repository root (no powered hardware required):

```bash
PYTHONPATH=.:ros2_ws/src/rover_bridge python -m pytest -q \
  tests/test_rover_lidar.py tests/test_rover_agent.py \
  tests/test_camera_stream_protocol.py \
  ros2_ws/src/rover_bridge/test/test_navigation_safety.py \
  ros2_ws/src/rover_bridge/test/test_protocol.py \
  ros2_ws/src/rover_bridge/test/test_odometry.py
```

On WSL2 after sourcing Jazzy and the built workspace:

```bash
colcon test --packages-select rover_bridge rover_slam
colcon test-result --verbose
```

ROS-specific message/gate and launch tests require Jazzy and are skipped if
ROS is unavailable. They do not replace RViz, real QoS/TF checks, Nav2 plugin
activation or physical failure-stop tests.
