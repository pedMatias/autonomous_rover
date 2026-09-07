# Offline RTAB-Map First Database Plan

**Project:** autonomous rover  
**Scope:** first usable RTAB-Map database, created from the Raspberry Pi and WSL2 code in this repository  
**Plan date:** 2026-09-01

## 1. Outcome

The immediate deliverable is a saved RTAB-Map database created from a controlled indoor RGB-D recording:

```text
recordings/<run>/  ->  ros2 bag play --clock  ->  RTAB-Map mapping mode
                                              ->  maps/<run>.db
```

The first database must be reproducible, inspectable, and reloadable. It does not need to solve autonomous navigation yet.

The recommended first route is **offline mapping from a Pi bag**. This remains the shortest reliable route because the repository already contains a Pi-side recorder that writes raw RGB, aligned depth, camera info, wheel odometry, and `odom -> base_link` TF into one bag. The WSL2 bridge now also integrates the live encoder telemetry and publishes `/odom` plus `odom -> base_link`, so direct WSL2 recording is available after the live motion gate passes.

## 2. Current Project State

### 2.1 Working pieces

| Area | Current state | Evidence |
| --- | --- | --- |
| OAK-D RGB-D capture | Implemented on the Pi | `recorder/oak_camera.py` |
| RGB/depth synchronization | Implemented by the OAK-D `Sync` node | `recorder/oak_camera.py` |
| Depth registration | Depth is aligned to RGB with `setDepthAlign(CAM_A)` | `recorder/oak_camera.py` |
| Camera calibration | Intrinsics are read from the OAK-D at runtime | `recorder/oak_camera.py` |
| Encoder feedback | ESP32 feedback contains cumulative `odl` and `odr` counts | `recorder/base_poller.py`, `rover_agent.py` |
| Wheel odometry integrator | Implemented in the WSL2 bridge with reset/jump protection; Pi recorder retains its local fallback | `ros2_ws/src/rover_bridge/rover_bridge/odometry.py`, `recorder/odometry.py` |
| Bag writer | Writes ROS2 bag messages for RGB, depth, camera info, odometry, and TF | `recorder/bag_writer.py` |
| Pi camera network stream | Validated at approximately 14.5 FPS raw TCP | `camera_stream.py`, Phase 1 measurements |
| WSL compressed camera topics | Validated at approximately 14 FPS | `ros2_ws/src/rover_bridge/rover_bridge/node.py` |
| ROS2 compressedDepth decoding | Validated after adding the Pi compressedDepth header | `camera_stream.py`, `camera_stream_protocol.py` |
| RTAB-Map packages | Installed in the current WSL2 Jazzy environment | `rtabmap_ros`, `rtabmap_launch`, `rtabmap_slam`, `rtabmap_sync` |
| ROS2 SLAM workflow | Live bringup, static camera TF, offline RTAB-Map launch, and bag validator implemented | `ros2_ws/src/rover_slam`, `tools/validate_rgbd_bag.py` |

### 2.2 Current blockers for a live mapping run

The live `rover_bridge` publishes:

```text
/rover/telemetry              std_msgs/msg/String
/battery_state                sensor_msgs/msg/BatteryState
/camera/color/image_raw/compressed
/camera/depth/image_raw/compressedDepth
/oak/rgb/camera_info          sensor_msgs/msg/CameraInfo
/odom                         nav_msgs/msg/Odometry
/tf                           odom -> base_link
```

It does **not** publish the fixed camera transform itself:

```text
/tf_static                    base_link -> camera frame
```

The `rover_slam/live_bringup.launch.py` launch file supplies that static transform and republishers for direct WSL2 recording. The Pi recorder remains the preferred first capture path because it avoids network loss and writes raw RGB-D locally.

### 2.3 Existing calibration status

The current `config/calibration.yaml` contains:

```text
meters_per_tick_left:  0.034664082687338504
meters_per_tick_right: 0.03631593874078276
track_width_m:         0.33185
left_encoder_sign:     1
right_encoder_sign:    1
```

It contains two straight trials and two arc trials. The straight-derived wheel scales are close between trials, but the two independently derived track widths are `0.3033 m` and `0.3604 m`. Their average is the current `0.33185 m`, but the spread is large enough that this calibration should be treated as **provisional** until more trials are repeated.

The existing gyro bias is useful for future IMU fusion, but `recorder/base_poller.py` currently integrates encoder counts only. The first map therefore uses wheel odometry as the external motion estimate and does not claim to use the IMU.

## 3. Target Data Flow for the First Map

```text
Pi OAK-D
  -> synchronized RGB + aligned uint16 depth
  -> recorder/record.py

Pi ESP32
  -> encoder feedback odl/odr
  -> OdometryIntegrator

Pi recorder
  -> /oak/rgb/image_raw       sensor_msgs/Image, bgr8
  -> /oak/stereo/image_raw    sensor_msgs/Image, 16UC1 millimetres
  -> /oak/rgb/camera_info     sensor_msgs/CameraInfo
  -> /odom                    nav_msgs/Odometry
  -> /tf                      odom -> base_link

WSL2
  -> rover_slam/live_bringup.launch.py (optional direct capture)
  -> ros2 bag record, or Pi bag transfer
  -> ros2 bag play --clock
  -> rover_slam/offline_mapping.launch.py
  -> RTAB-Map mapping mode
  -> maps/<run>.db
```

The offline bag uses raw images deliberately. The network compressed transport is already validated, but it is not necessary for the first database because the Pi recorder and the OAK-D are on the same Pi during capture.

The intended frame tree is:

```text
map                         RTAB-Map publishes during mapping
  |
  +-- odom                  recorded wheel odometry frame
        |
        +-- base_link       rover body frame
              |
              +-- oak_rgb_camera_optical_frame
                                measured fixed camera transform
```

The camera must remain physically fixed during the first recording. If it is mounted on a pan/tilt mechanism, center and lock pan/tilt before recording. A moving camera requires a dynamic gimbal TF and is a separate task.

## 4. Phase 0: Freeze and Verify the Environment

### 4.1 Synchronize the two checkouts

The Pi must receive the same revision as WSL2. The current Pi-side camera change is only available to the Pi after it has been committed and pushed.

From WSL2:

```bash
cd ~/projects/autonomous_rover
git status --short
git log -1 --oneline
```

Push the intended branch/commit. Do not overwrite unrelated local changes on either machine.

On the Pi:

```bash
export PI_REPO=/home/ws/ugv_rpi
cd "$PI_REPO"
git status --short
git pull --ff-only
git log -1 --oneline
```

Verify the Pi-side helper and camera files:

```bash
"$PI_REPO/ugv-env/bin/python" -m py_compile \
  camera_stream.py camera_stream_protocol.py \
  recorder/oak_camera.py recorder/record.py
```

### 4.2 Verify WSL2 packages

Run in every WSL2 ROS2 terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
```

The current WSL2 environment already contains these packages:

```bash
for package in \
  rtabmap_ros rtabmap_launch rtabmap_slam rtabmap_sync \
  image_transport compressed_image_transport compressed_depth_image_transport \
  robot_localization tf2_ros tf2_tools nav2_bringup; do
  printf '%-40s' "$package"
  ros2 pkg prefix "$package" 2>/dev/null || echo NOT_FOUND
done
```

`pointcloud_to_laserscan` is not currently installed, but it is not required to create the first RTAB-Map database.

### 4.3 Establish the camera frame and extrinsics

Use one camera frame name everywhere in the first-map path:

```text
oak_rgb_camera_optical_frame
```

This is the frame name already used by `recorder/bag_writer.py`.

Measure and record the camera transform relative to `base_link`:

```text
camera_x_m:
camera_y_m:
camera_z_m:
camera_roll_rad:
camera_pitch_rad:
camera_yaw_rad:
```

Do not copy example values from documentation. Measure the actual mounting point and orientation. If the camera is on a gimbal, lock it at the documented center position and use that fixed pose.

## 5. Phase 1: Validate Calibration and Motion

### 5.1 Stop competing Pi processes

The recorder directly owns the Pi UART and the OAK-D. Stop the production services first:

```bash
cd "$PI_REPO"
sudo systemctl stop rover-camera.service rover-agent.service
pkill -9 -f app.py || true
pgrep -af 'app.py|rover_agent.py|camera_stream.py' || true
```

No `app.py`, `rover_agent.py`, calibration script, link diagnostic, or recorder may share `/dev/ttyAMA0`.

### 5.2 Repeat calibration before the serious mapping run

The current values are sufficient for a short software smoke test, but repeat the physical calibration before collecting the definitive map dataset.

Run the following interactively on the Pi, one trial at a time, in a clear area:

```bash
cd "$PI_REPO"
"$PI_REPO/ugv-env/bin/python" pedro_scripts/calibrate_imu.py --duration 30
```

Run at least five straight trials:

```bash
"$PI_REPO/ugv-env/bin/python" pedro_scripts/calibrate_odometry.py \
  straight --speed 0.39 --duration 4
```

Run at least five diagonal arc trials in both directions. Use the same safe-sign convention as the existing tool:

```bash
"$PI_REPO/ugv-env/bin/python" pedro_scripts/calibrate_odometry.py \
  arc --left 0.06 --right 0.39 --duration 4

"$PI_REPO/ugv-env/bin/python" pedro_scripts/calibrate_odometry.py \
  arc --left 0.39 --right 0.06 --duration 4
```

Review the result:

```bash
"$PI_REPO/ugv-env/bin/python" pedro_scripts/calibrate_odometry.py summary
sed -n '1,120p' config/calibration.yaml
```

Acceptance for the calibration gate:

- Encoder signs are confirmed by a forward movement test.
- Both wheel scales are positive and do not change materially between trials.
- Track-width estimates are repeatable enough to explain the recorded arcs.
- The rover stops immediately after every trial.
- The calibration file is backed up before changing it.

The track width does not need to be perfect for RTAB-Map to work; visual registration and loop closures can correct wheel-odometry drift. It must be plausible and finite.

### 5.3 Confirm the motion protocol before recording

The recorder is the safest first capture path because it has its own timeout and zero-command cleanup. Start with a short run rather than a full mapping run:

```bash
cd "$PI_REPO"
"$PI_REPO/ugv-env/bin/python" recorder/record.py \
  --out recordings/smoke_$(date +%Y%m%d_%H%M%S) \
  --fps 10 \
  --duration 30
```

Use the keyboard controls shown by the recorder. Confirm that:

- `w` moves forward.
- `s` moves backward.
- `a` and `d` produce safe same-sign diagonal arcs.
- `space` stops.
- `q` or `Ctrl+C` stops, flushes, and closes the bag.
- The printed pose changes when the rover moves.

Inspect the smoke bag on WSL2 before committing to a long run.

## 6. Phase 2: Capture the First Mapping Bag

### 6.1 Prepare the indoor environment

Choose a controlled area with:

- Good, stable lighting.
- Textured walls, furniture, corners, or other features for visual matching.
- A safe loop or repeated route so RTAB-Map can detect revisits.
- Enough space to drive without collisions.
- No moving people in the first dataset.
- The camera at a fixed height and orientation.

The first useful dataset should cover approximately `50-100 m2` for `5-10 minutes`, with deliberate overlap. Drive slowly and revisit at least three recognizable places.

### 6.2 Start the recorder

On the Pi, with `rover-camera.service` and `rover-agent.service` stopped:

```bash
cd "$PI_REPO"
RUN_ID="indoor_map_$(date +%Y%m%d_%H%M%S)"
"$PI_REPO/ugv-env/bin/python" recorder/record.py \
  --out "recordings/$RUN_ID" \
  --fps 10 \
  --rgb-width 640 \
  --rgb-height 480 \
  --drive-speed 0.20 \
  --turn-ratio 0.5
```

The output path must not already exist. The recorder prints the current frame rate, pose, bag size, and free disk space. Stop with `q` after the route has been completed, or `Ctrl+C` in an emergency.

The recorder produces these topics:

```text
/oak/rgb/image_raw       bgr8 RGB
/oak/stereo/image_raw    16UC1 depth in millimetres
/oak/rgb/camera_info     OAK-D intrinsics
/odom                    wheel odometry
/tf                      odom -> base_link
```

### 6.3 Shut down the Pi recording process safely

After the recorder exits:

```bash
cd "$PI_REPO"
find "recordings/$RUN_ID" -maxdepth 3 -type f -printf '%p %s bytes\n' | sort
ls -lh "recordings/$RUN_ID"
```

Do not restart the production services until the recorder has released the OAK-D and UART. When the bag has been verified or copied, restore the normal services if needed:

```bash
sudo systemctl enable --now rover-agent.service rover-camera.service
```

## 7. Phase 3: Transfer and Validate the Bag on WSL2

### 7.1 Copy the bag

From WSL2, use the Pi account and address appropriate to the network:

```bash
export PI_USER=ws
export PI_HOST=192.168.0.12
export PI_RUN=/home/ws/ugv_rpi/recordings/<RUN_ID>
export WSL_RUN="$HOME/projects/autonomous_rover/recordings/<RUN_ID>"

mkdir -p "$(dirname "$WSL_RUN")"
rsync -a --info=progress2 "$PI_USER@$PI_HOST:$PI_RUN/" "$WSL_RUN/"
```

If `rsync` is unavailable, use `scp -r` instead. Do not delete the source bag until the WSL copy is verified.

### 7.2 Inspect bag metadata

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 bag info "$WSL_RUN"
```

The bag must contain at least:

```text
/oak/rgb/image_raw
/oak/stereo/image_raw
/oak/rgb/camera_info
/odom
/tf
```

Check that RGB and depth have the expected dimensions and encodings. Check that `/odom` and `/tf` contain many samples, not only one initialization message.

Check the camera calibration matrix explicitly: `fx = K[0]`, `fy = K[4]`,
`cx = K[2]`, and `cy = K[5]` must be finite and physically plausible. The
distortion model and coefficient vector must also be present.

If `ros2 bag info` fails, stop here and repair the bag writer/metadata compatibility before trying RTAB-Map. Do not debug RTAB-Map against an unreadable bag.

### 7.3 Check the frame and timestamp assumptions

During a short replay, inspect the topics:

```bash
ros2 bag play "$WSL_RUN" --clock
```

In another terminal:

```bash
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/stereo/image_raw
ros2 topic hz /odom
ros2 topic echo --once /oak/rgb/camera_info
ros2 topic echo --once /odom
ros2 topic echo --once /tf
```

For replay, every consumer that needs ROS time must use simulation time. Stop the replay before the next test.

Known current limitation: existing bags contain `/tf` for `odom -> base_link`, but do not yet contain a recorded `/tf_static` transform for the camera. Until the recorder is hardened, publish the measured camera transform during replay as described below.

## 8. Phase 4: Static Camera TF

Start this in a WSL2 terminal before RTAB-Map and before bag playback. Replace every placeholder with the measured camera pose:

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 run tf2_ros static_transform_publisher \
  --x CAMERA_X_M \
  --y CAMERA_Y_M \
  --z CAMERA_Z_M \
  --roll CAMERA_ROLL_RAD \
  --pitch CAMERA_PITCH_RAD \
  --yaw CAMERA_YAW_RAD \
  --frame-id base_link \
  --child-frame-id oak_rgb_camera_optical_frame
```

Verify the transform:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link oak_rgb_camera_optical_frame
```

Acceptance:

- `base_link` is the parent.
- `oak_rgb_camera_optical_frame` is the child.
- The transform is stable and physically plausible.
- There is no second competing transform for the same child frame.

## 9. Phase 5: First Offline RTAB-Map Run

### 9.1 Use a unique database path

Do not reuse a database from another experiment:

```bash
export MAP_ID=indoor_map_$(date +%Y%m%d_%H%M%S)
export MAP_DB="$HOME/projects/autonomous_rover/maps/$MAP_ID.db"
mkdir -p "$(dirname "$MAP_DB")"
```

### 9.2 Start RTAB-Map in mapping mode

In a WSL2 terminal with the static TF process running:

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 launch rover_slam offline_mapping.launch.py \
  database_path:="$MAP_DB" \
  camera_x:=CAMERA_X_M \
  camera_y:=CAMERA_Y_M \
  camera_z:=CAMERA_Z_M \
  camera_roll:=CAMERA_ROLL_RAD \
  camera_pitch:=CAMERA_PITCH_RAD \
  camera_yaw:=CAMERA_YAW_RAD \
  localization:=false \
  rtabmap_viz:=true \
  rviz:=true \
  output:=screen
```

Important choices:

- `use_sim_time:=true` matches `ros2 bag play --clock`.
- `visual_odometry:=false` prevents a second odometry source from competing with the recorded wheel odometry.
- `rgbd_sync:=true` makes RTAB-Map synchronize RGB, depth, and camera info.
- `approx_rgbd_sync:=true` tolerates the small timestamp difference between independently queued bag messages.
- `frame_id:=base_link` matches the recorded odometry child frame.
- `database_path` is unique for this run.

The repository launch wrapper supplies the static camera TF and forwards the
raw bag topics to the installed `rtabmap_launch` package. The actual mapping
executable is supplied by `rtabmap_slam`. Do not use a nonexistent
`rtabmap_ros/rtabmap` executable directly.

### 9.3 Start bag playback only after RTAB-Map is ready

In another WSL2 terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 bag play "$WSL_RUN" --clock
```

Start RTAB-Map before the bag so it does not miss the initial camera-info or TF messages. Let playback finish. Then stop RTAB-Map cleanly with `Ctrl+C` so it closes the database.

## 10. Phase 6: Map Acceptance Gate

The first mapping run passes only when all of these are true:

### Input gate

```text
/oak/rgb/image_raw       messages received
/oak/stereo/image_raw    messages received
/oak/rgb/camera_info     messages received
/odom                    messages received
/tf                      odom -> base_link available
base_link -> camera      static TF available
```

### Runtime gate

- RTAB-Map starts without a missing-transform error.
- No `TF_OLD_DATA`, invalid timestamp, or repeated queue starvation errors occur.
- The RTAB-Map log shows RGB-D frames being processed.
- The database path is the intended unique path.
- The bag reaches the end without a serialization or DDS error.

### Map gate

```bash
ls -lh "$MAP_DB"
ros2 topic list | grep -E 'rtabmap|grid_map|cloud_map|mapData'
ros2 topic echo --once /rtabmap/info
```

The database must be non-empty and reloadable. RViz or RTAB-Map UI should show:

- A growing pose graph.
- A 3D cloud on revisited surfaces.
- A 2D grid map when the scene contains suitable ground/wall geometry.
- At least three plausible loop closures on a route that deliberately revisits places.

The exact number of loop closures is environment-dependent. A database file larger than 1 MB by itself is not sufficient evidence of a good map.

## 11. Phase 7: Reload and Localization Smoke Test

After the mapping process has been stopped, run the same bag again with the saved database in localization mode:

```bash
ros2 launch rover_slam offline_mapping.launch.py \
  database_path:="$MAP_DB" \
  camera_x:=CAMERA_X_M \
  camera_y:=CAMERA_Y_M \
  camera_z:=CAMERA_Z_M \
  camera_roll:=CAMERA_ROLL_RAD \
  camera_pitch:=CAMERA_PITCH_RAD \
  camera_yaw:=CAMERA_YAW_RAD \
  localization:=true \
  rtabmap_viz:=true \
  rviz:=true \
  output:=screen
```

Replay the same bag in a second terminal:

```bash
ros2 bag play "$WSL_RUN" --clock
```

Acceptance:

- RTAB-Map loads the existing database without creating a new mapping database.
- Localization output and `map -> odom` TF appear.
- The loaded map is visible in RViz/RTAB-Map UI.
- No new database corruption or frame errors occur.

This is a smoke test, not proof of outdoor localization accuracy.

## 12. Required Repository Implementation Work

The first offline map can be attempted with the current recorder plus the
repository's `rover_slam/offline_mapping.launch.py`. The following changes
make the workflow repeatable and production-quality.

### 12.1 Make the bag self-contained

Add a shared frame configuration, for example:

```text
config/frames.yaml
```

with:

```yaml
base_frame: base_link
camera_frame: oak_rgb_camera_optical_frame
camera_to_base:
  translation_m: [x, y, z]
  rpy_rad: [roll, pitch, yaw]
```

Then update `recorder/record.py` and `recorder/bag_writer.py` to:

- Load the measured extrinsics.
- Write one `/tf_static` `TFMessage` containing `base_link -> oak_rgb_camera_optical_frame`.
- Keep `/tf` for time-varying `odom -> base_link` only.
- Use the configured camera frame consistently in RGB, depth, and camera-info headers.
- Write non-zero, defensible odometry covariance after estimating calibration uncertainty instead of the current all-zero covariance.
- Join the poller thread during shutdown before closing the bag.

Do not put the camera transform in `/tf` as a time-varying transform. It belongs in `/tf_static` if the camera is fixed.

### 12.2 Automated bag validation

`tools/validate_rgbd_bag.py` now checks:

- Required topic presence.
- Message counts and approximate rates.
- RGB/depth dimensions and encodings.
- Camera-info dimensions and a valid 3x3 intrinsic matrix.
- Monotonic timestamps.
- RGB/depth timestamp separation within a configured tolerance.
- `/odom` and `/tf` timestamp coverage over the camera interval.
- Exactly one parent for each important frame.
- No NaN or infinite odometry values.

Run it before every RTAB-Map attempt:

```bash
python3 tools/validate_rgbd_bag.py "$WSL_RUN"
```

For a direct WSL2 recording that contains static TF, add
`--require-tf-static`.

### 12.3 Add a reproducible WSL2 mapping package

The repository now contains:

```text
ros2_ws/src/rover_slam/
  package.xml
  setup.py
  resource/rover_slam
  launch/offline_mapping.launch.py
  launch/live_bringup.launch.py
  rover_slam/config/frames.yaml
  rover_slam/config/rtabmap_mapping.yaml
```

The offline launch includes:

- Static camera TF.
- `rtabmap_launch` with the raw bag topic names.
- `use_sim_time`.
- External odometry mode.
- Unique database path supplied as a launch argument.
- Optional RViz configuration.

The live launch includes:

- Explicit compressed RGB republishing.
- Explicit compressedDepth republishing.
- Live `/odom` and TF from `rover_bridge`.
- A static camera transform.
- The same frame names and database parameters as offline mode.

The installed Jazzy `image_transport republish` command must receive explicit `in_transport` and `out_transport` parameters. Do not rely on the broken positional form used in the original experiment.

## 13. Phase 8: Live Odometry and Live RTAB-Map

The bridge portion of this phase is implemented. Physical motion and network
validation are still required before trusting a live dataset.

### 13.1 Add odometry to the WSL bridge

`rover_bridge` now:

- Parse `payload["data"]["odl"]` and `payload["data"]["odr"]` from `/rover/telemetry`.
- Use the same calibration constants as the recorder.
- Publish `nav_msgs/msg/Odometry` on `/odom`.
- Broadcast `odom -> base_link` on `/tf`.
- Use the Pi telemetry `ts` value for the message timestamp.
- Reset the integrator safely on reconnect or encoder-counter reset.
- Publish finite covariance values.

The WSL2 integrator is the authoritative live odometry source. The Pi
recorder retains its independent local fallback because it runs without ROS2;
the two processes must never be used simultaneously as `/odom` publishers.

### 13.2 Validate live odometry before connecting RTAB-Map

With Pi services and bridge running:

```bash
ros2 topic hz /odom
ros2 topic echo --once /odom
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
```

Drive a measured straight distance and compare the `/odom` displacement with a tape measure. Repeat an arc and compare the heading sign. Do not start live RTAB-Map until this gate passes.

### 13.3 Validate live camera frames through image transport

The current reliable compressed topics are:

```text
/camera/color/image_raw/compressed
/camera/depth/image_raw/compressedDepth
```

Use the explicit Jazzy republisher form for consumers. Verify both compressed rates remain close to the Pi source rate and that depth republishing returns `16UC1`.

Then configure the live RTAB-Map launch to decode compressed topics in the consumer process, not in the TCP bridge. Keep `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` in every ROS2 process.

Once the motion gate passes, start the complete live stack:

```bash
ros2 launch rover_slam live_bringup.launch.py \
  pi_host:=192.168.50.5 \
  camera_x:=CAMERA_X_M \
  camera_y:=CAMERA_Y_M \
  camera_z:=CAMERA_Z_M \
  camera_roll:=CAMERA_ROLL_RAD \
  camera_pitch:=CAMERA_PITCH_RAD \
  camera_yaw:=CAMERA_YAW_RAD
```

The launch uses explicit `in_transport` and `out_transport` parameters for
both image republishers and provides the camera static TF. Record the raw
topics it produces:

```bash
ros2 bag record -o "$HOME/projects/autonomous_rover/recordings/live_run" \
  /oak/rgb/image_raw \
  /oak/stereo/image_raw \
  /oak/rgb/camera_info \
  /odom /tf /tf_static
```

## 14. What Is Out of Scope for the First Database

Do not block the first map on:

- Nav2.
- GPS/MAVLink integration.
- `robot_localization` EKF.
- `pointcloud_to_laserscan`.
- Human detection safety override.
- Outdoor waypoint navigation.
- Pi stereo resource ladder/error-525 work.
- A production systemd unit for RTAB-Map.
- Live compressed-camera mapping before `/odom` and TF are implemented.

These can use the first map later, but they are not prerequisites for proving that RGB-D plus calibrated odometry can create and reload a database.

## 15. Ordered Execution Checklist

```text
[ ] Push the current repository revision and pull it on the Pi.
[ ] Confirm WSL2 RTAB-Map and image-transport packages.
[ ] Stop Pi production services before direct recorder use.
[ ] Repeat and review wheel calibration; back up calibration.yaml.
[ ] Measure and record the fixed camera extrinsic transform.
[ ] Run a 30-second recorder smoke bag.
[ ] Validate the smoke bag with ros2 bag info and a short replay.
[ ] Run the 5-10 minute indoor mapping recording.
[ ] Transfer the bag to WSL2 and validate all five required topics.
[ ] Start the static camera TF.
[ ] Start RTAB-Map mapping mode with use_sim_time and external odometry.
[ ] Play the bag with --clock.
[ ] Verify point cloud, grid map, pose graph, and loop closures.
[ ] Stop RTAB-Map cleanly and verify the database.
[ ] Reload the database in localization mode with the same bag.
[ ] Harden the bag with recorded /tf_static and automated validation.
[ ] Implement live /odom and TF in rover_bridge.
[ ] Only then attempt live RTAB-Map mapping/localization.
```

## 16. Definition of Done

The first RTAB-Map creation phase is complete when:

- A Pi recording contains valid RGB, aligned depth, camera info, `/odom`, and `/tf`.
- The bag replays on WSL2 with `--clock` without serialization or timestamp errors.
- The static camera transform connects the camera frame to `base_link`.
- RTAB-Map mapping mode processes the replayed RGB-D sequence.
- A non-empty database is created at a known path.
- The resulting map is visible and contains a plausible pose graph/cloud.
- The same database loads again in localization mode.
- The run notes record calibration revision, camera transform, bag path, RTAB-Map parameters, and observed warnings.

A successful first database does not mean autonomous driving is ready. It proves the mapping substrate; localization, live odometry, Nav2, and safety still require their own validation gates.
