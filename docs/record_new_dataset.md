Everything required is in place. Here's the readiness check and the full runbook.

## Readiness (verified)

- RTAB-Map tools installed: `/opt/ros/jazzy/bin/rtabmap-export`, `rtabmap-info`, `rtabmap-databaseViewer`, and `rtabmap_launch/rtabmap.launch.py`.
- Workspace built: `install/rover_bridge/lib/rover_bridge/{rover_bridge,rover_teleop}` and the three `rover_slam` launch files are installed.
- `recordings/` and `maps/` exist; your last bag `room_run_02` contains exactly the topics we need (`/camera/color/image_raw/compressed` 1636, `/camera/depth/image_raw/compressedDepth` 1636, `/oak/rgb/camera_info` 1636, `/odom` 2131, `/tf` 2131, `/tf_static` 1).
- **One bug fixed:** `quickstart.md` §4 omitted the compressed topic/transport args, so it would have mapped nothing (offline launch defaults to raw `/oak/...`). Corrected.

**One thing to re-check at start:** the last bridge log (today 01:02) ends with `Rover camera connection lost: Connection refused`, i.e. `rover-camera.service` was down when you stopped. Confirm both Pi services before starting.

## Runbook — new run `room_run_05`

### 0. Pre-flight: Pi services (SSH)

```bash
ssh ws@192.168.0.12
cd /home/ws/ugv_rpi
sudo systemctl restart rover-agent.service rover-camera.service
systemctl --no-pager --full status rover-agent.service rover-camera.service
ss -ltn | grep -E ':(8765|8766|8770)\b'
```

All three ports must listen. If 8770 is missing, restart `rover-camera.service` and check `journalctl -u rover-camera.service -n 50`.

### 1. WSL: build + clean the DDS daemon

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rover_bridge rover_slam
source install/setup.bash
ros2 daemon stop
```

### 2. Terminal 1 — live bringup (keep open)

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 launch rover_slam live_bringup.launch.py pi_host:=192.168.0.12
```

Wait for `Connected to rover command/telemetry/camera port` and `Camera stats: rx_fps=~13`.

### 3. Terminal 2 — verify (optional but recommended)

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 node list   # expect /rover_bridge and /camera_static_transform
ros2 topic hz /camera/color/image_raw/compressed
ros2 topic hz /camera/depth/image_raw/compressedDepth
ros2 topic hz /odom
```

### 4. Terminal 3 — record the bag

```bash
cd ~/projects/autonomous_rover
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 bag record -o recordings/room_run_05 --topics \
  /camera/color/image_raw/compressed \
  /camera/depth/image_raw/compressedDepth \
  /oak/rgb/camera_info \
  /odom /tf /tf_static \
  /cmd_vel /rover/telemetry /battery_state
```

### 5. Terminal 4 — drive (same env as above)

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 run rover_bridge rover_teleop --drive-speed 0.15 --turn-speed 0.10
```

Drive rules for a clean database:
- Keep moving slowly (this preset ~0.15 m/s) to avoid motion blur.
- Drive a **full loop back to the start** for loop closure; revisit corners with overlap.
- Pause with `space`, not `Ctrl+C`, whenever you stop.
- When done, stop the rover, send `space`, then `q`, then `Ctrl+C` the recorder.

### 6. Verify the bag

```bash
cd ~/projects/autonomous_rover
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 bag info recordings/room_run_05
```

RGB, depth, and camera_info counts should be comparable and nonzero.

### 7. Terminal A — RTAB-Map offline mapping (start before replay)

#### 7.1. Wheel-odom clean re-process (baseline)
Terminal A (start first):
```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 launch rover_slam offline_mapping.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/oak/rgb/camera_info \
  rgb_image_transport:=compressed \
  depth_image_transport:=compressedDepth \
  approx_sync:=true approx_sync_max_interval:=0.20 \
  topic_queue_size:=30 sync_queue_size:=30 \
  qos:=1 qos_camera_info:=2 \
  use_sim_time:=true \
  database_path:=/home/pmati/projects/autonomous_rover/maps/room_run_05_v2.db \
  rtabmap_args:="--Reg/Force3DoF true --RGBD/NeighborLinkRefining true --Reg/Strategy 0 --Optimizer/Strategy 2 --RGBD/ProximityBySpace true --RGBD/ProximityByTime true --RGBD/LinearUpdate 0.05 --RGBD/AngularUpdate 0.05 --Grid/RangeMax 2.5 --Grid/RangeMin 0.3 --Grid/DepthDecimation 2 --Mem/NotLinkedNodesKept false --Rtabmap/DetectionRate 5"
```
Terminal B (full bag — wheel run needs `/odom` + `/tf`):
```bash
cd ~/projects/autonomous_rover
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 bag play recordings/room_run_05 --clock --rate 0.5
```
Stop Terminal A with `Ctrl+C` after playback ends (flushes the DB).

#### 7.2. Corrected VO re-process
Terminal A:
```bash
ros2 launch rover_slam offline_mapping.launch.py \
  visual_odometry:=true \
  odom_topic:=odom \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/oak/rgb/camera_info \
  rgb_image_transport:=compressed \
  depth_image_transport:=compressedDepth \
  approx_sync:=true approx_sync_max_interval:=0.20 \
  topic_queue_size:=30 sync_queue_size:=30 \
  qos:=1 qos_camera_info:=2 \
  use_sim_time:=true \
  database_path:=/home/pmati/projects/autonomous_rover/maps/room_run_05_vo3.db \
  rtabmap_args:="--Reg/Force3DoF true --RGBD/NeighborLinkRefining true --Reg/Strategy 0 --Optimizer/Strategy 2 --RGBD/ProximityBySpace true --RGBD/ProximityByTime true --RGBD/LinearUpdate 0.05 --RGBD/AngularUpdate 0.05 --Grid/RangeMax 2.5 --Grid/RangeMin 0.3 --Grid/DepthDecimation 2 --Mem/NotLinkedNodesKept false --Rtabmap/DetectionRate 5"
```
Terminal B (**whitelist — no `/odom`, no `/tf`**):
```bash
ros2 bag play recordings/room_run_05 --clock --rate 0.5 --topics \
  /camera/color/image_raw/compressed \
  /camera/depth/image_raw/compressedDepth \
  /oak/rgb/camera_info
```
The launch's static TF provides `base_link→oak_rgb_camera_optical_frame`; VO provides
`odom→base_link`; RTAB-Map publishes `map→odom`. No duplicate publishers.
Do **not** include `/tf_static` (redundant with the launch publisher) and never `/odom` or `/tf`.


### 8. Terminal B — replay the bag

```bash
cd ~/projects/autonomous_rover
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 bag play recordings/room_run_05 --clock --rate 0.5
```

When playback ends, press `Ctrl+C` in Terminal A to stop RTAB-Map cleanly (this flushes the `.db`).

### 9. Inspect the database

```bash
rtabmap-info ~/projects/autonomous_rover/maps/room_run_05.db
```

Expect nonzero sessions, nodes, poses, RGB/depth, and graph links.

### 10. Optional — export a textured mesh for Blender

```bash
rtabmap-databaseViewer ~/projects/autonomous_rover/maps/room_run_05.db 
```

or

```bash
cd ~/projects/autonomous_rover
rtabmap-export \
  --mesh --texture \
  --max_range 2.2 --min_range 0.3 \
  --decimation 2 --voxel 0.015 \
  --poisson_depth 9 --min_cluster 100 \
  --output_dir maps/export_05 \
  maps/room_run_05.db
```

## Important caveats

- **Camera transform consistency:** `live_bringup.launch.py` and `offline_mapping.launch.py` both default `camera_x=0.0`, `camera_y=0.0`, `camera_z=0.12`. Keep both identical (or pass the same values to both), otherwise the recorded `/tf_static` and the replay's static TF conflict. Bags recorded before this default changed carried `camera_z=0.0`; replaying one now shifts the camera frame 12 cm, so pass `camera_z:=0.0` for those older runs.
- **All ROS terminals must share** `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and `FASTDDS_BUILTIN_TRANSPORTS`; otherwise nodes won't see each other (the earlier daemon/discovery issue).
- **Only one camera client:** do not run the GCS dashboard or a raw 8770 measurement while the bridge holds the camera.
- **Teleop is open-loop and laggy** (~0.7 s key window + ~0.4 s accel ramp). It's fine for mapping if you move in deliberate segments and pause; avoid rapid direction changes, which will blur the map. If you prefer, use the GCS buttons (`gcs.launch.py`, 0.30 m/s / 0.15 m/s diagonals) instead of keyboard.
- **If RTAB-Map creates very few nodes,** add these to `rtabmap_args` for denser keyframes: `--Grid/RangeMax 2.5 --Grid/RangeMin 0.3 --RGBD/LinearUpdate 0.05 --RGBD/AngularUpdate 0.05` (these are in `config/rtabmap_mapping.yaml` but that file is not auto-loaded by the launch).

I changed only `quickstart.md` §4 (added the required `rgb_topic`/`depth_topic`/transport/QoS arguments to both mapping options).