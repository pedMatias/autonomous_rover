# Rover Quickstart Cheat Sheet

## 0. First-Time Environment Setup (WSL2)
*Only needed once on a fresh machine or new container:*

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-rosbridge-server \
  ros-jazzy-web-video-server \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-compressed-depth-image-transport \
  ros-jazzy-rtabmap-ros

cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rover_bridge rover_slam
```

---

## 1. Pi Services (Raspberry Pi)

```bash
ssh ws@192.168.0.12
cd /home/ws/ugv_rpi

# Start production services
sudo systemctl daemon-reload
sudo systemctl enable --now rover-agent.service rover-camera.service

# Check health
systemctl --no-pager status rover-agent.service rover-camera.service
```

---

## 2. WSL2: Setup & Live Bringup

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

# Option A: Headless Bridge
ros2 launch rover_slam live_bringup.launch.py pi_host:=192.168.0.12
---

## 2.5. Teleop Control (Drive the Rover)

### Option A: Keyboard Teleop (Terminal)

Requires an interactive TTY. Run in a separate terminal after live bringup:

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 run rover_bridge rover_teleop
```

> **Every ROS2 process must use the same `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`,
> and `FASTDDS_BUILTIN_TRANSPORTS`.** The teleop talks to `rover_bridge` over
> DDS; if these differ, `/cmd_vel` never reaches the bridge and the rover will
> not move. If `ros2 node list` is empty even though the launch is running, the
> `ros2` CLI daemon cached an old environment — run `ros2 daemon stop` and
> retry.

| Keys | Motion |
| --- | --- |
| `w` / `s` | forward / reverse straight |
| `a` / `d` | gentle forward-left / forward-right diagonal |
| `w`+`a` / `w`+`d` | tighter left / right arc |
| `s`+`a` / `s`+`d` | reverse-left / reverse-right arc |
| `space` / `x` | stop |
| `q` | quit |

> **Note:** `a`/`d` alone creep forward instead of spinning in-place. In-place turns are blocked by design to avoid opposite-sign wheel commands that caused issues historically.

### Option B: GCS Web Dashboard

The GCS launch (`gcs.launch.py`) provides on-screen drive buttons:
- Forward/reverse: `vx = ±0.30 m/s`
- Left/right: diagonal arcs at `vx = 0.15 m/s, wz = ±0.75 rad/s`

Open http://localhost:8000 and use the button grid.

---

## 3. WSL2: Record a Mapping Run

While `live_bringup` or `gcs.launch.py` is running in another terminal:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

# Record topics to a bag
ros2 bag record -o recordings/room_run_04 --topics \
  /camera/color/image_raw/compressed \
  /camera/depth/image_raw/compressedDepth \
  /oak/rgb/camera_info \
  /odom /tf /tf_static \
  /cmd_vel /rover/telemetry /battery_state
```

> **Scanning Tips for Clean 3D Meshes:**
> - Drive slowly ($\le 0.15\text{ m/s}$) to prevent camera blur.
> - Complete a full loop back to the start so RTAB-Map can close the loop.
> - Press `Ctrl+C` to stop recording once the rover is stopped.

---

## 4. WSL2: Replay Bag & Build RTAB-Map Database

### Terminal 1: Launch RTAB-Map Offline Mapping
```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

# Option A: Wheel odometry + visual link refining (Default, highly robust)
ros2 launch rover_slam offline_mapping.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/oak/rgb/camera_info \
  rgb_image_transport:=compressed \
  depth_image_transport:=compressedDepth \
  approx_sync:=true \
  approx_sync_max_interval:=0.20 \
  topic_queue_size:=30 \
  sync_queue_size:=30 \
  qos:=1 \
  qos_camera_info:=2 \
  use_sim_time:=true \
  database_path:=/home/pmati/projects/autonomous_rover/maps/room_run_04.db

# Option B: Pure Visual Odometry (RGB-D VO instead of wheel odom)
ros2 launch rover_slam offline_mapping.launch.py \
  visual_odometry:=true \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/oak/rgb/camera_info \
  rgb_image_transport:=compressed \
  depth_image_transport:=compressedDepth \
  approx_sync:=true \
  approx_sync_max_interval:=0.20 \
  topic_queue_size:=30 \
  sync_queue_size:=30 \
  qos:=1 \
  qos_camera_info:=2 \
  use_sim_time:=true \
  database_path:=/home/pmati/projects/autonomous_rover/maps/room_run_04_vo.db
```

### Terminal 2: Play the Bag
Start replay after Terminal 1 initializes:
```bash
cd ~/projects/autonomous_rover
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 bag play recordings/room_run_04 --clock --rate 0.5
```
*Stop RTAB-Map (`Ctrl+C` in Terminal 1) after playback finishes.*

---

## 5. Inspect & Export 3D Textured Mesh for Blender

### Check Map Poses & Links
```bash
rtabmap-info ~/projects/autonomous_rover/maps/room_run_04.db
```

### CLI Export (Fast, Clean OBJ + Texture)
```bash
rtabmap-export \
  --mesh \
  --texture \
  --max_range 2.2 \
  --min_range 0.3 \
  --decimation 2 \
  --voxel 0.015 \
  --poisson_depth 9 \
  --min_cluster 100 \
  --output_dir maps/export_04 \
  maps/room_run_04.db
```

### GUI Export (Interactive)
```bash
rtabmap-databaseViewer ~/projects/autonomous_rover/maps/room_run_04.db
```
Follow [docs/export_mesh_using_rtab_map_viewer.md](docs/export_mesh_using_rtab_map_viewer.md) for step-by-step dialog settings.
