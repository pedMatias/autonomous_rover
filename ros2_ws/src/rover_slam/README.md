# rover_slam

`rover_slam` contains the host-side launch and visualization tools for the
rover RGB-D workflow. It does not own the Pi hardware connection. The
[`rover_bridge`](../rover_bridge) package supplies compressed RGB-D,
CameraInfo, telemetry, and provisional odometry; this package starts the
bridge, publishes the camera TF, feeds RTAB-Map, and serves the optional GCS
dashboard.

```text
Pi services -> rover_bridge -> compressed ROS2 RGB-D + /odom + /tf
                                      |              |
                                      v              v
                              GCS previews       RTAB-Map
```

## Contents

| Path | Purpose |
| --- | --- |
| [launch/live_bringup.launch.py](launch/live_bringup.launch.py) | Start `rover_bridge` and the static base-to-camera transform. |
| [launch/gcs.launch.py](launch/gcs.launch.py) | Start live bringup, preview nodes, rosbridge, web video, and the dashboard server. |
| [launch/offline_mapping.launch.py](launch/offline_mapping.launch.py) | Republish recorded or compressed images and include RTAB-Map. |
| [rover_slam/depth_preview.py](rover_slam/depth_preview.py) | Decode depth locally and publish a capped JPEG visualization. |
| [rover_slam/pointcloud_preview.py](rover_slam/pointcloud_preview.py) | Project decimated uint16 depth into an optical-frame `PointCloud2`. |
| [gcs/index.html](gcs/index.html) | Browser dashboard for video, telemetry, odometry, point cloud, and guarded drive control. |
| [config/frames.yaml](config/frames.yaml) | Reference frame names and camera transform values. |
| [config/rtabmap_mapping.yaml](config/rtabmap_mapping.yaml) | Reference RTAB-Map registration, proximity, optimizer, and grid settings. |

The two YAML files under `config/` are installed as package data but are not
automatically loaded by the current launch files. Their active equivalents
are the launch defaults and the explicit RTAB-Map arguments in
`offline_mapping.launch.py`.

## Requirements

- ROS2 Jazzy on the host.
- The `rover_bridge` package and its Pi services for live operation.
- `image_transport` plus the compressed image and compressed-depth plugins.
- `rtabmap_launch` and its RTAB-Map runtime for mapping.
- `rosbridge_server` and `web_video_server` for the GCS launch.
- Python ROS2 dependencies declared in [package.xml](package.xml): `rclpy`,
  NumPy, OpenCV, `sensor_msgs`, `tf2_ros`, and the launch packages.

Install the common ROS dependencies on Jazzy with:

```bash
sudo apt-get install -y \
  ros-jazzy-rosbridge-server \
  ros-jazzy-web-video-server \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-compressed-depth-image-transport
```

Build both local packages from the workspace:

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rover_bridge rover_slam
source install/setup.bash
```

## Live bringup

`live_bringup.launch.py` starts:

1. `rover_bridge/rover_bridge` with the selected bridge YAML and Pi address.
2. `tf2_ros/static_transform_publisher` from `base_link` to the camera optical
   frame.

Run it with the Pi address for the current network:

```bash
ros2 launch rover_slam live_bringup.launch.py pi_host:=192.168.0.12
```

Launch arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `pi_host` | `192.168.0.12` | Pi host passed to `rover_bridge`. |
| `bridge_config` | installed `rover_bridge.yaml` | Bridge parameter file. |
| `camera_info_topic` | `/oak/rgb/camera_info` | CameraInfo topic passed to the bridge. |
| `base_frame` | `base_link` | Parent of the static camera transform. |
| `camera_frame` | `oak_rgb_camera_optical_frame` | Child and image frame. |
| `camera_x`, `camera_y`, `camera_z` | `0.0` | Camera translation in metres. |
| `camera_roll`, `camera_pitch`, `camera_yaw` | `-1.5707963`, `0.0`, `-1.5707963` | Camera rotation in radians. |

The zero translation is provisional. Measure the physical camera offset before
using the transform for navigation. The default rotation is the tested
forward-facing optical-frame rotation used by this project.

Live outputs come from `rover_bridge`: compressed RGB on
`/camera/color/image_raw/compressed`, compressed depth on
`/camera/depth/image_raw/compressedDepth`, CameraInfo on
`/oak/rgb/camera_info`, telemetry on `/rover/telemetry`, voltage on
`/battery_state`, odometry on `/odom`, and `odom -> base_link` on TF.

## Ground control dashboard

`gcs.launch.py` includes live bringup and starts:

- `gcs_depth_preview`, which publishes `/gcs/depth_preview/image_raw/compressed`.
- `gcs_pointcloud_preview`, which publishes `/gcs/pointcloud`.
- `rosbridge_websocket` on port `9090`.
- `web_video_server` on port `8080`.
- Python's static HTTP server on port `8000`, serving [gcs/index.html](gcs/index.html).

Start it with:

```bash
ros2 launch rover_slam gcs.launch.py pi_host:=192.168.0.12
```

Open <http://localhost:8000>. The dashboard subscribes through rosbridge to
`/odom`, `/battery_state`, `/cmd_vel`, `/rover/telemetry`, and
`/gcs/pointcloud`. The RGB stream is the compressed base camera topic; the
depth stream is the local 5 Hz JPEG preview. `web_video_server` performs the
host-side MJPEG serving, so no additional camera connection or Pi traffic is
created.

The browser also contains an arm-gated `/cmd_vel` control. It repeats a
selected command at 10 Hz, limits its buttons to 0.30 m/s and 0.20 rad/s, and
sends zero on stop, browser blur, hidden-page transitions, disconnect, or
disarm. This is still a motion-control interface: use it only with a clear
test area and the rover's physical power switch accessible.

The dashboard accepts URL overrides for deployments on another host:

```text
http://HOST:8000/?rosbridge=ws://HOST:9090&video=http://HOST:8080
```

The `rgb`, `rgb_type`, `depth`, and `depth_type` query parameters can also
override the two video streams. The default video type is `ros_compressed`.

### Depth preview parameters

The `gcs_depth_preview` executable is registered by [setup.py](setup.py):

```bash
ros2 run rover_slam gcs_depth_preview --ros-args \
  -p publish_rate_hz:=5.0 \
  -p min_depth_m:=0.3 \
  -p max_depth_m:=5.0 \
  -p jpeg_quality:=70
```

Parameters:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `depth_topic` | `/camera/depth/image_raw/compressedDepth` | Compressed-depth input. |
| `preview_topic` | `/gcs/depth_preview/image_raw` | Base preview output; `/compressed` is appended. |
| `publish_rate_hz` | `5.0` | Maximum preview rate. |
| `min_depth_m` / `max_depth_m` | `0.3` / `5.0` | Visualization range. Zero depth remains black. |
| `jpeg_quality` | `70` | JPEG quality for the browser stream. |

The node keeps only the latest depth message. It strips either a bare PNG or
the standard 12-byte compressedDepth header, requires a uint16 depth image,
maps the selected range through the Turbo color map, and JPEG-encodes it. It
does not publish raw depth and does not send data back to the Pi.

### Point cloud preview parameters

The `gcs_pointcloud_preview` executable publishes a binary `sensor_msgs/msg/PointCloud2`:

```bash
ros2 run rover_slam gcs_pointcloud_preview --ros-args \
  -p decimation:=4 \
  -p min_depth_m:=0.3 \
  -p max_depth_m:=5.0
```

| Parameter | Default | Purpose |
| --- | --- | --- |
| `depth_topic` | `/camera/depth/image_raw/compressedDepth` | Compressed-depth input. |
| `camera_info_topic` | `/oak/rgb/camera_info` | Intrinsics input. |
| `pointcloud_topic` | `/gcs/pointcloud` | Point cloud output. |
| `publish_rate_hz` | `5.0` | Maximum cloud rate. |
| `decimation` | `4` | Samples every fourth row and column by default. |
| `min_depth_m` / `max_depth_m` | `0.3` / `5.0` | Accepted point range. |

Depth is interpreted as uint16 millimetres. The node scales camera intrinsics
to the actual depth image dimensions and emits little-endian XYZ float32
points in the camera optical frame. Invalid and out-of-range pixels are
discarded. The latest depth and latest CameraInfo are used; there is no exact
timestamp synchronization between them.

## Offline RTAB-Map mapping

`offline_mapping.launch.py` is for bag replay or another local image source.
It starts two explicit `image_transport republish` nodes, a static camera TF,
and the installed RTAB-Map launch file. The republishers convert the selected
RGB and depth transports to raw relay topics, then RTAB-Map consumes those
relay topics with RGB-D synchronization.

The recommended compressed bridge input is:

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rover_slam offline_mapping.launch.py \
  use_sim_time:=true \
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
  database_path:=/home/pmati/projects/autonomous_rover/maps/room_run_03.db
```

Start this launch before replaying the bag. In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
ros2 bag play ~/projects/autonomous_rover/recordings/room_run_02 --clock --rate 0.5
```

The launch defaults are also suitable for a legacy raw recorder bag:
`/oak/rgb/image_raw`, `/oak/stereo/image_raw`, and raw transports. The
compressed arguments must be supplied for the current bridge bag interface.

Important mapping behavior:

- The relay topics are the selected input topics with `_relay` appended.
- RTAB-Map receives `rgbd_sync:=true` and approximate RGB-D synchronization.
- The outer odometry/RGB-D synchronization is approximate by default with a
  0.20 second maximum interval and queues of 30.
- RTAB-Map visual and ICP odometry are disabled because `/odom` is supplied by
  the bridge or bag.
- `Reg/Force3DoF true` constrains the rover mapping problem to planar motion.
- `publish_tf_map` is enabled and the database path is passed through to
  RTAB-Map.
- `use_sim_time:=true` is required for `ros2 bag play --clock`; use false for
  a live source.

### Offline launch arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `base_frame` / `camera_frame` | `base_link` / `oak_rgb_camera_optical_frame` | Static TF frames and RTAB-Map base frame. |
| `camera_x`, `camera_y`, `camera_z` | `0.0` | Camera translation in metres. |
| `camera_roll`, `camera_pitch`, `camera_yaw` | `-1.5707963`, `0.0`, `-1.5707963` | Camera rotation in radians. |
| `map_frame_id` | `map` | RTAB-Map map frame. |
| `rgb_topic` / `depth_topic` | `/oak/rgb/image_raw` / `/oak/stereo/image_raw` | Input image base topics. |
| `rgb_image_transport` / `depth_image_transport` | `raw` / `raw` | Input transports; use `compressed` / `compressedDepth` for bridge bags. |
| `camera_info_topic` | `/oak/rgb/camera_info` | Camera intrinsics. |
| `odom_topic` | `/odom` | Wheel odometry input. |
| `use_sim_time` | `false` | Use bag clock when true. |
| `approx_sync` / `approx_sync_max_interval` | `true` / `0.20` | Approximate outer synchronization. |
| `topic_queue_size` / `sync_queue_size` | `30` / `30` | RGB-D and synchronization queues. |
| `qos` / `qos_camera_info` | `0` / `0` | RTAB-Map image and CameraInfo QoS values. |
| `database_path` | empty | RTAB-Map database output path. |
| `localization` | `false` | Pass localization mode to RTAB-Map. |
| `rtabmap_viz` / `rviz` | `true` / `true` | Enable RTAB-Map visualization and RViz. |

## Validation

For live bringup, verify the bridge before debugging the dashboard:

```bash
ros2 node list
ros2 topic info /camera/color/image_raw/compressed
ros2 topic info /camera/depth/image_raw/compressedDepth
ros2 topic hz /camera/color/image_raw/compressed
ros2 topic hz /camera/depth/image_raw/compressedDepth
ros2 topic hz /odom
```

For compressed-depth compatibility, republish it to raw in a separate test:

```bash
ros2 run image_transport republish --ros-args \
  -p in_transport:=compressedDepth \
  -p out_transport:=raw \
  -r in/compressedDepth:=/camera/depth/image_raw/compressedDepth \
  -r out:=/camera/depth/image_raw_check
ros2 topic hz /camera/depth/image_raw_check
```

The package has no package-local automated test directory. Validate its launch
files with a successful build, then validate the live or replay graph with
the commands above and check that an offline database contains nonzero nodes,
poses, RGB/depth data, and graph links.

The package entry points are `gcs_depth_preview` and
`gcs_pointcloud_preview`. Its installed data includes all `*.launch.py`, all
YAML files under `config/`, and the contents of `gcs/`.