# Rover Quickstart

## Pi

```bash
ssh ws@192.168.0.12
cd /home/ws/ugv_rpi

# Install or refresh the production services.
sudo install -m 0644 systemd/rover-agent.service /etc/systemd/system/rover-agent.service
sudo install -m 0644 systemd/rover-camera.service /etc/systemd/system/rover-camera.service
sudo systemctl daemon-reload
sudo systemctl enable --now rover-agent.service rover-camera.service

hostname -I
ss -ltn | grep -E ':(8765|8766|8770)\b'
systemctl --no-pager --full status rover-agent.service rover-camera.service
journalctl -u rover-agent.service -f
```

Do not run `app.py`, calibration, or the standalone recorder while these
services are using `/dev/ttyAMA0`.

## WSL2: build and live ROS2

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
sudo apt-get install ros-jazzy-rosbridge-server ros-jazzy-web-video-server
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-compressed-depth-image-transport
colcon build --symlink-install --packages-select rover_bridge rover_slam
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

# live_bringup keeps the network stream compressed and publishes camera TF.
ros2 launch rover_slam live_bringup.launch.py pi_host:=192.168.0.12
```

## WSL2: rover GCS dashboard

The read-only dashboard observes the existing ROS2 graph. It opens a native
WebSocket for low-rate telemetry and uses host-side MJPEG streams for the two
camera panes:

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
ros2 launch rover_slam gcs.launch.py pi_host:=192.168.0.12
```

Open <http://localhost:8000> in a browser. The dashboard subscribes to
`/odom`, `/battery_state`, `/cmd_vel`, and `/rover/telemetry`. RGB and depth
are served by `web_video_server` using compressed image transport from
the base topics `/camera/color/image_raw` and
`/gcs/depth_preview/image_raw`. The dashboard uses the server's direct
`ros_compressed` stream mode, which resolves their compressed transport topics;
raw `/oak/...` image topics are not required. The depth preview is generated at 5 Hz from
`/camera/depth/image_raw/compressedDepth`.

The depth preview performs one additional, capped host-side decode and JPEG
encode for visualization. It does not publish raw images and does not send any
additional data to the Pi. RTAB-Map should continue subscribing directly to
the original compressed RGB-D topics.

This does not add traffic to the Pi and does not create another camera
connection. The bridge, RTAB-Map, and the dashboard are all on WSL2; the
dashboard only adds local ROS subscriptions and host-side MJPEG encoding.
Keep the dashboard read-only while validating autonomous driving. If the
host becomes CPU-bound, close the browser or stop `web_video_server`; the
dashboard's telemetry subscriptions are throttled to at most 5 Hz and the
camera streams are limited to 720 pixels and quality 70.

If the bridge reports `rx_fps=0.00` and `Rover camera connection lost:
timed out`, the GCS is not yet receiving camera frames. Check the Pi service
before changing ROS topics:

```bash
ssh ws@192.168.0.12
sudo systemctl --no-pager --full status rover-camera.service
sudo journalctl -u rover-camera.service -n 100 --no-pager
ss -ltn | grep ':8770\b'
sudo systemctl restart rover-camera.service
```

The camera service accepts one TCP client and must be able to dequeue
synchronized RGB-D pairs from the OAK-D. After restarting it, confirm the
bridge reports nonzero `rx_fps` before starting RTAB-Map or debugging the
browser. Do not run a second camera reader against the OAK-D at the same time.

If ports `8000`, `8080`, or `9090` are already in use, stop the previous GCS
launch with `Ctrl+C` before starting it again. There must be exactly one
`rover_bridge`, one `gcs_depth_preview`, one `web_video_server`, and one
`rosbridge_websocket` process. Confirm the bridge publisher exists before
debugging the browser:

```bash
ros2 node list
ros2 topic info /camera/color/image_raw/compressed
ros2 topic info /gcs/depth_preview/image_raw/compressed
```

## WSL2: verify

Run in another WSL2 terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 topic list
# The bridge publishes these compressed camera topics only.
ros2 topic hz /camera/color/image_raw/compressed
ros2 topic hz /camera/depth/image_raw/compressedDepth
ros2 topic hz /odom
ros2 topic hz /tf
ros2 topic echo /battery_state
ros2 param get /rover_bridge calibration_file
ros2 param get /rover_bridge max_distance_delta_m
```

## WSL2: drive and stop

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

# Slow straight test. linear.y is unused by a differential-drive rover.
timeout 8.6s ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: -0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Explicit stop.
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## Pi: calibrate or record

Stop the production services first:

```bash
ssh ws@192.168.0.12
cd /home/ws/ugv_rpi
sudo systemctl stop rover-camera.service rover-agent.service

./ugv-env/bin/python pedro_scripts/calibrate_odometry.py straight --speed 0.39 --duration 4
./ugv-env/bin/python pedro_scripts/calibrate_odometry.py arc --left 0.06 --right 0.39 --duration 4
./ugv-env/bin/python pedro_scripts/calibrate_imu.py --duration 30
./ugv-env/bin/python pedro_scripts/calibrate_odometry.py summary

# Or record a synchronized RGB-D and wheel-odometry bag.
./ugv-env/bin/python recorder/record.py --out recordings/run1
```

The only editable calibration source is:

```text
/home/ws/ugv_rpi/config/calibration.yaml
```

Copy the updated file to the WSL checkout before live ROS2 use:

```bash
scp ws@192.168.0.12:/home/ws/ugv_rpi/config/calibration.yaml \
  ~/projects/autonomous_rover/config/calibration.yaml
```

Restart the Pi services after calibration or recording:

```bash
sudo systemctl enable --now rover-agent.service rover-camera.service
```

## WSL2: offline RTAB-Map

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 launch rover_slam offline_mapping.launch.py \
  use_sim_time:=true \
  database_path:=/tmp/rover_mapping.db
```

The offline recorder writes raw `/oak/...` RGB-D topics, so the default launch
arguments above use the raw transport. For a live RTAB-Map run against the
compressed bridge topics, use:

```bash
ros2 launch rover_slam offline_mapping.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/oak/rgb/camera_info \
  rgb_image_transport:=compressed \
  depth_image_transport:=compressedDepth \
  qos:=1 \
  qos_camera_info:=2 \
  use_sim_time:=false \
  database_path:=/tmp/rover_live_mapping.db
```

In both cases, RTAB-Map performs the decoding in its ROS2 image pipeline; the
bridge does not publish raw images.

In a separate WSL2 terminal:

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 bag play ~/projects/autonomous_rover/recordings/run1 --clock
```
