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
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-compressed-depth-image-transport
colcon build --symlink-install --packages-select rover_bridge rover_slam
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

# live_bringup starts rover_bridge, both image republishers, and camera TF.
ros2 launch rover_slam live_bringup.launch.py pi_host:=192.168.0.12
```

## WSL2: verify

Run in another WSL2 terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 topic list
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /odom
ros2 topic hz /tf
ros2 topic echo /battery_state
ros2 param get /rover_bridge calibration_file
ros2 param get /rover_bridge meters_per_tick_left
ros2 param get /rover_bridge meters_per_tick_right
ros2 param get /rover_bridge track_width_m
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
  database_path:=/tmp/rover_mapping.db
```

In a separate WSL2 terminal:

```bash
cd ~/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 bag play ~/projects/autonomous_rover/recordings/run1 --clock
```
