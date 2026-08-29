# UGV Rover Pi + WSL2

This repository contains the complete shared software for a Waveshare UGV
Rover with a Raspberry Pi 5, an ESP32 base controller, an OAK-D camera, and a
Windows PC running ROS2 inside WSL2.

The Pi is the hardware gateway. WSL2 runs the real ROS2 graph and owns the
robotics stack: teleoperation, recording analysis, RTAB-Map, localization,
robot_localization, Nav2, and RViz. ROS2 is not installed on the Pi.

The repository is kept in both environments. Make changes in one checkout,
push them, and pull them in the other checkout. Do not maintain a separate
copy of the bridge or service configuration by hand.

## Architecture

```text
WSL2 / Windows PC                              Raspberry Pi 5
-----------------                              --------------
ROS2 nodes                                      rover_agent.py
  /cmd_vel                                      camera_stream.py
  RTAB-Map                                      BaseController -> UART
  Nav2                                          /dev/ttyAMA0 -> ESP32
        |                                               ^
        | TCP 8765: cmd_vel JSON                         |
        | TCP 8766: telemetry JSON                       |
        | TCP 8770: RGB-D packets                        |
        +-------------------- network ------------------+
```

The ROS2 bridge is [ros2_ws/src/rover_bridge](ros2_ws/src/rover_bridge). It is
a normal ROS2 Python node. It translates between ROS2 messages on the PC and
the explicit TCP protocol used by the Pi.

## Runtime ownership

| Component | Host | Role | Interface |
| --- | --- | --- | --- |
| `app.py` | Pi | Factory browser control and manual baseline | Web UI `5000`, UART |
| `rover_agent.py` | Pi | Production motor/telemetry gateway | Commands `8765`, telemetry `8766`, UART |
| `camera_stream.py` | Pi | OAK-D RGB-D network stream | Camera `8770` |
| `rover_bridge` | WSL2 | ROS2 network adapter | `/cmd_vel` and sensor topics |
| `recorder/record.py` | Pi | Standalone synchronized bag recorder | UART and OAK-D |

Only one process may own `/dev/ttyAMA0` at a time. Never run `app.py`,
`rover_agent.py`, `record.py`, or a UART diagnostic/calibration script together.
The factory app and production agent are mutually exclusive.

The factory movement convention is the hardware reference:

```text
forward:       L=+speed, R=+speed
reverse:       L=-speed, R=-speed
left spin:     L=-speed, R=+speed
right spin:    L=+speed, R=-speed
```

The factory UGV Rover configuration uses `max_speed=1.3`, `slow_speed=0.2`,
and starts with `speed_rate=0.3`. Therefore its effective startup commands are
`0.39 m/s` for full-wheel motion and `0.06/0.39 m/s` for diagonal motion. The
calibration, link-check, and recorder defaults use these effective values. The
factory can command up to `1.3 m/s` after selecting its fastest speed setting;
the production ROS2 gateway currently keeps its independent initial limit at
`0.5 m/s` until higher-speed network control is explicitly validated.

The production agent limits wheel speed and stops on command timeout,
disconnect, or shutdown. Opposite-sign wheel speeds are valid and are required
for factory-style in-place turns. The calibration and recorder tools use only
same-sign straight/diagonal commands because those controlled tests avoid the
highest power load; that local restriction does not change the production
agent protocol.

## Pi setup

The expected hardware configuration is:

```text
UART:       /dev/ttyAMA0 at 115200
Python env: /home/ws/ugv_rpi/ugv-env
Pi hotspot: 192.168.50.5
```

Update the Pi from the shared repository:

```bash
cd /home/ws/ugv_rpi
git pull --ff-only
```

Before using the production services, remove or disable the factory `app.py`
cron entry created by `autorun.sh`. Inspect the entry first:

```bash
crontab -l
```

Then remove only the line that starts the repository's `app.py`, if present:

```bash
crontab -l | grep -v 'ugv_rpi/ugv-env/bin/python .*app.py' | crontab -
```

Install the production services:

```bash
cd /home/ws/ugv_rpi
sudo cp systemd/rover-agent.service /etc/systemd/system/
sudo cp systemd/rover-camera.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rover-agent.service rover-camera.service
```

Check them:

```bash
systemctl status rover-agent.service rover-camera.service
journalctl -u rover-agent.service -f
```

Stop both services before using the factory web app, calibration tools, link
diagnostics, or the standalone recorder:

```bash
sudo systemctl stop rover-camera.service rover-agent.service
```

The factory baseline can then be run manually with `app.py` and opened at
`http://PI_ADDRESS:5000`. Stop it before restarting the production services.

## WSL2 setup

Use a checkout of this repository on the Windows/WSL2 machine. The ROS2
workspace is `ros2_ws` in that checkout:

```bash
cd ~/ugv_rpi
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run the bridge with the Pi address used by the current network:

```bash
ros2 run rover_bridge rover_bridge \
  --ros-args \
    --params-file install/rover_bridge/share/rover_bridge/config/rover_bridge.yaml
```

For a different address, override only `pi_host`:

```bash
ros2 run rover_bridge rover_bridge \
  --ros-args -p pi_host:=192.168.10.50
```

The bridge reconnects its command, telemetry, and camera sockets independently.
The Pi watchdog remains the final motor stop mechanism if WSL2, ROS2, or the
network fails.

## Start communication and control

Use one terminal connected to the Pi and two terminals in WSL2. This is the
complete production startup sequence after the Pi services and WSL2 workspace
have been installed.

### 1. Raspberry Pi: start the hardware gateway

Run on the Pi over SSH or its local terminal. Replace the repository path only
if this checkout lives somewhere else.

```bash
cd /home/ws/ugv_rpi
git pull --ff-only

# The factory app must not own the UART at the same time.
crontab -l
pgrep -af app.py
pkill -f '/home/ws/ugv_rpi/ugv-env/bin/python /home/ws/ugv_rpi/app.py' || true

# Install the repository units into systemd, then reload its unit database.
sudo install -m 0644 systemd/rover-agent.service /etc/systemd/system/rover-agent.service
sudo install -m 0644 systemd/rover-camera.service /etc/systemd/system/rover-camera.service
sudo systemctl daemon-reload
sudo systemctl enable --now rover-agent.service rover-camera.service
systemctl --no-pager --full status rover-agent.service rover-camera.service
hostname -I
ss -ltn | grep -E ':(8765|8766|8770)\b'
```

Leave this terminal available for diagnostics:

```bash
journalctl -u rover-agent.service -f
```

The address printed by `hostname -I` is the value to use as `PI_HOST` in WSL2.
The default hotspot address is `192.168.50.5`.

### 2. WSL2 terminal 1: build and start the ROS2 bridge

Run on the Windows PC inside WSL2:

```bash
cd ~/ugv_rpi/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash

export PI_HOST=192.168.50.5
ros2 run rover_bridge rover_bridge \
    --ros-args -p pi_host:=$PI_HOST
```

Keep this terminal running. The bridge should report independent connections to
the command, telemetry, and camera ports. Use the Pi address from `hostname -I`
if it is different from `192.168.50.5`.

### 3. WSL2 terminal 2: verify ROS2 topics

Open a second WSL2 terminal and run:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ugv_rpi/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /battery_state
```

In another WSL2 terminal, camera rates can be checked with:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ugv_rpi/ros2_ws/install/setup.bash
ros2 topic hz /camera/color/image_raw --qos-profile sensor_data
ros2 topic hz /camera/depth/image_raw --qos-profile sensor_data
```

You should see `/battery_state`, `/camera/color/image_raw`,
`/camera/depth/image_raw`, `/camera/camera_info`, and `/rover/telemetry`.

### 4. WSL2 terminal 3: send a controlled motion command

Start with a slow straight command in a clear area and keep the Pi power switch
accessible:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ugv_rpi/ros2_ws/install/setup.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

For a slow in-place left turn, use a separate test only after straight motion
works:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.20}}"
```

The factory convention produces opposite-sign wheel speeds for in-place turns;
the production agent supports that convention.

### 5. Stop motion safely

The bridge repeats the latest command. Therefore, do not stop a nonzero motion
publisher with `Ctrl+C` before sending a zero command. Send zero first, then
stop the publisher:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Then press `Ctrl+C` in the motion terminal. To test the emergency stop path,
kill the bridge instead; closing the command socket makes the Pi send zero
wheels. Confirm the stop in the Pi agent log.

### 6. Shut down the production stack

Press `Ctrl+C` in the bridge terminal after sending the zero command. The Pi
services can remain enabled for the next run. To stop them completely:

```bash
sudo systemctl stop rover-camera.service rover-agent.service
```

Do this before starting `app.py`, a calibration script, or the standalone
recorder, because those programs also require `/dev/ttyAMA0`.

## ROS2 interfaces

Subscribed:

```text
/cmd_vel                  geometry_msgs/msg/Twist
```

Published:

```text
/rover/telemetry          std_msgs/msg/String      compact JSON
/battery_state            sensor_msgs/msg/BatteryState
/camera/color/image_raw   sensor_msgs/msg/Image   bgr8
/camera/depth/image_raw   sensor_msgs/msg/Image   16UC1 millimetres
/camera/camera_info       sensor_msgs/msg/CameraInfo
```

The bridge does not publish wheel odometry or TF yet. Those remain deferred
until encoder calibration is repeatable. The Pi recorder can write provisional
`/odom` and `odom -> base_link` data for offline inspection, but it must not be
treated as navigation-grade ground truth.

## Verification sequence

Run these checks from the appropriate host, in order.

1. **Pi link:** stop all other UART owners, then run:

    ```bash
    ./ugv-env/bin/python pedro_scripts/verify_esp32_link.py --duration 10
    ```

2. **Pi movement:** only with clear space and the power switch accessible:

    ```bash
    ./ugv-env/bin/python pedro_scripts/verify_esp32_link.py --drive --duration 2
    ```

3. **Production network:** start both Pi services, build the WSL2 bridge, and
    confirm telemetry and camera topics. Send only a slow `/cmd_vel` first.

4. **Watchdog:** stop publishing `/cmd_vel` or kill the bridge and confirm the
    Pi sends zero wheel speeds within `0.5` seconds.

5. **Camera:** confirm RGB/depth dimensions, camera info, packet rate, latency,
    and OAK-D temperature during a 20-minute run.

6. **Calibration:** repeat at least five straight and five diagonal-arc trials
    with [config/calibration.yaml](config/calibration.yaml) as the only active
    calibration store:

    ```bash
    ./ugv-env/bin/python pedro_scripts/calibrate_odometry.py straight --speed 0.39 --duration 4
    ./ugv-env/bin/python pedro_scripts/calibrate_odometry.py arc --left 0.06 --right 0.39 --duration 4
    ./ugv-env/bin/python pedro_scripts/calibrate_imu.py --duration 30
    ./ugv-env/bin/python pedro_scripts/calibrate_odometry.py summary
    ```

7. **Recording:** after calibration is populated, use `recorder/record.py` only
    with the production services stopped. Transfer the resulting bag to WSL2
    and replay it with `ros2 bag play <bag> --clock`.

## Implementation plan

The current project path is deliberately staged:

1. **Factory baseline:** preserve and verify the known-good `app.py` path,
    UART, ESP32 feedback, and factory turn signs.
2. **Production gateway:** run `rover_agent.py` on the Pi and `rover_bridge`
    in WSL2 over ports `8765`, `8766`, and `8770`. This is the current phase.
3. **Calibration:** establish left/right encoder scales, effective track
    width, gyro bias, encoder signs, and repeatability after the 30A battery
    upgrade.
4. **Synchronized recording:** capture host-clock RGB-D and raw ESP32 feedback
    on the Pi with `recorder/record.py` and `rosbags`.
5. **Offline mapping:** replay a completed bag in WSL2 and build the first
    RTAB-Map database using RGB-D visual odometry.
6. **Localization:** validate `map -> odom -> base_link`, camera transforms,
    and live or fused odometry against the saved map.
7. **Nav2:** add the 2D map, live depth costmap, tested `/cmd_vel` path, and
    emergency-stop behavior only after localization is stable.
8. **Outdoor autonomy:** add human-detection stop logic, field-network
    profiles, and GPS/MAVLink work only after indoor navigation is reliable.

## Definition of done

The rover is ready for autonomous experiments when all of the following pass:

- Factory manual control remains functional.
- `rover-agent.service` and `rover-camera.service` start without `app.py`.
- WSL2 reconnects after a network interruption.
- Telemetry and RGB-D topics arrive at the expected rates.
- A killed bridge stops the rover within `0.5` seconds.
- Encoder calibration is repeatable in forward, reverse, and diagonal motion.
- RGB-D bags replay without clock or TF errors.
- RTAB-Map creates and reloads a usable map.
- Localization publishes stable transforms.
- Nav2 completes a short indoor goal with a tested emergency stop.

## Repository layout

```text
app.py, base_ctrl.py, templates/    Factory manual-control baseline
rover_agent.py                      Production Pi hardware gateway
camera_stream.py                    Pi OAK-D RGB-D server
config/                             Runtime and calibration configuration
pedro_scripts/                      Link, IMU, and odometry tools
recorder/                           RGB-D, feedback, odometry, and bag code
ros2_ws/src/rover_bridge/           WSL2 ROS2 network bridge
systemd/                            Pi service units and clock example
requirements.txt                    Pi Python dependencies
LICENSE                             Upstream license
```

Generated bags, logs, caches, and local virtual environments are not source
files and should remain outside version control.
