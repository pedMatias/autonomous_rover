# Phase 1 WSL2 Test Guide

Run these tests from WSL2. The Raspberry Pi must be running `rover-camera.service` for the live tests.

Set the Pi address used by the current network:

```bash
export PI_HOST=192.168.0.12
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
```

The Pi camera server accepts one TCP client. Stop the bridge before running the raw TCP test, and do not run the raw TCP test and bridge at the same time.

## 1. Prepare WSL2

```bash
cd ~/projects/autonomous_rover
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2_ws/install/setup.bash
```

If the ROS2 image transport packages are not installed:

```bash
sudo apt install -y \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-compressed-depth-image-transport
```

## 2. Offline Tests

Run the bridge package tests and build:

```bash
cd ~/projects/autonomous_rover/ros2_ws
colcon test --packages-select rover_bridge
colcon test-result --verbose
colcon build --symlink-install --packages-select rover_bridge
source install/setup.bash
```

Expected result:

```text
4 tests passed
rover_bridge build succeeded
```

The exact test count may change as tests are added; all tests must pass.

## 3. Check Pi Reachability

Run before starting the bridge:

```bash
cd ~/projects/autonomous_rover
nc -vz -w 3 "$PI_HOST" 8765
nc -vz -w 3 "$PI_HOST" 8766
nc -vz -w 3 "$PI_HOST" 8770
```

Expected result: all three TCP ports are reachable.

## 4. Raw TCP Baseline

Make sure no `rover_bridge` process is connected to the camera port before running this test:

```bash
cd ~/projects/autonomous_rover
python3 - <<'PY'
import os
import socket
import sys
import time

sys.path.insert(0, os.path.abspath("ros2_ws/src/rover_bridge"))
from rover_bridge.protocol import read_rgbd_packet

host = os.environ["PI_HOST"]
port = 8770
duration = 30.0
count = 0
bytes_total = 0
start = time.monotonic()

with socket.create_connection((host, port), timeout=10) as sock:
    sock.settimeout(10)
    deadline = start + duration
    while time.monotonic() < deadline:
        header, rgb, depth = read_rgbd_packet(sock)
        count += 1
        bytes_total += len(rgb) + len(depth)

elapsed = time.monotonic() - start
print(f"packets={count}")
print(f"elapsed={elapsed:.2f}s")
print(f"fps={count / elapsed:.2f}")
print(f"throughput={bytes_total / elapsed / 1024 / 1024:.2f} MiB/s")
PY
```

Record `packets`, `fps`, and `throughput`. The previous baseline was approximately `14.39 FPS`.

## 5. Bridge Launch

Use a dedicated terminal:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2_ws/install/setup.bash
export PI_HOST=192.168.0.12
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 run rover_bridge rover_bridge \
  --ros-args \
  -p pi_host:="$PI_HOST" \
  -p publish_compressed:=true \
  -p publish_raw:=false \
  -p camera_socket_timeout_s:=10.0
```

Expected bridge log entries:

```text
Connected to rover command port 8765
Connected to rover telemetry port 8766
Connected to rover camera port 8770
```

Leave this terminal running during the ROS2 tests.

## 6. Verify Topics and QoS

In another terminal:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 topic list
ros2 topic info -v /camera/color/image_raw/compressed
ros2 topic info -v /camera/depth/image_raw/compressedDepth
ros2 topic info -v /camera/color/camera_info
```

Expected topics:

```text
/camera/color/image_raw/compressed
/camera/depth/image_raw/compressedDepth
/camera/color/camera_info
```

Expected image publisher QoS:

```text
Reliability: BEST_EFFORT
Durability: VOLATILE
```

## 7. Measure Compressed Topic Rates

Use separate terminals, with the bridge still running:

Terminal 1:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
ros2 topic hz -w 30 /camera/color/image_raw/compressed
```

Terminal 2:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
ros2 topic hz -w 30 /camera/depth/image_raw/compressedDepth
```

Run each for at least 60 seconds and record the average rate. Target:

```text
RGB compressed:   >= 13 FPS
Depth compressed: >= 13 FPS
```

## 8. Measure Compressed Bandwidth

With the bridge still running, use separate terminals:

```bash
ros2 topic bw /camera/color/image_raw/compressed
```

```bash
ros2 topic bw /camera/depth/image_raw/compressedDepth
```

Record the average bandwidth for each topic and their sum. Target:

```text
Combined compressed bandwidth: <= 1.5 MB/s
```

## 9. Validate Compressed Depth

Start the republisher in a separate terminal:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 run image_transport republish compressedDepth raw \
  --ros-args \
  -r in/compressedDepth:=/camera/depth/image_raw/compressedDepth \
  -r out:=/camera/depth/image_raw_check
```

In another terminal, measure the republished topic:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 topic hz /camera/depth/image_raw_check
ros2 topic echo --once /camera/depth/image_raw_check
```

Pass condition:

- The republished topic receives messages.
- The message type is `sensor_msgs/msg/Image`.
- The encoding is `16UC1`.
- Width and height match the camera stream.
- Depth values are finite and plausible.

If republishing rejects the Pi PNG bytes or produces invalid values, record the failure. The implementation must then retain compressed RGB but add the planned WSL2 depth fallback: decode PNG in the publisher thread and publish raw `16UC1` depth.

## 10. Check Bridge Statistics and Timeouts

Leave the bridge running for 10 minutes. In the bridge terminal, record the 5-second statistics lines:

```text
rx_fps=...
pub_fps=...
dropped_frames=...
queue_drops=...
```

After 10 minutes, count camera connection-loss messages from the bridge output or ROS log:

```bash
grep -R -Ei \
  'Rover camera connection lost|could not publish rover RGB-D packet|decode|error|timeout' \
  ~/.ros/log 2>/dev/null | tail -100
```

Pass condition:

```text
No camera connection-loss events during the 10-minute soak.
```

Some `queue_drops` are acceptable if publishing or a test subscriber is slower than the source. The important behavior is that the reader continues receiving packets and stale frames do not accumulate.

## 11. Optional Legacy Raw A/B Test

Stop the normal bridge with `Ctrl+C`, then start it with raw publishing enabled:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2_ws/install/setup.bash
export PI_HOST=192.168.0.12
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 run rover_bridge rover_bridge \
  --ros-args \
  -p pi_host:="$PI_HOST" \
  -p publish_compressed:=false \
  -p publish_raw:=true \
  -p camera_socket_timeout_s:=10.0
```

Measure the legacy topics only if a raw comparison is needed:

```bash
ros2 topic hz -w 30 /camera/color/image_raw
ros2 topic hz -w 30 /camera/depth/image_raw
```

Stop this bridge before returning to the compressed test.

## 12. Shutdown

Send a zero motion command before stopping the bridge if any motion command was sent:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Then press `Ctrl+C` in the bridge, republisher, and measurement terminals.

## Results to Record

```text
Pi host:
Raw TCP FPS:
Raw TCP throughput:
Compressed RGB FPS:
Compressed depth FPS:
Compressed RGB bandwidth:
Compressed depth bandwidth:
Depth republish: pass/fail
Bridge rx_fps:
Bridge pub_fps:
Bridge dropped_frames:
Bridge queue_drops:
Camera connection-loss events in 10 min:
Notes/errors:
```
