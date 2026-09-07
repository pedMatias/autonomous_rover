# Raspberry Pi Camera Update

Run these commands on the Raspberry Pi. They update the shared checkout, verify the new compressed-depth payload helper, restart the camera service, and leave the WSL2 bridge ready for live validation.

The commands assume the Pi checkout is `/home/ws/ugv_rpi`, matching the systemd service configuration. Set `PI_REPO` to the actual checkout if it is different.

Before running `git pull` on the Pi, commit and push these WSL2 changes to the
remote branch. The Pi cannot pull uncommitted changes that exist only in the
WSL2 working tree.

## 1. Stop the Camera Service

Do this before updating or testing the camera process:

```bash
export PI_REPO=/home/ws/ugv_rpi
cd "$PI_REPO"
sudo systemctl stop rover-camera.service
```

Confirm that no camera process remains:

```bash
pgrep -af 'camera_stream.py' || true
```

Do not run the standalone camera process and `rover-camera.service` at the same time.

## 2. Update the Pi Checkout

Check for local changes first:

```bash
cd "$PI_REPO"
git status --short
```

If the checkout is clean, pull the WSL/Pi code:

```bash
git pull --ff-only
```

If `git status --short` shows local changes, stop here and resolve them before pulling. Do not overwrite Pi-local changes with a forced reset.

## 3. Verify the Pi Python Environment

The camera service uses the project virtual environment:

```bash
cd "$PI_REPO"
"$PI_REPO/ugv-env/bin/python" -c \
  'import depthai; print(depthai.__version__)'
"$PI_REPO/ugv-env/bin/python" -m py_compile \
  camera_stream.py camera_stream_protocol.py recorder/oak_camera.py
```

Verify the compressed-depth header helper without connecting to the camera:

```bash
cd "$PI_REPO"
"$PI_REPO/ugv-env/bin/python" - <<'PY'
import struct

from camera_stream_protocol import build_compressed_depth_payload

png = b"\x89PNG\r\n\x1a\nprobe"
payload = build_compressed_depth_payload(png)
format_value, depth_param_a, depth_param_b = struct.unpack("<iff", payload[:12])
assert format_value == 0
assert depth_param_a == 0.0
assert depth_param_b == 0.0
assert payload[12:] == png
print("compressedDepth header: PASS")
PY
```

Expected result:

```text
compressedDepth header: PASS
```

## 4. Start the Updated Camera Service

```bash
sudo systemctl start rover-camera.service
systemctl --no-pager --full status rover-camera.service
ss -ltn | grep ':8770\b'
```

Check recent service logs:

```bash
journalctl -u rover-camera.service -n 50 --no-pager
```

Expected results:

- Service state is `active (running)`.
- TCP port `8770` is listening.
- The log shows the OAK-D stream is listening.
- No Python traceback or camera initialization error is present.

Do not start another `camera_stream.py` process if the service is active.

## 5. Prepare the WSL2 Bridge

Run on WSL2 in the repository checkout:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PI_HOST=192.168.0.12
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
```

Start the bridge in a dedicated terminal:

```bash
ros2 run rover_bridge rover_bridge \
  --ros-args \
  -p pi_host:="$PI_HOST" \
  -p camera_socket_timeout_s:=10.0
```

Wait for these bridge log entries:

```text
Connected to rover command port 8765
Connected to rover telemetry port 8766
Connected to rover camera port 8770
```

## 6. Validate Compressed Depth from WSL2

In another WSL2 terminal, use the explicit transport parameters and keep the process running:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 run image_transport republish \
  --ros-args \
  -p in_transport:=compressedDepth \
  -p out_transport:=raw \
  -r in/compressedDepth:=/camera/depth/image_raw/compressedDepth \
  -r out:=/camera/depth/image_raw_check
```

The startup log must say:

```text
The 'in_transport' parameter is set to: compressedDepth
The 'out_transport' parameter is set to: raw
```

In a third WSL2 terminal, check the output:

```bash
cd ~/projects/autonomous_rover
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 topic info -v /camera/depth/image_raw_check
ros2 topic hz -w 30 /camera/depth/image_raw_check
ros2 topic echo --once /camera/depth/image_raw_check --field encoding
```

Expected result:

```text
encoding: 16UC1
```

The republished depth rate should be close to the bridge rate, approximately `14-15 FPS`.

## 7. Validate Both Compressed Topics

With the bridge and republisher still running, use separate WSL2 terminals:

```bash
ros2 topic hz -w 30 /camera/color/image_raw/compressed
```

```bash
ros2 topic hz -w 30 /camera/depth/image_raw/compressedDepth
```

Expected result for both topics:

```text
approximately 14-15 FPS
```

Check the bridge statistics for dropped frames:

```text
Camera stats: rx_fps=..., pub_fps=..., dropped_frames=..., queue_drops=...
```

The normal result should have `rx_fps` and `pub_fps` near the camera rate, with no unexpected connection-loss messages.

## 8. Stop the Test

Stop the WSL2 republisher and bridge with `Ctrl+C` in their terminals.

When finished with the Pi camera service:

```bash
sudo systemctl stop rover-camera.service
```

Do not run a direct raw TCP counter while the bridge is connected because the Pi camera server accepts one client.
