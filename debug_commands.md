# Rover Quickstart

## Pi

### Restart services
Recommended recovery order
```bash
sudo systemctl restart rover-agent.service
sudo systemctl restart rover-camera.service
sudo journalctl -u rover-agent.service -f
```

Confirm the new process and listeners:

```bash
pgrep -af 'rover_agent'
sudo ss -ltnp | grep -E ':(8765|8766)\b'
```

### Check camera


```bash
# OAK-D health: the service must be active and its RGB-D stream must be listening.
if sudo systemctl is-active --quiet rover-camera.service && \
   ss -ltn | grep -qE ':(8770)\b'; then
  echo "OAK-D camera healthy: rover-camera.service is active and port 8770 is listening"
else
  echo "OAK-D camera NOT READY"
  sudo systemctl --no-pager --full status rover-camera.service
  sudo journalctl -u rover-camera.service -n 30 --no-pager
fi

# Recovery: a local connect timeout while port 8770 is listening usually means
# that the single-client server is holding stale WSL2 connections. Confirm the
# listener/backlog and then restart the camera service to clear those sockets.
ss -ltnp '( sport = :8770 )'
ss -tanp '( sport = :8770 or dport = :8770 )'
sudo journalctl -u rover-camera.service -n 30 --no-pager

sudo systemctl restart rover-camera.service
if timeout 30s bash -c 'until ss -ltn | grep -qE ":(8770)\b"; do sleep 1; done'; then
  echo "OAK-D camera listener recovered on port 8770"
else
  echo "OAK-D camera listener did not return after restart"
  sudo systemctl --no-pager --full status rover-camera.service
  sudo journalctl -u rover-camera.service -n 30 --no-pager
fi

# Pi-local OAK-D stream test. Stop live_bringup in WSL2 first because the
# camera service accepts one TCP client at a time.
./ugv-env/bin/python - <<'PY'
import json
import socket
import struct
import time

def receive(sock, size):
  data = bytearray()
  while len(data) < size:
    chunk = sock.recv(size - len(data))
    if not chunk:
      raise ConnectionError("camera stream closed")
    data.extend(chunk)
  return bytes(data)

started = time.monotonic()
frames = 0
previous_sequence = None
with socket.create_connection(("127.0.0.1", 8770), timeout=10.0) as client:
  client.settimeout(10.0)
  while time.monotonic() - started < 60.0:
    header_size = struct.unpack(">I", receive(client, 4))[0]
    header = json.loads(receive(client, header_size))
    rgb = receive(client, header["rgb_bytes"])
    depth = receive(client, header["depth_bytes"])
    sequence = header["seq"]
    if previous_sequence is not None and sequence != previous_sequence + 1:
      raise RuntimeError(f"camera sequence gap: {previous_sequence} -> {sequence}")
    if not rgb.startswith(b"\xff\xd8"):
      raise RuntimeError("RGB payload is not a JPEG")
    if not depth[12:].startswith(b"\x89PNG\r\n\x1a\n"):
      raise RuntimeError("depth payload is not compressedDepth PNG")
    previous_sequence = sequence
    frames += 1

elapsed = time.monotonic() - started
print(f"OAK-D local stream healthy: {frames} complete RGB-D packets in {elapsed:.1f}s ({frames / elapsed:.2f} Hz)")
PY
```

## WSL2: verify

Run in another WSL2 terminal:

## WSL2: camera stream stability test

With `live_bringup` running, use another WSL2 terminal. This watches the
compressed streams that actually cross the network for 60 seconds:

```bash
for topic in \
  /camera/color/image_raw/compressed \
  /camera/depth/image_raw/compressedDepth; do
  output="/tmp/$(basename "$topic").hz"
  timeout 60s ros2 topic hz --window 30 "$topic" >"$output" 2>&1 &
done
wait
cat /tmp/compressed.hz /tmp/compressedDepth.hz
```

Both topics should remain close to 15 Hz. A long maximum interval or a topic
with no rate output indicates an interruption. On the WSL2 terminal running
`live_bringup`, look for `Rover camera connection lost` and `Camera stats:`;
the latter should show `rx_fps` and `pub_fps` near 15 with no increasing
`dropped_frames` or `queue_drops`. On the Pi, also check:

```bash
sudo journalctl -u rover-camera.service -f
```

---

ros2 topic list relies heavily on the local ROS 2 CLI daemon’s cached graph. 
That daemon is stale or was started with a different DDS environment.
Restart the ROS 2 CLI daemon from the same environment you use for the bridge:

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 topic list -t
```