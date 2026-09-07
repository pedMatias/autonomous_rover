
# Phase 1 — Compressed Streaming Bridge (WSL2)

**Goal:** restore full-rate (~15 FPS) RGB-D delivery on the WSL2 ROS2 machine by
forwarding the Pi's already-compressed streams instead of decoding and republishing raw.
**Repo:** `pedMatias/autonomous_rover`
**Scope:** WSL2 side (`ros2_ws/src/rover_bridge`), plus one small Pi-side protocol addition.
**Explicitly out of scope:** Pi stereo resource ladder (error 525), odometry publishing,
RTAB-Map bring-up — those are Phase 1.5/2 items.

---

## 1. Current state (verified against repo)

| Piece | Path | Today |
|---|---|---|
| Pi streamer | `camera_stream.py` | JPEG-encodes RGB, emits ROS2 compressedDepth metadata + PNG `uint16` depth, and sends the length-prefixed JSON header plus both payloads over TCP :8770 (`send_packet`). |
| Wire protocol | `ros2_ws/src/rover_bridge/rover_bridge/protocol.py` | `read_rgbd_packet(sock)` → `(header, rgb_bytes, depth_bytes)`. Fine as-is. |
| Bridge node | `ros2_ws/src/rover_bridge/rover_bridge/node.py` | Single-threaded loop: blocking `read_rgbd_packet` → `_publish_rgbd` → `cv2.imdecode` both streams → builds raw `sensor_msgs/Image` (≈1.5 MB/pair) + per-frame `CameraInfo`, all on `qos_profile_sensor_data`. |
| Tests | `ros2_ws/src/rover_bridge/test/test_protocol.py` | Protocol round-trip tests exist — pattern to reuse. |

**Observed:** raw TCP = 14.39 FPS; ROS2 topics = 1.4–3.0 FPS; no publish call > 0.2 s;
no decode errors. Diagnosis: raw republish through rclpy + Fast DDS default transport on
WSL2, amplified by `ros2 topic hz` as a slow Python observer.

**Target architecture after Phase 1:**

```text
Pi camera_stream.py
  │ JPEG + compressedDepth PNG payloads
  ▼ TCP :8770
rover_bridge (WSL2)
  ├─ reader thread:   read_rgbd_packet → size-1 mailbox (drop-stale)
  └─ publisher thread: mailbox → CompressedImage pubs (NO decode)
        ├─ /camera/color/image_raw/compressed       format="jpeg"
        ├─ /camera/depth/image_raw/compressedDepth  format="16UC1; compressedDepth png"
        └─ /oak/rgb/camera_info                     per RGB-D frame
Consumers decode once, in their own process, via image_transport hints.
```

---

## 2. Tasks

### T0 — Environment prerequisites (WSL2)

```bash
sudo apt install -y \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-compressed-depth-image-transport
```

Add to `~/.bashrc` (or a sourced env file) in **every** ROS2 shell on WSL2:

```bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA   # Jazzy: SHM+TCP path for large payloads
```

Document both in `docs/` and the README runbook section.

### T1 — Protocol: add capture timestamp + sequence (Pi + bridge)

The bridge currently stamps messages at WSL2 arrival time. That drifts from any future
Pi-stamped odometry and breaks TF/SLAM sync later. Cheap fix now:

- `camera_stream.py`: add `"stamp": time.time()` (capture time) and `"seq": n` to the
  JSON header in `send_packet`, and prepend the ROS2 compressedDepth config header
  to the PNG depth payload.
- `protocol.py`: no change needed (header is opaque JSON), but bump `MAX_HEADER_BYTES`
  guard only if header grows meaningfully.
- `node.py`: if `header["stamp"]` exists, build `builtin_interfaces/Time` from it;
  else fall back to `self.get_clock().now()` and log once that Pi stamps are missing.
- `test/test_protocol.py`: extend round-trip test to assert `stamp`/`seq` survive.

### T2 — Bridge restructure (`node.py`)

Refactor into two threads; keep the reconnect loop semantics intact.

1. **Reader thread** — owns the socket. Blocking `read_rgbd_packet`, writes into a
   size-1 mailbox (`queue.Queue(maxsize=1)` with drop-oldest, or a lock-guarded slot),
   increments `rx_count`/`drop_count`.
2. **Publisher thread** — takes the newest packet, builds messages, publishes;
   increments `pub_count`. DDS slowness can never back up the TCP socket; stale frames
   are dropped, not queued.
3. **New publishers** (keep `qos_profile_sensor_data`):

```python
from sensor_msgs.msg import CompressedImage

self._color_compressed_pub = self.create_publisher(
    CompressedImage, "/camera/color/image_raw/compressed", qos_profile_sensor_data)
self._depth_compressed_pub = self.create_publisher(
    CompressedImage, "/camera/depth/image_raw/compressedDepth", qos_profile_sensor_data)
```

```python
def _build_color_compressed(self, header, rgb_jpeg: bytes) -> CompressedImage:
    msg = CompressedImage()
    msg.header.stamp = self._stamp_from(header)
    msg.header.frame_id = self._frame_id          # param, default "oakd_rgb_optical_frame"
    msg.format = "jpeg"
    msg.data = rgb_jpeg                            # zero-copy wrap, no cv2
    return msg

def _build_depth_compressed(self, header, depth_png: bytes) -> CompressedImage:
    msg = CompressedImage()
    msg.header.stamp = self._stamp_from(header)
    msg.header.frame_id = self._frame_id
    msg.format = "16UC1; compressedDepth png"      # image_transport compressedDepth convention
    msg.data = depth_png
    return msg
```

4. **Camera frame:** use the compressed-only bridge with `frame_id`
  (default `"oakd_rgb_optical_frame"`).
5. **CameraInfo:** publish once per RGB-D frame with the matching timestamp.
6. **Stats:** every 5 s log one line: `rx_fps, pub_fps, dropped_frames, queue_drops`.
7. Extract message construction into pure functions (as sketched above) so they are
   unit-testable without a running ROS2 graph.

### T3 — Validation of the compressedDepth convention (do this FIRST, half hour)

Before building everything on the depth PNG passthrough, verify that
`compressed_depth_image_transport` accepts the Pi's PNG bytes as-is:

```bash
# bridge running with only the two compressed publishers
ros2 run image_transport republish \
  --ros-args \
  -p in_transport:=compressedDepth \
  -p out_transport:=raw \
  -r in/compressedDepth:=/camera/depth/image_raw/compressedDepth \
  -r out:=/camera/depth/image_raw_check
ros2 topic hz /camera/depth/image_raw_check
```

- If the republished raw depth flows and values look sane (check one frame in RViz or
  `ros2 topic echo --once` on a downscaled consumer): convention matches, proceed.
- If it rejects the format string or decodes garbage: **fallback** — decode depth in the
  publisher thread (`cv2.imdecode(..., IMREAD_UNCHANGED)`) and publish raw `16UC1`
  (0.61 MB/frame — tolerable), while color still forwards compressed. Decision point
  recorded in the Phase 1 report.

### T4 — Measurement tooling (no code, runbook only)

With compressed topics (~40–60 KB messages), `ros2 topic hz` is trustworthy again —
the earlier under-reporting was an artifact of deserializing 0.9–1.5 MB messages in Python.

Measurement protocol, in order:

1. Bridge stopped → Pi raw TCP counter → expect ≈14–15 FPS (baseline).
2. Bridge running → `ros2 topic hz /camera/color/image_raw/compressed` and
   `/camera/depth/image_raw/compressedDepth` → expect within ~10% of baseline.
3. `ros2 topic bw` on both compressed topics → expect ≈1 MB/s total.
4. `image_transport republish` raw check (T3) → confirms consumer-side decode cost.
5. 10-minute soak → count `Rover camera connection lost` events in bridge log
   (hypothesis: backpressure removal eliminates the intermittent timeouts).

### T5 — Docs + repo hygiene

- `docs/phase1_compressed_streaming.md`: this plan + results table (baseline vs after).
- README: update the system diagram to show compressed topics as the primary interface.
- Note the single-client constraint of the Pi camera server in the runbook (raw TCP test
  and bridge must never run simultaneously).

---

## 3. Suggested branch / commit breakdown

Branch: `fix/bridge-compressed-forwarding`

1. `refactor(bridge): split camera read/publish into threads with drop-stale mailbox`
2. `feat(bridge): forward JPEG/PNG as CompressedImage + compressedDepth`
3. `feat(bridge): make compressed forwarding and per-frame CameraInfo the active path`
4. `feat(pi): add capture stamp + seq to camera packet header`
5. `test(bridge): header round-trip + message builders`
6. `docs: phase 1 plan, measurement runbook, LARGE_DATA env setup`

---

## 4. Acceptance criteria

| Metric | Before | Target |
|---|---|---|
| `/camera/color/image_raw/compressed` rate | n/a | ≥13 FPS sustained 60 s |
| `/camera/depth/image_raw/compressedDepth` rate | n/a | ≥13 FPS sustained 60 s |
| Compressed bandwidth on ROS2 | ~23 MB/s (raw attempt) | ≤1.5 MB/s |
| Camera socket timeouts in 10-min soak | intermittent | 0 |
| Bridge log stats | none | `rx/pub/drop` every 5 s |
| Consumer raw check (republish) | n/a | works, or fallback decision recorded |

Raw `/oak/...` recorder topics remain available through the separate Pi recorder;
the network bridge no longer exposes a raw-image fallback.

---

## 5. Risks and notes

- **compressedDepth mismatch** (T3) is the only real technical risk; fallback keeps
  color compressed and decodes depth only — still a ~60% bandwidth cut.
- **`FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` must be set in every ROS2 process**
  (bridge, hz, republish, later RTAB-Map). Forgetting it in one shell gives confusing
  asymmetric results.
- **Clock skew:** until `chrony`/NTP syncs Pi and WSL2, Pi-stamped headers are correct
  *relative* to each other but offset from WSL2 clock. Fine for Phase 1; required before
  odometry/TF fusion in Phase 2.
- **Pi-side error 525 work is decoupled:** use the known-good production camera config
  for Phase 1 measurements; the low-resource ladder (mono 320×240, disable subpixel/LR
  check) proceeds in parallel on the Pi and only changes the payload size, not this design.
