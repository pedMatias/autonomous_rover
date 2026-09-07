# Phase 1: Compressed Streaming Bridge

**Status:** WSL2 and Pi streamer implementation complete; live Pi validation pending.
**Date:** 2026-08-30

## Goal

Forward the JPEG RGB and ROS2 `compressedDepth` PNG payloads produced by the Raspberry Pi directly into ROS2 `sensor_msgs/msg/CompressedImage` messages. This avoids decoding and republishing approximately 1.5 MB of raw image data per RGB-D pair inside the WSL2 bridge.

## WSL2 Implementation

The bridge in `ros2_ws/src/rover_bridge` now:

- Reads camera packets on a dedicated reader thread.
- Uses a size-one, lock-guarded mailbox. A newly received packet replaces an older unread packet, so stale frames do not accumulate behind DDS delivery.
- Publishes from a separate publisher thread, preventing ROS2 publication work from blocking TCP reads.
- Publishes JPEG RGB on `/camera/color/image_raw/compressed` with format `jpeg`.
- Publishes ROS2 compressed-depth PNG on `/camera/depth/image_raw/compressedDepth` with format `16UC1; compressedDepth png`.
- Uses `RELIABLE` and `VOLATILE` QoS for the compressed publishers so standard `image_transport` subscribers can connect.
- Publishes compressed RGB/depth unconditionally on the active bridge topics.
- Publishes camera info for every RGB-D frame on `/oak/rgb/camera_info` with the matching timestamp.
- Prefers a future Pi `stamp` header field, accepts the current legacy `ts` field, and falls back to WSL2 arrival time with a one-time warning if neither is present.
- Tracks received frames, successfully published frames, source sequence gaps, and mailbox replacements.
- Logs `rx_fps`, `pub_fps`, `dropped_frames`, and `queue_drops` every 5 seconds.

The Pi streamer now adds the ROS2 compressed-depth config header, plus `stamp`
and `seq` fields. The bridge remains compatible with older bare-PNG packets and
legacy `ts` headers.

Pi deployment and service restart commands are in
[pi_deployment.md](pi_deployment.md).

## Parameters

```yaml
frame_id: oakd_rgb_optical_frame
camera_info_topic: /oak/rgb/camera_info
```

Compressed publishers use `RELIABLE` QoS. CameraInfo uses the sensor-data
`BEST_EFFORT` profile and is published once per RGB-D frame.

## WSL2 Prerequisites

Install the ROS2 image transport plugins:

```bash
sudo apt install -y \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-compressed-depth-image-transport
```

Set this in every WSL2 shell that runs ROS2, including the bridge, rate tools, and republish process:

```bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
```

The Pi camera service accepts only one TCP client. Stop the bridge before running a direct raw TCP counter, and do not run the raw counter and bridge against port `8770` at the same time.

## Tests

Pure message-builder tests cover:

- RGB compressed message type, timestamp, frame ID, JPEG format, and payload.
- Depth compressed message type, timestamp, frame ID, compressed-depth format, and payload.
- Protocol preservation of `stamp` and `seq` header fields.

The focused package test and build passed after the refactor:

```text
colcon test --packages-select rover_bridge: 2 passed
colcon build --symlink-install --packages-select rover_bridge: succeeded
```

## Measurement Results

The baseline before compressed forwarding was:

| Measurement | Before Phase 1 | Phase 1 target | Phase 1 result |
| --- | ---: | ---: | ---: |
| Direct Pi-to-WSL2 raw TCP | 14.39 FPS | 14-15 FPS | Pending rerun |
| ROS2 RGB raw topic | 1.4-2.9 FPS | n/a | Legacy/debug path only |
| ROS2 depth raw topic | 1.5-3.0 FPS | n/a | Legacy/debug path only |
| ROS2 RGB compressed topic | n/a | >=13 FPS | Pending live run |
| ROS2 depth compressed topic | n/a | >=13 FPS | Pending live run |
| ROS2 image bandwidth | approximately 23 MB/s | <=1.5 MB/s | Pending live run |
| Camera socket timeouts | intermittent | 0 in 10-minute soak | Pending live run |

The pre-Phase-1 raw measurements showed that the Pi-to-WSL2 TCP path itself was healthy, while raw ROS2 image delivery was slow. No bridge decode/publish call exceeded 0.2 seconds in the focused timing sample.

## Compressed Depth Decision

The Pi now prepends the standard 12-byte compressed-depth config header to the
PNG. The bridge forwards the resulting payload unchanged using:

```text
16UC1; compressedDepth png
```

The required first live validation is:

```bash
ros2 run image_transport republish \
  --ros-args \
  -p in_transport:=compressedDepth \
  -p out_transport:=raw \
  -r in/compressedDepth:=/camera/depth/image_raw/compressedDepth \
  -r out:=/camera/depth/image_raw_check
ros2 topic hz /camera/depth/image_raw_check
```

If the republished depth flows and contains sane values, the passthrough convention is confirmed. The WSL2 raw-debug path also strips the 12-byte header before OpenCV decoding.

## Live Validation Procedure

1. Stop the bridge and measure the Pi raw TCP baseline.
2. Start the bridge with its default compressed publishers.
3. Measure both compressed topics with `ros2 topic hz -w 30`.
4. Measure both compressed topics with `ros2 topic bw`.
5. Run the compressed-depth republish check above.
6. Run a 10-minute soak and count `Rover camera connection lost` messages.
7. Keep raw `/oak/...` topics scoped to the separate Pi recorder and offline-bag workflow.

The Pi-side low-resolution/resource experiment and Pi capture/encode/send timing are outside this WSL2-only change and must be reported from the Raspberry Pi.
