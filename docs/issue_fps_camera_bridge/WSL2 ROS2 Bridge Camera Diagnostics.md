# WSL2 ROS2 Bridge Camera Diagnostics

**Observation date:** 2026-08-30  
**Observer:** WSL2 side of the rover system  
**Pi address tested:** `192.168.0.12`  
**Camera TCP port:** `8770`

## Scope

This report describes what was observed on the WSL2 machine while receiving RGB-D data from the Raspberry Pi through the `rover_bridge` ROS2 node. It is intended to be compared with a corresponding Raspberry Pi-side report.

The path under test is:

```text
OAK-D -> Raspberry Pi camera_stream.py -> TCP port 8770
      -> WSL2 rover_bridge -> ROS2/DDS -> ROS2 subscribers
```

## Executive Summary

The Raspberry Pi camera stream and the Pi-to-WSL2 raw TCP path are working at the expected camera rate. A direct TCP client received approximately 14.39 packets per second for 30 seconds.

The ROS2 bridge also starts, connects to the Pi, creates the expected publishers, decodes packets, and publishes `sensor_msgs/msg/Image` messages. However, ROS2 subscribers on WSL2 receive only approximately 1.4 to 3.0 frames per second during the measurements. This is substantially below the approximately 15 FPS available on the raw TCP path.

The bridge's decode-and-publish operation did not take longer than the 200 ms warning threshold during the focused instrumented run. That makes synchronous Python processing blockage less likely as the primary cause. The remaining problem is in, or after, the bridge's ROS2 publication path: DDS transport, message copying/serialization, subscriber processing, or queue/sample loss. Earlier runs also showed camera socket timeouts, so upstream timing instability has been observed as well, but the raw TCP test demonstrates that the Pi can provide a stable stream when the socket is used directly.

## What Is Working

### Network connectivity

- WSL2 successfully connected to `192.168.0.12:8770`.
- The bridge also connected to the command and telemetry ports:
  - command: `8765`
  - telemetry: `8766`
  - camera: `8770`
- The TCP camera protocol was readable by the WSL2 client.
- The direct raw TCP test received complete RGB-D packets without decode errors:

```text
packets=432
elapsed=30.03s
fps=14.39
throughput=0.94 MiB/s
```

This confirms that the Pi-to-WSL2 camera connection can sustain approximately 14.4 packets per second independently of ROS2.

### ROS2 bridge startup

- `rover_bridge` launched successfully with:

```bash
ros2 run rover_bridge rover_bridge \
  --ros-args \
  -p pi_host:=192.168.0.12 \
  -p camera_socket_timeout_s:=10.0
```

- The bridge reported that it was bridging the expected Pi address and ports.
- The bridge connected to the camera service.
- The bridge source was rebuilt successfully with `colcon build --symlink-install --packages-select rover_bridge`.
- The bridge uses the expected sensor-data QoS profile for camera topics, including `BEST_EFFORT` reliability and `VOLATILE` durability.

### ROS2 topics and message construction

Both image publishers were present:

```text
/camera/color/image_raw   sensor_msgs/msg/Image
/camera/depth/image_raw  sensor_msgs/msg/Image
```

The bridge code successfully performed the following for received packets:

- JPEG RGB decode with OpenCV
- PNG depth decode with OpenCV
- Conversion of depth to single-channel `uint16` when needed
- Construction of `bgr8` RGB messages
- Construction of `16UC1` depth messages
- Publication of camera info messages

The bridge did not report `could not decode rover RGB-D packet` during the focused instrumented run.

### DepthAI availability in the sibling project

The local active interpreter did not have DepthAI:

```text
Interpreter: /home/pmati/venv-ardupilot/bin/python3
DepthAI import: unavailable
```

DepthAI is declared in this repository's `requirements.txt` as:

```text
depthai==3.8.0
```

An existing sibling-project environment does have DepthAI installed:

```text
Interpreter: /home/pmati/projects/depthai/depthai-core/.venv/bin/python
DepthAI version: 3.9.0
Module: /home/pmati/projects/depthai/depthai-core/.venv/lib/python3.12/site-packages/depthai.cpython-312-x86_64-linux-gnu.so
```

Using that interpreter, `camera_stream.py --help` ran successfully. This confirms that the missing import in the WSL2 system interpreter is an environment-selection issue, not proof that the project cannot use DepthAI.

## What Is Not Working as Expected

### ROS2 image delivery rate

The raw TCP stream is approximately 14.39 FPS, but ROS2 subscribers receive much less.

Initial measurement with the existing bridge:

```text
Color: approximately 1.410 Hz
Depth: approximately 1.494 Hz
```

A second measurement after restarting the bridge from the instrumented build:

```text
Color: 2.867 Hz
Depth: 2.984 Hz
```

The exact rate varied between runs, but remained far below the approximately 15 FPS source rate. The ROS2 topic tools reached their 30-message measurement windows, so these results are not just a small-initial-sample artifact.

### Lightweight combined subscriber

The one-process Python subscriber intended to count both image topics for 30 seconds did not complete. It failed while reading the camera stream with:

```text
TimeoutError: timed out
```

This occurred in `read_rgbd_packet()` while waiting for the RGB-D packet header. The failure means that this particular counter run cannot be used as a valid ROS2 receive-rate result. It also shows that camera timing can be intermittent under the tested conditions.

### Camera socket timeouts in earlier bridge logs

The WSL2 ROS2 logs contained repeated messages of the form:

```text
Rover camera connection lost: timed out
```

The logs also contained successful connection messages, including:

```text
Connected to rover camera port 8770
Bridging rover at 192.168.0.12 (commands 8765, telemetry 8766, camera 8770)
```

During the focused instrumented run, no camera connection-loss messages occurred. Therefore, timeouts are an observed intermittent condition, not a constant failure of the TCP link.

### No stable full-rate ROS2 result

No WSL2 ROS2 measurement approached the raw TCP rate. At present, WSL2 is receiving valid data, but not at the camera's available rate. For SLAM or other consumers, this means the ROS2 image topics may be too sparse and may have irregular timing.

## Focused Publish-Timing Experiment

Temporary timing instrumentation measured the elapsed time around:

```python
self._publish_rgbd(header, rgb_bytes, depth_bytes)
```

The instrumentation warned for calls taking more than `0.2` seconds. Results:

```text
RGB-D processing warnings: none
Camera connection-loss messages: none
Color ROS2 rate: 2.867 Hz
Depth ROS2 rate: 2.984 Hz
```

No call exceeded the warning threshold during that run. This does not prove that every operation is always fast, but it does show that the low observed ROS2 rate was not accompanied by obvious long synchronous publish/decode stalls in that sample.

The temporary instrumentation was removed afterward, and the package was rebuilt successfully.

## ROS2 Publisher Information

For both image topics, `ros2 topic info -v` reported:

```text
Type: sensor_msgs/msg/Image
Publisher count: 1
Publisher: rover_bridge in /
Reliability: BEST_EFFORT
Durability: VOLATILE
Subscription count: 0
```

The zero subscription count was observed when no `ros2 topic hz` or `ros2 topic bw` process was active. It is therefore not evidence that the publishers have no subscribers in general; those command-line measurement tools create temporary subscriptions.

## Data Size and Likely Load

The bridge receives compressed JPEG/PNG packets from the Pi but publishes raw ROS2 image messages. At the previously observed full resolution, approximate raw message sizes were:

```text
RGB:   0.92 MB per frame
Depth: 0.61 MB per frame
Total: 1.53 MB per RGB-D pair
```

At 15 FPS, this represents approximately:

```text
RGB:   13.8 MB/s
Depth:   9.2 MB/s
Total:  23.0 MB/s
```

These figures exclude DDS metadata, serialization, memory copies, queueing, and subscriber overhead. `BEST_EFFORT` permits dropped samples, but it does not reduce the size or processing cost of messages that the bridge creates and attempts to publish.

This is consistent with the observed behavior: the TCP stream transfers compressed data efficiently, while the ROS2 path expands it into large raw messages and delivers only a fraction of the source frame rate.

## Current WSL2 Diagnosis

The evidence supports the following diagnosis:

```text
OAK-D capture:                 not directly tested from WSL2; no USB access here
Pi camera process:             presumed active; raw TCP proves stream availability
Pi -> WSL2 raw TCP:            working at 14.39 FPS
Bridge startup:                working
Bridge TCP packet decoding:    working for received packets
RGB ROS2 publisher:            present and publishing
Depth ROS2 publisher:          present and publishing
Camera info publisher:         present
Bridge CPU starvation:         not observed in earlier system measurements
Bridge publish blocking:       no >0.2 s calls in focused sample
ROS2 receive rate:             not working at target rate, approximately 1.4-3.0 FPS
Camera socket stability:       intermittent timeouts observed in earlier run
Primary suspected issue:      raw Image delivery through ROS2/DDS and/or subscriber loss
```

The direct TCP result rules out a continuously saturated or fundamentally broken Pi-to-WSL2 network path. The low ROS2 rates, large raw messages, and absence of long bridge publish calls point toward the DDS delivery path, ROS2 middleware configuration, queue/sample dropping, or the overhead of the subscriber tools. Intermittent camera socket timeouts remain a secondary issue to correlate with the Raspberry Pi report.

## Tests Not Completed from WSL2

### Raspberry Pi low-resolution experiment

The proposed Pi-side test was not run from WSL2:

```bash
./ugv-env/bin/python camera_stream.py \
  --fps 5 \
  --rgb-width 320 \
  --rgb-height 240 \
  --jpeg-quality 60 \
  --png-compression 1
```

Reasons:

- No SSH username or configured host alias was available for `192.168.0.12`.
- The local WSL2 environment has no OAK-D USB access.
- The local active interpreter lacked DepthAI, although the sibling `depthai-core/.venv` interpreter can import it.

This experiment must be run on the Raspberry Pi, using the Pi's DepthAI environment and camera hardware, then measured again from WSL2.

### Simultaneous raw TCP and bridge measurement

A direct raw TCP client cannot be run at the same time as the normal bridge against the same camera service when the Pi camera server accepts only one client. The raw measurement was therefore performed with the bridge stopped, while ROS2 measurements were performed with the bridge connected.

## Recommended Next Comparison with the Pi Report

The Raspberry Pi-side report should include:

- OAK-D capture FPS and whether frames are being dropped before encoding
- RGB and depth dimensions and dtypes
- JPEG and PNG encoded byte sizes over time
- Per-frame capture, encode, and `sendall()` durations
- Camera service logs around the times WSL2 reported socket timeouts
- CPU, memory, temperature, and swap state
- Whether the camera server accepts one client and whether any other client connects
- Results using `320x240`, `--fps 5`, JPEG quality `60`, and PNG compression `1`
- The exact Python interpreter and installed DepthAI version used by `camera_stream.py`

The most discriminating next test is the lower-resolution stream. If WSL2 receives close to 5 FPS at `320x240`, the evidence will strongly confirm that raw ROS2 image bandwidth and delivery cost are the limiting factor. If the rate remains near 1-3 FPS, DDS configuration, WSL2 networking, subscriber behavior, or intermittent packet timing requires further investigation.

## Suggested Production Direction

The current bridge processes one packet at a time and publishes full raw RGB and depth messages. Likely improvements are:

- Use a bounded latest-frame queue so stale frames are discarded.
- Avoid allowing old RGB-D pairs to build up behind slow ROS2 delivery.
- Publish camera information once per connection or only when calibration changes.
- Use lower RGB-D resolution for SLAM, such as `320x240` or `424x240`, if the algorithm permits it.
- Consider compressed ROS image transport for the color stream.
- Consider publishing depth at a lower rate if the SLAM configuration permits it.
- Measure with a minimal compiled or non-Python subscriber in addition to `ros2 topic hz`.
- Correlate bridge socket timeouts with Pi capture and encoding timing before changing the network configuration.

## Bottom Line

WSL2 is connected to the Raspberry Pi and is receiving valid RGB-D data through the ROS2 bridge. The camera stream itself is available at approximately 14.4 FPS over raw TCP. The failure is that the ROS2 image topics deliver only approximately 1.4-3.0 FPS under the tested full-resolution conditions, with occasional camera socket timeouts in earlier runs.

The strongest current hypothesis is that the expanded raw RGB-D messages are being dropped or throttled somewhere in ROS2/DDS or by the subscriber path on WSL2. The Raspberry Pi-side report and the lower-resolution experiment are needed to determine whether the intermittent socket timeouts originate on the Pi and to confirm how strongly message size controls the ROS2 rate.
