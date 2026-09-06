# Camera and RTAB-Map Pipeline Changelog

Date: 2026-09-05

## Summary

The live rover camera pipeline was receiving RGB-D data from the Raspberry Pi over TCP, but the expected raw ROS2 image topics were not producing messages. The initial live launch used `image_transport republish` nodes to convert compressed topics to raw topics, but those nodes advertised raw outputs without subscribing to the compressed inputs. Enabling raw publication inside the Python bridge later made the raw topics appear, but caused severe camera stalls and repeated TCP reconnects.

The applied solution keeps the Pi-to-WSL stream compressed, keeps the Python bridge as a lightweight network/telemetry gateway, and lets RTAB-Map's ROS2 image pipeline decode compressed RGB-D data for live mapping. Existing offline recorder bags continue to use their original raw `/oak/...` topics.

## Observed Symptoms

Initial live launch output showed:

- `rover_bridge` connected to command port `8765`, camera port `8770`, and telemetry port `8766`.
- Camera receive/publication statistics around 13-17 FPS with no dropped frames before raw publication was enabled.
- `/camera/color/image_raw/compressed` and `/camera/depth/image_raw/compressedDepth` existed and received messages.
- `/oak/rgb/image_raw` and `/oak/stereo/image_raw` appeared in `ros2 topic list`, but `ros2 topic hz` produced no measurements.
- `/odom`, `/tf`, `/battery_state`, and `/oak/rgb/camera_info` were active.

The following diagnostic was especially significant:

```text
ros2 topic info -v /camera/color/image_raw/compressed
Publisher count: 1
Node name: rover_bridge
Subscription count: 0
```

`ros2 node info /rgb_republisher` and `/depth_republisher` showed only `/parameter_events` subscribers. The republishers were not connected to the compressed image topics.

After direct raw publication was enabled in `rover_bridge`, the camera stream degraded:

- Compressed depth rate fell to roughly 1-3 FPS with gaps up to about 11 seconds.
- Logs repeatedly reported `Rover camera connection lost: timed out`.
- `rx_fps` and `pub_fps` dropped to zero or very low values.
- `dropped_frames` and `queue_drops` increased.
- The Pi camera server uses a synchronous `read -> encode -> sendall` loop, so TCP backpressure could stall the Pi sender when the WSL side could not process and publish raw RGB-D quickly enough.

## Root Causes and Issues

### 1. Topic and transport mismatch

The bridge publishes compressed messages on:

```text
/camera/color/image_raw/compressed
/camera/depth/image_raw/compressedDepth
```

The original live launch expected transport base topics and attempted to republish them to:

```text
/oak/rgb/image_raw
/oak/stereo/image_raw
```

The deployed republisher configuration did not establish image subscriptions. Listing a topic only proves that a publisher has advertised it; it does not prove that messages are flowing.

### 2. Raw publication was not free

The compressed bytes had already crossed the Wi-Fi/TCP link, but publishing raw images still required substantial local work:

- JPEG decoding for RGB.
- PNG compressed-depth decoding for depth.
- Allocation and copying of approximately 1.4 MB of raw RGB-D data per frame at the observed dimensions.
- ROS2/DDS serialization and delivery of large `sensor_msgs/msg/Image` messages.
- Additional memory pressure and scheduling overhead in the Python `rclpy` bridge.

The bridge has separate reader and publisher threads, but that does not make the processing cost disappear. Once the host-side processing and DDS path fall behind, the TCP receive window can shrink. The Pi's blocking `sendall()` then stalls, and the bridge eventually reaches its camera socket timeout.

The important distinction is:

> Compressed network transport is efficient, but raw ROS2 publication after reception still has a significant CPU, memory, serialization, and DDS cost.

### 3. RTAB-Map cannot operate on compressed bytes without decoding

RTAB-Map's visual and depth algorithms ultimately need decoded pixel data. The preferred location for this decoding is a compiled C++ image transport/RTAB-Map pipeline, rather than the Python network bridge. RTAB-Map can subscribe through image transport so the compressed topics remain efficient until the consumer needs the pixels.

### 4. Live and offline topic layouts are different

The live bridge receives compressed topics under `/camera/...`.

The Pi-side recorder writes raw image messages into bags under:

```text
/oak/rgb/image_raw
/oak/stereo/image_raw
/oak/rgb/camera_info
```

Changing the offline mapping defaults to compressed `/camera/...` topics would break playback of existing recorded bags. The two workflows must remain explicit.

## Solutions Applied

### Bridge configuration

Updated `ros2_ws/src/rover_bridge/config/rover_bridge.yaml`:

```yaml
rgb_topic: /camera/color/image_raw
depth_topic: /camera/depth/image_raw
camera_info_topic: /oak/rgb/camera_info
publish_compressed: true
publish_raw: false
```

The Python bridge now forwards compressed RGB-D data and continues publishing telemetry, battery state, odometry, TF, and camera info. Its raw decode/publish implementation remains in the source for reversibility, but it is disabled in the live configuration.

### Live bringup

The live launch no longer starts the two broken `image_transport republish` processes. It starts:

- `rover_bridge`
- The static camera transform publisher

This keeps the camera socket path free from raw image processing in Python and avoids an unnecessary intermediate raw DDS hop.

### RTAB-Map transport configuration

Updated `ros2_ws/src/rover_slam/launch/offline_mapping.launch.py` to pass these arguments to the installed RTAB-Map launch:

```text
rgb_image_transport
depth_image_transport
```

The defaults remain raw for existing recorder bags:

```text
rgb_topic=/oak/rgb/image_raw
depth_topic=/oak/stereo/image_raw
rgb_image_transport=raw
depth_image_transport=raw
```

For live mapping, RTAB-Map can be launched against the bridge's compressed topics:

```bash
ros2 launch rover_slam offline_mapping.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  rgb_image_transport:=compressed \
  depth_image_transport:=compressedDepth \
  database_path:=/tmp/rover_live_mapping.db
```

This puts decompression in the RTAB-Map/image transport path instead of the Python bridge.

### Documentation

Updated `quickstart.md` to:

- Keep the image transport plugin installation step.
- Verify compressed live camera topics.
- Document the distinction between live compressed input and raw recorded-bag input.
- Provide the explicit live RTAB-Map command.

The GCS was intentionally left out of this change, as requested. Its current dashboard configuration still requests raw `/oak/...` topics and may need a separate transport adjustment later.

### GCS follow-up

The GCS was subsequently updated to work with raw bridge publication disabled:

- RGB uses `web_video_server` MJPEG with `default_transport=compressed` from
  `/camera/color/image_raw`.
- A dashboard-only `gcs_depth_preview` node decodes
  `/camera/depth/image_raw/compressedDepth` at 5 Hz, colorizes it, and publishes
  a JPEG preview on `/gcs/depth_preview/image_raw/compressed`.
- The preview node never publishes `sensor_msgs/msg/Image` raw topics and does
  not add any camera connection or network traffic to the Pi.

This adds a small, capped host-side decode/encode cost for the dashboard only;
RTAB-Map remains on the original compressed RGB-D topics.

## Validation Completed

The following checks passed after the changes:

- `colcon build --symlink-install --packages-select rover_bridge rover_slam`
- `python3 -m py_compile src/rover_slam/launch/offline_mapping.launch.py`
- `ros2 launch rover_slam offline_mapping.launch.py --show-args`
  - Confirmed `rgb_image_transport` and `depth_image_transport` are exposed.
- `python3 -m pytest src/rover_bridge/test -q`
  - 13 tests passed.
- `git diff --check`
  - No whitespace errors.

## Possible Next Steps

1. Restart `live_bringup` so the running process uses the compressed-only bridge configuration.
2. Verify the source stream before starting RTAB-Map:

   ```bash
   ros2 topic hz /camera/color/image_raw/compressed
   ros2 topic hz /camera/depth/image_raw/compressedDepth
   ```

   The camera should remain near its normal sustained rate without repeated socket timeouts.

3. Start live RTAB-Map with the explicit compressed transport arguments shown above.
4. Inspect RTAB-Map and `rgbd_sync` subscriptions to confirm they are connected to the compressed transport topics and that synchronized RGB-D messages are arriving.
5. Check that the generated database contains usable RGB-D data and that mapping remains stable under the available CPU and memory budget.
6. If compressed transport is not honored by a particular installed RTAB-Map component, use the verified C++ `image_transport republish` form as a fallback:

   ```bash
   ros2 run image_transport republish compressed raw \
     --ros-args \
     -r in:=/camera/color/image_raw \
     -r out:=/oak/rgb/image_raw

   ros2 run image_transport republish compressedDepth raw \
     --ros-args \
     -r in:=/camera/depth/image_raw \
     -r out:=/oak/stereo/image_raw
   ```

   The positional transport names are important. The remappings should use base topics; the selected input transport adds `/compressed` or `/compressedDepth` automatically.

7. Validate the GCS separately by confirming `/gcs/depth_preview/image_raw/compressed`
  and both browser streams while raw bridge publication remains disabled.
8. Consider adding an explicit live-mapping launch file separate from `offline_mapping.launch.py` once the compressed live path is validated, so raw recorded-bag defaults and live compressed defaults cannot be confused.
9. Only if the compressed RTAB-Map path still cannot keep up, evaluate rate limiting, resolution reduction, or a dedicated C++ RGB-D consumer. Do not move raw decoding back into the Python bridge without measuring the camera stream and host load again.
