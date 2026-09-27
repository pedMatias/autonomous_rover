# First `rover_avoidance` test

This guide is for the experimental, short-goal prototype. It is not a general
navigation stack. The controller evaluates a handful of local paths, follows
forward arcs to align with a selected path, then drives straight along it.
Only measured depth returns block a path: pixels the stereo matcher cannot
measure are treated as dropouts, because a textureless ceiling or floor would
otherwise veto every corridor. Set `missing_depth_blocks_paths: true` in
`config/avoidance.yaml` to restore the conservative "unmeasured space blocks
the path" rule.

The experimental `obstacle_clearance_m: 0.18` setting accepts closer arcs than
the original 0.22 m half-width. This may clip obstacles; use open space beside
the test obstacle, watch the rover, and keep a hand on the physical switch.
`BLOCKED` status includes the nearest measured obstacle point in `base_link`
coordinates to help diagnose an unexpected stop.

> **Hardware limit -- never creep.** Slow commands stall the motors against
> floor friction and pull locked-rotor current until the rover shuts down, so
> forward motion uses `max_linear_speed_mps: 0.35` (the measured minimum) and
> turns keep both wheel sides moving forward. Never lower the forward speed
> to make the rover feel gentle. See
> [`rover_low_speed_stall.md`](rover_low_speed_stall.md).

## 1. Build and run the software tests

In a terminal at the repository root:

```bash
cd /home/pmati/projects/autonomous_rover/ros2_ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
colcon build --symlink-install --packages-select rover_avoidance
colcon test --packages-select rover_avoidance --event-handlers console_direct+
colcon test-result --verbose
source install/setup.bash
```

Expected: the package builds and the software tests pass.
These are software tests; they do not move the rover or validate the camera
mount.

## 2. Check the live inputs before sending a goal

Use a separate terminal, with the same environment setup:

```bash
source /opt/ros/jazzy/setup.bash
source /home/pmati/projects/autonomous_rover/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
ros2 launch rover_slam live_bringup.launch.py
```

In another similarly configured terminal, check the expected topics:

```bash
ros2 topic hz /odom
ros2 topic hz /camera/depth/image_raw/compressedDepth
ros2 topic echo --once /oak/rgb/camera_info
ros2 run tf2_ros tf2_echo base_link oak_rgb_camera_optical_frame
```

Depth and camera info must have the same frame ID. The live bringup now
defaults `camera_z` to `0.12` m (the OAK-D is 17 cm above the floor and
`base_link` is 5 cm above it) and keeps `camera_x = camera_y = 0.0` until those
offsets are measured. Verify the translation matches the actual mount and that
`tf2_echo` resolves before testing movement. Adjust `camera_x`, `camera_y`,
`camera_z`, `camera_roll`, `camera_pitch`, or `camera_yaw` on the live bringup
if needed.

## 3. Start the opt-in controller

In a third terminal, use the same ROS environment exports and run:

```bash
ros2 launch rover_avoidance avoidance.launch.py
```

The node is intentionally not included in the normal bringup. Do not start
teleop at the same time; both publish `/cmd_vel` and this prototype does not
arbitrate between command sources.

Watch its state and commands from another terminal:

```bash
ros2 topic echo /avoidance/status
ros2 topic echo /cmd_vel
```

## 4. Send one short test goal

Start with a short, clear, level path and a small relative distance. Positive
`x` is forward and positive `y` is left:

```bash
ros2 topic pub --once --keep-alive 3 /avoidance/goal_relative geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link}, point: {x: 0.5, y: 0.0, z: 0.0}}"
```

Expected behavior: status changes to `DRIVE`; `/cmd_vel` keeps positive linear
speed even when angular speed is nonzero while following a clear arc. It does
not rotate in place. The rover should stop and
publish `GOAL_REACHED` near the target. If it reports `FAULT`, check the
sensor topics, frame IDs, TF, and image depth decoding. If it reports
`BLOCKED`, it did not find an observed clear candidate path.

After the clear-path test, repeat with a soft, stationary obstacle placed in
the forward corridor and enough open space on either side. It may turn toward
an alternative local path, or report `BLOCKED` if its depth rays indicate
unknown/blocked space. This simple prototype does not reason about corners,
moving obstacles, or full routes.

Stop the goal by publishing a new goal only after the previous one terminates;
this version does not have a dedicated cancel topic. Stop the controller with
Ctrl+C. The bridge's existing command timeout provides a stop if commands
cease arriving, but treat the rover's physical power switch as the immediate
manual stop.

## Troubleshooting

- **No status or command:** confirm all terminals use the three ROS exports,
  and that the goal is `PointStamped` with `frame_id: base_link` and `x > 0.05`.
- **`FAULT: camera-to-base transform unavailable`:** start live bringup and
  check TF parent/child frame IDs.
- **`FAULT: too little valid depth`:** check that the camera is streaming and
  that the depth topic contains a `16UC1` compressedDepth PNG stream.
- **Straight path appears blocked:** the depth image may have unknown rays in
  the checked corridor. Confirm depth alignment and camera transform before
  changing the depth thresholds.
- **Motion does not match the command:** stop the test and check wheel/odometry
  direction conventions before trying another goal.