# rover_avoidance

An experimental ROS 2 short-goal controller for the small rover. It accepts a
one-shot point relative to `base_link`, freezes the target in `odom`, and
chooses a clear local heading from the OAK-D depth image. It steers along a
forward arc to align, then drives straight. The swept arc is checked before
turning. A candidate path is rejected only by measured depth returns; pixels the stereo matcher
cannot measure (zero, or farther than `depth_max_m`) are treated as dropouts,
not obstacles. If no candidate corridor is clear, it stops with `BLOCKED`.
This is not a global route planner and may stop at fences, corners, or
obstacles it cannot see around.

The same pure planner can also run directly on the Pi without ROS2 or depth
compression. See the [camera-only browser preview](../../../docs/rover_preview.md)
for no-motion goals, frame-consistent debug views, and recorded failure replay.
Install this Python package through the preview requirements; only the ROS node
and compressed-message adapter need ROS/bridge dependencies.

For a staged first software and rover test, follow
[`docs/rover_avoidance_first_test.md`](../../../docs/rover_avoidance_first_test.md).

## Build

From `ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
colcon build --symlink-in



stall --packages-select rover_avoidance
source install/setup.bash
```

## Run

Start the existing live rover bringup first so `/odom`, compressed depth,
camera calibration, and `base_link` to camera TF are available. Use the same
three ROS environment exports in every terminal. Then start avoidance
explicitly:

```bash
ros2 launch rover_avoidance avoidance.launch.py
```

Do not run teleop at the same time: this prototype and teleop both publish
`/cmd_vel`, and this first version does not include command arbitration. The
node is not included in the regular rover or GCS launch files.

Send a short forward-relative goal (positive `y` means left):

```bash
ros2 topic pub --once --keep-alive 3 /avoidance/goal_relative geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link}, point: {x: 0.5, y: 0.0, z: 0.0}}"
```

State and stop reasons are published on `/avoidance/status` (`IDLE`, `DRIVE`,
`BLOCKED`, `GOAL_REACHED`, or `FAULT`). While a goal is active the node
publishes to `/cmd_vel`; sensor timeouts, missing calibration/TF, invalid
depth, and blocked local corridors cause a zero command. Speed and depth
thresholds are prototype tuning values, not measured hardware guarantees.

The node requires the depth image and camera calibration to have the same
frame ID, a fresh depth image, fresh odometry, and a TF transform from the
depth camera frame to `base_link`. Check the camera mount transform before
testing. A relative goal must have positive `x` (forward) and is frozen in
`odom` when received.

`missing_depth_blocks_paths` (default `false`) selects the missing-depth policy.
With it off, only measured returns can block a path, which is what makes a
textureless indoor scene drivable. Turn it on to restore the earlier
conservative rule where every unmeasured ray vetoes a corridor. In both modes a
zero reading is never proof of free space: a textureless or dark obstacle can
return nothing, so drive at the rover's minimum effective speed and treat the
physical power switch as the immediate stop.

Low-speed commands are a hardware hazard on this rover: below roughly 0.05 m/s
per wheel the motors stall against floor friction, draw locked-rotor current and
brown the rover out. Forward motion therefore uses `max_linear_speed_mps: 0.35`;
turns use forward arcs with both wheel sides moving in the same direction and
within their configured speed limits. See
[`docs/rover_low_speed_stall.md`](../../../docs/rover_low_speed_stall.md).

## See what avoidance sees in RViz2

The experiment config enables read-only debug topics. Restart avoidance after
building so the new publishers are loaded. With live bringup and avoidance
running, use a separate terminal with the same ROS setup and three exports:

```bash
rviz2
```

Set **Fixed Frame** to `base_link` and add these displays using **Add > By topic**:

| Topic | RViz display | Meaning |
| --- | --- | --- |
| `/avoidance/debug/points` | PointCloud2 | All valid sampled depth points, including floor and distant returns. |
| `/avoidance/debug/considered` | PointCloud2 | Only measured points inside the obstacle height band and lookahead used for path rejection. |
| `/avoidance/debug/paths` | MarkerArray | Red rejected paths, green clear paths, cyan chosen path, red spheres at the first blocking point, and a count label. |

Give the two point clouds different colors and small pixel sizes in RViz. The
clouds and paths are in rover-relative `base_link` coordinates: forward is
positive X, left is positive Y. Paths are *centerlines*, not rover outlines;
the label shows the clearance radius applied around them. With no goal, the
sampled-depth cloud updates but there are no candidate paths. `BLOCKED` freezes
the last paths and clouds until another goal arrives; no extra movement is
required to examine that decision. The display never sends driving commands.

### Preview paths without moving

Stop the normal avoidance launch and any teleop first; bringup can remain
running to supply depth. In the same ROS environment, launch the
preview mode **instead** of normal avoidance:

```bash
ros2 launch rover_avoidance avoidance.launch.py preview_only:=true
```

This node has no `/cmd_vel` publisher and does not subscribe to the driving
`/avoidance/goal_relative` topic. Its separate preview topic is:

```bash
ros2 topic pub --once --keep-alive 3 /avoidance/preview_goal_relative geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link}, point: {x: 2.0, y: 0.0, z: 0.0}}"
```

That sends a **hypothetical** forward-relative goal; it does not move the
rover. Preview evaluates it in `base_link` and does not need `/odom`; keep the
rover stationary so the fixed relative goal remains meaningful. Watch the
existing point clouds and path markers in RViz, or run
`ros2 topic echo /avoidance/preview_status` and
`ros2 topic echo --once /avoidance/debug/summary`. The summary reports
`PREVIEW_CLEAR` or `PREVIEW_BLOCKED`; candidate `linear` and `angular` values
are hypothetical, never published as commands. Preview keeps recomputing on
new camera frames even after a blocked result. Another preview goal replaces
the previous one; Ctrl+C ends the preview. Even before a goal, the considered
cloud and obstacle count show points inside the planner's height and lookahead
filter. The `clearance` label is the fixed collision radius (`0.18 + 0.025 m`),
not a live distance measurement. Stop preview before restarting the
normal avoidance launch to avoid two nodes publishing the same debug topics.
Other nodes can still command `/cmd_vel` independently, so stop teleop as well.

If all full paths are blocked, preview also looks for a short usable prefix of
one of the same candidate arcs. A **green line and endpoint** show the proposed
search step, up to `preview_search_max_step_m: 1.0` m. The step stops at least
`preview_search_stop_margin_m: 0.3` m *before* the first path-footprint
collision. `PREVIEW_SEARCH` and the summary's `search_step` report its distance
and rover-relative `(x, y)` endpoint. If even a short prefix is unavailable,
the state stays `PREVIEW_BLOCKED` and there is no green endpoint. A cyan path
means the original full-lookahead path is clear; no fallback is needed.
These are **hypothetical** positions, not a request to drive. The 0.3 m margin
is measured along the proposed path from the first footprint collision, not
from the rover's bumper; live fallback driving remains disabled until the
bumper offset and stopping distance are verified.

The numbers and individual decisions are also available as JSON:

```bash
ros2 topic echo --once /avoidance/debug/summary
```

`sampled_depth_points` counts valid decimated pixels; `considered_obstacle_points`
counts points actually used by the planner. `unknown_points` is normally zero
with the current missing-depth policy. Each candidate includes its heading,
commanded linear/angular speeds, clear/blocked result, and first blocking
`(x, y, z)` point. These are **points, not detected objects**: there is no
object grouping or object count. The summary and markers are generated from
the same depth frame and planner call that produced the driving decision.

To review a short test later, start recording *before* sending the goal:

```bash
ros2 bag record /avoidance/debug/points /avoidance/debug/considered \
  /avoidance/debug/paths /avoidance/debug/summary /avoidance/status
```

Stop recording with Ctrl+C. Replay the bag into RViz **without launching
avoidance** so replay does not cause new `/cmd_vel` commands. Set RViz Fixed
Frame to `base_link` when replaying these rover-relative topics.

## Unit tests

```bash
colcon test --packages-select rover_avoidance --event-handlers console_direct+
```