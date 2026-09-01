# 1. Map collection + Auntonomous driving
## Flow 1 — Mapping run (creating the map)

RTAB-Map requires four synchronized inputs: RGB image, registered depth image, camera info, and an odometry stream.

```text
┌─ SENSORS (always on) ──────────────────────────────────────────────────┐
│                                                                        │
│  OAK-D Lite ──► Pi camera_stream.py ──► TCP ──► rover_bridge           │
│        ├─ /camera/color/image_raw          (compressed → decoded once) │
│        ├─ /camera/depth/image_raw          (aligned depth)             │
│        └─ /camera/color/camera_info        (intrinsics)                │
│                                                                        │
│  ESP32 encoders ──► rover_agent.py ──► /rover/telemetry                │
│        └─► odometry integrator ──► /odom  (nav_msgs/Odometry)          │
│                                   └─► TF: odom → base_link             │
└────────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─ SYNC (message_filters) ───────────────────────────────────────────────┐
│  rgbd_sync: pairs {rgb + depth + camera_info} with the nearest /odom   │
│  sample BY TIMESTAMP ──► /rtabmap/rgbd_image                           │
└────────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─ RTAB-MAP NODE — MAPPING MODE (Mem/IncrementalMemory=true) ────────────┐
│  per incoming RGB-D frame:                                             │
│    1. extract visual features from RGB                                 │
│    2. lift them to 3D using depth                                      │
│    3. add node to pose graph, constrained by /odom motion estimate     │
│    4. loop closure: "have I seen this place before?" (bag-of-words     │
│       on features) → if yes, add constraint + optimize whole graph     │
│                                                                        │
│  publishes:                                                            │
│    ├─ TF: map → odom        (the correction; odom drifts, map doesn't) │
│    ├─ /rtabmap/grid_map     2D occupancy grid  ──┐                     │
│    ├─ /rtabmap/cloud_map    3D point cloud     ──┤ the "nuvem de       │
│    └─ /rtabmap/mapData      full graph data    ──┘ pontos"             │
│                                                                        │
│  continuously appends to: ~/.ros/rtabmap.db  (the saved map)           │
└────────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
              drive until the area is covered,
              verify loop closures in rtamapviz/RViz, kill it.
              The .db file IS the deliverable.
```

## Flow 2 — Autonomous driving (using the map)

Now the data flow splits into two loops: a slow "where am I / where am I going" loop, and a fast "what's in front of me right now" loop. RTAB-Map runs in **localization mode** — it loads the `.db`, stops growing the map, and only matches live frames against it to output your pose in the map frame. 

```text
┌─ LOCALIZATION LOOP (slow, ~1-10 Hz) ───────────────────────────────────┐
│                                                                        │
│  sensors ──► (same plumbing as Flow 1) ──► RTAB-Map LOCALIZATION MODE  │
│    loads ~/.ros/rtabmap.db, matches live RGB-D against saved map       │
│    └─► TF: map → odom      ("you are HERE in the saved map")           │
│                                                                        │
│  wheel odom + IMU ──► robot_localization EKF ──► filtered /odom        │
│  (GPS via MAVROS later ──► navsat_transform ──► same EKF)              │
└────────────────────────────────────────────────────────────────────────┘
              │ pose in map frame                    │ smooth local motion
              ▼                                      ▼
┌─ NAV2 ─────────────────────────────────────────────────────────────────┐
│                                                                        │
│  GPS waypoint ──► convert to map frame ──► goal pose                   │
│                          │                                             │
│                          ▼                                             │
│  planner_server:  global path A→B on the STATIC map                    │
│                   (static layer = RTAB-Map's saved grid_map)           │
│                          │                                             │
│                          ▼                                             │
│  controller_server: local trajectory, re-planned at ~10-20 Hz          │
│    local costmap layers:                                               │
│      ├─ static layer     ← the saved map (known walls/rows)            │
│      ├─ obstacle layer   ← LIVE depth → pointcloud_to_laserscan        │
│      │                     or PointCloud2 voxel layer                  │
│      │                     (things that MOVED since mapping:           │
│      │                      people, tools, a parked tractor)           │
│      └─ inflation layer  ← safety margin around everything             │
│                          │                                             │
│                          ▼                                             │
│                     /cmd_vel  (geometry_msgs/Twist)                    │
└────────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─ SAFETY OVERRIDE (parallel, can veto anything above) ──────────────────┐
│  OAK-D onboard VPU runs YOLO (no Pi CPU cost) ──► /detections          │
│  watchdog node: person within X m ──► suppress /cmd_vel ──► STOP       │
│  (this node must work even if Nav2 crashes)                            │
└────────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─ ACTUATION ────────────────────────────────────────────────────────────┐
│  /cmd_vel ──► rover_bridge (or cmd_vel translator)                     │
│            ──► TCP :8765 ──► rover_agent.py ──► UART ──► ESP32         │
│            ──► motors                                                    │
│  encoders ──► back up to the odometry integrator (loop closes)         │
└────────────────────────────────────────────────────────────────────────┘
```

# 2. Autonomous driving (unknown terrain) — Mapless reactive navigation

Note: An EKF is a sensor-fusion algorithm: it continually estimates the rover’s best current position, heading, and velocity by combining measurements that are individually imperfect.

```text
Wheel encoders ──► distance travelled
                  but wheel slip / calibration drift
                                      │
IMU gyroscope ───► rotation rate ─────┤
                  but bias accumulates│
                                      ▼
GPS / RTK GPS ───► global position ─► EKF ──► best estimated pose
                  but noisy / slower          + uncertainty
                                      │
Visual odometry / RTAB-Map ───────────┘
                  but can lose tracking
```

```text
┌─ LOCALIZATION (no saved map → the map frame is invented) ────────────┐
│                                                                      │
│  LOCAL EKF:  wheel odom + IMU (+ visual odom)                        │
│              world_frame = odom ──► TF: odom → base_link             │
│              (smooth, continuous, but drifts over time)              │
│                                                                      │
│  navsat_transform_node: GPS fix + IMU compass heading                │
│              ──► GPS converted into your world frame                 │
│                                                                      │
│  GLOBAL EKF: same inputs + GPS ──► TF: map → odom                    │
│              "map" origin = wherever the rover booted;               │
│              GPS pins it to Earth, correcting drift                  │
│              (this dual-EKF pattern is the canonical outdoor         │
│               ROS2 setup)                                            │
└──────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─ NAV2 — MAPLESS MODE ────────────────────────────────────────────────┐
│  goal: GPS waypoint ──► navsat ──► pose in map frame                 │
│                                                                      │
│  global costmap: ROLLING window, sized to fit successive             │
│     waypoint pairs                                                   │
│     ├─ static layer:   ABSENT — nothing is known ahead               │
│     └─ obstacle layer: LIVE depth → laserscan/pointcloud only        │
│                                                                      │
│  local costmap: rolling, obstacle + inflation layers, all live       │
│                          │                                           │
│                          ▼                                           │
│  planner: best-effort global path across seen-so-far space           │
│  controller: reactive local trajectory ──► /cmd_vel                  │
│                                                                      │
│  ⚠ planner is GREEDY here: it can't know what's beyond sensor        │
│    range, so U-shaped traps / dead ends force recovery behaviors     │
└──────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
        (YOLO person detection → veto /cmd_vel → ESP32)       
```

# 3. Autonomous driving (unknown terrain) + mapping

```text
First visit:   GPS waypoints (coarse) + live avoidance + RTAB-Map MAPPING
               ──► rover gets there reactively, map gets built as a byproduct
               ──► ~/.ros/rtabmap.db now exists

Second visit:  switch RTAB-Map to localization mode, add static layer
               ──► full Flow 2: global paths on the real map,
                   GPS demoted to sanity-check / recovery
```

## What you trade, honestly

| | Mapless reactive (B/C) | With saved map (Flow 2) |
|---|---|---|
| Obstacle avoidance | Fully live, works day one | Live + static context |
| Global routing | Greedy, can dead-end | Knows the whole site |
| Global reference | GPS (±1–3 m consumer-grade drift) | Visual localization vs map (cm–dm) |
| Compass/IMU quality | Critical — navsat needs a real compass for global heading | Helpful, not critical |
| GPS dropout tolerance | Poor — it's your only global fix | Good — map + EKF carry you |
