# Autonomous Farm Rover — Project Summary \& Roadmap

## The Vision

Build an autonomous farm rover that, given only a GPS path, can follow it while avoiding obstacles, using a SLAM-generated point-cloud map ("nuvem de pontos") to navigate, and detecting/avoiding humans for safety. Target stack: ROS2 + MAVLink/ArduPilot, running in WSL2 on the Windows development PC and eventually on a Pixhawk-based rover, with the Raspberry Pi as the rover hardware agent and the Jetson held in reserve for heavier perception workloads.

## Your Role \& Constraints

You're the AI engineer responsible for the autonomy software (perception, planning, MAVLink/ROS2 integration) — not the hardware/electronics side. You're a software engineer/data scientist with no prior robotics-specific experience, currently without direct rover access, building up skills from first principles.

## Hardware You Own

| Item | Status |
| :-- | :-- |
| OAK-D Lite depth camera | Owned, runs YOLO onboard (Myriad X VPU) |
| Standard RGB camera | Owned |
| NVIDIA Jetson Nano Super 8GB | Owned — older Maxwell GPU, kept in reserve for future GPU-heavy work |
| Reachy Mini | Owned, separate project |
| Elegoo car v4 | Owned, potential low-stakes real-world test rig |
| Pixhawk + ArduPilot | Existing rover reference platform (long-term target) |
| **Waveshare UGV Rover PT PI5 AI Kit** (Amazon.nl, B0D2L27JDT) | **Ordered** — 6-wheel/4WD, all-metal, dual-controller (Pi 5 + ESP32), pan-tilt module, Raspberry Pi 5 4GB included |

## What Ships With the UGV Rover Kit

Camera, mounting accessories, TF card, cooling fan, Raspberry Pi 5 (4GB), pan-tilt module. **Confirmed NOT included:** 18650 batteries (compartment/holder is present, cells are not).

**Important caveat:** This is the standard AI Kit, not Waveshare's separate "ROS2 Kit" SKU — the SD card ships with Waveshare's Python/Flask/OpenCV control app, not ROS2. Keep the working factory control app as the teleoperation baseline while adding autonomy incrementally. The Pi remains a non-ROS2 hardware agent; ROS2 runs in WSL2 on the Windows PC and communicates with the Pi over the network.

## Batteries — Upgraded and validated

The original battery setup caused full-stack brownouts and BMS trips during
turns. A 30A battery upgrade has now been installed and field-tested: the
rover again drives and rotates normally with the full Raspberry Pi, camera, and
audio stack connected. The previous power incident remains useful as a
regression diagnosis, but it is no longer the active autonomy constraint.

For maintenance, continue using matched, reputable 3S-compatible cells with
adequate discharge headroom. No top-style restriction was found for the holder
(flat top is compatible). Good validated cell families include:

- Samsung INR18650-35E (3500mAh, 8A) — confirmed good fit
- LG INR18650-MJ1 (3500mAh, 10A) — confirmed equally good, slightly more discharge headroom

Buy from EU sources: Distrelec, Conrad, Kiwi Electronics, or verified Amazon.nl/de listings. Avoid unbranded cells.

## Compute Board — Decided

**Raspberry Pi 5 4GB** (already included in the kit) chosen over Jetson Orin Nano 4GB for this phase, since OAK-D Lite already offloads YOLO inference to its own onboard VPU. Existing Jetson Nano Super 8GB held in reserve for when GPU-heavy multi-model workloads (SLAM + detection + mapping simultaneously) become the bottleneck.

## Known Hardware Issues \& Fixes

**OAK-D Lite overheating:** Can hit 105°C shutdown within 10-15 min under sustained load. Fix: small 5V fan for airflow (tap the rover's spare fan port), reduce FPS/resolution, disable unused video encoder, avoid sealed enclosures.

**OAK-D Lite power draw (Y-adapter):** Pi 5 + OAK-D Lite under combined load can cause under-voltage crashes. No-electronics fix: keep the Y-adapter, data leg into Pi 5 blue USB3 port, power leg into an independent USB power bank strapped to the chassis.

## SLAM \& Mapping — Core to the Navigation Loop

SLAM is not optional add-on — it's the mechanism that generates the point-cloud map your rover needs to localize and navigate within, directly fulfilling the "nuvem de pontos" requirement from the original vision.

**How it fits together:**

1. **Mapping phase (one-time or periodic):** Drive the rover manually or teleoperated through the farm area. RTAB-Map consumes the OAK-D Lite's RGB + depth stream (via `depthai-ros`) and builds a 3D point-cloud map in real time while simultaneously tracking the rover's pose within it (SLAM = Simultaneous Localization And Mapping).
2. **Map storage:** RTAB-Map saves this as a persistent map database (a `.db` file) that can be reloaded on future runs — you don't need to re-map the farm every single time.
3. **Navigation phase:** Once a map exists, Nav2 uses it as the reference frame — your GPS waypoints get converted into goals within this map, and Nav2's costmap (fed by the live depth/pointcloud data) handles real-time obstacle avoidance against both the static map and any new objects RTAB-Map/depth sensing detects that weren't there during the original mapping run.
4. **Localization on subsequent runs:** RTAB-Map's localization mode (not full SLAM) matches live camera data against the saved map to figure out exactly where the rover is right now, which Nav2 needs before it can plan a path.

**Updated software stack table:**


| Task | Runs on | Tool/Package |
| :-- | :-- | :-- |
| YOLO detection | OAK-D Lite's own VPU | Existing trained model, `.blob` format |
| Depth/pointcloud publishing | OAK-D Lite + Pi 5 | `depthai-ros` |
| **SLAM (map building)** | Windows PC, inside WSL2 | **RTAB-Map (mapping mode)** |
| **Localization (map reuse)** | Windows PC, inside WSL2 | **RTAB-Map (localization mode)** |
| 2D obstacle layer | Windows PC, inside WSL2 | `pointcloud_to_laserscan` |
| Path planning + local avoidance | Windows PC, inside WSL2 | Nav2 (Planner + Controller Server), using RTAB-Map's output as its map source |
| Sensor fusion | Windows PC, inside WSL2 | `robot_localization` (EKF), fusing GPS + IMU + RTAB-Map pose |
| MAVLink bridge | Windows PC, inside WSL2 | MAVROS |
| Human-safety override | Windows PC, inside WSL2 | Custom ROS2 node (subscribes to detections, triggers stop) |

**Why RTAB-Map specifically:** It's purpose-built for exactly this RGB-D camera + real-time 3D mapping + localization combination, and integrates natively with Nav2 as a map source — meaning you don't need a separate mapping tool and a separate navigation tool, they're designed to hand off to each other directly.

## The Phased Plan (Software-First, De-risked)

**Current phase:** Validate the upgraded rover manually, recalibrate wheel odometry, and produce a first synchronized OAK-D RGB-D mapping run before changing the working control stack.

**Next week:** Drive simulated rover via pymavlink/MAVLink, bring up ROS2 topics from ArduPilot's DDS bridge, add simulated camera + basic detection node, start bridging real OAK-D Lite into ROS2 via `depthai-ros`.

**Following month:** Merge real camera perception with simulated rover control, bring up RTAB-Map for SLAM/point-cloud mapping using the real OAK-D Lite indoors as a first test, validate that a saved map can be reloaded and used for localization, only then decide on final hardware specs based on actual profiling data.

**On the current UGV Rover:** Keep the factory Flask app for teleoperation, complete odometry calibration with the upgraded battery, and use the Pi-side agent to expose the sensors and motors to ROS2 running in WSL2 on the Windows PC. Then run RTAB-Map mapping around the test area, localization against the saved map, Nav2 navigation, and MAVROS/GPS waypoint integration last.

## Immediate Next Steps

1. Record a short factory-app movement test, including controlled rotational movement, and keep the result as the new hardware baseline.
2. Repeat wheel odometry calibration and replace the provisional values in `pedro_scripts/odometry_calib.yaml` only after the measurements are consistent.
3. Run the synchronized OAK-D RGB-D recorder and build the first offline RTAB-Map database indoors.
4. Validate localization and transforms against that database.
5. Add Nav2 autonomous goals with a reliable emergency stop before outdoor GPS waypoint work.
