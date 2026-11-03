---
id: SRS-SCOUT
title: Scout — software requirements specification
status: Draft
owner: Argos (draft) · Jaime (approver)
updated: 2026-10-30
---

# SRS-SCOUT — Scout requirements
Priority: **M**ust · **S**hould · **C**ould (MoSCoW). Each requirement traces to goals in [PRD-SCOUT](../product/prd.md) and to a test in the [test plan](../testing/test-plan.md).

| ID | Area | Requirement | Prio | Acceptance | Verified by |
|---|---|---|---|---|---|
| REQ-SCOUT-001 | Drive | The robot shall accept `geometry_msgs/Twist` on `/cmd_vel` and drive as a differential drive base. | M | Robot follows linear 0.3 m/s and angular 1 rad/s commands | TC-SCOUT-001 |
| REQ-SCOUT-002 | Drive | Motor commands shall stop within 300 ms if `/cmd_vel` is not received (watchdog). | M | Unplug teleop: wheels stop < 300 ms | TC-SCOUT-002 |
| REQ-SCOUT-003 | Drive | Wheel encoders shall be read at ≥ 50 Hz and published as joint states. | M | `/joint_states` rate ≥ 50 Hz | TC-SCOUT-003 |
| REQ-SCOUT-004 | Odometry | The base shall publish wheel odometry on `/odom` and the `odom → base_link` transform (REP-105). | M | tf tree valid in RViz | TC-SCOUT-004 |
| REQ-SCOUT-005 | IMU | The MPU-6050 shall be published as `sensor_msgs/Imu` on `/imu/data_raw` at ≥ 100 Hz with covariances set. | M | rate + calibration check | TC-SCOUT-005 |
| REQ-SCOUT-006 | Fusion | An EKF (robot_localization) shall fuse wheel odometry and IMU yaw rate into `/odometry/filtered`. | M | heading drift < 5° / 10 m on carpet | TC-SCOUT-006 |
| REQ-SCOUT-007 | SLAM | The robot shall build a 2D occupancy map with slam_toolbox from `/scan` at 5 cm resolution. | M | map of the living area saved | TC-SCOUT-007 |
| REQ-SCOUT-008 | Nav | The robot shall navigate to a goal pose with Nav2, avoiding static and dynamic obstacles. | M | 9/10 runs arrive within 10 cm / 10° | TC-SCOUT-008 |
| REQ-SCOUT-009 | Safety | An obstacle closer than 15 cm in front (LiDAR or ultrasonic) shall stop forward motion. | M | box test at 3 speeds | TC-SCOUT-009 |
| REQ-SCOUT-010 | Perception | A Python node shall run object detection on Camera Module 3 frames at ≥ 10 FPS on the Pi 5 and publish `vision_msgs/Detection2DArray` on `/detections`. | S | FPS + mAP on a test set | TC-SCOUT-010 |
| REQ-SCOUT-011 | Model | A URDF/xacro shall describe the robot with correct sensor frames (laser, camera, imu). | M | RViz overlay matches the CAD | TC-SCOUT-011 |
| REQ-SCOUT-012 | Power | The Pi 5 shall be supplied 5.0–5.25 V at up to 5 A; no undervoltage events during a 30 min run. | M | `vcgencmd get_throttled` = 0x0 | TC-SCOUT-012 |
| REQ-SCOUT-013 | Power | Battery voltage shall be measured and published; below a threshold the robot shall stop and report. | S | threshold test | TC-SCOUT-013 |
| REQ-SCOUT-014 | Mission | A mission node shall run patrol and go-to-station missions via Nav2 actions. | S | patrol of 3 waypoints | TC-SCOUT-014 |
| REQ-SCOUT-015 | Telemetry | Robot status (pose, battery, mission state) shall be bridged to MQTT `terrahawk/v1/scout-01/...`. | C | messages visible in HiveMQ / Supabase | TC-SCOUT-015 |
| REQ-SCOUT-016 | Ops | The full stack shall start with one launch command and recover from a USB serial drop or R4 reset. | M | unplug/replug the R4 test | TC-SCOUT-016 |
| REQ-SCOUT-017 | Ops | Rosbag recording of core topics shall be available for every test run. | S | bag replays in Foxglove | TC-SCOUT-017 |
| REQ-SCOUT-018 | Perf | Command-to-motion latency shall be < 150 ms end to end. | S | scope/log measurement | TC-SCOUT-018 |
| REQ-SCOUT-019 | Sim | A simulation (Gazebo or Isaac) shall run the same launch with a simulated base. | C | Nav2 goal in sim | TC-SCOUT-019 |
| REQ-SCOUT-020 | Docs | Each milestone shall ship code, photos, a 30–60 s video and a README update. | M | release checklist | TC-SCOUT-020 |

## Open points (resolve before Approved)
- Target detection classes for REQ-SCOUT-010 (e.g. plant pot, cup, person, chair legs?)
- Map size/area for REQ-SCOUT-007 (living room only, or the whole apartment?)
- Battery cut-off threshold (REQ-SCOUT-013) depends on the chosen pack.
