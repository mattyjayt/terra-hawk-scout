---
id: IF-ROS-TOPICS
title: Scout — ROS 2 interface contract (topics, services, frames, QoS)
version: 1.0
status: Approved (Jaime, 2026-11-03) for phase P1–P2; P3+ rows are planned
owner: Argos (draft) · Jaime (approver)
updated: 2026-11-03
---

# IF-ROS-TOPICS — Scout ROS 2 interface contract

## 1. Purpose and scope
This document is the **contract** between the Scout's ROS 2 nodes. A node that claims to implement an interface must publish or accept exactly the names, types, units, frames, rates and QoS given here. Implementations may change freely behind the contract. Examples: the Python and C++ base bridges, or serial protocol v1 → v2 on the R4.

| Phase | Scope | Status |
|---|---|---|
| **P1 — Base** | base bridge (R4 over USB): drive, IMU, battery, pan servo, diagnostics; robot model / TF | **Approved** |
| **P2 — Perception** | camera, person detection, pan tracking | **Approved** |
| P3 — Localisation | odometry, EKF | planned |
| P4 — Mapping / navigation | LiDAR, SLAM, Nav2, mission | planned |

Related: [SRS-SCOUT](../srs.md) (requirements) · [DD-SCOUT-BRIDGE](../../design/DD-SCOUT-BRIDGE.md) (base bridge design) · [serial-r4-v1](serial-r4-v1.md) (R4 wire protocol, behind the bridge) · [ADR-0006](../../adr/0006-scout-pi-r4-only.md).

## 2. Conventions
- **Units and axes (REP-103):** SI units: metres, seconds, radians, m/s, rad/s, m/s², volts. Right-handed frames: x forward, y left, z up. Positive rotation about z is counter-clockwise, i.e. a **left turn is positive**.
- **Frames (REP-105):** `odom → base_link → sensor frames`. `map` is added in P4.
- **Time:** every stamped message uses the Pi's ROS clock at the moment the data was **received** (`node.now()`). The R4's `t_ms` is used only for diagnostics (jitter, dropped lines).
- **Names are relative** in code (no leading `/`), so a namespace (`/scout`, Phase 4 multi-robot) can be pushed onto the whole graph. This document writes them from the root.
- **Topic types in this document** use the ROS 2 form `package/msg/Type`.

## 3. QoS profiles
| Profile | Reliability | Durability | History | Use |
|---|---|---|---|---|
| `SENSOR` | best effort | volatile | keep last 5 | high-rate sensor streams (`rclpy.qos.qos_profile_sensor_data`, `rclcpp::SensorDataQoS()`) |
| `CMD` | reliable | volatile | keep last 10 | commands |
| `STATE` | reliable | volatile | keep last 10 | low-rate state and diagnostics (ROS default) |
| `LATCHED` | reliable | transient local | keep last 1 | `robot_description`, maps |

A best-effort subscriber can read a reliable publisher, but not the other way round. Foxglove (`foxglove_bridge`) subscribes to both.

## 4. Node graph (P1–P2)
```
teleop_twist_keyboard / Foxglove Teleop ──cmd_vel──▶ base_bridge ◀──USB serial (serial-r4-v1)──▶ UNO R4
                                         pan/cmd ──▶ base_bridge ──▶ imu/data_raw, battery, joint_states, diagnostics, base/debug/yaw
robot_state_publisher (URDF) + joint_states ──▶ tf, tf_static, robot_description
camera_ros ──camera/image_raw──▶ person_detector ──detections──▶ pan_tracker ──pan/cmd──▶ base_bridge
                                                              imu/data_raw, joint_states ──▶ pan_tracker
foxglove_bridge ◀── all of the above (ws://ghorman:8765)
```
Diagram (archify): [`docs/diagrams/scout-p1-graph/scout-p1-graph.html`](../../diagrams/scout-p1-graph/scout-p1-graph.html) (download and open in a browser).

## 5. Topics

### 5.1 Summary table
| # | Topic | Type | Publisher → subscribers | Rate | QoS | Phase |
|---|---|---|---|---|---|---|
| T1 | `/cmd_vel` | `geometry_msgs/msg/Twist` | teleop, Foxglove, (Nav2 P4) → base_bridge | 10–20 Hz (sender) | CMD | P1 |
| T2 | `/pan/cmd` | `std_msgs/msg/Float64` | pan_tracker, teleop tools → base_bridge | ≤ 20 Hz | CMD | P1 |
| T3 | `/imu/data_raw` | `sensor_msgs/msg/Imu` | base_bridge → EKF (P3), pan_tracker, Foxglove | 20 Hz (v1) → 100 Hz (v2) | SENSOR | P1 |
| T4 | `/battery` | `sensor_msgs/msg/BatteryState` | base_bridge → mission, UI | 1 Hz | STATE | P1 |
| T5 | `/joint_states` | `sensor_msgs/msg/JointState` | base_bridge → robot_state_publisher, pan_tracker | 20 Hz | STATE | P1 |
| T6 | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | base_bridge (and later others) → Foxglove, ops | 1 Hz | STATE | P1 |
| T7 | `/base/debug/yaw` | `std_msgs/msg/Float64` | base_bridge → Foxglove | 20 Hz | SENSOR | P1 (removed with serial v2) |
| T8 | `/robot_description` | `std_msgs/msg/String` | robot_state_publisher | once | LATCHED | P1 |
| T9 | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | robot_state_publisher (EKF adds `odom→base_link` in P3) | with joint_states / once | standard tf QoS | P1 |
| T10 | `/camera/image_raw` | `sensor_msgs/msg/Image` | camera_ros → person_detector | 15–30 Hz | SENSOR | P2 |
| T11 | `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | camera_ros (image_transport) → Foxglove | as T10 | SENSOR | P2 |
| T12 | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | camera_ros → perception | as T10 | SENSOR | P2 |
| T13 | `/detections` | `vision_msgs/msg/Detection2DArray` | person_detector → pan_tracker, Foxglove | every processed frame (target ≥ 10 Hz) | SENSOR | P2 |
| T14 | `/detections/image/compressed` | `sensor_msgs/msg/CompressedImage` | person_detector (debug, optional) → Foxglove | ≤ 5 Hz | SENSOR | P2 |
| — | `/odom`, `/odometry/filtered`, `/scan`, `/map`, `/range/front`, `/mission/state` | see §9 | — | — | — | P3–P4 |

### 5.2 Message shapes (field by field)
Fields not listed are left at their default (zero / empty) and must be ignored by subscribers.

**T1 `/cmd_vel` — `geometry_msgs/Twist`** (subscribed by base_bridge)
| Field | Unit | Meaning / rule |
|---|---|---|
| `linear.x` | m/s | forward speed, + = forward. Clamped to ± `max_linear` |
| `angular.z` | rad/s | yaw rate, + = left (CCW). Clamped to ± `max_angular` |
| all other fields | — | ignored (a differential drive can't move sideways, roll or pitch) |
**Timeout rule:** if no message arrives for `cmd_timeout` (0.25 s), base_bridge stops the motors (sends `S` once). The R4's own 300 ms watchdog is the second layer.
*Note: plain `Twist` for P1 (simple, works with `teleop_twist_keyboard` and the Foxglove Teleop panel). Move to `TwistStamped` when Nav2 arrives; that change is a contract version bump (§10).*

**T2 `/pan/cmd` — `std_msgs/Float64`** (subscribed by base_bridge)
| Field | Unit | Meaning / rule |
|---|---|---|
| `data` | rad | pan angle of `pan_joint`. 0 = straight ahead, + = left (REP-103). Clamped to [`pan_min`, `pan_max`] = [−π/2, +π/2] |
The bridge sends a servo command only when the angle changes by ≥ 1° and at most at 20 Hz (hobby servos jitter when flooded).

**T3 `/imu/data_raw` — `sensor_msgs/Imu`** (published by base_bridge)
| Field | Unit | Value |
|---|---|---|
| `header.stamp` | — | receive time of the `I` line |
| `header.frame_id` | — | `imu_link` |
| `orientation` | — | (0, 0, 0, 1). Not estimated here |
| `orientation_covariance[0]` | — | **−1** (REP-145: "orientation not provided"); others 0 |
| `angular_velocity.{x,y,z}` | rad/s | chip gyro × π/180, **in chip axes** (mounting goes in the URDF, not in code) |
| `angular_velocity_covariance` | (rad/s)² | diagonal `gyro_var` (parameter; measured at rest, see DD-SCOUT-BRIDGE §6.4), off-diagonal 0 |
| `linear_acceleration.{x,y,z}` | m/s² | chip accel × 9.80665, chip axes. At rest the vector magnitude is ≈ 9.81 (specific force, pointing **up**) |
| `linear_acceleration_covariance` | (m/s²)² | diagonal `accel_var` (parameter), off-diagonal 0 |
**Stale-data rule:** if the IMU values are bit-for-bit identical for ≥ `imu_stale_count` (10) consecutive lines, the bridge keeps publishing but reports **ERROR** on `/diagnostics` ("IMU frozen"). A real MEMS sensor always has noise in the last digit. *(Observed on 3 Nov: firmware v1 repeats the last sample when the I²C read fails.)*

**T4 `/battery` — `sensor_msgs/BatteryState`**
| Field | Value |
|---|---|
| `header.frame_id` | `base_link` |
| `voltage` | V, mean of the last 1 s of `I` lines |
| `present` | `voltage > battery_present_v` (2.0 V; below that the pack is switched off and USB powers the logic) |
| `power_supply_technology` | `POWER_SUPPLY_TECHNOLOGY_LION` (2) |
| `power_supply_status` / `health` | `UNKNOWN` (0) |
| `current`, `charge`, `capacity`, `design_capacity`, `percentage`, `temperature` | `NaN` (not measured; `percentage` gets a voltage→charge table later) |
| `cell_voltage` | `cell_count` entries of `voltage / cell_count` (estimate) |
| `location` / `serial_number` | `"base"` / `""` |

**T5 `/joint_states` — `sensor_msgs/JointState`**
| Field | Value |
|---|---|
| `name` | `["pan_joint"]` |
| `position` | rad, from the servo angle the R4 reports: `pan_sign × (servo_deg − 90) × π/180` |
| `velocity`, `effort` | empty |
The SG90 has no position feedback: this is the **commanded** angle echoed by the R4, not a measurement. Wheels are not listed until encoders exist (ADR-0003).

**T6 `/diagnostics` — `diagnostic_msgs/DiagnosticArray`**, one status named `base_bridge: R4 link`, `hardware_id` = USB serial number:
| `level` | When |
|---|---|
| OK (0) | port open, `I` lines at ≥ 90 % of the expected rate, IMU not frozen |
| WARN (1) | rate < 90 %, parse errors in the last second, battery not present, or a watchdog stop in the last second |
| ERROR (2) | port closed / reconnecting, no `I` line for 0.5 s, or IMU frozen |
`values` (key → value, strings): `port`, `connected`, `line_rate_hz`, `lines_rx`, `parse_errors`, `watchdog_stops`, `imu_frozen`, `last_ack`, `protocol` (`serial-r4-v1`).

**T7 `/base/debug/yaw` — `std_msgs/Float64`**: yaw in rad integrated **on the R4** (left +). For comparison plots only; nothing may depend on it. It is removed in serial v2, when yaw integration moves to the EKF.

**T13 `/detections` — `vision_msgs/Detection2DArray`** (vision_msgs 4.x, Jazzy)
| Field | Value |
|---|---|
| `header` | copied from the source image (`stamp` = image stamp, `frame_id` = `camera_optical_frame`) |
| `detections[i].bbox.center.position.{x,y}` | px, box centre in the **original image** resolution (not the network input size) |
| `detections[i].bbox.center.theta` | 0 |
| `detections[i].bbox.size_x / size_y` | px, box width / height |
| `detections[i].results[0].hypothesis.class_id` | `"person"` (COCO class 0; other classes are filtered out in P2) |
| `detections[i].results[0].hypothesis.score` | confidence 0…1, only ≥ `conf_threshold` (0.5) |
| `detections[i].id` | tracker ID as a string (e.g. `"3"`) when tracking is on, else `""` |
An empty `detections` array is published for frames with no person, so subscribers can tell "nobody there" from "detector dead".

**T10–T12 camera** (from `camera_ros`): `frame_id` = `camera_optical_frame` (set with the `frame_id` parameter), 640 × 480 for P2. The `encoding` is whatever `camera_ros` publishes for the chosen libcamera format. Read it from the message (`ros2 topic echo --once --field encoding`) rather than assuming.

## 6. Services
| # | Service | Type | Server | Effect |
|---|---|---|---|---|
| S1 | `/base/stop` | `std_srvs/srv/Trigger` | base_bridge | sends `S`; `success` = R4 replied `A S ok` within 0.5 s |
| S2 | `/imu/calibrate` | `std_srvs/srv/Trigger` | base_bridge | sends `C` (motors stop, keep still 2 s); `success` = `A C ok` within 3 s |
| S3 | `/imu/zero_yaw` | `std_srvs/srv/Trigger` | base_bridge | sends `Z` (resets T7) |

## 7. Frames (TF)
```
base_link ─┬─ imu_link            (fixed; carries the chip's mounting rotation)
           └─ pan_base_link       (fixed; servo body)
                └─ pan_link       (revolute pan_joint about +z, limits ±π/2)
                     └─ camera_link (fixed)
                          └─ camera_optical_frame (fixed, rpy = −π/2, 0, −π/2: z forward, x right, y down)
```
- `base_link`: chassis centre on the floor (z = 0 at floor contact), x forward.
- All offsets come from measurement or `cad/` and live only in `scout_description` (URDF/xacro). No node hard-codes them.
- P3 adds `odom → base_link` (EKF); P4 adds `map → odom` (slam_toolbox) and `base_laser`.

## 8. Parameters (base_bridge, both implementations)
| Name | Type | Default | Unit / note |
|---|---|---|---|
| `port` | string | `/dev/scout_base` | udev symlink; `/tmp/fake_r4` for software tests |
| `baud` | int | 115200 | ignored by USB CDC but kept for completeness |
| `track_width` | double | **measure** (Lab 1) | m, centre of left wheels to centre of right wheels |
| `max_wheel_speed` | double | **measure** | m/s of one side at PWM 255 on the floor |
| `pwm_min` | int | **measure** | smallest PWM that starts a wheel turning (deadband) |
| `max_linear` / `max_angular` | double | 0.3 / 1.5 | m/s / rad/s clamps |
| `cmd_timeout` | double | 0.25 | s |
| `send_rate` | double | 20.0 | Hz of `M` lines while a command is active |
| `gyro_var` / `accel_var` | double | 1.0e-5 / 2.0e-3 | placeholders; replace with measured values |
| `imu_stale_count` | int | 10 | lines |
| `pan_sign` | int | **measure** (+1 or −1) | so that + `pan/cmd` turns the camera left |
| `pan_min` / `pan_max` | double | −1.5708 / 1.5708 | rad |
| `cell_count` | int | 2 | Elegoo 2S pack on the shield today |
| `battery_present_v` | double | 2.0 | V |
| `frame_id_imu` / `frame_id_base` | string | `imu_link` / `base_link` | |

## 9. Planned interfaces (P3–P4, not yet binding)
| Topic | Type | Publisher → subscriber | Rate | QoS |
|---|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | LiDAR scan matching (`rf2o_laser_odometry`) → EKF (fused with the gyro) | 20–50 Hz | SENSOR |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | robot_localization EKF → slam_toolbox, Nav2 | 30–50 Hz | STATE |
| `/scan` | `sensor_msgs/msg/LaserScan` | `sllidar_ros2` (RPLIDAR C1) → slam_toolbox, Nav2 | 10 Hz | SENSOR |
| `/map` | `nav_msgs/msg/OccupancyGrid` | slam_toolbox → Nav2, Foxglove | on change | LATCHED |
| `/range/front` | `sensor_msgs/msg/Range` | base_bridge (HC-SR04, after serial v2) → safety | 20 Hz | SENSOR |
| `/mission/state` | `std_msgs/msg/String` (custom later) | mission → UI | on change | LATCHED |
Actions (P4): `/navigate_to_pose`, `/follow_waypoints` (Nav2) · `/slam_toolbox/save_map` · `/run_mission` (DD-SCOUT-MISSION).

## 10. Versioning and change rules
- This is **v1.0**. Adding a topic, field use or service is a minor bump (v1.1). Renaming, retyping or changing units or frames is a major bump (v2.0) and needs an ADR.
- Every implementation of a P1 interface must pass the contract tests (DD-SCOUT-BRIDGE §8) before merging. The **same** test suite runs against the Python and the C++ bridge.

## 11. Traceability
| Interface | Requirement | Test |
|---|---|---|
| T1 + timeout | REQ-SCOUT-001, 002 | TC-SCOUT-001, 002, 021 |
| T3 | REQ-SCOUT-005 | TC-SCOUT-005, 022 |
| T4 | REQ-SCOUT-013 | TC-SCOUT-013 |
| T2, T5 | REQ-SCOUT-021 | TC-SCOUT-023 |
| T6 | REQ-SCOUT-022 | TC-SCOUT-024 |
| T8, T9 | REQ-SCOUT-011 | TC-SCOUT-011 |
| T10–T13 | REQ-SCOUT-010 | TC-SCOUT-010 |
| T13 → T2 loop | REQ-SCOUT-023 | TC-SCOUT-025 |
