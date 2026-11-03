---
id: IF-ROS-TOPICS
title: Scout — ROS 2 topics, frames and QoS
status: Draft
owner: Argos (draft) · Jaime (approver)
updated: 2026-10-30
---

# IF-ROS-TOPICS — topics, frames, QoS
Namespace: `/scout` when multi-robot (Phase 4); single-robot runs use the root namespace. Frames follow REP-103/105.

## Frames
`map → odom → base_link → {base_laser, camera_link → camera_optical_frame, imu_link, left_wheel, right_wheel}`. `base_link` sits at the chassis centre on the ground plane (z = 0 at floor contact), +X forward.

## Topics
| Topic | Type | Publisher → subscriber | Rate | QoS |
|---|---|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist (TwistStamped when Jazzy diff_drive requires it) | teleop / Nav2 → diff_drive_controller | 10–20 Hz | reliable, depth 10 |
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster | 50 Hz | best effort |
| `/odom` (`diff_drive_controller/odom`) | nav_msgs/Odometry | diff_drive_controller → EKF | 50 Hz | best effort |
| `/imu/data_raw` | sensor_msgs/Imu | base_bridge (R4) → EKF | 100 Hz | sensor data |
| `/odometry/filtered` | nav_msgs/Odometry | EKF → slam_toolbox, Nav2 | 50 Hz | reliable |
| `/scan` | sensor_msgs/LaserScan | LiDAR driver → slam_toolbox, Nav2 | 10 Hz | sensor data |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox → Nav2, RViz | on change | transient local |
| `/camera/image_raw`, `/camera/camera_info` | sensor_msgs/Image, CameraInfo | camera_ros → perception | 15–30 Hz | sensor data |
| `/detections` | vision_msgs/Detection2DArray | perception → Nav2 / mission | ≥ 10 Hz | best effort |
| `/range/front` | sensor_msgs/Range | base_bridge (R4) → safety | 20 Hz | sensor data |
| `/battery` | sensor_msgs/BatteryState | base_bridge (R4) → mission | 1 Hz | reliable |
| `/mission/state` | std_msgs/String (custom msg later) | mission → bridge, UI | on change | transient local |

## Actions / services
`/navigate_to_pose`, `/follow_waypoints` (Nav2) · `/slam_toolbox/save_map` · mission: `/scout/run_mission` (custom action, defined in DD-SCOUT-MISSION).
