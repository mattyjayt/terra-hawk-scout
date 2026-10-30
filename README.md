# TerraHawk Scout

> A small autonomous ground robot that maps a room, localises itself and drives to targets, built from an Elegoo Smart Robot Car V4.0. It's the mobile "scout and messenger" of the **TerraHawk** home-farm robotics system.

**Status:** 🟡 Phase D (documentation sprint). No code is written before its design doc is approved → [docs/README.md](docs/README.md)

| | |
|---|---|
| Compute | Raspberry Pi 5 (8 GB) · Ubuntu 24.04 · ROS 2 Jazzy |
| Real-time I/O | ESP32 (WROOM-32) running micro-ROS: motor PWM, wheel encoders, IMU |
| Sensors | Pi Camera Module 3 · 2D LiDAR (LD19 / RPLIDAR C1, planned) · MPU-6050 · HC-SR04 |
| Software | ros2_control · robot_localization (EKF) · slam_toolbox · Nav2 · YOLO perception (C++) |

## Architecture
The target ROS 2 graph is an interactive diagram: [docs/diagrams/scout-ros2-graph/scout-ros2-graph.html](docs/diagrams/scout-ros2-graph/scout-ros2-graph.html) (download the file and open it in a browser).

## Milestones
| ID | Milestone | Status |
|---|---|---|
| D | Documentation sprint: PRD, spec, interfaces, design docs | 🟡 |
| S1 | Teardown and rewire (Pi 5 + ESP32 + power) | ⏳ |
| S2 | Teleop over ROS 2 | ⏳ |
| S3 | Robot model (URDF) and perception node | ⏳ |
| S4 | Wheel and IMU odometry fused with an EKF | ⏳ |
| S5 | LiDAR SLAM map | ⏳ |
| S6 | Autonomous navigation (Nav2) | ⏳ |
| S7 | Digital twin (Isaac Sim) | ⏳ |

## Repository layout
```
docs/        product/ specs/ design/ adr/ testing/ ops/ hardware/ diagrams/ figures/
ros2_ws/src/ scout_description · scout_bringup · scout_hardware · scout_perception · scout_navigation · scout_mission
firmware/    esp32_base (PlatformIO + micro-ROS)
cad/         build123d models + GLB exports (scout-layout fit check)
```

## License
Apache-2.0
