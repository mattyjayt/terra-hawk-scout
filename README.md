# TerraHawk Scout

> A small autonomous ground robot that maps a room, localises itself and drives to targets, built from an Elegoo Smart Robot Car V4.0. It's the mobile "scout and messenger" of the **TerraHawk** home-farm robotics system.

**Status:** 🟡 Phase D (documentation sprint). No code is written before its design doc is approved → [docs/README.md](docs/README.md)

| | |
|---|---|
| Compute | Raspberry Pi 5 (8 GB) · Ubuntu 24.04 · ROS 2 Jazzy |
| Base controller | Arduino UNO R4 WiFi + Elegoo shield over USB serial: motor PWM, servo, MPU-6050, battery, watchdog ([ADR-0006](docs/adr/0006-scout-pi-r4-only.md)) |
| Sensors | Pi Camera Module 3 · 2D LiDAR (LD19 / RPLIDAR C1, planned) · MPU-6050 · HC-SR04 |
| Software | ROS 2 base bridge (Python + C++) · robot_localization (EKF) · slam_toolbox · Nav2 · YOLO perception (Python) · Foxglove |

## Architecture
The target ROS 2 graph is an interactive diagram: [docs/diagrams/scout-ros2-graph/scout-ros2-graph.html](docs/diagrams/scout-ros2-graph/scout-ros2-graph.html) (download the file and open it in a browser).

## Milestones
| ID | Milestone | Status |
|---|---|---|
| D | Documentation sprint: PRD, spec, interfaces, design docs | 🟡 |
| S1 | Teardown and rewire (Pi 5 + R4 + power) | 🟡 |
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
firmware/    r4_base (UNO R4 WiFi, serial-r4-v1) · r4_bench (learning sketches)
cad/         build123d models + GLB exports (scout-layout fit check)
```

## License
Apache-2.0
