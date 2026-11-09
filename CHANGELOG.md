# Changelog
All notable changes to this project are documented here ([Keep a Changelog](https://keepachangelog.com/en/1.1.0/), [SemVer](https://semver.org/)).

## [Unreleased]
### Added
- Documentation skeleton (`docs/`), Apache-2.0 license, issue and PR templates (2026-10-30).
- R4 base firmware `r4_base` and the serial-r4-v1 protocol; Scout is Pi 5 + UNO R4 only (ADR-0006).
- ROS 2 contract IF-ROS-TOPICS v1.0 and design DD-SCOUT-BRIDGE.
- `scout_base_py`: base bridge (drive with command timeout, IMU, battery, pan servo, services, diagnostics) with unit tests.
- `scout_bringup`: `base.launch.py`, Foxglove layout, `launch_testing` contract suite against `tools/fake_r4.py`.
- `scout_description`: URDF/xacro and TF tree.
- `scout_perception`: person detection (YOLO + ByteTrack) and pan tracking; `camera_ros` pinned in `tools/scout.repos` with a Pi 5 patch.
- CI: `colcon build` + `colcon test` in `ros:jazzy` and an R4 firmware compile check.
