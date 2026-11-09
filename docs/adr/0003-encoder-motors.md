# ADR-0003 — Encoder motors for odometry
- **Status:** Deferred (2026-11-09). Accepted 2026-10-30 but not implemented: the robot runs on the stock TT motors without encoders, and odometry will come from LiDAR scan matching fused with the gyro ([ADR-0005](0005-r4-usb-serial-base.md)). If encoder motors are added later, the UNO R4 reads them ([ADR-0006](0006-scout-pi-r4-only.md)).
- **Context:** TT motors have no encoders; slip on carpet tiles makes open-loop odometry useless for SLAM/Nav2.
- **Decision:** Buy TT-size hall-encoder gear motors (~€30); fuse them with the IMU in an EKF.
- **Consequences:** recorded in the related design docs; revisit if the constraint changes.
