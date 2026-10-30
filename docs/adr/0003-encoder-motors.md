# ADR-0003 — Encoder motors for odometry
- **Status:** Accepted (2026-10-30)
- **Context:** TT motors have no encoders; slip on carpet tiles makes open-loop odometry useless for SLAM/Nav2.
- **Decision:** Buy TT-size hall-encoder gear motors (~€30); fuse them with the IMU in an EKF.
- **Consequences:** recorded in the related design docs; revisit if the constraint changes.
