# ADR-0005 — Arduino UNO R4 WiFi over USB serial as the interim base controller
- **Status:** Accepted (2026-11-02). Partly supersedes [ADR-0002](0002-esp32-micro-ros.md) for now.
- **Context:** Jaime wants to learn embedded basics on hardware he owns before buying anything. The Elegoo shield (TB6612 motors, MPU-6050, servo header, battery divider) plugs straight onto an UNO R4 WiFi, which replaces the Elegoo UNO. micro-ROS on the R4 is community-supported only and tight on RAM (32 KB).
- **Decision:** The R4 runs `firmware/r4_base`: a thin hardware layer (motors, servo, IMU, battery, 300 ms watchdog) that speaks a plain text line protocol ([serial-r4-v1](../specs/interfaces/serial-r4-v1.md)) over **USB** to the Pi (`/dev/ttyACM0`). A ROS 2 bridge node on the Pi turns it into `/cmd_vel`, `/imu` and `/battery_state`. All logic lives in ROS nodes.
- **Why USB, not pin UART:** the R4 is 5 V logic and the Pi's GPIO is 3.3 V (not 5 V tolerant). USB needs no level shifter and no parts.
- **Consequences:**
  - No wheel encoders yet (stock TT motors): odometry comes from LiDAR scan matching fused with the gyro, until encoder motors are bought ([ADR-0003](0003-encoder-motors.md) stays the target).
  - Only two drive channels (left pair, right pair).
  - The ESP32 + micro-ROS path (ADR-0002) is deferred, not dropped: revisit when encoder motors or the MDD3A arrive.
