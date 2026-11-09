# ADR-0002 — ESP32 + micro-ROS as the real-time I/O controller (superseded)

> **Superseded by [ADR-0006](0006-scout-pi-r4-only.md).** Kept as history. The Scout has no ESP32 and no micro-ROS.

- **Status:** Superseded by [ADR-0006](0006-scout-pi-r4-only.md) (2026-11-03). Was: Accepted (2026-10-30).
- **Context:** The Elegoo UNO can't run micro-ROS; the Pi GPIO isn't real-time for encoder counting.
- **Decision:** A classic ESP32 handles PWM, encoders, IMU and ultrasonic and talks to ROS 2 over USB serial.
- **Consequences:** Never implemented. The UNO R4 WiFi over USB serial replaced it (ADR-0005, ADR-0006).
