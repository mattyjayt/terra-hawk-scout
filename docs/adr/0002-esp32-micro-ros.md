# ADR-0002 — ESP32 + micro-ROS as the real-time I/O controller
- **Status:** Accepted (2026-10-30)
- **Context:** The Elegoo UNO can't run micro-ROS; the Pi GPIO isn't real-time for encoder counting.
- **Decision:** A classic ESP32 handles PWM, encoders, IMU and ultrasonic and talks to ROS 2 over USB serial.
- **Consequences:** recorded in the related design docs; revisit if the constraint changes.
