# Firmware
| Folder | Board | Status |
|---|---|---|
| `r4_base/` | Arduino UNO R4 WiFi + Elegoo V4 shield | **in use**: motors, servo, MPU-6050, battery, watchdog; protocol [serial-r4-v1](../docs/specs/interfaces/serial-r4-v1.md); decision [ADR-0005](../docs/adr/0005-r4-usb-serial-base.md) |
| `r4_bench/` | same | learning steps: `Wheel_Test` (each side fwd/back), `IMU_Test` (raw I²C, yaw), `Turn90` (gyro-guided 90° turn) |
| `esp32_base/` | classic ESP32 | deferred (ADR-0002): micro-ROS + encoders, after the motor upgrade |

Build/upload: Arduino IDE, board *Arduino UNO R4 WiFi* (core `arduino:renesas_uno`), or
`arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi firmware/r4_base`.
Motors need the battery switched on; USB alone runs the logic.

Bench teleop (Mac or Pi): `python3 tools/serial_teleop.py /dev/ttyACM0` (needs `pyserial`).
Verified 2026-11-02: wheels, IMU and Pi ↔ R4 over USB working; servo/battery lines added the same evening (to verify).
