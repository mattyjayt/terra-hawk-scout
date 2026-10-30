---
id: IF-MICROROS
title: ESP32 ↔ Pi micro-ROS contract
status: Draft
owner: Argos (draft) · Jaime (approver)
updated: 2026-10-30
---

# IF-MICROROS — ESP32 ↔ Raspberry Pi
Transport: **USB serial** (115200–921600 baud) to `micro_ros_agent` on the Pi. ESP32 = classic WROOM-32 (micro-ROS supported).

| Direction | Entity | Type | Rate | Notes |
|---|---|---|---|---|
| ESP32 → Pi | `/wheel_ticks` | std_msgs/Int32MultiArray [left, right] (or JointState) | 50 Hz | cumulative counts, signed |
| ESP32 → Pi | `/imu/data_raw` | sensor_msgs/Imu | 100 Hz | MPU-6050 over I²C; frame `imu_link` |
| ESP32 → Pi | `/range/front` | sensor_msgs/Range | 20 Hz | HC-SR04 |
| ESP32 → Pi | `/battery` | sensor_msgs/BatteryState | 1 Hz | voltage divider on ADC1 |
| Pi → ESP32 | `/wheel_cmd` | std_msgs/Float32MultiArray [left, right] rad/s | 50 Hz | from the ros2_control hardware interface |

**Open design choice (DD-SCOUT-HWIF):** the ros2_control hardware interface either talks micro-ROS topics (above) or a plain serial protocol. To be decided in the design doc with pros/cons.

## Behaviour
- **Watchdog:** no `/wheel_cmd` for 300 ms → motors off (REQ-SCOUT-002).
- **Agent loss:** the ESP32 reconnects automatically (state machine: WAITING_AGENT → AVAILABLE → CONNECTED → DISCONNECTED).
- Time sync with the agent at connect.
