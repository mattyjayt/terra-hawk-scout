---
id: IF-SERIAL-R4-V1
status: implemented (firmware/r4_base, 2026-11-02)
---
# Serial protocol v1: Pi ↔ Arduino UNO R4 base controller

**Link:** USB CDC, 115200 baud, ASCII lines ending in `\n` (a `\r` is ignored). Pi side: `/dev/ttyACM0` (plan: udev rule → `/dev/scout_base`).

## Commands (Pi → R4)
| Line | Meaning |
|---|---|
| `M <left> <right>` | drive; PWM −255…255 per side, sign = direction. **Must repeat within 300 ms** (watchdog). No ack. |
| `M <left> <right> <ms>` | drive for `<ms>` (≤ 5000), then stop. Acked `A M ok`. |
| `S` | stop now |
| `Z` | yaw = 0 |
| `C` | recalibrate the gyro bias (motors stop, 2 s, keep still) |
| `V <angle>` | servo (pin 10) to 0…180°, 90 = ahead |

## Replies (R4 → Pi)
| Line | Meaning |
|---|---|
| `I <t_ms> <ax> <ay> <az> <gx> <gy> <gz> <yaw> <servo> <batt>` | 20 Hz. Accel g, gyro °/s (bias removed, chip axes), yaw ° (left +, integrated on the R4), servo °, battery V |
| `A <cmd> ok` / `A err <reason>` / `A boot ...` | acknowledgements and boot messages |
| `W stop` | the watchdog (or a timed drive) stopped the motors |

## Hardware (Elegoo V4 shield)
Right side PWMA 5 / AIN1 8 (forward = LOW); left side PWMB 6 / BIN1 7 (forward = HIGH); TB6612 STBY 3; servo 10; battery A3 (10 k / 1.5 k divider, ×1.08 Elegoo correction); MPU-6050 I²C 0x68 (±2 g, ±250 °/s, `YAW_SIGN = −1`).

## Bridge node duties (Pi, to build)
- `/cmd_vel` → differential-drive kinematics → `M l r` at ≥ 10 Hz; send `S` on shutdown.
- `I` lines → `sensor_msgs/Imu` in SI units (rad/s, m/s²), `frame_id: imu_link`, covariances; chip mounting goes in the URDF, not in code.
- battery → `sensor_msgs/BatteryState`; servo → topic (and a joint if a sensor rides on it).

## Planned v2
IMU at 50–100 Hz, a checksum per line, raw gyro only (yaw integration moves to the EKF).
