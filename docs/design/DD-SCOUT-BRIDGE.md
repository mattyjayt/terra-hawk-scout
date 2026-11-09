---
id: DD-SCOUT-BRIDGE
title: Base bridge node (Pi ↔ UNO R4), Python and C++
version: 0.9
status: Draft · Python implementation (`scout_base_py`) merged and tested; C++ implementation in progress
owner: Argos (draft) · Jaime (approver, implementer)
updated: 2026-11-09
---

# DD-SCOUT-BRIDGE — base bridge node

## 1. Context and requirements
The UNO R4 under the Elegoo shield is a thin hardware layer: motors, pan servo, MPU-6050, battery and a 300 ms watchdog ([ADR-0005](../adr/0005-r4-usb-serial-base.md), [ADR-0006](../adr/0006-scout-pi-r4-only.md)). It speaks [serial-r4-v1](../specs/interfaces/serial-r4-v1.md) over USB. The **base bridge** is the only ROS 2 node that talks to it. It turns the ROS contract ([IF-ROS-TOPICS](../specs/interfaces/ros-topics.md) §5–§8, phase P1) into serial lines and back.

Requirements served: REQ-SCOUT-001 (drive from `cmd_vel`), 002 (stop on silence), 005 (IMU as `sensor_msgs/Imu`; v1 at 20 Hz, 100 Hz with serial v2), 013 (battery), 016 (recover from a serial drop), 021 (pan servo), 022 (diagnostics), 024 (two implementations, one contract).

## 2. Goals / non-goals
**Goals**
1. Implement IF-ROS-TOPICS P1 exactly, twice: `scout_base_py` (rclpy) and `scout_base_cpp` (rclcpp). Both have the same node name `base_bridge`, the same parameters and the same behaviour.
2. Never leave the motors running on bad input: command timeout, the R4 watchdog behind it, `S` on shutdown.
3. Survive the USB cable being pulled: report it, reconnect, carry on.
4. Make every decision testable without hardware (fake R4).

**Non-goals**
- Closed-loop wheel speed control (no encoders yet, ADR-0003). The drive is **open loop**: the bridge assumes PWM ∝ speed.
- Odometry (`/odom`) and orientation estimation: P3 (EKF).
- `ros2_control`. A `ros2_control` hardware interface (DD-SCOUT-HWIF, deferred) could reuse the same core code later.

## 3. Architecture: functional core, I/O shell
```
                ┌──────────────────────────── base_bridge (one process) ───────────────────────────┐
cmd_vel ──────▶ │ on_cmd_vel: store (v, ω, t)                                                      │
pan/cmd ──────▶ │ on_pan: store angle                                                              │
                │ send timer (20 Hz): timeout? → S once : M l r   ── kinematics.twist_to_pwm() ──┐ │
services ─────▶ │ stop / calibrate / zero_yaw → write + wait for ack (Event, timeout)            │ │
                │                                                       write_lock ──▶ serial port ─┼──▶ R4
                │ reader thread: read bytes → split lines → protocol.parse_line() ─┬─ I → publish ◀─┼── R4
                │                                                                  ├─ A → ack Event │
                │                                                                  └─ W → counter   │
                │ diag timer (1 Hz): rates, stale IMU, link state → /diagnostics                   │
                └──────────────────────────────────────────────────────────────────────────────────┘
```
- **Core** (no ROS, no serial; pure functions, unit-tested):
  - `protocol`: `parse_line(str) → ImuSample | Ack | WatchdogStop | Unknown`; `format_drive(l, r)`, `format_servo(deg)`.
  - `kinematics`: `twist_to_pwm(v, ω, params) → (l, r)`; `pan_rad_to_servo_deg`, `servo_deg_to_pan_rad`.
  - `conversions`: g → m/s², °/s → rad/s, `StaleDetector`, `RateMeter`.
- **Shell**: the node. Parameters, publishers, subscribers, timers, services, the serial port and the reader thread. Kept thin so that most bugs can only live in the core, where the tests are.
- **Why a reader thread** instead of a polling timer: the services wait up to 3 s for an `A … ok` line. In a single-threaded executor a waiting callback would block the timer that reads the port, which deadlocks. A dedicated reader thread keeps reading while a service waits on an `Event` / `std::condition_variable`. Publishing from that thread is allowed in rclpy and rclcpp. Writes come from the timer and the service callbacks, so they share a `write_lock` mutex.

## 4. Link state machine
```
DISCONNECTED ──open ok──▶ SYNCING ──first valid I line──▶ RUNNING
     ▲                       │ 2 s without I line              │ read/write error, or 0.5 s without I line
     └──── close, retry 1 s ─┴─────────────────────────────────┘
```
- On open: flush the input buffer, then send `S` (known state). Measured 3 Nov: opening `/dev/ttyACM0` does **not** reset the R4 WiFi, so don't wait for `A boot`. Still handle `A boot …` lines, which appear after a power cycle.
- In DISCONNECTED/SYNCING: no `M` lines are sent, services return `success=false, message="link down"`, `/diagnostics` = ERROR.

## 5. Drive: `cmd_vel` → `M l r`
### 5.1 Differential-drive kinematics
For linear speed *v* (m/s), yaw rate *ω* (rad/s) and track width *W* (m):

v_L = v − ω·W/2  v_R = v + ω·W/2

A left turn (ω > 0) makes the right side faster. That's the REP-103 sign.

### 5.2 Saturation preserves the turn
If either side exceeds `max_wheel_speed` (v_max), scale **both** by s = max(|v_L|, |v_R|)/v_max. Clipping each side on its own would change the curvature, so the robot would drive a different arc than commanded.

### 5.3 Speed → PWM with a deadband
TT motors don't turn below a minimum PWM (`pwm_min`, measured). For each side:
- |v| < 0.005 m/s → PWM 0
- otherwise PWM = sign(v) · round(pwm_min + (255 − pwm_min) · |v| / v_max), clamped to ±255.

### 5.4 Timing and safety layers
Sequence diagram (archify): [`docs/diagrams/scout-cmd-safety/scout-cmd-safety.html`](../diagrams/scout-cmd-safety/scout-cmd-safety.html). How-it-works figure for §5.1–5.3 (anatomy, interactive, example values): [`docs/figures/twist-to-pwm/twist-to-pwm.html`](../figures/twist-to-pwm/twist-to-pwm.html).

| Layer | Where | Rule |
|---|---|---|
| 1. Command timeout | bridge send timer | no `cmd_vel` for `cmd_timeout` (0.25 s) → send `S` once, then nothing until a new command |
| 2. Hardware watchdog | R4 firmware | no `M` line for 300 ms → motors off, `W stop` |
| 3. Shutdown | node destructor / SIGINT | send `S`, close port |
While a command is fresh, `M l r` is re-sent at `send_rate` (20 Hz) even if unchanged. That keeps the R4 watchdog fed.

### 5.5 Calibration that the maths needs (Lab 1)
- `max_wheel_speed`: drive PWM 255 straight for 2.0 s on the floor and measure the distance → v_max = d / 2.0.
- `pwm_min`: raise PWM in steps of 5 until both sides turn reliably on the floor.
- `track_width`: geometric centre-to-centre first. A 4-wheel skid-steer slips sideways in turns, so the *effective* track width is larger. Calibrate it later by commanding ω = 1 rad/s for 2π s and adjusting W until the robot turns 360°.

## 6. Telemetry: `I` line → messages
`I t_ms ax ay az gx gy gz yaw servo batt` (accel g, gyro °/s bias-removed, yaw °, servo °, battery V)

### 6.1 IMU (`imu/data_raw`)
- accel × 9.80665 → m/s²; gyro × π/180 → rad/s; **chip axes as-is**, `frame_id imu_link`.
- `orientation_covariance[0] = −1`; diagonal covariances from parameters.
- The chip's mounting rotation (e.g. upside down, rotated 90°) is expressed by the `base_link → imu_link` joint in the URDF. Code that "fixes" axes by flipping signs hides the truth from TF and the EKF.

### 6.2 Joint state, debug yaw, battery
- `pan_joint` position = `pan_sign · (servo − 90) · π/180`.
- `base/debug/yaw` = yaw × π/180.
- Battery: running mean over the last 20 lines, published at 1 Hz (IF-ROS-TOPICS T4).

### 6.3 Stale IMU detection
Seen on the real robot on 3 Nov: 120 consecutive `I` lines with identical IMU values. The I²C read fails and firmware v1 re-sends the last sample. The bridge counts consecutive identical (ax…gz) tuples. At ≥ `imu_stale_count` (10) it sets `imu_frozen = true` and raises ERROR on `/diagnostics`. It keeps publishing, because consumers decide; the EKF would be configured to reject it. Firmware v2 should report the failure itself (DD-SCOUT-FW).

### 6.4 Measuring the covariances
Robot still, after `C` (calibrate): record 30 s of `imu/data_raw` (`ros2 bag record`). Then take the variance of each gyro and accel axis, using the largest axis for the diagonal. Put the values in `config/base.yaml`. The defaults (1e-5 (rad/s)², 2e-3 (m/s²)²) are placeholders.

## 7. Pan servo: `pan/cmd` → `V deg`
`deg = round(90 + pan_sign · rad · 180/π)`, clamped 0…180 (and `rad` to [pan_min, pan_max] first). Sent only if it differs from the last sent value by ≥ 1° and at most at 20 Hz.

## 8. Test design
Three levels. The **same** level-2 and level-3 tests run against both implementations (`impl:=py|cpp`).

### 8.1 Level 1 — unit tests (pytest / gtest, no ROS graph, no serial)
| ID | Function | Cases |
|---|---|---|
| U1 | `parse_line` valid `I` | example line from serial-r4-v1 → every field, correct types |
| U2 | `parse_line` invalid | 10 fields instead of 11, a non-number, empty line, `\r` only, truncated line, 200 chars → returns *Unknown/None*, never raises |
| U3 | `parse_line` others | `A S ok`, `A err …`, `A boot ready`, `W stop` |
| U4 | `twist_to_pwm` | (0,0) → (0,0) · forward → equal positive · ω > 0 → right > left · pure rotation → opposite signs · saturation keeps the ratio v_R/v_L · tiny speed → 0 · just above zero → ±pwm_min · never beyond ±255 |
| U5 | servo mapping | 0 rad → 90°, +π/4 with pan_sign ±1, clamp beyond ±π/2, round-trip deg → rad → deg |
| U6 | conversions | 1 g → 9.80665, 180 °/s → π |
| U7 | `StaleDetector` | 9 identical → not stale, 10 → stale, one change → resets |
| U8 | diagnostic level | rate 19 Hz → OK, 15 Hz → WARN, link down → ERROR |
Plus the ament linters (`flake8`, `pep257`, `cpplint`, `uncrustify`/`clang-format`, `xmllint`, `copyright`) via `ament_lint_auto`.

### 8.2 Level 2 — software in the loop (launch_testing + `tools/fake_r4.py`)
The fake R4 creates a pty at `/tmp/fake_r4`, speaks serial-r4-v1 and logs every received line as JSON (`--log`).
| TC | Check | Pass |
|---|---|---|
| TC-SCOUT-021a | publish `cmd_vel` (0.1, 0) at 10 Hz for 1 s | fake log has `M l r` lines at 20 ± 2 Hz with l = r > 0 matching `twist_to_pwm` |
| TC-SCOUT-021b | stop publishing | an `S` line within 0.30 s of the last `cmd_vel`; no `M` after it |
| TC-SCOUT-022 | `imu/data_raw` | ≥ 18 Hz, `frame_id imu_link`, `orientation_covariance[0] == −1`, accel magnitude 9.81 ± 0.3 |
| TC-SCOUT-023 | `pan/cmd` 0.5 rad | fake log gets `V 119` (pan_sign +1) and `joint_states` shows 0.5 ± 0.02 rad |
| TC-SCOUT-024 | kill the fake R4, restart it | `/diagnostics` ERROR within 1 s; OK again within 3 s; services succeed again |
| TC-SCOUT-024b | fake R4 with frozen IMU (`--freeze-imu`) | `imu_frozen = true`, level ERROR |
| — | services | `/base/stop` → `success=true` and `S` in the log |

### 8.3 Level 3 — hardware in the loop (real R4, wheels off the ground)
Same launch test pointed at `/dev/scout_base`, plus manual checks with evidence (video, bag):
| TC | Check | Pass |
|---|---|---|
| TC-SCOUT-001 | teleop forward / left turn | both sides forward / right side faster (watch the wheels) |
| TC-SCOUT-002 | unplug USB while driving | wheels stop < 300 ms (video at 60 fps, count frames) |
| TC-SCOUT-005/022 | still robot | gyro mean < 0.01 rad/s after `C`, accel magnitude 9.81 ± 0.3, values not frozen |
| TC-SCOUT-013 | battery | `/battery.voltage` within ± 0.1 V of a multimeter at the pack |
| TC-SCOUT-023 | `pan/cmd` ±0.5 rad | camera turns **left** for + |
Results go into [TP-SCOUT](../testing/test-plan.md) with the date, commit and evidence path.

## 9. Packages and dependencies
| Package | Build type | Depends on |
|---|---|---|
| `scout_base_py` | ament_python | `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `diagnostic_msgs`, `python3-serial`; test: `python3-pytest`, `ament_flake8`, `ament_pep257`, `ament_copyright` |
| `scout_base_cpp` | ament_cmake | `rclcpp`, same msg/srv packages, `libserial-dev` (system, via rosdep); test: `ament_cmake_gtest`, `ament_lint_auto`, `ament_lint_common` |
| `scout_description` | ament_cmake | `xacro`, `robot_state_publisher`, `urdf` |
| `scout_bringup` | ament_cmake (launch + config only) | `launch_ros`, `foxglove_bridge`; test: `launch_testing`, `launch_testing_ros` |
Both bridges install an executable `base_bridge`. `scout_bringup/launch/base.launch.py impl:=py|cpp` starts one of them with `config/base.yaml`, `robot_state_publisher` and (optionally) `foxglove_bridge`.

## 10. Alternatives considered
| Option | Why not (now) |
|---|---|
| `ros2_control` hardware interface straight away (`diffdrive_arduino` pattern) | A possible end state (DD-SCOUT-HWIF, deferred), but it hides the ROS basics (topics, QoS, timers, threads) behind a framework. Bridge first, then port the core into a hardware interface. |
| micro-ROS on the R4 | Community-supported on the R4, tight on RAM, and it moves ROS onto the microcontroller, against the "thin hardware layer" decision (ADR-0005/0006). |
| Polling timer instead of a reader thread | Deadlocks while a service waits for an ack (§3). Only works with a multi-threaded executor + callback groups, which is more to learn at once. |
| Binary protocol | ASCII stays readable in `miniterm`. A checksum comes with serial v2. |

## 11. Risks
| Risk | Mitigation |
|---|---|
| Open-loop speed differs per side, with battery level and on carpet | accept for P1; LiDAR odometry + EKF in P3; encoder motors only if needed (ADR-0003, deferred) |
| Frozen IMU (seen 3 Nov) | stale detector → diagnostics ERROR; firmware v2 reports I²C failures |
| `/dev/ttyACM0` changes number after a replug | udev symlink `/dev/scout_base` on USB ID 2341:1002 + serial number |
| ModemManager probing the port | the udev rule sets `ENV{ID_MM_DEVICE_IGNORE}="1"` |
| Pi brown-outs on battery look like USB bugs | `vcgencmd get_throttled` in diagnostics later; proper 5 V/5 A supply (REQ-SCOUT-012) |

## 12. Open questions
1. Exact chip mounting (axes and sign of z), from a measurement in Lab 1 → URDF `imu_link` rotation.
2. Servo direction (`pan_sign`) and whether 0…180° on the SG90 really covers ±90° (often ~±80°).
3. Battery on the shield today: the Elegoo 2S box (`cell_count 2`) or the 3S pack later?
