---
id: DD-SCOUT-FW
title: R4 base firmware (UNO R4 WiFi + Elegoo shield)
status: Not started
owner: Jaime (author) · Argos (review)
updated: 2026-11-03
---
# DD-SCOUT-FW — R4 base firmware

**Learning-critical:** Jaime writes this design and the code (issue #16). Argos reviews.

## Context and requirements
`firmware/r4_base` (v1, 2 Nov) is one working sketch implementing [serial-r4-v1](../specs/interfaces/serial-r4-v1.md). Decisions: [ADR-0005](../adr/0005-r4-usb-serial-base.md), [ADR-0006](../adr/0006-scout-pi-r4-only.md). The ROS side is fixed by [IF-ROS-TOPICS](../specs/interfaces/ros-topics.md) and DD-SCOUT-BRIDGE; a firmware change may only change the serial protocol, never the ROS topics.

## Goals / non-goals
**Goals (to write):** modules (`Motors`, `Imu`, `ServoCtl`, `Battery`, `Link`, `Watchdog`); standard libraries where they fit; protocol v2; reproducible builds.
**Non-goals:** logic on the R4 (kinematics, fusion, control loops above the motor layer).

## Starting points (Argos's notes, to accept or reject)
- Libraries: Arduino `Servo` (servo); `Adafruit_MPU6050` (raw IMU in SI units; no on-chip fusion needed because fusion moves to the EKF). Keep a small own `Motors` class, because the Elegoo shield uses one direction pin per channel and generic TB6612 libraries expect two.
- Pin library versions in a `sketch.yaml` build profile (`arduino-cli compile --profile`), and add a compile check in CI.
- Protocol v2 ideas: raw gyro only (no yaw integration on the R4), IMU at 50–100 Hz, sequence number + checksum per line, version handshake.
- Encoder readiness (ADR-0006): list the free interrupt pins under the shield.

## Design
_TBD (Jaime)_

## Interfaces
_TBD: serial-r4-v2 spec_

## Alternatives considered
_TBD_ (e.g. micro-ROS on the R4, binary framing with COBS)

## Risks
_TBD_

## Test plan
The bridge's software-in-the-loop and HITL tests (DD-SCOUT-BRIDGE) are the regression suite for any firmware change.

## Open questions
_TBD_
