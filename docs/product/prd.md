---
id: PRD-SCOUT
title: Scout — product requirements
status: Draft
owner: Argos (draft) · Jaime (approver)
updated: 2026-10-30
---

# PRD-SCOUT — Scout product requirements

## Problem
The TerraHawk home farm needs a mobile robot that moves resources and seedlings between the seedling chamber (desk) and the grow frame (wall alcove), and that can inspect both. It also has to be **evidence** of robotics and perception skill for job applications (Lely Juno / Astronaut-style roles), so every capability must be demonstrable, measured and documented.

## Users
- **Jaime** (builder/operator): drives, maps, sends missions, reads telemetry.
- **Reviewers and interviewers**: watch the videos, read the docs, run the code.
- **The TerraHawk system** (later): the Farm Manager assigns delivery tasks.

## Goals
| ID | Goal | Success metric |
|---|---|---|
| G-SCOUT-1 | Drive reliably under ROS 2 control | Teleop at ≥ 0.3 m/s with < 150 ms command latency; no brown-outs in a 30 min session |
| G-SCOUT-2 | Know where it is | Fused odometry heading drift < 5° per 10 m on carpet tiles (vs < 20° for wheels alone) |
| G-SCOUT-3 | Map the apartment | A complete 2D occupancy map of the living area at 5 cm resolution, made by slam_toolbox |
| G-SCOUT-4 | Navigate autonomously | 9 of 10 goal runs from desk to grow frame arrive within 10 cm / 10° without a collision |
| G-SCOUT-5 | See | A perception node on the Pi detects the target classes at ≥ 10 FPS and publishes `/detections` |
| G-SCOUT-6 | Show it | A 30–60 s video, README and portfolio section for every milestone |

## Non-goals (for now)
- Arm handover and seedling transport (Phase 3, after the arm kits arrive)
- Outdoor driving, docking/charging, multi-robot coordination
- 3D printing (none available; reuse Elegoo parts)

## Constraints
- One Pi 5 (8 GB); the Pi moves to the tower arm after the Scout demo.
- Elegoo plates 132.5 × 230.8 mm with a **39.7 mm** gap between the decks (measured).
- Budget: LiDAR + encoder motors + 5 V/5 A buck (~€120–150).
- About 40 h/week; learning-critical code is written by Jaime.

## Milestones
S1 teardown/rewire → S2 teleop → S3 URDF + perception → S4 odometry + EKF → S5 SLAM → S6 Nav2 → S7 digital twin. Dates: the project plan in the TerraHawk vault, mirrored as GitHub milestones.

## Risks
| ID | Risk | Mitigation |
|---|---|---|
| RISK-01 | Encoder-less TT motors make odometry useless | buy encoder motors (ADR-0003) |
| RISK-02 | Pi brown-outs on battery | dedicated 5 V/5 A buck, power budget in DD-SCOUT-HW |
| RISK-03 | 3S pack (12.6 V) over-volts the TT motors and is near the TB6612 VM limit | PWM duty cap or separate motor supply (DD-SCOUT-HW §4) |
| RISK-04 | A tall battery doesn't fit between the decks | measure it; flat pack or top mount |
| RISK-05 | Wi-Fi DDS discovery is flaky | Pi-local stack; Mac only for visualisation; Zenoh/discovery server fallback |
