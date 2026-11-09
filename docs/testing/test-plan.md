---
id: TP-SCOUT
title: Scout test plan
status: Draft
owner: Argos (draft) · Jaime (approver)
updated: 2026-11-09
---

# TP-SCOUT — test plan
Each test case maps 1:1 to a requirement in the [SRS](../specs/srs.md) (`TC-SCOUT-nnn` ↔ `REQ-SCOUT-nnn`). Results are recorded here with the date, commit and evidence (rosbag/video).

| TC | Method | Pass criterion | Result | Date / commit |
|---|---|---|---|---|
| TC-SCOUT-001 | Robot follows linear 0.3 m/s and angular 1 rad/s commands | per REQ-SCOUT-001 | ⏳ | |
| TC-SCOUT-002 | Unplug teleop: wheels stop < 300 ms | per REQ-SCOUT-002 | ⏳ | |
| TC-SCOUT-003 | `/joint_states` rate ≥ 50 Hz | per REQ-SCOUT-003 | ⏳ | |
| TC-SCOUT-004 | tf tree valid in RViz | per REQ-SCOUT-004 | ⏳ | |
| TC-SCOUT-005 | rate + calibration check | per REQ-SCOUT-005 | ⏳ | |
| TC-SCOUT-006 | heading drift < 5° / 10 m on carpet | per REQ-SCOUT-006 | ⏳ | |
| TC-SCOUT-007 | map of the living area saved | per REQ-SCOUT-007 | ⏳ | |
| TC-SCOUT-008 | 9/10 runs arrive within 10 cm / 10° | per REQ-SCOUT-008 | ⏳ | |
| TC-SCOUT-009 | box test at 3 speeds | per REQ-SCOUT-009 | ⏳ | |
| TC-SCOUT-010 | FPS + mAP on a test set | per REQ-SCOUT-010 | ⏳ | |
| TC-SCOUT-011 | RViz overlay matches the CAD | per REQ-SCOUT-011 | ⏳ | |
| TC-SCOUT-012 | `vcgencmd get_throttled` = 0x0 | per REQ-SCOUT-012 | ⏳ | |
| TC-SCOUT-013 | threshold test | per REQ-SCOUT-013 | ⏳ | |
| TC-SCOUT-014 | patrol of 3 waypoints | per REQ-SCOUT-014 | ⏳ | |
| TC-SCOUT-015 | messages visible in HiveMQ / Supabase | per REQ-SCOUT-015 | ⏳ | |
| TC-SCOUT-016 | unplug/replug the R4 test | per REQ-SCOUT-016 | ⏳ | |
| TC-SCOUT-017 | bag replays in Foxglove | per REQ-SCOUT-017 | ⏳ | |
| TC-SCOUT-018 | scope/log measurement | per REQ-SCOUT-018 | ⏳ | |
| TC-SCOUT-019 | Nav2 goal in sim | per REQ-SCOUT-019 | ⏳ | |
| TC-SCOUT-020 | release checklist | per REQ-SCOUT-020 | ⏳ | |
| TC-SCOUT-021 | SIL + HITL: `cmd_vel` → `M` lines at 20 Hz; silence → `S` < 0.3 s (DD-SCOUT-BRIDGE §8) | per REQ-SCOUT-001/002/024 | 🟡 SIL passes in CI; HITL not recorded | CI, `main` @ 7b2d64e |
| TC-SCOUT-022 | SIL + HITL: `imu/data_raw` ≥ 18 Hz (v1), frame, covariance −1, \|a\| = 9.81 ± 0.3 | per REQ-SCOUT-005 | 🟡 SIL passes in CI; HITL not recorded | CI, `main` @ 7b2d64e |
| TC-SCOUT-023 | `pan/cmd` ±0.5 rad → `V` line + joint state; camera turns left for + | per REQ-SCOUT-021 | 🟡 SIL passes in CI; HITL not recorded | CI, `main` @ 7b2d64e |
| TC-SCOUT-024 | kill/restart fake R4, unplug R4, frozen IMU → diagnostics ERROR < 1 s, recover < 3 s | per REQ-SCOUT-016/022 | 🟡 SIL passes in CI; HITL not recorded | CI, `main` @ 7b2d64e |
| TC-SCOUT-025 | 20 s slow walk across the view: person centred (±10 %) in ≥ 80 % of frames | per REQ-SCOUT-023 | ⏳ | |
