---
id: DD-SCOUT-HW
title: Scout hardware: layout, power, wiring, pinout
status: Draft
owner: Argos (draft) · Jaime (approver)
updated: 2026-10-30
---

# DD-SCOUT-HW — hardware design

## 1. Context
Converts the Elegoo Smart Robot Car V4.0 into a ROS 2 robot. Requirements: REQ-SCOUT-001…005, 009, 011–013. Fit check model: [`cad/scout-layout`](../../cad/scout-layout/README.md).

## 2. Goals / non-goals
**Goals:** stable power for the Pi 5; a 360° clear LiDAR view; motors driven by the ESP32; reuse the Elegoo plates. **Non-goals:** printed parts, docking.

## 3. Layout (from the CAD fit check, v1)
| Deck | Parts | Notes |
|---|---|---|
| Lower (39.7 mm clear) | 4 TT/encoder motors at the corners, battery (flat), ESP32 + TB6612, 5 V/5 A buck, MPU-6050 at the centre of rotation | no collisions with a ~20 mm battery; a 40 mm pack hits the top plate |
| Upper | Pi 5 centre-rear, LiDAR on a centre mast (scan plane 131 mm above the floor), Camera 3 front, payload bay rear | LiDAR clears all parts by 5.5 mm |
Estimated mass ~794 g (empty). Centre of mass on the centre of rotation; a full bay moves it 20 mm rearward.

## 4. Power design (estimates, to be measured)
| Load | Rail | Typical | Peak |
|---|---|---|---|
| Pi 5 + Camera 3 | 5 V | 6 W | 12 W |
| LiDAR (LD19) | 5 V | 1.5 W | 2 W |
| ESP32 + MPU-6050 + HC-SR04 | 5 V → 3.3 V | 0.7 W | 1 W |
| 4 × TT motors | motor rail | 4 W | 15 W (stall) |
| **Total** | | **~12 W** | **~30 W** |
Pack: 3S Li-ion 11.1 V nominal, 3000 mAh ≈ 33 Wh → **~2 h** runtime at typical load (80 % usable).

```
3S pack ── fuse 5 A ── switch ──┬── buck 5 V/5 A ── Pi 5 (USB-C PD or GPIO 5 V) ── LiDAR, Camera
                                │                      └── ESP32 (5 V pin) → 3.3 V sensors
                                └── TB6612 VM ── motors   ⚠ see below
```
⚠ **Open issue:** TT motors are rated 3–6 V and the TB6612 VM max is 13.5 V. A 3S pack is 12.6 V when full. Options: (a) cap the PWM duty at ~45 % in firmware, (b) a second buck for the motor rail (6–7.4 V), (c) the Elegoo 2S box for the motors only. **Lean: (b)**, which gives predictable speed as the battery drains. Decide before S1.

## 5. ESP32 pinout (draft, avoids strapping pins and ADC2)
| Function | GPIO |
|---|---|
| TB6612 PWMA / AIN1 / AIN2 | 25 / 26 / 27 |
| TB6612 PWMB / BIN1 / BIN2 | 33 / 18 / 14 |
| TB6612 STBY | 13 |
| Encoder L A/B | 34 / 35 (input-only) |
| Encoder R A/B | 36 / 39 (input-only) |
| I²C SDA / SCL (MPU-6050) | 21 / 22 |
| HC-SR04 TRIG / ECHO | 4 / 16 (ECHO 5 V → divider to 3.3 V) |
| Battery sense (ADC1, divider) | 32 |
GPIO 34–39 have no internal pull-ups, so the encoders need external pull-ups (or open-collector-safe hall sensors).

## 6. Alternatives considered
- Keep the Elegoo UNO + shield → rejected: no micro-ROS, weak I/O (ADR-0002).
- Pi GPIO for the motors directly → rejected: no real-time encoder counting.

## 7. Risks and open questions
1. Battery dimensions (decides the lower deck) — **measure**.
2. Motor rail option (§4).
3. LiDAR choice: LD19 vs RPLIDAR C1 (C1 is 6 mm taller).
4. Exact buck model and its footprint.

## 8. Verification
TC-SCOUT-012 (no undervoltage), TC-SCOUT-002 (watchdog), TC-SCOUT-003/005 (sensor rates).

## Detailed design v2 (2 Nov 2026)
The detailed mechanical, electrical and physics design (CAD, fit checks, MuJoCo sim, final review and purchase list) lives in [`cad/scout-v2/`](../../cad/scout-v2/README.md). Where it differs from this outline, v2 wins: RPLIDAR C1 (not LD19), Hiwonder 1:42 encoder motors, Cytron MDD3A on the pack with an 8.4 V PWM cap (no motor buck), Pololu D36V50F5 for the Pi, LiDAR and ESP32 on Pi UARTs.
