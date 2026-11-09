---
id: DD-SCOUT-HW
title: Scout hardware: layout, power, wiring, pinout
status: Draft
owner: Argos (draft) · Jaime (approver)
updated: 2026-11-09
---

# DD-SCOUT-HW — hardware design

## 1. Context
Converts the Elegoo Smart Robot Car V4.0 into a ROS 2 robot. Requirements: REQ-SCOUT-001…005, 009, 011–013. Fit check model: [`cad/scout-layout`](../../cad/scout-layout/README.md).

## 2. Goals / non-goals
**Goals:** stable power for the Pi 5; a 360° clear LiDAR view; motors driven by the UNO R4 + Elegoo shield (ADR-0006); reuse the Elegoo plates. **Non-goals:** printed parts, docking.

## 3. Layout (from the CAD fit check, v1)
| Deck | Parts | Notes |
|---|---|---|
| Lower (39.7 mm clear) | 4 TT motors (no encoders) at the corners, battery (flat), UNO R4 + Elegoo shield (TB6612), 5 V/5 A buck, MPU-6050 at the centre of rotation | no collisions with a ~20 mm battery; a 40 mm pack hits the top plate |
| Upper | Pi 5 centre-rear, LiDAR on a centre mast (scan plane 131 mm above the floor), Camera 3 front, payload bay rear | LiDAR clears all parts by 5.5 mm |
Estimated mass ~794 g (empty). Centre of mass on the centre of rotation; a full bay moves it 20 mm rearward.

## 4. Power design (estimates, to be measured)
| Load | Rail | Typical | Peak |
|---|---|---|---|
| Pi 5 + Camera 3 | 5 V | 6 W | 12 W |
| LiDAR (RPLIDAR C1) | 5 V | 1.5 W | 2 W |
| UNO R4 + MPU-6050 + SG90 | 5 V (USB from the Pi) | 0.7 W | 1.5 W |
| 4 × TT motors | motor rail | 4 W | 15 W (stall) |
| **Total** | | **~12 W** | **~30 W** |
Pack: 3S Li-ion 11.1 V nominal, 3000 mAh ≈ 33 Wh → **~2 h** runtime at typical load (80 % usable).

```
3S pack ── fuse 5 A ── switch ──┬── buck 5 V/5 A ── Pi 5 (USB-C PD or GPIO 5 V) ── LiDAR, Camera
                                │                      └── USB → UNO R4 (logic + sensors)
                                └── TB6612 VM ── motors   ⚠ see below
```
⚠ **Open issue:** TT motors are rated 3–6 V and the TB6612 VM max is 13.5 V. A 3S pack is 12.6 V when full. Options: (a) cap the PWM duty at ~45 % in firmware, (b) a second buck for the motor rail (6–7.4 V), (c) the Elegoo 2S box for the motors only. **Today:** the Elegoo 2S box powers the shield and motors. **For the 3S pack** the v2 design (below) chose a PWM cap at 8.4 V instead of a motor buck.

## 5. UNO R4 pinout (Elegoo V4 shield, as wired)
| Function | Pin |
|---|---|
| Right motors: PWMA / AIN1 (forward = LOW) | D5 / D8 |
| Left motors: PWMB / BIN1 (forward = HIGH) | D6 / D7 |
| TB6612 STBY | D3 |
| Servo (pan) | D10 |
| I²C SDA / SCL (MPU-6050, 0x68) | SDA / SCL header |
| Battery sense (10 k / 1.5 k divider) | A3 |
Source: `firmware/r4_base` and [serial-r4-v1](../specs/interfaces/serial-r4-v1.md). Encoder pins are open (ADR-0006: map free interrupt pins before buying encoder motors).

## 6. Alternatives considered
- Keep the Elegoo UNO (ATmega328P) + shield → replaced by the UNO R4 WiFi under the same shield (ADR-0005).
- ESP32 + micro-ROS → dropped (ADR-0002 superseded by ADR-0006).
- Pi GPIO for the motors directly → rejected: Linux timing is not real-time (motor watchdog; encoder counting if encoders are ever added).

## 7. Risks and open questions
1. Battery dimensions: the 3S pack measures 70 × 50 × 15 mm, so it fits between the decks.
2. Motor rail option (§4).
3. LiDAR choice: decided, RPLIDAR C1 (v2 design).
4. Pi supply: Pololu D36V50F5 (5 V / 5 A) in the v2 design.

## 8. Verification
TC-SCOUT-012 (no undervoltage), TC-SCOUT-002 (watchdog), TC-SCOUT-003/005 (sensor rates).

## Detailed design v2 (2 Nov 2026)
The detailed mechanical, electrical and physics design (CAD, fit checks, MuJoCo sim, final review and purchase list) lives in [`cad/scout-v2/`](../../cad/scout-v2/README.md). It is a study for the next hardware revision; nothing in it is built yet. Where it differs from this outline, v2 is the target: RPLIDAR C1, Hiwonder 1:42 encoder motors (deferred, ADR-0003), Cytron MDD3A on the pack with an 8.4 V PWM cap (no motor buck), Pololu D36V50F5 for the Pi, LiDAR on a Pi UART. v2 places an ESP32 on Pi UART2; since ADR-0006 the UNO R4 + shield takes that slot, on USB.
