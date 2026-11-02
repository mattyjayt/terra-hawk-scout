# Scout v2 — final physics verdict (round 4, sparring review)

**Date:** 2026-11-02 · **Basis:** CAD v2-r3 (`out/robot_params.json`: 0.998 kg, 1.298 kg with cup; C1, Hiwonder 1:42 at the 8.4 V cap, MDD3A on the pack, D36V50F5), sim r4 (`sim/results.json`, `docs/sim-round4.md`) and analysis r4 (`analysis/results.json`).
**Verdict:** the design **fits, carries the load and is power-consistent on paper**. Three things are still unproven and only Jaime's measurements can close them:
- floor μ against the real threshold (M3, M4);
- pack energy and BMS (M1);
- the motor/bracket fit (M5).

Buy the electronics now. Buy the motors after M5.

## 1. Requirements and derived targets

| Item | Target | Result (evidence) | Status |
|---|---|---|---|
| REQ-001 linear | 0.3 m/s | 0.300 m/s on carpet and hard floor, with and without payload; 9.5–10 % of stall torque, 0.25 A/motor (sim a) | **PASS** |
| REQ-001 angular | 1 rad/s | gyro loop 0.999 rad/s at 22–24 % of stall (SF ≈ 4); open loop only 0.59–0.70 (sim b) | **PASS, closed-loop only** (requirement wording) |
| REQ-002 | stop ≤ 300 ms on lost /cmd_vel | brake LOW/LOW 0.127 s hard / 0.119 s carpet, 14 mm (sim c) → watchdog timeout ≤ 170 ms | **PASS** (firmware must short-brake) |
| REQ-009 | stop for obstacle < 15 cm | stop budget 82 mm (0.3 m/s, 1.5 m/s², 175 ms latency) + C1 ±30 mm → 37 mm margin; C1 in spec at 212 mm (a6, a8) | **PASS** |
| REQ-012 | Pi 5.0–5.25 V, ≤ 5 A, 30 min | D36V50F5 ~8 A capable; bus ≥ 7.4 V during a 4-motor stall vs 5.9 V needed; worst header 4.77 V (±4 % unit tolerance) (a5) | **CONDITIONAL**: M7 (≥ 5.0 V at 3 A) and M1 (BMS trip) |
| REQ-011 | URDF frames = CAD | `check_urdf` OK (both URDFs, xacro variants); laser / camera 15° down / IMU roll π: 0 mm error | **PASS** |
| REQ-006 | < 5°/10 m heading | wheel distance + gyro yaw 2.8°/10 m; wheel-only 6–9°/10 m (sim e) | **PASS with gyro yaw** (ZUPT + scale cal) |
| 10 mm threshold | climb with 300 g | climbs at 0.05/0.15/0.3 m/s with payload and empty (μ 0.8). Needs μ ≥ 0.4/0.5 (sim, payload/empty) or 0.48/0.62 (quasi-static square edge). Torque 0.056 N·m vs 0.118–0.165 available | **CONDITIONAL** on M3/M4 |
| Runtime | ≥ 30 min worst, aim 60 typ | typical 49–68 min, worst 31–43 min for a 12–16.5 Wh pack; needs ≥ 14.0 Wh (a5) | **CONDITIONAL** on M1 |
| Stability | no tip / lift with 300 g | front lift 5.8, rear lift 6.1 m/s² vs 0.6 required / 1.5 cap (SF 9.7 / 4.1); tip back 31°, side 43°. Full-duty start and hard PI stop *do* lift a wheel in sim | **PASS with firmware ramps** |
| LiDAR margin | ≥ 5 mm, blind sectors documented | 6.4 mm over the cup keep-out (22.4 mm over robot parts); no self-occlusion; cup grazed only at ≥ 5.4° dip; plant > rim blinds 42° at the rear | **PASS** (payload envelope ≤ Ø75 × 80 incl. plant) |
| Camera floor | ≤ 150 mm ahead | 104.7 mm, lower FOV edge clear (CAD r3) | **PASS** |
| Payload retention | cup stays put | free-cup sim inside the fence: ≤ 1.4 mm slide, < 0.3° tilt at μ 0.3, including e-stop and full-duty start (sim f); without fence 5.4 mm/event | **PASS** (keep the fence) |

## 2. Round-2 findings: closed vs open

**Closed (17):**
- #3 FIT0450 dropped.
- #4 sim/quasi-static gap: the r1 failure was the fallback params; sim 0.056 vs QS 0.048 N·m.
- #5 MDD3A placed.
- #6 motor buck deleted.
- #8 UARTs, no USB.
- #9 by requirement change (gyro loop).
- #10 cup at −50 mm, 49 % front.
- #13 fence (sim f).
- #14 C1.
- #16 URDF regenerated and verified.
- #17 DC rocker chosen.
- #18 IMU at −24 mm, roll π.
- #20 no-payload variant valid.
- #22 tower–cooler gap 37.6 mm.
- #25 brake 0.13 s.
- #26 moot.
- #27 stability.

**Still open:**

| # | What | Closed by |
|---|---|---|
| 1 | BMS trip | M1 |
| 2 | Threshold traction (μ) | M3, M4 |
| 7 | REQ-012 wording | M7 |
| 11 | Plant height, as a requirement | — |
| 12 | Pitch scan-gating | firmware |
| 15 | YOLO at 320 px | software |
| 19 | Pack energy | M1 |
| 21 | Battery 2.2 mm side gap | M5 |
| 23 | Encoder at 3.3 V | M7 |
| 24 | Encoder cpr | M7 |

**New:**
- **N1:** the Hiwonder at 8.4 V lifts wheels on full-duty steps. Firmware |a| ≤ 1.5 m/s² ramps are mandatory; e-stop = LOW/LOW, never reverse torque.
- **N2:** the Hiwonder listing has no reference voltage. The threshold sweep shows this doesn't matter for climbing; it only changes top speed and current.

## 3. Final purchase list (EU listings, Nov 2026; A = typical price, not looked up)

**Buy now: ≈ €203**

| Item | Qty | ~EUR | Link / note |
|---|---|---|---|
| SLAMTEC RPLIDAR C1 | 1 | 81 | https://www.generationrobots.com/en/404200-360-rplidar-c1-slamtec.html |
| Cytron MDD3A dual driver | 1 | 13 | https://opencircuit.shop/product/3amp-4v-16v-dc-motor-driver-2-channels |
| Pololu D36V50F5 (5 V 5.5 A) | 1 | 35 | https://www.berrybase.de/en/pololu-d36v50f5-5v-5.5a-step-down-voltage-regulator |
| SPST rocker rated **10 A at 12 V DC** (PRO ELEC PELR0230, CPC/Farnell SW02920; its cut-out is round → drill it instead of the 13 × 19 slot, CAD's documented alternative) | 1 | 2 | https://cpc.farnell.com/c/electronic-electrical-components/switches-relays-solenoids/switches-accessories/rocker-switches/10a-rocker-switches |
| Inline ATM mini-blade fuse holder (18 AWG) + 7.5 A fuses ×5 | 1 | 5 A | any automotive supplier |
| AMASS XT30U pairs ×5; JST-XH 2.54 5-pin + Dupont housing kit | 1 | 8 A | AMASS XT30U |
| 470 µF/25 V low-ESR (EEU-FR1E471) + 100 µF/25 V | 1+1 | 2 A | Panasonic FR |
| M3 brass M-F/F-F standoff kit 6–20 mm (fence 4 × M3 × 20 M-F, tower 15 mm) | 1 | 12 A | e.g. https://bc-robotics.com/shop/m3-hardware-kit/ |
| M3 brass M-F standoff **40 mm** (LiDAR tower) | 4 (pack of 10) | 6 A | RS PRO 222-452 family |
| M3 nylon standoff/spacer kit (MDD3A 5 mm, IMU 10 mm, 3 mm motor spacers) | 1 | 10 A | nylon M3 kit |
| M2.5 brass standoff kit (Pi 6 mm M-F, C1 M2.5 × 5 screws) | 1 | 10 A | M2.5 brass kit |
| Electrokit ProtoPCB 80 × 120 FR4 (LiDAR board) | 1 | 4 A | https://www.electrokit.com/en/experimentkort-1-hal-80x120mm-genomplaterade-hal |
| Silicone wire 18/20/22 AWG + F-F Dupont jumpers | 1 | 15 A | — |

**Buy after measuring: ≈ €43 (+ HK shipping)**

| Item | Qty | ~EUR | Gate |
|---|---|---|---|
| Hiwonder TT Hall-encoder 1:42, plastic shaft (SKU 21100132) | 4 + 1 spare | 33 ($6.99 ea) | **M5** fit (then M7 on arrival). https://www.hiwonder.com/products/tt-motor-plastic |
| Camera Module 3-compatible tilt mount (not the Pimoroni/V2 acrylic one, which doesn't fit CM3) | 1 | 5 A | Q-holders: fixes drill holes #19/#20 |
| HC-SR04 acrylic L-bracket (Berrybase HC-SR04-HOLDER) | 1 | 2 | Q-holders; first check whether the Elegoo kit holder can be reused |
| 3S LiPo buzzer alarm | 1 | 3 A | **only if M1 finds no BMS** |

**Don't buy:**
- LDROBOT LD19
- DFRobot FIT0450
- Yahboom TT encoder motor
- any motor buck (D36V28F-class)
- XL4015 or 5 A UBEC for the Pi
- level shifters (Hiwonder encoders)
- Pololu #713 TB6612
- LiDAR USB cable/adapter
- INA226 (decide later)

**Total ≈ €246 + shipping** (€203 now + €43 after measuring).

## 4. Measure first (free, parts on hand), ranked

1. **M5 – motor bracket.** Measure the Elegoo bracket hole pattern, the gearbox inner-face gap (54.4 mm assumed; it leaves 2.2 mm per side for the battery) and the free length behind each motor. This gates the motor order.
2. **M1 – battery.** Weigh the pack, read the cell and BMS markings, check for a balance lead. This settles runtime (needs ≥ 14 Wh), the BMS trip vs 7.7 A bursts, the fuse size and whether the alarm is needed.
3. **M4 – friction (tilt test).** Robot on a board on carpet tile and on hard floor; μ = tan(slip angle). The threshold needs ≥ 0.48–0.62 (quasi-static) or ≥ 0.4–0.5 (sim).
4. **M3 – real threshold.** Photograph and measure the profile (square or bevelled); drive the stock car over it with 300 g at crawl and at ~0.15 m/s. This decides the "threshold mode" (approach at ≥ 0.15 m/s).
5. **M6 – weigh parts.** Plates, wheels, Pi + cooler; tyre width and wheel–plate gap (1.57 mm). Corrects the CoM; the empty robot is 67 % front-loaded.
6. **M7 – on arrival.** D36V50F5 output ≥ 5.0 V at 3 A (REQ-012); encoder counts over 10 wheel turns (≥ 820 cpr); Hiwonder encoder at 3.3 V.
7. **M2 – stock TT stall torque** at 6 V and 7.4 V. This now only calibrates the motor model: the sim climb needs 0.056 N·m, below every candidate's torque.
8. **M8 – Elegoo 2S box.** Dimensions and mass, only if the two-pack option comes back.
