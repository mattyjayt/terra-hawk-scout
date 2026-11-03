# Scout v2 — physics / electrical review, round 2 (sparring partner)

**Author:** physics/electrical engineer (agent) · **Date:** 2026-11-01 · **Status:** for Jaime's review, nothing bought.
**Inputs used:** `checks/fit_check_v2.json` (CAD r1 mass/CoM/inertia — `out/robot_params.json` did **not** exist while
this ran, so every round-2 number uses the fit-check cases), `sim/results.json` (sim r1, v1-fallback params),
`BRIEF.md`. Code: `analysis/a5–a9*.py`, run with `analysis/run_all.py` → `analysis/results.json` (sections
`mass, drivetrain, skid_steer, driver` = round 1; `motor_review, power, stability, odometry, sensors` = round 2).
The unfinished round-1 a5–a8 are kept in `analysis/round1_unfinished/`.
Source tags: **D** datasheet (URL in `results.json → _meta.sources`), **M** measured, **A** assumed (reason given),
**C** computed. Graph values from Pololu plots were read by eye (±5 %).

Plots: `a5_power.png` (budget bars + runtime vs capacity), `a6_stability.png` (lift limits + CoM envelope),
`a7_odometry.png` (encoder quantisation + gyro budget), `a8_sensors.png` (beam spacing + obstacle heights),
`a9_motor_review.png` (threshold vs payload x + datasheet consistency), `a9_torque_speed.png` (torque–speed at the
voltage cap with the CAD-case operating points).

---

## 1. Verdict in five lines
1. **Power: passes.** A Pololu D36V50F5 on the pack covers the Pi 5 with margin (5 V rail 1.8 A typ / 2.8 A peak vs
   ~8 A capability at 12 V in); motor stalls can't brown it out (bus stays ≥ 7.3 V, buck needs ~5.9 V). The unknown is
   the **pack's BMS trip current**. Runtime 43–68 min typical, 25–42 min worst for a realistic 12–16.5 Wh pack.
2. **Threshold climb is the weakest physics.** A square 10 mm step at crawl speed is **traction-limited below
   μ ≈ 0.55–0.59 for every mass case**, whichever motor you buy. Motor torque is the second-order question.
3. **Datasheets can't be trusted to decide the motor.** DFRobot FIT0450's printed stall torque (0.0785 N·m) is 4.5–7×
   below what its own speed/current figures imply. The sim's "TT fails the climb" result rests on that kind of number.
   Two free bench tests settle it (M2, M3).
4. **Spin at 1 rad/s is achievable only with gyro closed-loop yaw control** (sim: 0.45–0.65 rad/s open loop, 1.0 closed).
   Torque SF is 1.1–1.5 for TT-class motors at 6 V and 2.5–3.0 (with payload) for the Hiwonder at 8.4 V.
5. **Buy now:** RPLIDAR C1, Cytron MDD3A, Pololu D36V50F5, fuse/switch/wiring. **Buy after measuring:** encoder motors
   (Hiwonder 1:42 preferred). **Don't buy:** LD19, motor buck, XL4015/UBEC for the Pi, Yahboom TT encoder motors.

---

## 2. Findings, ranked (adversarial review of CAD r1 + sim r1 + my own round 1)

Severity: **B** blocker (stops the purchase or the robot), **M** major (fails a requirement or makes the purchase
wrong), **m** minor. "Evidence" cites `results.json` sections or files.

| # | Sev | Finding | Evidence | Fix |
|---|---|---|---|---|
| 1 | **B** | **Pack BMS / over-current trip unknown.** Peak battery current is 7.7 A (4 motors stalled at the 8.4 V cap + Pi peak) and 11.8 A on a firmware fault. If a small pack's BMS trips at 3–5 A, the whole robot, Pi included, loses power mid-run (SD-card risk). Without a BMS, the firmware is the only over-discharge protection. | `power.currents_A`, `power.brownout` | M1: read the pack markings, check for a balance lead, measure. Firmware stall-current limit (back-EMF model from encoder speed). If there's no BMS, add a 3S alarm. |
| 2 | **B** | **10 mm threshold at crawl speed is traction-limited, not just torque-limited.** Quasi-static square edge: needs μ ≥ 0.55 (C1 + payload) to 0.59 (C1 empty), even with unlimited torque. With μ 0.6 the rear axle needs 0.061 N·m/wheel (C1 + 300 g). Carpet/rubber μ is *assumed* 0.5–0.7. | `motor_review.climb_quasi_static.*.mu_min_both_axles` | Climb at ≥ 0.15 m/s (sim passes at 0.15 and 0.3 with payload); measure μ (M4) and the threshold profile (M3; most door thresholds are bevelled, which changes everything); firmware "threshold mode" (approach 0.15 m/s, full duty allowed for 1 s). |
| 3 | **B** (for FIT0450) | **The CAD motor envelope is wrong for the chosen FIT0450.** The DFRobot drawing shows the encoder as a vertical PCB, 29.86 mm wide and 44.39 mm along the shaft axis, with its connector 16.8 mm *inboard* of the gearbox face (my reading: the shaft stub points down in both views). CAD models a Ø22 × 9 mm disc. Inboard connector + plug reach \|y\| ≈ 2–10 mm at x ≈ ±95–100: this collides with the ESP32 (x 60–115, \|y\| ≤ 14) and crowds the rear fuse/XT30. A 29.86 mm-tall PCB centred on the can probably also dips ~4 mm into the bottom plate. | FIT0450 dimension PDF (`SRC fit0450_dim`), `src/dims.py MOTOR_ENCODER` | Don't pick FIT0450, or import DFRobot's STEP (published) into CAD before deciding. Hiwonder's published envelope (79.95 × 22.4 × 25.79 mm) is compact, but still measure it against the bracket (M5). |
| 4 | **M** | **Motor datasheets are internally inconsistent; the sim's climb failure may be an artefact.** The stall torque implied by Ke·(I_stall−I0)·η (η 0.4–0.65) vs the printed value: TT stock 0.14–0.23 vs 0.0785 N·m; FIT0450 0.35–0.58 vs 0.0785; Hiwonder 0.15–0.25 vs 0.118; Yahboom 0.06–0.10 vs 0.118 (its listing also says "1.2 N·m, 16000 rpm": nonsense). The sim caps torque at the printed stall and still needs > 0.11 N·m, where my quasi-static model needs 0.057–0.066. That ~1.7× gap is unexplained. | `motor_review.datasheet_consistency`, `motor_review.sim_disagreement`, `a9_motor_review.png` | M2: stall-torque test on a stock TT (10 cm lever + kitchen scale: 80 g = 0.0785 N·m) at 6 V and 7.4 V, plus stall current. Feed the results into `sim/motor_profiles.json` and re-run the threshold scenario. |
| 5 | **M** | **TB6612 can't drive 2 motors per channel for climbs and stalls.** Hiwonder at 8.4 V: climb 1.35 A/channel (TB6612 1.2 A continuous), stall 3.36 A (3.2 A peak). FIT0450 at 7.4 V: stall 6.9 A/channel, over even the MDD3A's 5 A peak. Also, the CAD models a Pololu #713 TB6612 carrier, which Jaime doesn't own (he has the TB6612 on the Elegoo UNO shield). | `motor_review.driver_channels` | Buy the MDD3A (3 A continuous / 5 A peak per channel, 4–16 V). Re-layout the lower deck for the MDD3A (≈ 48 × 42 mm, round-1 datasheet reading): it does not fit the 25 mm gap between the battery (x ≤ 35) and the ESP32 (x ≥ 60). |
| 6 | **M** | **CAD r1 motor buck (D36V28F-class) is wrong twice.** It's 2.8 A-family, against 6–7 A motor-side stall and 1.2–2.4 A/channel climbs, and it overlaps `buck_5v` by 63 mm³ (0.3 mm from `motor_bracket_rl`). | `fit_check_v2.json overlaps_unintended`, `power.motor_rail` | Delete the motor buck. MDD3A goes straight on the pack, with the PWM duty capped at V_cap/V_batt (8.4 V for Hiwonder, 7.4 V for FIT0450). |
| 7 | **M** | **REQ-SCOUT-012 (5.0–5.25 V) is not guaranteed by any fixed 5 V buck.** D36V50F5 is 5.0 V ±4 % (tolerance from my round-1 reading; re-verify), and 20 AWG × 0.15 m drops 0.03 V at peak, so the worst case at the header is 4.77 V. The Pi only flags undervoltage below 4.63 V ±5 %. | `power.pi_supply` | Re-word REQ-012 to "≥ 4.9 V at the header under load and `vcgencmd get_throttled` = 0x0 over 30 min", or accept the unit only if it reads ≥ 5.0 V at 3 A (M7). Use pins 2+4 / 6+14, 20 AWG, ≤ 15 cm, keyed. |
| 8 | **M** | **Pi 5 USB budget without PD.** GPIO back-powering can't negotiate USB-PD, so the Pi limits USB to 600 mA total. LiDAR 230–300 mA plus the ESP32 over USB (100–250 mA) is 0.33–0.55 A: marginal at start-up. | `power.pi_supply.usb_current`, RPi power doc | `usb_max_current_enable=1` + EEPROM `PSU_MAX_CURRENT=5000`. Better: put the C1 on a Pi UART (3.3 V TTL, 460800 baud) and the ESP32 on a second UART, so no USB plugs are needed (this also frees the rear face for #12). |
| 9 | **M** | **Spin in place at 1 rad/s.** Traction is fine (L/B = 0.645 < 1), but the wheels must turn 1.7–1.9× the geometric speed (sim: effective track 0.21–0.24 m vs 0.124 m). Torque needed is 0.047–0.056 N·m/wheel. The SF is 1.15–1.4 for stock TT/FIT0450 at their printed torque and 2.5–3.0 for Hiwonder at 8.4 V (payload; 3.2–3.7 empty). Open loop reaches 0.45–0.65 rad/s. | `motor_review.spin_1rad_s`, `sim b_spin` | Requirement change: REQ-SCOUT-001 angular rate is closed-loop on the IMU yaw rate; Nav2 `max_vel_theta` 0.8 rad/s (margin); EKF never uses wheel yaw. Design change only if M2 shows the torque is short. |
| 10 | **M** | **The CAD payload sits at the rear (x = −73.9 mm), giving 60–63 % rear load.** Rear-axle climb torque 0.061 N·m vs 0.049 at the balanced optimum (payload x ≈ −40). The front-axle climb becomes traction-infeasible if the payload moves forward of x ≈ +10 mm. | `motor_review.payload_x_sweep_c1_mu0.6`, `a9_motor_review.png` | Move the cup to x = −50 mm (−16 % climb torque, 47 % front load). The cup then clears the LiDAR plate edge (x = −5) by 7.5 mm, but sits 12 mm from the Pi's rear USB/Ethernet face, so use UART instead of USB (#8). |
| 11 | **M** | **The payload (seedling) shadows the LiDAR.** The scan plane is only 5.2–5.4 mm above the 80 mm cup rim. Any plant taller than the rim blinds a 55° rear sector (cup Ø75 at 81 mm). | `sensors.payload_shadow`, `fit_check lidar_check` | Add a requirement: payload envelope ≤ Ø75 × 80 mm *including the plant*. Otherwise publish a laser angular-bounds filter for the rear 55° and forbid blind reversing. |
| 12 | **M** | **Pitch on the threshold puts the scan into the floor.** 7.2° nose-up at the 15.1 cm scan height hits the floor 1.2 m ahead (phantom walls, map corruption). | `sensors.pitch_floor_hits` | Scan filter: drop scans while \|IMU pitch\| > 1.5°; slam_toolbox `minimum_travel_*` so mapping skips the crossing. |
| 13 | **M** | **Unrestrained cup slides off.** It slides at 2.9–4.9 m/s² (μ 0.3–0.5 assumed). The sim's active e-stop decelerates at 5.9 m/s² and a full-duty start accelerates at 3.6 m/s². | `stability.payload_retention` | Retainer from off-the-shelf parts (4 standoffs as a fence, or a hook-and-loop pad). Firmware \|a\| ≤ 1.5 m/s² except a true e-stop. |
| 14 | **M** | **LD19 has no accuracy spec below 300 mm, but the 15 cm stop zone is 222 mm from the LiDAR** (x = 45 mm, front at 117.4). Angular resolution is 1.0° per the datasheet (round 1 used 0.8°). | `sensors.lidar_vs_req`, LD19 datasheet | Choose the C1 (±30 mm from 0.05 m, 0.72°). |
| 15 | **M** | **Camera detection ≥ 10 FPS fails at 640 px.** YOLO26n NCNN is 67 ms/img, 7.8 FPS end-to-end (Ultralytics' Pi 5 numbers), before SLAM/Nav2 load. | `sensors.camera.fps` | 320–416 px input (est. 12–28 FPS under load: a scaling estimate, not measured), NCNN, ≤ 3 threads. |
| 16 | **M** | **URDF r1 sensor frames don't match CAD** (REQ-SCOUT-011). laser x 0 / z 0.131 vs CAD 0.045 / 0.151; camera rpy 0 vs CAD 15° pitch-down; IMU rpy 0 vs CAD chip facing down (needs roll π); IMU z 0.059 vs ≈ 0.049. The sim used v1 fallback mass 0.80 kg vs CAD 0.97–1.03 kg (sim torques/currents ~20 % optimistic). | `urdf/scout.urdf`, `sim/results.json _meta` | CAD exports `out/robot_params.json` (still missing); sim regenerates URDF/MJCF from it and re-runs the threshold + spin scenarios. |
| 17 | **M** | **Main switch rating.** A CAD MTS-102-class mini toggle is usually AC-rated (~3 A at 250 VAC, DC not stated); it has to carry 7.7 A bursts. | `power.protection.switch` | ≥ 10 A DC-rated switch. |
| 18 | **M** | **Heading budget needs gyro calibration, not just ZUPT.** MPU-6050 scale tolerance ±3 % gives 10.8° on a 4 × 90° square (budget 5°). The worst-case ZRO temperature drift (±20 °/s over 125 °C → 0.32 °/s/°C) uses the whole 0.15 °/s budget in ~0.5 °C. The IMU sits under the Pi's rear edge. | `odometry.gyro` | Scale-calibrate to ≤ 1 % (10 slow turns against the LiDAR/SLAM heading); ZUPT bias at every stop; move the IMU to x ≈ −25 mm. Allocation: 1.7° bias + 1.8° scale + 0.03° ARW = 2.5° RSS. |
| 19 | **M** | **Battery energy.** The label's 33.3 Wh means 634 Wh/L at pack level, which is implausible for a 15 mm-thick pouch pack with BMS/wrap. Realistic range 12–27.5 Wh. The 30 min worst-case target needs ≥ 14.2 Wh; the 60 min typical aim needs ≥ 16.8 Wh. | `power.battery`, `power.runtime` | M1: weigh it (×150–250 Wh/kg) and read the cell markings. |
| 20 | m | **The `scout_v2_no_payload` CAD variant is invalid:** LiDAR plate overlaps the Pi cooler (8096 mm³) and the Pi (1478 mm³). | `fit_check_v2.json` | Clamp the tower length to cooler top + clearance in `tower_length()`. |
| 21 | m | **Battery lateral clearance is 2.2 mm per side** (50 mm pack in a 54.4 mm gap between the *assumed* 18.8 mm gearboxes). A 22.4 mm-thick gearbox (Hiwonder's published section, orientation unknown) would collide by 1.4 mm. | `src/dims.py` | M5: measure the gap between the gearbox inner faces with the real brackets. |
| 22 | m | **Tightest gaps:** lidar_tower–cooler 0.75 mm (the real Active Cooler has corner reliefs: check), wheel–plate 1.57 mm (TT shaft wobble at the rim is typically 1–2 mm, A). | `fit_check tightest_gaps_mm` | Check on the real parts; add a 1 mm nylon washer behind the wheel if it rubs. |
| 23 | m | **Encoder logic levels.** FIT0450 encoder needs 4.5–7.5 V supply, so its outputs need 8 level-shift channels for the ESP32. Hiwonder encoder supply voltage is unpublished. HC-SR04 ECHO is 5 V: use a 1 k/2 k divider. | `sensors.ultrasonic`, FIT0450 wiki | Check the Hiwonder encoder at 3.3 V on arrival; otherwise use dividers. |
| 24 | m | **Encoder resolution.** ≥ ~820 counts/rev (×4) is needed for a 4-count/20 ms PID at 0.05 m/s. Hiwonder claims 1008; one buyer measured 600 → 2.9 counts/sample. | `odometry.cpr_required_x4` | Period (edge-timing) velocity below 0.1 m/s; PCNT counters; measure ppr on arrival (M7). |
| 25 | m | **Watchdog must brake.** Coasting takes 0.32–0.42 s and 49–64 mm in the sim (fails 300 ms); braking (LL) takes 0.16–0.19 s. Stop budget from 0.3 m/s at 1.5 m/s² with 175 ms latency is 83 mm, leaving 37 mm margin (C1) inside 15 cm. | `stability.stop`, `sim c_estop_payload` | MDD3A inputs LOW/LOW on watchdog. Safety stop triggers at ≤ 0.20 m from the bumper. |
| 26 | m | Round-1 note "LD19 PWM pin grounded" is wrong per the datasheet: leave it floating/Hi-Z for internal 10 Hz control. Moot if C1 is chosen. | LD19 datasheet | — |
| 27 | m | Stability is **not** a problem. C1 + payload: front-lift accel 4.8 m/s² (8× the 0.6 requirement), rear-lift brake 7.2 m/s² (4.8× the 1.5 cap), tip-back 26°, side tip 43°, 5° ramp uphill SF 6.6, nose-up on the step SF 5.9. Full-duty starts can lift the front (4.8 < μg 7.8), so ramp the PWM. Keep CoM within the a6 envelope (x ≥ −22 mm at z 60) and \|y\| < 10 mm. | `stability.cases`, `a6_stability.png` | Firmware accel ramp; no hardware change. |

### Is 1 rad/s achievable at all with L/B 0.65 on carpet?
Yes. It needs feedback, and it isn't free:
- Traction: turning needs side traction ≥ M_r/B with M_r = μ·W·L/2. μ cancels, so it's feasible whenever L/B < 1. Scout's L/B = 0.645.
- Kinematics: the sim's effective track is 1.70× (carpet) to 1.93× (hard floor) geometric, so the wheels must run at
  31–35 rpm (0.105–0.12 m/s at the rim). Every candidate motor does that easily.
- Torque: 0.047 (carpet, μ 0.6) to 0.056 N·m (hard, μ 0.8) per wheel. Stock TT/FIT0450 by printed torque: SF 1.15–1.73.
  Hiwonder at 8.4 V: SF 2.5–3.0 with payload.
- Recommendation: keep 1 rad/s as a *closed-loop yaw-rate* requirement (IMU-measured), give Nav2 0.8 rad/s, and drop
  the open-loop expectation. A design change (shorter wheelbase, or omni/caster rear wheels) isn't justified.

---

## 3. Power

### 3.1 Budget (`a5_power.png`)
| Load | Typ W | Peak W | Source |
|---|---|---|---|
| Pi 5 8 GB + Active Cooler | 6.5 | 10.0 | D/M cnx-software: 3.0 W idle headless, 8.8 W `stress -c 4`; typ = A (SLAM + Nav2 + detector) |
| Camera Module 3 | 0.6 | 1.0 | A: no official figure |
| RPLIDAR C1 | 1.15 | 1.5 | D 230 mA @ 5 V; start-up A |
| ESP32 DevKit (Wi-Fi off) | 0.5 | 1.0 | A from the ESP32 datasheet modem-sleep currents |
| 4 encoders | 0.2 | 0.25 | A |
| HC-SR04 + IMU + MDD3A logic | 0.2 | 0.3 | D 15 mA, 3.9 mA; A |
| **5 V rail** | **9.15 W / 1.83 A** | **14.05 W / 2.81 A** | buck in 9.9 / 14.9 W (η 93–94 %) |
| Motors, cruise 0.3 m/s carpet (4) | 3.1–4.4 | | C: Hiwonder model / stock-TT model |
| Motors, accel 0.6 m/s² | 4.5–6.9 | | C |
| Motors, pivot 1 rad/s carpet | 9.4–20.5 | | C: TT model is pessimistic (sim 0.72 A/motor vs model 1.0 A) |
| Motors, 4 stalled at the cap | 56 (Hiwonder 8.4 V) | | C: windings, motor side 6.7 A |
| Servo SG90 | 0 | | dropped in the v2 CAD (fixed camera); +~1 W peak if kept (A) |

Mission profile (A): 40 % stationary, 45 % cruise, 10 % turning, 5 % accel/threshold → **12.4–14.3 W average**.
Worst continuous (Pi peak + 70 % cruise + 30 % pivoting) → **19.9–24.2 W**.

### 3.2 Battery and runtime (85 % usable, A)
| Pack energy | Typical (min) | Worst (min) | vs 30 min / 60 min aim |
|---|---|---|---|
| 12 Wh (realistic low) | 43–49 | 25–31 | typical ✔ / worst ✘ (TT model) |
| **16.5 Wh (design)** | **59–68** | **35–42** | ✔ / ≈ aim |
| 27.5 Wh (realistic high) | 98–113 | 58–70 | ✔ / ✔ |
| 33.3 Wh (label, implausible) | 119–137 | 70–85 | — |
Needs ≥ 14.2 Wh for 30 min worst-case and ≥ 16.8 Wh for 60 min typical.

### 3.3 5 V buck (Pi 5): **Pololu D36V50F5**
- Input range: the pack is 9.0–12.6 V; dropout is ~0.83 / 0.92 / 1.0 V at 1 / 3 / 5 A (D graph), so it regulates down to ~6 V in. That leaves ≥ 3 V headroom, ≥ 1.4 V even during a 4-motor stall on a 0.25 Ω pack.
- Current and heat: ~8 A continuous at 9–18 V in, still air (graph). η at 12 V in is 91 % @ 1 A and 94–95 % @ 3–5 A, so losses are 0.73 W typical, 0.86 W peak, 1.3 W at 5 A.
- Built in: reverse-input protection (40 V), OCP, thermal shutdown, soft start, PG pin, EN pin. 25.4 × 25.4 × 9.5 mm, 7 g. €34.95.
- XL4015-class: rated ~3.5 A sustained without a heatsink (fails the Pi's 5 A spec), no reverse protection, pot drift. **Don't.**
- 5 A UBEC: no verifiable datasheet. **Don't.** Owned LM2596: 2–3 A. **Not for the Pi.**
- Pi side: back-power via header pins 2+4 / 6+14. Set `usb_max_current_enable=1` + `PSU_MAX_CURRENT=5000`, because without PD the Pi gives USB only 600 mA (1.6 A with a 5 A-capable supply, RPi docs). The GPIO 5 V path bypasses the Pi's input protection: keyed connector, polarity checked, no hot-plugging.

### 3.4 Motor rail: **no motor buck; PWM-cap on the pack**
- MDD3A runs 4–16 V, so it sits straight on 9.0–12.6 V.
- Firmware caps the duty at V_cap/V_batt: 8.4 V for Hiwonder (rated 3–12 V, recommended 6–9 V), 7.4 V for FIT0450 (max 7.5 V). A constant cap below the 9.0 V pack floor gives the same dynamics over the whole discharge.
- Heating: at 20 kHz the motor's L/R (~0.1–0.5 ms, A: inductance unpublished) is much longer than the 50 µs period. Ripple ≤ ~0.3 A p-p worst, extra copper loss < 0.02 W/motor, so heating is the same as DC at V_cap.
- Brush wear: arcing scales with the *peak* applied voltage, so expect somewhat faster brush wear than at 6 V DC. I can't quantify it from any datasheet. Acceptable at €6–13/motor with a spare. Speed and gearbox wear follow the capped average.
- A 5 A motor buck would current-limit exactly during climbs and stalls (1.2–2.4 A/channel climbing, 6–7 A stalled). The CAD's 2.8 A-family D36V28F is far too small. **Don't buy.**
- Failure mode: a firmware bug at 100 % duty applies 12.6 V. Hiwonder then stalls at 2.5 A/motor = 5 A/channel (MDD3A peak); FIT0450 at 5.9 A/motor trips the MDD3A OCP. Mitigation: the cap lives inside the lowest PWM-write function, the watchdog brakes, and the PWM pins boot LOW.
- Option: motors on the owned Elegoo 2S box (7.4–8.4 V), Pi on the 3S pack. That removes all coupling. Measure the box first (M8).

### 3.5 Peak current and brown-out
Battery current: average 1.3 A, worst continuous 2.4 A, **burst 7.7 A** (4 motors stalled at the cap + Pi peak, pack at
9.6 V), firmware fault 11.8 A. Bus voltage during the burst: 8.8 V (R_pack 0.06 Ω) / 7.3 V (0.25 Ω, A bound) vs the
buck's ~5.9 V minimum, so **no Pi brown-out from motor bursts**. The real brown-out risk is a **BMS over-current trip**
(finding #1).

### 3.6 Protection, wiring, cutoff
- **Fuse:** 7.5 A ATM mini-blade in an inline holder right after the battery XT30. It carries the 2.4 A worst continuous and the < 1 s 7.7 A bursts, and blows on a pack short (≫ 20 A).
- **Switch:** ≥ 10 A DC-rated (not the MTS-102 mini toggle).
- **Wire:** battery → fuse → switch → split 18 AWG silicone; MDD3A VM 20 AWG; motors 22–24 AWG; buck → Pi 20 AWG pair (≤ 15 cm, 0.03 V drop at peak); signals 26–28 AWG.
- **Bulk capacitors:** 470 µF/25 V low-ESR at MDD3A VM, 47–100 µF at the buck input (Pololu warns about LC hot-plug spikes).
- **Cutoff (3S):** warn 10.5 V, stop motors 10.2 V, Pi shutdown 9.9 V, floor 9.0 V. 2 s average, ignore dips < 0.5 s, 0.3 V hysteresis. Measure on ESP32 ADC1 through a 47 k/10 k divider (12.6 V → 2.21 V) with 100 nF, calibrated against the multimeter (REQ-SCOUT-013).
- **BMS:** unknown. With one, its 2.5–3.0 V/cell cutoff is the last line. Without one, the firmware is the only protection, so add a 3S buzzer alarm.

### 3.7 Power-distribution table
| Node | Voltage | I typ (A) | I peak (A) | Protection | Wire | Connector |
|---|---|---|---|---|---|---|
| Battery 3S | 9.0–12.6 V | 1.29 | 7.7 (11.8 fault) | BMS? + 7.5 A ATM fuse | 18 AWG | XT30 |
| Main switch | 9.0–12.6 V | 1.29 | 7.7 | — | 18 AWG | ≥ 10 A DC switch |
| MDD3A VM → 4 motors (2/channel) | 9.0–12.6 V, PWM avg ≤ 8.4 V | 0.40 | 6.2 | MDD3A OCP/UVLO/thermal + firmware cap | 20 AWG (motors 22–24) | screw terminal |
| D36V50F5 VIN | 9.0–12.6 V | 0.89 | 1.66 | reverse/OCP/thermal built in | 20 AWG | soldered |
| 5 V → Pi 5 header pins 2/4 | 5.0 V (4.8–5.2) | 1.30 | 2.0 | buck PG → ESP32 | 20 AWG pair | keyed 2×2 / JST-XH |
| Pi USB (or UART + 5 V) → C1 | 5.0 V (C1: 4.8–5.2 D) | 0.23 | 0.30 | Pi USB limit 0.6/1.6 A | kit cable | USB-A or XH2.54-5P |
| Pi CSI → Camera 3 | 3.3 V | ~0.12 (A) | ~0.2 (A) | — | FFC | 22-pin mini FFC |
| 5 V → ESP32 VIN, encoders, HC-SR04 | 5.0 V | 0.18 | 0.30 | — | 22–24 AWG | Dupont/JST-PH |
| ESP32 3V3 → MPU-6050, pull-ups | 3.3 V | 0.01 | 0.02 | — | 26 AWG | Dupont |

---

## 4. Stability (`a6_stability.png`, `results.json → stability`)
| Case (CAD r1) | CoM x/z mm | Front-lift accel | Rear-lift brake | Tip back / fwd / side | Accel SF (0.6) | Decel-cap SF (1.5) |
|---|---|---|---|---|---|---|
| LD19 empty | +9.0 / 47.9 | 10.0 | 6.35 | 46° / 33° / 52° | 16.7 | 4.2 |
| LD19 + 300 g | −10.6 / 61.6 | 4.68 | 8.06 | 26° / 39° / 45° | 7.8 | 5.4 |
| C1 empty | +11.2 / 53.3 | 9.42 | 5.30 | 44° / 28° / 49° | 15.7 | 3.5 |
| C1 + 300 g | −8.0 / 65.1 | 4.82 | 7.23 | 26° / 36° / 43° | 8.0 | 4.8 |
Accelerations in m/s². Each case is checked on flat floor, on a 5° ramp (uphill accel SF ≥ 6.3) and nose-up 7.2° on the
10 mm step (accel SF ≥ 5.7). Spin effects are negligible (centripetal 0.01 m/s²; payload tangential 0.37 m/s² for a
0.2 s spin-up). Static CoM envelope for ≥ 3 m/s² lift margin: |x| ≤ 21.7 mm at z 60 mm (back limit −21.7 mm),
|y| ≤ 10 mm.

## 5. Odometry / IMU (`a7_odometry.png`)
- **Encoder requirement:** ≥ ~820 counts/rev (×4) gives 4 counts per 20 ms at 0.05 m/s. Position for Nav2's 10 cm arrival is not the constraint (slip and localisation dominate). Hiwonder 1008 (claimed) → 4.9 counts/sample ✔; 600 (one buyer's measurement) → 2.9, so use edge timing below 0.1 m/s. FIT0450 → 9.4–18.8 ✔. Count with PCNT (≤ 16 kHz/motor, trivial); don't use GPIO ISRs.
- **Gyro:** see finding #18. Wheel yaw stays out of the EKF (sim: 184°/10 m uncalibrated; 55°/10 m with the slip calibration from the wrong floor; wheel distance + gyro yaw: 2.7°/10 m ✔).
- **IMU placement:** within 25 mm of the CoR, away from the Pi's heat (x ≈ −25 mm), nylon standoffs, DLPF 44 Hz, sample ≥ 200 Hz. Magnetics are irrelevant (no magnetometer). The URDF must carry the chip-down orientation.
- **ZUPT:** when |cmd_vel| = 0 and the encoders have been static for 0.5 s, update the bias by EMA with τ = 5 s.

## 6. Sensors (`a8_sensors.png`)
| | RPLIDAR C1 | LDROBOT LD19 |
|---|---|---|
| Range / min | 0.05–12 m (D) | 0.02–12 m (D) |
| Accuracy | ±30 mm from 0.05 m | ±45 mm only for 0.3–12 m; **unspecified ≤ 0.3 m** |
| Rate / angular res. | 5 kHz, 10 Hz, 0.72° | 4.5 kHz, 10 Hz, 1.0° (datasheet "typical"), angular error ≤ 2° |
| 2 beams per 5 cm cell up to | 2.0 m | 1.4 m |
| Interface / power | UART TX+RX 460800, 3.3 V; 4.8–5.2 V, 230 mA | UART TX 230400, 3.3 V; 4.5–5.5 V, 180/300 mA |
| ROS 2 driver | `sllidar_ros2`, official, C1 launch | `ldlidar_stl_ros2`, vendor; community Jazzy forks |
| Mass / height | 110 g / 41.3 mm | 47 g / 34.8 mm |
| Price (EU, Nov 2026) | €81.12 (Generation Robots) | €142.80 (Robot Italy) |
**Pick C1.** It's in spec at the 15 cm stop distance, has better angular sampling and an official driver, and costs €62 less.
Cost: +63 g at 15 cm height (stability margins still ≥ 3.5×) and a tighter 4.8–5.2 V supply window.

- **Camera:** the floor is visible from 109 mm ahead of the front (requirement ≤ 150 mm ✔). The lower FOV edge clears the mount by 9.4 mm, and the HC-SR04 does not block it. FPS: see #15.
- **HC-SR04:** 5 V supply; divide ECHO 1 k/2 k to 3.3 V. 2–400 cm, 15° cone, 60 ms cycle (16 Hz; 18 mm of travel per ping at 0.3 m/s). Reliable from ~3–5 cm (A). The beam covers z 2.9–6.9 cm at 15 cm range, which catches shoes, bowls and door stops below the 15.1 cm scan plane. Misses soft fabric and angled thin legs. Never the sole safety sensor.

---

## 7. Purchase recommendation
Prices are EU listings seen on 2026-11-01 unless marked A (not looked up; typical).

| Item | Recommendation | Why (quantitative) | Risks | Example product | ~EUR |
|---|---|---|---|---|---|
| 2D LiDAR | **BUY RPLIDAR C1** | ±30 mm from 0.05 m (stop zone at 0.22 m); 0.72° → 2 beams/5 cm cell to 2 m; official ROS 2 driver | 110 g high up; 4.8–5.2 V supply window; CAD margin over the payload only 5.2 mm | generationrobots.com 360° RPLIDAR C1 | 81 |
| 2D LiDAR alt. | **DON'T BUY LD19** | No accuracy spec ≤ 0.3 m; 1.0° res.; +€62 | — | robot-italy SEN0547 | 143 |
| Encoder motors | **BUY AFTER M2/M3/M5: Hiwonder TT Hall-encoder 1:42, 4 + 1 spare** | 3–12 V rating fits a PWM cap of 8.4 V; climb SF 2.5 (printed) / 3.3 (implied); stall 1.68 A at 8.4 V → 3.36 A/channel within MDD3A peak; 1008 cpr claimed (≥ 820 needed); compact encoder | Listing specs unverifiable ("torque 1.2 kg·cm" may not be stall); buyer measured 600 cpr; plastic shaft; envelope vs Elegoo bracket unmeasured; ships from HK | hiwonder.com/products/tt-motor-plastic ($6.99) | ~33 for 5 |
| Encoder motors alt. | **DON'T BUY FIT0450** unless its STEP passes a CAD check | 29.9 × 44.4 mm encoder PCB collides with the ESP32/lower deck; 2.8 A stall (6.9 A/channel at 7.4 V > MDD3A 5 A peak); 4.5–7.5 V encoder needs 8 level shifters; max 7.5 V | Printed torque inconsistent (×4.5–7) | opencircuit.shop FIT0450 €13.00 | 65 for 5 |
| Encoder motors alt. | **DON'T BUY Yahboom TT encoder** | Published specs self-contradictory (1.2 N·m, 16 000 rpm) | — | — | — |
| How many motors | **All 4 the same model, all 4 encoders wired** | Skid steer can't mix 1:42 and 1:48 on one side; 4 encoders allow slip detection (front vs rear); ESP32 pins fit (18 of 19 safe pins with MDD3A) | — | — | — |
| Motor driver | **BUY Cytron MDD3A** | 3 A continuous / 5 A peak per channel, 4–16 V (pack direct), 20 kHz, 1.8–12 V logic, OCP/UVLO/thermal; TB6612 fails climb/stall with 2 motors/channel | ~48 × 42 mm doesn't fit the CAD gap: re-layout | opencircuit.shop MDD3A | 13 |
| 5 V buck (Pi) | **BUY Pololu D36V50F5** | Dropout 1.0 V @ 5 A, ~8 A at 12 V in, η 94 %, reverse/OCP/thermal, PG/EN | Fixed 5.0 V vs REQ-012 floor (measure, M7) | berrybase.de POL4091 | 35 |
| 5 V buck alts | **DON'T BUY** XL4015 / 5 A UBEC | ~3.5 A sustained / no datasheet | — | — | — |
| Motor buck | **DON'T BUY** | Would starve climbs; PWM cap is thermally equivalent | Firmware-fault exposure (mitigated) | — | 0 |
| Fuse + holder | **BUY** inline ATM mini-blade holder (18 AWG leads) + 7.5 A fuses (pack of 5) | Carries 2.4 A continuous and 7.7 A < 1 s bursts; blows on a short | Re-size after M1 if the BMS trips lower | any automotive mini-blade inline holder | ~5 (A) |
| Main switch | **BUY** ≥ 10 A DC rocker/toggle | 7.7 A bursts; the CAD's MTS-102 is AC-rated ~3 A | Panel cut-out in the top plate (drill) | e.g. Pololu #1406 10 A rocker (check DC rating) | ~3 (A) |
| Connectors | **BUY** XT30 pairs ×5; JST-PH 2.0 / Dupont housings; JST-XH 2.54 5-pin (C1 UART lead) | Rated well above 7.7 A; keyed | — | AMASS XT30 | ~8 (A) |
| Capacitors | **BUY** 470 µF/25 V low-ESR + 100 µF/25 V | MDD3A VM ripple, buck hot-plug spike | — | — | ~2 (A) |
| Standoffs | **BUY** M2.5 brass M-F kit (6–40 mm) + M3 nylon kit; GeeekPi cluster-case layer (100 × 72) for the LiDAR plate | CAD tower stack uses catalogue lengths ≤ 2 per stack | Tower–cooler gap 0.75 mm: check the real cooler | GeeekPi B07MW24S61 (CAD ref) | ~15 + ~20 (A) |
| Wiring | **BUY** silicone wire 18/20/22 AWG, 26 AWG ribbon/Dupont | Section 3.6 | — | — | ~15 (A) |
| Battery alarm | **BUY ONLY IF M1 finds no BMS**: 3S LiPo buzzer | Only protection against over-discharge without a BMS | — | — | ~3 (A) |
| Level shifters | **DON'T BUY** (Hiwonder) / buy 2× 4-ch if FIT0450 | 5 V encoder outputs | — | — | 0–4 |
| Current sensor (INA226) | **DECIDE LATER** | Nice for REQ-013 and runtime logging, not required | — | — | ~5 (A) |

**Total if all "BUY" items:** ≈ €230–245 with the Hiwonder motors.

---

## 8. What is still unproven: Jaime measures first (free, ~2 h, all with parts on hand)
| ID | Measure | Decides |
|---|---|---|
| **M1** | Weigh the blue pack; read the cell/pack markings; look for a balance lead / BMS board; note any C-rating | Energy 12–27.5 Wh, runtime, BMS trip, fuse rating, alarm |
| **M2** | Stock TT stall torque at 6 V and 7.4 V (Elegoo 2S box): 10 cm lever on the wheel hub pressing on a kitchen scale (80 g ≙ 0.0785 N·m) + stall current on the multimeter 10 A range | Motor model and sim calibration; the printed-vs-implied discrepancy (#4) |
| **M3** | Drive the stock car (UNO firmware) with 300 g at the rear over the real door threshold, slow and at ~0.15 m/s; photograph the threshold profile (square or bevelled, height) | Whether #2 is real; the threshold mode |
| **M4** | Tilt test on carpet tile and hard floor: robot on a board, raise until it slides, μ = tan(angle) | Traction limit (μ ≥ 0.55–0.59 needed), spin torque |
| **M5** | Elegoo motor bracket: hole pattern, gearbox inner-face spacing (battery gap, 54.4 mm assumed), free length behind each motor to the plate end and to the ESP32 position | Encoder-motor fit (#3, #21) |
| **M6** | Weigh the plates, wheels, Pi 5 + cooler; measure tyre width | CAD mass (assumed 113–115 g plates, 25 g wheels, 25 g cooler) |
| **M7** | On arrival: D36V50F5 output at ~3 A load (≥ 5.0 V?); encoder pulses per wheel turn (10 turns); Hiwonder encoder at 3.3 V | REQ-012, odometry, level shifting |
| **M8** | Elegoo 2S battery box dimensions/mass | Two-pack option |

## 9. Assumptions (A) used
- Floor: rolling resistance Crr 0.015 hard / 0.08 carpet; lateral μ 0.6 carpet, 0.7–0.8 hard (sim uses 0.8). Not measured.
- Pi 5 typical 6.5 W under SLAM + Nav2 + detector (between the measured 3.0 W idle and 8.8 W full stress); peak 10 W. Camera 0.6/1.0 W. ESP32 0.5/1.0 W. Encoders 10 mA each.
- Pack energy 12–27.5 Wh (design 16.5); 85 % usable; pack resistance 0.06–0.25 Ω; path 0.04 Ω.
- Driver efficiency 0.95.
- TT gearbox efficiency 0.40–0.65, used only for the consistency check.
- 130-can inductance 0.3–1 mH, used only for PWM ripple.
- Mission profile 40/45/10/5 %.
- Effective spin track 1.70× (carpet) / 1.93× (hard), taken from the sim.
- Quasi-static square-edge climb with all four wheels driving equally.
- Cup on deck μ 0.3–0.5.
- Latency budget 175 ms (10 Hz scan 100 ms + ROS 50 + micro-ROS 20 + driver 5).
- YOLO 320 px FPS scaled from 640 px by FLOPs (not measured).
- D36V50F5 output tolerance ±4 % (round-1 reading, re-verify on the Pololu page).
- MDD3A size ≈ 48 × 42 mm (round-1 datasheet reading).
- Hiwonder specs as listed (unverified).
- CAD r1 masses: battery 150 g (A), plates by volume × 1.19 g/cm³ (A).
