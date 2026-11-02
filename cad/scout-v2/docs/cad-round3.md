# Scout v2 — CAD round 3 (physics review applied, deliverables)

**Author:** CAD engineer (agent) · **Date:** 2026-11-01 · **Branch:** `design/scout-mech-v2` (not committed) · **Status:** for Jaime's review, nothing bought.
**Default model:** `scout_v2` = SLAMTEC RPLIDAR C1 + Hiwonder TT Hall-encoder 1:42 ×4 + Cytron MDD3A on the pack + Pololu D36V50F5, 300 g cup at x = −50 mm.
Frame: REP-103 `base_footprint` (origin on the ground at the centre of rotation, +X forward, +Y left, +Z up), mm in CAD, m in `out/robot_params.json`.

## 1. What changed vs round 2 (finding by finding, `docs/physics-round2.md`)

| # | Finding | Round 3 | Status |
|---|---|---|---|
| 14 | Choose the C1 over the LD19 | Default LiDAR = **RPLIDAR C1** (110 g, 55.6 × 55.6 × 41.3 mm, scan plane 29.8 mm above its base, 4 × M2.5 at 43 × 43, datasheet v1.0 fig 4-1). LD19 kept as variants `scout_v2__ld19_*`. | applied |
| 3 / M5 | Motor envelope vs bracket | Hiwonder 1:42 kept as default (published drawing envelope, 79.95 × 22.4 × 25.79 mm, encoder cap Ø25.2 at the can end, 6-pin connector outboard). Sits on 3 mm M3 nylon spacers so the encoder clears the bottom plate (1.6 mm). `motor_fit.fits = True`; battery ↔ gearbox 2.2 mm per side. Bracket envelope is still **assumed** → M5. | applied, to measure |
| 6 | Delete the motor buck | `buck_motor` and its two cables are gone; the MDD3A runs straight on the switched pack (PWM cap in firmware). Asserted in the model body (no `buck_motor`, no USB parts). | applied |
| 5 | MDD3A re-layout | MDD3A (48.26 × 41.66, datasheet) hung under the top plate on 4 × M3 × 5 nylon standoffs, moved forward to x = +28 (x 3.9…52.1) to free x = −24 for the IMU. 7 mm above the battery. | applied |
| 17 | ≥ 10 A DC switch | MTS-102 envelope replaced by a **KCD1-form-factor panel rocker** (21 × 15 bezel, 13 × 19 cut-out, 15 mm below the plate incl. QC pins, 8 g) through the top plate at (−100, 0), 5 mm behind the cup keep-out, 10.6 mm above the ESP32. Generic KCD1 listings are rated 10 A 125 VAC / 6 A 250 VAC with **no DC rating** → buy one whose listing states ≥ 10 A at 12–14 VDC (Q-SW). | applied (part choice open) |
| 3.6 | Fuse, caps, power chain | battery → XT30 → 7.5 A ATM inline fuse holder (lower deck right) → rocker → split at the rocker's output QC → MDD3A VB (20 AWG, **470 µF/25 V** D10 × 16 at the terminal, `cap_driver`) and D36V50F5 VIN (20 AWG, 100 µF/25 V `cap_buck`). | applied |
| 4 (buck) | D36V50F5 | `buck_5v` = Pololu D36V50F5, 25.4 × 25.4 × 9.5 mm, 7 g (datasheet), lower deck front bay on a VHB pad. | applied |
| 10 | Payload at x = −50 | Keep-out Ø75 × 80 centred at x = −50 (asserted). Front load with the cup: **48.9 %** (physics target ≈ 47 %). LiDAR board rear edge moved to x = −5 → 7.5 mm from the cup (asserted ≥ 5). | applied |
| 13 | Cup retention | Fence of **4 × M3 × 20 brass M-F standoffs** at 45° around the cup, 2 mm radial play (asserted), on drilled Ø3.2 holes. | applied |
| 18 | IMU placement | GY-521 moved to x = −24 (chip centre 24.0 mm from the CoR, asserted ≤ 25), hung chip-down on 2 × M3 × 10 nylon standoffs; frame below. | applied |
| 8 | No USB | LiDAR USB adapter and USB cable removed. C1 → Pi UART0 (GPIO14/15, pins 8/10, 460800 baud), C1 5 V from the D36V50F5 output; ESP32 → Pi UART2 (GPIO4/5, pins 7/29). ESP32 USB-C only for flashing. | applied |
| 20 | Invalid no-payload variant | `tower_length()` now takes max(scan plane ≥ keep-out top + 5, board ≥ cooler top + 3) and the body asserts board − cooler ≥ 3 mm on the built parts. One tower (40 + 15 mm legs) for every variant, because the cup is a mission payload. `scout_v2__c1_hiwonder_nopay`: 0 overlaps, board − cooler 37.6 mm. | applied |
| 16 | URDF frames must match CAD | `out/robot_params.json` (v2-r3) exported from the built model: laser (0.055, 0, 0.1489), camera pitch +15° (down), IMU roll π, ultrasonic. The sim engineer regenerates URDF/MJCF from it. | applied (sim to regenerate) |
| 22 | Tightest gaps | lidar_tower–cooler gap is now 37.6 mm (tower no longer beside the cooler); wheel–plate 1.57 mm unchanged (assumed 2 mm wheel gap, M6). | improved |
| 21 | Battery lateral clearance | Still 2.2 mm per side against the assumed bracket/gearbox position → M5. | open (measure) |
| 11, 12 | Payload shadow / pitch on threshold | Scan plane 6.4 mm above the cup rim (dip that grazes the cup: 5.4°). Plant must stay inside Ø75 × 80. Firmware/scan-filter items are not CAD. | geometry reported |
| 1, 2, 4, 7, 9, 15, 19, 23–27 | Electrical / firmware / requirement items | Not geometry: left to physics/firmware. Cable plan updated where they touch wiring (fuse, switch, caps, UART). | not CAD |

Also kept (item 10 of the brief): HC-SR04 at the front below the scan plane (now shares 2 M3 screws with the camera holder, sensor 3 mm lower so the nuts clear the cans); Camera Module 3 fixed at 15° pitch-down (no SG90).

## 2. Check results (`checks/fit_check_v2.py` → `checks/results/*.json`)

| Check | `scout_v2` | `scout_v2__c1_hiwonder_nopay` | Requirement |
|---|---|---|---|
| Unintended overlaps | 0 | 0 | 0 |
| Intended overlaps | pi5/pi5_cooler 1659.62 mm³ | pi5/pi5_cooler 1659.62 mm³ | envelope only |
| Tightest gap (non-cable) | front_screw_1–top_plate 0.15 mm | front_screw_1–top_plate 0.15 mm | > 0 |
| Power-part min gap | 2.2 | 2.2 | ≥ 2 mm |
| LiDAR tower min gap | 3.25 | 3.25 | ≥ 2 mm |
| Scan plane z (mm) | 148.91 | 148.91 |  |
| Scan margin over highest part (mm) | 6.4 (payload) | 22.39 (cable_lidar_uart) | ≥ 5 |
| Scan margin over payload keep-out (mm) | 6.4 | 6.4 | ≥ 5 |
| Blind sectors (parts above the plane) | none | none | documented |
| Smallest dip that grazes a part (deg) | 5.42 (payload) | 26.21 (fence_post_3) | > 1.5 (C1 flatness) |
| Tower legs (catalogue stack, mm) | 40+15 | 40+15 |  |
| LiDAR board − cooler top (mm) | 37.6 | 37.6 | ≥ 3 |
| Camera floor visible ahead of front (mm) | 104.7 | 104.7 | ≤ 150 |
| Camera lower FOV edge blocked by | nothing | nothing | nothing |
| Ground clearance (mm) | 13.8 (bottom_plate) | 13.8 (bottom_plate) | ≥ 12 (10 mm step + 2) |
| Tail clearance, front wheels on 10 mm step | 4.63 | 4.63 | > 0 |
| Motor fits | True | True | True |
| Mass empty (kg) | 0.9981 | 0.9981 |  |
| CoM empty (mm) | [13.8, 0.0, 55.0] | [13.8, 0.0, 55.0] | |y| < 10, x ≥ −22 (a6) |
| Mass + 300 g (kg) | 1.2981 | 1.2981 |  |
| CoM + 300 g (mm) | [-0.9, 0.0, 66.0] | [-0.9, 0.0, 66.0] | |y| < 10, x ≥ −22 (a6) |
| Front/rear load empty | 0.673 / 0.327 | 0.673 / 0.327 |  |
| Front/rear load + 300 g | 0.489 / 0.511 | 0.489 / 0.511 | ≈ 0.47 front (finding 10) |
| Wheel loads + 300 g (N, fl/fr/rl/rr) | 3.11/3.11/3.26/3.26 | 3.11/3.11/3.26/3.26 |  |
| IMU chip centre (mm), dist. to CoR | [-24.0, 0.0, 45.95], 24.0 | [-24.0, 0.0, 45.95], 24.0 | ≤ 25 |
| Jaime's numbers on the geometry | deck_gap_39.7=39.705, plate_t_4.5=4.5, wheel_d_65=65.0, battery_70x50x15=[70.0, 50.0, 15.0], payload_keepout_d75_h80=[75.0, 80.0], payload_centre_x_-50=-50.0, board_minus_cooler_ge_3=37.6, board_rear_edge_minus_cup_ge_5=7.5, no_motor_buck_no_usb=True | deck_gap_39.7=39.705, plate_t_4.5=4.5, wheel_d_65=65.0, battery_70x50x15=[70.0, 50.0, 15.0], board_minus_cooler_ge_3=37.6, no_motor_buck_no_usb=True | match |

Notes: the 0.15 mm "tightest gap" is an M3 × 12 shaft (Ø2.9) in its Ø3.2 drilled hole (a clearance fit, not a clash); the next tightest are wheel ↔ plate 1.57 mm (assumed wheel gap) and cooler ↔ Pi standoffs 1.59 mm. In the no-payload variant the highest part under the scan plane is the C1 UART lead over the board (22.4 mm). With no parts above the plane there are **no LiDAR self-occlusion sectors**; the only near-plane object is the cup keep-out behind the LiDAR (rear sector 180 ± 21° (cup Ø75 at 105 mm from the LiDAR axis), grazed only if the scan dips ≥ 5.4°). Empty CoM is front-heavy (67 % front, C1 at x = +55); with the cup it is balanced (49 / 51). Both CoMs are inside the a6 envelope (x ≥ −22 mm, |y| < 10).

IMU frame (`sensors.imu`): MPU-6050 die centre at (−0.024, 0.000, 0.04596) m, rpy (π, 0, 0) — chip facing down, board +X arrow forward, so sensor +Y = robot −Y and sensor +Z = robot −Z.

## 3. Deliverables

| File | What |
|---|---|
| `out/robot_params.json` | BRIEF §4 schema, v2-r3, default model; plus `robot_params_c1_hiwonder_nopay.json`, `robot_params_ld19_hiwonder_pay.json` |
| `out/GLB/scout_v2.glb` | coloured robot for the web viewer |
| `out/GLB/scout_v2_assembly.glb` | animation `assembly`: steps 01–09 fly in, 1.6 s per step (16.4 s) |
| `out/GLB/scout_v2_exploded.glb` | static exploded view |
| `out/STL/scout_v2.stl`, `out/STL/parts/<label>.stl` | whole assembly + one STL per part, mm, in the robot frame |
| `out/3MF/scout_v2.3mf`, `out/STEP/scout_v2.step` | 3MF; STEP AP242 (git-ignored) |
| `assembly/step01.png … step09.png` | assembly guide (new parts in colour, earlier parts grey; step 05 drawn lifted above the chassis and seen from below; from step 06 the top plate is translucent) |
| `shots/scout_v2_iso.png`, `_side.png`, `_top.png`, `_exploded.png` | overview shots |
| `src/exports.py` | rewrites every file above (`cadgen +0.8.4 run src/exports.py`) |
| `checks/report_tables.py` | regenerates the tables in this doc from `checks/results/*.json` |

### Assembly steps (`scout_v2.BUILD_STEPS`, same order in the animation)
01 bottom plate, 3 mm motor spacers, Elegoo brackets, Hiwonder motors, wheels · 02 battery, XT30, fuse holder, D36V50F5 + 100 µF · 03 ESP32 + encoder and 5 V leads · 04 deck pillars · 05 top plate on the bench: drill (table below), MDD3A + 470 µF, IMU chip-down, rocker, HC-SR04 + camera holder (shared screws) · 06 top plate onto the pillars, fuse → rocker → MDD3A/buck, motor and signal leads · 07 Pi 5 on M2.5 × 6 standoffs + cooler, 5 V to GPIO 2/4 + 6/14, ESP32 UART, camera + FFC · 08 LiDAR legs (M3 40 + 15 stacked), FR4 board, C1, UART + 5 V leads · 09 fence posts (+ cup).

## 4. BOM (every physical part in the model)

| Label(s) | Qty | Product | Source / URL | Mass each (g) | Dim./mass source |
|---|---|---|---|---|---|
| `battery` | 1 | Blue 3S Li-ion pack 70 x 50 x 15, 11.1 V 3000 mAh label (owned) | Jaime | 110 | measured |
| `bottom_plate` | 1 | Elegoo Smart Robot Car V4.0 bottom plate (owned) | Elegoo kit STL 'Bottom Plate.stl' | 114.5 | measured |
| `buck_5v` | 1 | Pololu D36V50F5 5 V 5.5 A step-down regulator (#4091) | https://www.pololu.com/product/4091/specs | 7 | datasheet |
| `camera` | 1 | Raspberry Pi Camera Module 3 (owned) | https://datasheets.raspberrypi.com/camera/camera-module-3-standard-mechanical-drawing.pdf | 4 | datasheet |
| `camera_mount` | 1 | generic tilt-adjustable acrylic Raspberry Pi camera holder (2-piece, hinge screw; e.g. listing 'Raspberry Pi camera module acrylic holder bracket'), locked at 15 deg down; dims assumed | generic listing | 4.9 | assumed |
| `cap_buck` | 1 | 100 uF / 25 V radial electrolytic D6.3 x 11 (buck VIN) | any (e.g. Panasonic FR series) | 0.6 | assumed |
| `cap_driver` | 1 | 470 uF / 25 V low-ESR radial electrolytic D10 x 16 (Panasonic EEU-FR1E471) | https://industrial.panasonic.com/cdbs/www-data/pdf/RDF0000/ABA0000C1209.pdf | 2 | datasheet |
| `deck_pillar_1..6` | 6 | Elegoo kit M3 brass hex pillar, 39.7 mm between the decks (owned) | Elegoo V4.0 kit | 8 | measured |
| `esp32` | 1 | ESP32 DevKit WROOM-32 USB-C (owned) | - | 10 | assumed |
| `fence_nut_1..4` | 4 | M3 nut under the top plate (fence) | ISO 4032 | 0.5 | datasheet |
| `fence_post_1..4` | 4 | M3 x 20 brass hex M-F standoff (payload fence) | RS PRO 222-452 family | 4 | datasheet |
| `front_screw_1..2` | 2 | M3 x 12 pan screw + M3 nut (camera holder + HC-SR04 bracket, shared) | ISO 7045 / ISO 4032 | 1.3 | datasheet |
| `fuse_holder` | 1 | Inline ATM mini-blade fuse holder (18 AWG) + 7.5 A ATM fuse | any automotive supplier | 5 | assumed |
| `imu` | 1 | GY-521 MPU-6050 breakout (owned) | - | 2 | assumed |
| `imu_standoff_1..2` | 2 | M3 x 10 nylon hex F-F standoff + M3 x 6 nylon screws | nylon standoff kit | 0.2 | datasheet |
| `lidar` | 1 | SLAMTEC RPLIDAR C1 | https://www.slamtec.com/en/C1 (datasheet v1.0 fig 4-1) | 110 | datasheet |
| `lidar_board` | 1 | Electrokit ProtoPCB 80 x 120 mm (EKB12-class FR4 prototyping board) | https://www.electrokit.com/en/experimentkort-1-hal-80x120mm-genomplaterade-hal | 28.3 | datasheet |
| `lidar_board_screw_1..4` | 4 | M3 x 6 pan screw on the LiDAR board | ISO 7045 | 0.5 | datasheet |
| `lidar_leg_1..4` | 4 | M3 brass hex M-F standoffs, 5.5 AF (e.g. RS PRO 222-452 family / Wurth 971xxx), M3 nut under the plate, M3 x 6 screw on top | RS PRO 222-452 family | 11.1 | datasheet |
| `lidar_leg_screw_1..4` | 4 | M3 nut under the top plate (LiDAR leg) | ISO 4032 | 0.5 | datasheet |
| `lidar_screw_1..4` | 4 | M2.5 x 5 pan screw (LiDAR to board) | ISO 7045 | 0.2 | datasheet |
| `mdd3a` | 1 | Cytron MDD3A 4-16 V 3 A dual DC motor driver | https://www.cytron.io/p-3amp-4v-16v-dc-motor-driver-2-channels | 14.5 | datasheet |
| `mdd3a_standoff_1..4` | 4 | M3 x 5 nylon hex F-F standoff + M3 x 6 nylon screws | nylon standoff kit | 0.1 | datasheet |
| `motor_bracket_fl, motor_bracket_fr, motor_bracket_rl, motor_bracket_rr` | 4 | Elegoo kit TT motor holder (owned) | Elegoo V4.0 kit | 1.4 | assumed |
| `motor_fl, motor_fr, motor_rl, motor_rr` | 4 | Hiwonder TT Hall-encoder gear motor 1:42, plastic shaft (SKU 21100132) | https://www.hiwonder.com/products/tt-motor-plastic | 34 | datasheet |
| `motor_spacer_fl, motor_spacer_fr, motor_spacer_rl, motor_spacer_rr` | 4 | M3 nylon round spacer 3 mm (2 per bracket) | Essentra/Keystone M3 nylon spacer kit | 0.2 | datasheet |
| `payload` | 1 | Seedling cup keep-out Ø75 x 80, <= 300 g (not bought) | BRIEF | 300 | assumed |
| `pi5` | 1 | Raspberry Pi 5 8 GB (owned) | vendor/raspberry_pi_5.step | 46 | datasheet |
| `pi5_cooler` | 1 | Raspberry Pi Active Cooler (owned) | RP-008187-DS-1 mechanical drawing | 25 | datasheet |
| `pi_screw_1..4` | 4 | M2.5 nut under the top plate (Pi standoff) | ISO 4032 | 0.2 | datasheet |
| `pi_standoff_1..4` | 4 | M2.5 x 6 brass hex M-F standoff | M2.5 brass standoff kit | 1 | datasheet |
| `switch` | 1 | KCD1-form-factor SPST rocker, >= 10 A @ 12 VDC rated (DC rating to confirm on the listing) | https://probots.co.in/kcd1-spdt-momentary-rocker-switch-3-pin-motor-control.html (envelope) | 8 | datasheet |
| `top_plate` | 1 | Elegoo Smart Robot Car V4.0 top plate (owned), drilled per the drilling table | Elegoo kit STL 'Top Plate.stl' | 110.8 | measured |
| `ultrasonic` | 1 | HC-SR04 ultrasonic sensor (owned) | https://www.mouser.com/datasheet/2/813/HCSR04-1022824.pdf | 8.5 | datasheet |
| `ultrasonic_bracket` | 1 | generic HC-SR04 L-mount bracket (acrylic/ABS, 2 x M3 base holes), e.g. listing 'HC-SR04 ultrasonic sensor mounting bracket'; dims assumed | generic listing | 3.6 | assumed |
| `wheel_fl, wheel_fr, wheel_rl, wheel_rr` | 4 | Elegoo kit 65 mm wheel (owned) | Elegoo V4.0 kit | 25 | datasheet |
| `xt30` | 1 | AMASS XT30U-M/F pair | https://www.amass.com.cn/ | 3 | datasheet |

Cables (approximate routes, +15 % slack):

| Route | Kind | Path (mm) | +15 % (mm) | Cable |
|---|---|---|---|---|
| `cable_batt_xt30` | 18awg | 14.8 | 17.0 | battery pack lead (as supplied) + XT30U-M, 18 AWG silicone |
| `cable_xt30_fuse` | 18awg | 94.0 | 108.1 | XT30U-F pigtail 18 AWG into the inline ATM holder's lead (solder + heat-shrink) |
| `cable_fuse_switch` | 18awg | 97.2 | 111.7 | 18 AWG silicone, 6.3 mm insulated female QC on the rocker's input terminal |
| `cable_switch_driver` | 20awg | 106.2 | 122.1 | 20 AWG silicone pair (+ GND return) from the rocker output QC to the MDD3A VB+/VB- terminal; 470 uF/25 V soldered across VB+/VB- at the terminal |
| `cable_switch_buck5` | 20awg | 159.4 | 183.3 | 20 AWG silicone pair to the D36V50F5 VIN/GND (both 20 AWG wires crimped together in the rocker's output QC = the split); 100 uF/25 V across VIN |
| `cable_buck5_pi` | 20awg | 113.2 | 130.1 | 20 AWG pair, 2.54 mm Dupont housings on GPIO pins 2/4 (+5 V) and 6/14 (GND), <= 15 cm |
| `cable_5v_esp32` | 22awg | 108.3 | 124.6 | 22 AWG pair from the D36V50F5 VOUT to the ESP32 VIN/GND (also feeds encoders + HC-SR04) |
| `cable_buck5_lidar` | 22awg | 234.1 | 269.2 | 22 AWG pair from the D36V50F5 VOUT through the top-plate front slot to the C1 lead's VCC/GND |
| `cable_lidar_uart` | signal | 121.3 | 139.5 | C1 XH2.54-5P lead: TX/RX/GND as F Dupont to Pi GPIO 14/15 (pins 8/10, UART0) + GND pin 9 |
| `cable_driver_esp32` | signal | 66.5 | 76.5 | 5 x F-F Dupont jumpers (M1A/M1B/M2A/M2B PWM + GND) |
| `cable_imu_esp32` | signal | 26.1 | 30.0 | 4 x F-F Dupont jumpers (VCC 3.3 V, GND, SDA, SCL) |
| `cable_us_esp32` | signal | 174.8 | 201.0 | 4 x F-F Dupont jumpers 20-30 cm (VCC, TRIG, ECHO via 1k/2k divider, GND) |
| `cable_esp32_pi` | signal | 218.4 | 251.1 | 3 x F-F Dupont jumpers 30 cm (TX, RX, GND) to Pi GPIO4/5 (pins 7/29, UART2, dtoverlay=uart2-pi5) |
| `cable_camera_ffc` | ffc | 121.0 | 139.2 | Raspberry Pi 5 camera cable, 22-pin 'standard-mini' (official 200 mm) |
| `cable_driver_motor_fl` | 22awg | 118.4 | 136.2 | 22 AWG pair from the MDD3A outputs to each motor (encoder motors: the motor pins of the 6-pin lead) |
| `cable_enc_fl` | signal | 174.7 | 200.9 | motor's own 6-pin lead (PH2.0 / XH) + Dupont breakout to the ESP32 (A, B, VCC 3.3 V, GND) |
| `cable_driver_motor_fr` | 22awg | 118.4 | 136.2 | 22 AWG pair from the MDD3A outputs to each motor (encoder motors: the motor pins of the 6-pin lead) |
| `cable_enc_fr` | signal | 174.7 | 200.9 | motor's own 6-pin lead (PH2.0 / XH) + Dupont breakout to the ESP32 (A, B, VCC 3.3 V, GND) |
| `cable_driver_motor_rl` | 22awg | 128.3 | 147.5 | 22 AWG pair from the MDD3A outputs to each motor (encoder motors: the motor pins of the 6-pin lead) |
| `cable_enc_rl` | signal | 60.7 | 69.8 | motor's own 6-pin lead (PH2.0 / XH) + Dupont breakout to the ESP32 (A, B, VCC 3.3 V, GND) |
| `cable_driver_motor_rr` | 22awg | 128.3 | 147.5 | 22 AWG pair from the MDD3A outputs to each motor (encoder motors: the motor pins of the 6-pin lead) |
| `cable_enc_rr` | signal | 60.7 | 69.8 | motor's own 6-pin lead (PH2.0 / XH) + Dupont breakout to the ESP32 (A, B, VCC 3.3 V, GND) |

Mass note: brass standoffs/pillars are modelled as solid Ø5.5 cylinders (23.8 mm² section) where a real 5.5 AF hex with an M3 bore is ≈ 19 mm², so their ≈ 112 g is ≈ 20 % (≈ 20 g) heavy.

## 5. Drilling table (Jaime drills; XY in the robot frame, origin = centre of rotation, +X toward the camera end, +Y left)

The Elegoo plate origin: x = 0 is midway between the axles (pillar holes at x −80.7/−0.7/+84.4 are a handy reference), y = 0 is the centreline. Every hole is asserted in the model to lie inside its plate with ≥ 3 mm of material to the edge, to every existing hole and to the other new holes, and is cut in the CAD top plate. Mark from the pillar holes, centre-punch, pilot Ø1.5, then final size.

| # | Part | Purpose | X (mm) | Y (mm) | Hole |
|---|---|---|---|---|---|
| 1 | top_plate | Pi 5 M2.5 standoff | +74.0 | +24.5 | Ø2.7 |
| 2 | top_plate | Pi 5 M2.5 standoff | +74.0 | -24.5 | Ø2.7 |
| 3 | top_plate | Pi 5 M2.5 standoff | +16.0 | +24.5 | Ø2.7 |
| 4 | top_plate | Pi 5 M2.5 standoff | +16.0 | -24.5 | Ø2.7 |
| 5 | top_plate | MDD3A M3 nylon standoff | +49.6 | +18.3 | Ø3.2 |
| 6 | top_plate | MDD3A M3 nylon standoff | +49.6 | -18.3 | Ø3.2 |
| 7 | top_plate | MDD3A M3 nylon standoff | +6.4 | +18.3 | Ø3.2 |
| 8 | top_plate | MDD3A M3 nylon standoff | +6.4 | -18.3 | Ø3.2 |
| 9 | top_plate | IMU M3 nylon standoff | -16.4 | +5.6 | Ø3.2 |
| 10 | top_plate | IMU M3 nylon standoff | -16.4 | -5.6 | Ø3.2 |
| 11 | top_plate | LiDAR tower leg M3 | +95.0 | +34.0 | Ø3.2 |
| 12 | top_plate | LiDAR tower leg M3 | +95.0 | -34.0 | Ø3.2 |
| 13 | top_plate | LiDAR tower leg M3 | +2.0 | +34.0 | Ø3.2 |
| 14 | top_plate | LiDAR tower leg M3 | +2.0 | -34.0 | Ø3.2 |
| 15 | top_plate | payload fence post M3 | -20.1 | +29.9 | Ø3.2 |
| 16 | top_plate | payload fence post M3 | -20.1 | -29.9 | Ø3.2 |
| 17 | top_plate | payload fence post M3 | -79.9 | +29.9 | Ø3.2 |
| 18 | top_plate | payload fence post M3 | -79.9 | -29.9 | Ø3.2 |
| 19 | top_plate | camera holder + HC-SR04 bracket shared M3 | +110.0 | +10.0 | Ø3.2 |
| 20 | top_plate | camera holder + HC-SR04 bracket shared M3 | +110.0 | -10.0 | Ø3.2 |
| 21 | top_plate | KCD1 rocker cut-out 13 (X) x 19 (Y): 2 x Ø12 + file | -100.0 | +0.0 | 13 x 19 mm cut-out |
| 22 | lidar_board | leg M3 through the FR4 board | +95.0 | +34.0 | Ø3.2 |
| 23 | lidar_board | leg M3 through the FR4 board | +95.0 | -34.0 | Ø3.2 |
| 24 | lidar_board | leg M3 through the FR4 board | +2.0 | +34.0 | Ø3.2 |
| 25 | lidar_board | leg M3 through the FR4 board | +2.0 | -34.0 | Ø3.2 |
| 26 | lidar_board | SLAMTEC RPLIDAR C1 M2.5 fixing | +76.5 | +21.5 | Ø2.7 |
| 27 | lidar_board | SLAMTEC RPLIDAR C1 M2.5 fixing | +76.5 | -21.5 | Ø2.7 |
| 28 | lidar_board | SLAMTEC RPLIDAR C1 M2.5 fixing | +33.5 | +21.5 | Ø2.7 |
| 29 | lidar_board | SLAMTEC RPLIDAR C1 M2.5 fixing | +33.5 | -21.5 | Ø2.7 |

The rocker cut-out (#21) is the only non-round hole: drill 2 × Ø12 at x −100, y ±3.5, then file to 13 × 19 until the KCD1 snaps in. Alternative if filing is unwanted: a round Ø20 rocker drilled with a step drill (re-check the rating).

## 6. Assumptions (every value not measured or from a datasheet)

| Value | Used | Why / how to retire it |
|---|---|---|
| Battery mass 110 g | `battery` | 52.5 cm³ × 2.1 g/cm³ (physics); weigh (M1) |
| Plates 1.19 g/cm³ (≈ 111–115 g each) | plates | acrylic; weigh (M6) |
| Wheel 25 g, tyre width 26 mm, wheel mid-plane y = ±62 (2 mm gap to the spine) | wheels, track 0.124 m | weigh/measure (M6) |
| Gearbox rests on the bottom plate in the Elegoo bracket (shaft 11.2 mm above the plate); bracket wall 12 × 5 × 22 mm at y 22.2–27.2 | motors, battery gap | measure the bracket (M5) |
| Hiwonder encoder-cap height/position, connector block 10 mm tall | motors | from the published drawing, scaled; measure on arrival |
| 3 mm motor spacer stack | ground clearance 13.8 mm | follows from the encoder envelope (M5) |
| MDD3A component height 11 mm, solder side 1.5 mm, terminal layout (VB on the rear short edge) | under-plate stack, cable ends | check the board |
| ESP32 55 × 28 × 13 mm, 10 g; GY-521 2 g, holes at (+7.6, ±5.6) | lower deck, IMU | measure the owned boards |
| HC-SR04 bracket 15 × 47 × 2 mm + 22 mm wall; camera holder 20 × 30 × 3 mm + 26 mm upright, hinge for 15° | front | generic listings: measure what is bought |
| KCD1 split 6 mm above / 15 mm below the plate | switch | listing gives 21 mm overall only |
| Fuse holder body 35 × 13 × 10 mm, 5 g; XT30 pair 3 g | lower deck | measure what is bought |
| 100 µF cap D6.3 × 11, 0.6 g; 470 µF 2 g | caps | typical radial sizes |
| C1 turret Ø50; LiDAR lead exits the base's rear face | LiDAR, cable | drawing proportions; check the C1 |
| Pi 5 46 g, cooler 25 g | top deck | retailer figure / assumed (M6) |
| Cable g/m and bundle Ø | cable masses (≈ 15 g total) | silicone-wire tables |
| Active Cooler as a solid 63.5 × 42.5 × 10 envelope | intended overlap with the Pi (1660 mm³) | the real heatsink has cut-outs |
| Payload as a uniform Ø75 × 80 cylinder, 300 g | with-payload mass/CoM | BRIEF limit |

## 7. Open questions / things Jaime must measure

- **M5 (motor fit):** Elegoo bracket hole pattern, gearbox inner-face spacing (54.4 mm assumed between gearboxes → 2.2 mm each side of the 50 mm battery), whether the Hiwonder encoder cap clears the plate with 3 mm spacers, free length behind each motor. Decides whether the Hiwonder fits as modelled.
- **M1:** battery mass, markings, BMS → fuse size (7.5 A assumed OK), battery mass in the CoM.
- **M6:** weigh plates, wheels, Pi + cooler; measure tyre width and the wheel-to-plate gap (1.57 mm in CAD).
- **M7 (on arrival):** D36V50F5 output at 3 A; Hiwonder encoder at 3.3 V; ppr.
- **M8:** only if the two-pack option (motors on the Elegoo 2S box) is taken — that would add a box to the lower deck, not modelled.
- **Q-SW:** a KCD1-size rocker with a published **DC** rating ≥ 10 A at 12–14 V (generic KCD1 = AC ratings only). If none is found, use a larger DC-rated rocker and re-place the cut-out.
- **Q-C1:** C1 connector position and pinout (lead assumed to exit the base's rear; 5 V from the buck, UART to GPIO14/15). Check the SLAMTEC C1 datasheet before wiring.
- **Q-UART:** Pi 5 UART2 on GPIO4/5 (`dtoverlay=uart2-pi5`) for the ESP32 and UART0 on GPIO14/15 for the C1 (disable the serial console). Confirm in the Pi docs.
- **Q-MDD3A:** which edge carries the VB+/VB− terminal (cap and cable ends assume the rear short edge).
- **Q-holders:** the exact HC-SR04 bracket and tilting camera holder to buy (dimensions assumed); the shared-screw holes (#19, #20) follow the holder bought.
- Brass standoff masses are over-estimated (solid cylinders): expect the real robot ≈ 20 g lighter than 0.998 kg (before M1/M6 corrections).

## 8. Not done / limits
- No vendor STEPs exist on step.parts for the C1, MDD3A, D36V50F5, Hiwonder motor or KCD1 (searched 2026-11-01): all are datasheet envelopes (tagged in `src/dims.py`).
- Cables are routing plans (tubes), excluded from the interference check.
- The variants other than `scout_v2` and `scout_v2__c1_hiwonder_nopay` build with the round-3 layout but were not re-checked this round (`cadgen +0.8.4 run checks/fit_check_v2.py` with no arguments runs all 17).
