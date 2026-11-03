## Checks

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

## BOM

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

## Cables

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

## Drilling

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
