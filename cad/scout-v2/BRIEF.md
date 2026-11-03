# Scout mechanical design v2 — engineering brief (shared by every engineer)

**Owner:** Argos (lead) · **Approver:** Jaime Tellie · **Started:** 2026-11-01
**Why:** Jaime will not order parts (LiDAR, encoder motors, buck converters) until a detailed CAD +
physics design proves they fit, carry the load, and work together. This package is also his
**visual assembly guide**. The v1 fit check (`v1-reference/`) was a coarse block layout; v2 is the
in-depth version that drives the purchase.

Work runs in **rounds**. Round 1: three engineers in parallel (CAD, physics/electrical, simulation).
Round 2: cross-review and fix (the physics engineer spars with the CAD design). Round 3: consolidate
docs and visual artifacts.

## 1. Fixed inputs (decided by Jaime, 1 Nov 2026)
| Topic | Decision |
|---|---|
| Battery | Blue pack **70 × 50 × 15 mm** (measured by Jaime). Labelled 3S Li-ion, 12.6 V full / 11.1 V nominal, 3000 mAh (≈33 Wh). **Sanity-check this:** 52.5 cm³ for 33 Wh is ~0.63 Wh/cm³, plausible only for LiPo/Li-ion pouch cells. Mass unknown → estimate from energy density and mark assumed. |
| Fabrication | **Off-the-shelf only**: standoffs/spacers (M2.5/M3 brass or nylon), metal L-brackets, screws, zip ties, adhesive pads, and **drilling the Elegoo plates**. **No 3D-printed or laser/CNC-cut custom parts.** Any mast, platform or bracket must be a purchasable item (or an existing Elegoo part) with a real product reference. |
| Environment | Indoor only: carpet tiles + hard floor, **10 mm door thresholds**, payload **≤ 300 g** (e.g. a seedling cup). |
| Chassis | Reuse the Elegoo Smart Robot Car V4.0: two plates 132.52 × 230.78 × 4.5 mm, **39.7 mm clear between decks** (measured), 4 wheels Ø65 mm, TT motors 1:48 (stock, no encoders). Plate STLs: `~/Downloads/ELEGOO Smart Robot Car Kit V4.0 2023.02.01/02 Manual & Main Code & APP/05 3D Model/{Top Plate,Bottom Plate,Dead Plate}.stl` |
| Compute/sensors (owned) | Raspberry Pi 5 8 GB (+ Active Cooler), Pi Camera Module 3, classic ESP32 DevKit (WROOM-32, USB-C), MPU-6050 (GY-521), HC-SR04 (+ SG90 servo on the stock car), TB6612FNG (on the Elegoo shield), 2× LM2596 bucks (≤ 2–3 A), Elegoo 2S battery box (7.4 V), multimeter. |
| To buy (selection is the point of this work) | 2D LiDAR (**LDROBOT LD19** vs **SLAMTEC RPLIDAR C1**), **4 (or 2) TT-form-factor gear motors with hall encoders** that fit the Elegoo mounts and 65 mm wheels, a **5 V / 5 A buck** for the Pi 5 (XL4015-class, Pololu D36V50F5-class, or a 5 A UBEC), possibly a **motor-rail buck** (TT motors are rated 3–6 V; the 3S pack is 12.6 V full; TB6612 VM max 13.5 V), fuse + holder, switch, connectors (XT30/JST), standoffs. |

## 2. Requirements to design against (from `docs/specs/srs.md`)
- REQ-SCOUT-001: differential/skid drive; follow **0.3 m/s linear and 1 rad/s angular**.
- REQ-SCOUT-002: stop within 300 ms on lost `/cmd_vel`.
- REQ-SCOUT-003: encoders read ≥ 50 Hz. REQ-SCOUT-004/006: wheel odometry + IMU EKF, heading drift < 5° / 10 m on carpet.
- REQ-SCOUT-007/008: 2D SLAM at 5 cm; Nav2 arrives within 10 cm / 10°.
- REQ-SCOUT-009: stop forward motion for an obstacle < 15 cm in front.
- REQ-SCOUT-010: camera detection ≥ 10 FPS. REQ-SCOUT-011: URDF with correct laser/camera/imu frames that match CAD.
- REQ-SCOUT-012: Pi 5 gets 5.0–5.25 V up to 5 A, no undervoltage in a 30 min run. REQ-SCOUT-013: battery voltage measured.
- Derived targets for this design: ≥ 30 min runtime at typical load (aim ≥ 60); climb a 10 mm threshold; no tip-over or wheel lift at max accel/brake with 300 g payload; LiDAR 360° scan plane above every robot part (≥ 5 mm margin) and documented blind sectors; camera sees the floor ≥ 15 cm ahead.

## 3. Conventions (v2 changes the frame — important)
- **v2 uses ROS REP-103 directly**, so CAD → URDF needs no transform: origin = `base_footprint` on the ground plane, at the **centre of rotation** (midpoint between front and rear axles, on the centreline); **+X forward** (the Elegoo servo / "Dead Plate" end), **+Y left**, **+Z up**, millimetres in CAD, metres in URDF/MJCF. v1 used +Y forward, +X right, Z=0 at the bottom-plate underside — convert when reusing v1 numbers.
- Every dimension and mass carries a source: `measured` (by Jaime or from the Elegoo STLs), `datasheet` (with URL), or `assumed` (with reason). No invented values presented as facts.
- Part labels (snake_case) are shared across CAD, URDF and analysis: e.g. `bottom_plate`, `top_plate`, `motor_fl`, `wheel_fl`, `battery`, `buck_5v`, `buck_motor`, `esp32`, `tb6612`, `imu`, `pi5`, `pi5_cooler`, `camera`, `lidar`, `ultrasonic`, `payload`.

## 4. Shared interface file: `out/robot_params.json` (written by the CAD engineer)
```json
{
  "version": "v2-rN", "frame": "REP-103 base_footprint, metres, kg",
  "variant": {"lidar": "ld19", "motor": "...", "battery": "blue_3s_70x50x15"},
  "total_mass": 0.0, "com": [x, y, z],
  "links": {"<label>": {"mass": 0.0, "com": [x,y,z], "inertia": [ixx, iyy, izz, ixy, ixz, iyz], "bbox_min": [..], "bbox_max": [..], "mass_source": "datasheet|measured|assumed|cad-volume×density"}},
  "wheels": {"fl": {"pos": [x,y,z], "radius": 0.0325, "width": 0.0}, "fr": {}, "rl": {}, "rr": {}},
  "track_width": 0.0, "wheelbase": 0.0,
  "sensors": {"laser": {"pos": [..], "rpy": [..]}, "camera": {}, "imu": {}, "ultrasonic": {}},
  "footprint": [[x,y], ...], "lidar_scan_plane_z": 0.0
}
```
Until it exists, the physics and sim engineers use v1 numbers (converted) and say so.

## 5. Ownership in `cad/scout-v2/`
| Folder | Owner | Holds |
|---|---|---|
| `src/`, `checks/`, `out/`, `shots/`, `assembly/`, `vendor/` | CAD engineer | build123d models, checks, exports, snapshots, assembly-step images, vendor STEPs |
| `analysis/` | Physics/electrical engineer | Python calcs (`*.py`), `results.json`, plots, report |
| `urdf/`, `sim/` | Simulation engineer | xacro/URDF, MJCF, scenario scripts, results, videos |
| `docs/` | everyone writes `docs/<role>-round<N>.md`; Argos writes the final docs | |
| `v1-reference/` | read-only | the v1 layout sources and README |

**Nobody commits or pushes, and nobody edits files outside their own folders** (Argos integrates).
No purchases, no posts.

## 6. Tools
- CAD: `~/.local/bin/cadgen +0.8.4` (engine already running); load skill `cad`; vendor parts via skill `step-parts`. Real Pi 5 model: `vendor/raspberry_pi_5.xbf` (+ `.step`).
- Python for analysis/sim: `~/.venvs/scout-sim/bin/python` (mujoco 3.17 with offscreen rendering, numpy, scipy, matplotlib, imageio + ffmpeg, trimesh).
- URDF validation: `ssh ghorman 'source /opt/ros/jazzy/setup.bash && check_urdf /tmp/x.urdf'` (the Pi has ROS 2 Jazzy, no Gazebo). Load skill `urdf`.
- Screenshots of HTML: headless Chrome at `/Applications/Google Chrome.app/Contents/MacOS/Google Chrome` (`--use-angle=swiftshader --enable-unsafe-swiftshader` for WebGL).
- The terminal tool refuses foreground servers: background them and stop them when done.
