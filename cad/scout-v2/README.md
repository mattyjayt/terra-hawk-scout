# Scout v2 — detailed mechanical, electrical and physics design

**Status:** design complete for review (rounds 1–4, 1–2 Nov 2026). Nothing bought yet.
**Start here:** open [`guide/index.html`](guide/index.html) in Chrome (double-click). It has the 3D model (assembled / exploded / animated assembly), the 9 build steps with checks, the wiring and power diagrams, the how-it-works figure, the sim videos, the BOM and the drilling table.

## Verdict (from `docs/physics-final.md`)
The design fits, carries the load and is power-consistent on paper.
- **Pass:** 0.3 m/s; 1 rad/s with the gyro yaw loop; e-stop 0.13 s; obstacle stop margin 37 mm; URDF frames = CAD; heading drift 2.8°/10 m (wheel distance + gyro yaw); stability margins ≥ 4×; LiDAR 6.4 mm above every part; camera sees the floor 105 mm ahead; cup held by the fence.
- **Conditional:** 5 V at the Pi (M7, M1), the 10 mm threshold (floor friction ≥ 0.4–0.6: M3/M4), runtime 31–68 min (pack ≥ 14 Wh: M1).
- **Firmware must:** ramp acceleration and deceleration, brake LOW/LOW on a ≤ 170 ms watchdog, cap the PWM at 8.4 V, gate LiDAR scans on IMU pitch, calibrate the gyro scale and update its bias at every stop.

## Design
| | Choice |
|---|---|
| Chassis | Elegoo Smart Robot Car V4.0 plates (drilled, 29 holes), 39.7 mm deck gap |
| Drive | 4 × Hiwonder TT Hall-encoder 1:42, Cytron MDD3A on the 3S pack, PWM capped at 8.4 V |
| Power | 3S pack → XT30 → 7.5 A fuse → 10 A DC rocker → MDD3A + Pololu D36V50F5 (5 V/5 A) → Pi 5 header |
| Compute | Pi 5 8 GB + Active Cooler; classic ESP32 (micro-ROS) on Pi UART2 — *superseded: UNO R4 + Elegoo shield on USB (ADR-0006)* |
| Sensors | RPLIDAR C1 on Pi UART0 (scan plane 149 mm), Camera Module 3 at 15° down, MPU-6050 at the centre of rotation, HC-SR04 |
| Mass | 0.998 kg empty, 1.298 kg with the 300 g cup (49/51 front/rear) |

## Buy (from `docs/physics-final.md`, ~€246)
- **Now (~€203):** RPLIDAR C1, Cytron MDD3A, Pololu D36V50F5, 10 A DC rocker, fuse holder + 7.5 A fuses, XT30/JST connectors, capacitors, M3/M2.5 standoff kits, proto board, silicone wire.
- **After measuring (~€43):** Hiwonder motors 4 + 1 spare (after M5), camera tilt mount, HC-SR04 bracket, 3S buzzer only if the pack has no BMS.
- **Don't:** LD19, FIT0450, Yahboom, motor buck, XL4015/UBEC, TB6612 carrier, LiDAR USB adapter.

## Measure first (ranked)
M5 motor bracket fit · M1 battery mass/markings/BMS · M4 floor friction · M3 real threshold · M6 part masses · M7 on arrival (buck voltage, encoder counts) · M2 stock TT torque · M8 2S box.

## Files
| Path | What |
|---|---|
| `BRIEF.md` | the shared engineering brief (inputs, frame, conventions) |
| `src/` | build123d models (`scout_v2.py`, `dims.py` with every dimension's source, `variants.py`, `cables.py`, `exports.py`) |
| `checks/` | fit check and its results; `export_params.py` writes `out/robot_params.json` |
| `out/` | GLB (static, exploded, animated assembly), STL per part, 3MF, STEP (git-ignored), `robot_params.json` |
| `assembly/`, `shots/` | step pictures and overview shots |
| `urdf/`, `sim/` | URDF/xacro and MuJoCo model generated from `robot_params.json`; scenarios, results, videos |
| `analysis/` | power, stability, odometry, sensor and motor calculations |
| `docs/` | `physics-round2.md` (27-finding review), `cad-round3.md` (BOM, drilling table), `sim-round4.md`, `physics-final.md` (verdict, purchase list), `diagrams/` (archify), `figures/` (anatomy) |
| `guide/` | the assembly guide page (rebuild GLB copies: `python3 -I guide/vendor/make_glb_js.py`) |

Rebuild: `cadgen +0.8.4 run src/exports.py` → `cadgen +0.8.4 run checks/fit_check_v2.py scout_v2` → `~/.venvs/scout-sim/bin/python urdf/gen_description.py` and `sim/gen_mjcf.py` → `sim/scenarios.py`.
