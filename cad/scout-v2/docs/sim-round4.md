# Scout v2 — simulation round 4 (CAD v2-r3 params, Hiwonder at the 8.4 V cap)

**Author:** simulation engineer (agent) · **Date:** 2026-11-02 · **Branch:** `design/scout-mech-v2` (not committed) · nothing bought.
**Inputs:** `out/robot_params.json` v2-r3 (0.998 kg empty, 1.298 kg with the Ø75 × 80 mm 300 g cup at x = −50 mm; C1, Hiwonder 1:42 ×4, MDD3A, D36V50F5).
Round 1 used the v1 fallback (0.80 kg, payload box at x = −83 mm), so its numbers were ~20 % optimistic on mass and wrong on the load split.

## 1. What changed in the tooling

| File | Change |
|---|---|
| `sim/scout_params.py` | Reads only `out/robot_params.json` (no v1 fallback). Payload = the CAD cup (cylinder, own inertia). `cable_*` links stay in the lumped mass/inertia but get no geometry. |
| `sim/gen_mjcf.py` | Default motor `hiwonder_1_42_8v4`. Payload as a cylinder. `payload="free"` puts the cup on its own freejoint inside the 4-post fence (`fence=False` disables the posts). `threshold_mu` sets the edge friction. |
| `sim/motor_profiles.json` | Added `hiwonder_1_42_8v4` (0.1648 N·m, 22.56 rad/s, 1.68 A stall at 8.4 V, from the listing via the physics-r2 interpretation). Dropped the FIT0450 profile. |
| `sim/scenarios.py` | Brake = MDD3A LOW/LOW. Spin video is now the closed-loop gyro run. Threshold: a crawl video, a stall-torque sweep and a μ sweep (with payload and empty). New scenario **f**: free-cup retention. Gyro integrated at 1 kHz (r1 sampled at 100 Hz, which added ~2°/10 m of aliasing). |
| `sim/compare.py` (new) | Writes the old-vs-new table into `results.json → comparison`. r1 is read from git (`fc9355d`). |
| `urdf/gen_description.py` | Wheel limits come from the Hiwonder profile. Writes `scout.urdf`, `scout.urdf.xacro` and **`scout_payload.urdf`** (new, payload cylinder). |

Regenerate everything with:

```
gen_description.py → gen_mjcf.py (+ --payload --threshold 0.3 --floor hard_floor --out sim/scout_payload_threshold.xml)
→ scenarios.py → scenarios.py --motor tt_1_48_6v --no-video --out sim/results_tt_1_48_6v.json → compare.py → analysis/run_all.py
```

## 2. URDF check (REQ-SCOUT-011)

- `check_urdf` on ghorman (ROS 2 Jazzy) passes for `scout.urdf` and `scout_payload.urdf`. The xacro (default, `payload:=true`, `use_ros2_control:=true`) expanded with PyPI `xacro` also passes. The Pi has no `xacro` installed.
- Forward kinematics of every sensor frame against `robot_params.sensors` gives **0.0 m position error and ≤ 4e-6 rotation error**:
  - laser (0.055, 0, 0.1489), rpy 0.
  - camera (0.1105, 0, 0.0796), pitched **15.0° down** (optical axis (0.966, 0, −0.259)).
  - camera_optical = camera · (−π/2, 0, −π/2), equal to the CAD `camera_optical`.
  - imu (−0.024, 0, 0.0460), **roll π** (z axis = −Z).
  - ultrasonic (0.1169, 0, 0.0430).
- Mass 0.9981 kg (1.2981 kg with payload), the same as CAD.

## 3. Results: old vs new (`sim/results.json → comparison`)

| Metric | r1 (v1 fallback, TT 6 V) | r4 TT 6 V | **r4 Hiwonder 8.4 V** |
|---|---|---|---|
| Straight 0.3 m/s, carpet + payload: speed / torque % stall / A per motor | 0.300 / 16 % / 0.38 | 0.300 / 17 % / 0.39 | **0.300 / 9.5 % / 0.25** |
| Full-duty start + payload: front wheel lift | yes | no | **yes** (5.7 m/s² vs 5.8 lift limit) |
| Spin, open loop, carpet / hard (rad/s) | 0.59 / 0.51 | 0.72 / 0.61 | **0.70 / 0.59** |
| Spin, gyro loop, carpet (rad/s, torque % stall) | 1.00, 42 % | 1.00, 45 % | **1.00, 22 %** |
| Effective track ratio, carpet / hard | 1.70 / 1.93 | 1.38 / 1.59 | **1.43 / 1.74** |
| E-stop, brake LOW/LOW + payload, hard / carpet (s) | 0.187 / 0.163 | 0.205 / 0.177 | **0.127 / 0.119** (14 mm) |
| E-stop, active PI to zero, hard: wheel lift | no | no | **yes** (7.3 m/s²) |
| 10 mm threshold, crawl 0.05 m/s + payload | **fails** | climbs (0.058 N·m) | **climbs (0.056 N·m)** |
| Threshold at 0.15 / 0.3 m/s + payload | ok / ok (13.8° pitch) | ok / ok (7.3°) | **ok / ok (8.5°)** |
| Crawl + payload: lowest stall torque that climbs | none of 0.079–0.110 | 0.0785 | **0.0785** |
| Crawl: lowest μ (floor and edge) that climbs, payload / empty | — | 0.4 / 0.5 | **0.4 / 0.5** |
| 2 m square, wheel distance + gyro yaw (°/10 m) | 2.7 | 2.8 | **2.8** |
| 2 m square, wheel-only with calibrated track (°/10 m) | 3.1 | 6.3 | 8.6 (fails) |
| Free cup inside the fence, μ 0.3: max slide | — | 1.3 mm | **1.4 mm**, tilt < 0.3° |
| Free cup without the fence, μ 0.3: max slide | — | 3.8 mm | 5.4 mm per event |

How to read it:

1. **The r1 threshold failure came from the params, not the motor.** With the cup at x = −50 mm (49/51 load split), a crawl climb needs 0.056–0.058 N·m peak. Even the stock TT's printed 0.0785 N·m is enough, and every Hiwonder interpretation (0.118 / 0.134 / 0.165 N·m) is too. The physics-r2 finding #4 gap (sim 0.11 vs quasi-static 0.06) is closed: the sim is now within ~15 % of the quasi-static 0.048 N·m.
2. **The threshold is traction-limited.** The sim needs μ ≥ 0.4 with payload and ≥ 0.5 empty (the empty robot is 67 % front-loaded). The quasi-static square-edge model says 0.48 and 0.62. The sim edge has a 1.5 mm radius (assumed), which makes it more forgiving. At 0.15 m/s it climbs in every case.
3. **The Hiwonder at 8.4 V has torque to spare.** It runs at 10 % of stall when cruising and 22 % when spinning (SF ≈ 4). That same torque lifts wheels on a full-duty start or a hard PI stop, so firmware accel/decel ramps are mandatory, not optional.
4. **The watchdog must short-brake (LOW/LOW), within ≤ 170 ms of the last /cmd_vel.** The brake takes 0.13 s, which leaves 0.17 s of the 0.3 s REQ-002 budget.
5. **Wheel-only odometry is worse than in r1, so the EKF must take yaw from the gyro.** The faster spin calibration slips more. Gyro yaw gives 2.8°/10 m, but that assumes the 0.05°/s residual bias after ZUPT.

## 4. Artefacts

- Plots: `sim/plots/{a_straight,b_spin,c_estop,d_threshold,e_odometry,f_payload_retention}.png` (Hiwonder) and `sim/plots/tt_1_48_6v/` (TT).
- Videos (`sim/videos/`, overwritten):
  - `spin_in_place_carpet.mp4` (gyro loop, 1 rad/s)
  - `emergency_stop_payload.mp4` (brake LOW/LOW, hard floor)
  - `threshold_climb_crawl_payload.mp4` (new, 0.05 m/s)
  - `threshold_climb_0p3_payload.mp4`
- Models: `sim/scout.xml` (carpet, empty) and `sim/scout_payload_threshold.xml`.
- Results: `sim/results.json` (design) and `sim/results_tt_1_48_6v.json` (comparison).
- Removed: `sim/params_v1_fallback.json`, `sim/make_fallback_params.py`, `sim/results_fit0450.json`, `sim/plots/dfrobot_fit0450_6v/`, `analysis/round1_unfinished/`.

## 5. Limits (still assumed, not measured)

- Floor μ and Crr. The threshold edge profile (10 × 40 mm, 1.5 mm radius).
- Hiwonder torque/speed. The listing gives no reference voltage; the sweep shows the climb doesn't depend on it, but top speed and current do.
- Rigid tyres (MuJoCo soft contact). Gyro bias 0.05°/s after ZUPT.
- Cup–deck μ 0.3–0.5.
- No sensor simulation: LiDAR, camera and ultrasonic are covered by geometry in `analysis/a8`.
