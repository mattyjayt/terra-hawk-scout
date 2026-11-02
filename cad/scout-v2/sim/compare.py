#!/usr/bin/env python3
"""Fold the round-4 comparison into sim/results.json["comparison"]:
  r1    = sim round 1 (v1 fallback params 0.80 kg, stock TT @ 6 V), read from git commit fc9355d
  r4_tt = round 4, CAD v2-r3 params (0.998 / 1.298 kg), stock TT 1:48 @ 6 V  (sim/results_tt_1_48_6v.json)
  r4_hw = round 4, CAD v2-r3 params, Hiwonder 1:42 @ 8.4 V PWM cap        (sim/results.json, the design)
Usage: python sim/compare.py   (after scenarios.py has written both result files)
"""
import json
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
R1_REF = "fc9355d:cad/scout-v2/sim/results.json"

ROWS = [  # (label, path into the results dict)
    ("straight 0.3 m/s carpet + payload: achieved m/s", "a_straight.carpet_payload.achieved_mps"),
    ("straight carpet + payload: steady torque % stall", "a_straight.carpet_payload.steady_torque_pct_stall"),
    ("straight carpet + payload: steady A/motor", "a_straight.carpet_payload.steady_current_A_per_motor"),
    ("full-duty start + payload (hard): front wheel lift", "a_straight.full_duty_step_payload_hard_floor.front_wheel_lift_20ms"),
    ("spin open loop carpet: rad/s", "b_spin.open_loop_nominal_kinematics.carpet.achieved_yaw_rate"),
    ("spin open loop hard: rad/s", "b_spin.open_loop_nominal_kinematics.hard_floor.achieved_yaw_rate"),
    ("spin gyro loop carpet: rad/s", "b_spin.closed_loop_gyro.carpet.achieved_yaw_rate"),
    ("spin gyro loop carpet: torque % stall", "b_spin.closed_loop_gyro.carpet.torque_pct_stall"),
    ("spin effective track ratio carpet / hard", ("b_spin.closed_loop_gyro.carpet.effective_track_ratio",
                                                  "b_spin.closed_loop_gyro.hard_floor.effective_track_ratio")),
    ("e-stop brake LL, hard + payload: s", "c_estop_payload.hard_floor_brake.stop_time_s"),
    ("e-stop brake LL, carpet + payload: s", "c_estop_payload.carpet_brake.stop_time_s"),
    ("e-stop brake LL, hard: mm", "c_estop_payload.hard_floor_brake.stop_distance_m"),
    ("e-stop coast, hard: s", "c_estop_payload.hard_floor_coast.stop_time_s"),
    ("e-stop active (PI to 0), hard: wheel lift", "c_estop_payload.hard_floor_active.wheel_lift_20ms"),
    ("threshold 10 mm crawl 0.05 + payload: climbs", "d_threshold.v0.05_payload.success"),
    ("threshold crawl + payload: peak N m", "d_threshold.v0.05_payload.peak_wheel_torque_Nm"),
    ("threshold 0.15 + payload: climbs", "d_threshold.v0.15_payload.success"),
    ("threshold 0.3 + payload: climbs / max pitch deg", ("d_threshold.v0.3_payload.success", "d_threshold.v0.3_payload.max_pitch_deg")),
    ("odometry 2 m square: wheel+gyro deg/10 m", "e_odometry_square.wheel_distance_plus_gyro_yaw.heading_drift_deg_per_10m"),
    ("odometry: calibrated wheel-only deg/10 m", "e_odometry_square.wheel_odom_calibrated_track.heading_drift_deg_per_10m"),
    ("payload retention, fence, mu 0.3: max slide mm", "f_payload_retention.fence_mu0.3_estop_active.max_slide_mm"),
]


def get(d, path):
    if isinstance(path, tuple):
        return [get(d, p) for p in path]
    keys, cur = path.split("."), d
    i = 0
    while i < len(keys):   # keys may contain dots (v0.05_payload): greedily join
        for j in range(len(keys), i, -1):
            k = ".".join(keys[i:j])
            if isinstance(cur, dict) and k in cur:
                cur, i = cur[k], j
                break
        else:
            return None
    if path.endswith("stop_distance_m") and isinstance(cur, float):
        return round(cur * 1000, 1)
    return cur


def main():
    root = HERE.parent
    r1 = json.loads(subprocess.run(["git", "show", R1_REF], cwd=root, capture_output=True, text=True, check=True).stdout)
    hw = json.loads((HERE / "results.json").read_text())
    tt = json.loads((HERE / "results_tt_1_48_6v.json").read_text())
    rows = [{"metric": lab, "r1_v1fallback_tt6v": get(r1, p), "r4_r3params_tt6v": get(tt, p), "r4_r3params_hiwonder8v4": get(hw, p)}
            for lab, p in ROWS]
    # r1 had no closed-loop-free payload test and a voltage (not torque) sweep
    rows.append({"metric": "crawl + payload: min stall torque that climbs (sweep) N m",
                 "r1_v1fallback_tt6v": "none of 0.0785/0.0968/0.1099 climbed",
                 "r4_r3params_tt6v": min((r["stall_torque_Nm"] for r in tt["d_threshold"]["stall_torque_sweep_crawl_payload"] if r["success"]), default=None),
                 "r4_r3params_hiwonder8v4": min((r["stall_torque_Nm"] for r in hw["d_threshold"]["stall_torque_sweep_crawl_payload"] if r["success"]), default=None)})
    rows.append({"metric": "crawl + payload: lowest mu (floor and edge) that climbs", "r1_v1fallback_tt6v": "not run",
                 "r4_r3params_tt6v": min((r["mu_floor_and_edge"] for r in tt["d_threshold"]["mu_sweep_crawl_payload"] if r["success"]), default=None),
                 "r4_r3params_hiwonder8v4": min((r["mu_floor_and_edge"] for r in hw["d_threshold"]["mu_sweep_crawl_payload"] if r["success"]), default=None)})
    rows.append({"metric": "crawl EMPTY: lowest mu (floor and edge) that climbs", "r1_v1fallback_tt6v": "not run",
                 "r4_r3params_tt6v": min((r["mu_floor_and_edge"] for r in tt["d_threshold"]["mu_sweep_crawl_empty"] if r["success"]), default=None),
                 "r4_r3params_hiwonder8v4": min((r["mu_floor_and_edge"] for r in hw["d_threshold"]["mu_sweep_crawl_empty"] if r["success"]), default=None)})
    hw["comparison"] = {"columns": {"r1_v1fallback_tt6v": f"sim r1 ({R1_REF}): v1 fallback 0.80 kg, payload box at x=-83 mm, TT 1:48 @ 6 V",
                                    "r4_r3params_tt6v": "round 4: out/robot_params.json v2-r3 (0.998 kg, cup at x=-50 mm), TT 1:48 @ 6 V",
                                    "r4_r3params_hiwonder8v4": "round 4: v2-r3, Hiwonder 1:42 @ 8.4 V PWM cap (design)"},
                        "rows": rows}
    (HERE / "results.json").write_text(json.dumps(hw, indent=1, default=float))
    w = max(len(r["metric"]) for r in rows)
    for r in rows:
        print(f"{r['metric']:{w}s} | {r['r1_v1fallback_tt6v']!s:22s} | {r['r4_r3params_tt6v']!s:16s} | {r['r4_r3params_hiwonder8v4']!s}")


if __name__ == "__main__":
    main()
