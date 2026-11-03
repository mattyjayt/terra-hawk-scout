"""7. Odometry + IMU (round 2): encoder resolution needed (velocity control at 50 Hz, position for
Nav2 10 cm arrival), candidate encoders, gyro error budget for REQ-SCOUT-006 (5 deg / 10 m),
IMU placement (vibration, heat, magnetics) and ZUPT.
"""
import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import ENV, HERE, MOTORS, ROOT, SRC, r3, save_section
from loads import B, R

CIRC = 2 * math.pi * R
F_S = 50.0                    # REQ-SCOUT-003
N_MIN = 4                     # A: >= 4 counts per sample -> <= 25 % velocity quantisation (PID usable)
SIM = json.loads((ROOT / "sim" / "results.json").read_text()) if (ROOT / "sim" / "results.json").exists() else {}

# MPU-6050 product spec PS-MPU-6000A-00 rev 3.4 (SRC mpu6050_ps): gyro
MPU = {"zro_initial_dps": 20.0, "zro_over_temp_dps": 20.0, "temp_range_C": 125.0, "noise_dps_rtHz": 0.005,
       "sens_tol_pct": 3.0, "nonlin_pct": 0.2, "cross_axis_pct": 2.0, "dlpf_hz": [5, 10, 21, 44, 94, 184, 260]}


def main():
    res = {"sample_rate_Hz": F_S, "wheel_circumference_m": CIRC}
    t10 = 10 / ENV["v_cruise"]
    # ---- encoder requirement
    v_min = {"nav2_creep_0.02": 0.02, "threshold_crawl_0.05": 0.05,
             "pivot_wheel_1rad_s": 1.0 * B * 1.70 / 2, "cruise_0.3": 0.3}
    need = {k: N_MIN * CIRC / (v / F_S) for k, v in v_min.items()}
    res["cpr_required_x4"] = {"rule": f">= {N_MIN} counts per {1000 / F_S:.0f} ms sample", "per_speed": need,
                              "position_for_nav2_10cm": "any encoder > ~20 cpr (10 mm/count) suffices for position; "
                              "Nav2 arrival error is dominated by slip and map/AMCL error, not quantisation",
                              "verdict": "~1000 counts/rev (x4) for PID down to 0.05 m/s at 50 Hz; below that use edge "
                                         "timing (period measurement) or a 100 ms window"}
    cands = {}
    variants = {"hiwonder_claimed_252ppr": 252 * 4, "hiwonder_buyer_measured_150ppr": 150 * 4,
                "fit0450_960_per_ch": 960 * 4, "fit0450_sim_1920": 1920, "yahboom_13line_1:45": 13 * 45 * 4,
                "stock_tt_no_encoder": 0}
    for k, cpr in variants.items():
        if not cpr:
            cands[k] = {"verdict": "no encoder -> REQ-SCOUT-003/004 impossible"}
            continue
        d = {"counts_per_rev_x4": cpr, "mm_per_count": 1000 * CIRC / cpr}
        for lbl, v in v_min.items():
            d[f"counts_per_sample_{lbl}"] = v / CIRC * cpr / F_S
        d["max_edge_rate_kHz_250rpm"] = cpr * 250 / 60 / 1000
        d["meets_1000cpr"] = cpr >= 0.98 * need["threshold_crawl_0.05"]
        cands[k] = d
    res["candidates"] = cands
    res["esp32_counting"] = ("use the PCNT peripheral (classic ESP32 has 8 units; 4 needed, x4 decoding with both "
                             "channels), glitch filter ~1 us; edge rate <= 16 kHz/motor is trivial for PCNT, a "
                             "GPIO-ISR design would cost ~4 x 16k = 64k interrupts/s -> don't")

    # ---- gyro budget (REQ-SCOUT-006: < 5 deg heading drift per 10 m on carpet)
    turned = {"straight_10m": 0.0, "square_4x90": 360.0, "patrol_with_U_turns": 720.0}
    budget = {"t_10m_s": t10, "bias_max_dps_if_all_budget": 5.0 / t10,
              "ARW_deg_over_10m": MPU["noise_dps_rtHz"] * math.sqrt(t10),
              "scale_factor_error_deg": {k: MPU["sens_tol_pct"] / 100 * v for k, v in turned.items()},
              "scale_max_pct_for_5deg_square": 5.0 / 360 * 100,
              "zro_temp_coeff_worst_dps_per_C": (MPU["zro_over_temp_dps"] * 2) / MPU["temp_range_C"],
              "deg_C_drift_that_uses_whole_budget": (5.0 / t10) / ((MPU["zro_over_temp_dps"] * 2) / MPU["temp_range_C"]),
              "sim_gyro_fused_drift_deg_per_10m": SIM.get("e_odometry_square", {}).get("wheel_distance_plus_gyro_yaw", {}).get("heading_drift_deg_per_10m"),
              "sim_wheel_only_drift_deg_per_10m": SIM.get("e_odometry_square", {}).get("wheel_odom_nominal_track", {}).get("heading_drift_deg_per_10m"),
              "sim_wheel_calibrated_on_wrong_floor": SIM.get("e_odometry_square", {}).get("wheel_odom_carpet_calibration_on_hard_floor", {}).get("heading_drift_deg_per_10m"),
              }
    budget["allocation"] = {"bias_after_ZUPT_dps": 0.05, "bias_deg": 0.05 * t10,
                            "scale_after_cal_pct": 0.5, "scale_deg_square": 0.005 * 360,
                            "ARW_deg": budget["ARW_deg_over_10m"]}
    budget["allocation"]["total_deg_rss"] = math.sqrt(budget["allocation"]["bias_deg"] ** 2 +
                                                      budget["allocation"]["scale_deg_square"] ** 2 +
                                                      budget["allocation"]["ARW_deg"] ** 2)
    budget["verdict"] = ("feasible ONLY with (1) bias re-estimated at every stop (ZUPT), (2) gyro scale factor "
                         "calibrated to <= 1 % (datasheet tolerance 3 % alone gives 10.8 deg on a 4x90 deg square), "
                         "(3) wheel yaw excluded from the EKF (sim: 184 deg/10 m wheel-only; 55 deg/10 m when the slip "
                         "calibration is for the wrong floor)")
    res["gyro"] = budget
    res["imu_placement"] = {
        "cad_r3": "GY-521 hung chip-down on 2 x M3 x 10 nylon standoffs under the top plate, die at (-24, 0, 46) mm, "
                  "24 mm from the CoR, rpy (pi, 0, 0); round-4 URDF imu_joint = robot_params sensors.imu (verified)",
        "issues": ["CLOSED (r4): URDF imu_joint now carries roll pi and the CAD position; the driver must NOT flip axes again",
                   "CLOSED (r3): IMU moved from under the Pi to x = -24 mm, away from the CPU heat; ZUPT still mandatory "
                   "(0.32 dps/C worst coefficient)",
                   "LiDAR motor (C1 600 rpm = 10 Hz) and gear mesh vibration couple through the brass towers: set "
                   "MPU DLPF to 44 Hz, sample >= 200 Hz on the ESP32, average to 50-100 Hz; nylon standoffs "
                   "(already in CAD) help",
                   "magnetics irrelevant: MPU-6050 has no magnetometer; motor magnets do not affect a MEMS gyro"],
        "zupt": "when |cmd_vel| = 0 and all encoder counts unchanged for 0.5 s: bias <- EMA(gyro, tau 5 s); publish "
                "with covariance; 40 % stationary duty in the mission profile gives frequent updates",
    }
    save_section("odometry", r3(res, 4))

    v = np.linspace(0.01, 0.4, 200)
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 4.6))
    for k, d in cands.items():
        if "counts_per_rev_x4" in d:
            a.plot(v, 100 * (CIRC / d["counts_per_rev_x4"] * F_S) / v, label=f"{k} ({d['counts_per_rev_x4']} cpr)")
    a.axhline(100 / N_MIN, color="r", ls="--", label="25 % (4 counts/sample)")
    for k, vv in v_min.items():
        a.axvline(vv, color="0.6", ls=":")
    a.set_yscale("log"); a.set_xlabel("wheel surface speed [m/s]"); a.set_ylabel("1-count velocity step at 50 Hz [%]")
    a.grid(alpha=.3, which="both"); a.legend(fontsize=7); a.set_title("Encoder velocity quantisation (x4)")
    lbl = ["bias 0.15 dps\n(no ZUPT)", "bias 0.05\n(ZUPT)", "scale 3 %\nsquare", "scale 0.5 %\nsquare", "ARW"]
    val = [0.15 * t10, 0.05 * t10, 0.03 * 360, 0.005 * 360, budget["ARW_deg_over_10m"]]
    b.bar(lbl, val, color=["C3", "C2", "C3", "C2", "C2"]); b.axhline(5, color="k", ls="--", label="REQ 5 deg / 10 m")
    b.set_ylabel("heading error over 10 m [deg]"); b.set_title("MPU-6050 heading budget, 33 s / 10 m"); b.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(HERE / "a7_odometry.png", dpi=110)
    print(json.dumps(r3({k: res[k] for k in ("cpr_required_x4", "gyro")}, 3), indent=1))
    for k, d in cands.items():
        print(k, r3(d, 3))


if __name__ == "__main__":
    main()
