"""6. Stability: static tip angles, accel/decel before wheel lift or sliding, threshold pitch,
and stopping distance from 0.3 m/s vs the 15 cm safety stop (REQ-SCOUT-009) and the
300 ms watchdog (REQ-SCOUT-002)."""
import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import ENV, G, HERE, MOTORS, load_results, r3, save_section
from loads import AX_F, AX_R, L, R, w_from_v

Y_C = 0.062           # wheel centre plane (contact), conservative (tyre outer edge is 75 mm)


def main():
    cases = load_results()["mass"]["cases"]
    res = {"cases": {}}
    phi = math.asin(ENV["threshold_h"] / L)
    for name, c in cases.items():
        x, y, z = [v / 1000 for v in c["com_mm"]]
        d = {
            "tip_front_deg": math.degrees(math.atan2(AX_F - x, z)),
            "tip_rear_deg": math.degrees(math.atan2(x - AX_R, z)),
            "tip_side_deg": math.degrees(math.atan2(Y_C - abs(y), z)),
            "a_rear_lift_braking_m_s2": G * (AX_F - x) / z,       # braking forward: rear wheels lift
            "a_front_lift_accel_m_s2": G * (x - AX_R) / z,        # accelerating forward: front wheels lift
            "a_lateral_tip_m_s2": G * (Y_C - abs(y)) / z,
            "front_lift_margin_on_threshold_m_s2": G * ((x - AX_R) * math.cos(phi) - z * math.sin(phi)) / z,
        }
        for s in ("carpet", "hard"):
            d[f"a_slide_{s}_m_s2"] = ENV["mu"][s] * G
        d["braking_limited_by"] = "sliding" if ENV["mu"]["hard"] * G < d["a_rear_lift_braking_m_s2"] else "wheel lift (nose dip)"
        res["cases"][name] = d

    # stop distance
    v = ENV["v_cruise"]
    lat = {"lidar_scan_worst_10Hz": 0.10, "driver+filter+ROS->micro-ROS": 0.05, "esp32_loop_50Hz": 0.02, "motor_current_rise": 0.005}
    t_lat = sum(lat.values())
    mot = MOTORS["hiwonder"]
    m = cases["proposed_c1_pay300"]["mass_kg"]
    # reverse-voltage braking available: stall torque at 9 V plus back-EMF help, per wheel -> force
    a_motor = 4 * mot.ts(9.0) / R / m
    stop = {}
    for a_cmd in (1.0, 1.5, 2.0, 3.0):
        a = min(a_cmd, ENV["mu"]["carpet"] * G, a_motor)
        stop[f"decel_{a_cmd}"] = {"latency_m": v * t_lat, "braking_m": v * v / (2 * a), "total_m": v * t_lat + v * v / (2 * a)}
    res["stop"] = {"latency_s": lat, "t_latency_total_s": t_lat, "a_motor_reverse_max_m_s2": a_motor,
                   "cases": stop,
                   "budget_vs_15cm_with_C1_range_error_0.03": 0.15 - stop["decel_1.5"]["total_m"] - 0.03,
                   "budget_vs_15cm_with_LD19_range_error_0.045": 0.15 - stop["decel_1.5"]["total_m"] - 0.045}
    # watchdog: 300 ms + either coast (rolling resistance only) or active brake
    wd = {}
    for s in ("carpet", "hard"):
        a_coast = ENV["crr"][s] * G
        wd[s] = {"coast_m": v * 0.3 + v * v / (2 * a_coast), "brake_1.5_m": v * 0.3 + v * v / 3.0}
    res["watchdog_stop"] = wd
    res["recommendation"] = ("firmware decel ramp 1.5 m/s^2 (below every lift/slide limit), watchdog must BRAKE "
                             "(MDD3A inputs LOW/LOW) not coast; trigger the safety stop at <= 0.20 m from the bumper "
                             "to keep 15 cm after latency + range error")
    save_section("stability", r3(res, 3))

    # plot: limits per case
    fig, ax = plt.subplots(figsize=(9, 4.5))
    names = [n for n in cases if "c1" in n]
    xs = np.arange(len(names))
    ax.bar(xs - 0.2, [res["cases"][n]["a_rear_lift_braking_m_s2"] for n in names], 0.2, label="braking: rear-wheel lift")
    ax.bar(xs, [res["cases"][n]["a_front_lift_accel_m_s2"] for n in names], 0.2, label="accel: front-wheel lift")
    ax.bar(xs + 0.2, [res["cases"][n]["front_lift_margin_on_threshold_m_s2"] for n in names], 0.2, label="accel on 10 mm step")
    ax.axhline(ENV["mu"]["hard"] * G, color="k", ls="--", label="slide, hard floor mu 0.7")
    ax.axhline(0.6, color="g", ls=":", label="required accel 0.6")
    ax.axhline(1.5, color="orange", ls=":", label="recommended decel cap 1.5")
    ax.set_xticks(xs); ax.set_xticklabels(names, rotation=15, fontsize=8); ax.set_ylabel("m/s^2"); ax.legend(fontsize=7)
    ax.set_title("Acceleration limits before wheel lift (C1 LiDAR)"); fig.tight_layout()
    fig.savefig(HERE / "a6_stability.png", dpi=110)
    print(json.dumps(r3(res, 3), indent=1))


if __name__ == "__main__":
    main()
