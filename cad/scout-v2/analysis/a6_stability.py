"""6. Stability (round 4: CAD v2-r3 mass cases, see common.cad_cases):
static tip angles, wheel-lift accelerations (accel / brake / lateral), the 10 mm threshold pitch,
a 5 deg ramp, spin, payload retention (cup sliding / tipping on the deck), static CoM limits,
and the stop budget vs REQ-SCOUT-002 / -009.
Rigid-body quasi-static: front axle unloads when a_fwd > g (x_c - x_r) / z_c; rear axle unloads when
a_brake > g (x_f - x_c) / z_c; side tip when a_lat > g (y_w - |y_c|) / z_c. Friction caps |a| at mu g.
"""
import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import ENV, G, HERE, MOTORS, PARAMS_JSON, ROOT, cad_cases, fit_check_raw, payload_x, r3, save_section
from loads import AX_F, AX_R, R

Y_W = 0.062                 # wheel centre plane = contact line (dims.WHEEL_Y; tyre outer edge 75 mm -> conservative)
MU_MAX = 0.8                # sim hard-floor mu (upper bound for available traction -> worst case for lift)
A_REQ = 0.6                 # m/s^2, derived target (0.3 m/s in 0.5 s)
A_DECEL_CAP = 1.5           # m/s^2, recommended firmware decel ramp
SIM = json.loads((ROOT / "sim" / "results.json").read_text()) if (ROOT / "sim" / "results.json").exists() else {}
# payload: seedling cup on the deck, d 75 x h 80 mm (dims.py), 300 g, CoM assumed at half height (A: wet soil)
CUP_R, CUP_H = 0.0375, 0.080
MU_CUP = (0.3, 0.5)         # A: plastic cup on acrylic / anodised plate, dry; measure with a tilt test


def limits(x, y, z, pitch_deg=0.0):
    th = math.radians(pitch_deg)    # + nose up (climbing)
    return {
        "tip_fwd_deg": math.degrees(math.atan2(AX_F - x, z)),
        "tip_back_deg": math.degrees(math.atan2(x - AX_R, z)),
        "tip_side_deg": math.degrees(math.atan2(Y_W - abs(y), z)),
        "a_front_lift_accel": G * ((x - AX_R) * math.cos(th) - z * math.sin(th)) / z,
        "a_rear_lift_brake": G * ((AX_F - x) * math.cos(th) + z * math.sin(th)) / z,
        "a_side_tip": G * (Y_W - abs(y)) / z,
    }


def main():
    cases = cad_cases()
    res = {"assumptions": {"y_wheel_m": Y_W, "mu_max": MU_MAX, "a_req": A_REQ, "a_decel_cap": A_DECEL_CAP,
                           "cup": {"r": CUP_R, "h": CUP_H, "mu_cup_deck": MU_CUP}},
           "cases": {}}
    hi = {c["model"]: c["ground_check"] for c in fit_check_raw()}
    pitch_step = hi.get("scout_v2", {}).get("pitch_with_front_wheels_on_10mm_step_deg", 7.18)
    sim_pitch = SIM.get("d_threshold", {}).get("v0.3_payload", {}).get("max_pitch_deg")
    for cn, c in cases.items():
        x, y, z = c["com_m"]
        flat = limits(x, y, z)
        d = {"com_mm": [x * 1000, y * 1000, z * 1000], "mass_kg": c["mass_kg"], "flat": flat,
             "ramp_5deg_uphill": limits(x, y, z, 5.0), "ramp_5deg_downhill": limits(x, y, z, -5.0),
             "on_10mm_step_front_up": limits(x, y, z, pitch_step),
             "on_10mm_step_rear_up": limits(x, y, z, -pitch_step)}
        d["margins"] = {
            "accel_req_SF": flat["a_front_lift_accel"] / A_REQ,
            "decel_cap_SF": flat["a_rear_lift_brake"] / A_DECEL_CAP,
            "front_lift_possible_at_mu_g": flat["a_front_lift_accel"] < MU_MAX * G,
            "rear_lift_possible_at_mu_g": flat["a_rear_lift_brake"] < MU_MAX * G,
            "rollover_possible_flat": flat["a_side_tip"] < MU_MAX * G,
            "tip_back_margin_vs_sim_dynamic_pitch_deg": (flat["tip_back_deg"] - (sim_pitch or 0)) if sim_pitch else None,
            "ramp5_uphill_accel_SF": d["ramp_5deg_uphill"]["a_front_lift_accel"] / A_REQ,
            "step_front_up_accel_SF": d["on_10mm_step_front_up"]["a_front_lift_accel"] / A_REQ,
        }
        res["cases"][cn] = d

    # payload retention (independent of the robot case)
    cup = {}
    for mu in MU_CUP:
        cup[f"slide_at_mu{mu}_m_s2"] = mu * G
    cup["tip_cup_m_s2"] = G * CUP_R / (CUP_H / 2)
    cup["sim_estop_active_peak_decel"] = SIM.get("c_estop_payload", {}).get("hard_floor_active", {}).get("peak_decel_mps2_20ms")
    cup["sim_full_duty_step_accel"] = SIM.get("a_straight", {}).get("full_duty_step_payload_hard_floor", {}).get("peak_accel_mps2_20ms")
    cup["sim_brake_LL_peak_decel"] = SIM.get("c_estop_payload", {}).get("hard_floor_brake", {}).get("peak_decel_mps2_20ms")
    fr = SIM.get("f_payload_retention", {})
    cup["sim_free_cup_max_slide_mm"] = {"fence": max((v["max_slide_mm"] for k, v in fr.items() if k.startswith("fence_")), default=None),
                                        "no_fence": max((v["max_slide_mm"] for k, v in fr.items() if k.startswith("no_fence_")), default=None)}
    cup["verdict"] = ("an unrestrained cup slides above 2.9-4.9 m/s^2 (mu 0.3-0.5); sim peaks reach that (active stop, full-duty "
                      "start). With the CAD r3 fence (4 x M3 x 20, 2 mm play) the free-cup sim moves <= "
                      f"{cup['sim_free_cup_max_slide_mm']['fence']} mm (no fence: {cup['sim_free_cup_max_slide_mm']['no_fence']} mm per event, "
                      "cumulative). Keep the fence; firmware |a| <= 1.5 m/s^2 except a true e-stop (brake LL).")
    res["payload_retention"] = cup

    # spin: yaw accel to 1 rad/s in 0.2 s; tangential accel on the payload at r = 74 mm (CAD payload x)
    res["spin"] = {"yaw_accel_rad_s2": 5.0, "payload_tangential_m_s2": 5.0 * abs(payload_x()),
                   "centripetal_on_com_1rad_s_m_s2": 1.0 ** 2 * 0.011, "verdict": "negligible for tip; payload ok"}

    # static CoM limits: keep front/rear lift >= 2 x decel cap (3 m/s^2) on flat and >= A_REQ on the 10 mm step
    z = np.linspace(0.04, 0.12, 9)
    a_design = 2 * A_DECEL_CAP
    th = math.radians(pitch_step)
    res["com_limits"] = {
        "rule": f"front-lift and rear-lift accel >= {a_design} m/s^2 on flat, front-lift >= {A_REQ} m/s^2 with the nose 7.2 deg up",
        "x_min_mm_vs_z_mm": {f"{zz*1000:.0f}": 1000 * max(AX_R + zz * a_design / G,
                                                          AX_R + (A_REQ * zz / G + zz * math.sin(th)) / math.cos(th))
                             for zz in z},
        "x_max_mm_vs_z_mm": {f"{zz*1000:.0f}": 1000 * (AX_F - zz * a_design / G) for zz in z},
        "lateral_y_max_mm_at_z65": 1000 * (Y_W - 0.065 * MU_MAX),
    }

    # stop budget: obstacle 15 cm ahead of the front (CAD front x = 117.4 mm, LiDAR at x = 45 mm)
    v = ENV["v_cruise"]
    lat = {"lidar_scan_worst_10Hz": 0.10, "ROS costmap/collision monitor": 0.05, "micro-ROS + ESP32 50 Hz loop": 0.02,
           "driver/motor current reversal": 0.005}
    t_lat = sum(lat.values())
    lidar_x = json.loads(PARAMS_JSON.read_text())["sensors"]["laser"]["pos"][0]
    front_x = max(p[0] for p in json.loads(PARAMS_JSON.read_text())["footprint"])
    stop = {}
    for a in (1.0, 1.5, 3.0):
        stop[f"decel_{a}"] = {"latency_m": v * t_lat, "braking_m": v * v / (2 * a), "total_m": v * t_lat + v * v / (2 * a)}
    res["stop"] = {
        "latency_s": lat, "t_latency_s": t_lat, "cases": stop,
        "lidar_range_to_15cm_zone_m": front_x - lidar_x + 0.15,
        "margin_C1_err30mm_m": 0.15 - stop["decel_1.5"]["total_m"] - 0.030,
        "margin_LD19_err45mm_m": 0.15 - stop["decel_1.5"]["total_m"] - 0.045,
        "LD19_note": f"LD19 datasheet gives NO accuracy below 300 mm; the 15 cm stop zone is {1000*(front_x - lidar_x + 0.15):.0f} mm from the LiDAR",
        "sim_watchdog": {k: SIM.get("c_estop_payload", {}).get(k, {}).get("stop_distance_m")
                         for k in ("hard_floor_active", "hard_floor_brake", "hard_floor_coast", "carpet_coast")},
        "sim_brake_LL_stop_s": {k: SIM.get("c_estop_payload", {}).get(k, {}).get("stop_time_s") for k in ("hard_floor_brake", "carpet_brake")},
    }
    sb = stop["decel_1.5"]["total_m"]
    res["stop"]["verdict"] = (f"stop budget {sb*1000:.0f} mm from 0.3 m/s at 1.5 m/s^2 ({t_lat*1000:.0f} ms latency) -> "
                              f"{res['stop']['margin_C1_err30mm_m']*1000:.0f} mm (C1) margin inside 15 cm. Watchdog must brake (MDD3A LL): "
                              f"sim brake {res['stop']['sim_brake_LL_stop_s']} s vs coast > 0.3 s.")
    save_section("stability", r3(res, 3))

    # plot
    names = [n for n in res["cases"] if n.startswith("cad_")]
    fig, (a, b) = plt.subplots(1, 2, figsize=(14, 5))
    xs = np.arange(len(names))
    a.bar(xs - 0.3, [res["cases"][n]["flat"]["a_front_lift_accel"] for n in names], 0.2, label="accel: front lift (flat)")
    a.bar(xs - 0.1, [res["cases"][n]["on_10mm_step_front_up"]["a_front_lift_accel"] for n in names], 0.2, label="accel: front lift, nose up 7.2 deg")
    a.bar(xs + 0.1, [res["cases"][n]["ramp_5deg_uphill"]["a_front_lift_accel"] for n in names], 0.2, label="accel: front lift, 5 deg uphill")
    a.bar(xs + 0.3, [res["cases"][n]["flat"]["a_rear_lift_brake"] for n in names], 0.2, label="brake: rear lift (flat)")
    a.axhline(MU_MAX * G, color="k", ls="--", label="traction limit mu 0.8")
    a.axhline(A_REQ, color="g", ls=":", label="required accel 0.6")
    a.axhline(A_DECEL_CAP, color="orange", ls=":", label="decel cap 1.5")
    a.axhspan(MU_CUP[0] * G, MU_CUP[1] * G, color="r", alpha=.12, label="unrestrained cup slides")
    a.set_xticks(xs); a.set_xticklabels([n.replace("cad_", "") for n in names], rotation=15, fontsize=8)
    a.set_ylabel("m/s^2"); a.legend(fontsize=7); a.set_title("Wheel-lift accelerations, CAD r3 mass cases")
    zz = np.array([float(k) for k in res["com_limits"]["x_min_mm_vs_z_mm"]])
    b.fill_betweenx(zz, list(res["com_limits"]["x_min_mm_vs_z_mm"].values()), list(res["com_limits"]["x_max_mm_vs_z_mm"].values()),
                    color="g", alpha=.15, label="allowed CoM region")
    for n in names:
        cx, _, cz = res["cases"][n]["com_mm"]
        b.plot(cx, cz, "o", label=n.replace("cad_", ""))
    b.axvline(-40, color="k", lw=.8); b.axvline(40, color="k", lw=.8)
    b.set_xlabel("CoM x [mm]"); b.set_ylabel("CoM z [mm]"); b.set_title("Static CoM envelope (lift >= 3 m/s^2, climb ok)")
    b.legend(fontsize=7); b.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(HERE / "a6_stability.png", dpi=110)
    for n in names:
        print(n, json.dumps(r3({"flat": res["cases"][n]["flat"], "margins": res["cases"][n]["margins"]}, 2)))
    print(json.dumps(r3({k: res[k] for k in ("payload_retention", "com_limits", "stop")}, 3), indent=1))


if __name__ == "__main__":
    main()
