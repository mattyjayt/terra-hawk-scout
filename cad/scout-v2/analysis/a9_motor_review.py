"""9. Drivetrain review against the CAD mass cases (round 4: CAD v2-r3, see common.cad_cases).

  a) Datasheet self-consistency of each TT-class motor: the stall torque a linear DC motor
     must produce given its own no-load speed, no-load current and stall current
     (T_stall_elec = eta * Ke * (I_stall - I0)), vs the printed stall torque.
  b) Quasi-static 10 mm threshold climb per axle with the CAD loads, traction limit vs mu.
  c) Payload position sweep: which payload x makes both axles climbable (layout fix).
  d) Spin in place at 1 rad/s: torque, wheel speed (sim slip ratio), and driver current per channel.
  e) Driver current per channel (2 motors in parallel) for each candidate, at climb and stall.
"""
import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import ENV, G, HERE, MOTORS, ROOT, cad_cases, payload_x, r3, save_section
from loads import AX_F, AX_R, B, L, R, axle_loads, skid_side_forces, step_climb, step_torque_required, w_from_v

SIM = json.loads((ROOT / "sim" / "results.json").read_text()) if (ROOT / "sim" / "results.json").exists() else {}
ETA_GEAR = (0.4, 0.65)        # A: plastic spur TT gearbox efficiency at low speed, bound (no datasheet value exists)
CRAWL = 0.05                  # m/s slow threshold approach (sim scenario)
# average-voltage (PWM duty) cap per motor; 8.4 V for Hiwonder keeps dynamics constant down to the 9.0 V pack floor
VOLTS = {"tt_stock": 6.0, "hiwonder": 8.4, "dfrobot": 7.4, "yahboom": 7.4}
DRIVERS = {"tb6612 (Elegoo shield)": (1.2, 3.2), "mdd3a": (3.0, 5.0)}         # cont / peak A per channel (datasheets)


def consistency(mot):
    ke = mot.Ke
    lo, hi = (e * ke * (mot.i_stall - mot.i0) for e in ETA_GEAR)
    return {"Ke_V_s_rad_output": ke, "R_ohm": mot.R, "printed_stall_Nm": mot.t_stall,
            "electrical_stall_Nm_range": [lo, hi], "ratio_elec_over_printed": [lo / mot.t_stall, hi / mot.t_stall]}


def main():
    cases = cad_cases()
    res = {"source": {k: v["source"] for k, v in cases.items()},
           "assumptions": {"eta_gear_bound": ETA_GEAR, "crawl_m_s": CRAWL, "crr_carpet": ENV["crr"]["carpet"],
                           "volts_cap": VOLTS, "drivers_cont_peak_A": DRIVERS}}

    # a) consistency
    res["datasheet_consistency"] = {k: consistency(m) for k, m in MOTORS.items()}

    # b) climb per CAD case, mu sweep
    climb = {}
    mus = [0.4, 0.5, 0.55, 0.6, 0.7, 0.8, 0.9]
    for cn, c in cases.items():
        m, com = c["mass_kg"], c["com_m"]
        nf, nr = axle_loads(m, com[0])
        row = {"mass_kg": m, "com_x_mm": com[0] * 1000, "front_share": nf / (m * G)}
        for mu in mus:
            tf = step_torque_required(nf, nr, ENV["threshold_h"], mu, ENV["crr"]["carpet"])
            tr = step_torque_required(nr, nf, ENV["threshold_h"], mu, ENV["crr"]["carpet"])
            row[f"mu{mu}"] = {"T_front_Nm": tf, "T_rear_Nm": tr}
        # traction-feasibility limit (lowest mu where both axles climb with unlimited torque)
        mu_min = next((mu for mu in np.arange(0.30, 1.2, 0.01)
                       if step_climb(nf, nr, ENV["threshold_h"], mu, ENV["crr"]["carpet"], 10)[0]
                       and step_climb(nr, nf, ENV["threshold_h"], mu, ENV["crr"]["carpet"], 10)[0]), None)
        row["mu_min_both_axles"] = mu_min
        climb[cn] = row
    res["climb_quasi_static"] = climb

    # torque available per motor at crawl (printed stall torque, linear model) vs need (design case)
    need = climb["cad_c1_pay300"]["mu0.6"]["T_rear_Nm"]
    avail = {}
    for k, mot in MOTORS.items():
        v = VOLTS[k]
        ta = mot.torque_avail(v, w_from_v(CRAWL))
        avail[k] = {"V": v, "T_avail_printed_Nm": ta, "SF_rear_climb_printed": ta / need,
                    "T_avail_if_electrical_stall_low_eta_Nm": consistency(mot)["electrical_stall_Nm_range"][0] * v / mot.v_ref
                    * (1 - w_from_v(CRAWL) / mot.w0(v)),
                    "I_motor_at_climb_A": mot.current(need), "I_channel_2_motors_A": 2 * mot.current(need),
                    "in_rated_voltage": mot.v_range[0] <= v <= mot.v_range[1]}
    res["climb_motor_SF_design_case_cad_c1_pay300_mu0.6"] = {"T_need_rear_Nm": need, "motors": avail}
    res["sim_disagreement"] = {
        "sim_crawl_payload_stall_torque_sweep": SIM.get("d_threshold", {}).get("stall_torque_sweep_crawl_payload"),
        "sim_crawl_payload_mu_sweep": SIM.get("d_threshold", {}).get("mu_sweep_crawl_payload"),
        "sim_mass_kg": SIM.get("_meta", {}).get("total_mass_with_payload_kg"),
        "note": "round 4 (CAD r3 params, cup at x = -50): the sim crawl climb now needs only ~0.056-0.058 N m peak and succeeds "
                "from 0.0785 N m stall and mu 0.4 up, i.e. it agrees with the quasi-static TORQUE (0.05-0.06 N m) but is MORE "
                "optimistic on traction (quasi-static square edge: mu >= ~0.55). The r1 'stall at 0.11' came from the v1 "
                "fallback params (rear payload x = -83 mm). The sim threshold has a 1.5 mm edge radius (A); measure the real "
                "threshold profile and mu (M3, M4).",
    }

    # c) payload x sweep on the CAD C1 case
    base = cases["cad_c1_pay300"]
    m = base["mass_kg"]
    x0 = payload_x()
    sweep = []
    for xp in np.arange(-0.08, 0.081, 0.01):
        cx = base["com_m"][0] + 0.3 * (xp - x0) / m
        nf, nr = axle_loads(m, cx)
        tf = step_torque_required(nf, nr, 0.010, 0.6, 0.08)
        tr = step_torque_required(nr, nf, 0.010, 0.6, 0.08)
        sweep.append({"payload_x_mm": xp * 1000, "com_x_mm": cx * 1000, "front_share": nf / (m * G),
                      "T_front": tf, "T_rear": tr,
                      "T_max": max(tf or 9, tr or 9) if (tf and tr) else None})
    best = min((s for s in sweep if s["T_max"]), key=lambda s: s["T_max"])
    res["payload_x_sweep_c1_mu0.6"] = {"rows": sweep, "best": best}
    # swap layout: Pi+LiDAR stack (~173 g) to x=-45, payload to x=+30 (proposed by this review)
    stack_g = 46 + 25 + 4 + 110 + 25.7 + 4 * 6.7 + 0.4
    cx_swap = base["com_m"][0] + (-stack_g / 1000 * 0.090 + 0.3 * (0.030 - x0)) / m
    nf, nr = axle_loads(m, cx_swap)
    res["swap_layout_c1"] = {"stack_mass_g": stack_g, "com_x_mm": cx_swap * 1000, "front_share": nf / (m * G),
                             "T_front": step_torque_required(nf, nr, 0.010, 0.6, 0.08),
                             "T_rear": step_torque_required(nr, nf, 0.010, 0.6, 0.08)}

    # d) spin in place
    spin = {}
    _cl = SIM.get("b_spin", {}).get("closed_loop_gyro", {})
    sim_ratio = {"carpet": _cl.get("carpet", {}).get("effective_track_ratio", 1.70),
                 "hard": _cl.get("hard_floor", {}).get("effective_track_ratio", 1.93)}   # sim/results.json, gyro loop
    for cn in ("cad_c1_pay300", "cad_ld19_pay300", "cad_c1_pay0"):
        m, com = cases[cn]["mass_kg"], cases[cn]["com_m"]
        for surf in ("carpet", "hard"):
            mu = ENV["mu"][surf] if surf == "carpet" else 0.8
            f_o, f_i, M = skid_side_forces(m, mu, surf, com[0])
            T = f_o / 2 * R
            w_wheel = 1.0 * B * sim_ratio[surf] / 2 / R
            row = {"M_resist_Nm": M, "T_wheel_Nm": T, "wheel_rpm": w_wheel * 60 / 2 / math.pi,
                   "traction_ok_L_over_B": L / B < 1}
            for k, mot in MOTORS.items():
                v = VOLTS[k]
                ta = mot.torque_avail(v, w_wheel)
                i = mot.current(T)
                row[k] = {"SF": ta / T, "I_motor_A": i, "I_channel_A": 2 * i}
            spin[f"{cn}_{surf}"] = row
    res["spin_1rad_s"] = spin
    res["spin_verdict"] = (
        "Traction allows it (L/B = 0.645 < 1). Kinematics: wheels must run 1.7-1.9x the geometric speed (sim), "
        "i.e. 32-37 wheel rpm -- easy. Torque: 0.047 (carpet) - 0.055 (hard, mu 0.8) N m per wheel vs 0.0785 N m printed "
        "stall of TT-class motors -> SF 1.1-1.5 at the motor's own rated voltage. Achievable ONLY with gyro-closed-loop "
        "yaw-rate control (sim open-loop: 0.45-0.65 rad/s; closed-loop gyro: 1.0 rad/s). Requirement change proposed: "
        "REQ-SCOUT-001 angular target 1 rad/s measured by the IMU, with Nav2 max_vel_theta 0.8 rad/s for margin.")

    # e) driver channel currents (2 motors in parallel) at climb and at stall (voltage cap)
    drv = {}
    for k, mot in MOTORS.items():
        v = VOLTS[k]
        i_climb = 2 * mot.current(need)
        i_stall = 2 * v / mot.R
        drv[k] = {"V_cap": v, "I_ch_climb_A": i_climb, "I_ch_stall_A": i_stall}
        for dk, (ic, ip) in DRIVERS.items():
            drv[k][dk] = {"climb": "ok" if i_climb <= ic else "peak-only" if i_climb <= ip else "OVER",
                          "stall": "ok" if i_stall <= ic else "peak-only" if i_stall <= ip else "OVER"}
    res["driver_channels"] = drv
    save_section("motor_review", r3(res, 4))

    # plot: payload sweep + motor consistency
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 4.8))
    xs = [s["payload_x_mm"] for s in sweep]
    a.plot(xs, [s["T_front"] or np.nan for s in sweep], "o-", label="front axle climbs")
    a.plot(xs, [s["T_rear"] or np.nan for s in sweep], "s-", label="rear axle climbs")
    a.axhline(0.0785, color="r", ls="--", label="printed stall 0.8 kgf.cm (TT, FIT0450)")
    a.axvline(x0 * 1000, color="k", ls=":", label=f"CAD r3 payload x = {x0*1000:.0f}")
    a.axvline(best["payload_x_mm"], color="g", ls=":", label=f"best x = {best['payload_x_mm']:.0f} mm")
    a.set_xlabel("payload x [mm] (+ forward)"); a.set_ylabel("torque per wheel to climb 10 mm [N m]")
    a.set_title(f"10 mm threshold, C1 build {cases['cad_c1_pay300']['mass_kg']:.2f} kg, mu 0.6, quasi-static"); a.grid(alpha=.3); a.legend(fontsize=7)
    ks = list(MOTORS)
    pr = [MOTORS[k].t_stall for k in ks]
    lo = [res["datasheet_consistency"][k]["electrical_stall_Nm_range"][0] for k in ks]
    hi = [res["datasheet_consistency"][k]["electrical_stall_Nm_range"][1] for k in ks]
    xx = np.arange(len(ks))
    b.bar(xx - 0.2, pr, 0.4, label="printed stall torque")
    b.bar(xx + 0.2, hi, 0.4, color="0.8", label="implied by Ke, I_stall (eta 0.65)")
    b.bar(xx + 0.2, lo, 0.4, color="0.5", label="implied (eta 0.40)")
    b.axhline(need, color="r", ls="--", label=f"need, rear climb {need:.3f}")
    b.set_xticks(xx); b.set_xticklabels(ks); b.set_ylabel("N m at the wheel (rated V)")
    b.set_title("Datasheet self-consistency: printed vs implied stall torque"); b.legend(fontsize=7); b.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(HERE / "a9_motor_review.png", dpi=110)
    # torque-speed curves at the voltage cap with the CAD-case operating points (carpet)
    m, com = cases["cad_c1_pay300"]["mass_kg"], cases["cad_c1_pay300"]["com_m"]
    from loads import straight_force
    f_o, _, _ = skid_side_forces(m, ENV["mu"]["carpet"], "carpet", com[0])
    w_pv = 1.0 * B * 1.70 / 2 / R
    opsp = {"cruise 0.3": (w_from_v(0.3), straight_force(m, "carpet") / 4 * R),
            "accel 0.6": (w_from_v(0.3), straight_force(m, "carpet", 0.6) / 4 * R),
            "pivot 1 rad/s": (w_pv, f_o / 2 * R), "arc 0.3 + 1 rad/s (outer)": (w_from_v(0.3) + w_pv, f_o / 2 * R),
            "climb rear axle 0.05": (w_from_v(CRAWL), need)}
    fig, axs = plt.subplots(1, 4, figsize=(20, 4.8), sharey=True)
    mk_ = {"cruise 0.3": "o", "accel 0.6": "s", "pivot 1 rad/s": "^", "arc 0.3 + 1 rad/s (outer)": "v",
           "climb rear axle 0.05": "X"}
    for ax, (k, mot) in zip(axs, MOTORS.items()):
        rpm = np.linspace(0, 450, 300)
        for v in sorted({mot.v_ref, VOLTS[k]}):
            ax.plot(rpm, [mot.torque_avail(v, r_ * 2 * np.pi / 60) for r_ in rpm], label=f"printed model {v} V")
        lo = consistency(mot)["electrical_stall_Nm_range"][0] * VOLTS[k] / mot.v_ref
        ax.plot(rpm, [max(lo * (1 - r_ * 2 * np.pi / 60 / mot.w0(VOLTS[k])), 0) for r_ in rpm], "--", color="0.5",
                label=f"implied (eta 0.4) {VOLTS[k]} V")
        for p, (w, t) in opsp.items():
            ax.plot(w * 60 / 2 / np.pi, t, mk_[p], color="k", ms=8, label=p)
        ax.set_title(f"{mot.name[:40]}\ncap {VOLTS[k]} V", fontsize=8); ax.set_xlabel("wheel rpm"); ax.grid(alpha=.3)
        ax.set_ylim(0, 0.25)
    axs[0].set_ylabel("torque per wheel [N m]"); axs[-1].legend(fontsize=7)
    fig.suptitle(f"Torque-speed at the PWM voltage cap vs operating points, CAD C1 + 300 g ({m:.2f} kg), carpet mu 0.6")
    fig.tight_layout(); fig.savefig(HERE / "a9_torque_speed.png", dpi=110)
    print(json.dumps(r3({k: res[k] for k in ("datasheet_consistency", "climb_motor_SF_design_case_cad_c1_pay300_mu0.6",
                                             "swap_layout_c1", "driver_channels")}, 3), indent=1))
    print("best payload x", r3(best, 3))
    for k, v in climb.items():
        print(k, r3({kk: vv for kk, vv in v.items() if kk in ("front_share", "mu_min_both_axles", "mu0.6")}, 3))
    for k, v in spin.items():
        print(k, r3({kk: (vv if not isinstance(vv, dict) else vv.get("SF")) for kk, vv in v.items()}, 3))


if __name__ == "__main__":
    main()
