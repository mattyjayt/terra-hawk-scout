"""2. Drivetrain: wheel torque/speed for cruise, acceleration, 10 mm threshold, and the
operating points against TT 1:48 (3/6/7.4 V) and three TT-form-factor encoder motors."""
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import ENV, G, HERE, MOTORS, r3, save_section
from loads import (B, R, axle_loads, design_case, rpm_from_v, skid_side_forces, step_climb,
                   step_torque_required, straight_force, w_from_v)

V = ENV["v_cruise"]
A = V / ENV["t_accel"]                       # 0.6 m/s^2
W_TURN = ENV["w_turn"]
CHI = ENV["chi"]["low"]                       # worst (most slip) -> fastest wheels in a turn
CRAWL = 0.05                                 # m/s approach speed for the step climb (A)


def operating_points(m, com_x, surface):
    mu = ENV["mu"][surface]
    pts = {}
    f = straight_force(m, surface) / 4
    pts["cruise"] = (w_from_v(V), f * R)
    f = straight_force(m, surface, A) / 4
    pts["accel_0.6"] = (w_from_v(V), f * R)                        # torque needed at the end of the ramp
    f_o, f_i, M = skid_side_forces(m, mu, surface, com_x)
    v_pivot = W_TURN * B / CHI / 2                                 # slip-corrected wheel speed (Mandow chi)
    pts["pivot_1rad"] = (w_from_v(v_pivot), f_o / 2 * R)
    pts["arc_0.3+1rad"] = (w_from_v(V + v_pivot), f_o / 2 * R)
    nf, nr = axle_loads(m, com_x)
    h = ENV["threshold_h"]
    crr = ENV["crr"][surface]
    for nm, nc, npush in (("step_front", nf, nr), ("step_rear", nr, nf)):
        t = step_torque_required(nc, npush, h, mu, crr)
        pts[nm] = (w_from_v(CRAWL), t if t is not None else float("nan"))
    return pts


def main():
    res = {"assumptions": {
        "accel_m_s2": A, "chi_used": CHI, "crawl_speed_m_s": CRAWL,
        "crr": ENV["crr"], "mu": ENV["mu"], "note": "4 wheels share traction equally (A); quasi-static step climb, square edge (worst case)"}}
    cases = {"proposed_c1_pay300": design_case("proposed_c1_pay300"),
             "v1_c1_pay300": design_case("v1_c1_pay300"),
             "proposed_c1_pay0": design_case("proposed_c1_pay0")}
    res["wheel_rpm"] = {"cruise_0.3": rpm_from_v(V),
                        "pivot_1rad_geometric": rpm_from_v(W_TURN * B / 2),
                        "pivot_1rad_slip_chi": rpm_from_v(W_TURN * B / CHI / 2),
                        "arc_outer_0.3+1rad": rpm_from_v(V + W_TURN * B / CHI / 2)}
    ops = {}
    for cname, (m, com) in cases.items():
        for surf in ("carpet", "hard"):
            ops[f"{cname}_{surf}"] = operating_points(m, com[0], surf)
    res["operating_points_w_T"] = {k: {p: {"rpm": w * 60 / 2 / np.pi, "T_Nm": t} for p, (w, t) in v.items()}
                                   for k, v in ops.items()}

    # step climb sensitivity: per axle, both layouts
    sc = {}
    for cname, (m, com) in cases.items():
        nf, nr = axle_loads(m, com[0])
        sc[cname] = {"front_axle_load_frac": nf / (m * G)}
        for mu in (0.5, 0.6, 0.7, 0.9):
            for nm, nc, npush in (("front", nf, nr), ("rear", nr, nf)):
                t = step_torque_required(nc, npush, 0.010, mu, 0.08)
                sc[cname][f"{nm}_mu{mu}_T_req_Nm"] = t
    res["step_climb"] = sc

    # safety factors per motor and voltage on the design case (proposed, carpet = worst)
    key = "proposed_c1_pay300_carpet"
    sf = {}
    volts = {"tt_stock": [3, 6, 7.4], "hiwonder": [6, 7.4, 9], "dfrobot": [6, 7.4], "yahboom": [6, 7.4]}
    for mk, mot in MOTORS.items():
        sf[mk] = {}
        for v in volts[mk]:
            row = {"no_load_rpm": mot.w0(v) * 60 / 2 / np.pi, "stall_Nm": mot.ts(v),
                   "stall_A": v / mot.R, "in_rated_range": mot.v_range[0] <= v <= mot.v_range[1]}
            for p, (w, t) in ops[key].items():
                ta = mot.torque_avail(v, w)
                row[f"SF_{p}"] = (ta / t) if t and t == t and t > 0 else None
                row[f"I_{p}_A"] = mot.current(t) if t == t else None
            sf[mk][f"{v}V"] = row
        sf[mk]["model"] = mot.as_dict()
    res["safety_factors_design_case_carpet"] = sf
    # v1 layout (rear payload) step climb with each motor at its main voltage
    v1 = {}
    for mk, v in (("tt_stock", 6), ("hiwonder", 7.4), ("dfrobot", 7.4), ("yahboom", 7.4)):
        mot = MOTORS[mk]
        for cname in ("v1_c1_pay300_carpet", "proposed_c1_pay300_carpet"):
            w, t = ops[cname]["step_rear"]
            v1[f"{mk}@{v}V_{cname}_rear_step_SF"] = (mot.torque_avail(v, w) / t) if t == t else None
            w, t = ops[cname]["step_front"]
            v1[f"{mk}@{v}V_{cname}_front_step_SF"] = (mot.torque_avail(v, w) / t) if t == t else None
    res["step_SF_layout_compare"] = v1

    # max climbable threshold vs height for the recommended motor
    hs = np.linspace(0.002, 0.025, 47)
    mot = MOTORS["hiwonder"]
    curves = {}
    for cname, (m, com) in cases.items():
        nf, nr = axle_loads(m, com[0])
        ok = []
        for h in hs:
            t = mot.torque_avail(7.4, w_from_v(CRAWL))
            okf = step_climb(nf, nr, h, 0.6, 0.08, t)[0]
            okr = step_climb(nr, nf, h, 0.6, 0.08, t)[0]
            ok.append(okf and okr)
        hmax = max([h for h, o in zip(hs, ok) if o], default=0)
        curves[cname] = hmax
    res["max_threshold_m_hiwonder_7.4V_mu0.6"] = curves
    save_section("drivetrain", r3(res, 4))

    # ---------- plot torque-speed curves with operating points
    fig, axs = plt.subplots(1, 4, figsize=(20, 5), sharey=True)
    marks = {"cruise": "o", "accel_0.6": "s", "pivot_1rad": "^", "arc_0.3+1rad": "v",
             "step_front": "P", "step_rear": "X"}
    for ax, (mk, mot) in zip(axs, MOTORS.items()):
        rpm = np.linspace(0, 420, 300)
        for v in volts[mk]:
            ta = [mot.torque_avail(v, r * 2 * np.pi / 60) for r in rpm]
            ax.plot(rpm, ta, label=f"{v} V")
        for p, (w, t) in ops[key].items():
            ax.plot(w * 60 / 2 / np.pi, t, marks[p], color="k", ms=8, label=p)
        w, t = ops["v1_c1_pay300_carpet"]["step_rear"]
        ax.plot(w * 60 / 2 / np.pi, t, "X", color="r", ms=10, label="step_rear (v1 layout)")
        ax.set_title(mot.name, fontsize=9); ax.set_xlabel("wheel rpm"); ax.grid(alpha=.3)
        ax.set_ylim(0, 0.16)
    axs[0].set_ylabel("torque per wheel [N m]")
    axs[-1].legend(fontsize=7)
    fig.suptitle("Linear motor model (datasheet) vs required operating points, 1.21 kg, carpet, proposed layout")
    fig.tight_layout(); fig.savefig(HERE / "a2_torque_speed.png", dpi=110)
    print(json.dumps(r3({k: res[k] for k in ("wheel_rpm", "step_climb", "step_SF_layout_compare",
                                             "max_threshold_m_hiwonder_7.4V_mu0.6")}, 3), indent=1))
    for mk in sf:
        for vk, row in sf[mk].items():
            if vk != "model":
                print(mk, vk, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()})


if __name__ == "__main__":
    main()
