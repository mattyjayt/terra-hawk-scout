"""3. Skid-steer turning resistance, motor torque/current for 1 rad/s, and odometry effects.

Model: Coulomb lateral friction at each wheel contact (Wong, Theory of Ground Vehicles,
skid-steering chapter): M_r = mu_l * sum(N_i |x_i|) = mu_l W L/2 for axles at +/-L/2.
Turn feasibility needs side traction >= M_r/B, i.e. L/B < 1 (mu cancels).
Slip/ICR: Mandow et al. 2007 measured steering efficiency chi = B / (x_ICRr - x_ICRl)
= 0.69-0.76 on a P3-AT (L/B = 0.675, close to Scout's 0.645).
"""
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import ENV, G, HERE, MOTORS, r3, save_section
from loads import B, L, R, axle_loads, design_case, skid_side_forces, w_from_v

m, com = design_case("proposed_c1_pay300")


def per_motor(mot, v_avg, mu, surface, w_body=1.0, chi=0.69):
    f_o, f_i, M = skid_side_forces(m, mu, surface, com[0])
    t = f_o / 2 * R
    w = w_from_v(w_body * B / chi / 2)
    return {"M_r_Nm": M, "F_side_N": f_o, "T_wheel_Nm": t, "wheel_rpm": w * 60 / 2 / np.pi,
            "T_avail_Nm": mot.torque_avail(v_avg, w), "SF": mot.torque_avail(v_avg, w) / t,
            "I_motor_A": mot.current(t), "V_needed": mot.voltage_needed(t, w)}


def main():
    res = {"L_over_B": L / B, "turn_feasible_traction": L / B < 1.0,
           "mass_kg": m, "com_x_m": com[0]}
    mus = np.linspace(0.3, 0.9, 13)
    sweep = {}
    for mk, v in (("tt_stock", 6.0), ("hiwonder", 7.4), ("hiwonder", 9.0), ("dfrobot", 7.4), ("yahboom", 7.4)):
        rows = [per_motor(MOTORS[mk], v, mu, "carpet") for mu in mus]
        sweep[f"{mk}@{v}V"] = {"mu": mus.tolist(), "SF": [r["SF"] for r in rows],
                               "I_motor_A": [r["I_motor_A"] for r in rows],
                               "T_wheel_Nm": [r["T_wheel_Nm"] for r in rows]}
    res["sweep_carpet"] = sweep
    res["design_points"] = {f"{mk}_{s}": per_motor(MOTORS[mk], 7.4 if mk != "tt_stock" else 6.0, ENV["mu"][s], s)
                            for mk in MOTORS for s in ("carpet", "hard")}
    # mechanical power dissipated scrubbing at 1 rad/s (heat in tyres, drawn from battery)
    res["scrub_power_W_hard"] = skid_side_forces(m, ENV["mu"]["hard"], "hard", com[0])[2] * 1.0

    # odometry: wheel-only yaw error if geometric track is used
    chi = np.array([ENV["chi"]["low"], ENV["chi"]["high"]])
    res["odometry"] = {
        "yaw_rate_overestimate_factor_if_geometric_B": (1 / chi).tolist(),
        "heading_error_deg_per_90deg_turn": (90 * (1 / chi - 1)).tolist(),
        "effective_track_m": (B / chi).tolist(),
        "chi_variation_deg_per_90deg_after_calibration": 90 * (chi[1] - chi[0]) / chi.mean(),
        "conclusion": "wheel yaw is unusable for the 5 deg/10 m budget; fuse vx from wheels, yaw rate from the gyro",
    }
    # gyro drift budget for REQ-SCOUT-006 (10 m at 0.3 m/s = 33 s)
    t10 = 10 / ENV["v_cruise"]
    res["gyro_budget"] = {
        "t_10m_s": t10, "max_bias_deg_s_for_5deg": 5 / t10,
        "MPU6050_noise_0.005dps_rtHz_ARW_deg": 0.005 * np.sqrt(t10),
        "note": "MPU-6050 datasheet: ZRO +/-20 dps initial, 0.005 dps/rtHz noise -> calibrate bias at start and at every stop (ZUPT); keep IMU away from the Pi heat",
    }
    save_section("skid_steer", r3(res, 4))

    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.5))
    for k, d in sweep.items():
        a.plot(d["mu"], d["SF"], label=k)
        b.plot(d["mu"], np.array(d["I_motor_A"]) * 2, label=k)
    a.axhline(1, color="r", ls="--"); a.axhline(1.5, color="orange", ls=":")
    a.set_xlabel("lateral friction mu"); a.set_ylabel("torque safety factor, pivot 1 rad/s"); a.grid(alpha=.3); a.legend(fontsize=7)
    b.axhline(1.2, color="r", ls="--", label="TB6612 1.2 A cont."); b.axhline(3.0, color="g", ls=":", label="MDD3A 3 A cont.")
    b.set_xlabel("lateral friction mu"); b.set_ylabel("current per driver channel (2 motors in parallel) [A]")
    b.grid(alpha=.3); b.legend(fontsize=7)
    fig.suptitle(f"Skid-steer pivot at 1 rad/s, {m:.2f} kg, L/B = {L/B:.2f}, carpet rolling + lateral scrub")
    fig.tight_layout(); fig.savefig(HERE / "a3_skid_steer.png", dpi=110)
    print(json.dumps(r3({k: v for k, v in res.items() if k != "sweep_carpet"}, 3), indent=1))


if __name__ == "__main__":
    main()
