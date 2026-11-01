"""7. Odometry resolution per encoder candidate: counts/rev, mm/count, counts per 20 ms
sample at 0.3 m/s and in a 1 rad/s pivot, velocity quantisation."""
import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import HERE, MOTORS, r3, save_section
from loads import B, R

CIRC = 2 * math.pi * R
F_S = 50.0          # REQ-SCOUT-003


def main():
    res = {"sample_rate_Hz": F_S, "wheel_circumference_m": CIRC, "candidates": {}}
    v_pivot = 1.0 * B / 0.69 / 2
    for mk, mot in MOTORS.items():
        if not mot.cpr:
            res["candidates"][mk] = "no encoder"
            continue
        variants = {"claimed": mot.cpr}
        if mk == "hiwonder":
            variants["buyer_measured_150ppr"] = 150 * 4
        for vk, cpr in variants.items():
            d = {"counts_per_rev_x4": cpr, "mm_per_count": CIRC / cpr * 1000}
            for lbl, v in (("0.3m_s", 0.3), ("pivot_wheel", v_pivot), ("0.05m_s", 0.05)):
                cps = v / CIRC * cpr
                d[f"counts_per_s_{lbl}"] = cps
                d[f"counts_per_sample_{lbl}"] = cps / F_S
                d[f"vel_quant_m_s_{lbl}"] = CIRC / cpr * F_S          # one count per sample
                d[f"vel_quant_pct_{lbl}"] = 100 * (CIRC / cpr * F_S) / v
            d["min_speed_5_counts_per_sample_m_s"] = 5 * CIRC / cpr * F_S
            d["isr_rate_4_motors_0.3m_s"] = 4 * 0.3 / CIRC * cpr
            res["candidates"][f"{mk}_{vk}"] = d
    res["note"] = ("use the ESP32 PCNT hardware counters (no ISR load); at low speed estimate velocity from "
                   "the time between edges or a 100 ms window; publish joint_states at 50 Hz regardless")
    save_section("odometry", r3(res, 4))

    v = np.linspace(0.02, 0.4, 100)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for k, d in res["candidates"].items():
        if isinstance(d, dict):
            ax.plot(v, 100 * (CIRC / d["counts_per_rev_x4"] * F_S) / v, label=f"{k} ({d['counts_per_rev_x4']} cpr)")
    ax.axvline(0.3, color="k", ls=":"); ax.axvline(v_pivot, color="gray", ls=":")
    ax.set_yscale("log"); ax.set_xlabel("wheel surface speed [m/s]"); ax.set_ylabel("1-count velocity step at 50 Hz [%]")
    ax.grid(alpha=.3, which="both"); ax.legend(fontsize=7); ax.set_title("Encoder velocity quantisation (x4 decoding)")
    fig.tight_layout(); fig.savefig(HERE / "a7_odometry.png", dpi=110)
    print(json.dumps(r3(res, 3), indent=1))


if __name__ == "__main__":
    main()
