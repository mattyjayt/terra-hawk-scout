"""5. Power tree: rail loads, buck sizing/dropout, Pi 5 power path, wiring, fuse, brown-out,
low-voltage cutoff, runtime."""
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import BATTERY, ENV, HERE, MOTORS, SRC, load_results, r3, save_section
from loads import R, design_case, skid_side_forces, straight_force, w_from_v

MOT = MOTORS["hiwonder"]
m, com = design_case("proposed_c1_pay300")

# ---- 5 V loads [W]: (typical, peak, source)
LOADS_5V = {
    "pi5_8gb_active_cooler": (5.0, 12.0, "A typ (idle 3.0 W cnx; ROS 2 SLAM+Nav2 load) / peak: stress ~8-9 W + USB, " + SRC["pi5_cnx"]),
    "camera_module_3": (1.0, 1.3, "A (no datasheet power figure)"),
    "lidar_c1": (1.15, 1.5, "D 230 mA x 5 V " + SRC["c1_wiki"] + "; start-up A"),
    "esp32_imu_sonar_encoders": (0.6, 1.2, "A ESP32 WiFi off ~60-100 mA + encoders 4x10 mA"),
}
ETA_BUCK = 0.90           # A: Pololu D36V50F5 typical 80-95 % (efficiency graph), 12 V in, 1-3 A out
ETA_DRIVER = 0.95         # A: MOSFET H-bridge
DROPOUT_5A = 1.0          # A conservative (Pololu dropout graph, approx linear with load)
R_PATH = 0.15             # A: 3 cells x ~35 mOhm + fuse/switch/XT30/wire 45 mOhm
AWG_MOHM_M = {18: 20.9, 20: 33.3, 22: 53.0, 24: 84.2}   # copper at 20 C (standard tables)


def motor_power(T, w, n=4):
    v = MOT.voltage_needed(T, w)
    i = MOT.current(T)
    return n * v * i / ETA_DRIVER, v, i


def main():
    res = {"loads_5V_W": {k: {"typ": v[0], "peak": v[1], "src": v[2]} for k, v in LOADS_5V.items()}}
    p5_typ = sum(v[0] for v in LOADS_5V.values())
    p5_peak = sum(v[1] for v in LOADS_5V.values())
    res["rail_5V"] = {"typ_W": p5_typ, "peak_W": p5_peak, "typ_A": p5_typ / 5, "peak_A": p5_peak / 5,
                      "design_max_A": 5.0, "buck_in_typ_W": p5_typ / ETA_BUCK, "buck_in_peak_W": p5_peak / ETA_BUCK}

    # motor electrical power at operating points (carpet, design mass)
    T_cr = straight_force(m, "carpet") / 4 * R
    T_ac = straight_force(m, "carpet", 0.6) / 4 * R
    f_o, _, _ = skid_side_forces(m, ENV["mu"]["carpet"], "carpet", com[0])
    T_pv = f_o / 2 * R
    w_cr = w_from_v(0.3)
    w_pv = w_from_v(1.0 * 0.124 / 0.69 / 2)
    p_cr, v_cr, i_cr = motor_power(T_cr, w_cr)
    p_ac, _, _ = motor_power(T_ac, w_cr)
    p_pv, v_pv, i_pv = motor_power(T_pv, w_pv)
    res["motors_W"] = {"cruise_carpet": p_cr, "accel": p_ac, "pivot_1rad": p_pv,
                       "cruise_V_avg": v_cr, "cruise_I_per_motor": i_cr, "pivot_I_per_motor": i_pv}

    # duty profile (A): 40 % stationary, 45 % cruise, 10 % turning, 5 % accel/threshold
    prof = {"idle": 0.40, "cruise": 0.45, "turn": 0.10, "accel": 0.05}
    p_bus = {"idle": 0.0, "cruise": p_cr, "turn": p_pv, "accel": p_ac}
    p_avg = p5_typ / ETA_BUCK + sum(prof[k] * p_bus[k] for k in prof)
    p_peak_cont = p5_peak / ETA_BUCK + p_pv
    usable = 0.85     # A: stop at 3.4 V/cell under load leaves ~10-15 % in the pack
    e_label = BATTERY["label_wh"]
    e_real = load_results()["mass"]["battery_realistic_Wh"]
    res["runtime"] = {
        "profile": prof, "avg_W": p_avg, "peak_continuous_W": p_peak_cont,
        "label_Wh": e_label, "realistic_Wh_range": e_real,
        "runtime_min_label_typ": e_label * usable / p_avg * 60,
        "runtime_min_realistic_low_typ": e_real[0] * usable / p_avg * 60,
        "runtime_min_realistic_high_typ": e_real[1] * usable / p_avg * 60,
        "runtime_min_realistic_low_peak": e_real[0] * usable / p_peak_cont * 60,
    }

    # battery currents and fuse
    i_avg = p_avg / BATTERY["v_nom"]
    i_peak_cont = p_peak_cont / BATTERY["v_low_load"]
    stall_ch_A = 9.0 / MOT.R            # per motor with the 9 V average cap
    i_stall_burst = 4 * stall_ch_A * 9.0 / BATTERY["v_low_load"] + p5_peak / ETA_BUCK / BATTERY["v_low_load"]
    res["battery_current_A"] = {"avg": i_avg, "peak_continuous": i_peak_cont,
                                "all_4_stalled_burst_at_9V_cap": i_stall_burst,
                                "fuse_recommendation": "7.5 A ATM mini-blade (>= 1.35 x burst? no: burst is < 1 s; "
                                                       "7.5 A carries 2x peak-continuous, blows on a hard short)"}
    # brown-out: battery sag on a 4-motor stall burst at the low-voltage threshold
    v_sag = BATTERY["v_low_load"] - i_stall_burst * R_PATH
    v_min_buck = 5.0 + DROPOUT_5A
    res["brownout"] = {"R_path_ohm_assumed": R_PATH, "v_batt_low": BATTERY["v_low_load"],
                       "v_at_buck_during_burst": v_sag, "buck_min_vin": v_min_buck,
                       "margin_V": v_sag - v_min_buck,
                       "verdict": "no Pi brown-out from motor bursts if the Pi has its own 5 V buck on the pack; "
                                  "the real risk is a pack BMS over-current trip (unknown rating)"}
    # 5 V lead voltage drop (buck -> Pi GPIO 5V pins, 0.15 m each way)
    drops = {}
    for awg, mo in AWG_MOHM_M.items():
        rr = 2 * 0.15 * mo / 1000
        drops[f"{awg}AWG_single_pair"] = {"R_ohm": rr, "drop_at_5A": 5 * rr, "drop_at_3A": 3 * rr}
    res["pi_5V_lead_drop"] = drops
    v_buck_min = 5.0 * 0.96
    res["pi_voltage_budget"] = {
        "buck_tolerance": "D36V50F5 5 V +/-4 % -> 4.80-5.20 V (" + SRC["d36v50f5"] + ")",
        "worst_at_pi_20AWG_3A": v_buck_min - 3 * 2 * 0.15 * 0.0333,
        "pi_undervoltage_flag_V": 4.63,
        "REQ_SCOUT_012_lower_V": 5.0,
        "verdict": "Pi will not flag undervoltage, but REQ-SCOUT-012's 5.0 V floor is not guaranteed by any fixed "
                   "5 V buck (+/-4 %). Measure the unit; relax REQ to 'get_throttled == 0x0 and >= 4.9 V' or use an "
                   "adjustable buck trimmed to 5.15 V.",
    }
    res["pi_power_path"] = {
        "recommended": "GPIO back-power: buck 5V -> header pins 2+4, GND -> pins 6+14, 20 AWG, <= 15 cm",
        "eeprom": "sudo rpi-eeprom-config --edit -> PSU_MAX_CURRENT=5000 (or usb_max_current_enable=1 in config.txt) "
                  "so USB gets 1.6 A instead of 600 mA (" + SRC["pi_pd"] + ")",
        "why_not_usb_c": "the Pi only uses 5 A if a PD source offers a 5V/5A PDO; a plain buck on USB-C is treated as 3 A "
                         "and an extra PD-source board is another failure point",
        "caveat": "GPIO 5 V input bypasses the Pi's input protection: reverse polarity or >5.5 V kills the board -> keyed "
                  "connector, buck with OVP, verify polarity with the multimeter before first connection",
    }
    res["low_voltage_cutoff_3S"] = {
        "warn_V": 10.5, "stop_motors_and_report_V": 10.2, "pi_shutdown_V": 9.9, "pack_bms_or_absolute_V": 9.0,
        "filter": "2 s moving average, ignore dips < 500 ms (motor starts); hysteresis 0.3 V",
        "adc_divider": "47k/10k -> 12.6 V reads 2.21 V, inside ESP32 11 dB range 0.15-2.45 V (ESP-IDF ADC docs); "
                       "100 nF at the ADC pin; calibrate against the multimeter (ESP32 ADC is +/-5 % raw)",
    }
    res["wire"] = {"battery_to_fuse_switch_split": "18 AWG silicone", "motor_rail_to_MDD3A": "20 AWG",
                   "MDD3A_to_motors": "22-24 AWG (<=1.5 A)", "buck_to_pi": "20 AWG pair, or two 22 AWG pairs",
                   "signal": "Dupont/JST-PH 26-28 AWG"}
    save_section("power", r3(res, 3))

    # plot runtime vs pack energy
    e = np.linspace(10, 35, 50)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(e, e * usable / p_avg * 60, label=f"typical mix ({p_avg:.1f} W)")
    ax.plot(e, e * usable / p_peak_cont * 60, label=f"continuous peak ({p_peak_cont:.1f} W)")
    ax.axhline(30, color="r", ls="--", label="30 min requirement"); ax.axhline(60, color="orange", ls=":", label="60 min aim")
    ax.axvspan(*e_real, color="0.85", label="realistic pack energy (mass-based)")
    ax.axvline(e_label, color="k", ls="-.", label="label 33 Wh")
    ax.set_xlabel("pack energy [Wh]"); ax.set_ylabel("runtime [min]"); ax.grid(alpha=.3); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(HERE / "a5_runtime.png", dpi=110)
    print(json.dumps(r3({k: res[k] for k in ("rail_5V", "motors_W", "runtime", "battery_current_A", "brownout",
                                             "pi_voltage_budget")}, 2), indent=1))


if __name__ == "__main__":
    main()
