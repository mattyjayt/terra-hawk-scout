"""5. Power (round 2): 5 V and motor-rail budgets, battery energy with the REALISTIC capacity,
runtime, 5 V buck selection over 9.0-12.6 V, heat, Pi 5 USB current limit, motor-rail decision,
fuse / wire / switch, low-voltage cutoff, peak current and brown-out, power-distribution table.

Mass/CoM: CAD fit check (checks/fit_check_v2.json, C1 + 300 g payload = worst), not the round-1 a1 layout.
Source tags: D datasheet (URL in SRC), M measured, A assumed (reason), C computed.
Round 4: mass/CoM from the CAD v2-r3 fit check (C1 + Hiwonder + MDD3A, cup at x = -50 mm).
"""
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import BATTERY, ENV, HERE, MOTORS, SRC, r3, save_section
from loads import B, R, design_case, skid_side_forces, straight_force, w_from_v

CASE = "cad_c1_pay300"
m, com = design_case(CASE)
MOT_PRIMARY, V_CAP = MOTORS["hiwonder"], 8.4       # recommended motor, PWM average-voltage cap
MOT_PROXY = MOTORS["tt_stock"]                      # conservative proxy: linear model 0.30 A cruise vs sim 0.36 A, 1.0 A pivot vs sim 0.72 A
SIM_TRACK_RATIO = {"carpet": 1.70, "hard": 1.93}    # sim/results.json closed-loop spin, effective track / geometric

# ---- 5 V loads [W]: (typ, peak, source)
LOADS_5V = {
    "pi5_8gb_active_cooler": (6.5, 10.0, "D/M cnx-software wall meter: idle 3.0 W headless, stress -c 4 8.8 W "
                              f"({SRC['pi5_cnx']}); typ A: SLAM+Nav2+detector ~2-3 cores busy; peak A: 8.8 W + 1.2 W USB/GPU margin"),
    "camera_module_3": (0.6, 1.0, "A: no official power figure published; IMX708 streaming estimate, bound 0.3-1.0 W"),
    "lidar_c1": (1.15, 1.5, f"D 5 V x 230 mA operating ({SRC['c1_wiki']}); start-up not published -> A 300 mA"),
    "esp32_devkit_wifi_off": (0.5, 1.0, f"A from ESP32 datasheet modem-sleep 20-68 mA @240 MHz + LDO/USB-UART losses ({SRC['esp32_ds']})"),
    "encoders_x4": (0.2, 0.25, "A ~10 mA each at 5 V (hall sensor + pull-ups); not in the Hiwonder listing"),
    "hcsr04_imu_mdd3a_logic": (0.2, 0.3, f"D HC-SR04 15 mA @5 V ({SRC['hcsr04']}); D MPU-6050 3.9 mA; A MDD3A logic"),
}
LD19_SWAP = (0.9, 1.5, f"D 180 mA work / 300 mA start ({SRC['ld19_pdf']})")

# ---- conversion / path
ETA_BUCK = {1.0: 0.91, 2.0: 0.93, 3.0: 0.945, 5.0: 0.95}   # D36V50F5, Vin 12 V curve read by eye (SRC d36v50f5_graphs)
DROPOUT = {1.0: 0.83, 3.0: 0.92, 5.0: 1.0}                  # D36V50F5 5 V curve read by eye
BUCK_MAX_A_12V = 8.0                                        # D graph, still air, room temp, Vin 9-18 V
ETA_DRIVER = 0.95                                           # A MOSFET H-bridge (MDD3A); TB6612 ~0.9 at 1 A
R_PACK = (0.06, 0.25)                                       # A bound: 3 cells x 10-60 mOhm + BMS FETs (unknown pack)
R_PATH = 0.04                                               # A: 0.4 m 18 AWG 8 mOhm + 7.5 A blade ~12 + switch ~15 + 2 XT30 ~2
AWG_MOHM_M = {18: 20.9, 20: 33.3, 22: 53.0, 24: 84.2}        # copper at 20 C (standard AWG tables)

# ---- battery energy: label vs realistic (pack volume 52.5 cm3, M)
VOL_L = 0.0525
E_LABEL = BATTERY["label_wh"]
E_RANGE = (12.0, 27.5)       # low: ~230 Wh/L (A, high-rate RC LiPo class); high: round-1 mass route 110 g x 250 Wh/kg
E_DESIGN = 16.5              # A: round-1 mass route low end (110 g x 150 Wh/kg); WEIGH + read cell markings
USABLE = 0.85                # A: stop at ~3.3-3.4 V/cell under load


def eta_buck(i):
    ks = sorted(ETA_BUCK)
    return float(np.interp(i, ks, [ETA_BUCK[k] for k in ks]))


def dropout(i):
    ks = sorted(DROPOUT)
    return float(np.interp(i, ks, [DROPOUT[k] for k in ks]))


def motor_elec(mot, T, w, n=4):
    """Electrical power drawn from the bus by n motors at wheel torque T, speed w (linear model)."""
    i = mot.current(T)
    v = mot.voltage_needed(T, w)
    return n * v * i / ETA_DRIVER, v, i


def main():
    res = {"case": CASE, "mass_kg": m, "com_m": com}
    # ---------------- 5 V rail
    p5_typ = sum(v[0] for v in LOADS_5V.values())
    p5_pk = sum(v[1] for v in LOADS_5V.values())
    res["loads_5V_W"] = {k: {"typ": v[0], "peak": v[1], "src": v[2]} for k, v in LOADS_5V.items()}
    res["loads_5V_W"]["(alt) lidar_ld19"] = {"typ": LD19_SWAP[0], "peak": LD19_SWAP[1], "src": LD19_SWAP[2]}
    i5_typ, i5_pk = p5_typ / 5.0, p5_pk / 5.0
    res["rail_5V"] = {"typ_W": p5_typ, "peak_W": p5_pk, "typ_A": i5_typ, "peak_A": i5_pk,
                      "pi_spec_A": 5.0, "eta_typ": eta_buck(i5_typ), "eta_peak": eta_buck(i5_pk),
                      "buck_in_typ_W": p5_typ / eta_buck(i5_typ), "buck_in_peak_W": p5_pk / eta_buck(i5_pk),
                      "buck_loss_typ_W": p5_typ / eta_buck(i5_typ) - p5_typ,
                      "buck_loss_peak_W": p5_pk / eta_buck(i5_pk) - p5_pk,
                      "buck_loss_at_5A_W": 25 / eta_buck(5) - 25}

    # ---------------- motor operating points (carpet = worst), primary and proxy motors
    ops = {}
    T_cr = straight_force(m, "carpet") / 4 * R
    T_ac = straight_force(m, "carpet", 0.6) / 4 * R
    f_o, _, _ = skid_side_forces(m, ENV["mu"]["carpet"], "carpet", com[0])
    T_pv = f_o / 2 * R
    w_cr = w_from_v(0.3)
    w_pv = 1.0 * B * SIM_TRACK_RATIO["carpet"] / 2 / R
    T_climb = 0.061      # a9 quasi-static rear-axle need, C1 + payload, mu 0.6 (all four wheels assumed loaded alike)
    for nm, (T, w) in {"cruise_0.3": (T_cr, w_cr), "accel_0.6": (T_ac, w_cr), "pivot_1rad_s": (T_pv, w_pv),
                       "arc_0.3+1rad_s": (T_pv, w_cr + w_pv), "climb_10mm_0.05": (T_climb, w_from_v(0.05))}.items():
        row = {"T_wheel_Nm": T, "wheel_rpm": w * 60 / 2 / np.pi}
        for mk, mot in (("hiwonder", MOT_PRIMARY), ("tt_proxy", MOT_PROXY)):
            p, v, i = motor_elec(mot, T, w)
            row[mk] = {"P_bus_W": p, "V_avg": v, "I_motor_A": i, "V_ok_under_cap": v <= (V_CAP if mk == "hiwonder" else 6.0)}
        ops[nm] = row
    # stall at the voltage cap (all four motors, e.g. wedged against a wall)
    stall = {}
    for mk, mot, vcap in (("hiwonder@8.4", MOT_PRIMARY, V_CAP), ("tt_stock@6", MOT_PROXY, 6.0),
                          ("fit0450@7.4", MOTORS["dfrobot"], 7.4), ("hiwonder@12.6_firmware_fault", MOT_PRIMARY, 12.6),
                          ("fit0450@12.6_firmware_fault", MOTORS["dfrobot"], 12.6)):
        i_m = vcap / mot.R
        stall[mk] = {"I_motor_A": i_m, "I_motor_side_4_A": 4 * i_m, "P_windings_W": 4 * vcap * i_m,
                     "I_batt_at_11.1V_A": 4 * vcap * i_m / 11.1 / ETA_DRIVER, "I_channel_2_motors_A": 2 * i_m}
    ops["stall_all_4"] = stall
    res["motor_ops"] = ops

    # ---------------- duty profile and runtime
    prof = {"stationary": 0.40, "cruise": 0.45, "turn": 0.10, "accel_climb": 0.05}   # A: indoor patrol mission
    p_mot = {}
    for mk in ("hiwonder", "tt_proxy"):
        p_mot[mk] = {"stationary": 0.0, "cruise": ops["cruise_0.3"][mk]["P_bus_W"],
                     "turn": ops["pivot_1rad_s"][mk]["P_bus_W"], "accel_climb": ops["accel_0.6"][mk]["P_bus_W"]}
    p5_in_typ = res["rail_5V"]["buck_in_typ_W"]
    p5_in_pk = res["rail_5V"]["buck_in_peak_W"]
    avg = {mk: p5_in_typ + sum(prof[k] * p_mot[mk][k] for k in prof) for mk in p_mot}
    worst = {mk: p5_in_pk + 0.7 * p_mot[mk]["cruise"] + 0.3 * p_mot[mk]["turn"] for mk in p_mot}
    rt = {}
    for lbl, e in (("label_33.3Wh", E_LABEL), ("realistic_low_12Wh", E_RANGE[0]), ("design_16.5Wh", E_DESIGN),
                   ("realistic_high_27.5Wh", E_RANGE[1])):
        rt[lbl] = {f"typ_{mk}_min": e * USABLE / avg[mk] * 60 for mk in avg} | \
                  {f"worst_{mk}_min": e * USABLE / worst[mk] * 60 for mk in worst}
    res["battery"] = {"label_Wh": E_LABEL, "label_Wh_per_L": E_LABEL / VOL_L, "realistic_Wh_range": E_RANGE,
                      "design_Wh": E_DESIGN, "usable_fraction": USABLE,
                      "why_label_implausible": "634 Wh/L at pack level; good cylindrical cells alone reach ~600-700 Wh/L and "
                                               "a 15 mm-thick pack cannot hold 18650s, so these are pouch cells + BMS + wrap: "
                                               "expect ~230-520 Wh/L -> 12-27.5 Wh (A)"}
    res["runtime"] = {"profile": prof, "avg_W": avg, "worst_continuous_W": worst, "minutes": rt,
                      "min_Wh_for_30min_worst": 0.5 * max(worst.values()) / USABLE,
                      "min_Wh_for_60min_typ": 1.0 * max(avg.values()) / USABLE}

    # ---------------- currents, brown-out, BMS
    i_avg = max(avg.values()) / BATTERY["v_nom"]
    i_worst = max(worst.values()) / 10.0
    i_burst = stall["hiwonder@8.4"]["I_batt_at_11.1V_A"] * 11.1 / 9.6 + p5_in_pk / 9.6
    i_fault = stall["hiwonder@12.6_firmware_fault"]["I_batt_at_11.1V_A"] * 11.1 / 12.6 + p5_in_pk / 12.6
    bo = {}
    for r_pack in R_PACK:
        r = r_pack + R_PATH
        v_bus = 9.6 - i_burst * r          # pack at 3.2 V/cell under load, end of discharge
        bo[f"Rpack_{r_pack}"] = {"V_bus_during_4_motor_stall": v_bus, "buck_min_Vin_at_3A": 5.0 + dropout(3.0),
                                 "margin_V": v_bus - 5.0 - dropout(3.0), "V_bus_full_pack": 12.6 - i_burst * r}
    res["currents_A"] = {"avg": i_avg, "worst_continuous": i_worst, "burst_4_stall_at_cap_plus_pi_peak": i_burst,
                         "firmware_fault_100pct_duty_at_12.6V": i_fault}
    res["brownout"] = {"R_pack_bound_ohm": R_PACK, "R_path_ohm": R_PATH, "cases": bo,
                       "verdict": "Pi 5 V rail survives a 4-motor stall even at R_pack 0.25 ohm and a 9.6 V pack "
                                  "(the buck needs only ~5.9 V in). The real brown-out mechanism is the pack's BMS "
                                  "over-current trip (rating UNKNOWN): if it trips below the burst current the whole "
                                  "robot, Pi included, loses power. Read the pack/BMS markings; add a firmware "
                                  "stall-current limit (back-EMF model from encoder speed)."}

    # ---------------- buck selection
    res["buck_5V_options"] = {
        "pololu_d36v50f5": {
            "verdict": "BUY (recommended)",
            "vin": "5.5-50 V; dropout ~0.83 V @1 A, ~0.92 V @3 A, ~1.0 V @5 A (graph) -> regulates down to ~6 V in; "
                   "pack floor 9.0 V leaves >= 3 V headroom",
            "iout": f"~{BUCK_MAX_A_12V} A continuous at 9-18 V in, still air (graph); 5.5 A at 36 V",
            "eff_12V": "91 % @1 A, ~94-95 % @3-5 A (graph)",
            "protection": "reverse input to 40 V, OCP, short, thermal, soft-start, PG output, EN pin (D)",
            "size_mass": "25.4 x 25.4 x 9.5 mm, 7 g (D)", "price_eur": "34.95 (BerryBase)", "src": SRC["d36v50f5"],
            "weak_point": "fixed 5.0 V: REQ-SCOUT-012 lower limit 5.0 V is not guaranteed after tolerance and lead drop"},
        "xl4015_generic": {
            "verdict": "DON'T BUY as the Pi supply",
            "why": "listing rates ~3.5 A sustained without extra cooling (round-1 source manuals.plus B078JTR9GM) < 5 A "
                   "Pi spec; no reverse protection, no PG; trim pot drifts with vibration; 51 x 26 x 14 mm. Only "
                   "advantage: can be trimmed to 5.15 V."},
        "ubec_5A_generic": {
            "verdict": "DON'T BUY (unverifiable)",
            "why": "no datasheet with output tolerance, ripple or dropout found; hobby UBECs are specified for servo "
                   "loads, not for a 5.0-5.25 V window"},
        "owned_lm2596": {"verdict": "NOT for the Pi", "why": "2-3 A class (BRIEF) < 5 A"},
    }
    v_lo, v_hi = 5.0 * 0.96, 5.0 * 1.04
    drop = {}
    for awg, mo in AWG_MOHM_M.items():
        rr = 2 * 0.15 * mo / 1000
        drop[f"{awg}AWG_0.15m_pair"] = {"R_ohm": rr, "drop_3A_V": 3 * rr, "drop_5A_V": 5 * rr}
    res["pi_supply"] = {
        "lead_drop": drop,
        "V_at_pi_worst_20AWG_peak": v_lo - i5_pk * drop["20AWG_0.15m_pair"]["R_ohm"],
        "V_at_pi_best": v_hi, "pi_undervoltage_threshold_V": "4.63 +/- 5 % (" + SRC["pi_power_doc"] + ")",
        "REQ_SCOUT_012": "5.0-5.25 V NOT guaranteed by a fixed 5.0 V +/-4 % buck (tolerance as read in round 1 -- "
                         "re-verify) -> propose: '>= 4.9 V at the header under load and vcgencmd get_throttled == 0x0 "
                         "over 30 min', or verify the delivered unit reads >= 5.0 V at 3 A before installing",
        "usb_current": "Without USB-PD the Pi 5 limits downstream USB to 600 mA total; with a 5 A-capable supply it gives "
                       "1.6 A (" + SRC["pi_power_doc"] + "). GPIO back-powering cannot negotiate PD, so set "
                       "usb_max_current_enable=1 in config.txt (and PSU_MAX_CURRENT=5000 in the EEPROM, round-1 source "
                       + SRC["pi_pd"] + "). USB load: LiDAR 230-300 mA (+ ESP32 ~100-250 mA if powered over USB) = "
                       "0.33-0.55 A -> fits even 600 mA, but start-up surges make it marginal: set the flag.",
        "wiring": "buck VOUT -> header pins 2 and 4, GND -> pins 6 and 14; 20 AWG, <= 15 cm; keyed connector; GPIO 5 V "
                  "bypasses the Pi's input protection -> verify polarity with the multimeter, never hot-plug the 5 V lead",
        "heat": f"buck loss {res['rail_5V']['buck_loss_typ_W']:.2f} W typical, {res['rail_5V']['buck_loss_peak_W']:.2f} W "
                f"peak, {res['rail_5V']['buck_loss_at_5A_W']:.2f} W at 5 A -- fine for a 25 mm board in the lower deck; "
                "keep >= 5 mm air gap to the battery",
    }

    # ---------------- motor rail decision
    res["motor_rail"] = {
        "decision": "NO separate motor buck. MDD3A (4-16 V) straight on the pack, PWM duty capped in firmware to "
                    f"V_cap / V_batt with V_cap = {V_CAP} V (Hiwonder, rated 3-12 V) or 7.4 V (FIT0450, max 7.5 V); "
                    "V_batt from the ESP32 ADC.",
        "why_no_buck": "a 5 A motor buck would current-limit exactly when torque is needed (climb 1.2-2.4 A/channel, "
                       "4-motor stall 6-7 A) and adds a part; the CAD's D36V28F-class motor buck (2.8 A family) is far "
                       "too small and also overlaps buck_5v in fit_check_v2.json",
        "pwm_heating": "at 20 kHz the motor's electrical time constant (L/R ~0.1-0.5 ms, A: 130-size can L 0.3-1 mH not "
                       "published) is >> the 50 us PWM period: ripple <= ~0.3 A p-p worst, extra copper loss "
                       "< 0.02 W/motor. Heating follows average current, i.e. the same as DC at V_cap.",
        "brush_wear": "commutation arcing scales with the PEAK applied voltage, so 12.6 V PWM wears brushes somewhat "
                      "faster than 6 V DC (not quantifiable from any datasheet); acceptable for a TT-class motor "
                      "(EUR 6-13, spare bought). Speed and gearbox wear follow the average voltage, which the cap limits.",
        "failure_mode": "firmware fault at 100 % duty puts 12.6 V on the motors: Hiwonder 2.5 A stall each, 5 A/channel "
                        "= MDD3A peak; FIT0450 5.9 A each -> MDD3A over-current shutdown. Mitigation: the cap lives in "
                        "the lowest PWM write function; watchdog brakes (LL); boot with PWM pins low.",
        "alt_two_packs": "Option: motors on the owned Elegoo 2S box (7.4-8.4 V, no cap needed), Pi on the 3S pack -> "
                         "removes all coupling, costs space/mass (measure the 2S box first).",
    }
    # ---------------- protection
    res["protection"] = {
        "fuse": "7.5 A ATM mini-blade in an inline holder with 18 AWG leads, right after the battery XT30. Normal max "
                f"{i_worst:.1f} A continuous; 4-motor stall burst {i_burst:.1f} A (< 1 s); a pack short delivers >> 20 A. "
                "If the BMS trips below 7.5 A the BMS is the real protection -> still fit the fuse (BMS failure, "
                "wiring short ahead of the BMS).",
        "switch": "the CAD's MTS-102-class mini toggle is typically AC-rated only (~3 A at 250 VAC, DC not stated): "
                  "inadequate for 6-7 A bursts. Use a >= 10 A DC-rated rocker/toggle (e.g. Pololu #1406 10 A rocker, "
                  "check its DC rating) or a MOSFET pushbutton switch.",
        "wire": {"battery->fuse->switch->split": "18 AWG silicone", "split->MDD3A VM": "20 AWG",
                 "MDD3A->motors": "22-24 AWG (<= 1.8 A each)", "buck->Pi header": "20 AWG pair or 2 x 22 AWG pairs",
                 "signals/encoders": "26-28 AWG, JST-PH/Dupont"},
        "bulk_cap": "470 uF / 25 V low-ESR electrolytic at the MDD3A VM input + 47-100 uF at the buck input (Pololu warns "
                    "about LC input spikes on hot-plug; a 3S pack can ring towards 2 x 12.6 V)",
        "low_voltage_cutoff_3S": {"warn_V": 10.5, "stop_motors_V": 10.2, "pi_shutdown_V": 9.9, "absolute_floor_V": 9.0,
                                  "filter": "2 s average, ignore dips < 0.5 s, 0.3 V hysteresis",
                                  "measure": "ESP32 ADC1 pin, 47k/10k divider (12.6 V -> 2.21 V), 100 nF, calibrate "
                                             "with the multimeter (REQ-SCOUT-013)",
                                  "bms": "UNKNOWN whether the blue pack has a BMS. With a BMS: its cutoff (2.5-3.0 V/cell) "
                                         "is the last line. Without (bare LiPo with a balance lead): the firmware cutoff "
                                         "is the ONLY protection -> add a 3S LiPo buzzer alarm (~EUR 3)."},
    }
    # ---------------- power-distribution table
    res["distribution_table"] = [
        {"node": "Battery 3S", "V": "9.0-12.6", "I_typ_A": round(i_avg, 2), "I_peak_A": round(i_burst, 1),
         "protection": "BMS? (unknown) + 7.5 A fuse", "wire": "18 AWG", "connector": "XT30"},
        {"node": "Main switch", "V": "9.0-12.6", "I_typ_A": round(i_avg, 2), "I_peak_A": round(i_burst, 1),
         "protection": "-", "wire": "18 AWG", "connector": ">= 10 A DC switch"},
        {"node": "MDD3A VM (4 motors, 2 per channel)", "V": "9.0-12.6 (PWM avg cap 8.4)",
         "I_typ_A": round(p_mot["tt_proxy"]["cruise"] / 11.1, 2),
         "I_peak_A": round(stall["hiwonder@8.4"]["I_batt_at_11.1V_A"] * 11.1 / 9.6, 1),
         "protection": "MDD3A OCP/UVLO/thermal + firmware cap", "wire": "20 AWG", "connector": "screw terminal"},
        {"node": "D36V50F5 VIN", "V": "9.0-12.6", "I_typ_A": round(p5_in_typ / 11.1, 2), "I_peak_A": round(p5_in_pk / 9.0, 2),
         "protection": "reverse, OCP, thermal (built in)", "wire": "20 AWG", "connector": "soldered"},
        {"node": "5 V bus -> Pi 5 header pins 2/4", "V": "5.0 (4.8-5.2)", "I_typ_A": round(LOADS_5V["pi5_8gb_active_cooler"][0] / 5, 2),
         "I_peak_A": round(LOADS_5V["pi5_8gb_active_cooler"][1] / 5, 2), "protection": "buck PG -> ESP32 input",
         "wire": "20 AWG pair", "connector": "keyed 2x2 housing / JST-XH"},
        {"node": "Pi USB -> LiDAR C1", "V": "5.0 (C1 accepts 4.8-5.2 D)", "I_typ_A": 0.23, "I_peak_A": 0.3,
         "protection": "Pi USB limit 0.6 / 1.6 A", "wire": "kit cable", "connector": "USB-A"},
        {"node": "Pi CSI -> Camera 3", "V": "3.3 (from Pi)", "I_typ_A": 0.12, "I_peak_A": 0.2, "protection": "-",
         "wire": "FFC", "connector": "22-pin mini FFC (Pi 5)"},
        {"node": "5 V bus -> ESP32 VIN, encoders, HC-SR04", "V": "5.0", "I_typ_A": 0.18, "I_peak_A": 0.3,
         "protection": "-", "wire": "22-24 AWG", "connector": "Dupont"},
        {"node": "ESP32 3V3 -> MPU-6050, encoder pull-ups", "V": "3.3", "I_typ_A": 0.01, "I_peak_A": 0.02,
         "protection": "-", "wire": "26 AWG", "connector": "Dupont"},
    ]
    save_section("power", r3(res, 3))

    # ---------------- plots
    fig, (a, b) = plt.subplots(1, 2, figsize=(15, 5.2))
    labels = list(LOADS_5V) + ["motors cruise 0.3", "motors pivot 1 rad/s", "motors accel 0.6"]
    typ = [v[0] for v in LOADS_5V.values()] + [p_mot["tt_proxy"]["cruise"], p_mot["tt_proxy"]["turn"],
                                               p_mot["tt_proxy"]["accel_climb"]]
    pk = [v[1] for v in LOADS_5V.values()] + [p_mot["hiwonder"]["cruise"], p_mot["hiwonder"]["turn"],
                                              p_mot["hiwonder"]["accel_climb"]]
    y = np.arange(len(labels))
    a.barh(y + 0.2, typ, 0.4, label="5 V: typical | motors: stock TT linear model (conservative vs sim)")
    a.barh(y - 0.2, pk, 0.4, label="5 V: peak | motors: Hiwonder linear model")
    a.set_yticks(y); a.set_yticklabels(labels, fontsize=8); a.set_xlabel("W (motors: from the bus, all 4)")
    a.set_title(f"Power budget {m:.2f} kg, carpet. Mission avg {max(avg.values()):.1f} W, worst {max(worst.values()):.1f} W",
                fontsize=10)
    a.legend(fontsize=7); a.grid(alpha=.3, axis="x")
    e = np.linspace(8, 35, 60)
    for mk, ls in (("tt_proxy", "-"), ("hiwonder", "--")):
        b.plot(e, e * USABLE / avg[mk] * 60, ls, color="C0", label=f"typical mission, {mk} ({avg[mk]:.1f} W)")
        b.plot(e, e * USABLE / worst[mk] * 60, ls, color="C3", label=f"worst continuous, {mk} ({worst[mk]:.1f} W)")
    b.axhline(30, color="r", ls=":", label="30 min requirement"); b.axhline(60, color="orange", ls=":", label="60 min aim")
    b.axvspan(*E_RANGE, color="0.88", label="realistic pack energy 12-27.5 Wh")
    b.axvline(E_DESIGN, color="k", ls=":", label="design 16.5 Wh"); b.axvline(E_LABEL, color="k", ls="-.", label="label 33.3 Wh")
    b.set_xlabel("pack energy [Wh]"); b.set_ylabel("runtime [min]"); b.grid(alpha=.3); b.legend(fontsize=7)
    b.set_title("Runtime vs pack energy (85 % usable)", fontsize=10)
    fig.tight_layout(); fig.savefig(HERE / "a5_power.png", dpi=110)
    print(json.dumps(r3({k: res[k] for k in ("rail_5V", "runtime", "currents_A", "brownout")}, 2), indent=1))
    for k, v in ops.items():
        print(k, json.dumps(r3(v, 3)))


if __name__ == "__main__":
    main()
