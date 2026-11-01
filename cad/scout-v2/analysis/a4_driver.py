"""4. Motor driver: per-channel current (cruise, accel, skid turn, step, stall) for the stock
Elegoo wiring (TB6612, 2 motors in parallel per channel) and alternatives; ESP32 pin budget."""
import json

from common import BATTERY, MOTORS, SRC, load_results, r3, save_section

DRIVERS = {
    "tb6612_elegoo_shield": {"cont_A": 1.2, "peak_A": 3.2, "vm_max": 13.5, "vm_abs": 15.0, "motors_per_ch": 2,
                             "pins": 7, "src": SRC["tb6612"], "note": "1.2 A ave / 3.2 A peak per ch; VM operating 2.5-13.5 V"},
    "2x_tb6612_one_ch_per_motor": {"cont_A": 1.2, "peak_A": 3.2, "vm_max": 13.5, "vm_abs": 15.0, "motors_per_ch": 1,
                                   "pins": 13, "src": SRC["tb6612"], "note": "12 PWM/IN + shared STBY"},
    "drv8833": {"cont_A": 1.5, "peak_A": 2.0, "vm_max": 10.8, "vm_abs": 11.8, "motors_per_ch": 2, "pins": 4,
                "src": "https://www.ti.com/product/DRV8833", "note": "10.8 V max -> cannot sit on 3S (12.6 V)"},
    "cytron_mdd3a": {"cont_A": 3.0, "peak_A": 5.0, "vm_max": 16.0, "vm_abs": 16.0, "motors_per_ch": 2, "pins": 4,
                     "src": SRC["mdd3a_pdf"], "note": "2 PWM inputs/ch, LL = brake, 20 kHz, 1.7-12 V logic, 48x42 mm"},
}

# classic ESP32-WROOM-32 DevKit: GPIOs that are free and not strapping/flash/UART0
SAFE_IO = [4, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33]
INPUT_ONLY = [34, 35, 36, 39]           # no pull-ups -> external 10k to 3.3 V
STRAPPING = [0, 2, 5, 12, 15]


def main():
    sf = load_results()["drivetrain"]["safety_factors_design_case_carpet"]
    res = {"drivers": DRIVERS, "per_channel_A": {}}
    for mk, vkey in (("tt_stock", "6V"), ("hiwonder", "7.4V"), ("hiwonder", "9V"), ("dfrobot", "7.4V"), ("yahboom", "7.4V")):
        row = sf[mk][vkey]
        mot = MOTORS[mk]
        per_motor = {p: row[f"I_{p}_A"] for p in ("cruise", "accel_0.6", "pivot_1rad", "step_front", "step_rear")}
        v = float(vkey[:-1])
        per_motor["stall_at_cap"] = v / mot.R
        per_motor["stall_unlimited_12.6V"] = BATTERY["v_full"] / (mot.R + 0.5)
        for dk, d in DRIVERS.items():
            n = d["motors_per_ch"]
            ch = {p: (i * n if i is not None else None) for p, i in per_motor.items()}
            verdict = {p: ("ok" if i is not None and i <= d["cont_A"] else
                           "peak-only" if i is not None and i <= d["peak_A"] else "OVER")
                       for p, i in ch.items()}
            res["per_channel_A"][f"{mk}@{vkey}|{dk}"] = {"I": ch, "verdict": verdict}

    # pin budget
    def budget(motor_pins, encoders):
        need = {"motor": motor_pins, "encoder_inputs": 2 * encoders, "i2c": 2, "hcsr04": 2, "batt_adc": 1, "buck_pg": 1}
        total = sum(need.values())
        avail = len(SAFE_IO) + len(INPUT_ONLY)
        return {"need": need, "total": total, "safe_available": avail, "fits_without_strapping": total <= avail,
                "strapping_pins_needed": max(0, total - avail)}
    res["pins"] = {
        "tb6612x1_2enc (DD-SCOUT-HW draft)": budget(7, 2),
        "tb6612x1_4enc": budget(7, 4),
        "tb6612x2_4enc": budget(13, 4),
        "mdd3a_2enc": budget(4, 2),
        "mdd3a_4enc (recommended)": budget(4, 4),
        "safe_io": SAFE_IO, "input_only": INPUT_ONLY, "strapping": STRAPPING,
    }
    res["recommended_pinout_mdd3a_4enc"] = {
        "MDD3A M1A/M1B (left side)": [25, 26], "MDD3A M2A/M2B (right side)": [27, 33],
        "ENC FL A/B": [34, 35], "ENC RL A/B": [36, 39], "ENC FR A/B": [18, 19], "ENC RR A/B": [16, 17],
        "I2C SDA/SCL (MPU-6050)": [21, 22], "HC-SR04 TRIG/ECHO": [4, 13],
        "BATT ADC (ADC1, 47k/10k divider)": [32], "buck PG (open drain, pull-up)": [14], "spare": [23],
        "note": "34-39 need external 10k pull-ups to 3.3 V; ECHO through 1k/2k divider; encoders powered at 3.3 V; use the PCNT peripheral (4 units)",
    }
    save_section("driver", r3(res, 3))
    for k, v in res["per_channel_A"].items():
        print(k, {p: (round(i, 2) if i else i) for p, i in v["I"].items()}, v["verdict"])
    print(json.dumps({k: v for k, v in res["pins"].items() if isinstance(v, dict)}, indent=1))


if __name__ == "__main__":
    main()
