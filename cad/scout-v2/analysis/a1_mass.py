"""1. Mass budget, centre of mass, axle loads.

Positions come from the v1 layout converted to REP-103 (x2 = y1 + 2 mm, y2 = -x1,
z2 = z1 + 17 mm) unless robot_params.json exists. Two layouts:
  v1       : payload bay at the rear overhang (x = -83 mm), Pi 5 at x = -22, LiDAR on a mast at x = 0
  proposed : payload bay over the centre of rotation (x = 0), Pi 5 rotated 90 deg at the rear
             (x = -75) with the LiDAR stacked on M2.5 standoffs above it, battery moved forward
             (x = +30, between the front motors), ESP32 + motor driver moved aft (x = -35).
"""
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import BATTERY, HERE, MOTORS, PARAMS_JSON, geometry, r3, save_section

GEO = geometry()
TOP = 65.7   # mm, top-plate top face above ground (V1 48.705 + 17)
BOT = 21.5   # mm, bottom-plate top face (V1 4.5 + 17)

# battery mass from volume x pack density (A: LiPo/Li-ion pouch pack incl. wrap 2.0-2.3 g/cm3)
batt_vol_cm3 = np.prod(BATTERY["dims_mm"]) / 1000
BATT_G = round(batt_vol_cm3 * 2.1)
BATT_RANGE = (round(batt_vol_cm3 * 1.9), round(batt_vol_cm3 * 2.4))


def parts(layout="v1", lidar="c1", motor="hiwonder"):
    """(label, mass_g, source, (x, y, z) mm)."""
    m_motor = MOTORS[motor].mass * 1000
    pi_x = -22 if layout == "v1" else -75
    lidar_x = 0 if layout == "v1" else -75
    bay_x = -83 if layout == "v1" else 0
    batt_x = 0 if layout == "v1" else 30          # proposed: battery forward on the lower deck
    esp_x, drv_x = (59.5, 52) if layout == "v1" else (-35, -35)   # proposed: electronics move aft
    scan_z = 131.2
    if lidar == "ld19":
        l_mass, l_src, l_z = 47 + 10, "D ld19 47 g w/o cable + A 10 g cable/USB board", scan_z - 24 + 15
    else:
        l_mass, l_src, l_z = 110 + 15, "D c1 110 g + A 15 g cable/USB board", scan_z - 28 + 18
    P = [
        ("bottom_plate", 115, "V1 CAD volume x A acrylic 1.19 g/cm3 (weigh!)", (2, 0, 19.25)),
        ("top_plate", 113, "V1 CAD volume x A acrylic 1.19 g/cm3 (weigh!)", (2, 0, 63.45)),
        ("dead_plate", 3, "V1 CAD", (106, 0, 67)),
        ("battery", BATT_G, f"C {batt_vol_cm3:.1f} cm3 x A 2.1 g/cm3 (range {BATT_RANGE})", (batt_x, 0, BOT + 7.5)),
        ("pi5", 46, "A commonly reported board mass", (pi_x, 0, TOP + 6 + 1)),
        ("pi5_cooler", 20, "A (weigh)", (pi_x, 0, TOP + 6 + 8)),
        ("pi5_standoffs", 4, "A 4x M2.5x6 brass", (pi_x, 0, TOP + 3)),
        ("camera", 7, "A 4 g module + 3 g cable", (106, 0, 80)),
        ("lidar", l_mass, l_src, (lidar_x, 0, l_z)),
        ("lidar_standoffs", 15, "A 4x M2.5 brass ~35-40 mm + plate/screws", (lidar_x, 0, (TOP + scan_z - 25) / 2)),
        ("esp32", 10, "A DevKit", (esp_x, 12, 33)),
        ("motor_driver", 15, "A MDD3A-class board", (drv_x, -15.5, 30)),
        ("imu", 2, "A GY-521", (0, 0, 59)),
        ("ultrasonic", 9, "A HC-SR04 8.5 g", (105.5, 0, 31.5)),
        ("buck_5v", 7, "A Pololu D36V50F5 (25x25x9.5 mm)", (-63, 0, 27)),
        ("fuse_switch_conn", 22, "A mini-blade holder + switch + 2x XT30", (-90, 0, 30)),
        ("wiring", 35, "A 20/22 AWG + Dupont + zip ties", (0, 0, 40)),
    ]
    for nm, x in (("fl", 64), ("fr", 64), ("rl", -64), ("rr", -64)):
        y = 36.5 if nm[1] == "l" else -36.5
        P.append((f"motor_{nm}", m_motor, f"D {MOTORS[motor].src}", (x, y, 32.5)))
        P.append((f"motor_bracket_{nm}", 4, "A Elegoo mount + M3 bolts", (x, y, 32.5)))
        ax = 40 if nm[0] == "f" else -40
        P.append((f"wheel_{nm}", 25, "A v1 value (weigh)", (ax, 62 if nm[1] == "l" else -62, 32.5)))
    return P, (bay_x, 0, TOP + 30)


def summarise(P, bay, payload_g):
    m = np.array([p[1] for p in P] + [payload_g], float)
    xyz = np.array([p[3] for p in P] + [bay], float)
    M = m.sum()
    com = (m[:, None] * xyz).sum(0) / M
    a_f, a_r = 40.0, -40.0
    n_f = (com[0] - a_r) / (a_f - a_r)        # front-axle load fraction
    return M, com, n_f


def main():
    cases = {}
    for layout in ("v1", "proposed"):
        for lidar in ("ld19", "c1"):
            for pay in (0, 300):
                P, bay = parts(layout, lidar)
                M, com, nf = summarise(P, bay, pay)
                cases[f"{layout}_{lidar}_pay{pay}"] = {
                    "mass_kg": M / 1000, "com_mm": com.tolist(), "front_axle_frac": nf}
    P, bay = parts("proposed", "c1")
    table = [{"label": p[0], "mass_g": p[1], "source": p[2], "pos_mm": p[3]} for p in P]
    empty = sum(p[1] for p in P)
    contingency = 0.10 * empty
    out = {
        "geometry_source": GEO["source"],
        "robot_params_json_present": PARAMS_JSON.exists(),
        "battery_mass_g": BATT_G, "battery_mass_range_g": BATT_RANGE,
        "battery_label_Wh": BATTERY["label_wh"],
        "battery_label_Wh_per_kg": BATTERY["label_wh"] / (BATT_G / 1000),
        "battery_label_Wh_per_L": BATTERY["label_wh"] / (batt_vol_cm3 / 1000),
        "battery_realistic_Wh": [round(BATT_G / 1000 * 150, 1), round(BATT_G / 1000 * 250, 1)],
        "components_proposed_c1": table,
        "empty_mass_g": empty, "contingency_10pct_g": contingency,
        "design_mass_kg_with_payload": (empty + contingency + 300) / 1000,
        "lidar_delta_c1_minus_ld19_g": (110 + 15) - (47 + 10),
        "cases": cases,
    }
    save_section("mass", r3(out))

    # ---- plot: mass breakdown + CoM side view
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5))
    groups = {}
    for p in P:
        k = p[0].split("_")[0] if p[0].startswith(("motor_", "wheel_")) else p[0]
        k = "motors" if k == "motor" else "wheels" if k == "wheel" else k
        groups[k] = groups.get(k, 0) + p[1]
    groups["payload (max)"] = 300
    ks = sorted(groups, key=lambda k: groups[k])
    a1.barh(ks, [groups[k] for k in ks], color="#3a7bd5")
    a1.set_xlabel("mass [g]")
    a1.set_title(f"Mass budget (proposed layout, C1, Hiwonder) empty {empty:.0f} g + 300 g payload")
    for lay, col in (("v1", "#c0392b"), ("proposed", "#27ae60")):
        for pay, mk in ((0, "o"), (300, "s")):
            c = cases[f"{lay}_c1_pay{pay}"]["com_mm"]
            a2.plot(c[0], c[2], mk, color=col, ms=9, label=f"{lay}, payload {pay} g")
    a2.plot([-40, 40], [0, 0], "k^", ms=12, label="wheel contacts (axles)")
    a2.add_patch(plt.Rectangle((-113.4, 17), 230.8, 4.5, color="0.7"))
    a2.add_patch(plt.Rectangle((-113.4, 61.2), 230.8, 4.5, color="0.7"))
    for x in (-40, 40):
        a2.add_patch(plt.Circle((x, 32.5), 32.5, fill=False))
    a2.axhline(131.2, color="r", ls="--", lw=1, label="LiDAR scan plane")
    a2.set_aspect("equal"); a2.set_xlim(-130, 130); a2.set_ylim(-5, 150)
    a2.set_xlabel("x [mm] (+ forward)"); a2.set_ylabel("z [mm]")
    a2.set_title("Centre of mass, side view"); a2.legend(fontsize=7, loc="upper right")
    fig.tight_layout(); fig.savefig(HERE / "a1_mass_com.png", dpi=120)
    print(json.dumps(r3({k: v for k, v in out.items() if k != "components_proposed_c1"}), indent=1))


if __name__ == "__main__":
    main()
