"""Shared inputs for the Scout v2 physics/electrical round-1 analysis.

Frame: REP-103 base_footprint (origin on the ground at the centre of rotation,
+X forward, +Y left, +Z up). SI units (m, kg, N, V, A, W) unless a name says _mm/_g.

Every number carries a source tag:
  D:<url>  datasheet / vendor page      M: measured (Jaime / Elegoo STL)
  V1:      converted from cad/scout-v2/v1-reference (itself measured/assumed)
  A:       assumed (reason given)         C: computed here
If cad/scout-v2/out/robot_params.json exists, geometry() uses it instead of v1.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
RESULTS = HERE / "results.json"
PARAMS_JSON = ROOT / "out" / "robot_params.json"

G = 9.81

# --------------------------------------------------------------------------- sources
SRC = {
    "tt_adafruit": "https://www.adafruit.com/product/3777",
    "hiwonder_tt_enc": "https://www.hiwonder.com/products/tt-motor-plastic",
    "dfrobot_fit0450": "https://www.dfrobot.com/product-1457.html",
    "yahboom_tt_enc": "https://www.yahboom.net/study/Quad-MD-Module/13991/",
    "ld19": "https://cdn.robotshop.com/media/D/Dfr/RB-Dfr-1273/pdf/dfrobot-dtof-ld19-laser-lidar-sensor-kit-12m-datasheet.pdf",
    "c1": "https://www.slamtec.com/en/c1/spec",
    "c1_wiki": "https://www.waveshare.com/wiki/RPLIDAR_C1",
    "tb6612": "https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG",
    "mdd3a": "https://www.cytron.io/p-3amp-4v-16v-dc-motor-driver-2-channels",
    "mdd3a_pdf": "https://download.kamami.pl/p578534-MDD3A%20Datasheet.pdf",
    "d36v50f5": "https://www.pololu.com/product/4091",
    "pi_pd": "https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf",
    "pi_power": "https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/computers/raspberry-pi/power-supplies.adoc",
    "pi5_cnx": "https://www.cnx-software.com/2023/11/05/raspberry-pi-5-review-raspberry-pi-os-bookworm-benchmarks-power-consumption/",
    "pi5_tips": "https://raspberry.tips/en/raspberrypi-tutorials/raspberry-pi-power-consumption-update-2026-all-models-compared",
    "cam3": "https://www.raspberrypi.com/products/camera-module-3/",
    "mandow2007": "http://babel.isa.uma.es/_utils/downloads/logdownloads.php?u=jafma&t=pdf&f=downloads/jafma/papers/mandow2007ekw.pdf",
    "rr_review": "https://pmc.ncbi.nlm.nih.gov/articles/PMC7863146/",
    "ld19_ros2": "https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2",
    "c1_ros2": "https://github.com/Slamtec/sllidar_ros2",
}

# --------------------------------------------------------------------------- geometry
def geometry() -> dict:
    """Wheel/plate geometry in metres. v1 numbers converted (+Y fwd -> +X fwd,
    origin moved from plate centre / plate underside to CoR / ground)."""
    if PARAMS_JSON.exists():
        p = json.loads(PARAMS_JSON.read_text())
        w = p["wheels"]
        return {
            "source": f"robot_params.json {p.get('version')}",
            "wheelbase": p["wheelbase"], "track": p["track_width"],
            "r_wheel": w["fl"]["radius"], "wheel_w": w["fl"].get("width", 0.026),
            "axle_x": [w["fl"]["pos"][0], w["rl"]["pos"][0]],
            "footprint": p.get("footprint"), "scan_z": p.get("lidar_scan_plane_z"),
            "raw": p,
        }
    # v1-reference/src/scout_dims.py: axle Y +38 / -42 (CoR at y=-2), wheel x = 47+2+13 = 62
    # wheel centre 15.5 mm above plate underside, R 32.5 -> plate underside 17 mm above ground
    return {
        "source": "v1-reference converted (robot_params.json not present)",
        "wheelbase": 0.080,            # V1 (measured wheel wells 2..74 / -78..-6 mm)
        "track": 0.124,                # V1 (spine half 47 M + gap 2 A + half wheel 13 A)
        "r_wheel": 0.0325,             # D Elegoo 65 mm wheel
        "wheel_w": 0.026,              # A (v1)
        "axle_x": [0.040, -0.040],
        "plate_x": [-0.1134, 0.1174],  # V1 plate 230.78 M, centre offset 2 mm from CoR
        "plate_half_y": 0.0663,        # M 132.52/2
        "ground_clearance": 0.017,     # V1: motors sit on the bottom plate (verify)
        "bottom_plate_z": [0.017, 0.0215],
        "top_plate_z": [0.0612, 0.0657],
        "scan_z": 0.1312,              # V1 fit check (clears payload bay by 5.5 mm)
    }


# --------------------------------------------------------------------------- motors
KGFCM = 0.0980665  # N*m per kgf*cm


class Motor:
    """Linear brushed-DC model at the gearbox output, fitted to a datasheet point
    (V_ref, no-load rpm, no-load current, stall torque, stall current) and scaled
    with voltage: R = V/I_stall, w0(V) = (V - I0 R)/Ke, T_stall(V) = T_s * V/V_ref."""

    def __init__(self, key, name, v_ref, rpm0, i0, t_stall_kgfcm, i_stall, ratio,
                 cpr_quad, mass_g, v_range, src, notes=""):
        self.key, self.name, self.v_ref = key, name, v_ref
        self.rpm0, self.i0, self.i_stall, self.ratio = rpm0, i0, i_stall, ratio
        self.t_stall = t_stall_kgfcm * KGFCM
        self.cpr = cpr_quad                     # quadrature counts per wheel rev (x4)
        self.mass = mass_g / 1000
        self.v_range, self.src, self.notes = v_range, src, notes
        self.R = v_ref / i_stall
        self.Ke = (v_ref - i0 * self.R) / (rpm0 * 2 * math.pi / 60)   # V per rad/s (output)
        self.kt = self.t_stall / (i_stall - i0)                         # N*m per A (output, incl. gear loss)

    def w0(self, v):
        return max(v - self.i0 * self.R, 0) / self.Ke

    def ts(self, v):
        return self.t_stall * v / self.v_ref

    def torque_avail(self, v, w):
        """Torque available at output speed w (rad/s) with average voltage v."""
        w0 = self.w0(v)
        return max(self.ts(v) * (1 - w / w0), 0.0) if w0 > 0 else 0.0

    def current(self, torque):
        return self.i0 + torque / self.kt

    def voltage_needed(self, torque, w):
        """Average terminal voltage to deliver torque at speed w (linear model)."""
        i = self.current(torque)
        return i * self.R + self.Ke * w

    def as_dict(self):
        return {k: getattr(self, k) for k in ("name", "v_ref", "rpm0", "i0", "i_stall", "ratio", "cpr",
                                               "mass", "v_range", "src", "notes")} | {
            "t_stall_Nm": self.t_stall, "R_ohm": self.R, "Ke": self.Ke, "kt": self.kt}


MOTORS = {
    "tt_stock": Motor("tt_stock", "Elegoo/Adafruit TT 1:48 (stock, no encoder)", 6.0, 200, 0.15, 0.8, 1.5, 48,
                      0, 30.6, (3, 6), SRC["tt_adafruit"],
                      "spec 200 rpm @6 V (Adafruit measured 250); stall 0.8 kgf.cm, 1.5 A measured @6 V"),
    "hiwonder": Motor("hiwonder", "Hiwonder TT Hall-encoder 1:42 (plastic shaft)", 6.0, 150, 0.10, 1.2, 1.2, 42,
                      6 * 42 * 4, 34, (3, 12), SRC["hiwonder_tt_enc"],
                      "'torque 1.2 kg.cm' treated as stall torque (worst case); 6-line ring x 42 = 252 ppr/ch claimed, "
                      "one buyer measured 150/ch -> verify on arrival; body 79.95x22.4x25.79 mm; recommend 6-9 V"),
    "dfrobot": Motor("dfrobot", "DFRobot FIT0450 TT encoder 1:120", 6.0, 160, 0.17, 0.8, 2.8, 120,
                     960 * 4, 50, (3, 7.5), SRC["dfrobot_fit0450"],
                     "960 pulses/output rev (8 ppr motor x 120) -> 3840 counts x4 if 960 is per channel; encoder "
                     "needs 4.5-7.5 V supply (5 V outputs -> level shift for ESP32); 0.8 kgf.cm at 1:120 is "
                     "suspiciously equal to the 1:48 TT"),
    "yahboom": Motor("yahboom", "Yahboom TT encoder 1:45", 6.0, 355, 0.08, 1.2, 1.1, 45,
                     13 * 45 * 4, 50, (5, 13), SRC["yahboom_tt_enc"],
                     "torque printed '1.2 N.m' (implausible) -> read as 1.2 kg.cm; 13-line encoder 3.3-5 V; "
                     "mass not published (A 50 g)"),
}

# --------------------------------------------------------------------------- environment
ENV = {
    # Rolling resistance coefficient Crr. Hard floor: rubber on laminate/tile ~0.01-0.02
    # (wheelchair/caster literature, rr_review); carpet tile: 0.03-0.1 for large wheels, small
    # 65 mm wheels sink relatively more -> A 0.08 design value (conservative), sweep 0.03-0.12.
    "crr": {"hard": 0.015, "carpet": 0.08},
    # Rubber tyre lateral (sliding) friction: A 0.7 hard floor, 0.6 carpet tile; sweep 0.3-0.9.
    # Measure: tilt a board with the robot on it until it slides, mu = tan(angle).
    "mu": {"hard": 0.7, "carpet": 0.6},
    "threshold_h": 0.010,          # brief
    "payload": 0.300,              # brief (max)
    "v_cruise": 0.30, "w_turn": 1.0, "t_accel": 0.5,   # REQ-SCOUT-001 + derived target
    # Mandow et al. 2007 Tables I/II (P3-AT, L/B=0.675 like Scout 0.645): steering efficiency chi 0.69-0.76
    "chi": {"low": 0.69, "high": 0.76},
}

BATTERY = {
    "dims_mm": (70, 50, 15), "label_wh": 3 * 3.7 * 3.0,  # M dims, label 3S 3000 mAh
    "v_full": 12.6, "v_nom": 11.1, "v_low_load": 9.9,
}


# --------------------------------------------------------------------------- round 2: CAD mass cases
FIT_CHECK = ROOT / "checks" / "fit_check_v2.json"
SRC.update({
    "fit0450_wiki": "https://wiki.dfrobot.com/fit0450/",
    "fit0450_dim": "https://dfimg.dfrobot.com/wiki/19292/FIT0450_tt-motor-with-encoder_dimension_v1.pdf",
    "fit0450_eur": "https://opencircuit.shop/product/tt-motor-with-encoder-6v-160rpm-1201",
    "d36v50f5_eur": "https://www.berrybase.de/en/pololu-d36v50f5-5v-5.5a-step-down-voltage-regulator",
    "d36v50f5_graphs": "https://www.pololu.com/product/4091 (dropout / efficiency / max-current graphs, read by eye)",
    "mdd3a_eur": "https://opencircuit.shop/product/3amp-4v-16v-dc-motor-driver-2-channels",
    "c1_eur": "https://www.generationrobots.com/en/404200-360-rplidar-c1-slamtec.html",
    "ld19_eur": "https://robot-italy.com/en/products/sen0547-dtof-ld19-laser-lidar-sensor-kit-12m",
    "ld19_pdf": "https://raw.githubusercontent.com/LetsOKdo/ld19/master/datasheet/LDROBOT_LD19_Datasheet_V1.0.pdf",
    "c1_pdf": "https://engineering.purdue.edu/477grp3/Files/refs/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf",
    "pi_power_doc": "https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/computers/raspberry-pi/power-supplies.adoc",
    "yolo_pi5": "https://docs.ultralytics.com/guides/raspberry-pi/",
    "mpu6050_ps": "https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf",
    "hcsr04": "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf",
    "esp32_ds": "https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf",
})


def payload_x() -> float:
    """CAD payload (cup keep-out) centre x [m] (v2-r3: -0.050)."""
    return json.loads(PARAMS_JSON.read_text())["payload"]["com"][0]


def cad_cases() -> dict:
    """Mass/CoM/inertia cases (round 4: CAD v2-r3). C1 default build from checks/fit_check_v2.json (written by the
    CAD r3 fit check: scout_v2 = C1 + Hiwonder + MDD3A, cup at x = -50 mm); LD19 variant from
    out/robot_params_ld19_hiwonder_pay.json. The no-payload variant has the same masses and is skipped."""
    out = {}
    ld = ROOT / "out" / "robot_params_ld19_hiwonder_pay.json"
    if FIT_CHECK.exists():
        names = {"scout_v2": "c1"}
        for m in json.loads(FIT_CHECK.read_text()):
            if m["model"] not in names:
                continue
            k = names[m["model"]]
            out[f"cad_{k}_pay0"] = {"mass_kg": m["mass_empty_kg"], "com_m": [c / 1000 for c in m["com_empty_mm"]],
                                   "inertia": m.get("inertia_empty_about_com_kg_m2"),
                                   "scan_z_m": m["lidar_check"]["scan_plane_z_mm"] / 1000,
                                   "source": f"checks/fit_check_v2.json {m['model']} (empty)"}
            if "mass_with_payload_kg" in m:
                out[f"cad_{k}_pay300"] = {"mass_kg": m["mass_with_payload_kg"], "com_m": [c / 1000 for c in m["com_with_payload_mm"]],
                                         "inertia": m.get("inertia_with_payload_about_com_kg_m2"),
                                         "scan_z_m": m["lidar_check"]["scan_plane_z_mm"] / 1000,
                                         "source": f"checks/fit_check_v2.json {m['model']} (300 g payload at x={payload_x()*1000:.0f} mm)"}
    if ld.exists():
        p = json.loads(ld.read_text())
        out["cad_ld19_pay0"] = {"mass_kg": p["total_mass"], "com_m": p["com"], "inertia": p.get("inertia_about_com"),
                                "scan_z_m": p["lidar_scan_plane_z"], "source": f"{ld.name} {p['version']} (empty)"}
        w = p["with_payload"]
        out["cad_ld19_pay300"] = {"mass_kg": w["total_mass"], "com_m": w["com"], "inertia": w.get("inertia_about_com"),
                                  "scan_z_m": p["lidar_scan_plane_z"], "source": f"{ld.name} {p['version']} (300 g payload)"}
    return out


def fit_check_raw() -> list:
    return json.loads(FIT_CHECK.read_text()) if FIT_CHECK.exists() else []


# --------------------------------------------------------------------------- results io
def load_results() -> dict:
    return json.loads(RESULTS.read_text()) if RESULTS.exists() else {}


def save_section(name: str, data: dict) -> None:
    res = load_results()
    res[name] = data
    res["_meta"] = {"frame": "REP-103 base_footprint, SI units", "geometry_source": geometry()["source"],
                    "sources": SRC}
    RESULTS.write_text(json.dumps(res, indent=1, default=float))


def r3(x, n=3):
    if isinstance(x, dict):
        return {k: r3(v, n) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [r3(v, n) for v in x]
    if isinstance(x, float):
        return round(x, n)
    return x
