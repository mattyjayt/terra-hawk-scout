"""8. Sensors (round 2): LD19 vs RPLIDAR C1 against the 5 cm SLAM map and the 15 cm stop; scan plane
from the CAD fit check (blocked sectors, payload/plant growth, pitch-induced floor hits); camera floor
visibility and detection FPS on the Pi 5; HC-SR04 at 3.3 V and its blind zone.
"""
import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import HERE, PARAMS_JSON, SRC, fit_check_raw, payload_x, r3, save_section

_P = json.loads(PARAMS_JSON.read_text())
LIDAR_X, SCAN_Z = _P["sensors"]["laser"]["pos"][0], _P["lidar_scan_plane_z"]
FRONT_X = max(p[0] for p in _P["footprint"])
STOP_RANGE = FRONT_X - LIDAR_X + 0.15         # 15 cm ahead of the bumper, measured from the LiDAR axis

LIDARS = {
    "ld19": {"mass_g": 47, "size_mm": "38.6 x 38.6 x 33.5 body (54 x 46.3 x 34.8 with ears)", "range_m": (0.02, 12),
             "acc_mm": 45, "acc_note": "+/-45 mm for 0.3-12 m; NO accuracy spec <= 300 mm (D)", "std_mm": 10,
             "rate_hz": 4500, "scan_hz": (5, 10, 13), "ang_res_deg": 1.0, "ang_err_deg": 2.0,
             "uart": "TX only, 230400 baud, 3.3 V, ZH1.5T-4P; PWM pin -> internal speed control",
             "power": "4.5-5.5 V, 180 mA work / 300 mA start (D)",
             "ros2": "ldlidar_stl_ros2 (vendor repo, ROS 2 support community-maintained for Jazzy)",
             "price_eur": 142.80, "price_src": SRC["ld19_eur"], "src": SRC["ld19_pdf"]},
    "c1": {"mass_g": 110, "size_mm": "55.6 x 55.6 x 41.3", "range_m": (0.05, 12), "acc_mm": 30,
           "acc_note": "+/-30 mm, 15 mm resolution, from 0.05 m (D)", "std_mm": None,
           "rate_hz": 5000, "scan_hz": (8, 10, 12), "ang_res_deg": 0.72, "ang_err_deg": None,
           "uart": "TX+RX 460800 baud 3.3 V, XH2.54-5P; USB adapter in kit",
           "power": "4.8-5.2 V, 230 mA (1.15 W) (D)", "ros2": "sllidar_ros2 (Slamtec official, C1 launch file)",
           "price_eur": 81.12, "price_src": SRC["c1_eur"], "src": SRC["c1_pdf"]},
}
MAP_RES = 0.05


def main():
    fc = {c["model"]: c for c in fit_check_raw()}
    res = {"lidars": LIDARS}
    # ---- LiDAR vs requirements
    cmp = {}
    for k, l in LIDARS.items():
        d = {}
        for rng in (1, 2, 4, 6):
            d[f"beam_spacing_{rng}m_mm"] = 1000 * rng * math.radians(l["ang_res_deg"])
        d["max_range_2_beams_per_5cm_cell_m"] = MAP_RES / 2 / math.radians(l["ang_res_deg"])
        d["max_range_1_beam_per_5cm_cell_m"] = MAP_RES / math.radians(l["ang_res_deg"])
        d["range_err_vs_5cm_cell"] = l["acc_mm"] / 50
        d["stop_zone_range_m"] = STOP_RANGE
        d["stop_zone_spec_ok"] = l["range_m"][0] <= STOP_RANGE and (k == "c1")
        d["mass_on_tower_g"] = l["mass_g"]
        cmp[k] = d
    res["lidar_vs_req"] = cmp
    res["lidar_choice"] = {
        "recommend": "RPLIDAR C1",
        "why": ["accuracy specified (+/-30 mm) from 0.05 m, so the 15 cm stop zone (222 mm from the LiDAR) is inside "
                "spec; LD19 has no accuracy spec below 300 mm",
                "0.72 deg vs 1.0 deg: 2 beams per 5 cm cell out to 2.0 m (LD19 1.4 m)",
                "official, maintained ROS 2 driver (sllidar_ros2) with a C1 launch file vs vendor/community LD19 driver",
                "EUR 81 vs 143 (EU shops, Nov 2026 listings)"],
        "against": ["+63 g mass 15 cm above the floor (CoM +3.5 mm up; all stability margins still >= 3.5x, a6)",
                    "41.3 mm tall vs 34.8 mm, 55.6 mm square base; VCC window 4.8-5.2 V is tighter than LD19's 4.5-5.5 V "
                    "-> feed it from the Pi USB (regulated) or straight from the D36V50F5, never via a long thin lead"],
    }
    # ---- scan plane (CAD fit check)
    scan = {}
    for mdl in ("scout_v2", "scout_v2__c1_hiwonder_nopay"):
        if mdl not in fc:
            continue
        lc = fc[mdl]["lidar_check"]
        scan[mdl] = {"scan_z_mm": lc["scan_plane_z_mm"], "margin_mm": lc["margin_mm"],
                     "highest_other": lc["highest_other_part"], "dip_tolerance": lc["dip_tolerance_deg"][:2],
                     "margin_over_payload_keepout_mm": lc.get("margin_over_payload_keepout_mm"),
                     "unintended_overlaps": fc[mdl]["overlaps_unintended"]}
    res["scan_plane_cad"] = scan
    # payload / plant shadow: cup d 75 at 81.4 mm horizontal from the LiDAR axis
    d_cup, r_cup = LIDAR_X - payload_x(), 0.0375
    res["payload_shadow"] = {
        "sector_deg_if_anything_rises_above_scan": 2 * math.degrees(math.asin(r_cup / d_cup)),
        "plant_height_allowed_above_cup_rim_mm": scan.get("scout_v2", {}).get("margin_over_payload_keepout_mm"),
        "verdict": f"a seedling in the cup that grows > {scan.get('scout_v2', {}).get('margin_over_payload_keepout_mm')} mm above the 80 mm rim blinds a "
                   f"~{2 * math.degrees(math.asin(r_cup / d_cup)):.0f} deg rear sector; "
                   "either cap the payload envelope at 80 mm total height (REQ) or accept and filter a documented "
                   "rear blind sector (laser angular-bounds filter) -- and never reverse blind",
    }
    # pitch -> floor hits
    pitch = {}
    for th in (1.5, 3.0, 5.0, 7.18):
        z = SCAN_Z
        pitch[f"{th}deg"] = {"floor_hit_range_m": z / math.tan(math.radians(th))}
    res["pitch_floor_hits"] = {"cases": pitch, "verdict": "climbing the threshold pitches the robot 7.2 deg: the scan "
                               "plane hits the floor 1.2 m away -> phantom obstacles / map corruption. Gate scans when "
                               "|IMU pitch| > 1.5 deg (filter node) and set slam_toolbox minimum_travel to skip them."}
    # ---- camera
    cam = fc.get("scout_v2", {}).get("camera_check", {})
    res["camera"] = {
        "cad": cam,
        "floor_requirement": f"floor visible <= 150 mm ahead of the front: CAD r3 gives {cam.get('nearest_visible_floor_ahead_of_robot_front_mm')} mm -> "
                             f"{'PASS' if cam.get('pass') else 'FAIL'}",
        "occlusion": "lower FOV edge clears the camera mount by 9.4 mm and does not hit the HC-SR04 (it hangs under the "
                     "top plate, below the lens line); check the real holder's front lip",
        "urdf": "round 4: urdf/scout.urdf camera_joint = robot_params sensors.camera (rpy 0 +0.2618 0, xyz 0.1105 0 0.0796); verified",
        "fps": {"yolo26n_ncnn_640_ms": 67.0, "fps_640_inference_only": 1000 / 67.0,
                "ultralytics_quoted_fps_640": 7.79,
                "fps_320_estimate": "~4x fewer FLOPs -> 25-40 FPS inference-only (A: scaling, not measured)",
                "with_slam_nav2_load": "expect 0.5-0.7x when 1-2 cores are busy with SLAM/Nav2 (A)",
                "verdict": "REQ-SCOUT-010 (>= 10 FPS) FAILS at 640 px (7.8-15 FPS before other load) and PASSES at "
                           "320 px (est. 12-28 FPS under load). Use 320-416 px input, NCNN, 4 threads max 3.",
                "src": SRC["yolo_pi5"]},
    }
    # ---- HC-SR04
    res["ultrasonic"] = {
        "supply": "5 V only (datasheet 5 V DC, 15 mA); ECHO is a 5 V output -> divider to the ESP32 (not 5 V tolerant): "
                  "1 k (series) / 2 k (to GND) -> 3.33 V; TRIG accepts 3.3 V logic in practice (VIH not specified)",
        "range": "2-400 cm, 15 deg measuring angle (datasheet) -> blind < 2 cm, but reliable readings start ~3-5 cm (A)",
        "rate": "60 ms measurement cycle recommended (datasheet) -> 16 Hz max; at 0.3 m/s the robot moves 18 mm per ping",
        "coverage": "beam centre z ~0.049 m (CAD, under the top plate); 15 deg cone covers z 0.029-0.069 m at 15 cm -> "
                    "catches low obstacles the LiDAR (z 0.151 m) misses: shoes, pet bowls, door stops",
        "misses": "soft fabric, thin chair legs at an angle, objects > 7.5 deg off-axis; never sole safety sensor",
        "src": SRC["hcsr04"],
    }
    # ---- obstacle classes vs sensor heights
    obst = {"chair leg": (0.0, 0.45), "chair rung": (0.15, 0.18), "shoe": (0.0, 0.10), "door stop": (0.0, 0.05),
            "cable on floor": (0.0, 0.01), "small plant pot": (0.0, 0.12), "sofa front rail (gap 8-15 cm)": (0.10, 0.40),
            "kick-plate gap": (0.12, 0.30)}
    sz = SCAN_Z
    top = sz + 0.012
    cls = {}
    for k, (b, t) in obst.items():
        if b <= sz <= t:
            c = "LiDAR"
        elif t < sz:
            c = "below scan: HC-SR04 / camera" + (" (also below sonar cone)" if t < 0.02 else "")
        elif b < top:
            c = "OVERHANG: robot hits it, LiDAR blind"
        else:
            c = "above robot"
        cls[k] = {"bottom": b, "top": t, "class": c}
    res["obstacles"] = cls
    save_section("sensors", r3(res, 4))

    fig, (a, b) = plt.subplots(1, 2, figsize=(14, 4.8))
    rng = np.linspace(0.2, 6, 100)
    for k, l in LIDARS.items():
        a.plot(rng, 1000 * rng * math.radians(l["ang_res_deg"]), label=f"{k} {l['ang_res_deg']} deg")
    a.axhline(50, color="r", ls="--", label="5 cm map cell"); a.axhline(25, color="orange", ls=":", label="2 beams / cell")
    a.set_xlabel("range [m]"); a.set_ylabel("beam spacing [mm]"); a.legend(fontsize=8); a.grid(alpha=.3)
    a.set_title("LiDAR angular sampling vs 5 cm SLAM grid")
    for i, (k, v) in enumerate(cls.items()):
        col = "g" if v["class"] == "LiDAR" else "r" if "OVERHANG" in v["class"] else "orange"
        b.plot([i, i], [v["bottom"] * 100, v["top"] * 100], lw=8, color=col, solid_capstyle="butt")
    b.axhline(sz * 100, color="b", ls="--", label=f"scan plane {SCAN_Z*100:.1f} cm (CAD r3)")
    b.axhspan(2.9, 6.9, color="purple", alpha=.15, label="HC-SR04 cone at 15 cm")
    b.set_xticks(range(len(cls))); b.set_xticklabels(list(cls), rotation=35, ha="right", fontsize=7)
    b.set_ylim(0, 50); b.set_ylabel("height [cm]"); b.legend(fontsize=7); b.set_title("Obstacles vs sensor heights")
    fig.tight_layout(); fig.savefig(HERE / "a8_sensors.png", dpi=110)
    print(json.dumps(r3({k: res[k] for k in ("lidar_vs_req", "payload_shadow", "pitch_floor_hits", "obstacles")}, 3), indent=1))


if __name__ == "__main__":
    main()
