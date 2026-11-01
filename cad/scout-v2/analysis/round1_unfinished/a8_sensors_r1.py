"""8. Sensors: LiDAR scan plane vs indoor obstacles, min range, occlusion blind angles,
LD19 vs RPLIDAR C1, camera floor visibility."""
import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import SRC, r3, save_section

SCAN_Z = 0.131            # V1 fit check (scan plane clears payload bay by 5.5 mm)
US_Z, US_HALF = 0.0315, math.radians(7.5)    # HC-SR04 axis height (V1), ~15 deg cone (D, typical)

LIDARS = {
    "ld19": {"mass_g": 47, "size_mm": "54 x 46.8 x 34.8 (base 38.6 turret)", "height_mm": 34.8, "range_m": (0.02, 12),
             "acc_mm": 45, "acc_note": "+/-45 mm for 0.3-12 m; below 0.3 m accuracy unspecified",
             "rate_hz": 4500, "scan_hz": 10, "ang_res_deg": 0.8, "uart": "TX only, 230400, 3.3 V; PWM pin grounded",
             "power": "5 V, 180 mA work / 300 mA start", "ros2": "ldlidar_stl_ros2 (vendor, community forks for Jazzy)",
             "price_eur": "130.90 RobotShop EU (DFRobot SEN0547 kit); 142.80 Robot Italy; Waveshare D300 kit discontinued",
             "src": SRC["ld19"]},
    "c1": {"mass_g": 110, "size_mm": "55.6 x 55.6 x 41.3", "height_mm": 41.3, "range_m": (0.05, 12),
           "acc_mm": 30, "acc_note": "+/-30 mm, 15 mm resolution; black targets 6 m",
           "rate_hz": 5000, "scan_hz": 10, "ang_res_deg": 0.72, "uart": "TX+RX, 460800, 3.3 V; USB adapter in kit",
           "power": "5 V, 230 mA (1.15 W)", "ros2": "sllidar_ros2 (Slamtec official, C1 launch file)",
           "price_eur": "69.80 Amazon.be (youyeetoo) / 81.12 Generation Robots / 99.95 MYBOTSHOP",
           "src": SRC["c1"]},
}

# typical indoor obstacles: (bottom_m, top_m, note) -- ASSUMED typical values, measure your rooms
OBST = {
    "chair leg (vertical)": (0.0, 0.45, "visible"),
    "chair rung / stretcher": (0.15, 0.18, "A 15-25 cm"),
    "table leg": (0.0, 0.75, ""),
    "small plant pot": (0.0, 0.12, "A 10-14 cm"),
    "large plant pot": (0.0, 0.25, ""),
    "shoe": (0.0, 0.10, ""),
    "cable on floor": (0.0, 0.01, ""),
    "door stop": (0.0, 0.05, ""),
    "pet bowl": (0.0, 0.07, ""),
    "sofa underside (front rail)": (0.10, 0.40, "A gap 8-15 cm"),
    "bed / cabinet kick plate gap": (0.12, 0.30, "A"),
    "seedling cup on robot (payload)": (0.066, 0.126, "own payload must stay below scan plane"),
}


def main():
    robot_top = SCAN_Z + 0.012          # LiDAR top above scan plane (A ~ 10-17 mm)
    obs = {}
    for k, (b, t, n) in OBST.items():
        if b <= SCAN_Z <= t:
            cls = "LiDAR sees it"
        elif t < SCAN_Z:
            cls = "BELOW scan plane: ultrasonic/camera only" + (" (also below sonar)" if t < US_Z - 0.01 else "")
        elif b < robot_top:
            cls = "OVERHANG hazard: robot hits it, LiDAR blind"
        else:
            cls = "above robot"
        obs[k] = {"bottom": b, "top": t, "class": cls, "note": n}
    # occlusion blind angle of a vertical post crossing the scan plane
    blind = {}
    for d_mm in (3, 5.5, 8, 12):
        for r_mm in (20, 30, 45, 60):
            blind[f"d{d_mm}_r{r_mm}"] = math.degrees(2 * math.asin(d_mm / 2 / r_mm))
    # angular resolution -> beam spacing
    spacing = {k: {f"{rng}m_mm": 1000 * rng * math.radians(v["ang_res_deg"]) for rng in (1, 2, 4)} for k, v in LIDARS.items()}
    # min range vs stop zone: obstacle 15 cm ahead of the bumper, LiDAR at x=0 (v1) or x=-75 mm (proposed)
    bumper = 0.1174
    ranges = {lay: bumper - lx + 0.15 for lay, lx in (("v1_lidar_x0", 0.0), ("proposed_lidar_x-75", -0.075))}
    # camera: Camera Module 3 standard 66 x 41 deg, wide 102 x 67 deg (raspberrypi.com docs)
    cam = {}
    cam_x, cam_z = 0.106, 0.080
    for lens, vfov in (("standard", 41.0), ("wide", 67.0)):
        for tilt in (0, 5, 10, 15, 20):
            lo = math.radians(tilt + vfov / 2)
            near = cam_z / math.tan(lo)
            cam[f"{lens}_tilt{tilt}"] = {"near_floor_from_camera_m": near,
                                         "near_floor_ahead_of_bumper_m": near - (bumper - cam_x),
                                         "horizon_visible": tilt < vfov / 2}
    res = {"scan_plane_z_m": SCAN_Z, "robot_top_m_assumed": robot_top, "lidars": LIDARS, "obstacles": obs,
           "blind_angle_deg_post_diam_dist": blind, "beam_spacing": spacing,
           "range_to_stop_zone_edge_m": ranges, "camera_floor": cam,
           "occlusion_note": "with the LiDAR as the topmost part on standoffs BELOW its base, nothing crosses the scan "
                             "plane -> 0 deg blind; v1's printed mast/platform is not allowed (off-the-shelf rule)",
           "ultrasonic": {"axis_z_m": US_Z, "covers_z_at_0.15m": [US_Z - 0.15 * math.tan(US_HALF), US_Z + 0.15 * math.tan(US_HALF)]}}
    save_section("sensors", r3(res, 4))

    fig, ax = plt.subplots(figsize=(9, 4.8))
    for i, (k, v) in enumerate(obs.items()):
        col = "g" if v["class"].startswith("LiDAR") else "r" if "OVERHANG" in v["class"] else "orange"
        ax.plot([i, i], [v["bottom"] * 100, v["top"] * 100], lw=8, color=col, solid_capstyle="butt")
    ax.axhline(SCAN_Z * 100, color="b", ls="--", label="LiDAR scan plane 13.1 cm")
    ax.axhspan((SCAN_Z) * 100, robot_top * 100, color="b", alpha=.15, label="LiDAR body (blind overhang band)")
    ax.axhspan(res["ultrasonic"]["covers_z_at_0.15m"][0] * 100, res["ultrasonic"]["covers_z_at_0.15m"][1] * 100,
               color="purple", alpha=.15, label="HC-SR04 cone at 15 cm")
    ax.set_xticks(range(len(obs))); ax.set_xticklabels(list(obs), rotation=40, ha="right", fontsize=7)
    ax.set_ylim(0, 50); ax.set_ylabel("height [cm]"); ax.legend(fontsize=7)
    ax.set_title("Indoor obstacles vs sensor coverage (green = LiDAR, orange = below, red = overhang)")
    fig.tight_layout(); fig.savefig(HERE / "a8_sensors.png", dpi=110)
    print(json.dumps(r3({k: res[k] for k in ("obstacles", "beam_spacing", "range_to_stop_zone_edge_m", "camera_floor")}, 3), indent=1))


if __name__ == "__main__":
    main()
