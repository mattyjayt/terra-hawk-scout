"""Write sim/params_v1_fallback.json: v1 layout numbers converted to the v2 REP-103 schema.

Used only until the CAD engineer writes out/robot_params.json. Every number comes from
v1-reference/src/scout_dims.py, v1-reference/checks/fit_check.py (MASS_G) or the BRIEF
fixed inputs; the source tag says which.

v1 frame: +Y forward, +X right, Z=0 at the bottom-plate underside, origin at the plate centre.
v2 frame (REP-103 base_footprint): +X forward, +Y left, Z=0 on the ground, origin at the
centre of rotation (midpoint of the axles, v1 y = -2).
  axle height above v1 Z0 = bottom plate top (4.5) + TT motor half height (11) = 15.5 mm
  ground below v1 Z0      = 32.5 - 15.5 = 17.0 mm  (checks: v1 LiDAR scan 114.2 -> 131.2 above ground)
  => X = y_v1 + 2, Y = -x_v1, Z = z_v1 + 17   (mm)
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "v1-reference" / "src"))
import scout_dims as D  # noqa: E402

CY = D.CENTRE_OF_ROTATION[1]          # -2
GROUND = D.WHEEL_D / 2 - (D.BOTTOM_Z[1] + D.MOTOR[2] / 2)   # 17.0


def to_rep(x, y, z):
    return [(y - CY) / 1000.0, -x / 1000.0, (z + GROUND) / 1000.0]


def box_inertia(m, sx, sy, sz):
    return [m / 12 * (sy**2 + sz**2), m / 12 * (sx**2 + sz**2), m / 12 * (sx**2 + sy**2), 0.0, 0.0, 0.0]


def part(mass_g, size_v1, x, y, z0, source, note=""):
    """size_v1 = (X right, Y fwd, Z) mm, centred on (x,y), bottom at z0 (v1 frame)."""
    sx, sy, sz = size_v1
    c = to_rep(x, y, z0 + sz / 2)
    size = [sy / 1000, sx / 1000, sz / 1000]          # REP X = v1 Y extent
    m = mass_g / 1000
    return {
        "mass": m, "com": c, "inertia": box_inertia(m, *size),
        "bbox_min": [c[i] - size[i] / 2 for i in range(3)],
        "bbox_max": [c[i] + size[i] / 2 for i in range(3)],
        "mass_source": source, "note": note,
    }


def build():
    top = D.TOP_Z[1]
    mx = D.SPINE_HALF - 1.0 - D.MOTOR[0] / 2
    links = {
        "bottom_plate": part(115, (D.PLATE_X, D.PLATE_Y, D.PLATE_T), 0, 0, 0, "cad-volume×density", "v1 fit_check: acrylic 1.19 g/cm3 assumed"),
        "top_plate": part(113, (D.PLATE_X, D.PLATE_Y, D.PLATE_T), 0, 0, D.TOP_Z[0], "cad-volume×density", "v1 fit_check"),
        "dead_plate": part(3, (40, 30, D.DEAD_Z[1] - D.DEAD_Z[0]), 0, 100, D.DEAD_Z[0], "cad-volume×density", "v1 fit_check; footprint assumed 40x30"),
        "battery": part(160, (70, 50, 15), 0, CY, D.BOTTOM_Z[1], "assumed",
                        "BRIEF: 70x50x15 measured; 33 Wh at ~200 Wh/kg pouch -> ~160 g ASSUMED, weigh it"),
        "esp32": part(10, D.ESP32, -12.0, 57.5, D.BOTTOM_Z[1] + D.STANDOFF, "assumed", "v1 MASS_G"),
        "tb6612": part(3, D.TB6612, 15.5, 50.0, D.BOTTOM_Z[1] + D.STANDOFF, "assumed", "v1 MASS_G"),
        "ultrasonic": part(9, D.HCSR04, 0, 103.5, D.BOTTOM_Z[1], "assumed", "v1 MASS_G"),
        "buck_5v": part(25, D.BUCK, 0, -65.0, D.BOTTOM_Z[1], "assumed", "v1 MASS_G"),
        "imu": part(2, D.IMU, 0, CY, D.TOP_Z[0] - D.IMU[2], "assumed", "v1 MASS_G; hangs under top plate"),
        "pi5": part(66, (85, 56, 20), 0, D.PI_Y, top + D.PI_STANDOFF, "assumed", "v1 MASS_G pi5 46 + cooler 20; 85x56 board envelope"),
        "camera": part(4, D.CAMERA, 0, 104.0, D.DEAD_Z[1], "assumed", "v1 MASS_G"),
        "lidar_mast": part(15, (D.MAST[0], D.MAST[1], 87.2 - top), 0, D.MAST_Y, top, "assumed",
                           "v1 printed mast - NOT allowed in v2 (off-the-shelf only); placeholder"),
        "lidar_platform": part(12, D.PLATFORM, 0, CY, 87.2, "assumed", "v1 placeholder"),
        "lidar": part(47, D.LIDAR, 0, CY, 90.2, "assumed", "LD19 ~47 g (v1 MASS_G)"),
    }
    for name, ay, outward in (("f", D.AXLE_Y_FRONT, 1), ("r", D.AXLE_Y_REAR, -1)):
        my = ay + outward * (D.MOTOR[1] / 2 - D.MOTOR_SHAFT_FROM_END)
        for side, s in (("l", -1), ("r", 1)):
            links[f"motor_{name}{side}"] = part(30, D.MOTOR, s * mx, my, D.BOTTOM_Z[1], "assumed", "TT motor ~30 g (v1 MASS_G)")

    wx = D.SPINE_HALF + D.WHEEL_GAP + D.WHEEL_W / 2         # 62 mm
    zc = D.BOTTOM_Z[1] + D.MOTOR[2] / 2
    r = D.WHEEL_D / 2000
    wheels = {}
    for key, ay in (("f", D.AXLE_Y_FRONT), ("r", D.AXLE_Y_REAR)):
        for side, s in (("l", -1), ("r", 1)):
            k = ("f" if key == "f" else "r") + ("l" if side == "l" else "r")
            k = {"fl": "fl", "fr": "fr", "rl": "rl", "rr": "rr"}[k]
            m = 0.025
            w = D.WHEEL_W / 1000
            wheels[k] = {"pos": to_rep(s * wx, ay, zc), "radius": r, "width": w, "mass": m,
                         "inertia": [m / 12 * (3 * r * r + w * w), m / 2 * r * r, m / 12 * (3 * r * r + w * w), 0, 0, 0],
                         "mass_source": "assumed (v1 MASS_G 25 g)",
                         "pos_source": "v1 scout_dims: axle y +38/-42, x +-62, axle z 15.5"}
    wheel_links = {f"wheel_{k}": {"mass": v["mass"], "com": v["pos"], "inertia": v["inertia"],
                                  "mass_source": v["mass_source"]} for k, v in wheels.items()}

    total = sum(l["mass"] for l in links.values()) + sum(w["mass"] for w in wheels.values())
    com = [sum(l["mass"] * l["com"][i] for l in list(links.values()) + list(wheel_links.values())) / total for i in range(3)]

    scan_z = to_rep(0, CY, 90.2 + D.LIDAR_SCAN_ABOVE_BASE)[2]
    px0, px1 = to_rep(0, -D.PLATE_Y / 2, 0)[0], to_rep(0, D.PLATE_Y / 2, 0)[0]
    wy = (wx + D.WHEEL_W / 2) / 1000
    return {
        "version": "v1-fallback", "frame": "REP-103 base_footprint, metres, kg",
        "source": "converted from v1-reference/src/scout_dims.py + checks/fit_check.py MASS_G by sim/make_fallback_params.py",
        "variant": {"lidar": "ld19", "motor": "tt_1_48_6v (stock, no encoder)", "battery": "blue_3s_70x50x15"},
        "total_mass": total, "com": com,
        "links": links, "wheels": wheels,
        "track_width": 2 * wx / 1000, "wheelbase": (D.AXLE_Y_FRONT - D.AXLE_Y_REAR) / 1000,
        "sensors": {
            "laser": {"pos": [0.0, 0.0, scan_z], "rpy": [0, 0, 0], "source": "v1 LD19 scan plane (optical centre 24 mm above base, assumed)"},
            "camera": {"pos": to_rep(0, 104.0 + D.CAMERA[1] / 2, D.DEAD_Z[1] + D.CAMERA[2] / 2), "rpy": [0, 0, 0],
                       "source": "v1 camera block front-face centre; pitch 0 ASSUMED (CAD to set tilt for 15 cm floor view)"},
            "imu": {"pos": to_rep(0, CY, D.TOP_Z[0] - D.IMU[2] / 2), "rpy": [0, 0, 0], "source": "v1 IMU under top plate over CoR"},
            "ultrasonic": {"pos": to_rep(0, 103.5 + D.HCSR04[1] / 2, D.BOTTOM_Z[1] + D.HCSR04[2] / 2), "rpy": [0, 0, 0],
                           "source": "v1 HC-SR04 front-face centre"},
        },
        "payload": {"mass": 0.300, "com": to_rep(0, -85.0, top + 30.0), "size": [0.060, 0.080, 0.060],
                    "source": "BRIEF payload <=300 g; v1 payload bay 80x60x60 centre (rear upper deck)"},
        "footprint": [[px1, wy], [px0, wy], [px0, -wy], [px1, -wy]],
        "lidar_scan_plane_z": scan_z,
    }


if __name__ == "__main__":
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "params_v1_fallback.json"
    p = build()
    out.write_text(json.dumps(p, indent=1))
    print(f"wrote {out}: mass {p['total_mass']:.3f} kg, com {[round(c, 4) for c in p['com']]}, "
          f"track {p['track_width']:.3f} wheelbase {p['wheelbase']:.3f}")
