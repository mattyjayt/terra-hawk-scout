"""Write the shared interface file out/robot_params*.json (BRIEF §4) from the models as built.

Run:  ~/.local/bin/cadgen +0.8.4 run checks/export_params.py [variant names...]
  out/robot_params.json                 default model scout_v2 (RPLIDAR C1 + Hiwonder encoder TT 1:42 + MDD3A, payload on)
  out/robot_params_<variant>.json       every variant in scout_v2.VARIANTS
Units: metres, kg, kg.m^2; REP-103 base_footprint frame. Inertia is about each link's own CoM,
expressed in base_footprint axes: [ixx, iyy, izz, ixy, ixz, iyz]. Links exclude the payload (see 'payload').
"""
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(HERE))

import dims as D
from fit_check_v2 import _GEO, bbox_of, camera_checks, groups, mass_block, part_props, wheel_loads
from scout_v2 import DEFAULT, VARIANTS, Geo, scout_v2, tower_length
from variants import MODELS

MM = 1e-3
ROUND = 6

SOURCE = [  # (label prefix, mass_source)
    ("bottom_plate", "cad-volume×density"), ("top_plate", "cad-volume×density"),
    ("motor_bracket_", "cad-volume×density"), ("motor_spacer_", "cad-volume×density"), ("motor_", "datasheet"),
    ("wheel_", "assumed"), ("battery", "assumed"), ("buck_5v", "datasheet"), ("cap_buck", "assumed"),
    ("cap_driver", "assumed"), ("fence_", "cad-volume×density"), ("front_screw_", "cad-volume×density"),
    ("mdd3a_standoff_", "cad-volume×density"), ("mdd3a", "datasheet"), ("tb6612", "assumed"), ("esp32", "assumed"),
    ("imu_standoff_", "cad-volume×density"), ("imu", "assumed"), ("ultrasonic_bracket", "cad-volume×density"),
    ("ultrasonic", "assumed"), ("camera_mount", "cad-volume×density"), ("camera", "assumed"),
    ("pi5_cooler", "assumed"), ("pi5", "datasheet"), ("lidar_board", "cad-volume×density"),
    ("lidar_leg", "cad-volume×density"), ("lidar_screw_", "cad-volume×density"), ("lidar", "datasheet"),
    ("payload", "assumed"), ("fuse_holder", "assumed"), ("xt30", "assumed"), ("switch", "datasheet"),
    ("cable_", "assumed"),
]


def source_of(label):
    for p, s in SOURCE:
        if label == p or (p.endswith("_") and label.startswith(p)) or (not p.endswith("_") and label.startswith(p + "_") is False and label == p):
            return s
    for p, s in SOURCE:
        if label.startswith(p):
            return s
    return "cad-volume×density"    # standoffs, pillars, screws


def r(v):
    return round(float(v), ROUND)


def inertia6(I):
    return [r(I[0][0] * 1e-6), r(I[1][1] * 1e-6), r(I[2][2] * 1e-6), r(I[0][1] * 1e-6), r(I[0][2] * 1e-6), r(I[1][2] * 1e-6)]


def rot_to_rpy(R):
    """URDF rpy (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll, yaw = math.atan2(-R[1][2], R[1][1]), 0.0
    return [roll, pitch, yaw]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def footprint(g):
    from shapely.geometry import MultiPoint
    pts = []
    for k, items in g.items():
        if k == "payload" or k.startswith("cable_"):
            continue
        for s, _ in items:
            bb = s.bounding_box()
            pts += [(x, y) for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y)]
    hull = MultiPoint(pts).convex_hull.simplify(0.5)
    return [[r(x * MM), r(y * MM)] for x, y in list(hull.exterior.coords)[:-1]]


def params(handle, spec, path):
    lidar_key, motor_key, with_payload, driver = spec
    geo = Geo(motor_key)
    _GEO["geo"] = geo
    g = groups(handle.shape)
    bbs = {k: bbox_of(v) for k, v in g.items()}
    L = D.LIDARS[lidar_key]
    scan_z = bbs["lidar"][0][2] + L["scan_above_base"]
    links = {}
    for k in sorted(g):
        if k == "payload":
            continue
        m, c, I = part_props(g, k)
        lo, hi = bbs[k]
        links[k] = {"mass": r(m), "com": [r(v * MM) for v in c], "inertia": inertia6(I),
                    "bbox_min": [r(v * MM) for v in lo], "bbox_max": [r(v * MM) for v in hi],
                    "mass_source": source_of(k)}
    wheels = {}
    for w in ("fl", "fr", "rl", "rr"):
        lo, hi = bbs[f"wheel_{w}"]
        wheels[w] = {"pos": [r((lo[i] + hi[i]) / 2 * MM) for i in range(3)], "radius": r(D.WHEEL_D / 2 * MM),
                     "width": r(D.WHEEL_W * MM)}
    cam = camera_checks(g, bbs)["lens_centre_mm"]
    t = math.radians(D.CAMERA_TILT_DEG)
    R_opt = matmul(Ry(t), matmul(Rz(-math.pi / 2), Rx(-math.pi / 2)))
    imu_lo, imu_hi = bbs["imu"]
    us_lo, us_hi = bbs["ultrasonic"]
    sensors = {
        "laser": {"pos": [r((bbs["lidar"][0][0] + bbs["lidar"][1][0]) / 2 * MM), r((bbs["lidar"][0][1] + bbs["lidar"][1][1]) / 2 * MM),
                          r(scan_z * MM)], "rpy": [0.0, 0.0, 0.0],
                  "note": f"{L['name']} rotation centre on the scan plane; the device's cable/ear 'front' faces +X. "
                          "Native angles are clockwise (left-handed): the ROS driver converts; check angle 0 on the bench."},
        "camera": {"pos": [r(v * MM) for v in cam], "rpy": [0.0, r(t), 0.0],
                   "note": "camera_link (x forward) at the lens front-face centre, pitched down 15 deg. "
                           "FOV 66 x 41 deg (CM3 standard)."},
        "camera_optical": {"pos": [r(v * MM) for v in cam], "rpy": [r(v) for v in rot_to_rpy(R_opt)],
                           "note": "camera_optical_frame (z forward, x right, y down) = camera_link * rpy(-pi/2, 0, -pi/2), "
                                   "expressed here directly in base_footprint"},
        "imu": {"pos": [r((imu_lo[0] + imu_hi[0]) / 2 * MM), r((imu_lo[1] + imu_hi[1]) / 2 * MM), r((imu_lo[2] + 0.45) * MM)],
                "rpy": [r(math.pi), 0.0, 0.0],
                "note": "MPU-6050 die centre (mid-thickness of the 4 x 4 x 0.9 QFN). GY-521 hung chip-down on 2 x M3 x 10 nylon "
                        "standoffs under the top plate, board +X silk arrow pointing forward -> roll pi (sensor +Y = robot -Y, "
                        "+Z = robot -Z). 24 mm behind the CoR on the centreline."},
        "ultrasonic": {"pos": [r(us_hi[0] * MM), 0.0, r((us_lo[2] + us_hi[2]) / 2 * MM)], "rpy": [0.0, 0.0, 0.0],
                       "note": "HC-SR04 transducer front faces, between the two cans; beam ~ +-15 deg"},
    }
    empty = [k for k in g if k != "payload"]
    m0, c0, I0 = mass_block(g, empty)
    mot = D.MOTORS[motor_key]
    tl, stack = tower_length(lidar_key)
    out = {
        "version": "v2-r3", "frame": "REP-103 base_footprint, metres, kg",
        "variant": {"lidar": lidar_key, "motor": motor_key, "battery": "blue_3s_70x50x15", "driver": driver,
                    "payload_keepout": with_payload, "motor_name": mot["name"], "model": handle.name, "cad_id": handle.id},
        "total_mass": r(m0), "com": [r(v * MM) for v in c0], "inertia_about_com": inertia6(I0),
        "links": links, "wheels": wheels,
        "track_width": r(2 * D.WHEEL_Y * MM), "wheelbase": r(2 * D.AXLE_X * MM),
        "sensors": sensors, "footprint": footprint(g), "lidar_scan_plane_z": r(scan_z * MM),
        "decks": {"bottom_plate_top_z": r(geo.B1 * MM), "top_plate_top_z": r(geo.T1 * MM),
                  "motor_raise_spacer": r(geo.raise_ * MM), "lidar_tower_leg": r(tl * MM), "lidar_tower_stack_mm": list(stack)},
        "encoder": {k: mot[k] for k in ("ratio", "ppr_motor", "counts_per_wheel_rev_x4", "rpm_noload", "v_rated")},
        "encoder_source": mot["src"] + " (ratio/counts as used by analysis/results.json; verify on arrival)",
        "wheel_loads_empty": wheel_loads(m0, c0),
        "notes": "total_mass/com/links exclude the payload; see 'payload' and 'with_payload'. Cable tubes (links cable_*) "
                 "are an approximate harness (assumed g/m). Masses tagged 'assumed' must be weighed (docs/cad-round3.md). "
                 "laser frame = scan-plane centre; native LiDAR angle 0 points along the device's own front (check on the bench).",
    }
    if "payload" in g:
        m, c, I = part_props(g, "payload")
        lo, hi = bbs["payload"]
        out["payload"] = {"mass": r(m), "com": [r(v * MM) for v in c], "inertia": inertia6(I),
                          "bbox_min": [r(v * MM) for v in lo], "bbox_max": [r(v * MM) for v in hi],
                          "mass_source": "assumed", "shape": "cylinder keep-out D75 x 80 mm, uniform density"}
        m1, c1, I1 = mass_block(g, list(g))
        out["with_payload"] = {"total_mass": r(m1), "com": [r(v * MM) for v in c1], "inertia_about_com": inertia6(I1),
                               "wheel_loads": wheel_loads(m1, c1)}
    path.write_text(json.dumps(out, indent=1))
    print("wrote", path.name, "mass", out["total_mass"], "com", out["com"], "scan", out["lidar_scan_plane_z"], flush=True)


if __name__ == "__main__":
    names = [a for a in sys.argv[1:] if not a.startswith("-")] or (["scout_v2"] + list(VARIANTS))
    for name in names:
        if name == "scout_v2":
            params(scout_v2(), (DEFAULT["lidar"], DEFAULT["motor"], DEFAULT["payload"], DEFAULT["driver"]),
                   ROOT / "out" / "robot_params.json")
        else:
            tag = name.replace("scout_v2__", "").replace("scout_v2_", "")
            params(MODELS[name](), VARIANTS[name], ROOT / "out" / f"robot_params_{tag}.json")
