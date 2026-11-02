"""Scout v2 fit check (round 3): overlaps, clearances, LiDAR scan plane + blind sectors, camera floor view,
ground clearance on a 10 mm step, mass / CoM / inertia (empty + payload), static wheel loads, motor fit,
cable lengths. Measures the models as built, for every variant.

Run:  ~/.local/bin/cadgen +0.8.4 run checks/fit_check_v2.py [variant names...]
      -> checks/fit_check_v2.json (all variants) and checks/results/<variant>.json (one per variant)
"""
import itertools
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))

import build123d as bd
from cadgen.geometry import closest_points, mass_properties, overlap_volume

import dims as D
from cables import CABLE_CHOICE
from scout_v2 import DEFAULT, VARIANTS, Geo, drills, lidar_leg_xy, scout_v2, tower_length
from variants import MODELS

G = 9.81
SLACK = 1.15
# overlaps accepted on purpose (envelope approximations), with the reason
INTENDED = {
    frozenset(("pi5", "pi5_cooler")): "cooler is a solid envelope; the real heatsink has cut-outs over the board's tall parts",
}
REFERENCE_ONLY = {"payload"}     # keep-out: weighed separately, not part of the empty robot
POWER = ("battery", "buck_5v", "cap_buck", "mdd3a", "cap_driver", "tb6612", "esp32", "fuse_holder", "xt30", "switch")
REQ_GAP = 2.0


# ------------------------------------------------------------------ geometry access
def walk(node, parent=None):
    here = (bd.Location() if parent is None else parent) * node.location
    if node.children:
        for c in node.children:
            yield from walk(c, here)
    else:
        yield node, node.located(here)


def density_kg_mm3(leaf):
    m = leaf.material
    for get in (lambda: m.material.density, lambda: m.density):
        try:
            return float(get()) * 1e-9
        except Exception:
            pass
    raise ValueError(f"no density on {leaf.label}")


def groups(tree):
    """top-level label -> list of (world solid, density kg/mm3)"""
    out = {}
    for child in tree.children:
        items = []
        for orig, placed in walk(child, tree.location):
            d = density_kg_mm3(orig)
            items += [(s, d) for s in placed.solids()]
        out[child.label] = items
    return out


def bbox_of(items):
    bbs = [s.bounding_box() for s, _ in items]
    lo = [min(getattr(b.min, a) for b in bbs) for a in "XYZ"]
    hi = [max(getattr(b.max, a) for b in bbs) for a in "XYZ"]
    return lo, hi


def boxes_gap(a, b):
    d2 = 0.0
    for i in range(3):
        g = max(a[0][i] - b[1][i], b[0][i] - a[1][i], 0.0)
        d2 += g * g
    return math.sqrt(d2)


def min_dist(g, a, b):
    return min(closest_points(x, y).distance for x, _ in g[a] for y, _ in g[b])


# ------------------------------------------------------------------ checks
def interference(g, bbs):
    overlaps, intended, contacts, gaps = [], [], [], []
    keys = sorted(k for k in g if not k.startswith("cable_"))
    for a, b in itertools.combinations(keys, 2):
        lb = boxes_gap(bbs[a], bbs[b])
        if lb > 3.0:
            continue
        ov = 0.0
        if lb == 0.0:
            for x, _ in g[a]:
                bx = x.bounding_box()
                for y, _ in g[b]:
                    by = y.bounding_box()
                    if (bx.min.X > by.max.X or by.min.X > bx.max.X or bx.min.Y > by.max.Y or by.min.Y > bx.max.Y
                            or bx.min.Z > by.max.Z or by.min.Z > bx.max.Z):
                        continue
                    ov += overlap_volume(x, y)
        if ov > 1e-3:
            rec = {"a": a, "b": b, "volume_mm3": round(ov, 2)}
            key = frozenset((a, b))
            if key in INTENDED:
                rec["reason"] = INTENDED[key]
                intended.append(rec)
            else:
                overlaps.append(rec)
            continue
        dist = min_dist(g, a, b)
        if dist < 1e-3:
            contacts.append(f"{a} | {b}")
        else:
            gaps.append({"a": a, "b": b, "gap_mm": round(dist, 2)})
    gaps.sort(key=lambda r: r["gap_mm"])
    return overlaps, intended, contacts, gaps


def requirement_gaps(g, bbs, gaps, contacts):
    """Round-1 findings 1 + 2: power parts and the LiDAR tower need >= 2 mm to everything they do not sit on."""
    mounts = {"bottom_plate", "top_plate"} | {k for k in g if k.startswith(("mdd3a_standoff_",))}
    res = {"power_parts": [], "lidar_tower": []}
    import re

    def stacked(a, b):     # coaxial fastener stacks through a plate/board (screw -> standoff), not a clearance
        ia, ib = re.findall(r"_(\d+)$", a), re.findall(r"_(\d+)$", b)
        tower = ("lidar_leg_", "lidar_board_screw_", "lidar_leg_screw_")
        fence = ("fence_post_", "fence_nut_")
        return ia and ia == ib and ((a.startswith(tower) and b.startswith(tower)) or (a.startswith(fence) and b.startswith(fence)))
    for r in gaps:
        a, b = r["a"], r["b"]
        if stacked(a, b):
            continue
        if (a in POWER or b in POWER) and r["gap_mm"] < 10:
            res["power_parts"].append(r)
        if (a.startswith("lidar_leg") or b.startswith("lidar_leg")) and r["gap_mm"] < 10:
            res["lidar_tower"].append(r)
    for c in contacts:
        a, b = c.split(" | ")
        for p in POWER:
            if p in (a, b):
                other = b if a == p else a
                if other not in mounts and not other.startswith(("mdd3a_standoff_",)):
                    res["power_parts"].append({"a": a, "b": b, "gap_mm": 0.0})
    res["power_parts"].sort(key=lambda r: r["gap_mm"])
    res["lidar_tower"].sort(key=lambda r: r["gap_mm"])
    res["power_min_gap_mm"] = res["power_parts"][0]["gap_mm"] if res["power_parts"] else None
    res["lidar_tower_min_gap_mm"] = res["lidar_tower"][0]["gap_mm"] if res["lidar_tower"] else None
    res["pass"] = ((res["power_min_gap_mm"] is None or res["power_min_gap_mm"] >= REQ_GAP)
                   and (res["lidar_tower_min_gap_mm"] is None or res["lidar_tower_min_gap_mm"] >= REQ_GAP))
    res["power_parts"] = res["power_parts"][:8]
    res["lidar_tower"] = res["lidar_tower"][:6]
    return res


def lidar_checks(g, bbs, scan_z, lidar_key):
    cx, cy = D.LIDAR_X, 0.0
    tops = {k: hi_[2] for k, (lo_, hi_) in bbs.items() if k != "lidar"}
    highest = max(tops, key=tops.get)
    tl, stack = tower_length(lidar_key)
    res = {"scan_plane_z_mm": round(scan_z, 2), "lidar_centre_xy_mm": [cx, cy],
           "tower_leg_length_mm": tl, "tower_leg_stack_mm": list(stack),
           "highest_other_part": highest, "highest_other_z_mm": round(tops[highest], 2),
           "margin_mm": round(scan_z - tops[highest], 2), "required_margin_mm": D.SCAN_MARGIN,
           "margin_over_payload_keepout_mm": round(scan_z - (Geo_from(bbs).T1 + D.PAYLOAD_H), 2)}

    def sector(k):
        (x0, y0, _), (x1, y1, _) = bbs[k]
        angs = [math.degrees(math.atan2(y - cy, x - cx)) for x in (x0, x1) for y in (y0, y1)]
        return [round(min(angs), 1), round(max(angs), 1)]
    res["blind_sectors_deg"] = [{"part": k, "azimuth_deg": sector(k)} for k, t in tops.items() if t >= scan_z]
    tilt = []
    for k, ((x0, y0, _), (x1, y1, z1)) in bbs.items():
        if k in ("lidar", "lidar_board", "lidar_adapter") or k.startswith(("lidar_board_screw", "cable_lidar")):
            continue
        dx = max(x0 - cx, 0, cx - x1)
        dy = max(y0 - cy, 0, cy - y1)
        r = math.hypot(dx, dy)
        if r < 1e-6:
            continue
        tilt.append((math.degrees(math.atan2(scan_z - z1, r)), k, r))
    tilt.sort()
    res["dip_tolerance_deg"] = [{"part": k, "deg": round(a, 2), "horizontal_distance_mm": round(r, 1)} for a, k, r in tilt[:5]]
    res["c1_scan_flatness_spec_deg"] = 1.5
    res["sectors_blocked_at_1p5deg_dip"] = [{"part": k, "azimuth_deg": sector(k)} for a, k, r in tilt if a < 1.5]
    res["note"] = ("scan plane above every robot part -> no self-occlusion sectors; the only near-plane object is the "
                   "payload keep-out (worst case Ø75 x 80 cup) which a dip of the listed angle would graze")
    return res


_GEO = {}


def Geo_from(bbs):
    return _GEO["geo"]


def camera_checks(g, bbs):
    t = math.radians(D.CAMERA_TILT_DEG)
    n = bd.Vector(math.cos(t), 0, -math.sin(t))
    lens_area = D.CAMERA_LENS_BLOCK[1] * D.CAMERA_LENS_BLOCK[2]
    faces = [f for s, _ in g["camera"] for f in s.faces()
             if f.geom_type == bd.GeomType.PLANE and f.normal_at().dot(n) > 0.999 and abs(f.area - lens_area) < 1.0]
    assert len(faces) == 1, len(faces)
    lens = faces[0].center()
    hfov, vfov = D.CAMERA_FOV
    low = D.CAMERA_TILT_DEG + vfov / 2
    up = D.CAMERA_TILT_DEG - vfov / 2
    near = lens.Z / math.tan(math.radians(low))
    front_x = max(hi[0] for k, (lo, hi) in bbs.items() if k not in ("payload",) and not k.startswith("cable_"))
    hits = set()
    for i in range(-4, 5):
        az = math.radians(hfov / 2 * i / 4)
        el = -math.radians(low)
        d = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
        for k, (lo, hi) in bbs.items():
            if k in ("camera", "camera_mount") or k.startswith("cable_camera"):
                continue
            tmin, tmax = 1e-3, 1e4
            p = (lens.X, lens.Y, lens.Z)
            ok = True
            for a in range(3):
                if abs(d[a]) < 1e-12:
                    if not (lo[a] <= p[a] <= hi[a]):
                        ok = False
                        break
                else:
                    t1, t2 = (lo[a] - p[a]) / d[a], (hi[a] - p[a]) / d[a]
                    tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            if ok and tmin <= tmax and p[2] + d[2] * tmin > 0.5:
                hits.add(k)
    mlo, mhi = bbs["camera_mount"]
    base_clear = lens.Z - (mlo[2] + D.CAMERA_MOUNT[2]) - (mhi[0] - lens.X) * math.tan(math.radians(low))
    ahead = lens.X + near - front_x
    return {"lens_centre_mm": [round(lens.X, 2), round(lens.Y, 2), round(lens.Z, 2)],
            "pitch_down_deg": D.CAMERA_TILT_DEG, "fov_deg_h_v": [hfov, vfov],
            "lower_edge_below_horizon_deg": low, "upper_edge_deg": round(-up, 1),
            "nearest_visible_floor_from_lens_mm": round(near, 1),
            "nearest_visible_floor_x_mm": round(lens.X + near, 1),
            "nearest_visible_floor_ahead_of_robot_front_mm": round(ahead, 1),
            "robot_front_x_mm": round(front_x, 1),
            "requirement": "floor visible <= 150 mm ahead of the robot front (BRIEF: sees the floor >= 15 cm ahead)",
            "pass": ahead <= 150.0,
            "lower_fov_edge_hits_parts_aabb": sorted(hits),
            "lower_edge_clearance_over_mount_base_mm": round(base_clear, 1)}


def ground_checks(bbs, geo):
    under = {k: lo[2] for k, (lo, hi) in bbs.items() if not k.startswith(("wheel_", "cable_"))}
    k = min(under, key=under.get)
    clr = under[k]
    wb = 2 * D.AXLE_X
    r = D.WHEEL_D / 2
    pitch = math.asin(10.0 / wb)
    x_rear_end = min(lo[0] for kk, (lo, hi) in bbs.items() if not kk.startswith(("wheel_", "cable_")))
    x_front_end = max(hi[0] for kk, (lo, hi) in bbs.items() if not kk.startswith(("wheel_", "cable_")) and kk != "payload")
    # front wheels on top of a 10 mm step, rear wheels on the floor: underside line rises from the rear axle
    rear_drop = (abs(x_rear_end) - D.AXLE_X) * math.sin(pitch)
    # belly over the step edge: worst point is just behind the front wheel's contact patch when the front wheel
    # has climbed (contact on the step top at x_f); the edge is then at most r*sin(asin((r-10)/r)) behind it.
    edge_behind = math.sqrt(r * r - (r - 10.0) ** 2)            # horizontal distance wheel centre -> edge at first touch
    belly_at_edge = clr + (wb - edge_behind) * math.sin(pitch) - 10.0
    return {"lowest_non_wheel_part": k, "ground_clearance_mm": round(clr, 2), "threshold_mm": 10.0,
            "margin_mm": round(clr - 10.0, 2),
            "pitch_with_front_wheels_on_10mm_step_deg": round(math.degrees(pitch), 2),
            "rear_overhang_tail_clearance_on_step_mm": round(clr - rear_drop, 2),
            "front_overhang_approach_clearance_mm": round(clr - 10.0, 2),
            "belly_clearance_over_step_edge_mm": round(belly_at_edge, 2),
            "wheel_contact_to_edge_at_first_touch_mm": round(edge_behind, 1),
            "front_overhang_mm": round(x_front_end - D.AXLE_X, 1), "rear_overhang_mm": round(abs(x_rear_end) - D.AXLE_X, 1),
            "motor_raise_spacer_mm": geo.raise_,
            "pass": clr - 10.0 >= 2.0,
            "note": "geometric only (wheel R 32.5 >> 10 mm step); traction/torque is the physics engineer's"}


def mass_block(g, include):
    items = [(s, d) for k in include for s, d in g[k]]
    mp = mass_properties(items)
    c = mp.center_of_mass
    return mp.mass, (c.X, c.Y, c.Z), mp.inertia


def part_props(g, k):
    mp = mass_properties(g[k])
    c = mp.center_of_mass
    I = [[float(v) for v in row] for row in mp.inertia]
    return mp.mass, (c.X, c.Y, c.Z), I


def wheel_loads(m, com):
    wb, tw = 2 * D.AXLE_X, 2 * D.WHEEL_Y
    front = (com[0] + D.AXLE_X) / wb
    left = (com[1] + D.WHEEL_Y) / tw
    W = m * G
    return {"front_share": round(front, 3), "rear_share": round(1 - front, 3),
            "left_share": round(left, 3), "right_share": round(1 - left, 3),
            "per_wheel_N": {"fl": round(W * front * left, 2), "fr": round(W * front * (1 - left), 2),
                            "rl": round(W * (1 - front) * left, 2), "rr": round(W * (1 - front) * (1 - left), 2)}}


def motor_fit(g, bbs, overlaps, gaps, contacts, motor_key, geo):
    m = D.MOTORS[motor_key]
    mot = [k for k in g if k in ("motor_fl", "motor_fr", "motor_rl", "motor_rr")]
    own = lambda k: k.startswith(("motor_bracket_", "motor_spacer_", "wheel_"))
    ov = [o for o in overlaps if o["a"] in mot or o["b"] in mot]
    near = [r for r in gaps if (r["a"] in mot or r["b"] in mot)]
    to_plate = round(min(min_dist(g, k, "bottom_plate") for k in mot), 2)
    to_top = round(min(min_dist(g, k, "top_plate") for k in mot), 2)
    near_other = [r for r in near if not own(r["b"] if r["a"] in mot else r["a"])]
    near_wheel = [r for r in near if (r["b"] if r["a"] in mot else r["a"]).startswith("wheel_")]
    cont = [c for c in contacts if any(x in c.split(" | ") for x in mot)]
    unexpected_contacts = [c for c in cont if not any(own(x) for x in c.split(" | ") if x not in mot)]
    plate_ok = to_plate >= D.ENCODER_PLATE_CLEARANCE - 1e-6 or (m["raise"] == 0 and not m["enc"])  # stock gearbox rests on the plate
    fits = not ov and plate_ok and not unexpected_contacts
    tower = {}
    for other in ("pi5", "pi5_cooler", "pi_standoff_1", "payload", "camera_mount", "deck_pillar_6"):
        if other in g:
            tower[other] = round(min(min_dist(g, f"lidar_leg_{i}", other) for i in range(1, 5)), 2)
    return {"motor": motor_key, "name": m["name"], "source": m["src"], "mass_g": m["mass_g"],
            "fits": fits, "overlaps": ov, "unexpected_contacts": unexpected_contacts,
            "motor_to_bottom_plate_mm": to_plate, "motor_to_top_plate_mm": to_top,
            "tightest_to_non_mount_parts": near_other[:4], "tightest_to_wheels": near_wheel[:2],
            "raise_spacer_mm": m["raise"], "cable_exit": m["cable"],
            "lidar_leg_min_distance_mm": tower,
            "motor_envelope_x_mm": [round(min(bbs[k][0][0] for k in mot), 1), round(max(bbs[k][1][0] for k in mot), 1)],
            "motor_envelope_y_abs_mm": [round(min(abs(bbs[k][0][1]) if bbs[k][0][1] > 0 else abs(bbs[k][1][1]) for k in mot), 1),
                                        round(max(max(abs(bbs[k][0][1]), abs(bbs[k][1][1])) for k in mot), 1)]}


def cable_report(g, bbs):
    out = []
    for k in sorted(g):
        if not k.startswith("cable_"):
            continue
        choice = CABLE_CHOICE.get(k) or next((v for p, v in CABLE_CHOICE.items() if p.endswith("_") and k.startswith(p)), "")
        out.append({"route": k, "path_length_mm": None, "with_slack_mm": None, "cable": choice})
    return out


def analyse(handle, spec):
    lidar_key, motor_key, with_payload, driver = spec
    geo = Geo(motor_key)
    _GEO["geo"] = geo
    tree = handle.shape
    g = groups(tree)
    bbs = {k: bbox_of(v) for k, v in g.items()}
    L = D.LIDARS[lidar_key]
    scan_z = bbs["lidar"][0][2] + L["scan_above_base"]
    overlaps, intended, contacts, gaps = interference(g, bbs)
    empty = [k for k in g if k not in REFERENCE_ONLY]
    m0, c0, I0 = mass_block(g, empty)
    res = {"model": handle.name, "id": handle.id,
           "variant": {"lidar": lidar_key, "motor": motor_key, "payload": with_payload, "driver": driver},
           "parts": len(g), "lidar": L["name"],
           "decks_mm": {"bottom_plate_top": round(geo.B1, 2), "top_plate_top": round(geo.T1, 2), "motor_raise": geo.raise_},
           "overlaps_unintended": overlaps, "overlaps_intended": intended,
           "contacts_count": len(contacts), "tightest_gaps_mm": gaps[:15],
           "required_clearances": requirement_gaps(g, bbs, gaps, contacts),
           "lidar_check": lidar_checks(g, bbs, scan_z, lidar_key), "camera_check": camera_checks(g, bbs),
           "ground_check": ground_checks(bbs, geo),
           "motor_fit": motor_fit(g, bbs, overlaps, gaps, contacts, motor_key, geo),
           "mass_empty_kg": round(m0, 4), "com_empty_mm": [round(v, 1) for v in c0],
           "inertia_empty_about_com_kg_m2": [[round(v * 1e-6, 7) for v in row] for row in I0],
           "wheel_loads_empty": wheel_loads(m0, c0)}
    if "payload" in g:
        m1, c1, I1 = mass_block(g, list(g))
        res.update({"mass_with_payload_kg": round(m1, 4), "com_with_payload_mm": [round(v, 1) for v in c1],
                    "inertia_with_payload_about_com_kg_m2": [[round(v * 1e-6, 7) for v in row] for row in I1],
                    "wheel_loads_with_payload": wheel_loads(m1, c1)})
    else:   # no keep-out part in this variant: add the 300 g cup analytically at the keep-out centroid
        mp, cp = D.PAYLOAD_MASS_G / 1000.0, (D.PAYLOAD_X, 0.0, geo.T1 + D.PAYLOAD_H / 2)
        m1 = m0 + mp
        c1 = tuple((m0 * a + mp * b) / m1 for a, b in zip(c0, cp))
        res.update({"mass_with_payload_kg": round(m1, 4), "com_with_payload_mm": [round(v, 1) for v in c1],
                    "with_payload_note": "300 g cup added analytically (point mass at the keep-out centroid)",
                    "wheel_loads_with_payload": wheel_loads(m1, c1)})
    ib = bbs["imu"]
    res["imu_frame_mm"] = {"chip_centre": [round((ib[0][0] + ib[1][0]) / 2, 2), round((ib[0][1] + ib[1][1]) / 2, 2),
                                           round(ib[0][2] + 0.45, 2)],
                           "distance_from_cor_xy_mm": round(math.hypot((ib[0][0] + ib[1][0]) / 2, (ib[0][1] + ib[1][1]) / 2), 2),
                           "rpy": [round(math.pi, 6), 0.0, 0.0], "note": "MPU-6050 chip facing down -> roll pi"}
    res["drilling_table"] = [{"part": p, "purpose": w, "x_mm": round(x, 2), "y_mm": round(y, 2),
                              "dia_mm": (list(d) if isinstance(d, tuple) else d)} for p, w, x, y, d in drills(lidar_key, driver)]
    req = {"deck_gap_39.7": round(bbs["top_plate"][0][2] - bbs["bottom_plate"][1][2], 3),
           "plate_t_4.5": round(bbs["top_plate"][1][2] - bbs["top_plate"][0][2], 3),
           "wheel_d_65": round(bbs["wheel_fl"][1][2] - bbs["wheel_fl"][0][2], 3),
           "battery_70x50x15": [round(bbs["battery"][1][i] - bbs["battery"][0][i], 3) for i in range(3)],
           "payload_keepout_d75_h80": ([round(bbs["payload"][1][0] - bbs["payload"][0][0], 3),
                                        round(bbs["payload"][1][2] - bbs["payload"][0][2], 3)] if "payload" in bbs else None),
           "payload_centre_x_-50": (round((bbs["payload"][1][0] + bbs["payload"][0][0]) / 2, 3) if "payload" in bbs else None),
           "board_minus_cooler_ge_3": round(bbs["lidar_board"][0][2] - bbs["pi5_cooler"][1][2], 2),
           "board_rear_edge_minus_cup_ge_5": (round(bbs["lidar_board"][0][0] - bbs["payload"][1][0], 2) if "payload" in bbs else None),
           "no_motor_buck_no_usb": not any(k in g for k in ("buck_motor", "lidar_adapter", "cable_lidar_pi"))}
    res["jaime_numbers_on_geometry"] = req
    res["mass_breakdown_g"] = {k: round(part_props(g, k)[0] * 1000, 1) for k in sorted(g)}
    return res, g, bbs, scan_z


class _V:
    def __init__(self, x, y, z):
        self.X, self.Y, self.Z = x, y, z


class _BB:
    def __init__(self, lo, hi):
        self.min, self.max = _V(*lo), _V(*hi)


def cable_lengths(spec, bbs):
    """Route lengths straight from the waypoints (the same function the model sweeps, fed the built bboxes)."""
    import scout_v2 as S
    from cables import cable_routes
    geo = Geo(spec[1])
    bb = {k: _BB(*v) for k, v in bbs.items()}
    ctx = {"geo": geo, "motor": spec[1], "lidar": spec[0], "driver": spec[3], "bb": bb,
           "pi_vendor_to_world": lambda xv, zv, yv=0.0: S.pi_vendor_to_world(geo, xv, zv, yv),
           "pi_pcb_top": S.pi_pcb_top(geo), "switch_xy": D.SWITCH_XY}
    out = []
    for label, kind, pts in cable_routes(ctx):
        L = sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))
        choice = CABLE_CHOICE.get(label) or next((v for p, v in CABLE_CHOICE.items() if p.endswith("_") and label.startswith(p)), "")
        out.append({"route": label, "kind": kind, "path_mm": round(L, 1), "with_15pct_slack_mm": round(L * SLACK, 1),
                    "from": [round(v, 1) for v in pts[0]], "to": [round(v, 1) for v in pts[-1]], "cable": choice})
    return out


def summary_line(r):
    lc, gc, rc = r["lidar_check"], r["ground_check"], r["required_clearances"]
    return (f"{r['model']}: overlaps={len(r['overlaps_unintended'])} {r['overlaps_unintended'][:3]} power_gap={rc['power_min_gap_mm']} "
            f"tower_gap={rc['lidar_tower_min_gap_mm']} scan_margin={lc['margin_mm']} ground={gc['ground_clearance_mm']} "
            f"motor_fits={r['motor_fit']['fits']} mass={r['mass_empty_kg']} com={r['com_empty_mm']} "
            f"pay={r.get('mass_with_payload_kg')} com_pay={r.get('com_with_payload_mm')} cam={r['camera_check']['nearest_visible_floor_ahead_of_robot_front_mm']} "
            f"tight={r['tightest_gaps_mm'][:4]}")


if __name__ == "__main__":
    out_dir = HERE / "results"
    out_dir.mkdir(exist_ok=True)
    names = [a for a in sys.argv[1:] if not a.startswith("-")] or (["scout_v2"] + list(VARIANTS))
    for name in names:
        if name == "scout_v2":
            spec = (DEFAULT["lidar"], DEFAULT["motor"], DEFAULT["payload"], DEFAULT["driver"])
            h = scout_v2()
        else:
            spec = VARIANTS[name]
            h = MODELS[name]()
        r, g, bbs, _ = analyse(h, spec)
        r["cables"] = cable_lengths(spec, bbs)
        (out_dir / f"{name}.json").write_text(json.dumps(r, indent=1))
        print(summary_line(r), flush=True)
    allr = [json.loads(p.read_text()) for p in sorted(out_dir.glob("*.json"))]
    allr.sort(key=lambda r: (r["model"] != "scout_v2", r["model"]))
    (HERE / "fit_check_v2.json").write_text(json.dumps(allr, indent=1))
    print("wrote", HERE / "fit_check_v2.json", len(allr), "variants")
