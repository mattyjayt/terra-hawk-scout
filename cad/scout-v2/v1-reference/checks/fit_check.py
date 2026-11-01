"""Fit check for the Scout layout: interference, footprint overhang, LiDAR scan
clearance and combined centre of mass. Run: cadgen +0.8.4 run checks/fit_check.py"""
import itertools
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))

import build123d as bd
from cadgen.geometry import closest_points, overlap_volume
from shapely.geometry import Point, Polygon, box as sbox

import scout_dims as D
from cadgen import read_mesh
from plate_outline import ELEGOO_DIR, face_polygon
from scout_layout import scout_layout, scout_layout_tall_battery, to_robot

# masses in grams (ASSUMED rough values unless noted); plates use volume x density
MASS_G = {
    "motor": 30, "wheel": 25, "battery_3s": 150, "esp32_devkit": 10, "tb6612_driver": 3,
    "hc_sr04": 9, "buck_5v5a": 25, "mpu6050_imu": 2, "pi5_stack": 46 + 20, "pi_camera_3": 4,
    "lidar_ld19": 47, "lidar_mast": 15, "lidar_platform": 12, "payload_bay_keepout": 0,
}
PLATE_DENSITY = 1.19e-3   # g/mm^3, assumed acrylic
PAYLOAD_FULL_G = 250      # assumed full water reservoir + seedling cup
REFERENCE = {"lidar_scan_plane"}


def world_parts(node, parent=None):
    here = (bd.Location() if parent is None else parent) * node.location
    if node.children:
        for child in node.children:
            yield from world_parts(child, here)
    else:
        yield node.located(here)


def groups(tree):
    """Top-level occurrences: a part, or a sub-assembly (pi5_stack) as one group."""
    out = {}
    for child in tree.children:
        solids = list(world_parts(child, tree.location))
        out[child.label] = solids
    return out


def mass_of(label, solids):
    if label.endswith("plate"):
        return sum(s.volume for s in solids) * PLATE_DENSITY
    for key, m in MASS_G.items():
        if label.startswith(key):
            return m
    raise KeyError(label)


def centroid(solids):
    vol = sum(s.volume for s in solids)
    c = sum((s.center(bd.CenterOf.MASS) * s.volume for s in solids), bd.Vector()) / vol
    return c


def check(handle):
    tree = handle.shape
    g = groups(tree)
    res = {"model": handle.name, "id": handle.id}

    # 1. interference: every pair of top-level groups, bbox prefilter then exact
    clashes, tight = [], []
    labels = sorted(g)
    for a, b in itertools.combinations(labels, 2):
        sa = [s for p in g[a] for s in p.solids()]
        sb = [s for p in g[b] for s in p.solids()]
        ov = 0.0
        for x in sa:
            bx = x.bounding_box()
            for y in sb:
                by = y.bounding_box()
                if (bx.min.X > by.max.X or by.min.X > bx.max.X or bx.min.Y > by.max.Y or by.min.Y > bx.max.Y
                        or bx.min.Z > by.max.Z or by.min.Z > bx.max.Z):
                    continue
                ov += overlap_volume(x, y)
        if ov > 1e-6:
            clashes.append((a, b, round(ov, 2)))
    res["overlaps"] = clashes

    # minimum gaps of interest
    def gap(a, b):
        return round(min(closest_points(x, y).distance for x in g[a] for y in g[b]), 2)
    res["gaps_mm"] = {
        "battery-motor_front_left": gap("battery_3s", "motor_front_left"),
        "battery-motor_rear_left": gap("battery_3s", "motor_rear_left"),
        "battery-top_plate": gap("battery_3s", "top_plate"),
        "battery-imu": gap("battery_3s", "mpu6050_imu"),
        "wheel_front_left-top_plate": gap("wheel_front_left", "top_plate"),
        "wheel_rear_left-bottom_plate": gap("wheel_rear_left", "bottom_plate"),
        "pi5-lidar_mast": gap("pi5_stack", "lidar_mast"),
        "pi5-payload_bay": gap("pi5_stack", "payload_bay_keepout"),
        "lidar_scan_plane-payload_bay": gap("lidar_scan_plane", "payload_bay_keepout"),
    }

    # 2. footprint: XY envelope of each part vs the outer outline of the plate polygon
    outlines = {}
    for name, use_top in (("Top Plate.stl", True), ("Bottom Plate.stl", False)):
        mesh = read_mesh(ELEGOO_DIR / name)
        zs = [v[2] for v in mesh.vertices]
        poly = face_polygon(mesh.vertices, mesh.triangles, max(zs) if use_top else min(zs))
        outlines[name] = Polygon(to_robot(list(poly.exterior.coords)))   # outer edge, holes ignored
    over = {}
    for k, solids in g.items():
        if k in REFERENCE or k.endswith("plate"):
            continue
        xs = [s.bounding_box() for s in solids]
        fp = sbox(min(b.min.X for b in xs), min(b.min.Y for b in xs), max(b.max.X for b in xs), max(b.max.Y for b in xs))
        zmin = min(b.min.Z for b in xs)
        outline = outlines["Top Plate.stl"] if zmin >= D.TOP_Z[1] - 1 else outlines["Bottom Plate.stl"]
        outside = fp.difference(outline)
        if outside.area > 0.5:
            far = max(outline.distance(Point(x, y)) for x, y in fp.exterior.coords)
            over[k] = {"area_mm2": round(outside.area), "max_mm": round(far, 1)}
    res["overhangs_vs_own_deck_outline"] = over

    # 3. LiDAR scan plane vs every other part's top
    lidar_bb = g["lidar_ld19"][0].bounding_box()
    scan_z = lidar_bb.min.Z + D.LIDAR_SCAN_ABOVE_BASE
    tops = {k: round(max(s.bounding_box().max.Z for s in v), 2) for k, v in g.items()
            if k not in REFERENCE and k != "lidar_ld19"}
    highest = max(tops, key=tops.get)
    res["lidar"] = {"base_z": round(lidar_bb.min.Z, 2), "scan_plane_z": round(scan_z, 2),
                    "highest_other": highest, "highest_other_z": tops[highest],
                    "clearance_mm": round(scan_z - tops[highest], 2),
                    "height_above_ground_mm": round(scan_z + D.WHEEL_D / 2 - (D.BOTTOM_Z[1] + D.MOTOR[2] / 2), 1)}

    # 4. centre of mass (empty bay and full bay)
    tot, mom = 0.0, bd.Vector()
    for k, v in g.items():
        if k in REFERENCE:
            continue
        m = mass_of(k, v)
        tot += m
        mom += centroid(v) * m
    com = mom / tot
    bay_c = centroid(g["payload_bay_keepout"])
    bay_c = bd.Vector(bay_c.X, bay_c.Y, D.TOP_Z[1] + 25)   # water low in the bay (assumed 25 mm up)
    com_full = (mom + bay_c * PAYLOAD_FULL_G) / (tot + PAYLOAD_FULL_G)
    ground = D.BOTTOM_Z[1] + D.MOTOR[2] / 2 - D.WHEEL_D / 2
    f = lambda c: [round(c.X, 1), round(c.Y, 1), round(c.Z - ground, 1)]
    res["mass_g"] = round(tot)
    res["com_empty_xyz_above_ground"] = f(com)
    res["com_full_bay_xyz_above_ground"] = f(com_full)
    res["centre_of_rotation_y"] = D.CENTRE_OF_ROTATION[1]
    res["plate_masses_g"] = {k: round(mass_of(k, g[k])) for k in g if k.endswith("plate")}
    return res


if __name__ == "__main__":
    out = [check(scout_layout()), check(scout_layout_tall_battery())]
    text = json.dumps(out, indent=1)
    print(text)
    (HERE / "fit_check_results.json").write_text(text)
