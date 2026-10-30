"""TerraHawk Scout layout on the Elegoo Smart Robot Car V4.0 plates.

Plates are B-rep extrusions of the flat-face outlines measured from Elegoo's STLs
(holes included); every other part is a dimensioned envelope except the Pi 5,
which is the Raspberry Pi mechanical reference model from step.parts.
"""
from pathlib import Path

import build123d as bd
from cadgen import model, read_mesh, read_xbf

import scout_dims as D
from plate_outline import ELEGOO_DIR, face_polygon, rings

HERE = Path(__file__).resolve().parent


# ---------------------------------------------------------------- helpers
def to_robot(pts):
    cx, cy = D.STL_CENTRE
    return [(-(x - cx), -(y - cy)) for x, y in pts]


def plate_from_stl(name, z0, thickness, use_top_face, label, color):
    mesh = read_mesh(ELEGOO_DIR / name)
    zs = [v[2] for v in mesh.vertices]
    level = max(zs) if use_top_face else min(zs)
    outer, holes = rings(face_polygon(mesh.vertices, mesh.triangles, level))
    face = bd.Polygon(*to_robot(outer), align=None)
    for h in holes:
        face = face - bd.Polygon(*to_robot(h), align=None)
    body = bd.Pos(0, 0, z0) * bd.extrude(face, amount=thickness, dir=(0, 0, 1))   # ring winding is arbitrary
    body.label, body.color = label, color
    bb = body.bounding_box()
    assert abs(bb.min.Z - z0) < 1e-6 and abs(bb.size.Z - thickness) < 1e-6, (label, bb)
    return body


def block(size, x, y, z0, label, color):
    """Envelope box: centred on (x, y), bottom at z0."""
    sx, sy, sz = size
    b = bd.Pos(x, y, z0) * bd.Box(sx, sy, sz, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    b.label, b.color = label, color
    return b


# ---------------------------------------------------------------- sub-groups
def drivetrain():
    parts = []
    mx = D.SPINE_HALF - 1.0 - D.MOTOR[0] / 2              # motor 1 mm inboard of spine edge
    zc = D.BOTTOM_Z[1] + D.MOTOR[2] / 2                   # shaft height
    wx = D.SPINE_HALF + D.WHEEL_GAP + D.WHEEL_W / 2
    for name, ay, outward in (("front", D.AXLE_Y_FRONT, 1), ("rear", D.AXLE_Y_REAR, -1)):
        my = ay + outward * (D.MOTOR[1] / 2 - D.MOTOR_SHAFT_FROM_END)   # body runs toward the robot's end
        for side, s in (("left", -1), ("right", 1)):
            parts.append(block(D.MOTOR, s * mx, my, D.BOTTOM_Z[1], f"motor_{name}_{side}", "#e2c044"))
            w = bd.Pos(s * wx, ay, zc) * bd.Cylinder(D.WHEEL_D / 2, D.WHEEL_W, rotation=(0, 90, 0))
            w.label, w.color = f"wheel_{name}_{side}", "#2b2b2b"
            parts.append(w)
    return parts


def lower_deck(battery):
    z = D.BOTTOM_Z[1]
    cy = D.CENTRE_OF_ROTATION[1]
    return [
        block(battery, 0, cy, z, "battery_3s", "#3a7bd5"),
        block(D.ESP32, -12.0, 57.5, z + D.STANDOFF, "esp32_devkit", "#1f6f43"),
        block(D.TB6612, 15.5, 50.0, z + D.STANDOFF, "tb6612_driver", "#c0392b"),
        block(D.HCSR04, 0, 103.5, z, "hc_sr04", "#5dade2"),
        block(D.BUCK, 0, -65.0, z, "buck_5v5a", "#8e44ad"),
        # IMU hangs under the top plate over the centre of rotation
        block(D.IMU, 0, cy, D.TOP_Z[0] - D.IMU[2], "mpu6050_imu", "#16a085"),
    ]


def pi5():
    pi = read_xbf(HERE / "../vendor/raspberry_pi_5.xbf")       # Y-up vendor model
    z0 = D.TOP_Z[1] + D.PI_STANDOFF + 1.8                       # vendor min Y is -1.8
    placed = bd.Pos(0, D.PI_Y, z0) * bd.Rot(90, 0, 0) * pi
    placed.label = "raspberry_pi_5"
    cooler = block(D.COOLER, -8.0, D.PI_Y, z0 + 1.5, "pi5_active_cooler", "#9aa5b1")
    return bd.Compound(children=[placed, cooler], label="pi5_stack")


def upper_deck():
    top = D.TOP_Z[1]
    lidar_z = D.BAY[2] + top + D.SCAN_MARGIN + 0.5 - D.LIDAR_SCAN_ABOVE_BASE   # scan plane clears bay
    plat_z = lidar_z - D.PLATFORM[2]
    cy = D.CENTRE_OF_ROTATION[1]
    mast = block((D.MAST[0], D.MAST[1], plat_z - top), 0, D.MAST_Y, top, "lidar_mast", "#f39c12")
    platform = block(D.PLATFORM, 0, cy, plat_z, "lidar_platform", "#f39c12")
    lidar = bd.Pos(0, cy, lidar_z) * bd.Cylinder(19.0, D.LIDAR[2], align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    lidar = lidar + block((D.LIDAR[0], D.LIDAR[1], 12.0), 0, cy, lidar_z, "x", None)   # base + turret
    lidar.label, lidar.color = "lidar_ld19", "#222831"
    scan_z = lidar_z + D.LIDAR_SCAN_ABOVE_BASE
    ring = bd.Pos(0, cy, scan_z) * (bd.Cylinder(130, 0.2) - bd.Cylinder(37, 0.2))   # reaches past every plate corner (max r ~117)
    ring.label, ring.color = "lidar_scan_plane", bd.Color(1.0, 0.1, 0.1, 0.25)
    return [
        pi5(),
        block(D.CAMERA, 0, 104.0, D.DEAD_Z[1], "pi_camera_3", "#27ae60"),
        block(D.BAY, 0, -85.0, top, "payload_bay_keepout", bd.Color(0.2, 0.6, 1.0, 0.35)),
        mast, platform, lidar, ring,
    ]


def scout(battery, label):
    plates = [
        plate_from_stl("Bottom Plate.stl", D.BOTTOM_Z[0], D.PLATE_T, False, "bottom_plate", "#d9d9d9"),
        plate_from_stl("Top Plate.stl", D.TOP_Z[0], D.PLATE_T, True, "top_plate", "#bfbfbf"),
        plate_from_stl("Dead Plate.stl", D.DEAD_Z[0], D.DEAD_Z[1] - D.DEAD_Z[0], True, "camera_mount_dead_plate", "#a6a6a6"),
    ]
    kids = plates + drivetrain() + lower_deck(battery) + upper_deck()
    asm = bd.Compound(children=kids, label=label)
    bb = plates[0].bounding_box()
    assert abs(bb.size.X - D.PLATE_X) < 0.2 and abs(bb.size.Y - D.PLATE_Y) < 0.2, bb.size
    return asm


@model
def scout_layout():
    """Recommended: flat 3S1P 18650 pack (70 x 57 x 20)."""
    return scout(D.BATT_FLAT, "scout_layout")


@model
def scout_layout_tall_battery():
    """Brief's assumed 70 x 55 x 40 pack, to show whether it fits the 39.7 mm deck gap."""
    return scout(D.BATT_BOX, "scout_layout_tall_battery")


if __name__ == "__main__":
    scout_layout()
    scout_layout_tall_battery()
