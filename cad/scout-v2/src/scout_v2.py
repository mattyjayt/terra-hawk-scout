"""TerraHawk Scout v2 (round 3): detailed assembly on the Elegoo Smart Robot Car V4.0 plates.

REP-103 frame (see dims.py): origin on the ground at the centre of rotation, +X forward,
+Y left, +Z up, mm. Every component is its own labelled part with a colour and a material
whose density gives the part its datasheet / assumed mass.

Round-3 layout (physics review round 2 applied; off-the-shelf parts only, drilled Elegoo plates allowed):
  * motors: Hiwonder TT Hall-encoder 1:42 (default) in the stock Elegoo brackets on 3 mm M3 nylon spacers;
    the other TT variants stay available (variants.py).
  * lower deck: battery on the centre of rotation; Pololu D36V50F5 (+100 uF) in the front bay; ESP32 in the
    rear bay; XT30 (left) and the inline 7.5 A ATM fuse holder (right) beside the battery. NO motor buck.
  * under the top plate (bench pre-fit): Cytron MDD3A on M3 nylon standoffs (straight on the pack via the
    switch, 470 uF at VM), GY-521 IMU chip-down at x = -24 on nylon standoffs, KCD1-form-factor rocker through
    the plate (rear-centre), HC-SR04 bracket + camera holder sharing 2 screws at the front edge.
  * top deck: Pi 5 + Active Cooler; LiDAR tower = 4 M3 brass standoff legs on drilled holes + an 80 x 120 FR4
    board (x -5..115); SLAMTEC RPLIDAR C1 (default) on the board, wired to Pi UART0 (no USB anywhere).
  * payload keep-out Ø75 x 80 at x = -50 inside a fence of 4 M3 x 20 standoffs.
  * cables: approximate harness routes as thin swept tubes (labels cable_*), see cables.py.
  * DRILLS: every hole drilled in a plate/board, asserted to lie inside it (docs/cad-round3.md drilling table).

Models: scout_v2 (default) and one model per variant scout_v2__<lidar>_<motor>_<pay|nopay>, plus
scout_v2_tb6612 (TB6612 carrier instead of the MDD3A).
"""
from pathlib import Path

import build123d as bd
from cadgen import model, read_mesh, read_xbf

import dims as D
from cables import cable_routes
from plate_outline import ELEGOO_DIR, face_polygon, rings

HERE = Path(__file__).resolve().parent
DEFAULT = {"lidar": "c1", "motor": "hiwonder", "payload": True, "driver": "mdd3a"}   # r3: C1 (finding 14)

COL = {
    "plate": "#2b2f36", "wheel": "#1c1c1c", "motor": "#f2c230", "motor_dark": "#202326", "encoder": "#1f7a3a",
    "bracket": "#3d3d3d", "battery": "#2f6fd6", "buck5": "#1d5fa8", "buckm": "#2a7fbf", "driver": "#b71c1c",
    "tb6612": "#7a1fa2", "esp32": "#202020", "imu": "#1565c0", "hcsr04": "#1e88e5", "brass": "#c8a24a",
    "nylon": "#f1efe6", "fr4": "#2e7d32", "cooler": "#a7adb5", "camera": "#2e7d32", "cam_mount": "#e0e0e0",
    "lidar": "#151515", "fuse": "#d32f2f", "switch": "#9e9e9e", "xt30": "#f9a825", "screw": "#8a8f96",
    "adapter": "#283593", "cap": "#37474f", "cap_drv": "#263238",
    "cable_18awg": "#e53935", "cable_20awg": "#fb8c00", "cable_22awg": "#8e24aa", "cable_signal": "#00acc1",
    "cable_ffc": "#fdd835",
}

# Assembly order for the guide (assembly/steps.json, assembly/step_NN.png) and the exploded view:
# (step id, title, label prefixes). A label ending in "_" is a prefix; others match exactly.
BUILD_STEPS = [
    ("01", "Bottom plate: motor spacers + Elegoo brackets, Hiwonder encoder motors, wheels",
     ["bottom_plate", "motor_spacer_", "motor_bracket_", "motor_fl", "motor_fr", "motor_rl", "motor_rr", "wheel_"]),
    ("02", "Lower-deck power: battery, XT30, 7.5 A fuse holder, D36V50F5 5 V buck (+100 uF)",
     ["battery", "xt30", "fuse_holder", "buck_5v", "cap_buck", "cable_batt_xt30", "cable_xt30_fuse"]),
    ("03", "ESP32 in the rear bay + encoder and 5 V leads",
     ["esp32", "cable_enc_", "cable_5v_esp32"]),
    ("04", "Deck pillars (Elegoo M3 brass)", ["deck_pillar_"]),
    ("05", "Top plate pre-fit on the bench: drill, MDD3A + 470 uF, IMU (chip down), rocker switch, HC-SR04 + camera holder",
     ["top_plate", "mdd3a_standoff_", "mdd3a", "cap_driver", "tb6612", "imu_standoff_", "imu", "switch",
      "ultrasonic_bracket", "ultrasonic", "camera_mount", "front_screw_"]),
    ("06", "Close the decks: top plate on the pillars; fuse -> switch -> MDD3A / buck, motor and signal leads",
     ["cable_fuse_switch", "cable_switch_", "cable_driver_", "cable_imu_esp32", "cable_us_esp32"]),
    ("07", "Pi 5 on M2.5 standoffs + Active Cooler, 5 V to GPIO, ESP32 UART, Camera Module 3 + FFC",
     ["pi_screw_", "pi_standoff_", "pi5", "pi5_cooler", "camera", "cable_buck5_pi", "cable_esp32_pi", "cable_camera_ffc"]),
    ("08", "LiDAR tower: 4 M3 standoff legs + FR4 board, RPLIDAR C1, UART + 5 V leads",
     ["lidar_leg_screw_", "lidar_leg_", "lidar_board", "lidar_board_screw_", "lidar_screw_", "lidar",
      "cable_lidar_uart", "cable_buck5_lidar"]),
    ("09", "Payload fence: 4 M3 x 20 standoffs around the Ø75 x 80 cup keep-out (<= 300 g)",
     ["fence_post_", "fence_nut_", "payload"]),
]


# ------------------------------------------------------------------ helpers
def mat(name, mass_g, volume_mm3, color):
    from bd_materials import plastics
    dens = (mass_g / 1000.0) / (volume_mm3 * 1e-9)            # kg/m^3 so the part weighs mass_g
    return plastics.custom_plastic(name=name, density=dens, color=color)


def finish(part, label, color, mass_g=None, density=None):
    """Label, colour and a material: an explicit mass (datasheet/assumed) or a real density (g/cm3)."""
    part.label, part.color = label, color
    vol = sum(s.volume for s in part.solids())
    if mass_g is None:
        mass_g = vol * density / 1000.0
    m = mat(f"{label}_m", mass_g, vol, color)
    part.material = m
    for leaf in getattr(part, "leaves", []) or []:
        if leaf is not part:
            leaf.material = m
    return part


def blk(sx, sy, sz, x, y, z0):
    """Box centred on (x, y), bottom at z0."""
    return bd.Pos(x, y, z0) * bd.Box(sx, sy, sz, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def box_span(x0, x1, y0, y1, z0, z1):
    return blk(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), (x0 + x1) / 2, (y0 + y1) / 2, min(z0, z1))


def zcyl(d, h, x, y, z0):
    return bd.Pos(x, y, z0) * bd.Cylinder(d / 2, h, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def ycyl(d, y0, y1, x, z):
    lo, hi = min(y0, y1), max(y0, y1)
    return bd.Pos(x, lo, z) * bd.Rot(-90, 0, 0) * bd.Cylinder(d / 2, hi - lo, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def xcyl(d, x0, x1, y, z):
    lo, hi = min(x0, x1), max(x0, x1)
    return bd.Pos(lo, y, z) * bd.Rot(0, 90, 0) * bd.Cylinder(d / 2, hi - lo, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


class Geo:
    """Deck heights for one motor variant (encoder motors raise the motors -> the chassis drops)."""

    def __init__(self, motor_key):
        self.motor = motor_key
        self.raise_ = D.MOTORS[motor_key]["raise"]
        self.B1 = D.SHAFT_Z - D.SHAFT_ABOVE_PLATE - self.raise_      # bottom plate top face
        self.B0 = self.B1 - D.PLATE_T
        self.T0 = self.B1 + D.DECK_GAP                                # top plate underside
        self.T1 = self.T0 + D.PLATE_T


def stack_for(need):
    """Shortest pair of catalogue M3 standoffs (a >= b, b may be 0) whose sum is >= need."""
    best = None
    for a in D.M3_STANDOFF_LEN:
        for b in (0,) + D.M3_STANDOFF_LEN:
            if b > a or a + b < need - 1e-6:
                continue
            if best is None or a + b < sum(best) or (a + b == sum(best) and b < best[1]):
                best = (a, b)
    return best


# ------------------------------------------------------------------ chassis
PLATE_POLY = {}     # label -> shapely polygon of the plate in v2 XY (outline + existing holes), for the drill asserts


def plate_from_stl(name, z0, use_top_face, label):
    mesh = read_mesh(ELEGOO_DIR / name)
    zs = [v[2] for v in mesh.vertices]
    outer, holes = rings(face_polygon(mesh.vertices, mesh.triangles, max(zs) if use_top_face else min(zs)))
    face = bd.Polygon(*[D.stl_to_v2(*p) for p in outer], align=None)
    for h in holes:
        face = face - bd.Polygon(*[D.stl_to_v2(*p) for p in h], align=None)
    body = bd.Pos(0, 0, z0) * bd.extrude(face, amount=D.PLATE_T, dir=(0, 0, 1))
    from shapely.geometry import Polygon
    PLATE_POLY[label] = Polygon([D.stl_to_v2(*p) for p in outer], [[D.stl_to_v2(*p) for p in h] for h in holes])
    bb = body.bounding_box()
    assert abs(bb.size.X - D.PLATE_X) < 0.2 and abs(bb.size.Y - D.PLATE_Y) < 0.2, bb.size
    return finish(body, label, COL["plate"], density=D.PLATE_DENSITY)


def motor_local(key):
    """Motor in its local frame: u = +X (toward the can), v = +Y (wheel side, 0 at the gearbox wheel-side
    face), w = +Z (0 on the shaft axis). Returns a shape (Compound for the vendor STEP)."""
    m = D.MOTORS[key]
    if m.get("step"):
        raw = read_xbf(HERE / ".." / m["step"])
        sx, sz = m["vendor_shaft"]
        y_in, y_out = m["vendor_gear_y"]
        if m["pcb_side"] == "out":      # encoder PCB on the wheel side -> vendor +Y is +v, outer face = vendor Y 46.22
            plane = bd.Plane(origin=(-sz, -y_out, sx), x_dir=(0, 0, -1), z_dir=(1, 0, 0))
        else:                           # PCB inboard: vendor -Y is +v, outer face = vendor Y 27.32
            plane = bd.Plane(origin=(-sz, y_in, -sx), x_dir=(0, 0, 1), z_dir=(1, 0, 0))
        return bd.Location(plane) * raw
    t, h = m["gear_t"], m["gear_h"]
    g0 = -D.TT_SHAFT_FROM_END
    body = box_span(g0, g0 + D.TT_GEARBOX_L, -t, 0, -h / 2, h / 2)
    tl, tw, tt = D.TT_TAB
    body = body + box_span(g0 - tl, g0, -t / 2 - tt / 2, -t / 2 + tt / 2, -tw / 2, tw / 2)
    enc = m["enc"]
    can_end = m["body_end_u"] - (enc["len"] if enc else 0.0)
    cv, cw = m["can"]
    body = body + box_span(g0 + D.TT_GEARBOX_L, can_end, -t / 2 - cv / 2, -t / 2 + cv / 2, -cw / 2, cw / 2)
    so, si = m["shaft_len"]
    shaft = ycyl(D.TT_SHAFT_D, 0, so, 0, 0)
    if si > 0:
        shaft = shaft + ycyl(D.TT_SHAFT_D, -t - si, -t, 0, 0)
    parts = [body, shaft]
    if enc:
        e = box_span(can_end, m["body_end_u"], enc["v"][0], enc["v"][1], -enc["h"] / 2, enc["h"] / 2)
        if "conn" in enc:
            c = enc["conn"]
            e = e + box_span(c["u"][0], c["u"][1], c["v"][0], c["v"][1], -c["h"] / 2, c["h"] / 2)
        if "plug" in enc:
            vc = (enc["v"][0] + enc["v"][1]) / 2
            e = e + box_span(m["body_end_u"], m["body_end_u"] + enc["plug"], vc - 5, vc + 5, -3, 3)
        parts.append(e)
    return parts


def drivetrain(geo, motor_key):
    m = D.MOTORS[motor_key]
    out = []
    y_out = D.GEARBOX_Y_IN + m["gear_t"]                       # gearbox wheel-side face
    shaft_trim = (D.WHEEL_Y - D.WHEEL_W / 2) - y_out          # shaft drawn up to the wheel's inner face
    local = motor_local(motor_key)
    for fr, sx in (("f", 1), ("r", -1)):
        for lr, sy in (("l", 1), ("r", -1)):
            if isinstance(local, list):
                body, shaft, *enc = local
                shaft = ycyl(D.TT_SHAFT_D, 0, shaft_trim, 0, 0) if m["shaft"] == "single" else \
                    ycyl(D.TT_SHAFT_D, 0, shaft_trim, 0, 0) + ycyl(D.TT_SHAFT_D, -m["gear_t"] - m["shaft_len"][1], -m["gear_t"], 0, 0)
                pieces = [body, shaft] + enc
                shape = pieces[0]
                for p in pieces[1:]:
                    shape = shape + p
            else:
                shape = local
            if sx < 0:
                shape = shape.mirror(bd.Plane.YZ)
            if sy < 0:
                shape = shape.mirror(bd.Plane.XZ)
            placed = bd.Pos(sx * D.AXLE_X, sy * y_out, D.SHAFT_Z) * shape
            if not isinstance(local, list):            # vendor STEP: one compound part
                placed = bd.Compound(children=[c for c in placed.children] if placed.children else [placed])
            out.append(finish(placed, f"motor_{fr}{lr}", COL["motor"] if motor_key != "yahboom" else COL["motor_dark"],
                              mass_g=m["mass_g"]))
            w = ycyl(D.WHEEL_D, sy * (D.WHEEL_Y - D.WHEEL_W / 2), sy * (D.WHEEL_Y + D.WHEEL_W / 2), sx * D.AXLE_X, D.SHAFT_Z)
            out.append(finish(w, f"wheel_{fr}{lr}", COL["wheel"], mass_g=D.WHEEL_MASS_G))
            bx, by, bz = D.MOTOR_BRACKET
            xb = sx * sum(D.MOTOR_BRACKET_HOLES_X) / 2
            yb = sy * (D.BRACKET_Y_IN + by / 2)
            br = blk(bx, by, bz, xb, yb, geo.B1 + geo.raise_)
            out.append(finish(br, f"motor_bracket_{fr}{lr}", COL["bracket"], density=1.05))
            if geo.raise_ > 0:
                sp = None
                for hx in D.MOTOR_BRACKET_HOLES_X:
                    c = zcyl(D.MOTOR_RAISE_SPACER_D, geo.raise_, sx * hx, sy * D.MOTOR_BRACKET_HOLE_Y, geo.B1)
                    sp = c if sp is None else sp + c
                out.append(finish(sp, f"motor_spacer_{fr}{lr}", COL["nylon"], density=D.NYLON_DENSITY))
    return out


def deck_pillars(geo):
    h = geo.T0 - geo.B1
    return [finish(zcyl(D.PILLAR_D, h, x, y, geo.B1), f"deck_pillar_{i + 1}", COL["brass"], density=D.BRASS_DENSITY)
            for i, (x, y) in enumerate(D.DECK_PILLARS)]


# ------------------------------------------------------------------ lower deck
def lower_deck(geo):
    z = geo.B1
    zp = z + D.PAD_T                                  # boards stuck down with a foam pad
    bx, by, bz = D.BATTERY
    out = [finish(blk(bx, by, bz, *D.BATTERY_XY, z), "battery", COL["battery"], mass_g=D.BATTERY_MASS_G)]
    def padded(sx, sy, sz, x, y):          # board on a 3M VHB / foam pad (pad modelled as part of the board)
        return blk(sx, sy, sz, x, y, zp) + blk(sx - 2.0, sy - 2.0, D.PAD_T, x, y, z)
    bxy = D.BUCK_5V_XY
    out.append(finish(padded(*D.BUCK_5V, *bxy), "buck_5v", COL["buck5"], mass_g=D.BUCK_5V_MASS_G))
    cd, cl = D.CAP_BUCK          # 100 uF on the VIN pins, lying along X beside the buck (between it and the gearbox)
    yc = -(D.BUCK_5V[1] / 2 + (D.GEARBOX_Y_IN - D.BUCK_5V[1] / 2) / 2)
    xc = bxy[0] - 5.7                                   # clear of the battery (x <= 35) and the FR bracket (x >= 55.6)
    out.append(finish(xcyl(cd, xc - cl / 2, xc + cl / 2, yc, z + cd / 2),
                      "cap_buck", COL["cap"], mass_g=D.CAP_BUCK_MASS_G))
    ex0, ex1 = D.ESP32_X
    out.append(finish(padded(ex1 - ex0, D.ESP32[1], D.ESP32[2], (ex0 + ex1) / 2, 0.0), "esp32", COL["esp32"],
                      mass_g=D.ESP32_MASS_G))
    out.append(finish(blk(*D.XT30_PAIR, *D.XT30_XY, z), "xt30", COL["xt30"], mass_g=D.MASS_MISC["xt30"]))
    out.append(finish(blk(*D.FUSE_HOLDER, *D.FUSE_XY, z), "fuse_holder", COL["fuse"], mass_g=D.MASS_MISC["fuse_holder"]))
    return out


def mdd3a_holes():
    hx, hy = D.MDD3A_HOLES
    return [(D.MDD3A_X + sx * hx / 2, sy * hy / 2) for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1))]


def imu_holes():
    ix, iy = D.IMU_XY
    dx, dy = D.IMU_HOLES
    return [(ix + dx, iy + sy * dy) for sy in (1, -1)]


def front_screw_xy():
    """2 shared M3 screws: camera-holder base (top) + top plate + HC-SR04 bracket base (under the plate)."""
    return [(CAM_X_UP + 13.0, sy * D.CAMERA_MOUNT_HOLES_Y) for sy in (1, -1)]


def under_top_plate(geo, driver):
    """Bench sub-assembly fitted to the top plate: motor driver + 470 uF, IMU, rocker switch."""
    zt = geo.T0
    out = []
    if driver == "mdd3a":
        L, W, t = D.MDD3A
        zb = zt - D.MDD3A_STANDOFF                    # PCB solder side (faces up)
        for i, (x, y) in enumerate(mdd3a_holes()):
            out.append(finish(zcyl(5.0, D.MDD3A_STANDOFF, x, y, zb),
                              f"mdd3a_standoff_{i + 1}", COL["nylon"], density=D.NYLON_DENSITY))
        pcb = blk(L, W, t, D.MDD3A_X, 0, zb - t)
        comp = blk(L - 6.0, W - 8.0, D.MDD3A_COMP_H, D.MDD3A_X, 0, zb - t - D.MDD3A_COMP_H)   # components hang down
        out.append(finish(pcb + comp, "mdd3a", COL["driver"], mass_g=D.MDD3A_MASS_G))
        cd, cl = D.CAP_DRIVER                         # 470 uF on the VB terminal, lying along Y under the plate
        out.append(finish(ycyl(cd, -cl / 2, cl / 2, D.CAP_DRIVER_X, zt - 3.0 - cd / 2), "cap_driver", COL["cap_drv"],
                          mass_g=D.CAP_DRIVER_MASS_G))
    else:
        out.append(finish(blk(*D.TB6612, D.MDD3A_X, 0, zt - D.TB6612[2]), "tb6612", COL["tb6612"], mass_g=D.TB6612_MASS_G))
    px, py, pt = D.IMU_PCB
    ix, iy = D.IMU_XY
    for i, (x, y) in enumerate(imu_holes()):
        out.append(finish(zcyl(5.0, D.IMU_STANDOFF, x, y, zt - D.IMU_STANDOFF),
                          f"imu_standoff_{i + 1}", COL["nylon"], density=D.NYLON_DENSITY))
    pcb = blk(px, py, pt, ix, iy, zt - D.IMU_STANDOFF - pt)
    chip = blk(4.0, 4.0, 0.9, ix, iy, zt - D.IMU_STANDOFF - pt - 0.9)           # MPU-6050 QFN 4 x 4 x 0.9, facing down
    out.append(finish(pcb + chip, "imu", COL["imu"], mass_g=D.IMU_MASS_G))
    # KCD1-form-factor rocker through the plate: body below, bezel + rocker above
    x, y = D.SWITCH_XY
    bx_, by_, bz_ = D.SWITCH_BODY
    fx, fy, ft = D.SWITCH_FACE
    sw = blk(bx_, by_, bz_, x, y, zt - bz_) + blk(bx_ - 0.2, by_ - 0.2, D.PLATE_T, x, y, zt) \
        + blk(fx, fy, ft, x, y, geo.T1) + blk(fx - 4.0, fy - 5.0, D.SWITCH_ROCKER_H - ft, x, y, geo.T1 + ft)
    out.append(finish(sw, "switch", COL["switch"], mass_g=D.SWITCH_MASS_G))
    return out


# ------------------------------------------------------------------ upper deck
def pi_holes():
    hx, hy = D.PI_HOLES
    return [(D.PI_X + sx * hx / 2, sy * hy / 2) for sx in (1, -1) for sy in (1, -1)]


def pi_pcb_top(geo):
    return geo.T1 + D.PI_STANDOFF + (D.PI_PCB_VENDOR_Y[1] - D.PI_PCB_VENDOR_Y[0])


def pi_vendor_to_world(geo, xv, zv, yv=0.0):
    """Vendor STEP coords of the Pi -> world (same transform as pi_stack)."""
    z0 = geo.T1 + D.PI_STANDOFF - D.PI_PCB_VENDOR_Y[0]
    ox = D.PI_X + D.PI_HOLE_CENTRE_VENDOR_X
    return (ox - xv, zv, z0 + yv)


def pi_stack(geo):
    out = []
    for i, (x, y) in enumerate(pi_holes()):
        out.append(finish(zcyl(5.0, D.PI_STANDOFF, x, y, geo.T1),
                          f"pi_standoff_{i + 1}", COL["brass"], density=D.BRASS_DENSITY))
        out.append(finish(zcyl(4.5, 1.7, x, y, geo.T0 - 1.7), f"pi_screw_{i + 1}", COL["screw"], density=7.9))
    pi = read_xbf(HERE / "../vendor/raspberry_pi_5.xbf")
    z0 = geo.T1 + D.PI_STANDOFF - D.PI_PCB_VENDOR_Y[0]
    ox = D.PI_X + D.PI_HOLE_CENTRE_VENDOR_X
    placed = bd.Pos(ox, 0, z0) * bd.Rot(0, 0, 180) * bd.Rot(90, 0, 0) * pi
    vol = sum(s.volume for s in placed.solids())
    m = mat("pi5_m", D.PI_MASS_G, vol, "#1b5e20")
    for leaf in placed.leaves:
        leaf.material = m
    placed.label = "pi5"
    out.append(placed)
    cx, cy, ch = D.COOLER
    out.append(finish(blk(cx, cy, ch, D.PI_X, 0, pi_pcb_top(geo)), "pi5_cooler", COL["cooler"], mass_g=D.COOLER_MASS_G))
    return out


CAM_X_UP = 97.0      # design: camera holder upright X (round 1)
US_X = 101.3         # design: HC-SR04 bracket wall X (round 1)
US_DROP = 3.0        # design (r3): PCB top 3 mm below the bracket base (slotted bracket holes, assumed)


def camera_parts(geo):
    zt = geo.T1
    bxl, byl, bt, uh = D.CAMERA_MOUNT
    mount = blk(bxl, byl, bt, CAM_X_UP + bxl / 2, 0, zt) + blk(3.0, byl, uh, CAM_X_UP + 1.5, 0, zt + bt)
    for x, y in front_screw_xy():
        mount = mount - zcyl(3.2, bt, x, y, zt)
    t, w, h = D.CAMERA_PCB
    zb = zt + bt + 2.0                                            # PCB lower edge
    lx, ly, lz = D.CAMERA_LENS_BLOCK
    cam = blk(t, w, h, t / 2, 0, 0) + blk(lx, ly, lz, t + lx / 2, 0, D.CAMERA_LENS_FROM_BOTTOM - lz / 2)
    cam = bd.Pos(CAM_X_UP + 3.0, 0, zb) * bd.Rot(0, D.CAMERA_TILT_DEG, 0) * cam
    return [finish(mount, "camera_mount", COL["cam_mount"], density=D.ACRYLIC_DENSITY),
            finish(cam, "camera", COL["camera"], mass_g=D.CAMERA_MASS_G)]


def ultrasonic_parts(geo):
    bx, by, bt2, wh = D.HCSR04_BRACKET
    zu = geo.T0
    br = blk(bx, by, bt2, US_X + bx / 2, 0, zu - bt2) + blk(2.0, by, wh, US_X + 1.0, 0, zu - bt2 - wh)
    for x, y in front_screw_xy():
        br = br - zcyl(3.2, bt2, x, y, zu - bt2)
    pt, pw, ph = D.HCSR04_PCB
    zc = zu - bt2 - US_DROP - ph / 2          # sensor hangs US_DROP below the base so the shared nuts clear the cans
    us = blk(pt, pw, ph, US_X + 2.0 + pt / 2, 0, zc - ph / 2)
    td, tl, tp = D.HCSR04_TRANSDUCER
    for sy in (1, -1):
        us = us + xcyl(td, US_X + 2.0 + pt, US_X + 2.0 + pt + tl, sy * tp / 2, zc)
    out = [finish(br, "ultrasonic_bracket", COL["bracket"], density=1.05),
           finish(us, "ultrasonic", COL["hcsr04"], mass_g=D.HCSR04_MASS_G)]
    # shared M3 x 12 screws: head on the camera-holder base, nut under the HC-SR04 bracket base
    zc_top = geo.T1 + D.CAMERA_MOUNT[2]
    for i, (x, y) in enumerate(front_screw_xy()):
        sc = zcyl(5.5, 1.7, x, y, zc_top) + zcyl(2.9, zc_top - (zu - bt2 - 2.4), x, y, zu - bt2 - 2.4) \
            + zcyl(5.5, 2.4, x, y, zu - bt2 - 2.4)
        out.append(finish(sc, f"front_screw_{i + 1}", COL["screw"], density=7.9))
    return out


def lidar_leg_xy():
    return list(D.LIDAR_LEG_XY)


def cooler_top_above_t1():
    return D.PI_STANDOFF + (D.PI_PCB_VENDOR_Y[1] - D.PI_PCB_VENDOR_Y[0]) + D.COOLER[2]


LIDAR_COOLER_CLEAR = 3.0     # finding 20: the LiDAR board must clear the Pi cooler by >= 3 mm


def tower_length(lidar_key):
    """Leg length (catalogue stack, from the top-plate top face) = the shortest stack that puts the scan plane
    >= SCAN_MARGIN above the payload keep-out top AND keeps the board >= 3 mm above the Pi cooler (finding 20).
    The payload term applies to every variant (the cup is a mission payload), so one tower fits all."""
    L = D.LIDARS[lidar_key]
    need_scan = D.PAYLOAD_H + D.SCAN_MARGIN - L["scan_above_base"] - D.LIDAR_BOARD[2]
    need_cooler = cooler_top_above_t1() + LIDAR_COOLER_CLEAR
    a, b = stack_for(max(need_scan, need_cooler))
    return a + b, (a, b)


def lidar_body(lidar_key):
    """LiDAR in its own frame: rotation centre at (0, 0), base at z = 0, 'forward' = +X."""
    L = D.LIDARS[lidar_key]
    if lidar_key == "ld19":
        body_h = L["overall"][2] - L["turret_h"]
        b = blk(L["body"], L["body"], body_h, 0, 0, 0)
        for hx, hy in L["holes"]:                                   # thin mounting ears at the base
            ear = blk(abs(hx), 2 * L["ear_r"], 3.0, hx / 2, 0, 0) if hy == 0 else blk(2 * L["ear_r"], abs(hy), 3.0, hx, hy / 2, 0)
            b = b + zcyl(2 * L["ear_r"], 3.0, hx, hy, 0) + ear
        b = b + zcyl(L["turret_d"], L["turret_h"], 0, 0, body_h)
    else:
        b = blk(L["body"], L["body"], L["base_h"], 0, 0, 0) + zcyl(L["turret_d"], L["overall"][2] - L["base_h"], 0, 0, L["base_h"])
    return b


def lidar_tower(geo, lidar_key):
    tl, stack = tower_length(lidar_key)
    out = []
    for i, (x, y) in enumerate(lidar_leg_xy()):
        out.append(finish(zcyl(D.M3_STANDOFF_D, tl, x, y, geo.T1), f"lidar_leg_{i + 1}", COL["brass"], density=D.BRASS_DENSITY))
        out.append(finish(zcyl(5.5, 2.4, x, y, geo.T0 - 2.4), f"lidar_leg_screw_{i + 1}", COL["screw"], density=7.9))
    zb = geo.T1 + tl
    bx, by, bt = D.LIDAR_BOARD
    board = blk(bx, by, bt, D.LIDAR_X, 0, zb)
    for x, y in lidar_leg_xy():
        board = board - zcyl(3.2, bt, x, y, zb)
    out.append(finish(board, "lidar_board", COL["fr4"], density=D.LIDAR_BOARD_DENSITY))
    for i, (x, y) in enumerate(lidar_leg_xy()):
        out.append(finish(zcyl(5.5, 2.4, x, y, zb + bt), f"lidar_board_screw_{i + 1}", COL["screw"], density=7.9))
    return out, zb + bt, tl, stack


def lidar_parts(geo, lidar_key, base_z):
    """LiDAR on the board + its M2.5 screws from below. No USB adapter (finding 8)."""
    L = D.LIDARS[lidar_key]
    lid = bd.Pos(D.LIDAR_X, 0, base_z) * lidar_body(lidar_key)
    out = [finish(lid, "lidar", COL["lidar"], mass_g=L["mass_g"])]
    zb = base_z - D.LIDAR_BOARD[2]
    for i, (hx, hy) in enumerate(L["holes"]):
        out.append(finish(zcyl(4.5, 1.7, D.LIDAR_X + hx, hy, zb - 1.7), f"lidar_screw_{i + 1}", COL["screw"], density=7.9))
    return out, base_z + L["scan_above_base"]


def fence_xy():
    r = D.PAYLOAD_D / 2 + D.FENCE_PLAY + D.M3_STANDOFF_D / 2
    c = r / 2 ** 0.5
    return [(D.PAYLOAD_X + sx * c, sy * c) for sx in (1, -1) for sy in (1, -1)]


def payload_fence(geo):
    out = []
    for i, (x, y) in enumerate(fence_xy()):
        out.append(finish(zcyl(D.M3_STANDOFF_D, D.FENCE_H, x, y, geo.T1), f"fence_post_{i + 1}", COL["brass"],
                          density=D.BRASS_DENSITY))
        out.append(finish(zcyl(5.5, 2.4, x, y, geo.T0 - 2.4), f"fence_nut_{i + 1}", COL["screw"], density=7.9))
    return out


def payload(geo):
    p = zcyl(D.PAYLOAD_D, D.PAYLOAD_H, D.PAYLOAD_X, 0, geo.T1)
    return finish(p, "payload", bd.Color(0.15, 0.55, 1.0, 0.35), mass_g=D.PAYLOAD_MASS_G)


# ------------------------------------------------------------------ cables
def tube(points, d):
    shape = None
    for a, b in zip(points[:-1], points[1:]):
        va, vb = bd.Vector(*a), bd.Vector(*b)
        L = (vb - va).length
        if L < 1e-6:
            continue
        seg = bd.Solid.make_cylinder(d / 2, L, bd.Plane(origin=va, z_dir=(vb - va).normalized()))
        shape = seg if shape is None else shape + seg
    for p in points[1:-1]:
        shape = shape + bd.Pos(*p) * bd.Sphere(d / 2)
    return shape


def cable_parts(ctx):
    out = []
    for label, kind, pts in cable_routes(ctx):
        L = sum((bd.Vector(*b) - bd.Vector(*a)).length for a, b in zip(pts[:-1], pts[1:]))
        mass = D.CABLE_G_PER_M[kind] * L / 1000.0
        out.append(finish(tube(pts, D.CABLE_D[kind]), label, COL[f"cable_{kind}"], mass_g=max(mass, 0.05)))
    return out


# ------------------------------------------------------------------ drilling table
def drills(lidar_key="c1", driver="mdd3a"):
    """Every hole Jaime drills: (part, purpose, x, y, diameter or (w, h) cut-out). XY in REP-103 mm."""
    out = []
    for x, y in pi_holes():
        out.append(("top_plate", "Pi 5 M2.5 standoff", x, y, 2.7))
    if driver == "mdd3a":
        for x, y in mdd3a_holes():
            out.append(("top_plate", "MDD3A M3 nylon standoff", x, y, 3.2))
    for x, y in imu_holes():
        out.append(("top_plate", "IMU M3 nylon standoff", x, y, 3.2))
    for x, y in lidar_leg_xy():
        out.append(("top_plate", "LiDAR tower leg M3", x, y, 3.2))
    for x, y in fence_xy():
        out.append(("top_plate", "payload fence post M3", x, y, 3.2))
    for x, y in front_screw_xy():
        out.append(("top_plate", "camera holder + HC-SR04 bracket shared M3", x, y, 3.2))
    out.append(("top_plate", "KCD1 rocker cut-out 13 (X) x 19 (Y): 2 x Ø12 + file", *D.SWITCH_XY, D.SWITCH_CUTOUT))
    for x, y in lidar_leg_xy():
        out.append(("lidar_board", "leg M3 through the FR4 board", x, y, 3.2))
    for hx, hy in D.LIDARS[lidar_key]["holes"]:
        out.append(("lidar_board", f"{D.LIDARS[lidar_key]['name']} M2.5 fixing", D.LIDAR_X + hx, hy, 2.7))
    return out


def _hole(x, y, d):
    from shapely.geometry import Point, box
    if isinstance(d, tuple):
        return box(x - d[0] / 2, y - d[1] / 2, x + d[0] / 2, y + d[1] / 2)
    return Point(x, y).buffer(d / 2, 32)


def assert_drills(lidar_key, driver):
    """Each drilled hole lies inside its plate with >= 3 mm of material to the edge, to every existing hole
    and to the other drilled holes."""
    from shapely.geometry import box
    bx, by, _ = D.LIDAR_BOARD
    polys = dict(PLATE_POLY)
    polys["lidar_board"] = box(D.LIDAR_X - bx / 2, -by / 2, D.LIDAR_X + bx / 2, by / 2)
    rows = drills(lidar_key, driver)
    for part, why, x, y, d in rows:
        hole = _hole(x, y, d)
        poly = polys[part]
        assert poly.contains(hole), (part, why, x, y)
        web = min([poly.exterior.distance(hole)] + [r.distance(hole) for r in poly.interiors])
        assert web >= 3.0 - 1e-6, (part, why, x, y, round(web, 2))
    for i, a in enumerate(rows):
        for b in rows[i + 1:]:
            if a[0] == b[0]:
                assert _hole(*a[2:]).distance(_hole(*b[2:])) >= 3.0, (a, b)
    return rows


# ------------------------------------------------------------------ assembly
def scout_parts(lidar_key="c1", motor_key="hiwonder", with_payload=True, driver="mdd3a"):
    """All parts of one variant (list of labelled shapes) + facts the checks need."""
    geo = Geo(motor_key)
    kids = [plate_from_stl("Bottom Plate.stl", geo.B0, False, "bottom_plate"),
            plate_from_stl("Top Plate.stl", geo.T0, True, "top_plate")]
    kids += drivetrain(geo, motor_key) + deck_pillars(geo) + lower_deck(geo) + under_top_plate(geo, driver)
    kids += pi_stack(geo) + camera_parts(geo) + ultrasonic_parts(geo)
    tower, base_z, tl, stack = lidar_tower(geo, lidar_key)
    kids += tower
    lid, scan_z = lidar_parts(geo, lidar_key, base_z)
    kids += lid
    kids += payload_fence(geo)
    if with_payload:
        kids.append(payload(geo))
    ctx = {"geo": geo, "motor": motor_key, "lidar": lidar_key, "driver": driver,
           "bb": {k.label: k.bounding_box() for k in kids}, "pi_vendor_to_world": lambda xv, zv, yv=0.0: pi_vendor_to_world(geo, xv, zv, yv),
           "pi_pcb_top": pi_pcb_top(geo), "switch_xy": D.SWITCH_XY}
    kids += cable_parts(ctx)
    rows = assert_drills(lidar_key, driver)
    # drill the top plate (the guide shows every hole Jaime makes)
    top = kids[1]
    cut = None
    for part, _, x, y, d in rows:
        if part != "top_plate":
            continue
        tool = blk(d[0], d[1], D.PLATE_T, x, y, geo.T0) if isinstance(d, tuple) else zcyl(d, D.PLATE_T, x, y, geo.T0)
        cut = tool if cut is None else cut + tool
    kids[1] = finish(top - cut, "top_plate", COL["plate"], density=D.PLATE_DENSITY)
    info = {"geo": geo, "scan_z": scan_z, "tower_len": tl, "tower_stack": stack, "lidar_base_z": base_z,
            "drills": rows}
    return kids, info


def scout(lidar_key="c1", motor_key="hiwonder", with_payload=True, driver="mdd3a", label="scout_v2"):
    kids, info = scout_parts(lidar_key, motor_key, with_payload, driver)
    labels = [k.label for k in kids]
    assert len(labels) == len(set(labels)), "duplicate labels"
    asm = bd.Compound(children=kids, label=label)
    # in-body checks of the numbers the brief fixes
    others = [k for k in kids if k.label != "lidar" and not k.label.startswith("cable_")]
    top_other = max(k.bounding_box().max.Z for k in others)
    assert info["scan_z"] - top_other >= D.SCAN_MARGIN - 1e-6, (info["scan_z"], top_other)
    wheels = [k for k in kids if k.label.startswith("wheel_")]
    assert all(abs(w.bounding_box().min.Z) < 1e-6 for w in wheels), "wheels must touch the ground z=0"
    bb = {k.label: k.bounding_box() for k in kids}
    geo = info["geo"]
    # finding 20: LiDAR board >= 3 mm above the Pi cooler (measured on the built parts)
    assert bb["lidar_board"].min.Z - bb["pi5_cooler"].max.Z >= LIDAR_COOLER_CLEAR - 1e-6, "board vs cooler"
    # finding 18: IMU chip <= 25 mm from the centre of rotation (horizontal)
    ic = bb["imu"].center()
    assert (ic.X ** 2 + ic.Y ** 2) ** 0.5 <= 25.0 + 1e-6, ("imu", ic)
    # finding 10: payload keep-out centred at x = -50 on the top plate, Ø75 x 80 (Jaime's cup)
    if "payload" in bb:
        p = bb["payload"]
        assert abs(p.center().X - D.PAYLOAD_X) < 1e-6 and abs(p.size.X - D.PAYLOAD_D) < 1e-6 \
            and abs(p.size.Z - D.PAYLOAD_H) < 1e-6 and abs(p.min.Z - geo.T1) < 1e-6, "payload keep-out"
        assert bb["lidar_board"].min.X - p.max.X >= 5.0, "LiDAR board rear edge vs cup"
    # finding 13: fence posts leave 2 mm radial play to the cup
    for x, y in fence_xy():
        assert abs(((x - D.PAYLOAD_X) ** 2 + y ** 2) ** 0.5 - D.M3_STANDOFF_D / 2 - D.PAYLOAD_D / 2 - D.FENCE_PLAY) < 1e-6
    # Jaime's measured numbers: 39.7 mm between the decks, plates 4.5 thick, wheels Ø65, battery 70 x 50 x 15
    assert abs(bb["top_plate"].min.Z - bb["bottom_plate"].max.Z - D.DECK_GAP) < 1e-6, "deck gap"
    assert abs(bb["top_plate"].size.Z - D.PLATE_T) < 1e-6 and abs(bb["bottom_plate"].size.Z - D.PLATE_T) < 1e-6
    assert all(abs(w.bounding_box().size.Z - D.WHEEL_D) < 1e-3 for w in wheels), "wheel Ø65"
    bs = bb["battery"].size
    assert max(abs(bs.X - 70), abs(bs.Y - 50), abs(bs.Z - 15)) < 1e-6, "battery 70 x 50 x 15"
    # finding 6 / 8: no motor buck, no USB parts
    assert not any(l in bb for l in ("buck_motor", "lidar_adapter")) and not any("usb" in l for l in bb)
    return asm


def variant_name(lidar, motor, payload):
    return f"scout_v2__{lidar}_{motor}_{'pay' if payload else 'nopay'}"


VARIANTS = {}      # name -> (lidar, motor, payload, driver); the @model functions live in variants.py
for _l in ("ld19", "c1"):
    for _m in ("tt_stock", "hiwonder", "fit0450", "yahboom"):
        for _p in (True, False):
            VARIANTS[variant_name(_l, _m, _p)] = (_l, _m, _p, "mdd3a")
VARIANTS["scout_v2_tb6612"] = (DEFAULT["lidar"], DEFAULT["motor"], True, "tb6612")


@model
def scout_v2():
    """Default (r3): RPLIDAR C1 + Hiwonder encoder TT 1:42 + Cytron MDD3A on the pack, 300 g cup at x = -50."""
    return scout(DEFAULT["lidar"], DEFAULT["motor"], DEFAULT["payload"], DEFAULT["driver"], "scout_v2")


if __name__ == "__main__":
    scout_v2()
