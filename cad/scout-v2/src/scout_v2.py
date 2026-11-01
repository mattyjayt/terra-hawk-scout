"""TerraHawk Scout v2 (round 2): detailed assembly on the Elegoo Smart Robot Car V4.0 plates.

REP-103 frame (see dims.py): origin on the ground at the centre of rotation, +X forward,
+Y left, +Z up, mm. Every component is its own labelled part with a colour and a material
whose density gives the part its datasheet / assumed mass.

Round-2 layout (off-the-shelf parts only):
  * motors: 4 TT-form-factor variants (stock, Hiwonder, DFRobot FIT0450 vendor STEP, Yahboom) in the
    stock Elegoo brackets; encoder variants sit on M3 nylon spacers under the brackets so the encoder
    clears the bottom plate (the whole chassis drops by that amount, wheels stay on the ground).
  * lower deck: battery on the centre of rotation; buck_5v + buck_motor in the front bay; ESP32 in the rear
    bay (USB-C at the rear end); XT30 (left) and fuse holder (right) beside the battery.
  * under the top plate (bench pre-fit): Cytron MDD3A on M3 nylon standoffs above the battery, GY-521 IMU,
    power switch (front-left), HC-SR04 bracket under the front edge.
  * top deck: Pi 5 + Active Cooler; LiDAR tower = 4 stacked M3 brass standoffs on the TOP PLATE outside the
    Pi footprint + an 80 x 120 FR4 prototyping board; LiDAR on the board. One tower height per LiDAR,
    valid with and without the 300 g payload cup (keep-out Ø75 x 80 at the rear of the deck).
  * cables: approximate harness routes as thin swept tubes (labels cable_*), see cables.py.

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
DEFAULT = {"lidar": "ld19", "motor": "hiwonder", "payload": True, "driver": "mdd3a"}

COL = {
    "plate": "#2b2f36", "wheel": "#1c1c1c", "motor": "#f2c230", "motor_dark": "#202326", "encoder": "#1f7a3a",
    "bracket": "#3d3d3d", "battery": "#2f6fd6", "buck5": "#1d5fa8", "buckm": "#2a7fbf", "driver": "#b71c1c",
    "tb6612": "#7a1fa2", "esp32": "#202020", "imu": "#1565c0", "hcsr04": "#1e88e5", "brass": "#c8a24a",
    "nylon": "#f1efe6", "fr4": "#2e7d32", "cooler": "#a7adb5", "camera": "#2e7d32", "cam_mount": "#e0e0e0",
    "lidar": "#151515", "fuse": "#d32f2f", "switch": "#9e9e9e", "xt30": "#f9a825", "screw": "#8a8f96",
    "adapter": "#283593",
    "cable_18awg": "#e53935", "cable_20awg": "#fb8c00", "cable_22awg": "#8e24aa", "cable_signal": "#00acc1",
    "cable_ffc": "#fdd835", "cable_usb": "#424242",
}

# Assembly order for the guide (assembly/steps.json, assembly/step_NN.png) and the exploded view:
# (step id, title, label prefixes). A label ending in "_" is a prefix; others match exactly.
BUILD_STEPS = [
    ("01", "Bottom plate, motor spacers + brackets, motors, wheels",
     ["bottom_plate", "motor_spacer_", "motor_bracket_", "motor_fl", "motor_fr", "motor_rl", "motor_rr", "wheel_"]),
    ("02", "Lower-deck power: battery, XT30, fuse holder, 5 V buck, motor buck",
     ["battery", "xt30", "fuse_holder", "buck_5v", "buck_motor",
      "cable_batt_xt30", "cable_xt30_fuse", "cable_fuse_switch"]),
    ("03", "ESP32 in the rear bay + encoder harness",
     ["esp32", "cable_enc_"]),
    ("04", "Deck pillars", ["deck_pillar_"]),
    ("05", "Top plate pre-fit on the bench: MDD3A, IMU, power switch (underside)",
     ["mdd3a_standoff_", "mdd3a", "tb6612", "imu_standoff_", "imu", "switch"]),
    ("06", "Close the decks: top plate on the pillars, connect power + motor + signal wires",
     ["top_plate", "cable_switch_", "cable_buckm_driver", "cable_driver_motor_", "cable_driver_esp32",
      "cable_imu_esp32"]),
    ("07", "Pi 5 on M2.5 standoffs + Active Cooler, 5 V feed to GPIO",
     ["pi_screw_", "pi_standoff_", "pi5", "pi5_cooler", "cable_buck5_pi", "cable_esp32_pi"]),
    ("08", "Camera Module 3 on its holder + 22-pin FFC to the Pi",
     ["camera_mount", "camera", "cable_camera_ffc"]),
    ("09", "HC-SR04 ultrasonic on its bracket under the front edge",
     ["ultrasonic_bracket", "ultrasonic", "cable_us_esp32"]),
    ("10", "LiDAR tower: 4 M3 standoff legs on the top plate + FR4 board",
     ["lidar_leg_screw_", "lidar_leg_", "lidar_board", "lidar_board_screw_"]),
    ("11", "LiDAR + USB adapter on the board, USB to the Pi",
     ["lidar", "lidar_adapter", "cable_lidar_pi"]),
    ("12", "Payload cup (keep-out Ø75 x 80, <= 300 g)", ["payload"]),
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
def plate_from_stl(name, z0, use_top_face, label):
    mesh = read_mesh(ELEGOO_DIR / name)
    zs = [v[2] for v in mesh.vertices]
    outer, holes = rings(face_polygon(mesh.vertices, mesh.triangles, max(zs) if use_top_face else min(zs)))
    face = bd.Polygon(*[D.stl_to_v2(*p) for p in outer], align=None)
    for h in holes:
        face = face - bd.Polygon(*[D.stl_to_v2(*p) for p in h], align=None)
    body = bd.Pos(0, 0, z0) * bd.extrude(face, amount=D.PLATE_T, dir=(0, 0, 1))
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
    out.append(finish(padded(*D.BUCK_5V, 50.7, 0.0), "buck_5v", COL["buck5"], mass_g=D.BUCK_5V_MASS_G))
    out.append(finish(padded(*D.BUCK_MOTOR, 78.2, 0.0), "buck_motor", COL["buckm"], mass_g=D.BUCK_MOTOR_MASS_G))
    ex0, ex1 = D.ESP32_X
    out.append(finish(padded(ex1 - ex0, D.ESP32[1], D.ESP32[2], (ex0 + ex1) / 2, 0.0), "esp32", COL["esp32"],
                      mass_g=D.ESP32_MASS_G))
    out.append(finish(blk(25.0, 10.0, 5.2, 0.0, 36.0, z), "xt30", COL["xt30"], mass_g=D.MASS_MISC["xt30"]))
    out.append(finish(blk(*D.FUSE_HOLDER, 0.0, -37.5, z), "fuse_holder", COL["fuse"], mass_g=D.MASS_MISC["fuse_holder"]))
    return out


def under_top_plate(geo, driver):
    """Bench sub-assembly hung from the top plate's underside: motor driver, IMU, power switch."""
    zt = geo.T0
    out = []
    if driver == "mdd3a":
        L, W, t = D.MDD3A
        hx, hy = D.MDD3A_HOLES
        zb = zt - D.MDD3A_STANDOFF                    # PCB solder side (faces up)
        for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
            out.append(finish(zcyl(5.0, D.MDD3A_STANDOFF, D.MDD3A_X + sx * hx / 2, sy * hy / 2, zb),
                              f"mdd3a_standoff_{i + 1}", COL["nylon"], density=D.NYLON_DENSITY))
        pcb = blk(L, W, t, D.MDD3A_X, 0, zb - t)
        comp = blk(L - 6.0, W - 8.0, D.MDD3A_COMP_H, D.MDD3A_X, 0, zb - t - D.MDD3A_COMP_H)   # components hang down
        out.append(finish(pcb + comp, "mdd3a", COL["driver"], mass_g=D.MDD3A_MASS_G))
    else:
        out.append(finish(blk(*D.TB6612, D.MDD3A_X, 0, zt - D.TB6612[2]), "tb6612", COL["tb6612"], mass_g=D.TB6612_MASS_G))
    px, py, pt = D.IMU_PCB
    ix, iy = D.IMU_XY
    for i, sy in enumerate((1, -1)):
        out.append(finish(zcyl(5.0, D.IMU_STANDOFF, ix + 7.6, iy + sy * 5.6, zt - D.IMU_STANDOFF),
                          f"imu_standoff_{i + 1}", COL["nylon"], density=D.NYLON_DENSITY))
    pcb = blk(px, py, pt, ix, iy, zt - D.IMU_STANDOFF - pt)
    chip = blk(4.0, 4.0, 0.9, ix, iy, zt - D.IMU_STANDOFF - pt - 0.9)           # MPU-6050 QFN, facing down
    out.append(finish(pcb + chip, "imu", COL["imu"], mass_g=D.IMU_MASS_G))
    sx_, sy_, sz = D.SWITCH_BODY
    x, y = SWITCH_XY_FRONT
    sw = blk(sx_, sy_, sz, x, y, zt - sz) + zcyl(D.SWITCH_LEVER[0], D.SWITCH_LEVER[1], x, y, geo.T1)
    out.append(finish(sw, "switch", COL["switch"], mass_g=D.MASS_MISC["switch"]))
    return out


SWITCH_XY_FRONT = (105.0, 32.0)    # design: top plate front-left, next to the bucks (short 18 AWG runs)


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


def camera_parts(geo):
    zt = geo.T1
    bxl, byl, bt, uh = D.CAMERA_MOUNT
    mount = blk(bxl, byl, bt, CAM_X_UP + bxl / 2, 0, zt) + blk(3.0, byl, uh, CAM_X_UP + 1.5, 0, zt + bt)
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
    pt, pw, ph = D.HCSR04_PCB
    zc = zu - bt2 - ph / 2
    us = blk(pt, pw, ph, US_X + 2.0 + pt / 2, 0, zc - ph / 2)
    td, tl, tp = D.HCSR04_TRANSDUCER
    for sy in (1, -1):
        us = us + xcyl(td, US_X + 2.0 + pt, US_X + 2.0 + pt + tl, sy * tp / 2, zc)
    return [finish(br, "ultrasonic_bracket", COL["bracket"], density=1.05),
            finish(us, "ultrasonic", COL["hcsr04"], mass_g=D.HCSR04_MASS_G)]


def lidar_leg_xy():
    bx, by, _ = D.LIDAR_BOARD
    i = D.LIDAR_BOARD_HOLE_INSET
    return [(D.LIDAR_X + sx * (bx / 2 - i), sy * (by / 2 - i)) for sx in (1, -1) for sy in (1, -1)]


def tower_length(lidar_key):
    """Leg length (catalogue stack) so the scan plane is >= SCAN_MARGIN above the payload keep-out top,
    measured from the top-plate top face. Independent of the payload option and of the motor variant."""
    L = D.LIDARS[lidar_key]
    need = D.PAYLOAD_H + D.SCAN_MARGIN - L["scan_above_base"] - D.LIDAR_BOARD[2]
    a, b = stack_for(need)
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
    out.append(finish(blk(bx, by, bt, D.LIDAR_X, 0, zb), "lidar_board", COL["fr4"], density=D.LIDAR_BOARD_DENSITY))
    for i, (x, y) in enumerate(lidar_leg_xy()):
        out.append(finish(zcyl(5.5, 2.4, x, y, zb + bt), f"lidar_board_screw_{i + 1}", COL["screw"], density=7.9))
    return out, zb + bt, tl, stack


def lidar_parts(geo, lidar_key, base_z):
    L = D.LIDARS[lidar_key]
    lid = bd.Pos(D.LIDAR_X, 0, base_z) * lidar_body(lidar_key)
    ax_, ay_, az_ = L["adapter"]
    ad = blk(ax_, ay_, az_, D.LIDAR_X - 47.0, 0, base_z)
    return [finish(lid, "lidar", COL["lidar"], mass_g=L["mass_g"]),
            finish(ad, "lidar_adapter", COL["adapter"], mass_g=5.0)], base_z + L["scan_above_base"]


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


# ------------------------------------------------------------------ assembly
def scout_parts(lidar_key="ld19", motor_key="hiwonder", with_payload=True, driver="mdd3a"):
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
    if with_payload:
        kids.append(payload(geo))
    ctx = {"geo": geo, "motor": motor_key, "lidar": lidar_key, "driver": driver,
           "bb": {k.label: k.bounding_box() for k in kids}, "pi_vendor_to_world": lambda xv, zv, yv=0.0: pi_vendor_to_world(geo, xv, zv, yv),
           "pi_pcb_top": pi_pcb_top(geo), "switch_xy": SWITCH_XY_FRONT}
    kids += cable_parts(ctx)
    info = {"geo": geo, "scan_z": scan_z, "tower_len": tl, "tower_stack": stack, "lidar_base_z": base_z}
    return kids, info


def scout(lidar_key="ld19", motor_key="hiwonder", with_payload=True, driver="mdd3a", label="scout_v2"):
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
    return asm


def variant_name(lidar, motor, payload):
    return f"scout_v2__{lidar}_{motor}_{'pay' if payload else 'nopay'}"


VARIANTS = {}      # name -> (lidar, motor, payload, driver); the @model functions live in variants.py
for _l in ("ld19", "c1"):
    for _m in ("tt_stock", "hiwonder", "fit0450", "yahboom"):
        for _p in (True, False):
            VARIANTS[variant_name(_l, _m, _p)] = (_l, _m, _p, "mdd3a")
VARIANTS["scout_v2_tb6612"] = ("ld19", DEFAULT["motor"], True, "tb6612")


@model
def scout_v2():
    """Default: LD19 + Hiwonder encoder TT motors + Cytron MDD3A, 300 g payload keep-out at the rear."""
    return scout(DEFAULT["lidar"], DEFAULT["motor"], DEFAULT["payload"], DEFAULT["driver"], "scout_v2")


if __name__ == "__main__":
    scout_v2()
