"""Every dimension and mass of the Scout v2 CAD (round 2), with its source.

Frame (REP-103, BRIEF §3): origin = base_footprint on the ground at the centre of
rotation (midpoint of the front and rear axles, on the centreline); +X forward
(the Elegoo servo / "Dead Plate" end), +Y left, +Z up; millimetres, grams here.

Source tags used in comments:
  measured  - by Jaime, or from the Elegoo STLs (plate outlines/holes: src/plate_outline.py)
  datasheet - with URL (copies of drawings that were downloadable live in vendor/)
  assumed   - with the reason; these are the numbers to measure / weigh (docs/cad-round2.md lists them)
  design    - a layout choice made here (free to change, checked by checks/fit_check_v2.py)
Only plain numbers live here (no kernel objects), so importing is free.
"""

# ----------------------------------------------------------------- frame conversion
# v1 frame: +Y forward, +X right, Z = 0 at the bottom-plate underside.
STL_CENTRE = (9.77, -50.99)   # measured (v1): plate footprint centre in STL x, y
CR_Y_V1 = -2.0                # measured (v1): centre of rotation y in v1 frame (axles at +38 / -42)


def stl_to_v2(x, y):
    """Elegoo STL x, y -> v2 X, Y (v1's to_robot, then v1 -> v2)."""
    cx, cy = STL_CENTRE
    x1, y1 = -(x - cx), -(y - cy)        # v1 robot frame
    return (y1 - CR_Y_V1, -x1)


# ----------------------------------------------------------------- Elegoo chassis (measured)
PLATE_X, PLATE_Y, PLATE_T = 230.78, 132.52, 4.5   # measured, both plates (BRIEF §1)
DECK_GAP = 39.705             # measured: clear gap between the decks (BRIEF §1)
WHEEL_D = 65.0                # datasheet: Elegoo V4 wheel Ø65 (BRIEF §1)
WHEEL_W = 26.0                # assumed (v1): tyre width, measure
SHAFT_Z = WHEEL_D / 2         # 32.5: wheels on the ground
# TT gearbox is 22.4 tall with the output shaft at mid-height (datasheet Adafruit 3777 drawing,
# https://cdn-shop.adafruit.com/product-files/3777/3777_diagram.jpg). ASSUMED: the gearbox rests on the
# bottom plate in the Elegoo bracket, so the shaft is 11.2 above the plate top. MEASURE on the car.
SHAFT_ABOVE_PLATE = 11.2
AXLE_X = 40.0                 # measured (v1 wells: centres +38 / -42 in v1 -> v2 +-40)
SPINE_HALF = 47.0             # measured: plate half-width beside the wheel wells (STL outline y 46.6-47.5)
WHEEL_Y = 62.0                # (v1) wheel mid-plane: spine 47 + 2 gap + 13 -> track 124 (assumed gap)
# deck pillars: the 6 Ø3.4 holes common to both plate STLs (measured), M3 brass hex pillars (Elegoo kit, owned)
DECK_PILLARS = [(-80.7, -56.1), (-80.7, 55.9), (-0.7, 55.9), (0.3, -56.1), (84.4, -52.3), (84.4, 52.7)]
PILLAR_D = 5.5                # M3 hex pillar 5.5 mm AF (modelled as a Ø5.5 cylinder)
# Elegoo motor brackets: bottom-plate hole pairs at X +-57.7/65.5, Y +-25.5 (measured, STL holes)
MOTOR_BRACKET_HOLES_X = (57.7, 65.5)
MOTOR_BRACKET_HOLE_Y = 25.5
MOTOR_BRACKET = (12.0, 5.0, 22.0)   # assumed envelope of the stock motor holder wall (X, Y, Z) around the hole pair 7.8 apart; MEASURE
BRACKET_Y_IN = 22.2                 # assumed: bracket wall inner face (round 1), gearbox inner face at 27.2
GEARBOX_Y_IN = BRACKET_Y_IN + MOTOR_BRACKET[1]   # 27.2
# existing plate holes used by this design (measured from the STLs)
BOTTOM_HOLES_BATTERY = [(19.3, 18.2), (-17.3, 18.2), (19.3, -18.4), (-17.3, -18.4)]   # Ø3.5, zip-tie points
TOP_FRONT_SLOT = ((88.7, 1.3), (33.0, 12.5))     # top plate 33 x 12.5 slot: harness pass-through
BIG_HOLES = [(55.6, 0.4), (-57.6, 0.4)]          # 18 x 18 holes in both plates

# ----------------------------------------------------------------- TT-form-factor motors
# Motor-local frame used by src/scout_v2.py: u along the motor body (0 at the output shaft axis,
# + toward the can / rear end, which points to the plate end: front motors' cans point forward),
# v along the shaft axis (0 at the gearbox wheel-side face, + toward the wheel), w up (0 at the shaft).
# TT gearbox (all candidates share it): 37 long, shaft 9.25 from the output end, end tab 4.8
#   datasheet Adafruit 3777 drawing (above); Hiwonder drawing gives the same 13.81 mm shaft->tab-hole.
TT_GEARBOX_L = 37.0
TT_SHAFT_FROM_END = 9.25
TT_TAB = (4.8, 5.0, 2.8)        # datasheet (Adafruit): tab length beyond the end, width (w), thickness (v)
TT_SHAFT_D = 5.4                # datasheet: double-D shaft
MOTORS = {
    "tt_stock": {   # Elegoo/Adafruit TT 1:48, no encoder (owned)
        "name": "Stock TT 1:48 (Elegoo kit, no encoder)", "src": "https://www.adafruit.com/product/3777",
        "gear_t": 18.6, "gear_h": 22.4, "body_end_u": 65.2 - 9.25,       # datasheet 70 overall incl. 4.8 tab
        "can": (19.0, 22.4),                                              # datasheet: can 19.0 (v) x 22.4 (w) (22.4 over the clip ears, ignored)
        "enc": None, "shaft": "double", "shaft_len": (9.3, 8.7),         # datasheet: 36.6 over both shafts
        "mass_g": 30.6, "mass_src": "datasheet (Adafruit 3777: 30.6 g)",
        "raise": 0.0, "cable": "2 motor wires from the can end (soldered tabs)",
        "ratio": 48, "ppr_motor": 0, "counts_per_wheel_rev_x4": 0, "rpm_noload": 200, "v_rated": 6.0,
    },
    "hiwonder": {   # Hiwonder TT Gear Motor Hall Encoder, plastic shaft, 1:42 (SKU 21100132)
        "name": "Hiwonder TT Hall-encoder motor 1:42 (plastic shaft)",
        "src": "https://www.hiwonder.com/products/tt-motor-plastic",
        "drawing": "vendor/hiwonder_tt_encoder_dimension.jpg",
        "gear_t": 18.75, "gear_h": 22.4, "body_end_u": 75.15 - 9.25,      # datasheet drawing: 75.15 body, 79.95 with tab
        "can": (19.4, 22.4),                                              # datasheet drawing: can region 19.4 (v, scaled) x 22.4
        # encoder cap at the rear: Ø25.2 (w) x 6.9 long, spans v from -17.45 (1.3 inside the inboard face)
        # to +1.0; connector block 12.6 long reaching v = +8.3 on the shaft side (drawing: 25.79 overall)
        "enc": {"len": 6.9, "h": 25.2, "v": (-17.45, 1.0),
                "conn": {"u": (53.3, 65.9), "v": (1.0, 8.3), "h": 10.0}},  # conn height 10 assumed
        "shaft": "single", "shaft_len": (9.3, 0.0),
        "mass_g": 34.0, "mass_src": "datasheet (Hiwonder page: 34 g plastic shaft)",
        "raise": 3.0, "cable": "6-pin connector on the encoder cap, facing outboard (wheel side) at the can end",
        "ratio": 42, "ppr_motor": 6, "counts_per_wheel_rev_x4": 1008, "rpm_noload": 150, "v_rated": 7.4,
    },
    "fit0450": {    # DFRobot FIT0450, vendor STEP (vendor/fit0450/FIT0450.STEP, from the DFRobot wiki)
        "name": "DFRobot FIT0450 TT motor + encoder 1:120",
        "src": "https://wiki.dfrobot.com/fit0450/",
        "step": "vendor/fit0450/fit0450.xbf",
        # vendor frame (measured in the STEP): length along +Z, shaft axis along Y at X=24.98, Z=39.6;
        # gearbox Y 27.32..46.22; encoder PCB + bracket Y 29.42..63.01, X 10.03..39.89 (29.86 tall)
        "vendor_shaft": (24.98, 39.6), "vendor_gear_y": (27.32, 46.22),
        "gear_t": 18.9, "gear_h": 22.6, "body_end_u": 98.88 - 39.6,
        "enc": "step", "shaft": "double", "pcb_side": "out",
        "mass_g": 50.0, "mass_src": "datasheet (DFRobot wiki: 50 g)",
        "raise": 5.0, "cable": "PH2.0-6P on the vertical encoder PCB at the can end, 16.8 mm beyond the gearbox face",
        "ratio": 120, "ppr_motor": 8, "counts_per_wheel_rev_x4": 3840, "rpm_noload": 160, "v_rated": 6.0,
    },
    "yahboom": {    # Yahboom 13-line metal single-shaft TT encoder motor 1:45
        "name": "Yahboom TT encoder motor 1:45 (13-line, metal single shaft)",
        "src": "https://www.yahboom.store/products/encoder-tt-motor",
        "gear_t": 18.75, "gear_h": 22.4, "body_end_u": 75.0 - 4.8 - 9.25,  # datasheet 75 overall (photo), tab 4.8 assumed
        "can": (19.4, 22.4),                                              # assumed = Hiwonder
        # ASSUMED from product photos (no drawing published): encoder housing Ø25 (w) x 25 (v, centred on the
        # gearbox) x 14 long at the rear, PH2.0-6P plug protruding 6 mm from the rear face
        "enc": {"len": 14.0, "h": 25.0, "v": (-21.9, 3.1), "plug": 6.0},
        "shaft": "single", "shaft_len": (9.3, 0.0),
        "mass_g": 32.0, "mass_src": "vendor answer on the store page ('1PCS weight is 32g')",
        "raise": 3.0, "cable": "PH2.0-6P plug at the rear face (along the motor axis, toward the plate end)",
        "ratio": 45, "ppr_motor": 13, "counts_per_wheel_rev_x4": 2340, "rpm_noload": 355, "v_rated": 7.4,
    },
}
# raise = M3 nylon spacer stack under each Elegoo motor bracket so the encoder clears the bottom plate
# (>= 1 mm): catalogue thicknesses 3 / 5 mm (e.g. Keystone/Essentra M3 nylon round spacers).
MOTOR_RAISE_SPACER_D = 6.0      # datasheet-ish: M3 round nylon spacer OD 6 mm
ENCODER_PLATE_CLEARANCE = 1.0   # design target: encoder underside >= 1 mm above the bottom plate

# ----------------------------------------------------------------- battery (measured by Jaime)
BATTERY = (70.0, 50.0, 15.0)   # measured (Jaime): X along robot, Y, Z
# Label 3S 3000 mAh 11.1 V = 33.3 Wh in 52.5 cm3 -> 634 Wh/L (optimistic for a wrapped pack: sanity flag).
BATTERY_MASS_G = 110.0         # assumed: physics engineer's estimate 52.5 cm3 x 2.1 g/cm3 (range 100-126); WEIGH IT
BATTERY_XY = (0.0, 0.0)        # design: centred on the centre of rotation

# ----------------------------------------------------------------- power electronics
BUCK_5V = (25.4, 25.4, 9.5)    # datasheet Pololu D36V50F5 1" x 1" x 0.375", 7.0 g, 5 V 5.5 A
BUCK_5V_URL = "https://www.pololu.com/product/4091/specs"
BUCK_5V_MASS_G = 7.0
BUCK_MOTOR = (20.3, 17.8, 8.8) # datasheet Pololu D36V28Fx 0.8" x 0.7" x 0.345" (e.g. D36V28F9 = 9 V)
BUCK_MOTOR_URL = "https://www.pololu.com/product/3785/specs"
BUCK_MOTOR_MASS_G = 3.0        # assumed (Pololu lists ~2-3 g for this size; weigh)
PAD_T = 1.1                    # datasheet-ish: 3M VHB 4910-class foam pad thickness under boards stuck to a plate
MDD3A = (48.26, 41.66, 1.6)    # datasheet Cytron MDD3A board 48.26 x 41.66, 4 x Ø3 holes 43.18 x 36.58
MDD3A_HOLES = (43.18, 36.58)   # https://cdn.robotshop.com/media/c/cyt/rb-cyt-260/pdf/cytron-3a-4-16v-dual-channel-dc-motor-driver-datasheet.pdf
MDD3A_COMP_H = 11.0            # assumed: tallest part (5.08 mm terminal blocks / push buttons) above the PCB; MEASURE
MDD3A_PIN_H = 1.5              # assumed: solder-side protrusion
MDD3A_MASS_G = 14.5            # datasheet-ish (distributor listing anodas.lt: 14.5 g)
MDD3A_URL = "https://www.cytron.io/p-3amp-4v-16v-dc-motor-driver-2-channels"
MDD3A_STANDOFF = 5.0           # design: M3 x 5 nylon F-F standoffs, board hung under the top plate
MDD3A_X = -13.0                # design: board centre X (clear of the Pi standoff screws at X 16 and the IMU)
TB6612 = (15.2, 20.3, 4.0)     # datasheet Pololu #713 carrier 0.6" x 0.8" (option, replaces MDD3A)
TB6612_MASS_G = 2.0            # assumed
FUSE_HOLDER = (35.0, 13.0, 10.0)   # assumed: inline mini-blade (ATM) fuse holder body; MEASURE the one bought
SWITCH_BODY = (13.0, 8.0, 10.0)    # assumed: MTS-102 class mini toggle body under the plate (Ø6 bushing hole drilled)
SWITCH_LEVER = (3.0, 10.0)         # assumed: lever Ø, length above the plate
SWITCH_XY = (-92.0, 42.0)          # design: top plate rear-left, clear of the payload keep-out
XT30_PAIR = (25.0, 10.0, 5.2)      # datasheet-ish AMASS XT30: 10 x 5.2 section, mated pair ~25 long (along X)

# ----------------------------------------------------------------- controllers and sensors
ESP32 = (55.0, 28.0, 13.0)     # datasheet-ish (v1): ESP32 DevKit WROOM-32 USB-C with pin headers, X long axis
ESP32_MASS_G = 10.0            # assumed
ESP32_X = (-94.0, -39.0)       # design: rear bay, USB-C facing the rear end (flashing access)
IMU_PCB = (21.2, 16.4, 1.6)    # datasheet-ish: GY-521 21 x 16 mm (v1), 1.6 PCB
IMU_STANDOFF = 10.0            # design: M3 x 10 nylon standoffs hanging the IMU under the top plate
IMU_XY = (25.0, 0.0)           # design: under the top plate, 25 mm ahead of the centre of rotation
IMU_MASS_G = 2.0               # assumed
HCSR04_PCB = (1.6, 45.0, 20.0) # datasheet 45 x 20 x 15, https://www.mouser.com/datasheet/2/813/HCSR04-1022824.pdf
HCSR04_TRANSDUCER = (16.0, 12.0, 26.0)   # Ø16, protrudes 12 from the PCB, centres 26 apart (datasheet-ish)
HCSR04_MASS_G = 8.5            # assumed
HCSR04_BRACKET = (15.0, 47.0, 2.0, 22.0) # assumed generic HC-SR04 bracket envelope: base X, Y, t, wall height
CAMERA_PCB = (1.12, 25.0, 23.862)        # datasheet CM3 mechanical drawing (thickness, width, height)
                                         # https://datasheets.raspberrypi.com/camera/camera-module-3-standard-mechanical-drawing.pdf
CAMERA_HOLES = (21.0, 12.5, 2.2)         # datasheet: hole spacing Y, Z, Ø (M2)
CAMERA_LENS_FROM_BOTTOM = 14.4           # datasheet drawing (lens centre above the PCB lower edge)
CAMERA_LENS_BLOCK = (5.86, 8.5, 8.5)     # datasheet: 6.98 total - 1.12 PCB protrusion; 8.5 square assumed
CAMERA_FOV = (66.0, 41.0)                # datasheet https://www.raspberrypi.com/documentation/accessories/camera.html
CAMERA_TILT_DEG = 15.0                   # design choice: pitched down so the floor is seen close ahead
CAMERA_MASS_G = 4.0                      # assumed
CAMERA_MOUNT = (20.0, 30.0, 3.0, 26.0)   # assumed envelope: generic acrylic Pi-camera holder, base X, Y, t, upright height

# ----------------------------------------------------------------- Pi 5 (vendor STEP) + cooler
PI_HOLES = (58.0, 49.0)        # datasheet (Pi 5 mechanical drawing)
PI_HOLE_CENTRE_VENDOR_X = -10.0   # measured from vendor STEP
PI_PCB_VENDOR_Y = (0.1, 1.5)   # measured from vendor STEP (PCB faces)
PI_STANDOFF = 6.0              # design: M2.5 x 6 brass M-F standoffs under the Pi
PI_X = 45.0                    # design: Pi hole-pattern centre X (USB/Ethernet face the rear)
PI_MASS_G = 46.0               # datasheet-ish (Pi 5 board, retailer listings ~46 g)
# Pi 5 22-pin CAM/DISP FPC connectors, measured in the vendor STEP (vendor X 4.8..13.9, Z 11.8..27.3)
PI_CAM_VENDOR = ((4.8, 13.9), (11.8, 27.3))
PI_GPIO_PIN1_VENDOR_X = -32.4  # assumed from the Pi pinout (pin 1 at the end away from USB); header Z -27..-22
COOLER = (63.5, 42.5, 10.0)    # datasheet footprint 63.5 x 42.5, overall 13.7 incl. push-pins
                               # https://pip-assets.raspberrypi.com/categories/993-raspberry-pi-active-cooler/documents/RP-008187-DS-1-raspberry-pi-active-cooler-mechanical-drawing.pdf
COOLER_MASS_G = 25.0           # assumed

# ----------------------------------------------------------------- LiDAR + off-the-shelf tower (round 2)
# Tower = 4 legs of M3 brass M-F hex standoffs (2 stacked) standing on the TOP PLATE (4 drilled Ø3.2 holes)
# outside the Pi footprint, carrying an 80 x 120 x 1.6 FR4 prototyping board (Electrokit ProtoPCB EKB12,
# 4 x Ø3 fixing holes; https://www.electrokit.com/en/experimentkort-1-hal-80x120mm-genomplaterade-hal).
# The tower no longer touches the Pi, and its height does not depend on the payload option.
LIDAR_BOARD = (120.0, 80.0, 1.6)   # datasheet (Electrokit): 80 x 120 x 1.6 FR4, long side along X
LIDAR_BOARD_HOLE_INSET = 3.5       # ASSUMED: fixing-hole centres 3.5 mm in from each edge -> 113 x 73; MEASURE
LIDAR_BOARD_DENSITY = 1.85         # datasheet-ish FR4 1.85 g/cm3
LIDAR_X = 35.0                     # design: LiDAR rotation centre / board centre X (board spans -25..95)
M3_STANDOFF_LEN = (5, 6, 8, 10, 12, 15, 20, 25, 30, 35, 40)   # catalogue M3 M-F brass hex (e.g. RS PRO 222-452 family)
M3_STANDOFF_D = 5.5            # M3 hex standoff 5.5 AF (modelled Ø5.5)
SCAN_MARGIN = 5.0              # BRIEF: scan plane >= 5 mm above every part
LIDARS = {
    "ld19": {   # datasheet https://raw.githubusercontent.com/LetsOKdo/ld19/master/datasheet/LDROBOT_LD19_Datasheet_V1.0.pdf (p.6-7)
        "name": "LDROBOT LD19", "overall": (54.0, 46.29, 34.8), "body": 38.59, "turret_d": 35.29,
        "turret_h": 12.6, "scan_above_base": 34.8 - 7.8, "mass_g": 47.0,
        "holes": [(23.7, 0.0), (-8.2, 23.4), (-8.2, -23.4)], "ear_r": 4.4,
        "screw": "M2.5 x 8 pan head + M2.5 nut (3x), through Ø2.7 holes drilled in the proto board",
        "adapter": (26.0, 18.0, 6.0),     # assumed: CP2102 USB adapter board from the kit, MEASURE
    },
    "c1": {     # datasheet https://engineering.purdue.edu/477grp3/Files/refs/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf (fig 4-1)
        "name": "SLAMTEC RPLIDAR C1", "overall": (55.6, 55.6, 41.3), "body": 55.6, "turret_d": 50.0,
        "turret_h": 18.2, "scan_above_base": 29.8, "mass_g": 110.0,
        "holes": [(21.5, 21.5), (21.5, -21.5), (-21.5, 21.5), (-21.5, -21.5)],  # 4 x M2.5 x 4 deep, 43 x 43
        "base_h": 23.1,
        "screw": "M2.5 x 5 pan head (4x) from below the proto board into the C1's threaded holes (max 4 deep)",
        "adapter": (30.0, 22.0, 6.0),     # assumed: SLAMTEC USB adapter board, MEASURE
    },
}
# turret_d of the C1 (50) is assumed from the drawing proportions.

# ----------------------------------------------------------------- payload keep-out
PAYLOAD_D, PAYLOAD_H = 75.0, 80.0   # BRIEF: seedling cup ~Ø75 x 80
PAYLOAD_MASS_G = 300.0              # BRIEF: <= 300 g
PAYLOAD_X = -113.4 + 2.0 + PAYLOAD_D / 2   # rear of the top deck, 2 mm inside the plate end (-73.9)

# ----------------------------------------------------------------- cables (approximate harness, src/cables.py)
CABLE_G_PER_M = {"18awg": 12.0, "20awg": 8.0, "22awg": 5.5, "signal": 4.0, "ffc": 3.0, "usb": 20.0}   # assumed (silicone wire tables)
CABLE_D = {"18awg": 2.8, "20awg": 2.4, "22awg": 2.0, "signal": 3.0, "ffc": 1.0, "usb": 4.0}           # assumed outer Ø of the bundle

# ----------------------------------------------------------------- materials (densities, g/cm3)
PLATE_DENSITY = 1.19       # assumed (v1): acrylic plate; weigh a plate to confirm
BRASS_DENSITY = 8.5        # brass (standoffs simplified as solid cylinders -> slight overestimate)
NYLON_DENSITY = 1.15
ACRYLIC_DENSITY = 1.19
WHEEL_MASS_G = 25.0        # assumed (v1): Elegoo 65 mm wheel incl. tyre; WEIGH
MASS_MISC = {"fuse_holder": 5.0, "switch": 6.0, "xt30": 3.0}   # assumed
