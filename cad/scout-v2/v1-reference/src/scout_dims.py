"""Every dimension the Scout layout uses, with its source.

Robot frame: origin at the plate footprint centre, +Y forward (the Elegoo servo /
'Dead Plate' end), +X to the robot's right, Z = 0 at the underside of the bottom
plate. The Elegoo STLs share one assembly frame; the robot frame is that frame
rotated 180 deg about Z and shifted (see STL_TO_ROBOT).
"""

# --- Elegoo plates: MEASURED from the STLs ---------------------------------
STL_CENTRE = (9.77, -50.99)        # plate footprint centre in STL x, y (measured)
STL_Z0 = -57.905                   # STL z of the bottom plate underside (measured)
PLATE_X, PLATE_Y, PLATE_T = 132.52, 230.78, 4.5   # measured bounds, both plates
BOTTOM_Z = (0.0, 4.5)              # measured: STL z -57.91 .. -53.41
TOP_Z = (44.205, 48.705)           # measured: STL z -13.70 .. -9.20
DECK_GAP = TOP_Z[0] - BOTTOM_Z[1]  # 39.705 mm clear between plates (measured)
DEAD_Z = (48.885, 51.485)          # measured: STL z -9.02 .. -6.42 (servo cover on top plate)
SPINE_HALF = 47.0                  # measured: plate half width beside the wheel wells
FRONT_WELL_Y = (2.0, 74.0)         # measured wheel-well extents along Y
REAR_WELL_Y = (-78.0, -6.0)

# --- drive: TT motors and wheels (brief: datasheet-ish / assumed) -------------
MOTOR = (19.0, 70.0, 22.0)         # X (shaft dir), Y (length), Z  - brief 70 x 22 x 19
MOTOR_SHAFT_FROM_END = 11.0        # assumed: TT output shaft 11 mm from gearbox end
WHEEL_D, WHEEL_W = 65.0, 26.0      # 65 brief/datasheet, width 26 assumed
WHEEL_GAP = 2.0                    # assumed wheel inner face to spine edge
AXLE_Y_FRONT = sum(FRONT_WELL_Y) / 2   # 38: wheel centred in its measured well
AXLE_Y_REAR = sum(REAR_WELL_Y) / 2     # -42

# --- lower deck ---------------------------------------------------------------
BATT_FLAT = (70.0, 57.0, 20.0)     # assumed 3S1P 18650 flat pack (recommended)
BATT_BOX = (70.0, 55.0, 40.0)      # brief's assumed 3S 3000 mAh pack envelope
ESP32 = (28.0, 55.0, 13.0)         # datasheet-ish DevKit V1 (brief 55 x 28 x 13)
TB6612 = (21.0, 20.0, 10.0)        # assumed bare TB6612FNG breakout with headers
BUCK = (35.0, 60.0, 20.0)          # assumed 5 V / 5 A buck (brief)
HCSR04 = (45.0, 15.0, 20.0)        # datasheet 45 x 20 x 15 (X, Y depth, Z height)
IMU = (21.0, 16.0, 4.0)            # datasheet GY-521 21 x 16, 4 mm height assumed
STANDOFF = 5.0                     # assumed PCB standoff height on the lower deck

# --- upper deck ---------------------------------------------------------------
PI_STANDOFF = 6.0                  # assumed M2.5 standoffs under the Pi 5
PI_Y = -24.0                       # Pi 5 centre, behind the mast
COOLER = (48.0, 40.0, 12.0)        # assumed Active Cooler envelope above PCB top
CAMERA = (25.0, 12.0, 24.0)        # datasheet Camera Module 3: 25 x 24 x 11.5(+) -> X, Y depth, Z
BAY = (80.0, 60.0, 60.0)           # brief: payload keep-out X, Y, Z
LIDAR = (54.0, 46.0, 35.0)         # LDROBOT LD19 datasheet-ish 54 x 46.3 x 34.8
LIDAR_SCAN_ABOVE_BASE = 24.0       # assumed: optical window centre above LD19 base
SCAN_MARGIN = 5.0                  # required clearance of scan plane over every part
MAST = (12.0, 12.0)                # assumed printed mast post cross-section
MAST_Y = 14.0
PLATFORM = (60.0, 60.0, 3.0)       # assumed printed LiDAR platform

CENTRE_OF_ROTATION = (0.0, (AXLE_Y_FRONT + AXLE_Y_REAR) / 2)   # (0, -2)
