"""Approximate harness routes (round 3 cable plan).

Each route is (label, kind, [waypoints in mm, REP-103]). The model sweeps a thin tube through the
waypoints (labels cable_*); checks/fit_check_v2.py reports each route's length (+15 % slack) and the
chosen off-the-shelf cable. The routes are drawn from the parts' own bounding boxes so they follow
every variant. They are a plan (which lane, which pass-through, how long), not a wire-exact model:
cable tubes are excluded from the interference check.

Round 3 power chain (physics findings 6, 17): battery -> XT30 -> 7.5 A ATM inline fuse -> KCD1-class rocker
(top plate, rear-centre) -> split at the switch output -> MDD3A VB (470 uF at the terminal) and D36V50F5 VIN.
No motor buck. No USB cables (finding 8): the C1 and the ESP32 use Pi UARTs.

kind: 18awg (battery/power), 20awg (5 V to the Pi, MDD3A VB), 22awg (motor leads, 5 V branches), signal
(Dupont/JST jumpers), ffc (Pi 5 camera cable).
"""
import dims as D


def _c(bb):
    return ((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2)


def motor_connector(ctx, sx, sy):
    """Where each motor's cable leaves (motor + encoder share one 6-pin lead on the encoder motors)."""
    m = D.MOTORS[ctx["motor"]]
    y_out = D.GEARBOX_Y_IN + m["gear_t"]
    if ctx["motor"] == "hiwonder":
        u, v = 59.6, 6.0
    elif ctx["motor"] == "yahboom":
        u, v = m["body_end_u"] + 4.0, -9.4
    elif ctx["motor"] == "fit0450":
        u, v = 57.0, 14.0 if m["pcb_side"] == "out" else -m["gear_t"] - 14.0
    else:
        u, v = m["body_end_u"] - 3.0, -m["gear_t"] / 2
    return (sx * (D.AXLE_X + u), sy * (y_out + v), D.SHAFT_Z + 9.0)


def cable_routes(ctx):
    g, bb = ctx["geo"], ctx["bb"]
    B1, T0, T1 = g.B1, g.T0, g.T1
    w2p = ctx["pi_vendor_to_world"]
    sw_x, sw_y = ctx["switch_xy"]
    sw_z = T0 - D.SWITCH_BODY[2] - 1.0                              # switch QC terminals under the plate
    drv = "mdd3a" if ctx["driver"] == "mdd3a" else "tb6612"
    dbb = bb[drv]
    d_front, d_rear = dbb.max.X, dbb.min.X
    lane_z = B1 + 31.0                      # over the motors, under the top-plate screw heads
    gpio_pwr = w2p(D.PI_GPIO_PIN1_VENDOR_X - 1.27, -24.5, 9.0)    # header pins 2/4 (5 V), 6 (GND)
    gpio_uart0 = w2p(D.PI_GPIO_PIN1_VENDOR_X - 6.35, -24.5, 9.0)  # pins 8/10 (UART0 TX/RX, GPIO14/15) -> LiDAR
    gpio_uart2 = w2p(D.PI_GPIO_PIN1_VENDOR_X - 7.62, -22.0, 9.0)  # pins 7/29 (UART2 TX/RX, GPIO4/5) -> ESP32
    cam_x = sum(D.PI_CAM_VENDOR[0]) / 2
    cam_conn = w2p(cam_x, 20.0, 4.0)
    cb = bb["camera"]
    lb = bb["lidar"]
    brd = bb["lidar_board"]
    eb = bb["esp32"]
    buck = bb["buck_5v"]
    c1_conn = (lb.min.X - 1.0, 0.0, lb.min.Z + 6.0)               # ASSUMED: LiDAR lead exits the base's rear face
    over = brd.max.Z + 3.0
    routes = [
        ("cable_batt_xt30", "18awg", [(-10, 25.6, B1 + 10), (-15, 30, B1 + 5), (-15, 36, B1 + 2.6)]),
        ("cable_xt30_fuse", "18awg", [(13.5, 36, B1 + 2.6), (22, 30, B1 + 18), (22, -30, B1 + 18), (18.5, -37.5, B1 + 5)]),
        ("cable_fuse_switch", "18awg", [(-18.5, -37.5, B1 + 5), (-26, -30, B1 + 18), (-40, -20, B1 + 24),
                                       (-88, -6, B1 + 24), (sw_x, sw_y - 4, sw_z)]),
        ("cable_switch_driver", "20awg", [(sw_x, sw_y + 4, sw_z), (-80, 14, B1 + 26), (-20, 14, B1 + 28),
                                          (d_rear - 1.0, 8, T0 - 12)]),
        ("cable_switch_buck5", "20awg", [(sw_x, sw_y - 4, sw_z), (-80, -23, B1 + 22), (30, -23, B1 + 22),
                                         (buck.min.X - 1.0, -10, B1 + 6)]),
        ("cable_buck5_pi", "20awg", [(buck.max.X - 12.7, buck.max.Y, B1 + 6), (70, 11, B1 + 20), (92, 2, B1 + 25),
                                    (92, 2, T1 + 3), (80, -24.5, gpio_pwr[2]), gpio_pwr]),
        ("cable_5v_esp32", "22awg", [(buck.min.X + 4, buck.max.Y, B1 + 6), (30, 22, B1 + 20), (-40, 18, B1 + 20),
                                     (-55, 10, eb.max.Z + 1.5)]),
        ("cable_buck5_lidar", "22awg", [(buck.max.X - 6, buck.max.Y, B1 + 6), (75, 8, B1 + 25), (90, 5, T0 - 2),
                                        (90, 5, T1 + 3), (97, 43, T1 + 12), (97, 43, over), (60, 34, over),
                                        (c1_conn[0] - 2, 8, over), c1_conn]),
        ("cable_lidar_uart", "signal", [c1_conn, (c1_conn[0] - 2, -8, over), (30, -43, over), (65, -43, brd.min.Z - 5),
                                        (72, -30, gpio_uart0[2] + 8), gpio_uart0]),
        ("cable_driver_esp32", "signal", [(d_rear + 6, -12, T0 - 18), (-5, -12, B1 + 20), (-45, -6, B1 + 20),
                                          (-55, -4, eb.max.Z + 1.5)]),
        ("cable_imu_esp32", "signal", [(D.IMU_XY[0] - 9, 0, T0 - 13), (-45, 6, B1 + 20), (-55, 8, eb.max.Z + 1.5)]),
        ("cable_us_esp32", "signal", [(103, -15, T0 - 15), (95, -19, B1 + 20), (-60, -19, B1 + 20), (-65, -12, eb.max.Z + 1.5)]),
        ("cable_esp32_pi", "signal", [(-50, -10, eb.max.Z + 1.5), (-40, -19, B1 + 21), (88, -19, B1 + 21),
                                     (92, -2, B1 + 25), (92, -2, T1 + 3), (80, -26, gpio_uart2[2]), gpio_uart2]),
        ("cable_camera_ffc", "ffc", [(cb.min.X + 2, 0, cb.min.Z + 1), (96, 10, T1 + 1), (80, 32, T1 + 1), (cam_conn[0], 32, T1 + 1),
                                     (cam_conn[0], 32, cam_conn[2] + 3), (cam_conn[0], cam_conn[1], cam_conn[2] + 3),
                                     cam_conn]),
    ]
    for fr, sx in (("f", 1), ("r", -1)):
        for lr, sy in (("l", 1), ("r", -1)):
            cx, cy, cz = motor_connector(ctx, sx, sy)
            routes.append((f"cable_driver_motor_{fr}{lr}", "22awg",
                           [(d_rear + 8 if sx < 0 else d_front - 8, sy * 18.5, T0 - 9), (sx * 22, sy * 36.5, lane_z),
                            (cx - sx * 6, sy * 36.5, lane_z), (cx, cy, cz)]))
            if ctx["motor"] != "tt_stock":
                xe = -45 if sx > 0 else -75
                routes.append((f"cable_enc_{fr}{lr}", "signal",
                               [(cx, cy, cz + 2), (cx - sx * 8, sy * 36.5, lane_z + 2), (sx * 72, sy * 19.5, B1 + 20),
                                (xe, sy * 19.5, B1 + 20), (xe, sy * 11, eb.max.Z + 1.5)]))
    return routes


# off-the-shelf cable chosen per route (BOM + report)
CABLE_CHOICE = {
    "cable_batt_xt30": "battery pack lead (as supplied) + XT30U-M, 18 AWG silicone",
    "cable_xt30_fuse": "XT30U-F pigtail 18 AWG into the inline ATM holder's lead (solder + heat-shrink)",
    "cable_fuse_switch": "18 AWG silicone, 6.3 mm insulated female QC on the rocker's input terminal",
    "cable_switch_driver": "20 AWG silicone pair (+ GND return) from the rocker output QC to the MDD3A VB+/VB- terminal; "
                           "470 uF/25 V soldered across VB+/VB- at the terminal",
    "cable_switch_buck5": "20 AWG silicone pair to the D36V50F5 VIN/GND (both 20 AWG wires crimped together in the "
                          "rocker's output QC = the split); 100 uF/25 V across VIN",
    "cable_buck5_pi": "20 AWG pair, 2.54 mm Dupont housings on GPIO pins 2/4 (+5 V) and 6/14 (GND), <= 15 cm",
    "cable_5v_esp32": "22 AWG pair from the D36V50F5 VOUT to the ESP32 VIN/GND (also feeds encoders + HC-SR04)",
    "cable_buck5_lidar": "22 AWG pair from the D36V50F5 VOUT through the top-plate front slot to the C1 lead's VCC/GND",
    "cable_lidar_uart": "C1 XH2.54-5P lead: TX/RX/GND as F Dupont to Pi GPIO 14/15 (pins 8/10, UART0) + GND pin 9",
    "cable_driver_esp32": "5 x F-F Dupont jumpers (M1A/M1B/M2A/M2B PWM + GND)",
    "cable_imu_esp32": "4 x F-F Dupont jumpers (VCC 3.3 V, GND, SDA, SCL)",
    "cable_us_esp32": "4 x F-F Dupont jumpers 20-30 cm (VCC, TRIG, ECHO via 1k/2k divider, GND)",
    "cable_esp32_pi": "3 x F-F Dupont jumpers 30 cm (TX, RX, GND) to Pi GPIO4/5 (pins 7/29, UART2, dtoverlay=uart2-pi5)",
    "cable_camera_ffc": "Raspberry Pi 5 camera cable, 22-pin 'standard-mini' (official 200 mm)",
    "cable_driver_motor_": "22 AWG pair from the MDD3A outputs to each motor (encoder motors: the motor pins of the 6-pin lead)",
    "cable_enc_": "motor's own 6-pin lead (PH2.0 / XH) + Dupont breakout to the ESP32 (A, B, VCC 3.3 V, GND)",
}
