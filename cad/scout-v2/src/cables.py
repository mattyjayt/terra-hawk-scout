"""Approximate harness routes (round 2 cable plan).

Each route is (label, kind, [waypoints in mm, REP-103]). The model sweeps a thin tube through the
waypoints (labels cable_*); checks/fit_check_v2.py reports each route's length (+15 % slack) and the
chosen off-the-shelf cable. The routes are drawn from the parts' own bounding boxes so they follow
every variant. They are a plan (which lane, which pass-through, how long), not a wire-exact model:
cable tubes are excluded from the interference check.

kind: 18awg (battery/power), 20awg (5 V to the Pi, motor rail), 22awg (motor leads), signal
(Dupont/JST jumpers), ffc (Pi 5 camera cable), usb.
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
    pcb = ctx["pi_pcb_top"]
    w2p = ctx["pi_vendor_to_world"]
    sw_x, sw_y = ctx["switch_xy"]
    drv = "mdd3a" if ctx["driver"] == "mdd3a" else "tb6612"
    dbb = bb[drv]
    d_front, d_rear = dbb.max.X, dbb.min.X
    lane_z = B1 + 31.0                      # over the motors, under the top-plate screw heads
    gpio_pwr = w2p(D.PI_GPIO_PIN1_VENDOR_X - 1.27, -24.5, 9.0)    # header pins 2/4 (5 V), 6 (GND)
    gpio_uart = w2p(D.PI_GPIO_PIN1_VENDOR_X - 6.35, -24.5, 9.0)   # pins 8/10 (UART TX/RX)
    cam_x = sum(D.PI_CAM_VENDOR[0]) / 2
    cam_conn = w2p(cam_x, 20.0, 4.0)
    cb = bb["camera"]
    lb = bb["lidar_adapter"]
    usb_port = w2p(30.0, -19.0, 8.0)      # lower USB stack, plug enters from the rear (vendor +X end)
    eb = bb["esp32"]
    routes = [
        ("cable_batt_xt30", "18awg", [(-10, 25.6, B1 + 10), (-15, 30, B1 + 5), (-15, 36, B1 + 2.6)]),
        ("cable_xt30_fuse", "18awg", [(13.5, 36, B1 + 2.6), (22, 30, B1 + 18), (22, -30, B1 + 18), (18.5, -37.5, B1 + 5)]),
        ("cable_fuse_switch", "18awg", [(-18.5, -37.5, B1 + 5), (-24, -30, B1 + 18), (-24, 20, B1 + 18),
                                       (20, 36.5, lane_z), (98, 36.5, lane_z), (sw_x, sw_y, T0 - 10)]),
        ("cable_switch_buck5", "18awg", [(sw_x, sw_y - 2, T0 - 10), (95, 18, B1 + 20), (66, 16, B1 + 14), (63.9, 8, B1 + 6)]),
        ("cable_switch_buckm", "18awg", [(sw_x, sw_y - 2, T0 - 10), (95, 12, B1 + 16), (88.8, 4, B1 + 6)]),
        ("cable_buck5_pi", "20awg", [(50.7, 13.2, B1 + 6), (70, 11, B1 + 20), (92, 2, B1 + 25), (92, 2, T1 + 3),
                                    (80, -24.5, gpio_pwr[2]), gpio_pwr]),
        ("cable_buckm_driver", "20awg", [(78.2, -9.4, B1 + 6), (70, -16, B1 + 18), (40, -16, B1 + 24),
                                         (d_front + 2, -14, T0 - 9)]),
        ("cable_driver_esp32", "signal", [(d_rear - 1, 0, T0 - 9), (-45, 0, B1 + 20), (-55, 0, eb.max.Z + 1.5)]),
        ("cable_imu_esp32", "signal", [(25, 9, T0 - 13), (14, 12, B1 + 26), (-40, 18, B1 + 20), (-55, 10, eb.max.Z + 1.5)]),
        ("cable_us_esp32", "signal", [(103, -15, T0 - 15), (95, -19, B1 + 20), (-60, -19, B1 + 20), (-65, -12, eb.max.Z + 1.5)]),
        ("cable_esp32_pi", "signal", [(-50, -10, eb.max.Z + 1.5), (-40, -19, B1 + 21), (88, -19, B1 + 21),
                                     (92, -2, B1 + 25), (92, -2, T1 + 3), (80, -26, gpio_uart[2]), gpio_uart]),
        ("cable_lidar_pi", "usb", [(lb.min.X, 0, lb.max.Z - 2), (lb.min.X - 15, 0, lb.min.Z - 3),
                                   (-28, -19, pcb + 20), (usb_port[0] - 12, -19, usb_port[2]), (usb_port[0] - 4, -19, usb_port[2])]),
        ("cable_camera_ffc", "ffc", [(cb.min.X + 2, 0, cb.min.Z + 1), (96, 10, T1 + 1), (80, 32, T1 + 1), (cam_conn[0], 32, T1 + 1),
                                     (cam_conn[0], 32, cam_conn[2] + 3), (cam_conn[0], cam_conn[1], cam_conn[2] + 3),
                                     cam_conn]),
    ]
    for fr, sx in (("f", 1), ("r", -1)):
        for lr, sy in (("l", 1), ("r", -1)):
            cx, cy, cz = motor_connector(ctx, sx, sy)
            routes.append((f"cable_driver_motor_{fr}{lr}", "22awg",
                           [(d_rear + 8 if sx < 0 else d_front - 8, sy * 22.5, T0 - 9), (sx * 22, sy * 36.5, lane_z),
                            (cx - sx * 6, sy * 36.5, lane_z), (cx, cy, cz)]))
            if ctx["motor"] != "tt_stock":
                xe = -45 if sx > 0 else -75
                routes.append((f"cable_enc_{fr}{lr}", "signal",
                               [(cx, cy, cz + 2), (cx - sx * 8, sy * 36.5, lane_z + 2), (sx * 72, sy * 19.5, B1 + 20),
                                (xe, sy * 19.5, B1 + 20), (xe, sy * 11, eb.max.Z + 1.5)]))
    return routes


# off-the-shelf cable chosen per route (BOM + report)
CABLE_CHOICE = {
    "cable_batt_xt30": "battery pack lead (as supplied) + XT30 male, 18 AWG silicone",
    "cable_xt30_fuse": "18 AWG silicone wire, XT30 female pigtail",
    "cable_fuse_switch": "18 AWG silicone wire",
    "cable_switch_buck5": "18 AWG silicone wire (split after the switch with a WAGO 221-413 or solder splice)",
    "cable_switch_buckm": "18 AWG silicone wire",
    "cable_buck5_pi": "20 AWG pair, 2.54 mm Dupont housings on GPIO pins 2/4 (+5 V) and 6/14 (GND)",
    "cable_buckm_driver": "20 AWG pair into the MDD3A power terminal",
    "cable_driver_esp32": "4 x F-F Dupont jumpers (M1A/M1B/M2A/M2B) + GND",
    "cable_imu_esp32": "4 x F-F Dupont jumpers (VCC, GND, SDA, SCL)",
    "cable_us_esp32": "4 x F-F Dupont jumpers 20-30 cm (VCC, TRIG, ECHO via divider, GND)",
    "cable_esp32_pi": "3 x F-F Dupont jumpers 30 cm (TX, RX, GND) to Pi GPIO 14/15; or a USB-C cable from the rear",
    "cable_lidar_pi": "LiDAR kit USB adapter + short USB-A cable (LD19 kit: ZH1.5-4P to adapter)",
    "cable_camera_ffc": "Raspberry Pi 5 camera cable, 22-pin 'standard-mini' (official 200 mm / 300 mm / 500 mm)",
    "cable_driver_motor_": "22 AWG pair from the MDD3A outputs to each motor (encoder motors: the motor pins of the 6-pin lead)",
    "cable_enc_": "motor's own 6-pin lead (PH2.0 / XH) + Dupont breakout to the ESP32 (A, B, VCC 3.3 V, GND)",
}
