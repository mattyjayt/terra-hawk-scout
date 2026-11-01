#!/usr/bin/env python3
"""robot_params.json -> MuJoCo MJCF (sim/scout.xml and variants).

Usage: python sim/gen_mjcf.py [params.json] [--floor carpet|hard_floor] [--motor tt_1_48_6v]
                              [--payload] [--threshold X] [--out sim/scout.xml]

Same params file and same lumping (sim/scout_params.py) as urdf/gen_description.py, so the
URDF and the MJCF always agree on masses and frames. World frame = REP-103 at start: the
chassis body origin is base_link (centre of rotation, at axle height).

Wheels: hinge joints about body +Y (positive = forward roll on both sides), `motor` actuators
in N m with ctrlrange = +-stall torque; scenarios.py closes the DC-motor model
(tau = Ts*u - Ts/w0*omega) and a per-wheel PI speed loop in Python. Contact: elliptic cones,
condim 4 (sliding + torsional friction) with the floor preset winning via priority.
Rolling resistance is NOT MuJoCo rolling friction (condim 6 made the wheels micro-hop and lose
contact ~50 % of steps in testing); scenarios.py applies it as a wheel torque Crr*N*r instead.
"""
import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from scout_params import WHEELS, Params  # noqa: E402

MOTORS = json.loads((HERE / "motor_profiles.json").read_text())
FLOORS = json.loads((HERE / "floor_presets.json").read_text())

THRESHOLD = {"height": 0.010, "depth": 0.040, "width": 0.8, "edge_radius": 0.0015}   # BRIEF: 10 mm; 40 mm deep; 1.5 mm edge radius ASSUMED


def v(x):
    return " ".join(f"{a:.6g}" for a in x)


def default_params():
    p = ROOT / "out" / "robot_params.json"
    return p if p.exists() else HERE / "params_v1_fallback.json"


def build(params, floor="carpet", motor="tt_1_48_6v", payload=False, threshold_x=None,
          threshold_floor="hard_floor", supply_voltage=None, mu=None):
    P = params if isinstance(params, Params) else Params(params)
    M = MOTORS[motor]
    F = list(FLOORS[floor]["friction"])
    if mu is not None:
        F[0] = mu
    Ts = M["stall_torque"] * (supply_voltage or M["rated_voltage"]) / M["rated_voltage"]
    bz = P.axle_z
    I = P.chassis_inertia6()
    fric_loss = M["stall_torque"] * M["no_load_current"] / M["stall_current"]
    com = [P.chassis_com[0], P.chassis_com[1], P.chassis_com[2] - bz]

    boxes = []
    for name, c, s in P.boxes():
        cc = [c[0], c[1], c[2] - bz]
        rgba = "0.85 0.85 0.85 1"
        if "motor" in name:
            rgba = "0.89 0.75 0.27 1"
        elif "battery" in name:
            rgba = "0.23 0.48 0.84 1"
        elif "lidar" == name:
            rgba = "0.13 0.16 0.19 1"
        elif "pi5" in name:
            rgba = "0.12 0.44 0.26 1"
        elif "plate" not in name:
            rgba = "0.55 0.35 0.68 1"
        boxes.append(f'      <geom name="{name}" type="box" pos="{v(cc)}" size="{v([x / 2 for x in s])}" rgba="{rgba}" class="chassis"/>')

    wheels = []
    for k in WHEELS:
        w = P.wheels[k]
        Iw = w["inertia"]
        pos = [w["pos"][0], w["pos"][1], w["pos"][2] - bz]
        wheels.append(f"""      <body name="wheel_{k}" pos="{v(pos)}">
        <inertial pos="0 0 0" mass="{w['mass']:.6g}" diaginertia="{v([Iw[0], Iw[1], Iw[2]])}"/>
        <joint name="wheel_{k}_joint" type="hinge" axis="0 1 0" armature="{M['armature']:.6g}" frictionloss="{fric_loss:.6g}"/>
        <geom name="wheel_{k}" type="cylinder" size="{w['radius']:.6g} {w['width'] / 2:.6g}" euler="90 0 0" class="wheel"/>
        <geom type="box" size="{w['radius'] * 0.7:.4g} {w['width'] / 2 + 0.0005:.4g} 0.004" rgba="0.9 0.9 0.9 1" contype="0" conaffinity="0" mass="0"/>
      </body>""")

    pl = ""
    if payload:
        p = P.payload
        pc = [p["com"][0], p["com"][1], p["com"][2] - bz]
        pl = (f'      <body name="payload" pos="{v(pc)}">\n'
              f'        <geom name="payload" type="box" size="{v([x / 2 for x in p["size"]])}" mass="{p["mass"]:.6g}" rgba="0.2 0.7 0.3 1" class="chassis"/>\n'
              '      </body>')

    thr = ""
    if threshold_x is not None:
        # Rounded-rectangle profile: two boxes + two capsules (edge radius rc). A sharp box edge gives a
        # wrong (horizontal) contact normal in MuJoCo's cylinder-box collider -> no climb at all.
        T = THRESHOLD
        TF = FLOORS[threshold_floor]["friction"]
        h, dd, ww, rc = T["height"], T["depth"], T["width"], T["edge_radius"]
        att = f'priority="1" condim="4" friction="{v(TF)}" rgba="0.55 0.35 0.2 1" solref="0.004 1"'
        thr = "\n".join([
            f'    <body name="threshold" pos="{threshold_x:.4g} 0 0">',
            f'      <geom name="threshold" type="box" pos="0 0 {h / 2:.5g}" size="{dd / 2 - rc:.5g} {ww / 2:.4g} {h / 2:.5g}" {att}/>',
            f'      <geom name="threshold_low" type="box" pos="0 0 {(h - rc) / 2:.5g}" size="{dd / 2:.5g} {ww / 2:.4g} {(h - rc) / 2:.5g}" {att}/>',
            f'      <geom name="threshold_edge_front" type="capsule" pos="{-(dd / 2 - rc):.5g} 0 {h - rc:.5g}" size="{rc:.5g} {ww / 2:.4g}" euler="90 0 0" {att}/>',
            f'      <geom name="threshold_edge_rear" type="capsule" pos="{dd / 2 - rc:.5g} 0 {h - rc:.5g}" size="{rc:.5g} {ww / 2:.4g}" euler="90 0 0" {att}/>',
            '    </body>'])

    sens = P.sensors.get("imu", {"pos": [0, 0, bz]})["pos"]
    imu = [sens[0], sens[1], sens[2] - bz]
    acts = "\n".join(f'    <motor name="m_{k}" joint="wheel_{k}_joint" gear="1" ctrllimited="true" ctrlrange="{-Ts:.6g} {Ts:.6g}"/>' for k in WHEELS)
    meta = {"params": str(P.path), "floor": floor, "mu": F[0], "motor": motor, "payload": payload,
            "threshold_x": threshold_x, "stall_torque": Ts, "no_load_speed": M["no_load_speed"] * (supply_voltage or M["rated_voltage"]) / M["rated_voltage"],
            "crr": FLOORS[floor]["friction"][2] / P.wheel_radius, "total_mass": P.total_mass + (P.payload["mass"] if payload else 0)}
    return f"""<mujoco model="scout">
  <!-- GENERATED by sim/gen_mjcf.py from {P.path.name} ({P.version}); do not hand-edit.
       meta: {json.dumps(meta)} -->
  <compiler angle="degree" autolimits="true"/>
  <option timestep="0.001" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <size memory="8M"/>
  <visual><global offwidth="1280" offheight="960"/><quality shadowsize="2048"/></visual>
  <default>
    <default class="chassis"><geom density="0" condim="3" solref="0.004 1"/></default>
    <default class="wheel"><geom density="0" condim="4" friction="0.1 0.0001 0.00001" solref="0.004 1" solimp="0.95 0.99 0.001" rgba="0.12 0.12 0.12 1"/></default>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="{v(FLOORS[floor]['rgba'][:3])}" rgb2="{v([c * 0.8 for c in FLOORS[floor]['rgba'][:3]])}"/>
    <material name="floor" texture="grid" texrepeat="10 10" reflectance="0"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.6 0.7 0.8" rgb2="0.15 0.15 0.2" width="256" height="256"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true" diffuse="0.8 0.8 0.8"/>
    <light pos="1 -1 2" dir="-0.5 0.5 -1" diffuse="0.3 0.3 0.3"/>
    <geom name="floor" type="plane" size="20 20 0.1" material="floor" priority="1" condim="4" friction="{v(F)}" solref="0.004 1"/>
{thr}
    <body name="chassis" pos="0 0 {bz + 0.0005:.6g}">
      <freejoint name="root"/>
      <inertial pos="{v(com)}" mass="{P.chassis_mass:.6g}" fullinertia="{v(I)}"/>
      <camera name="chase" mode="trackcom" pos="-0.55 -0.45 0.35" xyaxes="0.63 -0.77 0 0.3 0.25 0.92"/>
      <site name="imu" pos="{v(imu)}" size="0.005"/>
{chr(10).join(boxes)}
{pl}
{chr(10).join(wheels)}
    </body>
  </worldbody>
  <actuator>
{acts}
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>
</mujoco>
""", meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("params", nargs="?", default=None)
    ap.add_argument("--floor", default="carpet", choices=list(FLOORS.keys() - {"_doc"}))
    ap.add_argument("--motor", default="tt_1_48_6v", choices=[k for k in MOTORS if not k.startswith("_")])
    ap.add_argument("--payload", action="store_true")
    ap.add_argument("--threshold", type=float, default=None)
    ap.add_argument("--out", default=str(HERE / "scout.xml"))
    a = ap.parse_args()
    xml, meta = build(a.params or default_params(), a.floor, a.motor, a.payload, a.threshold)
    Path(a.out).write_text(xml)
    import mujoco
    m = mujoco.MjModel.from_xml_path(a.out)
    print(json.dumps({"out": a.out, "nbody": m.nbody, "mass": round(float(sum(m.body_mass)), 4), **meta}))


if __name__ == "__main__":
    main()
