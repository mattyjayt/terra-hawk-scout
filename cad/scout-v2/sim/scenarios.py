#!/usr/bin/env python3
"""Scout physics scenarios in MuJoCo -> sim/results.json, sim/plots/*.png, sim/videos/*.mp4.

Usage: python sim/scenarios.py [params.json] [--motor tt_1_48_6v] [--no-video] [--only a,b,c,d,e]

Drive model (per wheel): ESP32-style PI wheel-speed loop at 100 Hz -> PWM duty u in [-1, 1] ->
linear DC motor at the wheel: tau = Ts*u - (Ts/w0)*omega, clipped to +-Ts (stall-current limit),
plus joint frictionloss = gearbox drag. Short-brake (TB6612 both IN high) = u=0; coast = tau 0.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_mjcf  # noqa: E402
from scout_params import WHEELS, Params  # noqa: E402

LEFT, RIGHT = ("fl", "rl"), ("fr", "rr")
PLOTS, VIDEOS = HERE / "plots", HERE / "videos"


def quat_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Sim:
    def __init__(self, P, motor="tt_1_48_6v", ctrl_hz=100, kp=0.08, ki=1.5, record=None, **build_kw):
        self.P = P
        xml, self.meta = gen_mjcf.build(P, motor=motor, **build_kw)
        self.m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(self.m)
        M = gen_mjcf.MOTORS[motor]
        self.M = M
        self.Ts = self.meta["stall_torque"]
        fl = M["stall_torque"] * M["no_load_current"] / M["stall_current"]
        self.w0 = self.meta["no_load_speed"] / (1 - fl / M["stall_torque"])   # so net no-load speed = datasheet
        self.kt = M["stall_torque"] / (M["stall_current"] - M["no_load_current"])
        self.r = P.wheel_radius
        self.jq = {k: self.m.joint(f"wheel_{k}_joint").qposadr[0] for k in WHEELS}
        self.jv = {k: self.m.joint(f"wheel_{k}_joint").dofadr[0] for k in WHEELS}
        self.act = {k: self.m.actuator(f"m_{k}").id for k in WHEELS}
        self.wgeom = {self.m.geom(f"wheel_{k}").id: k for k in WHEELS}
        self.chassis_geoms = {i for i in range(self.m.ngeom) if self.m.geom_bodyid[i] == self.m.body("chassis").id}
        self.thr = ({i for i in range(self.m.ngeom) if self.m.geom_bodyid[i] == self.m.body("threshold").id}
                    if build_kw.get("threshold_x") is not None else set())
        self.dec = max(1, int(round(1 / (ctrl_hz * self.m.opt.timestep))))
        self.kp, self.ki = kp, ki
        self.ref = {k: 0.0 for k in WHEELS}
        self.integ = {k: 0.0 for k in WHEELS}
        self.u = {k: 0.0 for k in WHEELS}
        self.mode = "pi"   # pi | brake | coast
        self.n = 0
        self.tau = {k: 0.0 for k in WHEELS}
        self.crr = self.meta["crr"]
        self._N = {k: 0.0 for k in WHEELS}
        self.frames, self.record = [], record
        if record:
            self.renderer = mujoco.Renderer(self.m, 480, 640)
            self.cam = mujoco.MjvCamera()
            self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.cam.trackbodyid = self.m.body("chassis").id
            self.cam.distance, self.cam.azimuth, self.cam.elevation = record.get("dist", 0.75), record.get("az", 135), record.get("el", -25)
        mujoco.mj_forward(self.m, self.d)
        for _ in range(300):        # settle on the floor
            mujoco.mj_step(self.m, self.d)
        self.d.time = 0.0

    # ---------------------------------------------------------------- control
    def set_lr(self, wl, wr):
        for k in LEFT:
            self.ref[k] = wl
        for k in RIGHT:
            self.ref[k] = wr

    def set_twist(self, v, w, track=None):
        T = track or self.P.track
        self.set_lr((v - w * T / 2) / self.r, (v + w * T / 2) / self.r)

    def wheel_w(self, k):
        return self.d.qvel[self.jv[k]]

    def step(self):
        if self.n % self.dec == 0 and self.mode == "pi":
            dt = self.dec * self.m.opt.timestep
            for k in WHEELS:
                e = self.ref[k] - self.wheel_w(k)
                ff = self.ref[k] / self.w0 + math.copysign(self.M["no_load_current"] / self.M["stall_current"], self.ref[k]) * (abs(self.ref[k]) > 1e-6)
                u = ff + self.kp * e + self.ki * self.integ[k]
                if -1 < u < 1 or u * e < 0:
                    self.integ[k] += e * dt
                self.u[k] = max(-1.0, min(1.0, u))
        for k in WHEELS:
            w = self.wheel_w(k)
            if self.mode == "coast":
                t = 0.0
            else:
                u = 0.0 if self.mode == "brake" else self.u[k]
                t = self.Ts * u - self.Ts / self.w0 * w
            t = max(-self.Ts, min(self.Ts, t))
            self.tau[k] = t
            self.d.ctrl[self.act[k]] = t
        # rolling resistance as a wheel torque (see gen_mjcf docstring): Crr * N * r, smoothed sign
        if self.n % 5 == 0:
            self._N = self.wheel_normals()
        for k in WHEELS:
            w = self.wheel_w(k)
            self.d.qfrc_applied[self.jv[k]] = -self.crr * self._N[k] * self.r * math.tanh(w / 0.5)
        mujoco.mj_step(self.m, self.d)
        self.n += 1
        if self.record and self.n % 33 == 0:
            self.renderer.update_scene(self.d, self.cam)
            self.frames.append(self.renderer.render().copy())

    # ---------------------------------------------------------------- state
    def pose(self):
        x, y, z = self.d.qpos[0:3]
        r, p, yw = quat_rpy(self.d.qpos[3:7])
        return x, y, z, r, p, yw

    def body_vel(self):
        yaw = self.pose()[5]
        vx, vy = self.d.qvel[0:2]
        return math.cos(yaw) * vx + math.sin(yaw) * vy, -math.sin(yaw) * vx + math.cos(yaw) * vy

    def yaw_rate(self):
        return self.d.qvel[5]   # freejoint angular velocity is in the body frame -> z = yaw rate when level

    def wheel_normals(self):
        f6 = np.zeros(6)
        out = {k: 0.0 for k in WHEELS}
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            for g in (c.geom1, c.geom2):
                if g in self.wgeom:
                    mujoco.mj_contactForce(self.m, self.d, i, f6)
                    out[self.wgeom[g]] += f6[0]
        return out

    def chassis_hits_threshold(self):
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if (c.geom1 in self.thr and c.geom2 in self.chassis_geoms) or (c.geom2 in self.thr and c.geom1 in self.chassis_geoms):
                return True
        return False

    def current(self, k):
        return self.M["no_load_current"] + abs(self.tau[k]) / self.kt

    def save_video(self, name):
        import imageio.v2 as imageio
        VIDEOS.mkdir(exist_ok=True)
        path = VIDEOS / f"{name}.mp4"
        imageio.mimsave(path, self.frames, fps=30, quality=7, macro_block_size=16)
        return str(path.relative_to(HERE.parent))


def log_row(s):
    x, y, z, r, p, yw = s.pose()
    vx, vy = s.body_vel()
    n = s.wheel_normals()
    return {"t": s.d.time, "x": x, "y": y, "z": z, "roll": r, "pitch": p, "yaw": yw, "vx": vx, "vy": vy, "wz": s.yaw_rate(),
            **{f"w_{k}": s.wheel_w(k) for k in WHEELS}, **{f"tau_{k}": s.tau[k] for k in WHEELS},
            **{f"N_{k}": n[k] for k in WHEELS}, **{f"q_{k}": s.d.qpos[s.jq[k]] for k in WHEELS}}


def run(s, seconds, every=10, hook=None, stop=None):
    rows = []
    steps = int(seconds / s.m.opt.timestep)
    for i in range(steps):
        if hook:
            hook(s)
        s.step()
        if i % every == 0:
            rows.append(log_row(s))
            if stop is not None and stop(s):
                break
    return {k: np.array([r[k] for r in rows]) for k in rows[0]}


def accel20(t, v, sign=1):
    """Peak sign*acceleration averaged over 20 ms (raw gradients spike on contact impacts)."""
    dt = t[1] - t[0]
    w = max(1, int(round(0.02 / dt)))
    a = (v[w:] - v[:-w]) / (t[w:] - t[:-w])
    return float(np.max(sign * a))


def stats_window(L, key, t0, t1=None):
    m = (L["t"] >= t0) & (L["t"] <= (t1 if t1 is not None else L["t"][-1]))
    return L[key][m]


# ===================================================================== scenarios
def scen_a(P, motor, plt):
    out = {}
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for floor in ("carpet", "hard_floor"):
        for payload in (False, True):
            s = Sim(P, motor, floor=floor, payload=payload)
            s.set_twist(0.3, 0.0)
            L = run(s, 4.0, every=2)
            v_ss = float(np.mean(stats_window(L, "vx", 2.5)))
            i90 = np.argmax(L["vx"] >= 0.27) if np.any(L["vx"] >= 0.27) else None
            taus = np.stack([L[f"tau_{k}"] for k in WHEELS])
            tau_ss = float(np.mean(np.abs(taus[:, L["t"] >= 2.5])))
            def fmin(a, win=10):   # min over time of the 20 ms (10 x 2 ms) running max (ignores 1-step contact flicker)
                return float(min(a[i:i + win].max() for i in range(0, len(a) - win)))
            nmin = min(fmin(L[f"N_{k}"][5:]) for k in WHEELS)
            nfront = min(fmin(L["N_fl"][5:]), fmin(L["N_fr"][5:]))
            acc = accel20(L["t"], L["vx"])
            key = f"{floor}{'_payload' if payload else ''}"
            out[key] = {"target_mps": 0.3, "achieved_mps": round(v_ss, 4), "t_reach_90pct_s": round(float(L["t"][i90]), 3) if i90 is not None else None,
                        "peak_wheel_torque_Nm": round(float(np.abs(taus).max()), 4), "steady_wheel_torque_Nm": round(tau_ss, 4),
                        "steady_torque_pct_stall": round(100 * tau_ss / s.Ts, 1),
                        "steady_current_A_per_motor": round(s.M["no_load_current"] + tau_ss / s.kt, 3),
                        "peak_current_A_per_motor": round(s.M["no_load_current"] + float(np.abs(taus).max()) / s.kt, 3),
                        "peak_pitch_deg": round(math.degrees(float(np.abs(L["pitch"]).max())), 3),
                        "min_wheel_normal_N_20ms": round(nmin, 3), "min_front_wheel_normal_N_20ms": round(nfront, 3),
                        "peak_accel_mps2_20ms": round(acc, 2),
                        "pass": bool(abs(v_ss - 0.3) < 0.015 and i90 is not None)}
            ax[0].plot(L["t"], L["vx"], label=key)
            ax[1].plot(L["t"], L["tau_fl"], label=key)
    # worst case: firmware slams full duty (u=1) from standstill with the 300 g payload at the rear
    for floor in ("hard_floor", "carpet"):
        s = Sim(P, motor, floor=floor, payload=True)
        s.set_lr(s.w0 * 2, s.w0 * 2)
        L = run(s, 0.6, every=1)
        nf = np.minimum(L["N_fl"], L["N_fr"])
        win = 20
        low = (nf < 0.05).astype(int)
        lift_ms = int(np.convolve(low, np.ones(win, int), "valid").max()) if len(low) > win else 0
        out[f"full_duty_step_payload_{floor}"] = {
            "peak_accel_mps2_20ms": round(accel20(L["t"], L["vx"]), 2),
            "min_pitch_deg": round(math.degrees(float(L["pitch"].min())), 2),
            "max_nose_up_deg": round(-math.degrees(float(L["pitch"].min())), 2),
            "front_wheel_lift_20ms": bool(lift_ms >= win),
            "max_front_z_rise_mm": round(1000 * float((L["z"] - L["z"][0]).max() + P.wheels["fl"]["pos"][0] * np.sin(-L["pitch"]).max()), 2),
            "pass": bool(lift_ms < win)}
        ax[0].plot(L["t"], L["vx"], ls=":", label=f"full duty, payload, {floor}")
    ax[0].axhline(0.3, ls="--", c="k"); ax[0].set(xlabel="t [s]", ylabel="forward speed [m/s]", title="(a) straight 0.3 m/s step")
    ax[1].axhline(s.Ts, ls=":", c="r", label="stall"); ax[1].set(xlabel="t [s]", ylabel="wheel_fl torque [N m]")
    for a in ax:
        a.legend(fontsize=7); a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(PLOTS / "a_straight.png", dpi=110); plt.close(fig)
    return out


def spin_once(P, motor, floor, mu=None, record=None, seconds=4.0, closed_loop=False):
    s = Sim(P, motor, floor=floor, mu=mu, record=record)
    state = {"w_cmd": 1.0, "i": 0.0}

    def hook(s):
        if closed_loop and s.n % s.dec == 0:   # outer gyro loop: raise wheel speed until true yaw rate = 1 rad/s
            e = 1.0 - s.yaw_rate()
            state["i"] += e * s.dec * s.m.opt.timestep
            state["w_cmd"] = 1.0 + 1.0 * e + 4.0 * state["i"]
        s.set_twist(0.0, state["w_cmd"])
    L = run(s, seconds, hook=hook)
    m = L["t"] >= seconds / 2
    wz = float(np.mean(L["wz"][m]))
    wr = float(np.mean(np.abs(np.stack([L[f"w_{k}"][m] for k in WHEELS]))))
    taus = np.abs(np.stack([L[f"tau_{k}"][m] for k in WHEELS]))
    eff_track = 2 * wr * s.r / wz if wz > 1e-3 else float("inf")
    drift = math.hypot(L["x"][-1] - L["x"][0], L["y"][-1] - L["y"][0])
    return s, L, {"floor": floor, "mu": mu or gen_mjcf.FLOORS[floor]["friction"][0], "commanded_yaw_rate": 1.0,
                  "achieved_yaw_rate": round(wz, 4), "wheel_speed_rad_s": round(wr, 3),
                  "wheel_surface_speed_mps": round(wr * s.r, 4),
                  "kinematic_slip": round(1 - wz * P.track / 2 / (wr * s.r), 3) if wr > 0 else None,
                  "effective_track_m": round(eff_track, 4), "effective_track_ratio": round(eff_track / P.track, 3),
                  "mean_wheel_torque_Nm": round(float(taus.mean()), 4), "max_wheel_torque_Nm": round(float(taus.max()), 4),
                  "torque_pct_stall": round(100 * float(taus.mean()) / s.Ts, 1),
                  "current_A_per_motor": round(s.M["no_load_current"] + float(taus.mean()) / s.kt, 3),
                  "centre_drift_m": round(drift, 4),
                  "pass": bool(abs(wz - 1.0) < 0.1)}


def scen_b(P, motor, plt, video):
    out = {"open_loop_nominal_kinematics": {}, "closed_loop_gyro": {}, "mu_sweep_carpet_open_loop": []}
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for floor in ("carpet", "hard_floor"):
        rec = {"dist": 0.6, "az": 120, "el": -35} if (video and floor == "carpet") else None
        s, L, r = spin_once(P, motor, floor, record=rec, seconds=6.0 if rec else 4.0)
        if rec:
            r["video"] = s.save_video("spin_in_place_carpet")
        out["open_loop_nominal_kinematics"][floor] = r
        ax[0].plot(L["t"], L["wz"], label=f"{floor} open-loop")
        ax[1].plot(L["t"], np.abs(L["tau_fl"]), label=f"{floor} open-loop")
        s, L, r = spin_once(P, motor, floor, closed_loop=True, seconds=5.0)
        out["closed_loop_gyro"][floor] = r
        ax[0].plot(L["t"], L["wz"], ls="--", label=f"{floor} gyro loop")
        ax[1].plot(L["t"], np.abs(L["tau_fl"]), ls="--", label=f"{floor} gyro loop")
    for mu in (0.4, 0.6, 0.8, 1.0):
        _, _, r = spin_once(P, motor, "carpet", mu=mu, seconds=3.0)
        out["mu_sweep_carpet_open_loop"].append({k: r[k] for k in ("mu", "achieved_yaw_rate", "effective_track_ratio", "mean_wheel_torque_Nm", "torque_pct_stall")})
    ax[0].axhline(1.0, c="k", ls=":"); ax[0].set(xlabel="t [s]", ylabel="yaw rate [rad/s]", title="(b) spin in place, cmd 1 rad/s")
    ax[1].axhline(gen_mjcf.MOTORS[motor]["stall_torque"], c="r", ls=":", label="stall"); ax[1].set(xlabel="t [s]", ylabel="|tau| wheel_fl [N m]")
    for a in ax:
        a.legend(fontsize=7); a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(PLOTS / "b_spin.png", dpi=110); plt.close(fig)
    return out


def scen_c(P, motor, plt, video):
    out = {}
    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    t_stop = 2.0
    for floor in ("hard_floor", "carpet"):
        for mode in ("active", "brake", "coast"):
            rec = {"dist": 0.6, "az": 70, "el": -12} if (video and floor == "hard_floor" and mode == "active") else None
            s = Sim(P, motor, floor=floor, payload=True, record=rec)
            s.set_twist(0.3, 0.0)

            def hook(s, mode=mode):
                if s.d.time >= t_stop and s.mode == "pi" and s.ref["fl"] != 0.0 or (s.d.time >= t_stop and mode != "active"):
                    if mode == "active":
                        s.set_twist(0.0, 0.0)
                    else:
                        s.mode = mode
            L = run(s, 3.5 if mode != "coast" else 5.0, every=2, hook=hook)
            m = L["t"] >= t_stop
            v0 = float(np.interp(t_stop, L["t"], L["vx"]))
            x0 = float(np.interp(t_stop, L["t"], L["x"]))
            stopped = np.where(m & (np.abs(L["vx"]) < 0.005))[0]
            ts = float(L["t"][stopped[0]] - t_stop) if len(stopped) else None
            dist = float(L["x"][stopped[0]] - x0) if len(stopped) else float(L["x"][-1] - x0)
            nr = np.minimum(L["N_rl"], L["N_rr"])[m]
            nf = np.minimum(L["N_fl"], L["N_fr"])[m]
            # lift = a wheel carries < 0.05 N for >= 20 ms continuously (single-step contact flicker ignored)
            win = 10   # samples at every=2 ms
            def lifted(a):
                low = (a < 0.05).astype(int)
                return bool(len(low) >= win and np.convolve(low, np.ones(win, int), "valid").max() >= win)
            nr_f = np.array([nr[i:i + win].max() for i in range(0, max(1, len(nr) - win))])
            key = f"{floor}_{mode}"
            out[key] = {"v0_mps": round(v0, 3), "stop_time_s": round(ts, 3) if ts is not None else None, "stop_distance_m": round(dist, 4),
                        "max_pitch_deg": round(math.degrees(float(np.abs(L["pitch"][m]).max())), 3),
                        "min_rear_wheel_normal_N_20ms": round(float(nr_f.min()), 3), "min_front_wheel_normal_N": round(float(nf.min()), 3),
                        "static_rear_wheel_normal_N": round(float(np.mean(np.minimum(L["N_rl"], L["N_rr"])[(L["t"] > 1.0) & (L["t"] < t_stop)])), 3),
                        "peak_decel_mps2_20ms": round(accel20(L["t"][m], L["vx"][m], -1), 2),
                        "wheel_lift_20ms": lifted(nr) or lifted(nf),
                        "pass_300ms": bool(ts is not None and ts <= 0.3)}
            if rec:
                out[key]["video"] = s.save_video("emergency_stop_payload")
            ax[0].plot(L["t"] - t_stop, L["vx"], label=key)
            ax[1].plot(L["t"] - t_stop, np.degrees(L["pitch"]), label=key)
            ax[2].plot(L["t"] - t_stop, np.minimum(L["N_rl"], L["N_rr"]), label=key)
    mt = P.total_mass + P.payload["mass"]
    cx = (P.total_mass * P.raw["com"][0] + P.payload["mass"] * P.payload["com"][0]) / mt
    cz = (P.total_mass * P.raw["com"][2] + P.payload["mass"] * P.payload["com"][2]) / mt
    xr = P.wheels["rl"]["pos"][0]
    xf = P.wheels["fl"]["pos"][0]
    out["_analytic_with_payload"] = {"com_x_m": round(cx, 4), "com_z_m": round(cz, 4),
                                     "front_wheel_lift_forward_accel_mps2": round(9.81 * (cx - xr) / cz, 2),
                                     "rear_wheel_lift_forward_decel_mps2": round(9.81 * (xf - cx) / cz, 2),
                                     "note": "rigid-body quasi-static: front axle unloads when forward accel (or braking while reversing) > g*(cx-x_rear)/h; rear axle unloads when forward decel > g*(x_front-cx)/h. Friction caps |a| at mu*g (7.8 m/s2 hard floor) and motor stall caps 4*Ts/r/m."}
    ax[0].set(xlim=(-0.3, 1.5), xlabel="t - t_stop [s]", ylabel="vx [m/s]", title="(c) e-stop from 0.3 m/s, 300 g payload")
    ax[1].set(xlim=(-0.3, 1.5), xlabel="t - t_stop [s]", ylabel="pitch [deg] (+ = nose down)")
    ax[2].set(xlim=(-0.3, 1.5), xlabel="t - t_stop [s]", ylabel="min rear wheel normal [N]")
    for a in ax:
        a.legend(fontsize=6); a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(PLOTS / "c_estop.png", dpi=110); plt.close(fig)
    return out


def scen_d(P, motor, plt, video):
    out = {}
    thr_x = 0.30
    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    for v in (0.05, 0.15, 0.3):
        for payload in (False, True):
            rec = {"dist": 0.55, "az": 70, "el": -10} if (video and v == 0.3 and payload) else None
            s = Sim(P, motor, floor="hard_floor", payload=payload, threshold_x=thr_x, record=rec)
            s.set_twist(v, 0.0)
            hits = []

            def hook(s):
                if s.n % 10 == 0:
                    hits.append(s.chassis_hits_threshold())
            secs = 0.9 / v + 1.0
            L = run(s, secs, hook=hook)
            rear_x = L["x"] + P.wheels["rl"]["pos"][0]
            ok = bool(rear_x[-1] > thr_x + 0.06)
            front = np.maximum(np.abs(L["tau_fl"]), np.abs(L["tau_fr"]))
            allt = np.max(np.abs(np.stack([L[f"tau_{k}"] for k in WHEELS])), axis=0)
            climb = (L["x"] > thr_x - 0.15) & (L["x"] < thr_x + 0.15)
            sat = float(np.mean(allt[climb] >= 0.98 * s.Ts)) if climb.any() else None
            vmin = float(L["vx"][climb].min()) if climb.any() else None
            key = f"v{v}{'_payload' if payload else ''}"
            out[key] = {"speed_mps": v, "payload": payload, "success": ok,
                        "peak_wheel_torque_Nm": round(float(allt.max()), 4), "peak_front_wheel_torque_Nm": round(float(front.max()), 4),
                        "peak_torque_pct_stall": round(100 * float(allt.max()) / s.Ts, 1),
                        "fraction_of_climb_saturated": round(sat, 3) if sat is not None else None,
                        "min_speed_during_climb_mps": round(vmin, 3) if vmin is not None else None,
                        "max_pitch_deg": round(math.degrees(float(np.abs(L["pitch"]).max())), 2),
                        "chassis_touched_threshold": bool(any(hits)), "pass": ok}
            if rec:
                out[key]["video"] = s.save_video("threshold_climb_0p3_payload")
            ax[0].plot(L["t"], L["z"] - L["z"][0], label=key)
            ax[1].plot(L["t"], allt, label=key)
            ax[2].plot(L["t"], L["vx"], label=key)
    # slow crawl with payload stalls at 6 V: does a higher motor-rail voltage fix it? (TT rated 3-6 V; >6 V shortens life)
    out["slow_payload_voltage_sweep"] = []
    for V in (6.0, 7.4, 8.4):
        s = Sim(P, motor, floor="hard_floor", payload=True, threshold_x=thr_x, supply_voltage=V)
        s.set_twist(0.05, 0.0)
        L = run(s, 0.9 / 0.05 + 1.0)
        ok = bool(L["x"][-1] + P.wheels["rl"]["pos"][0] > thr_x + 0.06)
        allt = np.max(np.abs(np.stack([L[f"tau_{k}"] for k in WHEELS])), axis=0)
        out["slow_payload_voltage_sweep"].append({"supply_V": V, "stall_torque_Nm": round(s.Ts, 4), "success": ok,
                                                  "peak_wheel_torque_Nm": round(float(allt.max()), 4)})
    ax[0].set(xlabel="t [s]", ylabel="base_link z rise [m]", title=f"(d) 10 mm threshold at x={thr_x} m (hard floor)")
    ax[1].axhline(s.Ts, c="r", ls=":", label="stall"); ax[1].set(xlabel="t [s]", ylabel="max |wheel torque| [N m]")
    ax[2].set(xlabel="t [s]", ylabel="vx [m/s]")
    for a in ax:
        a.legend(fontsize=7); a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(PLOTS / "d_threshold.png", dpi=110); plt.close(fig)
    return out


def scen_e(P, motor, plt, eff_ratio):
    """2 m x 2 m square on carpet, driven by the robot's own estimate (as the real robot would)."""
    rng = np.random.default_rng(1)
    variants = {"wheel_odom_nominal_track": dict(track=P.track, gyro=False, floor="carpet"),
                "wheel_odom_calibrated_track": dict(track=P.track * eff_ratio, gyro=False, floor="carpet"),
                "wheel_odom_carpet_calibration_on_hard_floor": dict(track=P.track * eff_ratio, gyro=False, floor="hard_floor"),
                "wheel_distance_plus_gyro_yaw": dict(track=P.track, gyro=True, floor="carpet")}
    gyro_noise = math.radians(0.005) / math.sqrt(1 / 100) * 1.0   # MPU-6050 0.005 deg/s/sqrt(Hz) at 100 Hz sampling
    gyro_bias = math.radians(0.05)                                  # assumed residual bias after still-calibration
    out = {}
    fig, ax = plt.subplots(1, 1, figsize=(6, 6))
    for name, cfg in variants.items():
        s = Sim(P, motor, floor=cfg["floor"])
        est = {"x": 0.0, "y": 0.0, "th": 0.0, "q": {k: s.d.qpos[s.jq[k]] for k in WHEELS}}
        plan = []
        for i in range(4):
            plan += [("fwd", 2.0), ("turn", math.pi / 2)]
        st = {"seg": 0, "x0": 0.0, "y0": 0.0, "th_target": 0.0, "done": False, "dist": 0.0}
        path_est = []

        def hook(s):
            if s.n % s.dec:
                return
            dt = s.dec * s.m.opt.timestep
            q = {k: s.d.qpos[s.jq[k]] for k in WHEELS}
            dl = s.r * np.mean([q[k] - est["q"][k] for k in LEFT])
            dr = s.r * np.mean([q[k] - est["q"][k] for k in RIGHT])
            est["q"] = q
            ds = (dl + dr) / 2
            if cfg["gyro"]:
                dth = (s.yaw_rate() + gyro_bias + gyro_noise * rng.standard_normal()) * dt
            else:
                dth = (dr - dl) / cfg["track"]
            est["x"] += ds * math.cos(est["th"] + dth / 2)
            est["y"] += ds * math.sin(est["th"] + dth / 2)
            est["th"] += dth
            st["dist"] += ds
            path_est.append((est["x"], est["y"]))
            if st["done"]:
                s.set_twist(0, 0)
                return
            kind, val = plan[st["seg"]]
            if kind == "fwd":
                err_h = math.atan2(math.sin(st["th_target"] - est["th"]), math.cos(st["th_target"] - est["th"]))
                rem = val - st["dist"]
                v = min(0.3, max(0.05, 1.0 * rem))
                s.set_twist(v, 2.0 * err_h)
                if rem <= 0.002:
                    st["seg"] += 1; st["th_target"] += math.pi / 2
            else:
                err = st["th_target"] - est["th"]
                w = max(-1.0, min(1.0, 2.0 * err))
                if abs(w) < 0.25:
                    w = math.copysign(0.25, err)
                s.set_twist(0, w)
                if abs(err) < math.radians(0.5):
                    st["seg"] += 1; st["dist"] = 0.0
            if st["seg"] >= len(plan):
                st["done"] = True
        L = run(s, 80.0, every=20, hook=hook, stop=lambda s: st["done"])
        x, y, _, _, _, yaw = s.pose()
        travelled = float(np.sum(np.hypot(np.diff(L["x"]), np.diff(L["y"]))))
        herr = math.degrees(math.atan2(math.sin(est["th"] - yaw), math.cos(est["th"] - yaw)))
        perr = math.hypot(est["x"] - x, est["y"] - y)
        out[name] = {"completed": st["done"], "true_end": [round(x, 3), round(y, 3), round(math.degrees(yaw), 2)],
                     "estimated_end": [round(est["x"], 3), round(est["y"], 3), round(math.degrees(est["th"]) % 360, 2)],
                     "path_length_m": round(travelled, 3), "heading_error_deg": round(herr % 360 if herr < -180 else herr, 2),
                     "position_error_m": round(perr, 3),
                     "heading_drift_deg_per_10m": round(abs(herr) * 10 / max(travelled, 1e-6), 2),
                     "closure_error_true_m": round(math.hypot(x, y), 3),
                     "pass_req006_5deg_per_10m": bool(abs(herr) * 10 / max(travelled, 1e-6) < 5)}
        ax.plot(L["x"], L["y"], label=f"truth: {name}")
        pe = np.array(path_est)
        ax.plot(pe[:, 0], pe[:, 1], ls="--", lw=0.8, label=f"estimate: {name}")
    ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(fontsize=6); ax.set(xlabel="x [m]", ylabel="y [m]", title="(e) 2 m square, robot drives on its own estimate")
    fig.tight_layout(); fig.savefig(PLOTS / "e_odometry.png", dpi=110); plt.close(fig)
    out["_noise_model"] = {"gyro_noise_density": "0.005 deg/s/sqrt(Hz) (MPU-6050 datasheet PS-MPU-6000A rate noise spectral density)",
                           "gyro_bias_deg_s": "0.05 residual after still calibration (assumed)", "calibrated_track_ratio": eff_ratio}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("params", nargs="?", default=None)
    ap.add_argument("--motor", default="tt_1_48_6v")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--only", default="a,b,c,d,e")
    ap.add_argument("--out", default=str(HERE / "results.json"))
    a = ap.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    global PLOTS
    if a.motor != "tt_1_48_6v":
        PLOTS = PLOTS / a.motor
    PLOTS.mkdir(parents=True, exist_ok=True)
    P = Params(a.params or gen_mjcf.default_params())
    video = not a.no_video
    outp = Path(a.out)
    res = json.loads(outp.read_text()) if outp.exists() else {}
    res["_meta"] = {"params": str(P.path), "params_version": P.version, "motor": a.motor, "motor_profile": gen_mjcf.MOTORS[a.motor],
                    "total_mass_kg": P.total_mass, "track_m": P.track, "wheelbase_m": P.wheelbase, "wheel_radius_m": P.wheel_radius,
                    "floors": {k: v for k, v in gen_mjcf.FLOORS.items() if not k.startswith("_")},
                    "mujoco": mujoco.__version__, "generated": time.strftime("%Y-%m-%d %H:%M")}
    todo = a.only.split(",")
    for key, fn in (("a_straight", lambda: scen_a(P, a.motor, plt)), ("b_spin", lambda: scen_b(P, a.motor, plt, video)),
                    ("c_estop_payload", lambda: scen_c(P, a.motor, plt, video)), ("d_threshold", lambda: scen_d(P, a.motor, plt, video))):
        if key[0] in todo:
            t0 = time.time()
            res[key] = fn()
            print(f"{key} done in {time.time() - t0:.1f}s", flush=True)
            outp.write_text(json.dumps(res, indent=1, default=float))
    if "e" in todo:
        ratio = res.get("b_spin", {}).get("open_loop_nominal_kinematics", {}).get("carpet", {}).get("effective_track_ratio", 1.0)
        t0 = time.time()
        res["e_odometry_square"] = scen_e(P, a.motor, plt, ratio)
        print(f"e done in {time.time() - t0:.1f}s", flush=True)
    outp.write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({k: v for k, v in res.items() if k != "_meta"}, indent=1, default=float))


if __name__ == "__main__":
    main()
