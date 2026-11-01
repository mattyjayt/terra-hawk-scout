"""Shared loader for robot_params.json (BRIEF section 4 schema) used by the URDF and MJCF generators.

Tolerant of the CAD engineer's file: missing inertia -> solid box from bbox, missing com ->
bbox centre, wheels may or may not also appear in `links`. Everything rigidly attached to the
chassis (every link that is not a wheel and not the payload) is lumped into one chassis body.
"""
import json
from pathlib import Path

import numpy as np

WHEELS = ("fl", "fr", "rl", "rr")
EXCLUDE_PREFIX = ("wheel_", "payload")


def _inertia_matrix(v):
    ixx, iyy, izz, ixy, ixz, iyz = v
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], float)


def _link_inertia(l):
    if l.get("inertia") is not None:
        return _inertia_matrix(l["inertia"])
    s = np.array(l["bbox_max"]) - np.array(l["bbox_min"])
    m = l["mass"]
    return np.diag([m / 12 * (s[1]**2 + s[2]**2), m / 12 * (s[0]**2 + s[2]**2), m / 12 * (s[0]**2 + s[1]**2)])


def _link_com(l):
    if l.get("com") is not None:
        return np.array(l["com"], float)
    return (np.array(l["bbox_min"]) + np.array(l["bbox_max"])) / 2


def lump(parts):
    """parts: list of (mass, com[3], I3x3 about own com). Returns mass, com, I about com (parallel axis)."""
    m = sum(p[0] for p in parts)
    c = sum(p[0] * p[1] for p in parts) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in parts:
        d = ci - c
        I += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, c, I


class Params:
    def __init__(self, path):
        self.path = Path(path)
        self.raw = json.loads(self.path.read_text())
        p = self.raw
        self.version = p.get("version", "?")
        self.wheels = {}
        for k in WHEELS:
            w = dict(p["wheels"][k])
            link = p.get("links", {}).get(f"wheel_{k}", {})
            w.setdefault("mass", link.get("mass", 0.025))
            r, wd, m = w["radius"], w["width"], w["mass"]
            w.setdefault("inertia", link.get("inertia") or [m / 12 * (3 * r * r + wd * wd), m / 2 * r * r, m / 12 * (3 * r * r + wd * wd), 0, 0, 0])
            self.wheels[k] = w
        self.wheel_radius = float(np.mean([w["radius"] for w in self.wheels.values()]))
        self.wheel_width = float(np.mean([w["width"] for w in self.wheels.values()]))
        self.axle_z = float(np.mean([w["pos"][2] for w in self.wheels.values()]))
        self.track = p.get("track_width") or float(self.wheels["fl"]["pos"][1] - self.wheels["fr"]["pos"][1])
        self.wheelbase = p.get("wheelbase") or float(self.wheels["fl"]["pos"][0] - self.wheels["rl"]["pos"][0])
        self.chassis_links = {k: v for k, v in p.get("links", {}).items()
                              if not k.startswith(EXCLUDE_PREFIX) and v.get("mass", 0) > 0}
        parts = [(v["mass"], _link_com(v), _link_inertia(v)) for v in self.chassis_links.values()]
        self.chassis_mass, self.chassis_com, self.chassis_I = lump(parts)
        self.sensors = p.get("sensors", {})
        pl = p.get("payload") or p.get("links", {}).get("payload")
        if pl is None:   # default: v1 payload bay
            pl = {"mass": 0.3, "com": [-0.083, 0.0, 0.0957], "size": [0.06, 0.08, 0.06]}
        if "size" not in pl and "bbox_min" in pl:
            pl["size"] = list(np.array(pl["bbox_max"]) - np.array(pl["bbox_min"]))
        if "com" not in pl and "bbox_min" in pl:
            pl["com"] = list(_link_com(pl))
        self.payload = pl
        self.total_mass = self.chassis_mass + sum(w["mass"] for w in self.wheels.values())

    def boxes(self):
        """(name, centre, size) for every chassis link with a bbox — used for visuals/collision."""
        out = []
        for k, v in self.chassis_links.items():
            if "bbox_min" in v:
                lo, hi = np.array(v["bbox_min"]), np.array(v["bbox_max"])
                out.append((k, (lo + hi) / 2, hi - lo))
        return out

    def chassis_inertia6(self):
        I = self.chassis_I
        return [I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]
