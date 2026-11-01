"""Assembly-guide and exploded models of the default Scout v2 (round 2).

scout_v2_step_NN  : every part up to step NN; earlier steps in light grey, the parts added in this step in
                    their own colours (the 'highlight'). Snapshots -> assembly/step_NN.png.
scout_v2_exploded : the default robot with each build step lifted / pushed out -> shots/scout_v2_exploded.png.
"""
import build123d as bd
from cadgen import model

from scout_v2 import BUILD_STEPS, DEFAULT, scout_parts

GREY = "#d3d6db"
SPEC = (DEFAULT["lidar"], DEFAULT["motor"], DEFAULT["payload"], DEFAULT["driver"])


def step_of(labels):
    """label -> step index (first matching step)."""
    out = {}
    for i, (_, _, prefixes) in enumerate(BUILD_STEPS):
        for lab in labels:
            if lab in out:
                continue
            for p in prefixes:
                if lab == p or (p.endswith("_") and lab.startswith(p)):
                    out[lab] = i
                    break
    missing = [l for l in labels if l not in out]
    assert not missing, f"labels in no build step: {missing}"
    return out


def grey(part):
    part.color = GREY
    for leaf in getattr(part, "leaves", []) or []:
        leaf.color = GREY
    return part


def step_model(n):
    kids, _ = scout_parts(*SPEC)
    st = step_of([k.label for k in kids])
    keep = []
    for k in kids:
        if st[k.label] < n:
            keep.append(grey(k))
        elif st[k.label] == n:
            keep.append(k)
    return bd.Compound(children=keep, label=f"step_{BUILD_STEPS[n][0]}")


# explode offsets per step (mm): lift the upper stages, pull the wheels and side items outwards
EXPLODE = {0: (0, 0, 0), 1: (0, 0, 35), 2: (0, 0, 55), 3: (0, 0, 75), 4: (0, 0, 120), 5: (0, 0, 120),
           6: (0, 0, 165), 7: (110, 0, 165), 8: (95, 0, 105), 9: (0, 0, 235), 10: (0, 0, 290), 11: (-60, 0, 200)}


@model
def scout_v2_exploded():
    kids, _ = scout_parts(*SPEC)
    st = step_of([k.label for k in kids])
    out = []
    for k in kids:
        dx, dy, dz = EXPLODE[st[k.label]]
        if k.label.startswith("wheel_"):
            dy = 70 if k.label.endswith("l") else -70
        if st[k.label] == 4 and k.label.startswith(("mdd3a", "imu", "switch", "tb6612")):
            dz = 90                                # bench pre-fit hangs under the top plate
        out.append(bd.Pos(dx, dy, dz) * k if (dx, dy, dz) != (0, 0, 0) else k)
        out[-1].label = k.label
    return bd.Compound(children=out, label="scout_v2_exploded")


@model
def scout_v2_step_01():
    return step_model(0)


@model
def scout_v2_step_02():
    return step_model(1)


@model
def scout_v2_step_03():
    return step_model(2)


@model
def scout_v2_step_04():
    return step_model(3)


@model
def scout_v2_step_05():
    return step_model(4)


@model
def scout_v2_step_06():
    return step_model(5)


@model
def scout_v2_step_07():
    return step_model(6)


@model
def scout_v2_step_08():
    return step_model(7)


@model
def scout_v2_step_09():
    return step_model(8)


@model
def scout_v2_step_10():
    return step_model(9)


@model
def scout_v2_step_11():
    return step_model(10)


@model
def scout_v2_step_12():
    return step_model(11)


STEP_MODELS = [scout_v2_step_01, scout_v2_step_02, scout_v2_step_03, scout_v2_step_04, scout_v2_step_05,
               scout_v2_step_06, scout_v2_step_07, scout_v2_step_08, scout_v2_step_09, scout_v2_step_10,
               scout_v2_step_11, scout_v2_step_12]

if __name__ == "__main__":
    scout_v2_exploded()
    for f in STEP_MODELS:
        f()
