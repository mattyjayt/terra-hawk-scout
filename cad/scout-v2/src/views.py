"""Assembly-guide, exploded and animated models of the default Scout v2 (round 3).

scout_v2_step_NN  : every part up to step NN; earlier steps in light grey, the parts added in this step in
                    their own colours (the 'highlight'). Step 05 (bench pre-fit) is shown lifted above the
                    chassis; from step 06 the top plate is drawn translucent so the wiring between the decks
                    shows. Snapshots -> assembly/stepNN.png.
scout_v2_exploded : the default robot with each build step lifted / pushed out -> shots/scout_v2_exploded.png.
scout_v2_assembly : the default robot with a glTF animation 'assembly': the parts of each build step fly in,
                    in assembly order (export -> out/GLB/scout_v2_assembly.glb; plays in the CAD Viewer and any
                    glTF viewer).
"""
import build123d as bd
from cadgen import model
from cadgen.gltf import animation

from scout_v2 import BUILD_STEPS, DEFAULT, scout_parts

GREY = "#d3d6db"
SPEC = (DEFAULT["lidar"], DEFAULT["motor"], DEFAULT["payload"], DEFAULT["driver"])
PREFIT = 4                     # index of step 05 (bench pre-fit of the top plate)
PREFIT_LIFT = 90.0             # mm: how high the pre-fitted top plate is drawn above the pillars in step 05


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


def recolor(part, color):
    part.color = color
    for leaf in getattr(part, "leaves", []) or []:
        leaf.color = color
    return part


def step_model(n):
    kids, _ = scout_parts(*SPEC)
    st = step_of([k.label for k in kids])
    keep = []
    for k in kids:
        s = st[k.label]
        if s > n:
            continue
        if s < n:
            recolor(k, bd.Color(0.83, 0.84, 0.86, 0.30) if (k.label == "top_plate" and n >= PREFIT + 1) else GREY)
        if n == PREFIT and s == PREFIT:
            lab = k.label
            k = bd.Pos(0, 0, PREFIT_LIFT) * k
            k.label = lab
        keep.append(k)
    return bd.Compound(children=keep, label=f"step_{BUILD_STEPS[n][0]}")


# explode / fly-in offsets per step (mm)
EXPLODE = {0: (0, 0, 0), 1: (0, 0, 45), 2: (-40, 0, 60), 3: (0, 0, 95), 4: (0, 0, 140), 5: (0, 0, 140),
           6: (0, 0, 200), 7: (0, 0, 280), 8: (-90, 0, 230)}
FLY = {0: (0, 0, 120), 1: (0, 0, 120), 2: (-150, 0, 60), 3: (0, 0, 120), 4: (0, 0, 160), 5: (0, 0, 80),
       6: (0, 0, 150), 7: (0, 0, 180), 8: (-150, 0, 120)}


@model
def scout_v2_exploded():
    kids, _ = scout_parts(*SPEC)
    st = step_of([k.label for k in kids])
    out = []
    for k in kids:
        dx, dy, dz = EXPLODE[st[k.label]]
        if k.label.startswith("wheel_"):
            dy = 70 if k.label.endswith("l") else -70
        lab = k.label
        out.append(bd.Pos(dx, dy, dz) * k if (dx, dy, dz) != (0, 0, 0) else k)
        out[-1].label = lab
    return bd.Compound(children=out, label="scout_v2_exploded")


STEP_T = 1.6      # s per build step: 1.1 s fly-in + 0.5 s pause
FLY_T = 1.1


@model
def scout_v2_assembly():
    """Default robot, as built, carrying the 'assembly' glTF animation (parts appear and fly in step by step)."""
    kids, _ = scout_parts(*SPEC)
    st = step_of([k.label for k in kids])
    asm = bd.Compound(children=kids, label="scout_v2_assembly")
    n = len(BUILD_STEPS)
    dur = n * STEP_T + 2.0
    anim = animation(asm, "assembly", duration=dur)
    for i in range(n):
        labels = [k.label for k in kids if st[k.label] == i]
        t0 = i * STEP_T
        t1 = t0 + FLY_T
        for lab in labels:
            off = FLY[i]
            if lab.startswith("wheel_"):
                off = (0, 110 if lab.endswith("l") else -110, 0)
            if t0 > 0:
                anim.channel(lab, path="scale", keys=[(0.0, 0.0), (t0, 1.0)], interpolation="STEP")
            anim.channel(lab, path="translation", keys=[(0.0, off), (t0, off), (t1, (0, 0, 0))] if t0 > 0
                         else [(0.0, off), (t1, (0, 0, 0))], interpolation="CUBICSPLINE")
    return asm


def _step(n):
    return step_model(n)


@model
def scout_v2_step_01():
    return _step(0)


@model
def scout_v2_step_02():
    return _step(1)


@model
def scout_v2_step_03():
    return _step(2)


@model
def scout_v2_step_04():
    return _step(3)


@model
def scout_v2_step_05():
    return _step(4)


@model
def scout_v2_step_06():
    return _step(5)


@model
def scout_v2_step_07():
    return _step(6)


@model
def scout_v2_step_08():
    return _step(7)


@model
def scout_v2_step_09():
    return _step(8)


STEP_MODELS = [scout_v2_step_01, scout_v2_step_02, scout_v2_step_03, scout_v2_step_04, scout_v2_step_05,
               scout_v2_step_06, scout_v2_step_07, scout_v2_step_08, scout_v2_step_09]

if __name__ == "__main__":
    scout_v2_exploded()
    scout_v2_assembly()
    for f in STEP_MODELS:
        f()
