"""Rewrite every Scout v2 deliverable file from the models (round 3).

Run:  ~/.local/bin/cadgen +0.8.4 run src/exports.py
  out/GLB/scout_v2.glb            coloured default robot (web viewer), metres, glTF Y-up
  out/GLB/scout_v2_assembly.glb   same robot + glTF animation 'assembly' (parts fly in, build steps 01-09)
  out/GLB/scout_v2_exploded.glb   static exploded view (each build step pushed out)
  out/3MF/scout_v2.3mf            whole assembly, mm, colours
  out/STEP/scout_v2.step          AP242 assembly (git-ignored)
  out/STL/scout_v2.stl            whole assembly, mm
  out/STL/parts/<label>.stl       one STL per part, mm, in the robot frame (REP-103, as assembled)
"""
import sys
from pathlib import Path

from cadgen import export, model

from scout_v2 import DEFAULT, scout_parts, scout_v2
from views import scout_v2_assembly, scout_v2_exploded

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out"

if __name__ == "__main__":
    for d in ("GLB", "3MF", "STEP", "STL/parts"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    export(scout_v2(), OUT / "GLB/scout_v2.glb", OUT / "3MF/scout_v2.3mf", OUT / "STEP/scout_v2.step",
           OUT / "STL/scout_v2.stl")
    print("wrote whole-assembly GLB/3MF/STEP/STL", flush=True)
    export(scout_v2_assembly(), OUT / "GLB/scout_v2_assembly.glb")
    export(scout_v2_exploded(), OUT / "GLB/scout_v2_exploded.glb")
    print("wrote animated + exploded GLB", flush=True)
    if "--no-parts" not in sys.argv:
        kids, _ = scout_parts(DEFAULT["lidar"], DEFAULT["motor"], DEFAULT["payload"], DEFAULT["driver"])
        for old in (OUT / "STL/parts").glob("*.stl"):
            old.unlink()
        for k in kids:
            h = model(k, name=f"part_{k.label}")
            export(h, OUT / f"STL/parts/{k.label}.stl")
        print("wrote", len(kids), "part STLs", flush=True)
