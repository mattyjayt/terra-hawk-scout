import sys; sys.path.insert(0, "src")
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from cadgen import read_mesh
from plate_outline import ELEGOO_DIR, face_polygon, rings
fig, axs = plt.subplots(1, 3, figsize=(15, 9))
for ax, (n, lvl) in zip(axs, [("Top Plate.stl", -9.2), ("Bottom Plate.stl", -57.91), ("Dead Plate.stl", -6.42)]):
    m = read_mesh(ELEGOO_DIR / n)
    import numpy as np
    v = np.asarray(m.vertices); lvl = v[:,2].max() if n!="Bottom Plate.stl" else v[:,2].min()
    p = face_polygon(m.vertices, m.triangles, lvl)
    o, h = rings(p)
    print(n, "level", lvl, "area", round(p.area), "outer pts", len(o), "holes", len(h), "bounds", [round(b,2) for b in p.bounds])
    ax.fill(*zip(*o), color="#cde"); 
    for r in h: ax.fill(*zip(*r), color="white", edgecolor="k", lw=0.3)
    ax.set_aspect("equal"); ax.set_title(n); ax.grid(True)
plt.savefig("tmp/outlines.png", dpi=80)
