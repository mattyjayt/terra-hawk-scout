import sys
from cadgen import read_mesh
import numpy as np
D = "/Users/jaimetellie/Downloads/ELEGOO Smart Robot Car Kit V4.0 2023.02.01/02 Manual & Main Code & APP/05 3D Model/"
for n in ["Top Plate.stl", "Bottom Plate.stl", "Dead Plate.stl"]:
    m = read_mesh(D + n)
    v = np.asarray(m.vertices)
    bb = m.bounding_box()
    print(n, "nv", len(v), "min", v.min(0).round(2), "max", v.max(0).round(2), "size", (v.max(0)-v.min(0)).round(2))
    # histogram of z
    z = v[:, np.argmin(v.max(0)-v.min(0))]
    vals, cnt = np.unique(z.round(1), return_counts=True)
    print("  thin-axis levels:", list(zip(vals[:20], cnt[:20])))
