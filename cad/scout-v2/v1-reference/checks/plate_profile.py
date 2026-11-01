import sys; sys.path.insert(0, "src")
import numpy as np
from shapely.geometry import LineString
from cadgen import read_mesh
from plate_outline import ELEGOO_DIR, face_polygon
cx, cy = (-56.49+76.03)/2, (64.4-166.38)/2
for n in ["Top Plate.stl", "Bottom Plate.stl"]:
    m = read_mesh(ELEGOO_DIR / n); v = np.asarray(m.vertices)
    lvl = v[:,2].max() if n.startswith("Top") else v[:,2].min()
    p = face_polygon(m.vertices, m.triangles, lvl)
    from shapely.geometry import Polygon
    shell = Polygon(p.exterior)
    print(n)
    prev=None
    for yr in np.arange(-115, 116, 1.0):
        y = cy - yr
        ln = LineString([(-200, y), (200, y)]).intersection(shell)
        if ln.is_empty: continue
        xs = np.array(ln.bounds)[[0,2]]
        xr = sorted(-(xs - cx))
        w = (round(xr[0],1), round(xr[1],1))
        if prev is None or abs(w[0]-prev[0])>0.8 or yr in (-115,0,115): print("  y_r", yr, "x_r", w)
        prev = w
    for r in p.interiors:
        c = np.array(r.coords); xr = -(c[:,0]-cx); yr = -(c[:,1]-cy)
        d = max(np.ptp(xr), np.ptp(yr))
        if d > 8: print("  big hole centre", round(xr.mean(),1), round(yr.mean(),1), "size", round(np.ptp(xr),1), round(np.ptp(yr),1))
