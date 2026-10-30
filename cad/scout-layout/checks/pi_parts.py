from cadgen import read_xbf
import build123d as bd
t = read_xbf("vendor/raspberry_pi_5.xbf")
def walk(n, loc=None, d=0):
    here = (bd.Location() if loc is None else loc) * n.location
    bb = n.located(here).bounding_box()
    print("  "*d, n.label, [round(v,1) for v in (bb.min.X,bb.min.Y,bb.min.Z,bb.max.X,bb.max.Y,bb.max.Z)])
    for c in n.children: walk(c, here, d+1)
walk(t)
