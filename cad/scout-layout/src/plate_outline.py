"""Planform extraction from the Elegoo STL meshes (pure numpy + shapely, no kernel).

A mesh is data, never placed in a model: we take the triangles lying in one flat
face of a plate (the face at z = `level`), union them in 2D, and return the
outer boundary and the hole rings. The model extrudes those rings into a B-rep
plate of the measured thickness.
"""
from pathlib import Path

import numpy as np

ELEGOO_DIR = Path(
    "/Users/jaimetellie/Downloads/ELEGOO Smart Robot Car Kit V4.0 2023.02.01/"
    "02 Manual & Main Code & APP/05 3D Model"
)


def face_polygon(vertices, triangles, level, tol=0.05):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    v = np.asarray(vertices, dtype=float)
    t = np.asarray(triangles, dtype=int)
    z = v[t][:, :, 2]
    on = np.all(np.abs(z - level) < tol, axis=1)
    polys = []
    for tri in t[on]:
        p = Polygon(v[tri][:, :2])
        if p.area > 1e-6:
            polys.append(p.buffer(1e-4))
    merged = unary_union(polys).buffer(-1e-4)
    if merged.geom_type == "MultiPolygon":
        merged = max(merged.geoms, key=lambda g: g.area)
    return merged.simplify(0.02)


def rings(poly, min_hole_area=1.0):
    outer = list(poly.exterior.coords)[:-1]
    holes = [list(r.coords)[:-1] for r in poly.interiors if abs(_area(r.coords)) > min_hole_area]
    return outer, holes


def _area(coords):
    c = np.asarray(coords)
    x, y = c[:, 0], c[:, 1]
    return 0.5 * np.sum(x[:-1] * y[1:] - x[1:] * y[:-1])
