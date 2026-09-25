"""Low-poly military-style aircraft model, body FRD (+X Forward, +Y Right, +Z Down).

A generic swept-wing jet - fuselage, canopy, intakes, wings, tailplanes and a
single fin - built from lofted cross-sections and thin plates. It is not a
model of any particular real aircraft. Every vertex carries a ``part`` id:

    PART_BODY       muted gunmetal finish
    PART_DARK       canopy, intakes, nozzle
    PART_INDICATOR  fin-tip band and wingtip bands: the ONLY parts that take the
                    classification colour (or the ownship colour), so the body is
                    never painted green / white / red / yellow

Because the mesh is in the body frame, one 4x4 transform poses it:
x_world = P + scale * R_WB @ x_body. Two levels of detail: "high" for the
ownship (~600 triangles) and "low" for instanced targets (~250 triangles).

If ``assets/aircraft/aircraft.stl`` (or .obj / .ply) exists it replaces the
ownship model; it must be modelled in metres, nose +X, right wing +Y, fin -Z.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyvista as pv

from visualization import style

ASSET_DIR = Path(__file__).resolve().parent.parent / "assets" / "aircraft"
PART_BODY, PART_DARK, PART_INDICATOR = 0, 1, 2
LENGTH_M = 15.0


class _Builder:
    def __init__(self):
        self.pts: list[np.ndarray] = []
        self.tris: list[np.ndarray] = []
        self.parts: list[np.ndarray] = []
        self.n = 0

    def add(self, pts, tris, part):
        pts = np.asarray(pts, float)
        self.pts.append(pts)
        self.tris.append(np.asarray(tris, np.int64) + self.n)
        self.parts.append(np.full(len(pts), part, np.int64))
        self.n += len(pts)

    def plate(self, outline, axis_normal, thickness, part, offset=0.0):
        """Extrude a closed convex outline (in the plane normal to axis_normal) by thickness."""
        outline = np.asarray(outline, float)
        m = len(outline)
        a, b = [i for i in range(3) if i != axis_normal]
        top = np.zeros((m, 3))
        top[:, a], top[:, b] = outline[:, 0], outline[:, 1]
        bot = top.copy()
        top[:, axis_normal] = offset - thickness / 2
        bot[:, axis_normal] = offset + thickness / 2
        pts = np.vstack([top, bot])
        tris = [(0, i, i + 1) for i in range(1, m - 1)]
        tris += [(m, m + i + 1, m + i) for i in range(1, m - 1)]
        for i in range(m):
            j = (i + 1) % m
            tris += [(i, j, m + j), (i, m + j, m + i)]
        self.add(pts, tris, part)

    def loft(self, sections, n, part, cap_tail_part):
        """Fuselage from (x, half_width, half_height, z_centre) sections; first is the nose tip."""
        th = np.linspace(0, 2 * np.pi, n, endpoint=False)
        rings = []
        for x, w, h, zc in sections[1:]:
            rings.append(np.column_stack([np.full(n, x), w * np.cos(th), zc + h * np.sin(th)]))
        nose = np.array([[sections[0][0], 0.0, sections[0][3]]])
        pts = np.vstack([nose] + rings)
        tris = [(0, 1 + (i + 1) % n, 1 + i) for i in range(n)]
        for r in range(len(rings) - 1):
            a0, b0 = 1 + r * n, 1 + (r + 1) * n
            for i in range(n):
                j = (i + 1) % n
                tris += [(a0 + i, a0 + j, b0 + j), (a0 + i, b0 + j, b0 + i)]
        self.add(pts, tris, part)
        # tail cap (nozzle) in its own part
        last = rings[-1]
        cap = np.vstack([last, [[last[0, 0] - 0.05, 0.0, sections[-1][3]]]])
        self.add(cap, [(n, (i + 1) % n, i) for i in range(n)], cap_tail_part)

    def ellipsoid(self, centre, radii, part, res=8):
        s = pv.Sphere(radius=1.0, theta_resolution=res, phi_resolution=max(4, res // 2 + 1)).triangulate()
        pts = np.asarray(s.points) * np.asarray(radii) + np.asarray(centre)
        self.add(pts, s.faces.reshape(-1, 4)[:, 1:], part)

    def mesh(self) -> pv.PolyData:
        pts = np.vstack(self.pts)
        tris = np.vstack(self.tris)
        mesh = pv.PolyData(pts, faces=np.hstack([np.full((len(tris), 1), 3), tris]).ravel())
        mesh.point_data["part"] = np.concatenate(self.parts)
        return mesh


def _wing_outline(y0, y1, root_le, tip_le, root_te, tip_te, y_root, y_tip):
    """Planform strip between spans y0..y1 of a straight-tapered wing (x forward, y right)."""
    def le(y):
        return root_le + (tip_le - root_le) * (y - y_root) / (y_tip - y_root)

    def te(y):
        return root_te + (tip_te - root_te) * (y - y_root) / (y_tip - y_root)
    return [(le(y0), y0), (le(y1), y1), (te(y1), y1), (te(y0), y0)]


def military_jet(detail: str = "high") -> pv.PolyData:
    """Generic military jet in body FRD, metres, ~15 m long, origin near the centre of gravity."""
    n = 10 if detail == "high" else 6
    b = _Builder()
    # fuselage: (x, half-width, half-height, z-centre); z negative = up
    b.loft([(7.6, 0, 0, 0.05), (6.3, 0.30, 0.30, 0.05), (4.2, 0.62, 0.58, 0.0), (1.6, 0.92, 0.66, 0.0),
            (-3.4, 0.92, 0.62, 0.0), (-5.8, 0.72, 0.52, 0.0), (-7.0, 0.55, 0.46, 0.0)],
           n, PART_BODY, PART_DARK)
    b.ellipsoid((3.4, 0.0, -0.62), (1.7, 0.44, 0.42), PART_DARK, res=10 if detail == "high" else 6)
    for side in (1, -1):
        # intakes
        b.plate([(3.0, 0.2), (0.6, 0.2), (0.6, -0.45), (3.0, -0.45)], axis_normal=1, thickness=0.36,
                part=PART_DARK, offset=side * 1.02)
        # main wing: body section, then the classification band at the tip
        wing = dict(root_le=2.0, tip_le=-2.6, root_te=-3.9, tip_te=-3.7, y_root=0.8, y_tip=5.4)
        for (y0, y1), part in (((0.8, 4.75), PART_BODY), ((4.75, 5.4), PART_INDICATOR)):
            outline = _wing_outline(y0, y1, **wing)
            if side < 0:
                outline = [(x, -y) for x, y in reversed(outline)]
            b.plate(outline, axis_normal=2, thickness=0.16, part=part, offset=0.18)
        # tailplane
        tail = _wing_outline(0.6, 2.8, -4.7, -6.6, -7.0, -7.2, 0.6, 2.8)
        if side < 0:
            tail = [(x, -y) for x, y in reversed(tail)]
        b.plate(tail, axis_normal=2, thickness=0.10, part=PART_BODY, offset=0.05)
    # vertical fin in the x-z plane (up = -z): body, then the classification band at the top
    def fin(z0, z1):
        def le(z):
            return -3.7 + (-6.3 + 3.7) * (z - (-0.55)) / (-3.6 + 0.55)

        def te(z):
            return -7.0 + (-7.15 + 7.0) * (z - (-0.55)) / (-3.6 + 0.55)
        return [(le(z0), z0), (te(z0), z0), (te(z1), z1), (le(z1), z1)]
    b.plate(fin(-0.55, -2.95), axis_normal=1, thickness=0.14, part=PART_BODY)
    b.plate(fin(-2.95, -3.6), axis_normal=1, thickness=0.14, part=PART_INDICATOR)
    return b.mesh()


def paint(mesh: pv.PolyData, indicator_hex: str) -> pv.PolyData:
    """Per-vertex colours: military body / dark parts, indicator bands in the given colour."""
    palette = np.array([style.hex_to_rgb(style.AIRCRAFT_BODY), style.hex_to_rgb(style.AIRCRAFT_DARK),
                        style.hex_to_rgb(indicator_hex)], np.uint8)
    out = mesh.copy()
    out.point_data["rgb"] = palette[np.asarray(mesh.point_data["part"])]
    return out


def load_aircraft() -> tuple[pv.PolyData, str]:
    """Ownship model in body FRD (with rgb) and a description of where it came from."""
    for ext in ("stl", "obj", "ply"):
        f = ASSET_DIR / f"aircraft.{ext}"
        if f.exists():
            mesh = pv.read(str(f)).triangulate()
            mesh.point_data["rgb"] = np.tile(np.array(style.hex_to_rgb(style.AIRCRAFT_BODY), np.uint8),
                                             (mesh.n_points, 1))
            return mesh, f"asset {f.name}"
    return paint(military_jet("high"), style.OWNSHIP_COLOR), "procedural military jet"
