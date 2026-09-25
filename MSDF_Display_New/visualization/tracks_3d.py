"""Mesh helpers for the 3D view: persistent actors whose data is replaced each frame.

(The 3D world view shows the ownship and the sensors from the Scenario Export;
RDP tracks are sensor-relative and are shown in the Aircraft PPI.)
"""

from __future__ import annotations

import numpy as np
import pyvista as pv


def polylines(paths: list[np.ndarray], rgbs: list) -> pv.PolyData:
    paths = [(p, c) for p, c in zip(paths, rgbs) if len(p) >= 2]
    if not paths:
        return pv.PolyData()
    pts = np.vstack([p for p, _ in paths])
    cells, off = [], 0
    for p, _ in paths:
        cells.append(np.concatenate([[len(p)], np.arange(off, off + len(p))]))
        off += len(p)
    mesh = pv.PolyData(pts, lines=np.concatenate(cells))
    mesh.point_data["rgb"] = np.vstack([np.tile(np.asarray(c, np.uint8), (len(p), 1)) for p, c in paths])
    return mesh


# track symbology, the same vocabulary as the 2D view and the PPI (visualization.style):
# unit outlines in the camera plane, apex / first corner "up" on screen.
GLYPH_SHAPES = {
    "square": np.array([(-0.7, -0.7), (0.7, -0.7), (0.7, 0.7), (-0.7, 0.7), (-0.7, -0.7)]),
    "triangle": np.array([(0.0, 1.0), (0.92, -0.72), (-0.92, -0.72), (0.0, 1.0)]),
}


def camera_glyphs(points, colours, shape: str, sizes, right, up) -> pv.PolyData:
    """Flat track symbols facing the camera: one closed outline per point.

    ``right`` and ``up`` are the camera's screen axes in world coordinates, so a square looks
    like a square from wherever it is viewed, and ``sizes`` (metres, one per point) can be set
    from the camera distance to hold a constant size on screen.
    """
    pts = np.asarray(points, float).reshape(-1, 3)
    if not len(pts):
        return pv.PolyData()
    corners = GLYPH_SHAPES[shape]
    sizes = np.broadcast_to(np.asarray(sizes, float), (len(pts),))
    basis = np.outer(corners[:, 0], np.asarray(right, float)) + \
        np.outer(corners[:, 1], np.asarray(up, float))                 # (k, 3) unit outline
    paths = [p + basis * s for p, s in zip(pts, sizes)]
    return polylines(paths, list(colours))


def segments(starts: np.ndarray, ends: np.ndarray) -> pv.PolyData:
    n = len(starts)
    if n == 0:
        return pv.PolyData()
    pts = np.empty((2 * n, 3))
    pts[0::2], pts[1::2] = starts, ends
    lines = np.column_stack([np.full(n, 2), np.arange(0, 2 * n, 2), np.arange(1, 2 * n, 2)]).ravel()
    return pv.PolyData(pts, lines=lines)


class MeshLayer:
    """A persistent actor whose dataset is replaced every frame."""

    def __init__(self, plotter, name, rgb=True, color=None, **kw):
        self.mesh = pv.PolyData(np.zeros((1, 3)))
        self.rgb = rgb
        if rgb:
            self.mesh.point_data["rgb"] = np.zeros((1, 3), np.uint8)
            self.actor = plotter.add_mesh(self.mesh, scalars="rgb", rgb=True, name=name,
                                          show_scalar_bar=False, **kw)
        else:
            self.actor = plotter.add_mesh(self.mesh, color=color, name=name, **kw)
        self.actor.SetVisibility(False)

    def set(self, mesh: pv.PolyData | None, visible: bool = True):
        if mesh is None or mesh.n_points == 0 or not visible:
            self.actor.SetVisibility(False)
            return
        self.mesh.copy_from(mesh)
        self.actor.SetVisibility(True)
