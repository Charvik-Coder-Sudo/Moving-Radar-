"""numpy meshes -> Qt Quick 3D custom geometry.

Two QML-registered geometry types cover everything this display draws:

    MeshGeometry    indexed triangles with per-vertex colour - terrain, aircraft, coverage
                    volumes, the scan beam, camera-facing track symbols
    LineGeometry    line segments with per-vertex colour - trails, range rings, velocity and
                    acceleration vectors, boresights, the ENU grid and body axes

Both take world ENU metres and convert once, through ``transform`` - the rendering code
never sees a raw frame conversion. Vertex buffers are interleaved and uploaded in one call;
rebuilding a layer every frame was measured at a median of 0.74 ms for 2,400 line segments,
which is what makes per-frame updates affordable at all.

Colour arrives as 0-255 RGB because that is what ``visualization.scenery`` and
``visualization.style`` already produce.
"""

from __future__ import annotations

import numpy as np
from PySide6 import QtCore, QtGui, QtQml
from PySide6.QtQuick3D import QQuick3DGeometry

from visualization.quick3d import transform

QML_IMPORT_NAME = "Msdf.Geometry"
QML_IMPORT_MAJOR_VERSION = 1

_POS = QQuick3DGeometry.Attribute.Semantic.PositionSemantic
_NRM = QQuick3DGeometry.Attribute.Semantic.NormalSemantic
_COL = QQuick3DGeometry.Attribute.Semantic.ColorSemantic
_IDX = QQuick3DGeometry.Attribute.Semantic.IndexSemantic
_F32 = QQuick3DGeometry.Attribute.ComponentType.F32Type
_U32 = QQuick3DGeometry.Attribute.ComponentType.U32Type

MESH_STRIDE = 40                 # position 3f + normal 3f + colour 4f
LINE_STRIDE = 28                 # position 3f + colour 4f


def _rgba(colours_rgb, n: int, alpha) -> np.ndarray:
    """(n, 4) float RGBA from 0-255 RGB, broadcasting one colour over every vertex."""
    rgba = np.empty((n, 4), np.float32)
    rgba[:, :3] = np.asarray(colours_rgb, np.float32).reshape(-1, 3) / 255.0
    rgba[:, 3] = np.asarray(alpha, np.float32).reshape(-1)
    return rgba


@QtQml.QmlElement
class MeshGeometry(QQuick3DGeometry):
    """An indexed triangle mesh with per-vertex colour and smooth normals."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.vertex_count = 0
        self.triangle_count = 0

    def set_mesh(self, points_enu, tris, colours_rgb, face_up: bool = False, alpha=1.0,
                 world: bool = True, smooth_normals: bool = True):
        """Upload a mesh. points (N,3), tris (M,3), colours (3,) or (N,3) 0-255.

        ``world`` True means the points are world ENU metres and are mapped into renderer
        coordinates here. False means they are already in the local frame of the node that
        will place them - an aircraft in FRD, a coverage volume in radar FRD - so they are
        taken as given and the node's own transform does the rest
        (``transform.node_rotation``). A moving object is uploaded once that way and then
        only its node moves; no vertex is touched again.

        ``face_up`` forces the winding so normals point up, for ground surfaces - see
        ``transform.orient_up`` for why that is needed here and was not needed under VTK.
        """
        pts = (transform.enu_to_quick(points_enu) if world
               else np.asarray(points_enu, float).reshape(-1, 3)).astype(np.float32)
        tris = np.asarray(tris, np.uint32)
        if not len(pts) or not len(tris):
            return self.clear_mesh()
        if face_up:
            tris = transform.orient_up(pts, tris).astype(np.uint32)
        buf = np.empty((len(pts), 10), np.float32)
        buf[:, 0:3] = pts
        # Unlit geometry - ribbons, coverage volumes, the scan beam - is never shaded, and
        # computing smooth normals for it is the single most expensive thing in a frame
        # (an unbuffered scatter-add over every triangle). The attribute still has to be
        # there, because the stride and the offsets below declare it, so it is filled with a
        # constant instead of left out.
        if smooth_normals:
            buf[:, 3:6] = transform.vertex_normals(pts, tris)
        else:
            buf[:, 3:5], buf[:, 5] = 0.0, 1.0
        buf[:, 6:10] = _rgba(colours_rgb, len(pts), alpha)

        self.clear()
        self.setVertexData(QtCore.QByteArray(buf.tobytes()))
        self.setIndexData(QtCore.QByteArray(np.ascontiguousarray(tris).ravel().tobytes()))
        self.setStride(MESH_STRIDE)
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.addAttribute(_POS, 0, _F32)
        self.addAttribute(_NRM, 12, _F32)
        self.addAttribute(_COL, 24, _F32)
        self.addAttribute(_IDX, 0, _U32)
        self.setBounds(QtGui.QVector3D(*pts.min(axis=0)), QtGui.QVector3D(*pts.max(axis=0)))
        self.update()
        self.vertex_count, self.triangle_count = len(pts), len(tris)
        return self.vertex_count, self.triangle_count

    def clear_mesh(self):
        """Draw nothing, without destroying the object the QML scene holds."""
        self.clear()
        self.update()
        self.vertex_count = self.triangle_count = 0
        return 0, 0


@QtQml.QmlElement
class LineGeometry(QQuick3DGeometry):
    """Line segments with per-vertex colour: trails, rings, vectors, axes."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.vertex_count = 0

    def set_segments(self, segments_enu, colours_rgb, alpha=1.0):
        """Upload line segments. segments (M,2,3) ENU metres -> 2M vertices.

        Colour may be one RGB for the whole layer or one per vertex, which is how a trail
        fades with age.
        """
        segs = np.asarray(segments_enu, float)
        if not segs.size:
            return self.clear_lines()
        pts = transform.enu_to_quick(segs.reshape(-1, 3)).astype(np.float32)
        buf = np.empty((len(pts), 7), np.float32)
        buf[:, 0:3] = pts
        buf[:, 3:7] = _rgba(colours_rgb, len(pts), alpha)

        self.clear()
        self.setVertexData(QtCore.QByteArray(buf.tobytes()))
        self.setStride(LINE_STRIDE)
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Lines)
        self.addAttribute(_POS, 0, _F32)
        self.addAttribute(_COL, 12, _F32)
        self.setBounds(QtGui.QVector3D(*pts.min(axis=0)), QtGui.QVector3D(*pts.max(axis=0)))
        self.update()
        self.vertex_count = len(pts)
        return self.vertex_count

    def clear_lines(self):
        self.clear()
        self.update()
        self.vertex_count = 0
        return 0
