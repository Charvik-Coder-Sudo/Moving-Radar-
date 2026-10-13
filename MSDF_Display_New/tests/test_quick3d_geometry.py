"""The numpy-to-GPU buffer packing.

A vertex buffer with the wrong stride or a mis-placed attribute offset does not raise - it
draws a scrambled mesh, or nothing at all. So these tests read the packed bytes back out of
the geometry object and check that every field landed where the attribute declarations say
it did.

Runs off-screen: the geometry objects are built and filled, but nothing is rendered.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtGui  # noqa: E402
from PySide6.QtQuick3D import QQuick3DGeometry  # noqa: E402

from visualization.quick3d.geometry import (LINE_STRIDE, MESH_STRIDE,  # noqa: E402
                                            LineGeometry, MeshGeometry)
from visualization.quick3d.transform import enu_to_quick  # noqa: E402

QAPP = QtGui.QGuiApplication.instance() or QtGui.QGuiApplication([])

# A square of ground in ENU, 1 km on a side at 100 m altitude. The winding is deliberately
# the one that makes the normals point DOWN - which is what PyVista's structured-grid
# triangulation produces for terrain, and what face_up exists to correct.
GROUND = np.array([[0.0, 0.0, 100.0], [1000.0, 0.0, 100.0],
                   [1000.0, 1000.0, 100.0], [0.0, 1000.0, 100.0]])
TRIS = np.array([[0, 2, 1], [0, 3, 2]], np.uint32)


def unpack(geo, stride, count):
    """The vertex buffer as (count, stride/4) float32, straight from the geometry object."""
    raw = bytes(geo.vertexData())
    return np.frombuffer(raw, np.float32).reshape(count, stride // 4)


class TestTheFixture(unittest.TestCase):
    def test_the_ground_really_is_wound_facing_down(self):
        """If this ever stops being true, the face_up tests below stop testing anything."""
        a, b, c = GROUND[TRIS[:, 0]], GROUND[TRIS[:, 1]], GROUND[TRIS[:, 2]]
        self.assertTrue(np.all(np.cross(b - a, c - a)[:, 2] < 0))       # -Z in ENU is down


class TestMeshGeometry(unittest.TestCase):
    def mesh(self, **kw):
        g = MeshGeometry()
        n, m = g.set_mesh(GROUND, TRIS, (40, 80, 120), **kw)
        return g, n, m

    def test_it_reports_what_it_uploaded(self):
        _g, n, m = self.mesh()
        self.assertEqual((n, m), (4, 2))

    def test_the_declared_stride_matches_the_packing(self):
        g, n, _ = self.mesh()
        self.assertEqual(g.stride(), MESH_STRIDE)
        self.assertEqual(len(bytes(g.vertexData())), n * MESH_STRIDE)

    def test_positions_are_the_world_points_in_renderer_coordinates(self):
        g, n, _ = self.mesh()
        np.testing.assert_allclose(unpack(g, MESH_STRIDE, n)[:, 0:3],
                                   enu_to_quick(GROUND), atol=1e-3)

    def test_colour_is_broadcast_to_every_vertex_as_0_to_1(self):
        g, n, _ = self.mesh()
        rgba = unpack(g, MESH_STRIDE, n)[:, 6:10]
        np.testing.assert_allclose(rgba[:, 0], 40 / 255.0, atol=1e-6)
        np.testing.assert_allclose(rgba[:, 1], 80 / 255.0, atol=1e-6)
        np.testing.assert_allclose(rgba[:, 2], 120 / 255.0, atol=1e-6)
        np.testing.assert_allclose(rgba[:, 3], 1.0, atol=1e-6)

    def test_one_colour_per_vertex_is_kept_distinct(self):
        g = MeshGeometry()
        cols = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0]])
        n, _ = g.set_mesh(GROUND, TRIS, cols)
        np.testing.assert_allclose(unpack(g, MESH_STRIDE, n)[:, 6:9], cols / 255.0, atol=1e-6)

    def test_alpha_is_carried_through(self):
        g = MeshGeometry()
        n, _ = g.set_mesh(GROUND, TRIS, (40, 80, 120), alpha=0.25)
        np.testing.assert_allclose(unpack(g, MESH_STRIDE, n)[:, 9], 0.25, atol=1e-6)

    def test_face_up_makes_the_normals_point_up(self):
        """Without this the terrain is back-face culled and the ground vanishes."""
        g, n, _ = self.mesh(face_up=True)
        np.testing.assert_allclose(unpack(g, MESH_STRIDE, n)[:, 3:6][:, 1], 1.0, atol=1e-5)

    def test_without_face_up_this_ground_would_face_down(self):
        g, n, _ = self.mesh(face_up=False)
        self.assertTrue(np.all(unpack(g, MESH_STRIDE, n)[:, 3:6][:, 1] < 0))

    def test_normals_are_unit_length(self):
        g, n, _ = self.mesh(face_up=True)
        lengths = np.linalg.norm(unpack(g, MESH_STRIDE, n)[:, 3:6], axis=1)
        np.testing.assert_allclose(lengths, 1.0, atol=1e-5)

    def test_indices_survive_as_unsigned_32_bit(self):
        g, _, m = self.mesh()
        idx = np.frombuffer(bytes(g.indexData()), np.uint32)
        self.assertEqual(len(idx), m * 3)
        self.assertEqual(set(idx.tolist()), {0, 1, 2, 3})

    def test_bounds_cover_every_point(self):
        g, _, _ = self.mesh()
        q = enu_to_quick(GROUND)
        lo, hi = g.boundsMin(), g.boundsMax()
        for axis, name in enumerate("xyz"):
            self.assertLessEqual(getattr(lo, name)(), q[:, axis].min() + 1e-3)
            self.assertGreaterEqual(getattr(hi, name)(), q[:, axis].max() - 1e-3)

    def test_it_is_declared_as_triangles(self):
        g, _, _ = self.mesh()
        self.assertEqual(g.primitiveType(), QQuick3DGeometry.PrimitiveType.Triangles)

    def test_empty_input_draws_nothing_instead_of_raising(self):
        g = MeshGeometry()
        self.assertEqual(g.set_mesh(np.empty((0, 3)), np.empty((0, 3), np.uint32), (1, 2, 3)),
                         (0, 0))
        self.assertEqual(g.vertex_count, 0)

    def test_reloading_replaces_rather_than_appends(self):
        """Layers are rebuilt every frame, so a leak here would grow without bound."""
        g, _, _ = self.mesh()
        for _ in range(5):
            n, m = g.set_mesh(GROUND, TRIS, (10, 20, 30))
        self.assertEqual((n, m), (4, 2))
        self.assertEqual(len(bytes(g.vertexData())), 4 * MESH_STRIDE)

    def test_clearing_leaves_a_usable_object(self):
        g, _, _ = self.mesh()
        g.clear_mesh()
        self.assertEqual(g.vertex_count, 0)
        self.assertEqual(g.set_mesh(GROUND, TRIS, (1, 2, 3)), (4, 2))


class TestLineGeometry(unittest.TestCase):
    SEGS = np.array([[[0.0, 0.0, 500.0], [1000.0, 0.0, 500.0]],
                     [[1000.0, 0.0, 500.0], [1000.0, 1000.0, 700.0]]])

    def lines(self, **kw):
        g = LineGeometry()
        n = g.set_segments(self.SEGS, (250, 204, 21), **kw)
        return g, n

    def test_two_vertices_per_segment(self):
        _g, n = self.lines()
        self.assertEqual(n, 2 * len(self.SEGS))

    def test_the_declared_stride_matches_the_packing(self):
        g, n = self.lines()
        self.assertEqual(g.stride(), LINE_STRIDE)
        self.assertEqual(len(bytes(g.vertexData())), n * LINE_STRIDE)

    def test_endpoints_are_in_renderer_coordinates_and_in_order(self):
        g, n = self.lines()
        np.testing.assert_allclose(unpack(g, LINE_STRIDE, n)[:, 0:3],
                                   enu_to_quick(self.SEGS.reshape(-1, 3)), atol=1e-3)

    def test_colour_lands_after_the_position(self):
        g, n = self.lines()
        rgba = unpack(g, LINE_STRIDE, n)[:, 3:7]
        np.testing.assert_allclose(rgba[:, 0], 250 / 255.0, atol=1e-6)
        np.testing.assert_allclose(rgba[:, 3], 1.0, atol=1e-6)

    def test_a_fading_trail_can_colour_each_vertex(self):
        g = LineGeometry()
        cols = np.array([[255, 0, 0], [200, 0, 0], [150, 0, 0], [100, 0, 0]])
        n = g.set_segments(self.SEGS, cols)
        np.testing.assert_allclose(unpack(g, LINE_STRIDE, n)[:, 3], cols[:, 0] / 255.0, atol=1e-6)

    def test_it_is_declared_as_lines(self):
        g, _ = self.lines()
        self.assertEqual(g.primitiveType(), QQuick3DGeometry.PrimitiveType.Lines)

    def test_no_segments_draws_nothing_instead_of_raising(self):
        g = LineGeometry()
        self.assertEqual(g.set_segments(np.empty((0, 2, 3)), (1, 2, 3)), 0)

    def test_rebuilding_every_frame_does_not_grow_the_buffer(self):
        g = LineGeometry()
        for _ in range(20):
            n = g.set_segments(self.SEGS, (250, 204, 21))
        self.assertEqual(len(bytes(g.vertexData())), n * LINE_STRIDE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
