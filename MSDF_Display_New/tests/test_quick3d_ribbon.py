"""Lines with a width, built as camera-facing ribbons.

Qt Quick 3D renders every line one pixel wide, and this display uses line weight to say that
a fused system track outranks a sensor track. Ribbons put the weight back. What has to hold:
a ribbon is the width it was asked for, measured on screen rather than in metres; it faces
the camera from wherever it is seen; and it never collapses, including for a segment pointing
straight at the viewer.

Pure numpy - no Qt, no GPU.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

from visualization.quick3d.ribbon import (decimate, fade, glyph_segments,  # noqa: E402
                                          path_segments, paths_to_segments, per_vertex,
                                          pixel_scale, ribbons)
from visualization.quick3d.transform import look_at_basis, project  # noqa: E402

EYE = np.array([0.0, -10000.0, 0.0])
FWD = np.array([0.0, 1.0, 0.0])          # the camera looks North
FOV, W, H = 40.0, 1200.0, 800.0


class TestPixelScale(unittest.TestCase):
    def test_a_pixel_covers_more_ground_further_away(self):
        near, far = pixel_scale(1000.0, H, FOV), pixel_scale(2000.0, H, FOV)
        self.assertAlmostEqual(float(far / near), 2.0, places=9)

    def test_the_whole_viewport_spans_the_field_of_view(self):
        """height pixels at distance d must cover 2 d tan(fov/2), or every width is wrong."""
        d = 5000.0
        self.assertAlmostEqual(float(pixel_scale(d, H, FOV) * H),
                               2 * d * np.tan(np.radians(FOV / 2)), places=6)

    def test_it_is_vectorised(self):
        out = pixel_scale(np.array([100.0, 200.0, 400.0]), H, FOV)
        self.assertEqual(out.shape, (3,))


class TestRibbonWidth(unittest.TestCase):
    """The measurement that matters: how wide is the ribbon once it is on screen?"""

    def screen_width(self, seg, width_px, eye=EYE):
        focal = seg.mean(axis=0)
        pts, _tris = ribbons(seg[None], eye, focal - eye, width_px, H, FOV)
        basis = look_at_basis(eye, focal)
        sx, sy, front = project(pts, eye, basis, W, H, FOV)
        self.assertTrue(np.all(front))
        # the quad's two start corners are the width apart, on screen
        return float(np.hypot(sx[1] - sx[0], sy[1] - sy[0]))

    def test_a_ribbon_is_the_pixel_width_it_was_asked_for(self):
        seg = np.array([[-500.0, 0.0, 0.0], [500.0, 0.0, 0.0]])
        for want in (1.0, 2.6, 6.0):
            self.assertAlmostEqual(self.screen_width(seg, want), want, places=3)

    def test_the_width_holds_at_any_distance(self):
        """This is the whole point: a line width does not shrink with range, so nor may this."""
        near = np.array([[-500.0, -9000.0, 0.0], [500.0, -9000.0, 0.0]])
        far = np.array([[-500.0, 60000.0, 0.0], [500.0, 60000.0, 0.0]])
        self.assertAlmostEqual(self.screen_width(near, 3.0), 3.0, places=2)
        self.assertAlmostEqual(self.screen_width(far, 3.0), 3.0, places=2)

    def test_a_heavier_line_really_is_heavier(self):
        """Fused trails are drawn at 2.6 px against a sensor track's 1.1: the ratio has to
        survive, or the symbology stops saying anything."""
        seg = np.array([[-500.0, 0.0, 0.0], [500.0, 0.0, 0.0]])
        self.assertAlmostEqual(self.screen_width(seg, 2.6) / self.screen_width(seg, 1.1),
                               2.6 / 1.1, places=3)

    def test_a_minimum_world_width_can_be_forced(self):
        seg = np.array([[-500.0, 0.0, 0.0], [500.0, 0.0, 0.0]])
        pts, _ = ribbons(seg[None], EYE, FWD, 0.001, H, FOV, min_width_m=200.0)
        self.assertAlmostEqual(float(np.linalg.norm(pts[1] - pts[0])), 200.0, places=6)


class TestRibbonGeometry(unittest.TestCase):
    def test_two_triangles_and_four_points_per_segment(self):
        segs = np.random.default_rng(0).normal(size=(7, 2, 3)) * 1000
        pts, tris = ribbons(segs, EYE, FWD, 2.0, H, FOV)
        self.assertEqual(pts.shape, (28, 3))
        self.assertEqual(tris.shape, (14, 3))

    def test_each_quad_uses_only_its_own_four_points(self):
        segs = np.random.default_rng(1).normal(size=(5, 2, 3)) * 1000
        _pts, tris = ribbons(segs, EYE, FWD, 2.0, H, FOV)
        for i in range(5):
            self.assertEqual(set(tris[2 * i:2 * i + 2].ravel().tolist()),
                             {4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3})

    def test_the_ribbon_is_centred_on_the_line_it_replaces(self):
        seg = np.array([[[-500.0, 0.0, 300.0], [500.0, 0.0, 300.0]]])
        pts, _ = ribbons(seg, EYE, FWD, 4.0, H, FOV)
        np.testing.assert_allclose(0.5 * (pts[0] + pts[1]), seg[0, 0], atol=1e-6)
        np.testing.assert_allclose(0.5 * (pts[2] + pts[3]), seg[0, 1], atol=1e-6)

    def test_it_faces_the_camera(self):
        """A ribbon seen edge-on would vanish; its normal must point back at the viewer."""
        seg = np.array([[[-500.0, 0.0, 0.0], [500.0, 0.0, 0.0]]])
        pts, tris = ribbons(seg, EYE, FWD, 4.0, H, FOV)
        a, b, c = pts[tris[0, 0]], pts[tris[0, 1]], pts[tris[0, 2]]
        normal = np.cross(b - a, c - a)
        normal /= np.linalg.norm(normal)
        to_eye = EYE - seg[0].mean(axis=0)
        to_eye /= np.linalg.norm(to_eye)
        self.assertGreater(abs(float(np.dot(normal, to_eye))), 0.99)

    def test_it_turns_with_the_camera(self):
        seg = np.array([[[-500.0, 0.0, 0.0], [500.0, 0.0, 0.0]]])
        eye_a = np.array([0.0, -9000.0, 0.0])
        eye_b = np.array([0.0, 0.0, 9000.0])
        side_a = ribbons(seg, eye_a, -eye_a, 4.0, H, FOV)[0][1] - seg[0, 0]
        side_b = ribbons(seg, eye_b, -eye_b, 4.0, H, FOV)[0][1] - seg[0, 0]
        self.assertLess(abs(float(np.dot(side_a / np.linalg.norm(side_a),
                                         side_b / np.linalg.norm(side_b)))), 0.2)

    def test_a_segment_pointing_at_the_camera_does_not_collapse(self):
        """End-on there is no unique sideways direction; a zero-area quad would flicker."""
        seg = np.array([[[0.0, -5000.0, 0.0], [0.0, -1000.0, 0.0]]])   # straight at the eye
        pts, _ = ribbons(seg, EYE, FWD, 4.0, H, FOV)
        self.assertGreater(float(np.linalg.norm(pts[1] - pts[0])), 1e-6)
        self.assertTrue(np.all(np.isfinite(pts)))

    def test_a_zero_length_segment_is_still_finite(self):
        seg = np.array([[[100.0, 100.0, 100.0], [100.0, 100.0, 100.0]]])
        pts, _ = ribbons(seg, EYE, FWD, 3.0, H, FOV)
        self.assertTrue(np.all(np.isfinite(pts)))

    def test_no_segments_gives_no_geometry(self):
        pts, tris = ribbons(np.empty((0, 2, 3)), EYE, FWD, 2.0, H, FOV)
        self.assertEqual(len(pts), 0)
        self.assertEqual(len(tris), 0)


class TestPaths(unittest.TestCase):
    def test_a_polyline_becomes_consecutive_segments(self):
        path = np.array([[0.0, 0, 0], [1.0, 0, 0], [2.0, 0, 0]])
        segs = path_segments(path)
        self.assertEqual(segs.shape, (2, 2, 3))
        np.testing.assert_allclose(segs[0, 1], segs[1, 0])       # they join

    def test_a_path_too_short_to_draw_gives_nothing(self):
        self.assertEqual(len(path_segments(np.array([[0.0, 0, 0]]))), 0)
        self.assertEqual(len(path_segments(np.empty((0, 3)))), 0)

    def test_several_paths_keep_track_of_which_is_which(self):
        paths = [np.zeros((4, 3)), np.zeros((1, 3)), np.zeros((3, 3))]
        segs, owner = paths_to_segments(paths)
        self.assertEqual(len(segs), 3 + 0 + 2)
        self.assertEqual(owner.tolist(), [0, 0, 0, 2, 2])         # the 1-point path drops out

    def test_a_long_trail_is_thinned_before_it_is_drawn(self):
        """A trail reaches well over a thousand points; as ribbons that is four vertices and
        two triangles each, rebuilt every frame."""
        path = np.c_[np.arange(1500.0), np.zeros(1500), np.zeros(1500)]
        self.assertEqual(len(decimate(path, 240)), 240)

    def test_thinning_keeps_both_ends_so_the_trail_stays_attached(self):
        path = np.c_[np.arange(1500.0), np.zeros(1500), np.zeros(1500)]
        thin = decimate(path, 240)
        np.testing.assert_allclose(thin[0], path[0])
        np.testing.assert_allclose(thin[-1], path[-1])

    def test_thinning_preserves_the_order_and_the_shape(self):
        path = np.c_[np.arange(900.0), np.sin(np.arange(900.0) / 40.0) * 100, np.zeros(900)]
        thin = decimate(path, 120)
        self.assertTrue(np.all(np.diff(thin[:, 0]) > 0))
        self.assertLess(abs(float(thin[:, 1].max() - path[:, 1].max())), 5.0)

    def test_a_short_trail_is_left_alone(self):
        path = np.c_[np.arange(30.0), np.zeros(30), np.zeros(30)]
        np.testing.assert_array_equal(decimate(path, 240), path)

    def test_per_vertex_repeats_a_segment_colour_over_its_quad(self):
        cols = np.array([[255, 0, 0], [0, 255, 0]])
        out = per_vertex(cols)
        self.assertEqual(out.shape, (8, 3))
        np.testing.assert_array_equal(out[:4], np.tile([255, 0, 0], (4, 1)))

    def test_a_trail_fades_from_oldest_to_newest(self):
        cols = fade((200, 100, 50), 5, oldest=0.2)
        self.assertEqual(cols.shape, (5, 3))
        self.assertTrue(np.all(np.diff(cols[:, 0]) > 0))          # gets stronger
        np.testing.assert_allclose(cols[-1], (200, 100, 50))      # newest is full strength
        np.testing.assert_allclose(cols[0], (40, 20, 10))         # oldest is 0.2 of it


class TestGlyphs(unittest.TestCase):
    RIGHT, UP = np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])

    def test_a_square_is_a_closed_outline_of_four_sides(self):
        segs = glyph_segments(np.zeros((1, 3)), "square", [100.0], self.RIGHT, self.UP)
        self.assertEqual(len(segs), 4)
        np.testing.assert_allclose(segs[-1, 1], segs[0, 0], atol=1e-9)    # it closes

    def test_a_triangle_is_a_closed_outline_of_three_sides(self):
        segs = glyph_segments(np.zeros((1, 3)), "triangle", [100.0], self.RIGHT, self.UP)
        self.assertEqual(len(segs), 3)
        np.testing.assert_allclose(segs[-1, 1], segs[0, 0], atol=1e-9)

    def test_the_triangle_points_up_on_screen(self):
        """Apex up is what makes a fused track read as a triangle rather than a blob."""
        segs = glyph_segments(np.zeros((1, 3)), "triangle", [100.0], self.RIGHT, self.UP)
        apex = segs[0, 0]
        self.assertGreater(float(np.dot(apex, self.UP)), 0.0)
        self.assertAlmostEqual(float(np.dot(apex, self.RIGHT)), 0.0, places=9)

    def test_a_glyph_lies_in_the_camera_plane(self):
        segs = glyph_segments(np.array([[10.0, 20.0, 30.0]]), "square", [50.0],
                              self.RIGHT, self.UP)
        offsets = segs.reshape(-1, 3) - np.array([10.0, 20.0, 30.0])
        normal = np.cross(self.RIGHT, self.UP)
        np.testing.assert_allclose(offsets @ normal, 0.0, atol=1e-9)

    def test_size_scales_the_outline(self):
        small = glyph_segments(np.zeros((1, 3)), "square", [10.0], self.RIGHT, self.UP)
        big = glyph_segments(np.zeros((1, 3)), "square", [40.0], self.RIGHT, self.UP)
        np.testing.assert_allclose(big, small * 4.0, atol=1e-9)

    def test_every_track_gets_its_own_outline(self):
        pts = np.array([[0.0, 0, 0], [1000.0, 0, 0], [2000.0, 0, 0]])
        segs = glyph_segments(pts, "square", [50.0, 50.0, 50.0], self.RIGHT, self.UP)
        self.assertEqual(len(segs), 3 * 4)

    def test_no_tracks_gives_no_outlines(self):
        self.assertEqual(len(glyph_segments(np.empty((0, 3)), "square", [], self.RIGHT, self.UP)), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
