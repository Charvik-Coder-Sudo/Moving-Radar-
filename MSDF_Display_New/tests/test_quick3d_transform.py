"""The world-to-renderer transform, the camera basis and the screen projection.

These are the parts that fail silently. A mirrored axis does not raise; it draws an aircraft
banking the wrong way. A camera basis that disagrees with the projection does not raise; it
puts a track's label next to the wrong aircraft. So each one is checked against a value
worked out independently, not against whatever the code happens to return.

No Qt and no GPU: everything here is numpy.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

from msdf_math import attitude  # noqa: E402
from visualization.quick3d.transform import (ENU_TO_QUICK, basis_to_quaternion,  # noqa: E402
                                             enu_to_quick, look_at_basis, node_rotation,
                                             orient_up, project, quick_to_enu,
                                             ring_triangles, vertex_normals)


class TestFrameMapping(unittest.TestCase):
    def test_the_three_world_axes_land_where_they_should(self):
        # East stays right, Up becomes the renderer's up, North goes into the screen (-Z)
        np.testing.assert_allclose(enu_to_quick((1, 0, 0))[0], (1, 0, 0), atol=1e-12)
        np.testing.assert_allclose(enu_to_quick((0, 0, 1))[0], (0, 1, 0), atol=1e-12)
        np.testing.assert_allclose(enu_to_quick((0, 1, 0))[0], (0, 0, -1), atol=1e-12)

    def test_handedness_is_preserved_so_nothing_is_ever_mirrored(self):
        """A determinant of -1 would silently mirror every model. This is the check that
        stops an aircraft banking left when the data says right."""
        self.assertAlmostEqual(float(np.linalg.det(ENU_TO_QUICK)), 1.0, places=12)

    def test_it_is_a_rotation_not_a_scale_or_a_shear(self):
        np.testing.assert_allclose(ENU_TO_QUICK @ ENU_TO_QUICK.T, np.eye(3), atol=1e-12)

    def test_distances_survive_the_trip(self):
        rng = np.random.default_rng(1)
        pts = rng.uniform(-90000, 90000, size=(40, 3))
        d_enu = np.linalg.norm(pts[1:] - pts[:-1], axis=1)
        q = enu_to_quick(pts)
        np.testing.assert_allclose(np.linalg.norm(q[1:] - q[:-1], axis=1), d_enu, atol=1e-6)

    def test_round_trip(self):
        rng = np.random.default_rng(2)
        pts = rng.uniform(-90000, 90000, size=(25, 3))
        np.testing.assert_allclose(quick_to_enu(enu_to_quick(pts)), pts, atol=1e-9)

    def test_a_cross_product_maps_like_a_vector(self):
        """Because the mapping is a proper rotation, a normal can be transformed the same
        way as a position. If this failed, every lighting normal would need a separate rule."""
        rng = np.random.default_rng(5)
        u, v = rng.normal(size=3), rng.normal(size=3)
        direct = enu_to_quick(np.cross(u, v))[0]
        after = np.cross(enu_to_quick(u)[0], enu_to_quick(v)[0])
        np.testing.assert_allclose(direct, after, atol=1e-12)


class TestAircraftOrientation(unittest.TestCase):
    """Section 18 of the brief: the model must never come out mirrored or turned by 90/180
    degrees because of an undocumented renderer conversion."""

    def nose(self, yaw, pitch=0.0, roll=0.0):
        """Where the nose (+X forward in FRD) points, in renderer coordinates."""
        R = attitude.body_to_world(yaw, pitch, roll)
        return enu_to_quick(R @ np.array([1.0, 0.0, 0.0]))[0]

    def test_north_flight_points_into_the_screen(self):
        np.testing.assert_allclose(self.nose(0.0), (0, 0, -1), atol=1e-9)

    def test_east_flight_points_right(self):
        np.testing.assert_allclose(self.nose(90.0), (1, 0, 0), atol=1e-9)

    def test_south_flight_points_out_of_the_screen(self):
        np.testing.assert_allclose(self.nose(180.0), (0, 0, 1), atol=1e-9)

    def test_west_flight_points_left(self):
        np.testing.assert_allclose(self.nose(270.0), (-1, 0, 0), atol=1e-9)

    def test_climb_raises_the_nose(self):
        self.assertGreater(self.nose(0.0, pitch=10.0)[1], 0.0)

    def test_descent_lowers_the_nose(self):
        self.assertLess(self.nose(0.0, pitch=-10.0)[1], 0.0)

    def test_bank_right_drops_the_right_wingtip(self):
        """Right wing is +Y in FRD. A positive roll is right wing down, so its renderer
        height must be below the left wingtip's."""
        R = attitude.body_to_world(0.0, 0.0, 20.0)
        right = enu_to_quick(R @ np.array([0.0, 1.0, 0.0]))[0]
        left = enu_to_quick(R @ np.array([0.0, -1.0, 0.0]))[0]
        self.assertLess(right[1], left[1])

    def test_bank_left_drops_the_left_wingtip(self):
        R = attitude.body_to_world(0.0, 0.0, -20.0)
        right = enu_to_quick(R @ np.array([0.0, 1.0, 0.0]))[0]
        left = enu_to_quick(R @ np.array([0.0, -1.0, 0.0]))[0]
        self.assertLess(left[1], right[1])

    def test_the_model_keeps_its_shape(self):
        """A combined attitude must stay a rigid body: right angles stay right angles."""
        R = attitude.body_to_world(37.0, 12.0, -21.0)
        f, r, d = (enu_to_quick(R @ np.eye(3)[k])[0] for k in range(3))
        for u, v in ((f, r), (r, d), (d, f)):
            self.assertAlmostEqual(float(np.dot(u, v)), 0.0, places=9)
        for u in (f, r, d):
            self.assertAlmostEqual(float(np.linalg.norm(u)), 1.0, places=9)


class TestWinding(unittest.TestCase):
    """A square of flat ground in renderer coordinates (y is up, so the quad lies in x-z)."""

    PTS = np.array([[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0],
                    [1000.0, 0.0, 1000.0], [0.0, 0.0, 1000.0]])
    FACING_UP = np.array([[0, 2, 1], [0, 3, 2]])
    FACING_DOWN = np.array([[0, 1, 2], [0, 2, 3]])       # what PyVista's terrain produces

    def normals_y(self, tris):
        a, b, c = self.PTS[tris[:, 0]], self.PTS[tris[:, 1]], self.PTS[tris[:, 2]]
        return np.cross(b - a, c - a)[:, 1]

    def test_the_fixture_itself_is_wound_the_way_it_claims(self):
        self.assertTrue(np.all(self.normals_y(self.FACING_UP) > 0))
        self.assertTrue(np.all(self.normals_y(self.FACING_DOWN) < 0))

    def test_downward_wound_ground_is_flipped_up(self):
        """Qt Quick 3D back-face-culls, so downward-facing terrain disappears entirely.
        VTK lit both sides, which is why this never showed there."""
        fixed = orient_up(self.PTS, self.FACING_DOWN)
        self.assertTrue(np.all(self.normals_y(fixed) > 0))

    def test_ground_already_facing_up_is_left_alone(self):
        np.testing.assert_array_equal(orient_up(self.PTS, self.FACING_UP), self.FACING_UP)

    def test_a_mixed_mesh_ends_up_entirely_facing_up(self):
        mixed = np.vstack([self.FACING_UP[:1], self.FACING_DOWN[1:]])
        self.assertTrue(np.all(self.normals_y(orient_up(self.PTS, mixed)) > 0))

    def test_it_only_reorders_indices_never_changes_which_vertices_a_face_uses(self):
        out = orient_up(self.PTS, self.FACING_DOWN)
        for before, after in zip(self.FACING_DOWN, out):
            self.assertEqual(set(before), set(after))

    def test_the_caller_s_array_is_not_modified_in_place(self):
        original = self.FACING_DOWN.copy()
        orient_up(self.PTS, self.FACING_DOWN)
        np.testing.assert_array_equal(self.FACING_DOWN, original)

    def test_normals_follow_the_corrected_winding(self):
        n = vertex_normals(self.PTS.astype(np.float32), orient_up(self.PTS, self.FACING_DOWN))
        np.testing.assert_allclose(n[:, 1], 1.0, atol=1e-6)


class TestNodeRotation(unittest.TestCase):
    """An aircraft is uploaded once in its body frame and placed by its node's rotation.
    If that quaternion is wrong the model flies sideways, and nothing raises."""

    @staticmethod
    def matrix(q):
        x, y, z, w = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    def test_it_is_the_whole_body_to_renderer_chain(self):
        R = attitude.body_to_world(37.0, 12.0, -21.0)
        np.testing.assert_allclose(self.matrix(node_rotation(R)), ENU_TO_QUICK @ R, atol=1e-9)

    def test_the_quaternion_is_normalised(self):
        q = node_rotation(attitude.body_to_world(137.0, -8.0, 44.0))
        self.assertAlmostEqual(float(np.dot(q, q)), 1.0, places=9)

    def test_a_nose_in_the_body_frame_ends_up_where_the_attitude_points_it(self):
        """The whole point of the node transform: no vertex is touched, so the rotation
        alone has to carry the attitude."""
        for yaw, pitch, roll in ((0, 0, 0), (65, 10, 0), (200, -7, 33), (310, 4, -18)):
            R = attitude.body_to_world(yaw, pitch, roll)
            placed = self.matrix(node_rotation(R)) @ np.array([1.0, 0.0, 0.0])
            np.testing.assert_allclose(quick_to_enu(placed)[0], R @ np.array([1.0, 0.0, 0.0]),
                                       atol=1e-9)

    def test_the_wings_stay_level_when_the_aircraft_is_level(self):
        placed = self.matrix(node_rotation(attitude.body_to_world(90.0, 0.0, 0.0)))
        right_wing = placed @ np.array([0.0, 1.0, 0.0])      # +Y right in FRD
        self.assertAlmostEqual(float(right_wing[1]), 0.0, places=9)   # no renderer-up component

    def test_it_never_mirrors(self):
        for yaw in range(0, 360, 45):
            self.assertAlmostEqual(
                float(np.linalg.det(self.matrix(node_rotation(attitude.body_to_world(yaw, 5, 5))))),
                1.0, places=9)


class TestLodRing(unittest.TestCase):
    """A coarse terrain tile must keep only the ring outside the finer tile over it, or the
    two surfaces interpenetrate and the ground breaks into blocks."""

    def tile(self, half, n):
        """A flat square grid of triangles, half-width ``half``, ``n`` cells a side."""
        g = np.linspace(-half, half, n + 1)
        X, Y = np.meshgrid(g, g)
        pts = np.c_[X.ravel(), Y.ravel(), np.zeros(X.size)]
        tris = []
        for j in range(n):
            for i in range(n):
                a, b = j * (n + 1) + i, j * (n + 1) + i + 1
                c, d = (j + 1) * (n + 1) + i, (j + 1) * (n + 1) + i + 1
                tris += [[a, b, d], [a, d, c]]
        return pts, np.array(tris)

    def test_the_covered_middle_is_removed(self):
        pts, tris = self.tile(1000.0, 20)
        kept = ring_triangles(pts, tris, (0.0, 0.0), 400.0)
        centroid = pts[kept].mean(axis=1)[:, :2]
        self.assertTrue(np.all(np.abs(centroid).max(axis=1) > 400.0))
        self.assertLess(len(kept), len(tris))

    def test_the_ring_outside_is_kept_whole(self):
        """Nothing outside the hole may be dropped, or a gap opens in the far terrain."""
        pts, tris = self.tile(1000.0, 20)
        centroid = pts[tris].mean(axis=1)[:, :2]
        expected = int((np.abs(centroid).max(axis=1) > 400.0).sum())
        self.assertEqual(len(ring_triangles(pts, tris, (0.0, 0.0), 400.0)), expected)

    def test_the_hole_follows_the_aircraft_not_the_origin(self):
        pts, tris = self.tile(1000.0, 20)
        kept = ring_triangles(pts, tris, (600.0, -600.0), 300.0)
        centroid = pts[kept].mean(axis=1)[:, :2]
        self.assertTrue(np.all(np.abs(centroid - np.array([600.0, -600.0])).max(axis=1) > 300.0))

    def test_a_hole_larger_than_the_tile_removes_everything(self):
        pts, tris = self.tile(1000.0, 10)
        self.assertEqual(len(ring_triangles(pts, tris, (0.0, 0.0), 5000.0)), 0)

    def test_no_hole_keeps_everything(self):
        pts, tris = self.tile(1000.0, 10)
        np.testing.assert_array_equal(ring_triangles(pts, tris, (0.0, 0.0), 0.0), tris)

    def test_an_empty_tile_is_handled(self):
        self.assertEqual(len(ring_triangles(np.empty((0, 3)), np.empty((0, 3), int),
                                            (0.0, 0.0), 100.0)), 0)

    def test_only_the_horizontal_position_decides(self):
        """Terrain elevation must not affect which triangles a tile keeps."""
        pts, tris = self.tile(1000.0, 12)
        flat = ring_triangles(pts, tris, (0.0, 0.0), 400.0)
        hilly = pts.copy()
        hilly[:, 2] = 900.0 * np.sin(hilly[:, 0] / 250.0)
        np.testing.assert_array_equal(ring_triangles(hilly, tris, (0.0, 0.0), 400.0), flat)


class TestCameraBasis(unittest.TestCase):
    def test_columns_are_orthonormal_and_right_handed(self):
        B = look_at_basis((1000.0, -2000.0, 6000.0), (5000.0, 3000.0, 5000.0))
        np.testing.assert_allclose(B.T @ B, np.eye(3), atol=1e-9)
        self.assertAlmostEqual(float(np.linalg.det(B)), 1.0, places=9)

    def test_the_camera_looks_down_its_own_minus_z(self):
        eye, target = np.array([0.0, -10000.0, 5000.0]), np.array([0.0, 0.0, 5000.0])
        B = look_at_basis(eye, target)
        to_target = enu_to_quick(target)[0] - enu_to_quick(eye)[0]
        to_target /= np.linalg.norm(to_target)
        np.testing.assert_allclose(-B[:, 2], to_target, atol=1e-9)

    def test_up_stays_up(self):
        B = look_at_basis((0.0, -10000.0, 5000.0), (0.0, 0.0, 5000.0))
        self.assertGreater(B[1, 1], 0.9)              # the camera's up is mostly world up

    def test_looking_straight_down_still_gives_a_usable_basis(self):
        """Up is parallel to the view direction here, which is where a naive cross product
        collapses to zero and the camera loses all orientation."""
        B = look_at_basis((0.0, 0.0, 9000.0), (0.0, 0.0, 0.0))
        np.testing.assert_allclose(B.T @ B, np.eye(3), atol=1e-9)
        self.assertAlmostEqual(float(np.linalg.det(B)), 1.0, places=9)

    def test_quaternion_reproduces_the_basis(self):
        """Qt is given a quaternion, so a mistake here rotates the whole world quietly."""
        for args in (((1000.0, -2000.0, 6000.0), (5000.0, 3000.0, 5000.0)),
                     ((0.0, 0.0, 9000.0), (0.0, 0.0, 0.0)),
                     ((-52836.0, -26550.0, 5960.0), (-50390.0, -25410.0, 5000.0))):
            B = look_at_basis(*args)
            x, y, z, w = basis_to_quaternion(B)
            self.assertAlmostEqual(x * x + y * y + z * z + w * w, 1.0, places=9)
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            np.testing.assert_allclose(R, B, atol=1e-9)


class TestProjection(unittest.TestCase):
    EYE = np.array([0.0, -10000.0, 5000.0])
    TARGET = np.array([0.0, 0.0, 5000.0])
    W, H, FOV = 1200.0, 800.0, 40.0

    def project(self, pts):
        B = look_at_basis(self.EYE, self.TARGET)
        return project(pts, self.EYE, B, self.W, self.H, self.FOV)

    def test_the_focal_point_lands_in_the_centre_of_the_screen(self):
        sx, sy, front = self.project(self.TARGET[None])
        self.assertAlmostEqual(float(sx[0]), self.W / 2, places=6)
        self.assertAlmostEqual(float(sy[0]), self.H / 2, places=6)
        self.assertTrue(bool(front[0]))

    def test_something_behind_the_camera_is_reported_as_behind(self):
        behind = self.EYE + (self.EYE - self.TARGET)
        self.assertFalse(bool(self.project(behind[None])[2][0]))

    def test_higher_is_drawn_higher_on_screen(self):
        """Screen y grows downwards in Qt, so a point above the focal point must have the
        SMALLER y. Getting this backwards flips every label to the wrong side."""
        sy_low = self.project((self.TARGET + [0, 0, -500])[None])[1][0]
        sy_high = self.project((self.TARGET + [0, 0, 500])[None])[1][0]
        self.assertLess(sy_high, sy_low)

    def test_east_of_the_focal_point_is_drawn_to_the_right(self):
        sx = self.project((self.TARGET + [500, 0, 0])[None])[0][0]
        self.assertGreater(sx, self.W / 2)

    def test_the_vertical_field_of_view_is_exactly_the_angle_asked_for(self):
        """A point at half the vertical FOV above the axis must land on the top edge."""
        d = np.linalg.norm(self.TARGET - self.EYE)
        up = d * np.tan(np.radians(self.FOV / 2))
        sy = self.project((self.TARGET + [0, 0, up])[None])[1][0]
        self.assertAlmostEqual(float(sy), 0.0, places=5)

    def test_the_horizontal_field_of_view_follows_the_aspect_ratio(self):
        d = np.linalg.norm(self.TARGET - self.EYE)
        right = d * np.tan(np.radians(self.FOV / 2)) * (self.W / self.H)
        sx = self.project((self.TARGET + [right, 0, 0])[None])[0][0]
        self.assertAlmostEqual(float(sx), self.W, places=5)

    def test_perspective_divides_by_distance(self):
        """Twice as far along the view axis and twice as far off it lands on the same pixel.
        Without the divide, distant tracks would be drawn as if they were close."""
        near = self.project((self.TARGET + [1000, 0, 0])[None])[0][0] - self.W / 2
        far = self.project((self.TARGET + [2000, 10000, 0])[None])[0][0] - self.W / 2
        self.assertAlmostEqual(float(far), float(near), places=5)

    def test_many_points_at_once_match_one_at_a_time(self):
        rng = np.random.default_rng(9)
        pts = self.TARGET + rng.uniform(-4000, 4000, size=(30, 3))
        sx, sy, front = self.project(pts)
        for i, p in enumerate(pts):
            a, b, f = self.project(p[None])
            self.assertAlmostEqual(float(sx[i]), float(a[0]), places=9)
            self.assertAlmostEqual(float(sy[i]), float(b[0]), places=9)
            self.assertEqual(bool(front[i]), bool(f[0]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
