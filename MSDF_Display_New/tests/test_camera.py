"""The camera geometry behind the automatic views.

An automatic camera states where it wants to be and the camera eases towards it, so these
are the pieces that decide what the operator sees: how fast a gap closes, where a chase view
sits relative to the aircraft, and how far back the view must be to hold everything that
matters inside the picture. Nothing here touches the scenario data or the radar mathematics.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

from visualization.view_3d import (CAMERA_EASE_TAU, FRAME_FILL, FRAME_RELEVANT_M,  # noqa: E402
                                  RANGE_RINGS_M, VIEW_ANGLE_DEG, bounds_centre, chase_pose,
                                  deadbanded, ease_alpha, framing_distance)


class TestEasing(unittest.TestCase):
    def test_one_time_constant_closes_about_63_percent(self):
        self.assertAlmostEqual(ease_alpha(CAMERA_EASE_TAU, CAMERA_EASE_TAU), 1 - np.exp(-1), places=6)

    def test_a_long_step_arrives_rather_than_overshooting(self):
        # resuming after a pause must not throw the camera past its goal
        for dt in (1.0, 10.0, 600.0):
            a = ease_alpha(dt)
            self.assertGreater(a, 0.95)
            self.assertLessEqual(a, 1.0)

    def test_frame_rate_independence(self):
        """Two small steps must leave the camera where one big step of the same duration does:
        otherwise the camera moves at a different speed at 30 fps than at 60."""
        one = ease_alpha(0.1)
        half = ease_alpha(0.05)
        two_halves = 1.0 - (1.0 - half) ** 2
        self.assertAlmostEqual(one, two_halves, places=12)

    def test_no_time_no_movement_is_not_claimed(self):
        # dt <= 0 (a restarted clock) takes the goal rather than dividing by nothing
        self.assertEqual(ease_alpha(0.0), 1.0)
        self.assertEqual(ease_alpha(-1.0), 1.0)


class TestChasePose(unittest.TestCase):
    def test_camera_sits_behind_and_above(self):
        pos, focal, up = chase_pose((0.0, 0.0, 5000.0), 0.0, 3000.0)   # heading North
        self.assertAlmostEqual(pos[1], -3000.0)         # behind: South of the aircraft
        self.assertAlmostEqual(pos[2], 5000.0 + 0.32 * 3000.0)
        self.assertGreater(focal[1], 0.0)               # looking ahead of it
        self.assertEqual(tuple(up), (0, 0, 1))

    def test_it_follows_the_heading_round_the_compass(self):
        for heading, behind in ((0.0, (0.0, -1.0)), (90.0, (-1.0, 0.0)),
                                (180.0, (0.0, 1.0)), (270.0, (1.0, 0.0))):
            pos, _focal, _up = chase_pose((0.0, 0.0, 0.0), heading, 1000.0)
            np.testing.assert_allclose(pos[:2] / 1000.0, behind, atol=1e-9)

    def test_the_aircraft_stays_well_inside_the_picture(self):
        """The reason the chase view is framed this way: the aircraft must not drift to the
        edge of the screen. Its offset from the view centre stays inside the half-view."""
        half_view = 0.5 * VIEW_ANGLE_DEG
        for heading in range(0, 360, 15):
            p = np.array([1000.0, -2000.0, 6000.0])
            pos, focal, _up = chase_pose(p, float(heading), 3000.0)
            view = np.asarray(focal) - np.asarray(pos)
            to_aircraft = p - np.asarray(pos)
            cos = np.dot(view, to_aircraft) / (np.linalg.norm(view) * np.linalg.norm(to_aircraft))
            self.assertLess(np.degrees(np.arccos(np.clip(cos, -1, 1))), half_view * 0.75)

    def test_distance_scales_the_whole_pose(self):
        near = chase_pose((0, 0, 0), 45.0, 1000.0)[0]
        far = chase_pose((0, 0, 0), 45.0, 4000.0)[0]
        np.testing.assert_allclose(far, np.asarray(near) * 4.0, atol=1e-9)


class TestFraming(unittest.TestCase):
    def test_bounding_box_centre_not_the_mean(self):
        """A cluster plus one distant object: the centre must lie between them, not be
        dragged into the cluster by weight of numbers."""
        pts = [(0, 0, 0)] * 9 + [(10000.0, 0, 0)]
        np.testing.assert_allclose(bounds_centre(pts), (5000.0, 0, 0))

    def test_everything_given_is_inside_the_frame(self):
        rng = np.random.default_rng(7)
        pts = rng.uniform(-40000.0, 40000.0, size=(25, 3))
        centre = bounds_centre(pts)
        d = framing_distance(pts, centre)
        # place a camera at that distance and check the angle to every point
        cam = centre + np.array([0.0, -0.55, 0.80]) / np.linalg.norm([0.0, 0.55, 0.80]) * d
        view = centre - cam
        half = np.radians(0.5 * VIEW_ANGLE_DEG)
        for p in pts:
            to_p = p - cam
            cos = np.dot(view, to_p) / (np.linalg.norm(view) * np.linalg.norm(to_p))
            self.assertLessEqual(np.arccos(np.clip(cos, -1, 1)), half)

    def test_the_frame_is_not_wasted_either(self):
        """FRAME_FILL of the half-view means the objects fill the picture: the furthest one
        must not end up in the middle of an empty screen."""
        pts = np.array([[-20000.0, 0, 0], [20000.0, 0, 0]])
        centre = bounds_centre(pts)
        d = framing_distance(pts, centre)
        self.assertAlmostEqual(20000.0 / (d * np.tan(np.radians(0.5 * VIEW_ANGLE_DEG))),
                               FRAME_FILL, places=6)

    def test_a_wider_spread_pulls_the_camera_back(self):
        close = framing_distance([(0, 0, 0), (5000.0, 0, 0)], (2500.0, 0, 0))
        wide = framing_distance([(0, 0, 0), (50000.0, 0, 0)], (25000.0, 0, 0))
        self.assertAlmostEqual(wide / close, 10.0, places=6)

    def test_framing_reaches_as_far_as_the_range_rings(self):
        """The rings are what distance is read against in the 3D view. If the framing stopped
        short of them, a track inside the outermost ring could be drawn half off the screen."""
        self.assertGreaterEqual(FRAME_RELEVANT_M, max(RANGE_RINGS_M))

    def test_a_single_object_does_not_set_a_distance(self):
        # one point has no extent to frame: the preset's own distance decides
        self.assertEqual(framing_distance([(1000.0, 2000.0, 3000.0)], (1000.0, 2000.0, 3000.0)), 0.0)
        self.assertEqual(framing_distance([], (0, 0, 0)), 0.0)


class TestDeadband(unittest.TestCase):
    def test_the_first_value_is_taken(self):
        self.assertEqual(deadbanded(None, 30000.0), 30000.0)

    def test_small_drift_is_ignored(self):
        # a target edging away must not make the view breathe in and out
        self.assertEqual(deadbanded(30000.0, 31000.0, 0.06), 30000.0)

    def test_a_real_change_is_taken(self):
        self.assertEqual(deadbanded(30000.0, 60000.0, 0.06), 60000.0)
        self.assertEqual(deadbanded(30000.0, 10000.0, 0.06), 10000.0)

    def test_it_settles_instead_of_creeping(self):
        """Repeated small growth must not ratchet the distance upwards a step at a time."""
        held = 30000.0
        for i in range(50):
            held = deadbanded(held, 30000.0 + 20.0 * i, 0.06)
        self.assertEqual(held, 30000.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
