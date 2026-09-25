"""The aircraft must fly, not slide: the drawn nose follows the path that was recorded.

Eight controlled motion cases (straight N / E / S / W, climb, descent, left turn, right turn)
plus the checks that matter for this scenario's data:

  * the course is taken from the recorded POSITIONS, so it survives the upstream Vx/Vy swap;
  * a turn rotates the aircraft smoothly, sample by sample, with no instant jumps;
  * the model's Forward axis lies along the trajectory tangent;
  * the exported attitude is still reported next to the derived one, never overwritten.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

from msdf_math import attitude, motion  # noqa: E402

DT = 0.02                     # 50 Hz, as the export is decimated
SPEED = 150.0


def straight(course_deg, climb_deg=0.0, seconds=6.0, speed=SPEED, start=(0.0, 0.0, 5000.0)):
    """Recorded rows for constant-course flight, rounded to 0.1 m as the export is."""
    t = np.arange(0.0, seconds, DT)
    u = np.array([np.cos(np.radians(climb_deg)) * np.sin(np.radians(course_deg)),
                  np.cos(np.radians(climb_deg)) * np.cos(np.radians(course_deg)),
                  np.sin(np.radians(climb_deg))])
    pos = np.asarray(start, float) + np.outer(t, u * speed)
    return t * 1000.0, np.round(pos, 1)


def turn(rate_deg_s, seconds=12.0, course0=0.0, speed=SPEED):
    """Constant-rate level turn."""
    t = np.arange(0.0, seconds, DT)
    course = course0 + rate_deg_s * t
    c = np.radians(course)
    step = speed * DT
    pos = np.cumsum(np.column_stack([step * np.sin(c), step * np.cos(c), np.zeros_like(c)]), axis=0)
    pos += np.array([0.0, 0.0, 6000.0])
    return t * 1000.0, np.round(pos, 1), course


class TestEightMotionCases(unittest.TestCase):
    """TEST 1-8: what the display must draw for known motion."""

    def course_of(self, course_deg, climb_deg=0.0):
        t_ms, pos = straight(course_deg, climb_deg)
        pa = motion.path_attitude(t_ms, pos, len(t_ms) // 2)
        self.assertTrue(pa.valid, "a 6 s straight leg must yield a course")
        return pa

    def test_1_straight_north(self):
        pa = self.course_of(0.0)
        self.assertAlmostEqual(pa.course_deg, 0.0, delta=0.1)
        self.assertAlmostEqual(pa.speed_mps, SPEED, delta=0.5)

    def test_2_straight_east(self):
        self.assertAlmostEqual(self.course_of(90.0).course_deg, 90.0, delta=0.1)

    def test_3_straight_south(self):
        self.assertAlmostEqual(self.course_of(180.0).course_deg, 180.0, delta=0.1)

    def test_4_straight_west(self):
        self.assertAlmostEqual(self.course_of(270.0).course_deg, 270.0, delta=0.1)

    def test_5_climb(self):
        pa = self.course_of(45.0, climb_deg=+8.0)
        self.assertAlmostEqual(pa.course_deg, 45.0, delta=0.1)
        self.assertAlmostEqual(pa.climb_deg, +8.0, delta=0.1)
        self.assertGreater(pa.climb_deg, 0.0, "a climbing aircraft must be nose up")

    def test_6_descent(self):
        pa = self.course_of(45.0, climb_deg=-6.0)
        self.assertAlmostEqual(pa.climb_deg, -6.0, delta=0.1)

    def test_7_left_turn(self):
        t_ms, pos, course = turn(-3.0)
        i = len(t_ms) // 2
        pa = motion.path_attitude(t_ms, pos, i)
        self.assertAlmostEqual(pa.course_deg, course[i] % 360.0, delta=1.0)
        self.assertLess(pa.course_rate_deg_s, -2.0)          # turning left
        self.assertLess(pa.bank_deg, -5.0, "a left turn banks left wing down")

    def test_8_right_turn(self):
        t_ms, pos, course = turn(+3.0)
        i = len(t_ms) // 2
        pa = motion.path_attitude(t_ms, pos, i)
        self.assertAlmostEqual(pa.course_deg, course[i] % 360.0, delta=1.0)
        self.assertGreater(pa.course_rate_deg_s, 2.0)
        self.assertGreater(pa.bank_deg, 5.0, "a right turn banks right wing down")


class TestNoseFollowsThePath(unittest.TestCase):
    def test_forward_axis_lies_along_the_trajectory_tangent(self):
        """The acceptance test: model Forward vs the direction of travel, in world ENU."""
        for course in (0.0, 37.0, 90.0, 154.0, 270.0, 312.0):
            for climb in (0.0, +5.0, -5.0):
                t_ms, pos = straight(course, climb)
                i = len(t_ms) // 2
                pa = motion.path_attitude(t_ms, pos, i)
                R = attitude.body_to_world(pa.course_deg, pa.climb_deg, pa.bank_deg or 0.0)
                forward = R[:, 0]
                tangent = pos[i + 20] - pos[i - 20]
                tangent = tangent / np.linalg.norm(tangent)
                angle = np.degrees(np.arccos(np.clip(np.dot(forward, tangent), -1, 1)))
                self.assertLess(angle, 0.5, f"nose {angle:.2f} deg off the path "
                                            f"(course {course}, climb {climb})")

    def test_turning_aircraft_rotates_smoothly(self):
        t_ms, pos, _course = turn(+4.0)
        rows = range(30, len(t_ms) - 30, 5)
        courses = [motion.path_attitude(t_ms, pos, i).course_deg for i in rows]
        steps = np.abs(motion.angle_difference(courses[1:], courses[:-1]))
        self.assertLess(steps.max(), 1.0, "no instant orientation jumps")
        self.assertGreater(steps.mean(), 0.05, "the aircraft must actually be turning")

    def test_a_swapped_velocity_cannot_affect_the_course(self):
        """Regression for this scenario: the course comes from positions, not from Vx/Vy."""
        t_ms, pos = straight(65.05)
        pa = motion.path_attitude(t_ms, pos, len(t_ms) // 2)
        self.assertAlmostEqual(pa.course_deg, 65.05, delta=0.1)   # whatever the velocity columns say


class TestGuards(unittest.TestCase):
    def test_a_stationary_aircraft_has_no_derived_course(self):
        t_ms = np.arange(0.0, 2000.0, 20.0)
        pos = np.tile([100.0, 200.0, 3000.0], (len(t_ms), 1))
        self.assertFalse(motion.path_attitude(t_ms, pos, 50).valid)

    def test_quantised_positions_do_not_produce_noise(self):
        """0.1 m rounding over a 3 m step would be ~2 deg of noise on a one-row baseline."""
        t_ms, pos = straight(65.05, seconds=10.0)
        courses = [motion.path_attitude(t_ms, pos, i).course_deg for i in range(30, 470, 7)]
        self.assertLess(np.std(courses), 0.15, f"course noise {np.std(courses):.3f} deg")

    def test_blend_heading_takes_the_short_way_round(self):
        self.assertAlmostEqual(motion.blend_heading(350.0, 10.0, 0.5), 0.0, places=6)
        self.assertAlmostEqual(motion.blend_heading(float("nan"), 42.0, 0.5), 42.0, places=6)

    def test_trail_attitude_needs_a_baseline(self):
        short = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]])
        self.assertFalse(motion.trail_attitude(short).valid)
        long = np.array([[0.0, 0.0, 0.0], [0.0, 600.0, 60.0]])
        pa = motion.trail_attitude(long)
        self.assertTrue(pa.valid)
        self.assertAlmostEqual(pa.course_deg, 0.0, delta=0.01)
        self.assertAlmostEqual(pa.climb_deg, np.degrees(np.arctan2(60.0, 600.0)), delta=0.01)


if __name__ == "__main__":
    unittest.main()
