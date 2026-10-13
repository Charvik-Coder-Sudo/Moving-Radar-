"""The ownship attitude instrument: that it shows the recorded attitude and nothing else.

An instrument that is wrong is worse than no instrument, so what is pinned here is the sense
of each axis - which way the horizon moves for a nose-up pitch, which way it tilts for a
right-wing-down roll - and the separation between what was RECORDED and what was DERIVED.
"""

import math
import os
import sys
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtGui, QtWidgets  # noqa: E402

from widgets.attitude_indicator import (ESTIMATED, GROUND, PX_PER_DEG,  # noqa: E402
                                        RECORDED, SKY, AttitudeIndicator)

_app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class Own:
    """The few fields the instrument reads from an ownship row."""

    def __init__(self, yaw=0.0, pitch=0.0, roll=0.0, course=float("nan")):
        self.yaw_deg, self.pitch_deg, self.roll_deg = yaw, pitch, roll
        self.course_deg = course


def render(own, bank=float("nan"), size=(220, 200)):
    w = AttitudeIndicator()
    w.resize(*size)
    w.set_ownship(own, bank)
    image = QtGui.QImage(w.size(), QtGui.QImage.Format_ARGB32)
    image.fill(0)
    w.render(image)
    return w, image


def is_colour(pixel, colour, tol=16):
    """Whether a painted pixel really is this colour.

    A nearest-of-two test would label the bezel and the heading tape as sky or ground too,
    and the first 'horizon' found would be the top of the widget.
    """
    c = QtGui.QColor(pixel)
    return (c.alpha() > 0 and abs(c.red() - colour.red()) <= tol
            and abs(c.green() - colour.green()) <= tol
            and abs(c.blue() - colour.blue()) <= tol)


def horizon_row(image, x):
    """The y at which sky gives way to ground down a column, or None.

    Tolerant of the horizon line and the pitch ladder drawn across the boundary: it takes the
    first real ground pixel after real sky has been seen.
    """
    seen_sky = False
    for y in range(image.height()):
        pixel = image.pixel(x, y)
        if is_colour(pixel, SKY):
            seen_sky = True
        elif seen_sky and is_colour(pixel, GROUND):
            return y
    return None


class TestSenseOfEachAxis(unittest.TestCase):
    def test_wings_level_puts_the_horizon_across_the_middle(self):
        w, img = render(Own())
        centre = w.rect().center()
        left = horizon_row(img, centre.x() - 40)
        right = horizon_row(img, centre.x() + 40)
        self.assertIsNotNone(left)
        self.assertIsNotNone(right)
        self.assertLess(abs(left - right), 2, "a level aircraft must show a level horizon")

    def test_nose_up_carries_the_horizon_down(self):
        """Pitch is + nose up, and the instrument shows the world: pitching up must push the
        horizon DOWN the dial, not up."""
        _w, level = render(Own(pitch=0.0))
        _w2, up = render(Own(pitch=10.0))
        x = _w.rect().center().x()
        self.assertGreater(horizon_row(up, x), horizon_row(level, x))

    def test_pitch_is_linear_in_the_angle(self):
        """The ladder scales with the dial, so the fixed property is proportionality: twice
        the pitch must be twice the displacement, or the ladder would mislead."""
        w, level = render(Own(pitch=0.0))
        x = w.rect().center().x()
        base = horizon_row(level, x)
        ten = horizon_row(render(Own(pitch=10.0))[1], x) - base
        twenty = horizon_row(render(Own(pitch=20.0))[1], x) - base
        self.assertGreater(ten, 0)
        self.assertAlmostEqual(twenty / ten, 2.0, delta=0.15)

    def test_the_ladder_scale_holds_at_the_reference_size(self):
        from widgets.attitude_indicator import REFERENCE_BALL_PX
        size = (int(REFERENCE_BALL_PX) + 4, int(REFERENCE_BALL_PX) + 32)
        w, level = render(Own(pitch=0.0), size=size)
        x = w.rect().center().x()
        moved = horizon_row(render(Own(pitch=10.0), size=size)[1], x) - horizon_row(level, x)
        self.assertAlmostEqual(moved, 10.0 * PX_PER_DEG, delta=3.0)

    def test_right_wing_down_lifts_the_right_of_the_horizon(self):
        """Roll is + right wing down. Dropping the right wing raises the right-hand side of
        the horizon in the pilot's view, so that is what the instrument must draw."""
        w, img = render(Own(roll=20.0))
        centre = w.rect().center()
        left = horizon_row(img, centre.x() - 40)
        right = horizon_row(img, centre.x() + 40)
        self.assertIsNotNone(left)
        self.assertIsNotNone(right)
        self.assertLess(right, left, "right wing down must raise the horizon's right side")

    def test_the_two_rolls_are_mirror_images(self):
        w, left_roll = render(Own(roll=-20.0))
        _w2, right_roll = render(Own(roll=20.0))
        c = w.rect().center()
        self.assertAlmostEqual(horizon_row(left_roll, c.x() - 40), horizon_row(right_roll, c.x() + 40),
                               delta=2)


class TestRecordedVersusDerived(unittest.TestCase):
    """The rule this instrument exists under: an estimate must never look like a measurement."""

    def test_the_estimated_bank_is_drawn_in_its_own_colour(self):
        def count(image, colour):
            n = 0
            for y in range(image.height()):
                for x in range(image.width()):
                    px = QtGui.QColor(image.pixel(x, y))
                    if px.alpha() and (px.red(), px.green(), px.blue()) == (colour.red(),
                                                                            colour.green(),
                                                                            colour.blue()):
                        n += 1
            return n

        _w, without = render(Own(roll=0.0), bank=float("nan"))
        _w2, with_bank = render(Own(roll=0.0), bank=18.0)
        self.assertEqual(count(without, ESTIMATED), 0,
                         "no estimate offered, so nothing may be drawn as one")
        self.assertGreater(count(with_bank, ESTIMATED), 0)
        self.assertNotEqual((RECORDED.red(), RECORDED.green(), RECORDED.blue()),
                            (ESTIMATED.red(), ESTIMATED.green(), ESTIMATED.blue()))

    def test_an_estimate_equal_to_the_record_is_not_drawn_twice(self):
        w = AttitudeIndicator()
        w.resize(220, 200)
        w.set_ownship(Own(roll=12.0), 12.0)              # same value: one pointer, not two
        img = QtGui.QImage(w.size(), QtGui.QImage.Format_ARGB32)
        img.fill(0)
        w.render(img)        # must not raise, and the readout stays the recorded one


class TestMissingData(unittest.TestCase):
    def test_no_ownship_row_says_so_rather_than_showing_zero(self):
        """Blanking to a level horizon would claim the aircraft is straight and level."""
        w = AttitudeIndicator()
        w.resize(220, 200)
        w.set_ownship(None)
        img = QtGui.QImage(w.size(), QtGui.QImage.Format_ARGB32)
        img.fill(0)
        w.render(img)
        ground = any(is_colour(img.pixel(x, y), GROUND)
                     for y in range(img.height()) for x in range(0, img.width(), 3))
        self.assertFalse(ground, "a blank instrument must not draw a horizon")

    def test_a_non_finite_attitude_does_not_crash_the_panel(self):
        for bad in (float("nan"), float("inf"), None):
            w = AttitudeIndicator()
            w.resize(220, 200)
            w.set_ownship(Own(yaw=bad, pitch=bad, roll=bad))
            img = QtGui.QImage(w.size(), QtGui.QImage.Format_ARGB32)
            img.fill(0)
            w.render(img)


class TestItReadsTheRealExport(unittest.TestCase):
    def test_the_scenario_row_drives_it(self):
        """End to end on the real data: the exported 10 deg angle of attack must show as a
        nose-up horizon, which is the whole point of having the instrument."""
        import paths
        export, _tried = paths.find_export()
        if export is None:
            self.skipTest("no Scenario Export on this machine")
        import pandas as pd
        row = pd.read_csv(export / "3D Trajectory" / "Ownship_Secondary_Radar.csv",
                          nrows=2000).iloc[1500]
        own = Own(yaw=row["Yaw"], pitch=row["Pitch"], roll=row["Roll"])
        self.assertTrue(math.isfinite(own.pitch_deg))
        w, img = render(own)
        x = w.rect().center().x()
        _w2, level = render(Own())
        self.assertGreater(horizon_row(img, x), horizon_row(level, x),
                           "the scenario's nose-up attitude must read as nose-up")


if __name__ == "__main__":
    unittest.main(verbosity=2)
