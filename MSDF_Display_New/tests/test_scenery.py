"""The ground environment must be deterministic, batched, and never part of a calculation.

It is a visualisation layer: the same scenario must build the same world on every run (so a
screenshot is reproducible), it must stay out of the way of the data, and it must not cost more
than it is worth - one mesh per group rather than one actor per building.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

from visualization import scenery  # noqa: E402

LO = np.array([-70000.0, -60000.0, 0.0])
HI = np.array([95000.0, 60000.0, 15000.0])
TRACK = (np.array([-52836.9, -26550.9, 5000.0]), np.array([57961.4, 24989.6, 10000.0]))


def world():
    return scenery.Scenery(lo=LO, hi=HI, track=TRACK)


class TestDeterminism(unittest.TestCase):
    def test_two_worlds_from_the_same_scenario_are_identical(self):
        a, b = world(), world()
        np.testing.assert_allclose(a.airfield.center, b.airfield.center)
        self.assertEqual(a.airfield.heading_deg, b.airfield.heading_deg)
        for name in ("settlements", "masts", "airfield_buildings"):
            ma, mb = getattr(a, name)(), getattr(b, name)()
            self.assertEqual(ma.n_points, mb.n_points, name)
            np.testing.assert_allclose(ma.points, mb.points, err_msg=name)

    def test_building_a_group_twice_gives_the_same_world(self):
        """Each group has its own stream, so build order cannot change what is drawn."""
        w = world()
        for name in ("settlements", "woodland", "masts"):
            first, second = getattr(w, name)(), getattr(w, name)()
            np.testing.assert_allclose(first.points, second.points, err_msg=name)
        other = world()
        other.woodland()                               # a different order, same result
        np.testing.assert_allclose(other.settlements().points, w.settlements().points)

    def test_the_airfield_sits_on_the_ownship_ground_track(self):
        w = world()
        start, end = TRACK[0][:2], TRACK[1][:2]
        d = end - start
        t = np.dot(w.airfield.center - start, d) / np.dot(d, d)
        self.assertTrue(0.05 < t < 0.95, "the airfield must lie along the flown leg")
        perpendicular = np.linalg.norm(w.airfield.center - (start + t * d))
        self.assertLess(perpendicular, 12000.0, "and close beside it")

    def test_the_runway_is_named_for_its_heading(self):
        w = world()
        a, b = w.airfield.designators
        self.assertEqual(int(a), round(w.airfield.heading_deg / 10) % 36 or 36)
        self.assertEqual((int(a) - int(b)) % 36, 18, "reciprocal runway numbers")


class TestTerrain(unittest.TestCase):
    def test_terrain_is_flat_at_the_airfield(self):
        w = world()
        c = w.airfield.center
        heights = [w.height(c[0] + dx, c[1] + dy)
                   for dx in (-800, 0, 800) for dy in (-800, 0, 800)]
        self.assertLess(np.ptp(heights), 1.0, "the runway must not sit on a slope")
        self.assertAlmostEqual(float(np.mean(heights)), w.airfield.elevation_m, delta=1.0)

    def test_height_is_finite_and_vectorised(self):
        w = world()
        X, Y = np.meshgrid(np.linspace(LO[0], HI[0], 40), np.linspace(LO[1], HI[1], 40))
        Z = w.height(X, Y)
        self.assertEqual(Z.shape, X.shape)
        self.assertTrue(np.all(np.isfinite(Z)))
        self.assertGreater(Z.max(), 200.0, "there must be relief to see")

    def test_land_cover_is_in_range_and_water_is_water(self):
        w = world()
        X, Y = np.meshgrid(np.linspace(LO[0], HI[0], 30), np.linspace(LO[1], HI[1], 30))
        Z = w.height(X, Y)
        rgb = scenery.land_cover(X.ravel(), Y.ravel(), Z.ravel())
        self.assertEqual(rgb.dtype, np.uint8)
        self.assertEqual(len(rgb), X.size)
        low = scenery.land_cover(np.array([0.0]), np.array([0.0]),
                                 np.array([scenery.WATER_LEVEL_M - 1.0]))
        np.testing.assert_array_equal(low[0], scenery._hex(scenery.WATER))

    def test_nothing_in_the_scenery_reaches_a_classification_colour(self):
        """Scenery must never be mistaken for a Friendly / Hostile / Neutral / Unknown symbol."""
        from visualization import style
        class_rgb = [np.array(style.hex_to_rgb(style.class_color(c)), float)
                     for c in ("Friend", "Foe", "Neutral", "Unknown")]
        for name in ("WATER", "LOWLAND", "FIELD", "UPLAND", "ROCK", "FOREST", "BUILDING",
                     "HANGAR", "TOWER", "ROAD", "TREE", "RUNWAY"):
            c = np.array(scenery._hex(getattr(scenery, name)), float)
            for other in class_rgb:
                self.assertGreater(np.linalg.norm(c - other), 90.0,
                                   f"{name} is too close to a classification colour")


class TestBatching(unittest.TestCase):
    def test_every_group_is_a_single_mesh(self):
        w = world()
        for name in ("airfield_surfaces", "airfield_buildings", "settlements", "masts", "woodland"):
            mesh = getattr(w, name)()
            self.assertGreater(mesh.n_points, 0, name)
            self.assertIn("rgb", mesh.point_data, name)
            self.assertEqual(len(mesh.point_data["rgb"]), mesh.n_points, name)

    def test_the_world_stays_within_a_sensible_budget(self):
        """Ground context must not cost more than the data it frames."""
        w = world()
        total = sum(getattr(w, n)().n_points for n in
                    ("airfield_surfaces", "airfield_buildings", "settlements", "masts", "woodland"))
        self.assertLess(total, 400_000, f"{total} scenery points is too many")

    def test_buildings_are_not_built_on_water_or_on_the_runway(self):
        w = world()
        pts = w.settlements().points
        self.assertGreater(len(pts), 0)
        heights = w.height(pts[:, 0], pts[:, 1])
        self.assertTrue(np.all(heights > scenery.WATER_LEVEL_M), "a town in a lake")
        d = np.hypot(pts[:, 0] - w.airfield.center[0], pts[:, 1] - w.airfield.center[1])
        self.assertTrue(np.all(d > 2000.0), "a town on the runway")


if __name__ == "__main__":
    unittest.main()
