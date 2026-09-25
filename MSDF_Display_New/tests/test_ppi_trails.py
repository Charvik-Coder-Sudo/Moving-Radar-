"""PPI trails must be the object's path, not the history of the relative geometry.

The marker shows where something is now relative to the aircraft. If every trail point were
converted with the ownship pose of its own moment, the trail would be a different curve that
slides backwards under the marker as the ownship flies. These tests pin the drawn trail to the
recorded path: same shape, head on the marker, one ownship conversion, no mixing of targets.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

from models.ownship_state import OwnshipState  # noqa: E402
from models.track_state import KIND_SENSOR, KIND_TARGET, TrackState  # noqa: E402
from msdf_math import attitude, coordinates  # noqa: E402
from processing.frame import TrackView  # noqa: E402

KM = 1e-3


def ownship(x, y, z=6000.0, yaw=30.0):
    return OwnshipState(timestamp=0.0, track_id="OWNSHIP", x=x, y=y, z=z,
                        yaw_deg=yaw, pitch_deg=0.0, roll_deg=0.0, vx=0.0, vy=0.0, vz=0.0,
                        ax=np.nan, ay=np.nan, az=np.nan, omega=np.nan,
                        heading_valid=True, roll_valid=True,
                        R_WB=attitude.body_to_world(yaw, 0.0, 0.0))


def straight_path(n=40, start=(20000.0, 5000.0, 9000.0), step=(300.0, 120.0, 0.0)):
    return np.asarray(start) + np.outer(np.arange(n), step)


def view(kind, world, trail, own):
    st = TrackState(timestamp=0.0, track_id="T1", target_id="1", x=0, y=0, z=0, vx=0, vy=0, vz=0,
                    omega=0, ax=0, ay=0, az=0, classification="", source="TRUTH", status="ok")
    tv = TrackView(key=(kind, "T1"), kind=kind, state=st, age_s=0.0, heading_deg=0.0,
                   trail=trail, world=world, world_trail=trail)
    tv.body_xyz = coordinates.world_to_frame(own.position, own.R_WB, world[None])[0]
    return tv


class _Stub:
    """The pieces of AircraftPPI that _trail_xy touches, with its real helpers bound."""

    def __init__(self, own, layers=None, trail_seconds=0.0):
        from types import SimpleNamespace
        from visualization.aircraft_ppi import AircraftPPI
        self._frame = SimpleNamespace(ownship=own)
        self.ctrl = SimpleNamespace(layers=layers or {}, settings={"trail_seconds": trail_seconds})
        self._recent = AircraftPPI._recent.__get__(self)


def trail_xy(tv, own, layers=None, trail_seconds=0.0):
    from visualization.aircraft_ppi import AircraftPPI
    return AircraftPPI._trail_xy(_Stub(own, layers, trail_seconds), tv)


class TestTruthTrajectory(unittest.TestCase):
    def setUp(self):
        self.own = ownship(0.0, 0.0)
        self.path = straight_path()
        self.tv = view(KIND_TARGET, self.path[-1], self.path, self.own)

    def test_the_truth_trajectory_is_drawn_at_all(self):
        xy = trail_xy(self.tv, self.own)
        self.assertIsNotNone(xy, "the PPI drew no truth trajectory")
        self.assertEqual(len(xy), len(self.path))

    def test_the_head_of_the_trail_sits_on_the_marker(self):
        xy = trail_xy(self.tv, self.own)
        marker = (self.tv.body_xyz[1] * KM, self.tv.body_xyz[0] * KM)
        np.testing.assert_allclose(xy[-1], marker, atol=1e-9)

    def test_the_drawn_path_has_the_shape_of_the_recorded_path(self):
        xy = trail_xy(self.tv, self.own) / KM
        drawn = np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))
        real = np.sum(np.linalg.norm(np.diff(self.path[:, :2], axis=0), axis=1))
        self.assertAlmostEqual(drawn / real, 1.0, places=6)

    def test_the_trajectory_does_not_slide_when_the_ownship_moves(self):
        """The same recorded point must stay on the same part of the path, whatever the ownship
        does: in the aircraft frame it rotates and translates with the aircraft, together with
        the target, so target-to-trail geometry is unchanged."""
        before = trail_xy(self.tv, self.own)
        moved = ownship(9000.0, 4000.0, yaw=30.0)                 # 10 km along, same attitude
        tv2 = view(KIND_TARGET, self.path[-1], self.path, moved)
        after = trail_xy(tv2, moved)
        rel_before = before - before[-1]                          # relative to the target marker
        rel_after = after - after[-1]
        np.testing.assert_allclose(rel_after, rel_before, atol=1e-9)

    def test_a_turning_ownship_rotates_the_whole_path_rigidly(self):
        turned = ownship(0.0, 0.0, yaw=75.0)
        tv2 = view(KIND_TARGET, self.path[-1], self.path, turned)
        a = trail_xy(self.tv, self.own) / KM
        b = trail_xy(tv2, turned) / KM
        da = np.linalg.norm(np.diff(a, axis=0), axis=1)           # same internal distances
        db = np.linalg.norm(np.diff(b, axis=0), axis=1)
        np.testing.assert_allclose(da, db, atol=1e-6)

    def test_the_layer_switch_is_honoured(self):
        self.assertIsNone(trail_xy(self.tv, self.own, {"show_target_history": False}))
        self.assertIsNotNone(trail_xy(self.tv, self.own, {"show_target_trails": False}))


class TestTrackTrajectories(unittest.TestCase):
    def test_sensor_and_fused_trails_use_the_same_conversion(self):
        own = ownship(1000.0, -2000.0, yaw=200.0)
        path = straight_path(n=12)
        tv = view(KIND_SENSOR, path[-1], path, own)
        xy = trail_xy(tv, own)
        marker = (tv.body_xyz[1] * KM, tv.body_xyz[0] * KM)
        np.testing.assert_allclose(xy[-1], marker, atol=1e-9)

    def test_a_track_without_a_world_path_falls_back_to_its_aircraft_frame_history(self):
        own = ownship(0.0, 0.0)
        path = straight_path(n=5)
        tv = view(KIND_SENSOR, path[-1], path, own)
        tv.world_trail = None
        tv.body_trail = np.array([[100.0, 200.0, -50.0], [110.0, 210.0, -50.0]])
        xy = trail_xy(tv, own)
        np.testing.assert_allclose(xy, tv.body_trail[:, [1, 0]] * KM)

    def test_a_target_without_a_world_path_draws_nothing(self):
        own = ownship(0.0, 0.0)
        path = straight_path(n=5)
        tv = view(KIND_TARGET, path[-1], path, own)
        tv.world_trail = None
        self.assertIsNone(trail_xy(tv, own))


class TestHistoryWindow(unittest.TestCase):
    """A scope centred on a moving aircraft shows a tail, not the entire recorded flight."""

    def test_only_the_recent_part_of_a_truth_trajectory_is_drawn(self):
        own = ownship(0.0, 0.0)
        path = straight_path(n=300)
        tv = view(KIND_TARGET, path[-1], path, own)
        tv.meta = {"trail_t": np.arange(300.0)}                   # one row per second
        full = trail_xy(tv, own, trail_seconds=0.0)
        windowed = trail_xy(tv, own, trail_seconds=60.0)
        self.assertEqual(len(full), 300)
        self.assertEqual(len(windowed), 61)
        np.testing.assert_allclose(windowed[-1], full[-1])        # the head is still the marker

    def test_track_history_is_not_windowed(self):
        own = ownship(0.0, 0.0)
        path = straight_path(n=40)
        tv = view(KIND_SENSOR, path[-1], path, own)               # no trail_t: bounded by the store
        self.assertEqual(len(trail_xy(tv, own, trail_seconds=5.0)), 40)


class TestAnchoring(unittest.TestCase):
    """An RDP track's marker comes from its own packet; the trail must end exactly on it."""

    def test_the_trail_hangs_on_the_marker_even_when_the_clock_leads_the_packet(self):
        own = ownship(0.0, 0.0)
        path = straight_path(n=20)
        tv = view(KIND_SENSOR, path[-1], path, own)
        # the packet was placed 2 s ago: the marker keeps that placement, the pose has moved on
        tv.body_xyz = tv.body_xyz + np.array([300.0, -120.0, 5.0])
        xy = trail_xy(tv, own)
        marker = (tv.body_xyz[1] * KM, tv.body_xyz[0] * KM)
        np.testing.assert_allclose(xy[-1], marker, atol=1e-9)

    def test_anchoring_does_not_deform_the_path(self):
        own = ownship(0.0, 0.0)
        path = straight_path(n=20)
        tv = view(KIND_SENSOR, path[-1], path, own)
        tv.body_xyz = tv.body_xyz + np.array([300.0, -120.0, 0.0])
        xy = trail_xy(tv, own) / KM
        drawn = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        real = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
        np.testing.assert_allclose(drawn, real, atol=1e-6)


class TestNoMixing(unittest.TestCase):
    def test_each_object_keeps_its_own_points(self):
        own = ownship(0.0, 0.0)
        a = view(KIND_TARGET, straight_path()[-1], straight_path(), own)
        b_path = straight_path(start=(-30000.0, 12000.0, 7000.0), step=(-250.0, 90.0, 0.0))
        b = view(KIND_TARGET, b_path[-1], b_path, own)
        b.key = (KIND_TARGET, "T2")
        xa, xb = trail_xy(a, own), trail_xy(b, own)
        self.assertEqual(len(xa), len(a.world_trail))
        self.assertEqual(len(xb), len(b.world_trail))
        self.assertGreater(np.linalg.norm(xa[-1] - xb[-1]), 1.0, "two targets share a trail head")


if __name__ == "__main__":
    unittest.main()
