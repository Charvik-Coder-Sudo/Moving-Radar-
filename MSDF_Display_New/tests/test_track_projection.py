"""Placement of RDP tracks in world ENU and in the aircraft frame (processing.track_projection).

The contract these tests pin down, which was verified in the tracker's source and against the
exported truth before it was written here:

    the RDP track state is a WORLD ENU position, not a radar-relative one.

`MultiTrack/imm3dc.py::Plot.calculateCartesian` builds a plot as
``x = r cos(phi) sin(theta)``, ``y = r cos(phi) cos(theta)``, ``z = r sin(phi)`` with theta
measured from North - x East, y North, z Up - and the sensor CSV's ``r/theta/phi`` are the
target's spherical coordinates about the **world origin**, so that Cartesian position is the
target's world position. `imm3dc` then feeds ``plot.pos`` straight to the IMM filters, so the
track state and the fused ``Xfused`` stay in that frame.

So the display takes the state as sent for the 3D and 2D views, and derives the PPI's
aircraft-relative position by subtracting the ownship and rotating into the body frame.
"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))

from models.sensor_state import SensorConfig          # noqa: E402
from msdf_math import attitude, coordinates          # noqa: E402
from processing.track_projection import TrackProjector, rdp_to_world  # noqa: E402
from rdp.packet import decode, encode_for_test        # noqa: E402
from rdp.track_store import TrackStore               # noqa: E402

NAMES = {1: "PRIMARY", 2: "SECONDARY"}


def sensor(sid, mount_xyz, mount_ypr, kind):
    return SensorConfig.from_dict(dict(
        id=sid, name=sid, type=kind, r_max_m=150000.0, az_coverage_deg=120.0, el_coverage_deg=60.0,
        az_beamwidth_deg=2.0, el_beamwidth_deg=2.0, az_overlap_deg=0.0, el_overlap_deg=0.0,
        mount_xyz_frd=mount_xyz, mount_ypr_deg=mount_ypr, scan_time_s=0.9, start_time_s=0.0,
        stop_time_s=None))


SENSORS = [sensor("PRIMARY", (0.0, 0.0, 0.0), (-5.0, -5.0, -5.0), "primary"),
           sensor("SECONDARY", (0.0, -0.75, 0.0), (5.0, 5.0, 5.0), "secondary")]
P_WA = np.array([-52836.9, -26550.9, 5000.0])            # aircraft position
R_WB = attitude.body_to_world(19.9, 8.7, -6.2)            # aircraft attitude


class FakeOwnship:
    position = P_WA
    R_WB_drawn = R_WB


class FakeProcessor:
    """The ownship pose lookup - the only thing the projector needs from the export now."""

    def __init__(self, available: bool = True):
        self.available = available

    def ownship_at(self, t_ms):
        return (FakeOwnship(), 0) if self.available else (None, -1)


def rdp_state_of(world_point):
    """What the tracker carries for a point, through MultiTrack's own equations.

    The sensor CSV stores the target's range / azimuth / elevation about the WORLD ORIGIN,
    and `Plot.calculateCartesian` converts them back - so this is a round trip that must
    return the world point itself. Going the long way round is the point: it exercises the
    same spherical convention the producer uses rather than asserting the identity directly.
    """
    x, y, z = (float(v) for v in world_point)
    r = math.sqrt(x * x + y * y + z * z)
    theta = math.atan2(x, y)                       # from North, towards East
    phi = math.asin(z / r) if r else 0.0
    return (r * math.cos(phi) * math.sin(theta),
            r * math.cos(phi) * math.cos(theta),
            r * math.sin(phi))


def views_for(points_by_sensor, t=100.0, latest=1):
    store = TrackStore({}, NAMES)
    zero = (0.0,) * 10
    ids, states = [], []
    for sid, pt in points_by_sensor.items():
        x, y, z = rdp_state_of(pt)
        ids.append(sid)
        states.append((x, 0.0, 0.0, y, 0.0, 0.0, 0.0, z, 0.0, 0.0))
    fused = states[ids.index(latest)] if latest in ids else zero
    store.ingest(decode(encode_for_test(7, t, latest, fused, ids, states), received_wall=1.0))
    return store.views(1.0)


class TestFrames(unittest.TestCase):
    def test_the_producers_spherical_round_trip_is_the_world_point(self):
        """MultiTrack's r/theta/phi are about the world origin, so its own Cartesian
        conversion returns the world position. If this ever stops holding, the premise
        underneath every placement below has changed."""
        for p in (np.array([-20000.0, 15000.0, 7400.0]),
                  np.array([30000.0, -12000.0, -2500.0]),
                  np.array([0.0, 41000.0, 9000.0])):
            np.testing.assert_allclose(rdp_state_of(p), p, atol=1e-6)

    def test_rdp_state_is_taken_as_world_enu(self):
        p = np.array([-20000.0, 15000.0, 7400.0])
        np.testing.assert_allclose(rdp_to_world(*rdp_state_of(p)), p, atol=1e-6)

    def test_world_is_the_state_as_sent(self):
        target = np.array([-20000.0, 15000.0, 7400.0])
        views = views_for({1: target, 2: target})
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
        for tv in views:
            np.testing.assert_allclose(tv.world, target, atol=1e-6, err_msg=str(tv.key))

    def test_world_does_not_depend_on_which_radar_reported_it(self):
        """Two sensors with different mountings reporting the same target must put it in the
        same place - a world position is not a function of who saw it."""
        target = np.array([12000.0, -31000.0, 6000.0])
        views = views_for({1: target, 2: target})
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
        worlds = [tv.world for tv in views if tv.world is not None]
        self.assertGreaterEqual(len(worlds), 2)
        for w in worlds[1:]:
            np.testing.assert_allclose(w, worlds[0], atol=1e-9)

    def test_body_is_the_ownship_relative_position(self):
        target = np.array([-10000.0, 40000.0, 9000.0])
        views = views_for({1: target, 2: target})
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
        expected = coordinates.world_to_frame(P_WA, R_WB, target[None])[0]
        for tv in views:
            np.testing.assert_allclose(tv.body_xyz, expected, atol=1e-6, err_msg=str(tv.key))

    def test_the_ppi_range_is_the_distance_to_the_ownship(self):
        """What the operator reads off the scope has to be the real separation."""
        target = np.array([-10000.0, 40000.0, 9000.0])
        views = views_for({1: target})
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
        for tv in views:
            self.assertAlmostEqual(float(np.linalg.norm(tv.body_xyz)),
                                   float(np.linalg.norm(target - P_WA)), places=6)

    def test_without_an_ownship_pose_there_is_no_aircraft_frame_position(self):
        target = np.array([-10000.0, 40000.0, 9000.0])
        views = views_for({1: target})
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor(available=False))
        for tv in views:
            np.testing.assert_allclose(tv.world, target, atol=1e-6)   # world still known
            self.assertIsNone(tv.body_xyz)
            self.assertIn("ownship pose is unknown", tv.placement)

    def test_an_unknown_sensor_id_still_has_a_world_position(self):
        """The position came from the tracker, not from the radar's mounting, so a sensorId
        the display does not recognise must not lose the track."""
        store = TrackStore({}, NAMES)
        s = (1000.0, 0, 0, 2000.0, 0, 0, 0, 300.0, 0, 0)
        store.ingest(decode(encode_for_test(3, 10.0, 9, s, [9], [s]), received_wall=1.0))
        views = store.views(1.0)
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
        for tv in views:
            np.testing.assert_allclose(tv.world, (1000.0, 2000.0, 300.0), atol=1e-9)

    def test_fused_uses_latest_sensor(self):
        views = views_for({2: np.array([0.0, 30000.0, 8000.0])}, latest=2)
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
        fused = next(tv for tv in views if tv.kind == "fused")
        self.assertEqual(fused.latest_sensor, 2)


class TestCrossView(unittest.TestCase):
    """The same physical track in the 3D view, the 2D view and the PPI.

    3D and 2D both draw ``tv.world``; the PPI draws ``tv.body_xyz``. They agree only while
    the body position really is the ownship-relative form of the same world position, so
    that relationship is asserted rather than assumed - it is what stops the three views
    drifting apart again.
    """

    def setUp(self):
        self.target = np.array([-10000.0, 40000.0, 9000.0])
        views = views_for({1: self.target})
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
        self.tv = next(v for v in views if v.kind == "sensor")

    def test_3d_and_2d_read_the_same_world_position(self):
        np.testing.assert_allclose(self.tv.world, self.target, atol=1e-6)
        np.testing.assert_allclose(self.tv.world[:2], self.target[:2], atol=1e-6)  # 2D: E, N

    def test_the_ppi_is_that_same_point_seen_from_the_ownship(self):
        back = coordinates.frame_to_world(P_WA, R_WB, self.tv.body_xyz[None])[0]
        np.testing.assert_allclose(back, self.tv.world, atol=1e-6)

    def test_the_ppi_range_is_the_true_distance(self):
        rng, _az, _el = coordinates.radar_measurements(self.tv.body_xyz[None])
        self.assertAlmostEqual(float(rng[0]), float(np.linalg.norm(self.target - P_WA)), places=6)

    def test_the_ppi_bearing_convention(self):
        """0 deg is the nose and +90 deg the right wing, measured in the BODY frame.

        Checked by putting a track along each body axis in turn rather than by recomputing
        atan2 - a horizontal 'bearing' is not the same angle once the aircraft is pitched
        and rolled, and conflating the two is exactly the sort of error this file exists
        to catch.
        """
        # straight up is left out of the azimuth check: at 90 deg elevation the azimuth is
        # degenerate (atan2 of two zeros), so only its elevation means anything
        cases = ((np.array([1.0, 0.0, 0.0]), 0.0, 0.0, True),      # nose
                 (np.array([0.0, 1.0, 0.0]), 90.0, 0.0, True),     # right wing
                 (np.array([-1.0, 0.0, 0.0]), 180.0, 0.0, True),   # tail
                 (np.array([0.0, 0.0, -1.0]), None, 90.0, False))  # straight up
        for axis, want_az, want_el, check_az in cases:
            with self.subTest(axis=tuple(axis)):
                world = P_WA + R_WB @ (axis * 25000.0)
                views = views_for({1: world})
                TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
                tv = next(v for v in views if v.kind == "sensor")
                rng, az, el = coordinates.radar_measurements(tv.body_xyz[None])
                self.assertAlmostEqual(float(rng[0]), 25000.0, places=3)
                self.assertAlmostEqual(float(el[0]), want_el, places=3)
                if check_az:
                    self.assertAlmostEqual(float(abs(coordinates.wrap180(az[0]))), want_az,
                                           places=3)


class TestHistory(unittest.TestCase):
    def test_history_grows_once_per_packet_and_resets_on_restart(self):
        proj = TrackProjector(NAMES, SENSORS, trail_points=3)
        store = TrackStore({"trail_points": 3}, NAMES)
        for k, t in enumerate((10.0, 10.0, 11.0, 12.0, 13.0)):
            s = (1000.0 + k, 0, 0, 2000.0, 0, 0, 0, 300.0, 0, 0)
            store.ingest(decode(encode_for_test(5, t, 1, s, [1], [s]), received_wall=1.0 + k))
            views = store.views(1.0 + k)
            proj.project(views, FakeProcessor())
        tv = next(v for v in views if v.kind == "sensor")
        self.assertEqual(len(tv.world_trail), 3)                   # bounded; t=10 counted once
        store.tick(20.0)                                           # stale -> a restart is accepted
        s = (1.0, 0, 0, 2.0, 0, 0, 0, 3.0, 0, 0)
        store.ingest(decode(encode_for_test(5, 1.0, 1, s, [1], [s]), received_wall=20.0))
        views = store.views(20.0)
        proj.project(views, FakeProcessor())
        tv = next(v for v in views if v.kind == "sensor")
        self.assertEqual(len(tv.world_trail), 1)                   # history restarted with the track

    def test_hidden_tracks_forgotten(self):
        proj = TrackProjector(NAMES, SENSORS)
        views = views_for({1: np.array([1.0, 2.0, 3.0])})
        proj.project(views, FakeProcessor())
        self.assertTrue(proj._hist)
        proj.project([], FakeProcessor())
        self.assertFalse(proj._hist)


if __name__ == "__main__":
    unittest.main()
