"""Placement of RDP tracks in world ENU and in the aircraft frame (processing.track_projection).

Round trip through the project's own conventions: a world point is measured with
Measurements_Geometry's definitions (coordinates.radar_measurements), turned into the
RDP's Cartesian state with MultiTrack's Plot.calculateCartesian equations, and must be
placed back exactly where it was.
"""

import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))

from models.sensor_state import SensorConfig          # noqa: E402
from msdf_math import attitude, coordinates          # noqa: E402
from processing.track_projection import TrackProjector, rdp_to_frd  # noqa: E402
from rdp.packet import decode, encode_for_test        # noqa: E402
from rdp.track_store import TrackStore               # noqa: E402

NAMES = {1: "PRIMARY", 2: "SECONDARY"}


def sensor(sid, mount_xyz, mount_ypr, kind):
    return SensorConfig.from_dict(dict(
        id=sid, name=sid, type=kind, r_max_m=150000.0, az_coverage_deg=120.0, el_coverage_deg=60.0,
        az_beamwidth_deg=2.0, el_beamwidth_deg=2.0, az_overlap_deg=0.0, el_overlap_deg=0.0,
        mount_xyz_frd=mount_xyz, mount_ypr_deg=mount_ypr, scan_time_s=0.9, start_time_s=0.0, stop_time_s=None))


SENSORS = [sensor("PRIMARY", (0.0, 0.0, 0.0), (-5.0, -5.0, -5.0), "primary"),
           sensor("SECONDARY", (0.0, -0.75, 0.0), (5.0, 5.0, 5.0), "secondary")]
P_WA = np.array([-52836.9, -26550.9, 5000.0])            # aircraft position
R_WB = attitude.body_to_world(19.9, 8.7, -6.2)            # aircraft attitude


def radar_pose(cfg):
    return P_WA + R_WB @ np.asarray(cfg.mount_xyz_frd), R_WB @ cfg.R_BR


class FakeProcessor:
    """Exported radar pose lookup (the only thing the projector needs from the export)."""

    def radar_pose(self, num, t_ms):
        return radar_pose(SENSORS[num - 1]) if num in (1, 2) else None


def rdp_state_of(world_point, cfg):
    """What the RDP would carry for a point seen by this radar (MultiTrack's own equations)."""
    P_WR, R_WR = radar_pose(cfg)
    L = coordinates.world_to_frame(P_WR, R_WR, world_point[None])[0]
    rng, az, el = coordinates.radar_measurements(L)
    r, th, ph = rng[0], math.radians(az[0]), math.radians(el[0])
    return r * math.cos(ph) * math.sin(th), r * math.cos(ph) * math.cos(th), r * math.sin(ph)


def views_for(points_by_sensor, t=100.0, latest=1):
    store = TrackStore({}, NAMES)
    zero = (0.0,) * 10
    ids, states = [], []
    for sid, pt in points_by_sensor.items():
        x, y, z = rdp_state_of(pt, SENSORS[sid - 1])
        ids.append(sid)
        states.append((x, 0.0, 0.0, y, 0.0, 0.0, 0.0, z, 0.0, 0.0))
    fused = states[ids.index(latest)] if latest in ids else zero
    store.ingest(decode(encode_for_test(7, t, latest, fused, ids, states), received_wall=1.0))
    return store.views(1.0)


class TestFrames(unittest.TestCase):
    def test_rdp_state_is_radar_frd(self):
        L = np.array([30000.0, -12000.0, -2500.0])          # forward, left, up
        rng, az, el = coordinates.radar_measurements(L)
        r, th, ph = rng[0], math.radians(az[0]), math.radians(el[0])
        xyz = (r * math.cos(ph) * math.sin(th), r * math.cos(ph) * math.cos(th), r * math.sin(ph))
        np.testing.assert_allclose(rdp_to_frd(*xyz), L, atol=1e-6)

    def test_world_round_trip(self):
        target = np.array([-20000.0, 15000.0, 7400.0])
        views = views_for({1: target, 2: target})
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
        for tv in views:
            np.testing.assert_allclose(tv.world, target, atol=1e-6, err_msg=str(tv.key))

    def test_body_placement_matches_aircraft_attitude(self):
        target = np.array([-10000.0, 40000.0, 9000.0])
        views = views_for({1: target, 2: target})
        TrackProjector(NAMES, SENSORS).project(views, None)          # no export: body only
        expected = coordinates.world_to_frame(P_WA, R_WB, target[None])[0]
        for tv in views:
            np.testing.assert_allclose(tv.body_xyz, expected, atol=1e-6, err_msg=str(tv.key))
            self.assertIsNone(tv.world)                           # no export pose -> no world position

    def test_fused_uses_latest_sensor(self):
        views = views_for({2: np.array([0.0, 30000.0, 8000.0])}, latest=2)
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
        fused = next(tv for tv in views if tv.kind == "fused")
        self.assertEqual(fused.latest_sensor, 2)
        self.assertIn("latestSensor SECONDARY", fused.placement)

    def test_unknown_sensor_not_placed(self):
        store = TrackStore({}, NAMES)
        s = (1000.0, 0, 0, 2000.0, 0, 0, 0, 300.0, 0, 0)
        store.ingest(decode(encode_for_test(3, 10.0, 9, s, [9], [s]), received_wall=1.0))
        views = store.views(1.0)
        TrackProjector(NAMES, SENSORS).project(views, FakeProcessor())
        for tv in views:
            self.assertIsNone(tv.world)
            self.assertIsNone(tv.body_xyz)
            self.assertIn("not placed", tv.placement)


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
