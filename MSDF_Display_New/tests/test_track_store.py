"""TrackStore: packet -> TrackView mapping, associations, stale / hidden lifecycle."""

import math
import sys
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))

from rdp.packet import decode, encode_for_test                     # noqa: E402
from rdp.track_store import ACTIVE, HIDDEN, STALE, TrackStore      # noqa: E402

CFG = {"stale_timeout_s": 3.0, "hide_timeout_s": 10.0, "stream_stale_timeout_s": 5.0, "trail_points": 5}
NAMES = {1: "PRIMARY", 2: "SECONDARY"}
X = (1000.0, 11.0, 0.5, 2000.0, 22.0, 0.6, 0.02, 300.0, 3.0, 0.7)   # [x,vx,ax,y,vy,ay,w,z,vz,az]


def packet(track_id=7, t=100.0, wall=50.0, fused=X, sensors=((1, X), (2, X))):
    raw = encode_for_test(track_id, t, sensors[0][0] if sensors else 0, fused,
                          [s for s, _ in sensors], [st for _, st in sensors])
    return decode(raw, received_wall=wall, sender="127.0.0.1:5000")


def shifted(dx):
    return tuple(v + dx if i in (0, 3, 7) else v for i, v in enumerate(X))


class TestMapping(unittest.TestCase):
    def setUp(self):
        self.store = TrackStore(CFG, NAMES)
        self.store.ingest(packet(fused=X, sensors=((1, shifted(10.0)), (2, shifted(-10.0)))))
        self.views = {tv.key: tv for tv in self.store.views(50.0)}

    def test_keys_and_kinds(self):
        self.assertEqual(set(self.views), {("FUSED", "ST7"), ("PRIMARY", "ST7:S1"), ("SECONDARY", "ST7:S2")})
        self.assertEqual(self.views[("FUSED", "ST7")].kind, "fused")
        self.assertEqual(self.views[("PRIMARY", "ST7:S1")].kind, "sensor")

    def test_state_components(self):
        s = self.views[("FUSED", "ST7")].state
        self.assertEqual((s.x, s.vx, s.ax, s.y, s.vy, s.ay, s.omega, s.z, s.vz, s.az), X)
        self.assertEqual(s.timestamp, 100.0)
        p = self.views[("PRIMARY", "ST7:S1")].state
        self.assertEqual((p.x, p.y, p.z), (1010.0, 2010.0, 310.0))

    def test_fused_drawn_at_its_own_state(self):
        f = self.views[("FUSED", "ST7")]
        self.assertEqual((f.state.x, f.state.y, f.state.z), (X[0], X[3], X[7]))

    def test_associations_only_from_packet(self):
        f = self.views[("FUSED", "ST7")]
        self.assertEqual(f.state.associated_tracks, (("PRIMARY", "ST7:S1"), ("SECONDARY", "ST7:S2")))
        self.assertEqual(f.state.contributing_sensors, ("PRIMARY", "SECONDARY"))
        for key in (("PRIMARY", "ST7:S1"), ("SECONDARY", "ST7:S2")):
            self.assertEqual(self.views[key].fused_into, ("ST7",))
            self.assertEqual(self.views[key].state.associated_tracks, ())

    def test_nothing_invented(self):
        for tv in self.views.values():
            self.assertEqual(tv.state.target_id, "")              # a sensor id is not a target id
            self.assertEqual(tv.state.classification, "")
            self.assertEqual(tv.origin, "rdp")
            self.assertEqual(tv.system_track_id, 7)

    def test_geometry(self):
        store = TrackStore(CFG, NAMES)
        for tid, (x, y, z) in enumerate(((0, 1000, 0), (1000, 0, 0), (0, -1000, 0), (0, 1000, 1000)), 1):
            store.ingest(packet(track_id=tid, fused=(x, 0, 0, y, 0, 0, 0, z, 0, 0), sensors=()))
        v = {tv.system_track_id: tv for tv in store.views(50.0)}
        self.assertAlmostEqual(v[1].bearing_deg, 0.0)              # y = forward
        self.assertAlmostEqual(v[2].bearing_deg, 90.0)             # x = right
        self.assertAlmostEqual(v[3].bearing_deg, 180.0)
        self.assertAlmostEqual(v[4].elevation_deg, 45.0)           # z = up
        self.assertAlmostEqual(v[4].range_m, math.sqrt(2) * 1000.0)

    def test_heading_from_velocity(self):
        store = TrackStore(CFG, NAMES)
        store.ingest(packet(fused=(0, 100.0, 0, 0, 0.0, 0, 0, 0, 0, 0), sensors=()))
        self.assertAlmostEqual(store.views(50.0)[0].heading_deg, 90.0)


class TestLifecycle(unittest.TestCase):
    def test_active_stale_hidden(self):
        store = TrackStore(CFG, NAMES)
        store.ingest(packet(wall=50.0))
        store.tick(52.9)
        self.assertEqual(store.records[7].status, ACTIVE)
        self.assertFalse(any(tv.stale for tv in store.views(52.9)))
        self.assertTrue(store.tick(53.5))
        self.assertEqual(store.records[7].status, STALE)
        views = store.views(53.5)
        self.assertEqual(len(views), 3)
        self.assertTrue(all(tv.stale for tv in views))             # still drawn, faded
        st = store.stats(53.5)
        self.assertEqual((st["active_system_tracks"], st["stale_system_tracks"], st["active_sensor_tracks"]),
                         (0, 1, 0))
        store.tick(60.5)
        self.assertEqual(store.records[7].status, HIDDEN)
        self.assertEqual(store.views(60.5), [])
        self.assertEqual(store.stats(60.5)["stale_system_tracks"], 0)

    def test_update_revives(self):
        store = TrackStore(CFG, NAMES)
        store.ingest(packet(t=100.0, wall=50.0))
        store.tick(55.0)
        store.ingest(packet(t=101.0, wall=55.0))
        store.tick(55.1)
        self.assertEqual(store.records[7].status, ACTIVE)
        self.assertEqual(store.records[7].updates, 2)

    def test_behind_the_stream_is_stale(self):
        """A track not updated for > stream_stale_timeout_s of packet time is stale even if wall-recent."""
        store = TrackStore(CFG, NAMES)
        store.ingest(packet(track_id=1, t=100.0, wall=50.0))
        store.ingest(packet(track_id=2, t=106.0, wall=50.1))       # a burst replay
        store.tick(50.2)
        self.assertEqual(store.records[1].status, STALE)
        self.assertEqual(store.records[2].status, ACTIVE)

    def test_out_of_order_packet_ignored(self):
        store = TrackStore(CFG, NAMES)
        store.ingest(packet(t=100.0, wall=50.0, fused=X))
        self.assertFalse(store.ingest(packet(t=99.0, wall=50.1, fused=shifted(500.0))))
        self.assertEqual(store.records[7].state.time, 100.0)
        self.assertEqual(store.stale_packets, 1)
        self.assertEqual(store.restarts, 0)

    def test_restart_after_stale(self):
        store = TrackStore(CFG, NAMES)
        store.ingest(packet(t=800.0, wall=50.0))
        store.tick(54.0)
        self.assertTrue(store.ingest(packet(t=5.0, wall=54.0, fused=shifted(1.0))))
        self.assertEqual(store.restarts, 1)
        self.assertEqual(store.records[7].state.time, 5.0)
        self.assertEqual(store.records[7].updates, 1)

    def test_trail_capped(self):
        store = TrackStore(CFG, NAMES)
        for i in range(12):
            store.ingest(packet(t=100.0 + i, wall=50.0 + i * 0.1, fused=shifted(float(i))))
        fused = next(tv for tv in store.views(51.2) if tv.kind == "fused")
        self.assertEqual(fused.trail.shape, (5, 3))
        self.assertEqual(fused.trail[-1, 0], X[0] + 11.0)


class TestAccounting(unittest.TestCase):
    def test_unknown_sensor_id(self):
        store = TrackStore(CFG, NAMES)
        store.ingest(packet(sensors=((1, X), (7, X))))
        self.assertEqual(store.stats(50.0)["unknown_sensor_ids"], {7: 1})
        keys = {tv.key for tv in store.views(50.0)}
        self.assertIn(("SENSOR-7", "ST7:S7"), keys)

    def test_rejects_counted(self):
        store = TrackStore(CFG, NAMES)
        store.reject("length", "435 bytes", 50.0, "127.0.0.1:1")
        store.reject("numeric", "time = nan", 50.1)
        store.ingest(packet(wall=50.2))
        st = store.stats(50.5)
        self.assertEqual((st["packets"], st["invalid"]), (3, 2))
        self.assertEqual(st["invalid_by_reason"], {"length": 1, "numeric": 1})
        self.assertAlmostEqual(st["last_packet_age_s"], 0.3)
        self.assertEqual(st["last_packet_time"], 100.0)
        self.assertEqual((st["active_system_tracks"], st["active_sensor_tracks"]), (1, 2))


if __name__ == "__main__":
    unittest.main()
