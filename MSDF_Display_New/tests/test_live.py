"""Live clock (follows packet time), truth targets from the export model, aircraft ambience."""

import json
import os
import sys
import time
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtCore, QtWidgets  # noqa: E402

from data_loader.scenario_export import ScenarioData, TargetTimeline  # noqa: E402
from processing.scenario_processor import ScenarioProcessor  # noqa: E402
from visualization import view_controller as vc  # noqa: E402

QAPP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
CONFIG = json.loads((APP / "config" / "display_config.json").read_text(encoding="utf-8"))


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        QAPP.processEvents(QtCore.QEventLoop.AllEvents, 20)
        time.sleep(0.005)


class _NoFrames:
    def frame(self, t_ms, trail_s):
        return None

    def radar_pose(self, num, t_ms):
        return None


class TestLiveClock(unittest.TestCase):
    def setUp(self):
        cfg = json.loads(json.dumps(CONFIG))
        self.c = vc.ViewController(cfg, APP, rdp_enabled=False)
        self.c.processor = _NoFrames()              # "export loaded" for the clock (no views here)
        self.c.projector.project = lambda views, processor: None
        self.c.t0_ms, self.c.t1_ms = 30_000.0, 800_000.0
        self.c.t_ms = self.c.t0_ms

    def packet(self, t_s, wall):
        self.c.rdp.store.last_packet_wall = wall
        self.c._on_rdp([], {"newest_time": t_s, "packets": 5, "receiving": True})

    def test_waiting_until_first_packet(self):
        self.assertFalse(self.c.live_following)
        self.assertEqual(self.c.live_state, vc.RDP_OFF)             # this controller has RDP disabled

    def test_follows_packet_time(self):
        self.packet(100.0, 1.0)
        self.assertTrue(self.c.live_following)
        t = self.c._live_time()
        self.assertGreaterEqual(t, 100_000.0)
        self.assertLessEqual(t, 100_000.0 + self.c.max_lead_ms)      # never runs ahead of the data
        self.packet(101.5, 2.0)
        self.assertGreaterEqual(self.c._live_time(), 101_500.0)

    def test_monotonic_and_restart(self):
        self.packet(200.0, 1.0)
        self.c.t_ms = self.c._live_time()
        self.packet(198.0, 2.0)                                      # out of order: ignored
        self.assertEqual(self.c._anchor[0], 200_000.0)
        self.packet(50.0, 3.0)                                       # RDP restarted: jump back
        self.assertEqual(self.c._anchor[0], 50_000.0)
        self.assertEqual(self.c.t_ms, 50_000.0)

    def test_replay_and_go_live(self):
        self.packet(300.0, 1.0)
        self.c.seek(100.0)
        self.assertTrue(self.c.replay_mode)
        self.assertFalse(self.c.live_following)
        self.assertEqual(self.c.t_ms, 100_000.0)
        self.c.go_live()
        self.assertTrue(self.c.live_following)
        self.assertEqual(self.c.t_ms, 300_000.0)

    def test_clamped_to_export(self):
        self.packet(900.0, 1.0)                                      # beyond the export (800 s)
        self.assertEqual(self.c._live_time(), 800_000.0)
        self.assertIn("outside the Scenario Export", self.c.live_detail)


class TestTargets(unittest.TestCase):
    def setUp(self):
        t = np.arange(0, 10_000.0, 20.0) + 5_000.0                   # 50 Hz rows, 5..15 s
        pos = np.column_stack([t * 0.1, 1000.0 + t * 0.0, np.full_like(t, 3000.0)])
        vel = np.tile([100.0, 0.0, 0.0], (len(t), 1))
        self.tg = TargetTimeline(4, t, pos, vel, "Foe", "True", "1234", "memory")
        self.proc = ScenarioProcessor(ScenarioData(Path("."), None, [], [], [self.tg]))

    def test_outside_span_is_absent(self):
        self.assertIsNone(self.proc.target_at(self.tg, 4_999.0, None))
        self.assertIsNone(self.proc.target_at(self.tg, 20_000.0, None))

    def test_state_and_history(self):
        tv = self.proc.target_at(self.tg, 12_345.0, None)
        i = self.tg.index_at(12_345.0)
        np.testing.assert_array_equal(tv.world, self.tg.pos[i])        # recorded row, not interpolated
        self.assertEqual(tv.state.target_id, "4")
        self.assertEqual(tv.state.classification, "Foe")
        self.assertEqual(tv.meta["IFF key"], "1234")
        self.assertAlmostEqual(tv.heading_deg, 90.0)                   # east
        np.testing.assert_array_equal(tv.world_trail[-1], tv.world)      # history ends at "now"
        self.assertTrue(np.all(np.diff(tv.meta["trail_t"]) > 0))
        self.assertLessEqual(tv.meta["trail_t"][-1], 12.345)             # never the future


class TestAmbience(unittest.TestCase):
    def make(self, cfg):
        from widgets.aircraft_audio import AircraftAmbience
        view = QtWidgets.QWidget()
        view.resize(400, 300)
        return view, AircraftAmbience(view, cfg, APP)

    def test_missing_file_is_harmless(self):
        view, amb = self.make({"file": "assets/does_not_exist.wav"})
        self.assertFalse(amb.available)
        self.assertIn("not found", amb.error)
        view.show()
        amb.update()                                                  # no exception, nothing plays
        self.assertFalse(amb.playing)
        amb.shutdown()

    def test_single_instance_follows_visibility(self):
        from PySide6.QtMultimedia import QSoundEffect
        view, amb = self.make(CONFIG.get("audio", {}))
        self.assertIsNone(amb.error)
        pump(3.0)                                                     # asynchronous load
        if not amb.available:
            self.skipTest("no audio output device / backend available here")
        amb.update()
        self.assertFalse(amb.playing)                                 # view not on screen
        for _ in range(5):                                            # repeated open / close
            view.show()
            pump(0.3)                                                 # shown -> exposed by the platform
            amb.update()
            self.assertTrue(amb.playing)
            view.hide()
            pump(0.1)
            amb.update()
            self.assertFalse(amb.playing)
        self.assertEqual(len(amb.findChildren(QSoundEffect)), 1)      # never more than one instance
        view.show()
        pump(0.3)
        amb.update()
        self.assertTrue(amb.playing)
        amb.set_enabled(False)
        self.assertFalse(amb.playing)
        amb.shutdown()


if __name__ == "__main__":
    unittest.main()
