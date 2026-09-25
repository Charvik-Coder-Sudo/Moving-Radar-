"""Classification style, source symbols, aircraft model and hover content (no GUI)."""

import json
import re
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))

from rdp.packet import decode, encode_for_test                     # noqa: E402
from rdp.track_store import TrackStore                             # noqa: E402
from visualization import style                                    # noqa: E402

CONFIG = json.loads((APP / "config" / "display_config.json").read_text(encoding="utf-8"))
SENSOR_MAP = CONFIG["scenario_export"]["sensors"]
# what ViewController puts in config["sensors"] (ids / types / colours from the configured map)
CONFIG["sensors"] = [dict(id=m["id"], name=m["name"], type=m["type"], color=m["color"]) for m in SENSOR_MAP.values()]
GREEN, WHITE, RED, YELLOW = "#22c55e", "#ffffff", "#ef4444", "#facc15"


class TestClassification(unittest.TestCase):
    def test_exact_colours(self):
        self.assertEqual({k: c.color for k, c in style.CLASSIFICATIONS.items()},
                         {"FRIENDLY": GREEN, "NEUTRAL": WHITE, "HOSTILE": RED, "UNKNOWN": YELLOW})

    def test_aliases(self):
        for raw, colour in (("FRIENDLY", GREEN), ("Friend", GREEN), ("neutral", WHITE), ("HOSTILE", RED),
                            ("Foe", RED), ("UNKNOWN", YELLOW), ("PENDING", YELLOW), ("", YELLOW),
                            ("SOMETHING-ELSE", YELLOW)):
            self.assertEqual(style.class_color(raw), colour, raw)

    def test_single_definition(self):
        """No other display module defines a classification colour."""
        hexes = {GREEN, WHITE, RED, YELLOW}
        offenders = []
        for f in list((APP / "visualization").glob("*.py")) + list((APP / "widgets").glob("*.py")):
            if f.name == "style.py":
                continue
            text = f.read_text(encoding="utf-8").lower()
            if any(h in text for h in hexes) or re.search(r"classification_colou?rs", text):
                offenders.append(f.name)
        self.assertEqual(offenders, [])
        self.assertNotIn("classification_colors", CONFIG.get("display", {}))

    def test_non_classification_colours_differ(self):
        others = {style.OWNSHIP_COLOR, style.VELOCITY_COLOR, style.ACCELERATION_COLOR,
                  style.AIRCRAFT_BODY, style.AIRCRAFT_ICON_FILL}
        others |= {m["color"] for m in SENSOR_MAP.values()}
        self.assertFalse(others & {GREEN, WHITE, RED, YELLOW})

    def test_source_is_separate_from_classification(self):
        pal = style.Palette(CONFIG)
        self.assertEqual(pal.source_key("sensor", "PRIMARY"), style.PRIMARY)
        self.assertEqual(pal.source_key("sensor", "SECONDARY"), style.SECONDARY)
        self.assertEqual(pal.source_key("fused", "FUSED"), style.FUSED)
        # shape is the TRACK TYPE: both sensors share the square, the fused track is a triangle,
        # and the sensor a track came from is shown by outline style / size / label, never colour
        self.assertEqual(style.SOURCES[style.PRIMARY].symbol, style.SOURCES[style.SECONDARY].symbol)
        self.assertEqual({s.symbol for s in style.SOURCES.values()},
                         {"square", "triangle", "aircraft"})


class TestAircraftModel(unittest.TestCase):
    def test_indicator_only_on_fin_tip_and_wingtips(self):
        from visualization.aircraft_model import PART_INDICATOR, military_jet
        for detail in ("high", "low"):
            m = military_jet(detail)
            pts = np.asarray(m.points)[np.asarray(m.point_data["part"]) == PART_INDICATOR]
            self.assertTrue(len(pts) > 0)
            on_wingtip = np.abs(pts[:, 1]) >= 4.75 - 1e-9
            on_fin_top = pts[:, 2] <= -2.95 + 1e-9
            self.assertTrue(np.all(on_wingtip | on_fin_top))
            self.assertLess(m.n_cells, 600)                   # lightweight for real-time instancing


class TestHoverContent(unittest.TestCase):
    """Tooltips for tracks built from an RDP packet (the only track source)."""

    @classmethod
    def setUpClass(cls):
        names = {int(k): m["id"] for k, m in SENSOR_MAP.items()}
        store = TrackStore(CONFIG["rdp"], names)
        X = (1000.0, 11.0, 0.5, 2000.0, 22.0, 0.6, 0.02, 300.0, 3.0, 0.7)
        store.ingest(decode(encode_for_test(5, 815.25, 1, X, [1, 2], [X, X]), received_wall=10.0))
        store.ingest(decode(encode_for_test(6, 815.5, 1, X, [], []), received_wall=10.0))
        cls.views = store.views(11.0)
        cls.pal = style.Palette(CONFIG)
        from visualization.hover import describe_track
        cls.describe = staticmethod(describe_track)

    def test_fused_fields(self):
        fused = next(tv for tv in self.views if tv.kind == "fused" and tv.system_track_id == 5)
        html = self.describe(fused, self.pal, (1000.0, 12.0, 1.0), "sensor-relative")
        for field in ("Track ID", "Target ID", "Classification", "Source", "Range", "Azimuth", "Elevation",
                      "X / Y / Z", "Vx / Vy / Vz", "Contributing sensors", "No. of sensors", "Packet time",
                      "Frame"):
            self.assertIn(field, html)
        self.assertIn("PRIMARY", html)
        self.assertIn("SECONDARY", html)
        self.assertIn("815.250000 (as sent)", html)
        self.assertIn("not in packet", html)                   # classification / target id are not sent

    def test_fused_without_sensors(self):
        fused = next(tv for tv in self.views if tv.system_track_id == 6)
        self.assertIn("numSensors = 0", self.describe(fused, self.pal, (1.0, 0.0, 0.0), "test"))

    def test_sensor_track(self):
        sensor = next(tv for tv in self.views if tv.kind == "sensor")
        html = self.describe(sensor, self.pal, (1.0, 0.0, 0.0), "test")
        self.assertNotIn("Contributing sensors", html)
        self.assertIn("ST5", html)                              # the system track it belongs to


if __name__ == "__main__":
    unittest.main()
