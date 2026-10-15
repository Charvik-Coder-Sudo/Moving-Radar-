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
from models.track_state import KIND_FUSED, KIND_SENSOR, KIND_TARGET  # noqa: E402
from visualization import style                                    # noqa: E402

CONFIG = json.loads((APP / "config" / "display_config.json").read_text(encoding="utf-8"))
SENSOR_MAP = CONFIG["scenario_export"]["sensors"]
# what ViewController puts in config["sensors"] (ids / types / colours from the configured map)
CONFIG["sensors"] = [dict(id=m["id"], name=m["name"], type=m["type"], color=m["color"]) for m in SENSOR_MAP.values()]
GREEN, YELLOW = "#22c55e", "#facc15"
#: the colours the display no longer has a classification for
WHITE, RED = "#ffffff", "#ef4444"


class TestClassification(unittest.TestCase):
    def test_there_are_exactly_two_classifications(self):
        """The display supports Friendly and Unknown and nothing else: MultiTrack decides
        identity with one field holding 'friend' or 'unknown', so an option for anything else
        would invite an operator to read meaning into a state the system cannot produce."""
        self.assertEqual({k: c.color for k, c in style.CLASSIFICATIONS.items()},
                         {"FRIENDLY": GREEN, "UNKNOWN": YELLOW})
        self.assertEqual([c.label for c in style.CLASSIFICATIONS.values()],
                         ["Friendly", "Unknown"])

    def test_only_friendly_spellings_are_friendly(self):
        for raw in ("FRIENDLY", "Friend", "friendly", " friend "):
            self.assertEqual(style.class_color(raw), GREEN, raw)

    def test_everything_else_resolves_to_unknown(self):
        """Including values this display does not support. Unknown, never Friendly: Unknown
        means 'not established as friendly', which is the honest reading of a value that
        cannot be interpreted. The other way round would assert something nothing determined."""
        for raw in ("Foe", "HOSTILE", "neutral", "UNKNOWN", "PENDING", "", None,
                    "SOMETHING-ELSE"):
            self.assertEqual(style.class_color(raw), YELLOW, raw)

    def test_the_retired_colours_are_gone(self):
        in_use = {c.color for c in style.CLASSIFICATIONS.values()}
        self.assertNotIn(WHITE, in_use, "neutral white is still a classification colour")
        self.assertNotIn(RED, in_use, "hostile red is still a classification colour")

    def test_no_hostile_or_neutral_reaches_the_user(self):
        """Not in a label, not in the legend, not through classify()."""
        for raw in ("Foe", "HOSTILE", "Neutral", "anything"):
            self.assertIn(style.classify(raw).label, ("Friendly", "Unknown"))

    def test_single_definition(self):
        """No other display module defines a classification colour."""
        hexes = {GREEN, YELLOW}
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



class TestTruthIsNotClassified(unittest.TestCase):
    """Truth targets say where a simulated aircraft IS. They must never be drawn as though
    the tracking system had identified them, or an operator comparing truth against tracks
    cannot tell which of the two is making the claim."""

    def test_truth_is_always_the_neutral_colour(self):
        for raw in ("Friend", "Foe", "Neutral", "Unknown", "", None):
            self.assertEqual(style.track_color(KIND_TARGET, raw), style.TRUTH_COLOR)

    def test_tracks_still_carry_their_classification(self):
        for kind in (KIND_SENSOR, KIND_FUSED):
            self.assertEqual(style.track_color(kind, "Friend"), style.class_color("FRIENDLY"))
            self.assertEqual(style.track_color(kind, "Unknown"), style.class_color("UNKNOWN"))

    def test_the_truth_colour_is_not_a_classification_colour(self):
        truth = style.TRUTH_COLOR.lower()
        for cs in style.CLASSIFICATIONS.values():
            self.assertNotEqual(truth, cs.color.lower())

    def test_no_view_colours_a_track_behind_this_rule(self):
        """The rule is only worth anything if every renderer goes through it. If a view calls
        class_color on a track's own classification it can colour truth by accident."""
        root = Path(__file__).resolve().parents[1]
        offenders = []
        for rel in ("visualization/tracks_2d.py", "visualization/view_3d_quick.py",
                    "visualization/aircraft_ppi.py"):
            text = (root / rel).read_text(encoding="utf-8")
            if "class_color(tv.state.classification)" in text:
                offenders.append(rel)
        self.assertEqual(offenders, [], "these views bypass style.track_color")


class TestTrackDetailsStaysNeutral(unittest.TestCase):
    """The table is the readable record of what each packet said. Tinting a cell green or red
    turns a reported value into a judgement, and the colour-coded picture is the views' job."""

    def test_the_table_has_no_classification_colour(self):
        text = (Path(__file__).resolve().parents[1]
                / "widgets" / "track_table.py").read_text(encoding="utf-8")
        self.assertNotIn("ForegroundRole", text)
        self.assertNotIn("class_icons", text)
        self.assertNotIn("swatch_pixmap", text)

    def test_the_values_are_still_shown_as_text(self):
        from widgets.track_table import COLUMNS
        headers = [c[0] for c in COLUMNS]
        for wanted in ("Classification", "Authentication", "ICAO"):
            self.assertIn(wanted, headers)


if __name__ == "__main__":
    unittest.main()
