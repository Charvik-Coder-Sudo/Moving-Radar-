"""Sensor tracks and fused system tracks must never look like the same thing.

Checks the shared rules: label format from the real identifiers, one symbol per source type,
the fused track's stronger visual weight, and the age fade used by the 3D trails.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

from rdp.packet import decode, encode_for_test  # noqa: E402
from rdp.track_store import TrackStore  # noqa: E402
from visualization import style  # noqa: E402
from visualization.tracks_2d import track_label  # noqa: E402

X = (1000.0, 11.0, 0.5, 30000.0, 22.0, 0.6, 0.02, 2000.0, 3.0, 0.7)


def views():
    store = TrackStore({}, {1: "PRIMARY", 2: "SECONDARY"})
    store.ingest(decode(encode_for_test(12, 100.0, 1, X, [1, 2], [X, X]), received_wall=1.0))
    return {tv.key: tv for tv in store.views(1.0)}


class TestLabels(unittest.TestCase):
    def test_labels_use_the_real_identifiers(self):
        v = views()
        self.assertEqual(track_label(v[("FUSED", "ST12")]), "FUSED-12")
        self.assertEqual(track_label(v[("PRIMARY", "ST12:S1")]), "S1-ST12")
        self.assertEqual(track_label(v[("SECONDARY", "ST12:S2")]), "S2-ST12")

    def test_target_label(self):
        from processing.frame import TrackView
        from models.track_state import KIND_TARGET, TrackState
        st = TrackState(timestamp=0.0, track_id="T4", target_id="4", x=0, y=0, z=0, vx=0, vy=0, vz=0,
                        omega=0, ax=0, ay=0, az=0, classification="Foe", source="TRUTH", status="scenario")
        self.assertEqual(track_label(TrackView(key=("TARGET", "T4"), kind=KIND_TARGET, state=st,
                                               age_s=0.0, heading_deg=0.0, trail=np.empty((0, 3)))), "T4")


class TestVisualSeparation(unittest.TestCase):
    def test_shape_is_the_track_type_and_never_the_classification(self):
        """The whole convention in one place: sensor = square, fused = triangle."""
        srcs = style.SOURCES
        self.assertEqual(srcs[style.PRIMARY].symbol, "square")
        self.assertEqual(srcs[style.SECONDARY].symbol, "square", "both sensors use the same shape")
        self.assertEqual(srcs[style.FUSED].symbol, "triangle")
        self.assertEqual(srcs[style.TARGET].symbol, "aircraft")
        # primary vs secondary is told apart by outline style and size, not by shape or colour
        self.assertNotEqual(srcs[style.PRIMARY].dashed, srcs[style.SECONDARY].dashed)
        self.assertNotEqual(srcs[style.PRIMARY].size, srcs[style.SECONDARY].size)

    def test_the_fused_track_dominates_its_sensor_tracks(self):
        srcs = style.SOURCES
        fused, primary, secondary = srcs[style.FUSED], srcs[style.PRIMARY], srcs[style.SECONDARY]
        self.assertGreater(fused.size, primary.size, "the fused track must dominate its sensor tracks")
        self.assertGreater(fused.size, secondary.size)

    def test_the_three_views_agree_on_the_symbols(self):
        """3D glyphs, 2D/PPI symbols and the legend icons all come from the same table."""
        from visualization import symbols
        from visualization.quick3d.ribbon import GLYPH_SHAPES
        for key, shape in ((style.PRIMARY, "square"), (style.SECONDARY, "square"),
                           (style.FUSED, "triangle")):
            src = style.SOURCES[key]
            self.assertEqual(src.symbol, shape)            # 2D and PPI
            self.assertEqual(src.glyph_3d, shape)          # 3D
            self.assertIn(src.glyph_3d, GLYPH_SHAPES)
        self.assertIs(symbols.symbol(style.SOURCES[style.FUSED]), symbols.TRIANGLE)
        self.assertEqual(symbols.symbol(style.SOURCES[style.PRIMARY]), "s")

    def test_three_dimensional_weights(self):
        """3D: sensor squares stay small and thin, the fused triangle is larger and brighter."""
        from visualization import world_style as ws
        self.assertGreater(ws.FUSED_GLYPH_ANGULAR_SIZE, max(ws.SENSOR_GLYPH_ANGULAR_SIZE.values()))
        self.assertGreater(ws.FUSED_GLYPH_WIDTH, ws.SENSOR_GLYPH_WIDTH)
        self.assertGreater(ws.FUSED_GLYPH_OPACITY, ws.SENSOR_GLYPH_OPACITY)
        self.assertGreater(ws.FUSED_TRAIL_WIDTH, ws.SENSOR_TRAIL_WIDTH)
        self.assertGreater(ws.FUSED_TRAIL_OPACITY, ws.SENSOR_TRAIL_OPACITY)
        self.assertGreater(ws.SENSOR_GLYPH_ANGULAR_SIZE["PRIMARY"],
                           ws.SENSOR_GLYPH_ANGULAR_SIZE["SECONDARY"])

    def test_the_renderer_takes_its_layer_strengths_from_one_place(self):
        """scene.qml used to carry its own copy of these opacities. The draw order and the
        strengths are one vocabulary: if they live in two files they drift apart."""
        from visualization import world_style as ws
        layers = dict(ws.RIBBON_LAYERS)
        self.assertEqual(layers["fusedGlyphs"], ws.FUSED_GLYPH_OPACITY)
        self.assertEqual(layers["sensorTrails"], ws.SENSOR_TRAIL_OPACITY)
        self.assertEqual(layers["selection"], 1.00)          # never dimmed by what it marks
        qml = (Path(__file__).resolve().parents[1]
               / "visualization" / "quick3d" / "scene.qml").read_text(encoding="utf-8")
        self.assertIn("property var ribbonLayers: []", qml)   # filled from Python, not duplicated

    def test_the_glyph_outlines_are_closed_and_camera_facing(self):
        from visualization.quick3d.ribbon import GLYPH_SHAPES, glyph_segments
        right, up = np.array([1.0, 0, 0]), np.array([0, 0, 1.0])
        segs = glyph_segments([[0, 0, 0], [1000, 0, 0]], "triangle", [100.0, 200.0], right, up)
        per_glyph = len(GLYPH_SHAPES["triangle"]) - 1       # a closed outline: 3 segments
        self.assertEqual(segs.shape, (2 * per_glyph, 2, 3))
        for g in range(2):                                 # each outline returns to its start
            ring = segs[g * per_glyph:(g + 1) * per_glyph]
            np.testing.assert_allclose(ring[0, 0], ring[-1, 1], atol=1e-9)
        # the outline lies in the plane the camera axes span: it faces the camera squarely
        self.assertAlmostEqual(float(np.abs(segs[..., 1]).max()), 0.0, places=9)
        # and it is drawn at the size asked for: the second glyph is twice the first
        self.assertAlmostEqual(np.abs(segs[per_glyph:, :, 2]).max()
                               / np.abs(segs[:per_glyph, :, 2]).max(), 2.0, places=9)


class TestLabelPlacement(unittest.TestCase):
    """A fused track sits on its truth target, so their 3D labels must not land on each other."""

    def test_colocated_labels_are_stacked_one_line_apart(self):
        from visualization.world_style import LABEL_LINE_PX, LABEL_OFFSET_PX, stacked_offsets
        off = stacked_offsets([500.0, 500.0, 502.0], [400.0, 400.0, 399.0], [True] * 3)
        ys = sorted(o[1] + y for o, y in zip(off, (400.0, 400.0, 399.0)))
        self.assertTrue(all(b - a >= LABEL_LINE_PX - 1 for a, b in zip(ys, ys[1:])), ys)

    def test_labels_far_apart_keep_their_natural_offset(self):
        from visualization.world_style import LABEL_OFFSET_PX, stacked_offsets
        off = stacked_offsets([100.0, 900.0], [100.0, 700.0], [True, True])
        self.assertEqual(off, [LABEL_OFFSET_PX, LABEL_OFFSET_PX])

    def test_labels_behind_the_camera_are_left_alone(self):
        from visualization.world_style import LABEL_OFFSET_PX, stacked_offsets
        off = stacked_offsets([500.0, 500.0], [400.0, 400.0], [False, True])
        self.assertEqual(off[0], LABEL_OFFSET_PX)          # not placed, so it never pushes the other one
        self.assertEqual(off[1], LABEL_OFFSET_PX)


class TestAgeFade(unittest.TestCase):
    def test_trail_fades_from_the_oldest_end_to_the_track_colour(self):
        from visualization.quick3d.ribbon import fade
        rgb = fade((255, 255, 255), 10, oldest=0.2)
        self.assertEqual(len(rgb), 10)
        self.assertTrue(np.all(np.diff(rgb[:, 0]) > 0), "older segments must be dimmer")
        np.testing.assert_allclose(rgb[-1], [255, 255, 255])     # newest segment: full colour
        np.testing.assert_allclose(rgb[0], [51.0, 51.0, 51.0])   # oldest: 0.2 of the colour

    def test_an_empty_trail_fades_to_nothing_rather_than_raising(self):
        from visualization.quick3d.ribbon import fade
        self.assertEqual(fade((255, 0, 0), 0).shape, (0, 3))



class TestLabelStackingTerminates(unittest.TestCase):
    """Label placement runs inside the render path, so it must finish - always.

    It once stepped each label to `other_label_y - LABEL_LINE_PX`, which moves it by however
    far that neighbour happens to be. For a cluster of tracks a fraction of a pixel apart that
    is an unbounded number of passes, and the window stops responding mid-frame. Found by
    faulthandler on a wedged display fed real MultiTrack traffic.
    """

    def _within(self, sx, sy, seconds=5.0):
        import threading
        from visualization.world_style import stacked_offsets
        done = []
        t = threading.Thread(target=lambda: done.append(stacked_offsets(sx, sy, [True] * len(sx))),
                             daemon=True)
        t.start()
        t.join(seconds)
        self.assertTrue(done, f"stacked_offsets did not return within {seconds}s "
                              f"for {len(sx)} labels")
        return done[0]

    def test_a_pile_of_identical_positions_returns(self):
        n = 60
        self._within(np.full(n, 500.0), np.full(n, 400.0))

    def test_labels_a_hair_under_one_line_apart_return(self):
        from visualization.world_style import LABEL_LINE_PX
        n = 40
        self._within(np.full(n, 500.0), 400.0 + np.arange(n) * (LABEL_LINE_PX - 0.01))

    def test_sub_pixel_jitter_returns(self):
        rng = np.random.default_rng(1)
        n = 50
        self._within(np.full(n, 500.0), 400.0 + rng.normal(0.0, 0.3, n))

    def test_the_drop_is_capped(self):
        from visualization.world_style import LABEL_LINE_PX, MAX_LABEL_STACK, LABEL_OFFSET_PX
        n = 80
        off = self._within(np.full(n, 500.0), np.full(n, 400.0))
        deepest = min(o[1] for o in off)
        floor = LABEL_OFFSET_PX[1] - MAX_LABEL_STACK * LABEL_LINE_PX
        self.assertGreaterEqual(deepest, floor,
                                "a label was pushed past the cap, so the work is unbounded")

    def test_it_still_separates_labels_that_fit(self):
        from visualization.world_style import LABEL_LINE_PX
        off = self._within(np.array([500.0, 500.0, 502.0]), np.array([400.0, 400.0, 399.0]))
        ys = sorted(o[1] + y for o, y in zip(off, (400.0, 400.0, 399.0)))
        self.assertTrue(all(b - a >= LABEL_LINE_PX - 1 for a, b in zip(ys, ys[1:])), ys)


if __name__ == "__main__":
    unittest.main()
