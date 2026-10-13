"""The world view's visual language: what things look like, and how big.

Separate from `visualization.style`, which owns the project-wide symbology shared by all
three views - the square / triangle vocabulary and the classification colours. This module
holds what is specific to the 3D world: its palette, the sizes that keep an object readable
at range, the line weights that say which kind of track a trail belongs to, and the two
small pieces of geometry that go with them.

It exists so those numbers are not buried inside a renderer. They outlived one.

Sizing rule worth stating once: an aircraft drawn to true size is invisible at 100 km, so
every symbol has a minimum ANGULAR size - a fraction of its distance from the camera - and
is drawn at whichever is larger, the true size or that floor. The ownship is allowed to be
a little larger again, because it is the platform everything else is measured from.
"""

from __future__ import annotations

import numpy as np

from msdf_math import motion

# ---------------------------------------------------------------------------
# palette - a daylight environment that stays behind the tracks
# ---------------------------------------------------------------------------
SKY_TOP = "#1b3557"                    # zenith
SKY_HORIZON = "#5a748f"                # daylight haze: the scene fades into it with distance
GROUND_COLOR = "#76889c"               # beyond the terrain tiles: haze, not a green slab
RANGE_RING_COLOR = "#7d94ab"
AXIS_COLOR = "#94a3b8"                 # neutral slate, never a classification colour

# ---------------------------------------------------------------------------
# sizes
# ---------------------------------------------------------------------------
AIRCRAFT_LENGTH_M = 15.0
MIN_AIRCRAFT_ANGULAR_SIZE = 0.075      # of the distance to the camera
OWNSHIP_ANGULAR_SIZE = 0.095
# Track symbols: a SQUARE is a sensor track, a larger TRIANGLE a fused system track. Shape
# is the track type and colour is the classification; the sensor a track came from is said
# by size and by its label, never by shape.
SENSOR_GLYPH_ANGULAR_SIZE = {"PRIMARY": 0.010, "SECONDARY": 0.008}
FUSED_GLYPH_ANGULAR_SIZE = 0.017
GLYPH_MIN_SIZE_M, GLYPH_MAX_SIZE_M = 120.0, 9000.0

# ---------------------------------------------------------------------------
# line weights, in screen pixels (quick3d.ribbon turns them into camera-facing strips)
# ---------------------------------------------------------------------------
SENSOR_GLYPH_WIDTH, FUSED_GLYPH_WIDTH = 1.6, 2.8
SENSOR_TRAIL_WIDTH, FUSED_TRAIL_WIDTH = 1.1, 2.6
OWN_TRAIL_WIDTH = 1.6
VECTOR_WIDTH = 1.6
SELECTION_WIDTH = 2.2
# a fused system track outranks a sensor track in weight AND in strength, so the two say the
# same thing twice - the rule survives even where one of them is hard to judge
SENSOR_GLYPH_OPACITY, FUSED_GLYPH_OPACITY = 0.75, 1.00
SENSOR_TRAIL_OPACITY, FUSED_TRAIL_OPACITY = 0.50, 0.95
OWN_TRAIL_OPACITY = 0.90
TARGET_TRAIL_OPACITY = 0.85
VECTOR_OPACITY = 0.85

# ---------------------------------------------------------------------------
# how much of the scene is drawn, and how strongly
# ---------------------------------------------------------------------------
# Sensor geometry is context, not the subject: it must never compete with an aircraft.
COVERAGE_OPACITY = 0.022
BEAM_OPACITY = 0.40
RANGE_RING_OPACITY = 0.32
DETAIL_CEILING_M = 4_000.0      # above this camera height the metre-scale ground clutter is
                                # speckle, not scenery
MAX_ALWAYS_LABELLED = 6         # beyond this many targets, only the selected one is named
RANGE_RINGS_M = (25_000.0, 50_000.0, 100_000.0, 150_000.0)
FRAME_RELEVANT_M = RANGE_RINGS_M[-1]   # auto-framing reaches as far as the rings do
PICK_RADIUS_PX = 18.0           # how near a click has to be to select something

#: The ribbon layers the world view draws, in draw order, each with the strength it is drawn
#: at. The order is the reading order: context first, then trails, then the symbols that sit
#: on top of them, and the selection last so it is never hidden by what it marks.
RIBBON_LAYERS = (("rangeRings", RANGE_RING_OPACITY),
                 ("targetTrails", TARGET_TRAIL_OPACITY),
                 ("sensorTrails", SENSOR_TRAIL_OPACITY),
                 ("fusedTrails", FUSED_TRAIL_OPACITY),
                 ("ownTrail", OWN_TRAIL_OPACITY),
                 ("vectors", VECTOR_OPACITY),
                 ("sensorGlyphs", SENSOR_GLYPH_OPACITY),
                 ("fusedGlyphs", FUSED_GLYPH_OPACITY),
                 ("selection", 1.00))

# labels: a fused track usually sits on its truth target, so co-located labels are stacked
LABEL_OFFSET_PX = (12, 10)             # from the symbol, display pixels
LABEL_LINE_PX = 15.0                   # vertical step used when two labels collide
LABEL_WIDTH_PX = 74.0                  # assumed label width when testing for a collision

TRACK_COURSE_ALPHA = 0.25              # smoothing of a live track's heading (1.0 = none)


def stacked_offsets(sx, sy, front):
    """Label offsets (display pixels) that keep overlapping labels readable.

    sx, sy are the label anchors in display pixels and ``front`` says which ones are in
    front of the camera. Every label starts at LABEL_OFFSET_PX from its anchor; when a box
    would overlap one already placed it is stepped down a line, so a fused system track and
    the truth target under it read as two lines instead of one smear. The label positions
    are the only thing this changes.
    """
    sx, sy = np.asarray(sx, float), np.asarray(sy, float)
    out = [LABEL_OFFSET_PX] * len(sx)
    placed: list[tuple[float, float]] = []
    for i in np.argsort(-sy):                          # the topmost label keeps its natural place
        x, y = sx[i] + LABEL_OFFSET_PX[0], sy[i] + LABEL_OFFSET_PX[1]
        if front[i]:
            moved = True
            while moved:                               # y only ever decreases, so this terminates
                moved = False
                for px, py in placed:
                    if abs(x - px) < LABEL_WIDTH_PX and abs(y - py) < LABEL_LINE_PX:
                        y, moved = py - LABEL_LINE_PX, True
            placed.append((x, y))
        out[i] = (int(round(x - sx[i])), int(round(y - sy[i])))
    return out


def unit_from_course(course_deg, climb_deg) -> np.ndarray:
    """Compass course and climb angle -> a unit vector in world ENU."""
    c, g = np.radians(course_deg), np.radians(climb_deg)
    return np.array([np.sin(c) * np.cos(g), np.cos(c) * np.cos(g), np.sin(g)])


def own_course_vector(own) -> np.ndarray:
    """Ownship velocity along the flown path, at the recorded speed.

    The course the recorded positions imply, not the exported velocity columns - in this
    scenario the two disagree (msdf_math.motion) and the positions are authoritative. Falls
    back to the exported velocity when there is no usable course yet.
    """
    if not np.isfinite(own.course_deg):
        return own.velocity
    speed = own.path_speed_mps if np.isfinite(own.path_speed_mps) else own.speed
    return unit_from_course(own.course_deg, own.climb_deg) * speed


def course_vector(tv) -> np.ndarray:
    """Target velocity along its flown path, at the recorded speed."""
    speed = float(np.linalg.norm(tv.state.velocity))
    return unit_from_course(tv.heading_deg, tv.pitch_deg) * speed
