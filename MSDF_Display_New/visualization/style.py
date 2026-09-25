"""Central display style: the ONE place that maps classification and source type to looks.

Two independent concepts, never mixed:

    Classification  ->  colour        (Friendly green, Neutral white, Hostile red, Unknown yellow)
    Track type      ->  symbol shape  SENSOR TRACK = SQUARE, FUSED SYSTEM TRACK = TRIANGLE,
                                      truth target = aircraft silhouette, ownship = its own
                                      outlined triangle (PPI) / aircraft model (world views)

Primary and secondary sensor tracks are the same square - the sensor they came from is shown
by the outline style (primary solid, secondary dashed), a slightly smaller symbol and the
label (S1-.. / S2-..), never by a different shape and never by colour.

Every view (3D, 2D, Aircraft PPI, track table, labels, legends) asks this module;
no widget defines its own classification colours. Non-classification colours
(ownship, vectors, sensors) are deliberately chosen away from green / white /
red / yellow so they can never be read as a classification.
"""

from __future__ import annotations

from dataclasses import dataclass

from models.track_state import KIND_FUSED, KIND_SENSOR, KIND_TARGET

# ----------------------------------------------------------------------------
# classification
# ----------------------------------------------------------------------------

FRIENDLY, NEUTRAL, HOSTILE, UNKNOWN = "FRIENDLY", "NEUTRAL", "HOSTILE", "UNKNOWN"


@dataclass(frozen=True)
class ClassStyle:
    key: str
    label: str
    color: str          # hex


CLASSIFICATIONS: dict[str, ClassStyle] = {
    FRIENDLY: ClassStyle(FRIENDLY, "Friendly", "#22c55e"),     # green
    NEUTRAL: ClassStyle(NEUTRAL, "Neutral", "#ffffff"),        # white
    HOSTILE: ClassStyle(HOSTILE, "Hostile", "#ef4444"),        # red
    UNKNOWN: ClassStyle(UNKNOWN, "Unknown", "#facc15"),        # yellow
}

# file spellings -> canonical classification. Anything not listed is Unknown.
_ALIASES = {
    "FRIENDLY": FRIENDLY, "FRIEND": FRIENDLY,
    "NEUTRAL": NEUTRAL,
    "HOSTILE": HOSTILE, "FOE": HOSTILE,
    "UNKNOWN": UNKNOWN, "PENDING": UNKNOWN, "": UNKNOWN,
}


def classify(raw) -> ClassStyle:
    """Canonical classification style for a track-file value (case-insensitive)."""
    return CLASSIFICATIONS[_ALIASES.get(str(raw).strip().upper(), UNKNOWN)]


def class_color(raw) -> str:
    return classify(raw).color


# ----------------------------------------------------------------------------
# source / record type
# ----------------------------------------------------------------------------

TARGET, PRIMARY, SECONDARY, FUSED = "TARGET", "PRIMARY", "SECONDARY", "FUSED"


@dataclass(frozen=True)
class SourceStyle:
    key: str
    label: str
    symbol: str         # 2D / PPI symbol id (see visualization.symbols)
    size: int           # px
    pen_width: float
    dashed: bool
    glyph_3d: str       # 3D camera-facing outline (see visualization.tracks_3d)


SOURCES: dict[str, SourceStyle] = {
    TARGET: SourceStyle(TARGET, "Target (truth)", "aircraft", 26, 1.8, False, "aircraft"),
    PRIMARY: SourceStyle(PRIMARY, "Primary sensor track", "square", 13, 2.0, False, "square"),
    SECONDARY: SourceStyle(SECONDARY, "Secondary sensor track", "square", 11, 2.0, True, "square"),
    FUSED: SourceStyle(FUSED, "Fused system track", "triangle", 20, 1.2, False, "triangle"),
}


# ----------------------------------------------------------------------------
# fixed, non-classification colours
# ----------------------------------------------------------------------------

OWNSHIP_COLOR = "#60a5fa"          # blue: own aircraft, never a classification
OWNSHIP_OUTLINE = "#dbeafe"        # outline of the PPI ownship triangle
VELOCITY_COLOR = "#2dd4bf"         # teal
ACCELERATION_COLOR = "#e879f9"     # magenta
PREDICTION_COLOR = "#94a3b8"       # slate
SELECTION_COLOR = "#93c5fd"        # light blue
HOVER_COLOR = "#e0f2fe"
TYPE_ICON_COLOR = "#94a3b8"        # track-type icons shown without a classification colour

# muted military finish for aircraft bodies (classification is never painted on the body)
AIRCRAFT_BODY = "#8c99a3"          # light gunmetal: reads against terrain and sky
AIRCRAFT_DARK = "#2a3136"          # canopy, nozzle, intakes
AIRCRAFT_ICON_FILL = "#7d8a91"     # 2D / PPI silhouette fill (slightly lighter for a dark map)


class Palette:
    """Per-configuration colours: sensor colours and sensor types come from the config."""

    def __init__(self, config: dict):
        self.sensor_colors = {s["id"]: s.get("color", "#38bdf8") for s in config.get("sensors", [])}
        self.sensor_types = {s["id"]: s.get("type", "primary") for s in config.get("sensors", [])}
        self.sensor_names = {s["id"]: s.get("name", s["id"]) for s in config.get("sensors", [])}

    def sensor_color(self, sensor_id: str) -> str:
        return self.sensor_colors.get(sensor_id, "#38bdf8")

    def beam_color(self, sensor_id: str) -> str:
        """The scan beam uses its own sensor's colour (drawn stronger than the coverage)."""
        return self.sensor_color(sensor_id)

    def source_key(self, kind: str, source: str) -> str:
        """SOURCES key of a track: target, primary, secondary or fused."""
        if kind == KIND_TARGET:
            return TARGET
        if kind == KIND_FUSED:
            return FUSED
        if kind == KIND_SENSOR:
            return SECONDARY if self.sensor_types.get(source, "primary") == "secondary" else PRIMARY
        return TARGET

    def source_style(self, kind: str, source: str) -> SourceStyle:
        return SOURCES[self.source_key(kind, source)]


def hex_to_rgb(h: str) -> tuple[int, int, int]:
    h = h.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def rgb01(h: str) -> tuple[float, float, float]:
    r, g, b = hex_to_rgb(h)
    return r / 255.0, g / 255.0, b / 255.0
