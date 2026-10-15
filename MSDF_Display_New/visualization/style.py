"""Central display style: the ONE place that maps classification and source type to looks.

Two independent concepts, never mixed:

    Classification  ->  colour        (Friendly green, Unknown yellow)
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

#: The display supports exactly TWO classifications. There is no hostile and no neutral:
#: MultiTrack decides identity with a single field holding "friend" or "unknown"
#: (plotCluster.processDetectionsFile), so no other value can reach a track, and presenting
#: options the system cannot produce would invite an operator to read meaning into them.
FRIENDLY, UNKNOWN = "FRIENDLY", "UNKNOWN"


@dataclass(frozen=True)
class ClassStyle:
    key: str
    label: str
    color: str          # hex


CLASSIFICATIONS: dict[str, ClassStyle] = {
    FRIENDLY: ClassStyle(FRIENDLY, "Friendly", "#22c55e"),     # green
    UNKNOWN: ClassStyle(UNKNOWN, "Unknown", "#facc15"),        # yellow
}

#: Spellings that mean FRIENDLY. Everything else - including a value this display does not
#: support, such as a scenario file's "Foe" - resolves to UNKNOWN.
#:
#: Unknown, never Friendly: Unknown means "not established as friendly", which is the honest
#: reading of a value the display cannot interpret. Mapping an unsupported value the other way
#: would assert something no part of the system ever determined.
_FRIENDLY_SPELLINGS = {"FRIENDLY", "FRIEND"}


def classification_from_auth(auth) -> str:
    """MultiTrack's IFF result -> this display's classification vocabulary.

    MultiTrack decides identity with ONE field holding "friend" or "unknown", and nothing
    else: ``plotCluster.processDetectionsFile`` maps the secondary sensor's ``Auth`` column to
    exactly those two words, and no part of the tracker ever produces a hostile or a neutral.
    Those two words are the whole vocabulary this display has, which is why there is no
    hostile and no neutral anywhere in it.

    "unknown" means no IFF reply was received. That is the absence of evidence, not evidence
    of hostility; it maps to Unknown.
    """
    return FRIENDLY if str(auth).strip().lower() == "friend" else UNKNOWN


def classify(raw) -> ClassStyle:
    """Canonical classification style for a value (case-insensitive). Friendly, or Unknown."""
    return CLASSIFICATIONS[FRIENDLY if str(raw).strip().upper() in _FRIENDLY_SPELLINGS
                           else UNKNOWN]


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
    glyph_3d: str       # 3D camera-facing outline (visualization.quick3d.ribbon)


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

#: Truth targets are drawn in this, never in a classification colour. A truth target is the
#: simulation saying WHERE AN AIRCRAFT ACTUALLY IS - it is not the tracking system saying what
#: it has identified. Colouring it green or red would put an identification on the screen that
#: no sensor made, and an operator comparing truth against tracks could not tell which of the
#: two was claiming it. Slate, so it cannot be mistaken for Friendly or Unknown
#: (visualization/style.py CLASSIFICATIONS).
TRUTH_COLOR = "#9aa7b4"


def track_color(kind: str, classification) -> str:
    """The colour a track is drawn in. THE one place that decides it.

    Classification colour belongs to tracks the system has identified. Truth comes from the
    Scenario Export and carries the scenario's own Auth column, which is ground truth about
    the aircraft rather than a determination anything made - so it is drawn neutral.

    Every view calls this instead of ``class_color`` so the rule cannot hold in one view and
    not another.
    """
    return TRUTH_COLOR if kind == KIND_TARGET else class_color(classification)


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


def rgb255(h: str) -> tuple[int, int, int]:
    """0-255 components, which is what the Qt Quick 3D geometry bridge and
    ``visualization.scenery``'s vertex colours both use."""
    return hex_to_rgb(h)
