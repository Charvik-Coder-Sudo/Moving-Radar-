"""Track items for pyqtgraph views (2D global view and Aircraft PPI).

Both views place tracks with a caller-supplied projection ``xy(TrackView) ->
(x, y)`` so the same symbol / colour / label logic serves the Cartesian world
view (X East, Y North) and the aircraft-frame PPI (right, forward).

Shape = track type (style.SOURCES), colour = classification (style.classify):
    target (truth)    military aircraft silhouette, muted fill, classification outline
    sensor track      hollow SQUARE - primary solid outline, secondary dashed and smaller
    fused system track  filled TRIANGLE over a soft halo, larger and brighter than the
                      sensor tracks it came from, drawn only at the fused position
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui

from models.track_state import KIND_FUSED, KIND_SENSOR, KIND_TARGET
from visualization import style, symbols

# stagger labels by source type so co-located truth / sensor / fused labels don't overlap
LABEL_ANCHORS = {style.FUSED: (-0.3, 2.0), style.TARGET: (-0.3, 1.1), style.PRIMARY: (-0.3, 0.2),
                 style.SECONDARY: (-0.3, -0.7)}
DIM_ALPHA = 90


def track_label(tv) -> str:
    """Labels from the actual identifiers.

        truth target   T<TgtId>                     (Scenario Export)
        fused track    FUSED-<systemTrackId>        (the fusion engine's system track)
        sensor track   S<sensorId>-ST<systemTrackId>

    The SystemTrack packet carries no per-sensor track id, so a sensor track is named by the
    sensor that reported it and the system track that carries it. Nothing is invented."""
    if tv.kind == KIND_TARGET:
        return f"T{tv.state.target_id}"
    if tv.kind == KIND_FUSED:
        return f"FUSED-{tv.system_track_id}"
    sensor = tv.state.track_id.split(":S")[-1] if ":S" in tv.state.track_id else tv.state.source
    return f"S{sensor}-ST{tv.system_track_id}"


def visible_tracks(tracks, layers):
    """Tracks allowed by the layer switches (sensor / fused)."""
    out = []
    for tv in tracks:
        if tv.kind == KIND_TARGET and not layers.get("show_targets", True):
            continue
        if tv.kind == KIND_SENSOR and not layers.get("show_sensor_tracks", True):
            continue
        if tv.kind == KIND_FUSED and not layers.get("show_fused_tracks", True):
            continue
        out.append(tv)
    return out


class TrackItems2D:
    """Scatter markers per source type, trails, vectors, predictions and labels."""

    def __init__(self, plot: pg.PlotItem, palette: style.Palette, z=10, label_size=9):
        self.plot = plot
        self.palette = palette
        self.label_size = label_size
        self.pens = symbols.PenCache()
        self.halo = pg.ScatterPlotItem(pxMode=True, symbol="o", pen=None)
        self.halo.setZValue(z + 1)
        plot.addItem(self.halo)
        self.scatter = {}
        for key, zo in ((style.TARGET, 2), (style.PRIMARY, 3), (style.SECONDARY, 3), (style.FUSED, 4)):
            s = pg.ScatterPlotItem(pxMode=True)
            s.setZValue(z + zo)
            plot.addItem(s)
            self.scatter[key] = s
        self.sel = pg.ScatterPlotItem(pxMode=True, symbol="o", size=36, brush=None,
                                      pen=pg.mkPen(style.SELECTION_COLOR, width=1.6, style=QtCore.Qt.DashLine))
        self.sel.setZValue(z + 6)
        plot.addItem(self.sel)
        self.trails: dict = {}
        self.vel = pg.PlotCurveItem(pen=pg.mkPen(style.VELOCITY_COLOR, width=1.6), connect="pairs")
        self.acc = pg.PlotCurveItem(pen=pg.mkPen(style.ACCELERATION_COLOR, width=1.8), connect="pairs")
        self.pred = pg.PlotCurveItem(pen=pg.mkPen(style.PREDICTION_COLOR, width=1.2,
                                                  style=QtCore.Qt.DashLine), connect="finite")
        for it in (self.vel, self.acc, self.pred):
            it.setZValue(z + 1)
            plot.addItem(it)
        self.history = pg.ScatterPlotItem(pxMode=True, symbol="o", size=3, pen=None)
        self.history.setZValue(5)
        plot.addItem(self.history)
        self.labels: dict = {}
        # (x, y, TrackView) of every marker drawn in the last update - used for hover lookup
        self.entries: list = []

    def clear(self):
        for s in self.scatter.values():
            s.clear()
        self.halo.clear()
        self.sel.clear()
        self.history.clear()
        for c in self.trails.values():
            c.setVisible(False)
        for it in (self.vel, self.acc, self.pred):
            it.setData([], [])
        for lab in self.labels.values():
            lab.setVisible(False)
        self.entries = []

    def update(self, tracks, xy, layers, settings, selected=None, dim=None,
               trail_xy=None, vec_xy=None, heading=None, label_fn=None):
        """
        tracks    list of TrackView
        xy        TrackView -> (x, y) marker position
        trail_xy  TrackView -> (M,2) array or None
        vec_xy    (TrackView, world_vector) -> (x0, y0, x1, y1) or None
        heading   TrackView -> screen heading (deg clockwise from up) for the aircraft icon
        dim       TrackView -> bool, draw faded (e.g. outside every sensor's coverage)
        """
        groups: dict[str, list] = {k: [] for k in self.scatter}
        for tv in tracks:
            groups[self.palette.source_key(tv.kind, tv.state.source)].append(tv)
        positions = {tv.key: xy(tv) for tv in tracks}
        self.entries = [(positions[tv.key][0], positions[tv.key][1], tv) for tv in tracks]

        halo_pos, halo_brush, halo_size = [], [], []
        for key, tvs in groups.items():
            if not tvs:
                self.scatter[key].clear()
                continue
            src = style.SOURCES[key]
            pos = np.array([positions[tv.key] for tv in tvs], float)
            pens, brushes, syms = [], [], []
            for tv in tvs:
                alpha = DIM_ALPHA if dim is not None and dim(tv) else 255
                colour = style.class_color(tv.state.classification)
                pen, brush = self.pens.marker(src, colour, alpha)
                pens.append(pen)
                brushes.append(brush)
                syms.append(symbols.symbol(src, heading(tv) if heading else 0.0))
                if key == style.FUSED:
                    # a soft halo keeps the fused triangle the most prominent thing on the plot
                    halo_pos.append(positions[tv.key])
                    halo_brush.append(self.pens.brush(colour, 55 if alpha == 255 else 25))
                    halo_size.append(src.size * 1.9)
            self.scatter[key].setData(pos=pos, symbol=syms, size=src.size, pen=pens, brush=brushes)
        if halo_pos:
            self.halo.setData(pos=np.array(halo_pos), brush=halo_brush, size=halo_size)
        else:
            self.halo.clear()

        # selection ring
        if selected is not None and selected in positions:
            self.sel.setData(pos=[positions[selected]])
        else:
            self.sel.clear()

        # trails: a line per truth / fused track, dots for sensor-track update history
        live = set()
        dots_xy, dots_brush = [], []
        if layers.get("show_target_trails", True) and trail_xy is not None:
            for tv in tracks:
                pts = trail_xy(tv)
                if pts is None or len(pts) < 1:
                    continue
                colour = style.class_color(tv.state.classification)
                if tv.kind == KIND_SENSOR:
                    dots_xy.append(pts)
                    dots_brush += [self.pens.brush(colour, 170)] * len(pts)
                    continue
                if len(pts) < 2:
                    continue
                live.add(tv.key)
                c = self.trails.get(tv.key)
                if c is None:
                    c = pg.PlotCurveItem()
                    c.setZValue(5)
                    self.plot.addItem(c)
                    self.trails[tv.key] = c
                c.setPen(self.pens.pen(colour, 150, 1.2))
                c.setData(pts[:, 0], pts[:, 1])
                c.setVisible(True)
        for k, c in self.trails.items():
            if k not in live:
                c.setVisible(False)
        if dots_xy:
            self.history.setData(pos=np.vstack(dots_xy), brush=dots_brush)
        else:
            self.history.clear()

        # vectors
        self._vectors(self.vel, tracks, layers.get("show_velocity_vectors", True) and vec_xy is not None,
                      lambda tv: vec_xy(tv, tv.state.velocity * settings["velocity_vector_seconds"]))
        self._vectors(self.acc, [tv for tv in tracks if tv.state.has_acceleration],
                      layers.get("show_acceleration_vectors", True) and vec_xy is not None,
                      lambda tv: vec_xy(tv, tv.state.acceleration * settings["acceleration_vector_scale_s2"]))

        # labels: classification colour, staggered by source type
        seen = set()
        if layers.get("show_target_labels", True):
            for tv in tracks:
                seen.add(tv.key)
                lab = self.labels.get(tv.key)
                if lab is None:
                    key = self.palette.source_key(tv.kind, tv.state.source)
                    lab = pg.TextItem(anchor=LABEL_ANCHORS[key])
                    lab.setZValue(30)
                    lab.setFont(QtGui.QFont("Consolas", self.label_size))
                    self.plot.addItem(lab)
                    self.labels[tv.key] = lab
                    lab._shown = (None, None)
                text = label_fn(tv) if label_fn else tv.state.track_id
                colour = style.class_color(tv.state.classification)
                if lab._shown != (text, colour):          # re-layout text only when it changes
                    lab.setColor(colour)
                    lab.setText(text)
                    lab._shown = (text, colour)
                lab.setPos(*positions[tv.key])
                if not lab.isVisible():
                    lab.setVisible(True)
        for k, lab in self.labels.items():
            if k not in seen:
                lab.setVisible(False)

    @staticmethod
    def _vectors(item, tracks, visible, fn):
        if not visible or not tracks:
            item.setData([], [])
            return
        xs, ys = [], []
        for tv in tracks:
            seg = fn(tv)
            if seg is None:
                continue
            xs += [seg[0], seg[2]]
            ys += [seg[1], seg[3]]
        item.setData(np.array(xs, float), np.array(ys, float))

    def set_predictions(self, paths, visible):
        if not visible or not paths:
            self.pred.setData([], [])
            return
        xs, ys = [], []
        for p in paths:
            xs += list(p[:, 0]) + [np.nan]
            ys += list(p[:, 1]) + [np.nan]
        self.pred.setData(np.array(xs), np.array(ys))
