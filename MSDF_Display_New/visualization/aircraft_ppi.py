"""Aircraft PPI - the RDP track picture, permanently visible.

Shows, at the same time, what the RDP transmits in its SystemTrack packets:

    Primary sensor tracks     sensorStates of sensorId 1   hollow square
    Secondary sensor tracks   sensorStates of sensorId 2   hollow dashed diamond
    Fused / system tracks     Xfused, systemTrackId        military aircraft inside a
                                                           highlighted ring

Geometry: AIRCRAFT BODY FRAME, 0 deg at the top = nose, + clockwise (right).
    plot x = body Right, plot y = body Forward (km).

    RDP tracks   the RDP builds its states from each sensor's radar range / azimuth /
                 elevation (sensor-relative x Right, y Forward, z Up = radar FRD
                 (y, x, -z)); each is carried into the aircraft frame by its own
                 sensor's mount: P_B = mount_xyz + R_BR @ L (processing.track_projection).
                 This needs no ownship pose. Fused states use latestSensor's mount.
    targets      scenario truth targets at the displayed time: R_WB^T (P_W - P_own)
                 with the exported ownship attitude.
    trails       an object's own world path (exported target rows, or the world positions
                 placed once per packet) carried into the aircraft frame with the ownship
                 pose OF THE DISPLAYED FRAME - one conversion, applied to every point, so
                 the drawn path keeps the shape of the real one and its head sits on the
                 marker. Converting each point with the pose of its own moment instead
                 would draw the history of the relative geometry, which slides backwards
                 under the marker at the ownship's speed.
    coverage     each sensor's field of view through its mount rotation, and the
                 current scan-beam dwell (scan schedule) at the displayed time.

Colour = classification (RDP packets carry none, so RDP tracks are Unknown; truth
targets use the scenario's Auth); shape = track type. Faded = stale.
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from models.track_state import KIND_FUSED, KIND_TARGET
from msdf_math import coordinates
from processing.track_projection import rdp_to_frd
from visualization import hover, style, symbols
from visualization.tracks_2d import TrackItems2D, track_label, visible_tracks
from widgets.legend_widget import LegendWidget

KM = 1e-3
RANGES_KM = [10, 25, 50, 100, 150, 200]
OWNSHIP_SIZE_PX = 30            # larger than any track symbol: it is the platform, not a track
RING = "#334155"
RING_OUTER = "#64748b"
RING_TEXT = "#94a3b8"


def _ring_step(r_km):
    for s in (2, 5, 10, 20, 25, 50):
        if r_km / s <= 5:
            return s
    return 50


def _body_azimuths(cfg, az_deg, el_deg):
    """Radar-frame (az, el) -> azimuth in the aircraft body frame, via the sensor mount R_BR."""
    d = coordinates.frd_direction(np.atleast_1d(az_deg), np.atleast_1d(el_deg)) @ cfg.R_BR.T
    return np.degrees(np.arctan2(d[:, 1], d[:, 0]))


def _polar_xy(r, az_deg):
    a = np.radians(az_deg)
    return np.column_stack([r * np.sin(a), r * np.cos(a)])


class AircraftPPI(QtWidgets.QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.cfg = controller.config
        self.palette = style.Palette(self.cfg)
        self.range_km = float(self.cfg.get("display", {}).get("ppi_range_m", 100000.0)) * KM
        self.show_src = {style.PRIMARY: True, style.SECONDARY: True, style.FUSED: True}
        self._views: list = []
        self._stats: dict = {}
        self._frame = None

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(self._build_header())
        lay.addWidget(self._build_source_bar())
        self.pw = pg.PlotWidget(background="#050b14")
        self.pw.setMinimumSize(200, 180)
        lay.addWidget(self.pw, 1)
        self.footer = QtWidgets.QLabel("", objectName="PPIFooter")
        self.footer.setWordWrap(True)
        self.footer.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        self.footer.setMinimumHeight(48)
        lay.addWidget(self.footer)
        legend = LegendWidget(columns=4, include_target=True)
        # the same legend is in the Layers page: here it may shrink away when the PPI is small
        legend.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        lay.addWidget(legend)

        pl = self.pw.getPlotItem()
        self.plot = pl
        pl.hideAxis("left")
        pl.hideAxis("bottom")
        pl.setAspectLocked(True)
        pl.setMouseEnabled(False, False)
        pl.hideButtons()
        pl.setMenuEnabled(False)

        self.static_items: list = []
        self.fov: dict = {}
        for s in self.ctrl.sensors:
            col = QtGui.QColor(self.palette.sensor_color(s.sensor_id))
            fov = QtWidgets.QGraphicsPathItem()
            fill = QtGui.QColor(col)
            fill.setAlpha(22)
            fov.setBrush(QtGui.QBrush(fill))
            pen = QtGui.QPen(col)
            pen.setCosmetic(True)
            pen.setWidthF(1.4)
            fov.setPen(pen)
            fov.setZValue(2)
            pl.addItem(fov)
            self.fov[s.sensor_id] = fov
        self.beam: dict = {}
        for s in self.ctrl.sensors:
            col = QtGui.QColor(self.palette.beam_color(s.sensor_id))
            wedge = QtWidgets.QGraphicsPathItem()
            fill = QtGui.QColor(col)
            fill.setAlpha(110)
            wedge.setBrush(QtGui.QBrush(fill))
            pen = QtGui.QPen(col)
            pen.setCosmetic(True)
            wedge.setPen(pen)
            wedge.setZValue(3)
            pl.addItem(wedge)
            self.beam[s.sensor_id] = wedge
        self.own = pg.ScatterPlotItem(pxMode=True)
        self.own.setZValue(50)
        pl.addItem(self.own)
        # OWN AIRCRAFT: a large outlined triangle with a notched tail, in the ownship colour.
        # Nose up = the aircraft's own nose, because this plot IS the aircraft body frame.
        # A fused system track is a smaller plain filled triangle, so the two cannot be confused.
        self.own.setData([dict(pos=(0, 0), size=OWNSHIP_SIZE_PX, symbol=symbols.OWNSHIP,
                               pen=pg.mkPen(style.OWNSHIP_OUTLINE, width=2.0),
                               brush=pg.mkBrush(style.OWNSHIP_COLOR))])
        self.tracks = TrackItems2D(pl, self.palette, z=10, label_size=8)
        self.hover = hover.HoverInspector(self.pw, self._hover_entries, self._describe)
        self._draw_static()
        pl.vb.sigResized.connect(lambda *_: QtCore.QTimer.singleShot(0, self._fit))

        self._footer_clock = QtCore.QElapsedTimer()
        self._footer_clock.start()
        self._frame_clock = QtCore.QElapsedTimer()
        self._frame_clock.start()
        controller.rdp_updated.connect(self.on_rdp)
        controller.frame_ready.connect(self.on_frame)
        controller.layers_changed.connect(lambda: self.render())

    # ------------------------------------------------------------------ header
    def _build_header(self):
        bar = QtWidgets.QFrame(objectName="ViewToolbar")
        bar.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(8, 4, 8, 4)
        h.setSpacing(3)
        h.addWidget(QtWidgets.QLabel("AIRCRAFT PPI", objectName="ViewTitle"))
        h.addWidget(QtWidgets.QLabel("aircraft frame · 0° = nose", objectName="ViewInfo"))
        h.addStretch(1)
        self.range_box = QtWidgets.QComboBox()
        for r in RANGES_KM:
            self.range_box.addItem(f"{r} km", r)
        j = self.range_box.findData(int(self.range_km))
        self.range_box.setCurrentIndex(j if j >= 0 else 3)
        self.range_box.currentIndexChanged.connect(self._range_changed)
        h.addWidget(self.range_box)
        return bar

    def _build_source_bar(self):
        """Per-type show/hide (all on by default: the three track types together)."""
        bar = QtWidgets.QFrame(objectName="ViewToolbar")
        bar.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(8, 2, 8, 2)
        h.setSpacing(3)
        h.addWidget(QtWidgets.QLabel("Show", objectName="ViewInfo"))
        self.source_btns = {}
        for key, short in ((style.PRIMARY, "Primary"), (style.SECONDARY, "Secondary"), (style.FUSED, "Fused")):
            src = style.SOURCES[key]
            b = QtWidgets.QToolButton(text=short, checkable=True, checked=True)
            b.setIcon(QtGui.QIcon(symbols.icon_pixmap(src, style.TYPE_ICON_COLOR, 16)))
            b.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
            b.setToolTip(f"Show {src.label}s on the PPI")
            b.toggled.connect(lambda on, k=key: self._toggle(k, on))
            h.addWidget(b)
            self.source_btns[key] = b
        h.addStretch(1)
        return bar

    def _toggle(self, key, on):
        self.show_src[key] = on
        self.render()

    def _range_changed(self):
        self.range_km = float(self.range_box.currentData())
        self._draw_static()
        self.render()

    # ------------------------------------------------------------------ static scope
    def _draw_static(self):
        for it in self.static_items:
            self.plot.removeItem(it)
        self.static_items = []
        R = self.range_km
        th = np.linspace(0, 2 * np.pi, 241)
        step = _ring_step(R)
        for r in np.arange(step, R + 1e-6, step):
            self._static(pg.PlotCurveItem(r * np.sin(th), r * np.cos(th), pen=pg.mkPen(RING, width=1)))
            lab = pg.TextItem(f"{r:g}", color=RING_TEXT, anchor=(0.5, 0.5))
            lab.setFont(QtGui.QFont("Consolas", 7))
            lab.setPos(r * np.sin(np.radians(15)), r * np.cos(np.radians(15)))
            self._static(lab)
        self._static(pg.PlotCurveItem(R * np.sin(th), R * np.cos(th), pen=pg.mkPen(RING_OUTER, width=1.5)))
        for a in range(0, 360, 30):
            ar = np.radians(a)
            self._static(pg.PlotCurveItem([0, R * np.sin(ar)], [0, R * np.cos(ar)],
                                          pen=pg.mkPen(RING, width=1, style=QtCore.Qt.DotLine)))
            lab = pg.TextItem(f"{a:03d}", color=RING_TEXT, anchor=(0.5, 0.5))
            lab.setFont(QtGui.QFont("Consolas", 8))
            lab.setPos(1.08 * R * np.sin(ar), 1.08 * R * np.cos(ar))
            self._static(lab)
        self._fit()
        for s in self.ctrl.sensors:
            reach = min(s.r_max * KM, R)
            az = _body_azimuths(s, np.linspace(*s.az_limits, 64), np.zeros(64))
            pts = np.vstack([[0.0, 0.0], _polar_xy(reach, az), [0.0, 0.0]])
            path = QtGui.QPainterPath()
            path.moveTo(*pts[0])
            for x, y in pts[1:]:
                path.lineTo(x, y)
            self.fov[s.sensor_id].setPath(path)

    def _fit(self, *_):
        """Keep the whole scope (rings + bearing labels) visible whatever the widget shape."""
        R = 1.16 * self.range_km
        w, h = max(self.plot.vb.width(), 1.0), max(self.plot.vb.height(), 1.0)
        xr, yr = (R * w / h, R) if w >= h else (R, R * h / w)
        self.plot.vb.setRange(xRange=(-xr, xr), yRange=(-yr, yr), padding=0)

    def _static(self, item):
        item.setZValue(1)
        self.plot.addItem(item)
        self.static_items.append(item)

    # ------------------------------------------------------------------ dynamic
    def _describe(self, obj):
        if isinstance(obj, tuple):
            return hover.describe_ownship(obj[1])
        if obj.kind == KIND_TARGET:
            return hover.describe_target(obj)
        b = obj.body_xyz
        rng, az, el = coordinates.radar_measurements(b)
        return hover.describe_track(obj, self.palette, (float(rng[0]), float(coordinates.wrap180(az[0])),
                                                        float(el[0])), "aircraft frame, 0° = nose, + right")

    def _hover_entries(self):
        own = self._frame.ownship if self._frame is not None else None
        extra = [(0.0, 0.0, ("own", own), 1)] if own is not None else []
        return list(self.tracks.entries) + extra

    def on_rdp(self, views, stats):
        self._views, self._stats = views, stats
        self.render()

    def on_frame(self, frame):
        """Displayed-time content (truth targets, beam): at most 10 Hz - the PPI is a scope."""
        self._frame = frame
        if self._frame_clock.elapsed() >= 100:
            self._frame_clock.restart()
            self.render()

    def _body_rotation(self, tv):
        cfg = self.ctrl.projector.config_of.get(self.ctrl.projector.sensor_number(tv))
        return cfg.R_BR if cfg is not None else None

    def _body_vector(self, tv, d):
        """Sensor-relative vector (as sent) expressed along the aircraft axes (rotation only)."""
        R = self._body_rotation(tv)
        if R is None:
            return None
        v = R @ rdp_to_frd(*d)
        b = tv.body_xyz
        return b[1] * KM, b[0] * KM, (b[1] + v[1]) * KM, (b[0] + v[0]) * KM

    def _trail_xy(self, tv):
        """History in the aircraft frame: the object's own world path, converted ONCE with the
        ownship pose of the displayed frame (the same pose that places the marker).

        The marker shows where the object is now relative to the aircraft. A trail whose points
        were each converted with the ownship pose *of their own moment* is not the object's path
        at all - it is the history of the relative geometry, so it slides backwards under the
        marker at the ownship's own speed (measured: 0.6 km of drift after 4 s). Converting the
        recorded world positions once, now, keeps the drawn path identical in shape to the real
        one (measured ratio 1.000) and puts its head exactly on the marker.

        Truth targets use their exported X/Y/Z rows; RDP tracks use the world positions
        processing.track_projection placed once per packet. No position is ever integrated from
        velocity, and the ownship motion enters exactly once.
        """
        layers = self.ctrl.layers
        wanted = layers.get("show_target_history", True) if tv.kind == KIND_TARGET \
            else layers.get("show_target_trails", True)
        if not wanted:
            return None
        own = self._frame.ownship if self._frame is not None else None
        w = self._recent(tv)
        if own is not None and w is not None and len(w) and tv.body_xyz is not None:
            # rotate the recorded path into the aircraft frame ONCE and hang it on the marker:
            # the marker is the object's own placement (for an RDP track it comes straight from
            # the packet, with no ownship pose at all), so anchoring there keeps the head of the
            # trail on it even when the displayed time leads the newest packet.
            head = tv.world if tv.world is not None else w[-1]
            body = tv.body_xyz + coordinates.world_to_frame(head, own.R_WB, w)
            return body[:, [1, 0]] * KM                  # plot x = Right, plot y = Forward
        if tv.kind != KIND_TARGET and tv.body_trail is not None and len(tv.body_trail):
            # no exported radar pose at those packet times: the aircraft-frame history is all
            # there is, and it is relative-at-the-time (hover says how a track was placed)
            return tv.body_trail[:, [1, 0]] * KM
        return None

    def _recent(self, tv):
        """The last ``display.trail_seconds`` of a truth trajectory (the whole path for tracks).

        A scope centred on a moving aircraft is not the place for the entire recorded flight: a
        300 s trail sweeps across the whole display as the ownship flies. A tail of the same
        length as the ownship trail reads as history without taking the scope over. The geometry
        is untouched - these are the same recorded rows, just fewer of them.
        """
        w = tv.world_trail
        if w is None or tv.kind != KIND_TARGET:
            return w
        times = tv.meta.get("trail_t")
        if times is None or len(times) != len(w):
            return w
        seconds = float(self.ctrl.settings.get("trail_seconds") or 0.0)
        if seconds <= 0:
            return w
        keep = np.asarray(times) >= float(times[-1]) - seconds
        return w[keep] if keep.any() else w[-1:]

    def _heading(self, tv):
        if tv.kind == KIND_TARGET:
            # aspect relative to the ownship nose: both headings are courses over ground, so the
            # difference is the physical aspect angle (target placement itself is unchanged)
            own = self._frame.ownship if self._frame is not None else None
            return (tv.heading_deg - own.heading_drawn) % 360.0 if own is not None else tv.heading_deg
        R = self._body_rotation(tv)
        if R is None:
            return tv.heading_deg
        v = R @ rdp_to_frd(tv.state.vx, tv.state.vy, tv.state.vz)
        return float(np.degrees(np.arctan2(v[1], v[0])) % 360.0)

    def render(self):
        layers, settings = self.ctrl.layers, self.ctrl.settings
        R = self.range_km
        for sid, fov in self.fov.items():
            fov.setVisible(self.ctrl.sensor_visible.get(sid, True) and layers.get("show_coverage", True))
        self.own.setVisible(layers.get("show_ownship", True))
        self._draw_beams(layers)

        tracks = [tv for tv in visible_tracks(self._views, layers)
                  if tv.body_xyz is not None and np.linalg.norm(tv.body_xyz) * KM <= R
                  and self.show_src.get(self.palette.source_key(tv.kind, tv.state.source), True)]
        targets = []
        if self._frame is not None and layers.get("show_targets", True):
            targets = [tv for tv in self._frame.targets
                       if tv.body_xyz is not None and np.linalg.norm(tv.body_xyz) * KM <= R]
        objs = targets + tracks
        if not objs:
            self.tracks.clear()
            self.pw.scene().update()
        else:
            self.tracks.update(
                objs,
                xy=lambda tv: (tv.body_xyz[1] * KM, tv.body_xyz[0] * KM),
                # the per-kind layer switches are applied in _trail_xy (targets follow
                # "target history", tracks follow "track trails"), as in the 2D view
                layers={**layers, "show_target_trails": True},
                settings=settings, selected=self.ctrl.selected_key,
                dim=lambda tv: tv.stale,
                trail_xy=self._trail_xy,
                vec_xy=lambda tv, d: None if tv.kind == KIND_TARGET else self._body_vector(tv, d),
                heading=self._heading,
                label_fn=track_label)
        self.hover.refresh()

        if self.ctrl.is_playing and self._footer_clock.elapsed() < 100:
            return
        self._footer_clock.restart()
        counts = {k: 0 for k in style.SOURCES}
        for tv in objs:
            counts[self.palette.source_key(tv.kind, tv.state.source)] += 1
        out_of_range = sum(1 for tv in visible_tracks(self._views, layers)
                           if tv.body_xyz is not None and np.linalg.norm(tv.body_xyz) * KM > R)
        t = self._stats.get("last_packet_time")
        self.footer.setText(
            "Aircraft frame (0° = nose) · △ own aircraft (outlined) · □ sensor track · ▲ fused "
            "system track · colour = classification · faded = stale\n"
            f"{counts[style.PRIMARY]} primary · {counts[style.SECONDARY]} secondary · {counts[style.FUSED]} fused"
            f" · {counts[style.TARGET]} target(s)"
            + (f" · {out_of_range} beyond {R:g} km" if out_of_range else "")
            + (f" · last packet time {t:.3f} s" if t is not None else " · no RDP packets yet"))

    def _draw_beams(self, layers):
        states = {s.config.sensor_id: s for s in self._frame.sensors} if self._frame is not None else {}
        for s in self.ctrl.sensors:
            wedge = self.beam[s.sensor_id]
            st = states.get(s.sensor_id)
            on = (st is not None and st.beam is not None and layers.get("show_scan_beam", True)
                  and self.ctrl.sensor_visible.get(s.sensor_id, True))
            wedge.setVisible(on)
            if not on:
                continue
            b = st.beam
            az = _body_azimuths(s, np.linspace(b.left, b.right, 6), np.full(6, b.el_deg))
            pts = np.vstack([[0.0, 0.0], _polar_xy(min(s.r_max * KM, self.range_km), az), [0.0, 0.0]])
            path = QtGui.QPainterPath()
            path.moveTo(*pts[0])
            for x, y in pts[1:]:
                path.lineTo(x, y)
            wedge.setPath(path)
