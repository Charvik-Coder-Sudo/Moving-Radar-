"""2D global view: a top-down Cartesian engineering plot.

    horizontal axis  X = East  (km)
    vertical axis    Y = North (km)

Equal aspect, rectangular grid, no range rings, no polar axes and no
0/90/180/270 bearing marks - this is the world picture, not a radar scope.

A quiet ground image (visualization.scenery: terrain relief, water, farmland and the
airfield) sits behind the data as map context; it is deterministic, takes part in no
calculation and can be switched off (Display Layers -> Show Scenery).

Content at the displayed time (live: the packet time; replay: the export timeline):
    Scenario Export   ownship (position, heading, trail, velocity), each sensor's field of
                      regard as its azimuth sector out to its own maximum range (a quiet
                      fill, a clear outline and two range arcs) with the instantaneous scan
                      beam drawn separately as a narrow wedge, and the truth targets with
                      their trajectory history and velocity
    RDP packets       sensor / fused tracks placed in ENU from their radar frame with
                      the exported radar pose at the packet time (see
                      processing.track_projection), with their update history

Hover: targets, tracks, ownship, radars and trajectory points (visualization.hover).
The grid draws two tick levels (major + minor): pyqtgraph's third level multiplied
the paint time of this view by ~4 (measured), for lines too fine to read.
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from models.track_state import KIND_FUSED, KIND_SENSOR, KIND_TARGET
from msdf_math import coordinates
from visualization import hover, scan_beam, scenery, sensor_coverage, style, symbols
from visualization.tracks_2d import TrackItems2D, track_label, visible_tracks

KM = 1e-3
# coverage stays subordinate to the data drawn over it (tracks, targets, ownship)
COVERAGE_FILL_ALPHA = 22
COVERAGE_EDGE_ALPHA = 130


def _polygon(points_m) -> QtGui.QPolygonF:
    return QtGui.QPolygonF([QtCore.QPointF(x * KM, y * KM) for x, y in points_m])


class View2D(QtWidgets.QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.cfg = controller.config
        self.palette = style.Palette(self.cfg)
        self._last_frame = None

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(self._build_toolbar())

        self.pw = pg.PlotWidget(background="#0b1220")
        lay.addWidget(self.pw, 1)
        pl = self.pw.getPlotItem()
        self.plot = pl
        pl.setAspectLocked(True, ratio=1.0)
        pl.showGrid(x=True, y=True, alpha=0.18)
        for ax, lab in (("bottom", "X  —  East (km)"), ("left", "Y  —  North (km)")):
            a = pl.getAxis(ax)
            a.setStyle(maxTickLevel=1)              # major + minor grid only (see module note)
            a.setLabel(lab, color="#cbd5e1")
            a.setPen(pg.mkPen("#475569"))
            a.setTextPen(pg.mkPen("#cbd5e1"))
        # origin cross-hair of the ENU frame
        for ang in (0, 90):
            ln = pg.InfiniteLine(pos=(0, 0), angle=ang, pen=pg.mkPen("#64748b", width=1,
                                                                     style=QtCore.Qt.DotLine))
            ln.setZValue(1)
            pl.addItem(ln)

        # ground context: the same deterministic world the 3D view builds, as one image
        self.map_image = pg.ImageItem()
        self.map_image.setZValue(-10)
        self.map_image.setOpacity(0.85)
        pl.addItem(self.map_image)
        self.map_items: list = []
        self.scenery = None
        self.cov_items, self.beam_items, self.cov_arcs = {}, {}, {}
        self.message = pg.TextItem("", color="#fca5a5", anchor=(0.5, 0.5))
        self.message.setFont(QtGui.QFont("Segoe UI", 11))
        self.message.setZValue(100)
        pl.addItem(self.message)
        self.own_trail = pg.PlotCurveItem(pen=pg.mkPen(style.OWNSHIP_COLOR, width=1.6))
        self.own_heading = pg.PlotCurveItem(pen=pg.mkPen(style.OWNSHIP_COLOR, width=1.2,
                                                         style=QtCore.Qt.DashLine))
        self.own_vel = pg.PlotCurveItem(pen=pg.mkPen(style.VELOCITY_COLOR, width=2.2))
        self.own_acc = pg.PlotCurveItem(pen=pg.mkPen(style.ACCELERATION_COLOR, width=2.2))
        self.own_marker = pg.ScatterPlotItem(pxMode=True)
        for z, it in enumerate((self.own_trail, self.own_heading, self.own_vel, self.own_acc,
                                self.own_marker)):
            it.setZValue(40 + z)
            pl.addItem(it)
        self.objects = TrackItems2D(pl, self.palette, z=20, label_size=8)
        self._hover_extra: list = []
        self.hover = hover.HoverInspector(self.pw, self._hover_entries, self._describe)
        self._build_legend()

        controller.frame_ready.connect(self.on_frame)
        controller.data_loaded.connect(lambda *_: self._loaded())
        controller.loading.connect(lambda msg: self._show_message(msg, "#93c5fd"))
        controller.data_error.connect(lambda msg: self._show_message(
            "SCENARIO EXPORT UNAVAILABLE - nothing is drawn\n\n" + msg, "#fca5a5"))

    # ------------------------------------------------------------------ ui
    def _build_toolbar(self):
        bar = QtWidgets.QFrame(objectName="ViewToolbar")
        bar.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(8, 4, 8, 4)
        h.addWidget(QtWidgets.QLabel("2D GLOBAL VIEW", objectName="ViewTitle"))
        h.addWidget(QtWidgets.QLabel("Cartesian ENU  ·  X East / Y North  ·  1:1", objectName="ViewInfo"))
        h.addStretch(1)
        self.center_btn = QtWidgets.QToolButton(text="Center on Ownship", checkable=True)
        fit = QtWidgets.QToolButton(text="Fit All")
        fit.clicked.connect(self.fit_all)
        h.addWidget(self.center_btn)
        h.addWidget(fit)
        return bar

    def _show_message(self, text, colour):
        self.message.setColor(colour)
        self.message.setText(text)
        vr = self.plot.vb.viewRange()
        self.message.setPos((vr[0][0] + vr[0][1]) / 2, (vr[1][0] + vr[1][1]) / 2)
        self.message.setVisible(True)
        for it in (self.own_trail, self.own_heading, self.own_vel, self.own_acc, self.own_marker):
            it.setVisible(False)
        for it in list(self.cov_items.values()) + list(self.beam_items.values()):
            it.setVisible(False)
        for arcs in self.cov_arcs.values():
            for a in arcs:
                a.setVisible(False)

    def _build_map(self):
        """Terrain as a rendered image plus the airfield outline: context, never a measurement.

        The same visualization.scenery world as the 3D view, so the two views agree. It is a
        picture of invented relief - nothing is derived from it."""
        if self.scenery is not None or self.ctrl.scenario is None:
            return
        lo, hi = self.ctrl.scene_bounds()
        pos = self.ctrl.scenario.ownship.pos
        self.scenery = scenery.Scenery(lo=np.asarray(lo, float), hi=np.asarray(hi, float),
                                       track=(pos[0], pos[-1]))
        margin = 0.35 * max(hi[0] - lo[0], hi[1] - lo[1])
        x0, x1 = lo[0] - margin, hi[0] + margin
        y0, y1 = lo[1] - margin, hi[1] + margin
        n = 360
        xs = np.linspace(x0, x1, n)
        ys = np.linspace(y0, y1, n)
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        Z = self.scenery.height(X, Y)
        rgb = scenery.land_cover(X.ravel(), Y.ravel(), Z.ravel()).reshape(n, n, 3)
        # a little shading from the terrain slope, so relief reads on a flat map
        gy, gx = np.gradient(Z)
        shade = np.clip(1.0 + (gx + gy) / (np.abs(gx).max() + np.abs(gy).max() + 1e-6) * 0.5, 0.65, 1.35)
        rgb = np.clip(rgb * shade[..., None], 0, 255).astype(np.uint8)
        self.map_image.setImage(rgb)
        self.map_image.setRect(QtCore.QRectF(x0 * KM, y0 * KM, (x1 - x0) * KM, (y1 - y0) * KM))
        af = self.scenery.airfield
        a = np.array([np.sin(np.radians(af.heading_deg)), np.cos(np.radians(af.heading_deg))])
        ends = np.array([af.center - a * af.runway_length_m / 2, af.center + a * af.runway_length_m / 2])
        rwy = pg.PlotCurveItem(ends[:, 0] * KM, ends[:, 1] * KM,
                               pen=pg.mkPen(scenery.MARKING, width=3))
        rwy.setZValue(-9)
        self.plot.addItem(rwy)
        lab = pg.TextItem(f"AIRFIELD {af.designators[0]}/{af.designators[1]}", color="#94a3b8",
                          anchor=(0.5, -0.4))
        lab.setFont(QtGui.QFont("Consolas", 8))
        lab.setPos(af.center[0] * KM, af.center[1] * KM)
        lab.setZValue(-9)
        self.plot.addItem(lab)
        self.map_items = [rwy, lab]

    def set_map_visible(self, on: bool):
        self.map_image.setVisible(on)
        for it in self.map_items:
            it.setVisible(on)

    def _loaded(self):
        self._build_map()
        self.message.setVisible(False)
        self.fit_all()

    def _build_legend(self):
        """In-plot legend: ownship, vectors and each sensor's projected coverage / beam."""
        leg = pg.LegendItem(offset=(8, 8), labelTextColor="#cbd5e1", labelTextSize="7pt", verSpacing=-6,
                            brush=pg.mkBrush(11, 18, 32, 200), pen=pg.mkPen("#334155"))
        leg.setParentItem(self.plot.vb)
        leg.addItem(pg.ScatterPlotItem(symbol=symbols.AIRCRAFT, size=12, pen=pg.mkPen("#0f172a"),
                                       brush=pg.mkBrush(style.OWNSHIP_COLOR)), "Ownship")
        leg.addItem(pg.PlotDataItem(pen=pg.mkPen(style.VELOCITY_COLOR, width=2)), "Velocity vector")
        leg.addItem(pg.ScatterPlotItem(symbol=symbols.AIRCRAFT, size=12, pen=pg.mkPen(style.class_color("UNKNOWN")),
                                       brush=pg.mkBrush(style.AIRCRAFT_ICON_FILL)), "Target (truth) + trajectory")
        for key in (style.PRIMARY, style.SECONDARY, style.FUSED):
            src = style.SOURCES[key]
            leg.addItem(pg.ScatterPlotItem(symbol=symbols.symbol(src, 0.0), size=11, brush=None,
                                           pen=pg.mkPen(style.class_color(""), width=1.4)), f"RDP {src.label}")
        for s in self.ctrl.sensors:
            col = QtGui.QColor(self.palette.sensor_color(s.sensor_id))
            leg.addItem(pg.PlotDataItem(pen=pg.mkPen(col, width=6)),
                        f"{s.name}: field of regard (sector) · scan beam (narrow)")
        self.legend = leg
        # the in-plot legend only when the plot is large enough not to be covered by it
        self.plot.vb.sigResized.connect(
            lambda *_: leg.setVisible(self.plot.vb.width() > 520 and self.plot.vb.height() > 330))

    def fit_all(self):
        lo, hi = self.ctrl.scene_bounds()
        pts = [tg.pos[:, :2] for tg in (self.ctrl.scenario.targets if self.ctrl.scenario is not None else [])]
        pts += [np.array([tv.world[:2] for tv in self.ctrl.world_tracks()])] if self.ctrl.world_tracks() else []
        if pts:
            allp = np.vstack(pts)
            lo = np.minimum(lo[:2], allp.min(axis=0))
            hi = np.maximum(hi[:2], allp.max(axis=0))
        pad = 0.08 * max(hi[0] - lo[0], hi[1] - lo[1], 1000.0)
        self.plot.setRange(xRange=((lo[0] - pad) * KM, (hi[0] + pad) * KM),
                           yRange=((lo[1] - pad) * KM, (hi[1] + pad) * KM), padding=0)

    def _poly_item(self, store, key, color, alpha_fill, alpha_line, z):
        it = store.get(key)
        if it is None:
            it = QtWidgets.QGraphicsPolygonItem()
            c = QtGui.QColor(color)
            f = QtGui.QColor(c)
            f.setAlpha(alpha_fill)
            ln = QtGui.QColor(c)
            ln.setAlpha(alpha_line)
            it.setBrush(QtGui.QBrush(f))
            pen = QtGui.QPen(ln)
            pen.setCosmetic(True)
            pen.setWidthF(1.4)
            it.setPen(pen)
            it.setZValue(z)
            self.plot.addItem(it)
            store[key] = it
        return it

    # ------------------------------------------------------------------ frames
    def on_frame(self, frame):
        self._last_frame = frame
        if self.isVisible():
            self.render_frame(frame)

    def showEvent(self, e):
        super().showEvent(e)
        if self._last_frame is not None:
            self.render_frame(self._last_frame)

    def render_frame(self, frame):
        layers, settings = self.ctrl.layers, self.ctrl.settings
        own = frame.ownship
        self.message.setVisible(False)
        draw_range_cfg = self.cfg.get("display", {}).get("coverage_draw_range_m")

        # ---- coverage & beam footprints -------------------------------------
        states = {s.config.sensor_id: s for s in frame.sensors}
        for cfg in self.ctrl.sensors:
            sid = cfg.sensor_id
            st = states.get(sid)
            on = st is not None and self.ctrl.sensor_visible.get(sid, True)
            rng = draw_range_cfg or cfg.r_max
            # field of regard, in plan: the sensor's own azimuth sector out to its own range
            cov = self._poly_item(self.cov_items, sid, self.palette.sensor_color(sid),
                                  COVERAGE_FILL_ALPHA, COVERAGE_EDGE_ALPHA, 2)
            cov.setVisible(on and layers.get("show_coverage", True))
            arcs = self.cov_arcs.setdefault(sid, [])
            if cov.isVisible():
                outline, rings = sensor_coverage.ground_sector_xy(st, rng)
                cov.setPolygon(_polygon(outline))
                while len(arcs) < len(rings):
                    a = pg.PlotCurveItem(pen=pg.mkPen(self.palette.sensor_color(sid), width=1.0,
                                                      style=QtCore.Qt.DotLine))
                    a.setZValue(2)
                    a.setOpacity(0.45)
                    self.plot.addItem(a)
                    arcs.append(a)
                for a, ring in zip(arcs, rings):
                    a.setData(ring[:, 0] * KM, ring[:, 1] * KM)
                    a.setVisible(True)
            else:
                for a in arcs:
                    a.setVisible(False)
            beam = self._poly_item(self.beam_items, sid, self.palette.beam_color(sid), 120, 230, 3)
            beam.setVisible(on and st.beam is not None and layers.get("show_scan_beam", True))
            if beam.isVisible():
                beam.setPolygon(_polygon(scan_beam.beam_footprint_xy(st, rng)))

        self.set_map_visible(bool(layers.get("show_scenery", True)))

        # ---- ownship ----------------------------------------------------------
        show_own = own is not None and layers.get("show_ownship", True)
        for it in (self.own_trail, self.own_heading, self.own_vel, self.own_acc, self.own_marker):
            it.setVisible(show_own)
        if show_own:
            # drawn along the flown path (own.heading_drawn); the exported Yaw stays in the panel
            spot = dict(pos=(own.x * KM, own.y * KM), size=30,
                        symbol=symbols.rotated(symbols.AIRCRAFT, own.heading_drawn),
                        pen=pg.mkPen("#0f172a", width=1), brush=pg.mkBrush(style.OWNSHIP_COLOR))
            self.own_marker.setData([spot])
            tr = frame.ownship_trail
            if len(tr) > 1:
                self.own_trail.setData(tr[:, 0] * KM, tr[:, 1] * KM)
            else:
                self.own_trail.setData([], [])
            self.own_trail.setVisible(layers.get("show_ownship_trail", True))
            psi = np.radians(own.heading_drawn)
            L = 0.12 * self._view_span_m()
            self.own_heading.setData([own.x * KM, (own.x + L * np.sin(psi)) * KM],
                                     [own.y * KM, (own.y + L * np.cos(psi)) * KM])
            v = _course_velocity(own) * settings["velocity_vector_seconds"]
            self.own_vel.setData([own.x * KM, (own.x + v[0]) * KM], [own.y * KM, (own.y + v[1]) * KM])
            self.own_vel.setVisible(layers.get("show_velocity_vectors", True))
            a = own.acceleration * settings["acceleration_vector_scale_s2"]
            ok = np.all(np.isfinite(a))
            self.own_acc.setData([own.x * KM, (own.x + a[0]) * KM] if ok else [],
                                 [own.y * KM, (own.y + a[1]) * KM] if ok else [])
            self.own_acc.setVisible(layers.get("show_acceleration_vectors", True))

        self._draw_objects(frame)

        if self.center_btn.isChecked() and own is not None:
            vr = self.plot.vb.viewRange()
            hw, hh = (vr[0][1] - vr[0][0]) / 2, (vr[1][1] - vr[1][0]) / 2
            self.plot.setRange(xRange=(own.x * KM - hw, own.x * KM + hw),
                               yRange=(own.y * KM - hh, own.y * KM + hh), padding=0)

    # ------------------------------------------------------------------ targets / tracks
    def _draw_objects(self, frame):
        layers, settings = self.ctrl.layers, self.ctrl.settings
        objs = list(frame.targets) if layers.get("show_targets", True) else []
        objs += visible_tracks(self.ctrl.world_tracks(frame.t_ms), layers)

        def trail(tv):
            if tv.kind == KIND_TARGET:
                ok = layers.get("show_target_history", True)
            else:
                ok = layers.get("show_target_trails", True)
            return tv.world_trail[:, :2] * KM if ok and tv.world_trail is not None and len(tv.world_trail) else None

        def course(tv):
            if tv.kind == KIND_TARGET:
                return tv.heading_deg                        # course over ground (flown path)
            w = tv.world_trail                               # RDP: direction between placed positions
            if w is not None and len(w) >= 2:
                d = w[-1] - w[-2]
                return float(np.degrees(np.arctan2(d[0], d[1])) % 360.0)
            return 0.0

        self.objects.update(
            objs, xy=lambda tv: (tv.world[0] * KM, tv.world[1] * KM),
            layers={**layers, "show_target_trails": True}, settings=settings,
            selected=self.ctrl.selected_key, dim=lambda tv: tv.stale, trail_xy=trail, heading=course,
            # velocity vectors only where the velocity is world ENU (truth); RDP velocities are
            # relative to the moving radar and are not transformed
            vec_xy=lambda tv, d: ((tv.world[0] * KM, tv.world[1] * KM, (tv.world[0] + d[0]) * KM,
                                   (tv.world[1] + d[1]) * KM) if tv.kind == KIND_TARGET else None),
            label_fn=lambda tv: "" if tv.kind == KIND_SENSOR else track_label(tv))
        extra = []
        own = frame.ownship
        if own is not None and layers.get("show_ownship", True):
            extra.append((own.x * KM, own.y * KM, ("own", own), 1))
            for s in frame.sensors:
                extra.append((s.P_WR[0] * KM, s.P_WR[1] * KM, ("radar", s), 2))
        if layers.get("show_target_history", True):
            for tv in objs:
                if tv.kind == KIND_TARGET and tv.world_trail is not None:
                    ts = tv.meta.get("trail_t")
                    for k, pt in enumerate(tv.world_trail[:-1]):
                        extra.append((pt[0] * KM, pt[1] * KM, ("traj", tv, k, ts[k] if ts is not None else None), 3))
        self._hover_extra = extra
        self.hover.refresh()

    def _hover_entries(self):
        return list(self.objects.entries) + self._hover_extra

    def _describe(self, obj):
        if isinstance(obj, tuple):
            if obj[0] == "own":
                return hover.describe_ownship(obj[1])
            if obj[0] == "radar":
                return hover.describe_radar(obj[1])
            tv, k, t_s = obj[1], obj[2], obj[3]
            return hover.describe_trajectory_point(tv, tv.world_trail[k], t_s)
        if obj.kind == KIND_TARGET:
            return hover.describe_target(obj)
        return hover.describe_track(obj, self.palette,
                                    (obj.range_m, float(coordinates.wrap180(obj.bearing_deg)), obj.elevation_deg),
                                    "sensor-relative, 0° = boresight, + right")

    def _view_span_m(self):
        vr = self.plot.vb.viewRange()
        return (vr[0][1] - vr[0][0]) / KM


def _course_velocity(own) -> np.ndarray:
    """Ownship velocity along the flown path (the exported Vx/Vy are swapped upstream)."""
    if not np.isfinite(own.course_deg):
        return own.velocity
    speed = own.path_speed_mps if np.isfinite(own.path_speed_mps) else own.speed
    c, g = np.radians(own.course_deg), np.radians(own.climb_deg)
    return np.array([np.cos(g) * np.sin(c), np.cos(g) * np.cos(c), np.sin(g)]) * speed
