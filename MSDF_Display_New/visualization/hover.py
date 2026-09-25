"""Hover details for the 3D view, the 2D view and the Aircraft PPI.

The pyqtgraph inspector finds the object nearest the cursor (in screen pixels, among
the objects the view actually drew) and shows its details as a tooltip. It is
refreshed on every frame too, so an object moving under a still cursor keeps its
tooltip up to date. Entries are (x, y, obj) or (x, y, obj, priority); a lower
priority wins when several objects are within reach (markers before trajectories).

Only fields that exist in the data are shown. Derived values (range / bearing from
the ownship, aircraft-frame placement) are labelled as derived. Fused tracks list
their contributing sensors only from the sensorIds of the same RDP packet;
association is never inferred.
"""

from __future__ import annotations

import html
import math

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from models.track_state import KIND_FUSED, KIND_SENSOR, KIND_TARGET
from visualization import style

GREY = "<span style='color:#94a3b8'>{}</span>"


class HoverInspector(QtCore.QObject):
    def __init__(self, plot_widget: pg.PlotWidget, entries, describe, radius_px: float = 16.0):
        """
        entries   callable -> list of (x, y, TrackView) in view coordinates
        describe  callable (TrackView) -> HTML string
        """
        super().__init__(plot_widget)
        self.pw = plot_widget
        self.vb = plot_widget.getPlotItem().vb
        self.entries = entries
        self.describe = describe
        self.radius = radius_px
        self._pos = None
        self._html = None
        self.hovered = None
        self.ring = pg.ScatterPlotItem(pxMode=True, symbol="o", size=32, brush=None,
                                       pen=pg.mkPen(style.HOVER_COLOR, width=1.8))
        self.ring.setZValue(200)
        plot_widget.getPlotItem().addItem(self.ring)
        plot_widget.scene().sigMouseMoved.connect(self._moved)
        plot_widget.viewport().installEventFilter(self)

    def eventFilter(self, obj, event):
        if event.type() == QtCore.QEvent.Leave:
            self._pos = None
            self._hide()
        return False

    def _moved(self, scene_pos):
        self._pos = QtCore.QPointF(scene_pos)
        self.refresh()

    def _hide(self):
        self.hovered = None
        self.ring.clear()
        if self._html is not None:
            QtWidgets.QToolTip.hideText()
            self._html = None

    def pick(self, scene_pos):
        """Nearest (x, y, obj) within the radius of a scene position, or None (priority first).

        The ViewBox maps view -> scene with an axis-aligned affine transform, so all
        candidates are mapped at once (thousands of trajectory points stay cheap)."""
        items = self.entries()
        if not items:
            return None
        p0 = self.vb.mapViewToScene(QtCore.QPointF(0.0, 0.0))
        p1 = self.vb.mapViewToScene(QtCore.QPointF(1.0, 1.0))
        sx, sy = p1.x() - p0.x(), p1.y() - p0.y()
        xy = np.array([(e[0], e[1]) for e in items], float)
        prio = np.array([e[3] if len(e) > 3 else 0 for e in items], float)
        d = np.hypot(p0.x() + xy[:, 0] * sx - scene_pos.x(), p0.y() + xy[:, 1] * sy - scene_pos.y())
        near = np.flatnonzero(d <= self.radius)
        if not len(near):
            return None
        j = near[np.lexsort((d[near], prio[near]))[0]]
        return items[j][0], items[j][1], items[j][2]

    def refresh(self):
        if self._pos is None or not self.vb.sceneBoundingRect().contains(self._pos):
            self._hide()
            return
        hit = self.pick(self._pos)
        if hit is None:
            self._hide()
            return
        x, y, tv = hit
        self.hovered = tv
        self.ring.setData(pos=[(x, y)])
        text = self.describe(tv)
        if text != self._html:
            self._html = text
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), text, self.pw)


# ----------------------------------------------------------------------------
# tooltip content
# ----------------------------------------------------------------------------

def _fmt(v, spec, unit=""):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "—"
    return f"{v:{spec}}{unit}"


def describe_track(tv, palette: style.Palette, geometry: tuple, geometry_note: str) -> str:
    """HTML tooltip for a track.

    geometry       (range_m, azimuth_deg, elevation_deg) in the hosting view's reference
    geometry_note  e.g. "aircraft frame, 0° = nose" - says what the azimuth is measured from
    """
    st = tv.state
    cls = style.classify(st.classification)
    src = palette.source_style(tv.kind, st.source)
    rng, az, el = geometry
    raw = st.classification.strip()
    grey = "<span style='color:#94a3b8'>{}</span>"
    rdp = getattr(tv, "origin", "") == "rdp"
    cls_text = f"<b style='color:{cls.color}'>■ {cls.label}</b>"
    if rdp and not raw:
        cls_text += " " + grey.format("(not in packet)")
    elif raw.upper() != cls.key:
        cls_text += " " + grey.format(f"(file: {html.escape(raw) or '—'})")
    target = html.escape(st.target_id) if st.target_id else grey.format("— (not in packet)" if rdp else "—")
    rows = [
        ("Track ID", html.escape(st.track_id)),
        ("Target ID", target),
        ("Classification", cls_text),
        ("Source", f"{html.escape(st.source)} · {src.label}"),
        ("Range", _fmt(rng, ",.0f", " m")),
        ("Azimuth", _fmt(az, ".1f", "°") + f" <span style='color:#94a3b8'>({geometry_note})</span>"),
        ("Elevation", _fmt(el, "+.1f", "°")),
        ("X / Y / Z", f"{_fmt(st.x, ',.0f')} / {_fmt(st.y, ',.0f')} / {_fmt(st.z, ',.0f')} m"),
        ("Vx / Vy / Vz", f"{_fmt(st.vx, '.1f')} / {_fmt(st.vy, '.1f')} / {_fmt(st.vz, '.1f')} m/s"),
    ]
    if tv.kind == KIND_FUSED:
        sensors = st.contributing_sensors
        if sensors:
            names = ", ".join(f"{html.escape(palette.sensor_names.get(s, s))} ({html.escape(s)})"
                              for s in sensors)
            rows += [("Contributing sensors", names),
                     ("No. of sensors", str(len(sensors)))]
        else:
            rows += [("Contributing sensors", "<i>none in this packet (numSensors = 0)</i>" if rdp
                      else "<i>not provided</i>"),
                     ("No. of sensors", "0")]
    elif tv.kind == KIND_SENSOR and tv.fused_into:
        rows.append(("System track", ", ".join(html.escape(f) for f in tv.fused_into)))
    if rdp:
        rows.append(("Frame", grey.format(html.escape(getattr(tv, "frame", "")))))
        if getattr(tv, "placement", ""):
            rows.append(("Placement", grey.format(html.escape(tv.placement))))
        rows.append(("Packet time", f"{st.timestamp:.6f} (as sent)"))
        rows.append(("Status · age", f"{html.escape(st.status)} · updated {tv.age_s:.1f} s ago"))
    else:
        rows.append(("Status · time", f"{html.escape(st.status) or '—'} · t = {st.timestamp:.2f} s"))
    body = "".join(f"<tr><td style='color:#94a3b8;padding-right:10px'>{k}</td><td>{v}</td></tr>"
                   for k, v in rows)
    return (f"<div style='font-family:Consolas; font-size:9pt'>"
            f"<b>{html.escape(st.track_id)}</b> · {src.label}<table cellspacing='0'>{body}</table></div>")


# ----------------------------------------------------------------------------
# scenario objects (truth targets, trajectories, ownship, radars)
# ----------------------------------------------------------------------------

def _table(title: str, rows: list[tuple[str, str]]) -> str:
    body = "".join(f"<tr><td style='color:#94a3b8;padding-right:10px'>{k}</td><td>{v}</td></tr>"
                   for k, v in rows if v not in (None, ""))
    return (f"<div style='font-family:Consolas; font-size:9pt'><b>{title}</b>"
            f"<table cellspacing='0'>{body}</table></div>")


def _xyz(v, unit=" m", spec=",.0f"):
    return f"{_fmt(float(v[0]), spec)} / {_fmt(float(v[1]), spec)} / {_fmt(float(v[2]), spec)}{unit}"


def describe_target(tv) -> str:
    """Truth target from the Scenario Export (fields as exported + labelled derived values)."""
    st = tv.state
    cls = style.classify(st.classification)
    auth = html.escape(st.classification) if st.classification else "not exported"
    rows = [("Target ID", html.escape(st.target_id)),
            ("Classification", f"<b style='color:{cls.color}'>&#9632; {cls.label}</b> " + GREY.format(f"(Auth: {auth})"))]
    iff_on, iff_key = tv.meta.get("IFF enabled", ""), tv.meta.get("IFF key", "")
    if iff_on:
        rows.append(("IFF", html.escape(iff_on) + (f" &middot; key {html.escape(iff_key)}" if iff_key else "")))
    speed = math.sqrt(st.vx ** 2 + st.vy ** 2 + st.vz ** 2)
    rows += [("X / Y / Z (ENU)", _xyz((st.x, st.y, st.z))),
             ("Altitude (Z)", _fmt(st.z, ",.0f", " m")),
             ("Vx / Vy / Vz", _xyz((st.vx, st.vy, st.vz), " m/s", ".1f")),
             ("Speed &middot; course", f"{speed:.1f} m/s &middot; {tv.heading_deg:.1f}&deg; "
                                       + GREY.format(f"({tv.meta.get('course_source', 'course')})"))]
    # the exported velocity columns disagree with the flown path in this scenario: say so here too
    delta = tv.meta.get("course_delta_deg")
    if delta is not None and math.isfinite(delta) and abs(delta) > 2.0:
        rows.append(("Course from Vx/Vy", f"{tv.meta['heading_from_velocity_deg']:.1f}&deg; "
                                          + GREY.format(f"({delta:+.1f}&deg; from the flown path)")))
    if math.isfinite(tv.range_m):
        rows.append(("From ownship", f"{tv.range_m:,.0f} m &middot; bearing {tv.bearing_deg:.1f}&deg; &middot; "
                                     f"elev {tv.elevation_deg:+.1f}&deg; " + GREY.format("(derived)")))
    rows += [("Time", f"{st.timestamp:.3f} s " + GREY.format("(export row)")),
             ("Source", GREY.format("Scenario Export truth trajectory"))]
    return _table(f"Target {html.escape(st.target_id)} &middot; truth", rows)


def describe_trajectory_point(tv, point, t_s) -> str:
    """One recorded point of a target's trajectory history."""
    rows = [("Target ID", html.escape(tv.state.target_id)),
            ("Time", f"{t_s:.3f} s " + GREY.format("(export row)") if t_s is not None else ""),
            ("X / Y / Z (ENU)", _xyz(point)),
            ("Altitude (Z)", _fmt(float(point[2]), ",.0f", " m"))]
    return _table(f"Trajectory of target {html.escape(tv.state.target_id)}", rows)


def describe_ownship(own) -> str:
    rows = [("X / Y / Z (ENU)", _xyz((own.x, own.y, own.z))),
            ("Altitude (Z)", _fmt(own.z, ",.0f", " m")),
            ("Vx / Vy / Vz", _xyz((own.vx, own.vy, own.vz), " m/s", ".1f")),
            ("Speed", f"{own.speed:.1f} m/s"),
            ("Yaw / Pitch / Roll", f"{own.yaw_deg:.1f}&deg; / {own.pitch_deg:+.1f}&deg; / {own.roll_deg:+.1f}&deg; "
                                   + GREY.format("(exported)")),
            ("Time", f"{own.timestamp:.3f} s " + GREY.format("(export row)"))]
    return _table("Ownship", rows)


def describe_radar(sensor) -> str:
    """A sensor at the displayed time (exported radar pose + sensor properties + beam)."""
    cfg = sensor.config
    rows = [("Sensor", f"{html.escape(cfg.name)} ({html.escape(cfg.sensor_id)}, {html.escape(cfg.sensor_type)})"),
            ("Radar position (ENU)", _xyz(sensor.P_WR)),
            ("Radar yaw / pitch / roll", f"{sensor.yaw_deg:.1f}&deg; / {sensor.pitch_deg:+.1f}&deg; / "
                                         f"{sensor.roll_deg:+.1f}&deg; " + GREY.format("(exported)")),
            ("Mount (aircraft FRD)", f"{_xyz(cfg.mount_xyz_frd, ' m', '.2f')} &middot; ypr "
                                     f"{', '.join(f'{v:g}' for v in cfg.mount_ypr_deg)}&deg;"),
            ("Coverage", f"az {cfg.az_coverage_deg:g}&deg; &times; el {cfg.el_coverage_deg:g}&deg; &middot; "
                         f"R max {cfg.r_max / 1000:g} km"),
            ("Scan time", f"{cfg.scan_time_s:g} s")]
    if sensor.beam is not None:
        b = sensor.beam
        rows.append(("Beam", f"scan {b.scan_id} &middot; dwell {b.dwell_id} &middot; az {b.az_deg:+.1f}&deg; "
                             f"el {b.el_deg:+.1f}&deg; " + GREY.format("(scan schedule)")))
    return _table(html.escape(cfg.name), rows)
