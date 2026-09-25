"""Track Details table - the tracks received from the RDP.

One row per visible RDP track: each fused / system track (Xfused) and each
sensor track carried in its packet (sensorIds / sensorStates), filtered by the
same layer switches as the PPI, so the table lists exactly what the PPI draws.

State values are the packet values, in the RDP's sensor-relative frame
(x Right, y Forward, z Up). Columns marked * are derived by the display. The
packets carry no classification and no target id: those show as Unknown and "—".
"""

from __future__ import annotations

import math

from PySide6 import QtCore, QtGui, QtWidgets

from models.track_state import KIND_FUSED, KIND_SENSOR
from visualization import style, symbols
from visualization.tracks_2d import visible_tracks


def _association(tv) -> str:
    """Fused: sensorIds in the same packet. Sensor: the system track that carries it."""
    st = tv.state
    if tv.kind == KIND_FUSED:
        n = len(st.contributing_sensors)
        return (", ".join(st.contributing_sensors) + f"  ({n} sensor{'s' * (n != 1)})") if n else "none (0)"
    if tv.kind == KIND_SENSOR and tv.fused_into:
        return "in " + ", ".join(tv.fused_into)
    return ""


# (header, getter, format, tooltip)
COLUMNS = [
    ("Track ID", lambda tv: tv.state.track_id, None,
     "Fused: ST<systemTrackId>. Sensor: ST<systemTrackId>:S<sensorId> (from the packet)"),
    ("Target ID", lambda tv: tv.state.target_id or "—", None, "Not in the RDP packet"),
    ("Classification", lambda tv: style.classify(tv.state.classification).label, None,
     "Not in the RDP packet - shown as Unknown"),
    ("Source", lambda tv: tv.state.source, None, "FUSED (Xfused) or the sensor of sensorIds; the icon shows the type"),
    ("X (m)", lambda tv: tv.state.x, "{:.1f}", "x: Right (sensor-relative, packet)"),
    ("Y (m)", lambda tv: tv.state.y, "{:.1f}", "y: Forward (sensor-relative, packet)"),
    ("Z (m)", lambda tv: tv.state.z, "{:.1f}", "z: Up (sensor-relative, packet)"),
    ("Vx (m/s)", lambda tv: tv.state.vx, "{:.2f}", "packet"),
    ("Vy (m/s)", lambda tv: tv.state.vy, "{:.2f}", "packet"),
    ("Vz (m/s)", lambda tv: tv.state.vz, "{:.2f}", "packet"),
    ("Speed* (m/s)", lambda tv: tv.state.speed, "{:.1f}", "Derived: |V|"),
    ("w (rad/s)", lambda tv: tv.state.omega, "{:+.5f}", "Turn rate w (packet)"),
    ("Ax (m/s²)", lambda tv: tv.state.ax, "{:+.3f}", "packet"),
    ("Ay (m/s²)", lambda tv: tv.state.ay, "{:+.3f}", "packet"),
    ("Az (m/s²)", lambda tv: tv.state.az, "{:+.3f}", "packet"),
    ("Range* (m)", lambda tv: tv.range_m, "{:.0f}", "Derived: |(x, y, z)|, from the sensor"),
    ("Azimuth* (°)", lambda tv: tv.bearing_deg, "{:.2f}", "Derived: atan2(x, y); 0 = boresight, + right"),
    ("Elevation* (°)", lambda tv: tv.elevation_deg, "{:+.2f}", "Derived: atan2(z, horizontal)"),
    ("Status*", lambda tv: tv.state.status, None, "Display lifetime: ACTIVE or STALE (stale timeout)"),
    ("Sensors / system", _association, None, "Association carried in the packet - never inferred"),
    ("Packet time", lambda tv: tv.state.timestamp, "{:.6f}", "SystemTrack 'time' exactly as sent (RDP: seconds)"),
    ("Age* (s)", lambda tv: tv.age_s, "{:.1f}", "Wall-clock seconds since this system track was last updated"),
]
COL_CLASS = 2
COL_SOURCE = 3


class TrackTableModel(QtCore.QAbstractTableModel):
    def __init__(self, palette: style.Palette):
        super().__init__()
        self.rows: list = []
        self.palette = palette
        # indicators come from the central style, built once
        self.class_icons = {k: QtGui.QIcon(symbols.swatch_pixmap(cs.color, 12))
                            for k, cs in style.CLASSIFICATIONS.items()}
        self.source_icons = {k: QtGui.QIcon(symbols.icon_pixmap(src, style.TYPE_ICON_COLOR, 14))
                             for k, src in style.SOURCES.items()}

    def rowCount(self, parent=QtCore.QModelIndex()):
        return 0 if parent.isValid() else len(self.rows)

    def columnCount(self, parent=QtCore.QModelIndex()):
        return 0 if parent.isValid() else len(COLUMNS)

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if orientation != QtCore.Qt.Horizontal:
            return None
        if role == QtCore.Qt.DisplayRole:
            return COLUMNS[section][0]
        if role == QtCore.Qt.ToolTipRole:
            return COLUMNS[section][3]
        return None

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid():
            return None
        tv = self.rows[index.row()]
        header, get, fmt, _ = COLUMNS[index.column()]
        value = get(tv)
        if role == QtCore.Qt.DisplayRole:
            if fmt is None:
                return str(value)
            if value is None or (isinstance(value, float) and not math.isfinite(value)):
                return "—"
            return fmt.format(value)
        if role == QtCore.Qt.UserRole:            # sort key
            if isinstance(value, float) and not math.isfinite(value):
                return float("-inf")
            return value
        if role == QtCore.Qt.TextAlignmentRole:
            return int(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter) if fmt else \
                int(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        if role == QtCore.Qt.DecorationRole:
            if index.column() == COL_CLASS:
                return self.class_icons[style.classify(tv.state.classification).key]
            if index.column() == COL_SOURCE:
                return self.source_icons[self.palette.source_key(tv.kind, tv.state.source)]
            return None
        if role == QtCore.Qt.ForegroundRole and index.column() == COL_CLASS:
            return QtGui.QBrush(QtGui.QColor(style.class_color(tv.state.classification)))
        if role == QtCore.Qt.ToolTipRole:
            cs = style.classify(tv.state.classification)
            return (f"{tv.state.source}/{tv.state.track_id} · "
                    f"{self.palette.source_style(tv.kind, tv.state.source).label} · "
                    f"{cs.label} (classification not in packet) · "
                    f"packet time {tv.state.timestamp:.6f} · updated {tv.age_s:.1f} s ago · {tv.frame}")
        return None

    def set_rows(self, rows):
        self.beginResetModel()
        self.rows = rows
        self.endResetModel()

    def key_at(self, row):
        return self.rows[row].key if 0 <= row < len(self.rows) else None


class TrackTable(QtWidgets.QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.model = TrackTableModel(style.Palette(controller.config))
        self.proxy = QtCore.QSortFilterProxyModel(self)
        self.proxy.setSourceModel(self.model)
        self.proxy.setSortRole(QtCore.Qt.UserRole)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        head = QtWidgets.QFrame(objectName="ViewToolbar")
        h = QtWidgets.QHBoxLayout(head)
        h.setContentsMargins(8, 3, 8, 3)
        h.addWidget(QtWidgets.QLabel("TRACK DETAILS", objectName="ViewTitle"))
        self.count = QtWidgets.QLabel("", objectName="ViewInfo")
        h.addWidget(self.count)
        h.addStretch(1)
        info = QtWidgets.QLabel("RDP SystemTrack packets · sensor-relative x Right / y Forward / z Up · "
                                "* derived by the display", objectName="ViewInfo")
        info.setToolTip(info.text())
        for lab in (self.count, info):            # never let header text dictate the panel's width
            lab.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        h.addWidget(info, 1)
        lay.addWidget(head)

        self.view = QtWidgets.QTableView()
        self.view.setModel(self.proxy)
        self.view.setSortingEnabled(True)
        self.view.sortByColumn(0, QtCore.Qt.AscendingOrder)
        self.view.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.view.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.view.setAlternatingRowColors(True)
        self.view.verticalHeader().setVisible(False)
        self.view.verticalHeader().setDefaultSectionSize(20)
        self.view.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Interactive)
        self._sized = False
        self.view.horizontalHeader().setStretchLastSection(True)
        self.view.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        lay.addWidget(self.view, 1)
        self.view.selectionModel().selectionChanged.connect(self._on_select)

        self._restoring = False
        self._last_update = QtCore.QElapsedTimer()
        self._last_update.start()
        self._period_ms = 1000.0 / float(controller.config.get("display", {}).get("table_rate_hz", 5))
        self._views: list = []
        controller.rdp_updated.connect(self.on_rdp)
        controller.layers_changed.connect(lambda: self.on_rdp(self._views, None, force=True))

    def on_rdp(self, views, stats, force=False):
        self._views = views
        if not force and self._last_update.elapsed() < self._period_ms:
            return
        self._last_update.restart()
        rows = visible_tracks(views, self.ctrl.layers)
        self._restoring = True
        self.model.set_rows(rows)
        self._restore_selection()
        self._restoring = False
        if not self._sized and rows:
            self.view.resizeColumnsToContents()     # once; per-refresh sizing is very costly
            self._sized = True
        n_fused = sum(1 for tv in rows if tv.kind == KIND_FUSED)
        self.count.setText(f"{len(rows)} track(s): {n_fused} fused, {len(rows) - n_fused} sensor")

    def _restore_selection(self):
        key = self.ctrl.selected_key
        if key is None:
            return
        for r, tv in enumerate(self.model.rows):
            if tv.key == key:
                idx = self.proxy.mapFromSource(self.model.index(r, 0))
                self.view.selectionModel().select(
                    idx, QtCore.QItemSelectionModel.ClearAndSelect | QtCore.QItemSelectionModel.Rows)
                return

    def _on_select(self, *_):
        if self._restoring:
            return
        idxs = self.view.selectionModel().selectedRows()
        key = self.model.key_at(self.proxy.mapToSource(idxs[0]).row()) if idxs else None
        self.ctrl.select(key)
