"""Left navigation panel: open / maximise a view, display layers, sensors and settings."""

from __future__ import annotations

from PySide6 import QtCore, QtWidgets

from widgets.legend_widget import LegendWidget
from widgets.sensor_panel import SensorPanel

# (key, label, tooltip) - Scenario Export = ownship / sensors / truth targets; RDP = tracks
LAYERS = [
    ("show_ownship", "Show Ownship", "World views and PPI centre (Scenario Export)"),
    ("show_ownship_trail", "Show Ownship Trail", "World views (Scenario Export)"),
    ("show_sensors", "Show Sensors", "Radar boresights in the 3D view (Scenario Export)"),
    ("show_coverage", "Show Coverage", "Coverage volume / footprint / PPI field of view"),
    ("show_scan_beam", "Show Scan Beam", "Exported scan-beam dwell - world views and PPI"),
    ("show_range_rings", "Show Range Rings", "Ground range rings around the ownship (3D)"),
    ("show_scenery", "Show Scenery", "Terrain features, airfield, settlements and roads (3D, visualization only)"),
    ("show_targets", "Show Targets (truth)", "Scenario truth targets - 3D, 2D and PPI"),
    ("show_target_history", "Show Target Trajectories", "Truth trajectory history - 3D and 2D"),
    ("show_sensor_tracks", "Show Sensor Tracks", "RDP sensor tracks - all views and table"),
    ("show_fused_tracks", "Show Fused Tracks", "RDP fused / system tracks - all views and table"),
    ("show_target_trails", "Show Track Trails", "Update history of RDP tracks - all views"),
    ("show_velocity_vectors", "Show Velocity Vectors", "Ownship and targets (world views), RDP tracks (PPI)"),
    ("show_acceleration_vectors", "Show Acceleration Vectors", "RDP tracks (PPI); the export has none"),
    ("show_target_labels", "Show Labels", "Target and track labels"),
]


class LayersPage(QtWidgets.QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(QtWidgets.QLabel("DISPLAY LAYERS", objectName="PanelTitle"))
        self.boxes = {}
        for key, text, tip in LAYERS:
            cb = QtWidgets.QCheckBox(text)
            cb.setChecked(bool(controller.layers.get(key, True)))
            cb.toggled.connect(lambda on, k=key: controller.set_layer(k, on))
            cb.setToolTip(tip)
            lay.addWidget(cb)
            self.boxes[key] = cb
        row = QtWidgets.QHBoxLayout()
        for text, on in (("All", True), ("None", False)):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(lambda _=False, v=on: [cb.setChecked(v) for cb in self.boxes.values()])
            row.addWidget(b)
        lay.addLayout(row)
        lay.addSpacing(6)
        lay.addWidget(LegendWidget(columns=2, include_target=False))
        lay.addStretch(1)


class SettingsPage(QtWidgets.QWidget):
    reload_requested = QtCore.Signal()
    report_requested = QtCore.Signal()
    sound_toggled = QtCore.Signal(bool)
    volume_changed = QtCore.Signal(float)       # 0..1

    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(QtWidgets.QLabel("SETTINGS", objectName="PanelTitle"))
        form = QtWidgets.QFormLayout()
        form.setLabelAlignment(QtCore.Qt.AlignLeft)
        s = controller.settings

        def spin(key, lo, hi, step, dec, suffix, scale=1.0, zero_text=None):
            w = QtWidgets.QDoubleSpinBox()
            w.setRange(lo, hi)
            w.setSingleStep(step)
            w.setDecimals(dec)
            w.setSuffix(suffix)
            val = s.get(key)
            w.setValue(0.0 if val is None else float(val) / scale)
            if zero_text:
                w.setSpecialValueText(zero_text)
            w.setKeyboardTracking(False)
            w.valueChanged.connect(
                lambda v: controller.set_setting(key, (None if (zero_text and v == 0) else v * scale)))
            return w

        form.addRow("Ownship trail", spin("trail_seconds", 0, 3600, 10, 0, " s"))
        form.addRow("Velocity vector", spin("velocity_vector_seconds", 1, 600, 5, 0, " s of flight"))
        form.addRow("Acceleration vector", spin("acceleration_vector_scale_s2", 1, 5000, 10, 0, " s²"))
        form.addRow("Aircraft symbol scale", spin("aircraft_scale", 1, 400, 5, 0, " ×"))
        form.addRow("Coverage drawn to", spin("coverage_draw_range_m", 0, 400, 5, 0, " km",
                                              scale=1000.0, zero_text="full Rmax"))
        lay.addLayout(form)
        hint = QtWidgets.QLabel("Vectors: velocity arrow = V × seconds; acceleration arrow = A × s². "
                                "The aircraft model is drawn enlarged by the symbol scale.",
                                objectName="Hint")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        # aircraft background ambience (3D view only; visual immersion, linked to no data)
        audio = controller.config.get("audio", {})
        lay.addWidget(QtWidgets.QLabel("AIRCRAFT SOUND (3D VIEW)", objectName="SectionLabel"))
        self.sound = QtWidgets.QCheckBox("Aircraft background sound")
        self.sound.setChecked(bool(audio.get("enabled", True)))
        self.sound.setToolTip("Ambience while the 3D view is on screen. It represents no data.")
        self.sound.toggled.connect(self.sound_toggled)
        lay.addWidget(self.sound)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Volume"))
        self.volume = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.volume.setRange(0, 100)
        self.volume.setValue(int(round(100 * float(audio.get("volume", 0.25)))))
        self.volume_label = QtWidgets.QLabel(f"{self.volume.value()} %")
        self.volume.valueChanged.connect(lambda v: (self.volume_label.setText(f"{v} %"),
                                                    self.volume_changed.emit(v / 100.0)))
        row.addWidget(self.volume, 1)
        row.addWidget(self.volume_label)
        lay.addLayout(row)
        src = QtWidgets.QLabel(f"Scenario Export: {controller.export.root}\n"
                               f"RDP: UDP {controller.rdp.host}:{controller.rdp.port}"
                               + ("" if controller.rdp.enabled else " (disabled)"), objectName="Hint")
        src.setWordWrap(True)
        lay.addWidget(src)
        b1 = QtWidgets.QPushButton("Reload Scenario Export")
        b1.clicked.connect(self.reload_requested)
        b2 = QtWidgets.QPushButton("Validation Report…")
        b2.clicked.connect(self.report_requested)
        lay.addWidget(b1)
        lay.addWidget(b2)
        lay.addStretch(1)


class NavigationPanel(QtWidgets.QFrame):
    view_selected = QtCore.Signal(str)          # "3d" / "2d" / "ppi": open / maximise that view

    def __init__(self, controller, parent=None):
        super().__init__(parent, objectName="NavPanel")
        self.ctrl = controller
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(4)
        lay.addWidget(QtWidgets.QLabel("NAVIGATION", objectName="PanelTitle"))

        # the world view shows 3D or 2D; the Aircraft PPI can be opened in its own window
        self.view_group = QtWidgets.QButtonGroup(self)
        self.view_btns = {}
        for mode, text in (("3d", "3D View"), ("2d", "2D View"), ("ppi", "Aircraft PPI  ⤢")):
            b = QtWidgets.QPushButton(text, checkable=True, objectName="NavButton")
            if mode == "ppi":
                b.setToolTip("Open the Aircraft PPI in its own maximised window (close it or Esc to return)")
            else:
                b.setChecked(mode == controller.view_mode)
                self.view_group.addButton(b)
            b.clicked.connect(lambda _=False, m=mode: self.view_selected.emit(m))
            lay.addWidget(b)
            self.view_btns[mode] = b

        line = QtWidgets.QFrame(frameShape=QtWidgets.QFrame.HLine, objectName="Sep")
        lay.addWidget(line)

        self.page_group = QtWidgets.QButtonGroup(self)
        self.stack = QtWidgets.QStackedWidget()
        self.layers = LayersPage(controller)
        self.sensors = SensorPanel(controller)
        self.settings = SettingsPage(controller)
        for i, (text, page) in enumerate((("Tracks", self.layers), ("Sensors", self.sensors),
                                          ("Settings", self.settings))):
            b = QtWidgets.QPushButton(text, checkable=True, objectName="NavButton")
            b.setChecked(i == 0)
            self.page_group.addButton(b, i)
            b.clicked.connect(lambda _=False, j=i: self.stack.setCurrentIndex(j))
            lay.addWidget(b)
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
            scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            scroll.setWidget(page)
            self.stack.addWidget(scroll)
        lay.addSpacing(6)
        lay.addWidget(self.stack, 1)

    def show_page(self, i: int):
        self.page_group.button(i).setChecked(True)
        self.stack.setCurrentIndex(i)
