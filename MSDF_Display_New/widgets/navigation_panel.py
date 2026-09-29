"""Left navigation panel: open / maximise a view, drive the camera, display layers,
sensors and settings.

The layer switches are grouped by what they belong to rather than listed flat, and each
group folds away behind one switch: the operator turns a whole group on or off without
reading fifteen lines. The groups cover every layer exactly once (GROUPS vs LAYERS, checked
by tests/test_sidebar.py), so a new layer cannot quietly go missing from the panel.
"""

from __future__ import annotations

from PySide6 import QtCore, QtWidgets

from visualization.view_3d import CAMERA_PRESETS
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


# (title, layer keys, open by default). Every key of LAYERS appears exactly once.
GROUPS = [
    ("OWNSHIP", ["show_ownship", "show_ownship_trail", "show_range_rings"], True),
    ("TRUTH TARGETS", ["show_targets", "show_target_history"], True),
    ("RDP TRACKS", ["show_sensor_tracks", "show_fused_tracks", "show_target_trails"], True),
    ("SENSORS", ["show_sensors", "show_coverage", "show_scan_beam"], False),
    ("LABELS & VECTORS", ["show_target_labels", "show_velocity_vectors",
                          "show_acceleration_vectors"], False),
    ("SCENE", ["show_scenery"], False),
]


class Section(QtWidgets.QWidget):
    """A titled group of layer switches that folds away behind one switch of its own.

    The header switch reports the group - on, off, or partly on - and switching it turns the
    whole group on, or off if it is already fully on. Folding hides the rows, never the
    header switch, so a group can be turned off without being opened."""

    def __init__(self, title: str, boxes: list[QtWidgets.QCheckBox], opened: bool, parent=None):
        super().__init__(parent)
        self.boxes = boxes
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(1)
        head = QtWidgets.QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        head.setSpacing(4)
        self.switch = QtWidgets.QCheckBox(objectName="GroupSwitch")
        self.switch.setTristate(True)
        self.switch.setToolTip(f"Turn the whole {title} group on or off")
        self.switch.clicked.connect(self._switch_group)
        head.addWidget(self.switch)
        # a push button, not a tool button: only QPushButton honours text-align in a
        # stylesheet, and a centred group title reads as a clipped one in a narrow panel.
        # "&" in a button label is a mnemonic, so a title carrying one doubles it up.
        self.title = title
        self.fold = QtWidgets.QPushButton(self._label(True), objectName="SectionTitle")
        self.fold.setCheckable(True)
        self.fold.setChecked(opened)
        self.fold.setFlat(True)
        self.fold.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.fold.toggled.connect(self._set_open)
        head.addWidget(self.fold, 1)
        lay.addLayout(head)
        self.body = QtWidgets.QWidget()
        body = QtWidgets.QVBoxLayout(self.body)
        body.setContentsMargins(14, 0, 0, 2)
        body.setSpacing(1)
        for cb in boxes:
            cb.toggled.connect(self.sync)
            body.addWidget(cb)
        lay.addWidget(self.body)
        self._set_open(opened)
        self.sync()

    def _label(self, opened):
        return ("▾  " if opened else "▸  ") + self.title.replace("&", "&&")

    def _set_open(self, opened):
        self.body.setVisible(bool(opened))
        self.fold.setText(self._label(bool(opened)))
        self.fold.setToolTip("Fold this group away" if opened else "Open this group")

    def _switch_group(self, *_):
        on = not all(cb.isChecked() for cb in self.boxes)
        for cb in self.boxes:
            cb.setChecked(on)

    def sync(self, *_):
        """Show what the group is actually set to, without switching anything."""
        n = sum(cb.isChecked() for cb in self.boxes)
        state = QtCore.Qt.Checked if n == len(self.boxes) else \
            (QtCore.Qt.Unchecked if n == 0 else QtCore.Qt.PartiallyChecked)
        self.switch.blockSignals(True)
        self.switch.setCheckState(state)
        self.switch.blockSignals(False)


class LayersPage(QtWidgets.QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        lay.addWidget(QtWidgets.QLabel("DISPLAY LAYERS", objectName="PanelTitle"))
        tips = {key: tip for key, _text, tip in LAYERS}
        labels = {key: text for key, text, _tip in LAYERS}
        self.boxes = {}
        for key in labels:
            cb = QtWidgets.QCheckBox(labels[key])
            cb.setChecked(bool(controller.layers.get(key, True)))
            cb.toggled.connect(lambda on, k=key: controller.set_layer(k, on))
            cb.setToolTip(tips[key])
            self.boxes[key] = cb
        self.sections = {}
        for title, keys, opened in GROUPS:
            sec = Section(title, [self.boxes[k] for k in keys], opened)
            lay.addWidget(sec)
            self.sections[title] = sec
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
    collapse_requested = QtCore.Signal()        # the ◀ button in the panel header

    def __init__(self, controller, parent=None):
        super().__init__(parent, objectName="NavPanel")
        self.ctrl = controller
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(4)
        # title row: the heading, and the control that collapses this panel to a thin rail
        head = QtWidgets.QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        head.setSpacing(4)
        head.addWidget(QtWidgets.QLabel("NAVIGATION", objectName="PanelTitle"), 1)
        self.collapse_btn = QtWidgets.QToolButton(text="◀", objectName="SidebarToggle")
        self.collapse_btn.setToolTip("Hide the sidebar  (Ctrl+B)")
        self.collapse_btn.clicked.connect(self.collapse_requested)
        head.addWidget(self.collapse_btn)
        lay.addLayout(head)

        # The world view shows 3D or 2D; the Aircraft PPI can be opened in its own window.
        # One row, not three stacked buttons: everything above the pages is fixed height, and
        # what it takes comes straight out of the layer switches below.
        self.view_group = QtWidgets.QButtonGroup(self)
        self.view_btns = {}
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(3)
        for mode, text, tip in (("3d", "3D", "The 3D world view (key 3)"),
                                ("2d", "2D", "The 2D world view (key 2)"),
                                ("ppi", "PPI  ⤢", "Open the Aircraft PPI in its own maximised "
                                                  "window (close it or Esc to return)")):
            b = QtWidgets.QPushButton(text, checkable=True, objectName="NavTab", toolTip=tip)
            if mode != "ppi":
                b.setChecked(mode == controller.view_mode)
                self.view_group.addButton(b)
            b.clicked.connect(lambda _=False, m=mode: self.view_selected.emit(m))
            row.addWidget(b, 1)
            self.view_btns[mode] = b
        lay.addLayout(row)

        line = QtWidgets.QFrame(frameShape=QtWidgets.QFrame.HLine, objectName="Sep")
        lay.addWidget(line)
        lay.addWidget(self._build_camera(controller))
        line = QtWidgets.QFrame(frameShape=QtWidgets.QFrame.HLine, objectName="Sep")
        lay.addWidget(line)

        self.page_group = QtWidgets.QButtonGroup(self)
        self.stack = QtWidgets.QStackedWidget()
        self.layers = LayersPage(controller)
        self.sensors = SensorPanel(controller)
        self.settings = SettingsPage(controller)
        tabs = QtWidgets.QHBoxLayout()
        tabs.setSpacing(3)
        for i, (text, page) in enumerate((("Tracks", self.layers), ("Sensors", self.sensors),
                                          ("Settings", self.settings))):
            b = QtWidgets.QPushButton(text, checkable=True, objectName="NavTab")
            b.setChecked(i == 0)
            self.page_group.addButton(b, i)
            b.clicked.connect(lambda _=False, j=i: self.stack.setCurrentIndex(j))
            tabs.addWidget(b, 1)
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
            scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            scroll.setWidget(page)
            self.stack.addWidget(scroll)
        lay.addLayout(tabs)
        lay.addWidget(self.stack, 1)

    def _build_camera(self, controller):
        """The 3D camera, next to the view buttons: which preset holds it, and whether it
        follows. The same controls as the 3D toolbar, kept in step with it by bind_camera."""
        box = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(4)
        row.addWidget(QtWidgets.QLabel("CAMERA", objectName="SectionLabel"))
        row.addStretch(1)
        self.cam_follow = QtWidgets.QCheckBox("Follow")
        self.cam_follow.setToolTip("Keep the camera on the ownship, framing it automatically")
        self.cam_debug = QtWidgets.QCheckBox("DEBUG")
        self.cam_debug.setToolTip("Engineering overlay in the 3D view: attitude, velocity, "
                                  "camera, scan state (D)")
        row.addWidget(self.cam_follow)
        row.addWidget(self.cam_debug)
        lay.addLayout(row)
        self.cam_preset = QtWidgets.QComboBox()
        for label, key, shortcut in CAMERA_PRESETS:
            self.cam_preset.addItem(f"{label}  ({shortcut})", key)
        self.cam_preset.setToolTip("Camera preset. It eases into place and then holds the "
                                  "ownship; dragging in the view hands the camera back to you.")
        lay.addWidget(self.cam_preset)
        self.camera_box = box
        box.setEnabled(controller.view_mode == "3d")
        controller.view_mode_changed.connect(lambda m: box.setEnabled(m == "3d"))
        return box

    def bind_camera(self, view3d):
        """Two sets of controls, one camera: whichever the operator uses, both show it.

        Each control here calls the view directly rather than synthesising a click on the
        toolbar button, and the view's own ``toggled`` copies the resulting state back with
        signals blocked - so there is one code path per control and no loop between them."""
        def mirror(target, source):
            target.blockSignals(True)
            target.setChecked(source.isChecked())
            target.blockSignals(False)

        self.cam_preset.activated.connect(
            lambda i: view3d.apply_camera(self.cam_preset.itemData(i)))
        view3d.camera_changed.connect(self._show_preset)
        self.cam_preset.setCurrentIndex(view3d.preset_box.currentIndex())

        self.cam_follow.setChecked(view3d.follow_btn.isChecked())
        self.cam_follow.clicked.connect(lambda: view3d.set_follow(self.cam_follow.isChecked()))
        view3d.follow_btn.toggled.connect(
            lambda *_: mirror(self.cam_follow, view3d.follow_btn))

        self.cam_debug.setChecked(view3d.debug_btn.isChecked())
        self.cam_debug.clicked.connect(
            lambda: view3d.debug_btn.setChecked(self.cam_debug.isChecked()))
        view3d.debug_btn.toggled.connect(lambda *_: mirror(self.cam_debug, view3d.debug_btn))

    def _show_preset(self, key: str):
        """Show the preset the view has just taken up. Nothing is applied from here."""
        i = self.cam_preset.findData(key)
        if i >= 0 and i != self.cam_preset.currentIndex():
            self.cam_preset.setCurrentIndex(i)

    def show_page(self, i: int):
        self.page_group.button(i).setChecked(True)
        self.stack.setCurrentIndex(i)
