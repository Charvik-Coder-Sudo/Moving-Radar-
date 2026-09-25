"""Main window (the established layout: one world view, the PPI always visible).

┌───────────────────────────────────────────────────────────────────────────────┐
│ MSDF DISPLAY   [3D View] [2D View]     ● LIVE  UDP 127.0.0.1:9000  ● FOLLOWING│
│ Status · UDP · Packets · Rate · Tracks · Targets · Sim time · Last packet     │
├─────────────┬───────────────────────────────────────────┬─────────────────────┤
│ Navigation  │  WORLD VIEW: 3D or 2D              [⤢]    │ Aircraft PPI  [⤢]   │
│             │  playback bar (replay of the export)      │ Ownship             │
├─────────────┴───────────────────────────────────────────┴─────────────────────┤
│                               TRACK DETAILS                                   │
└───────────────────────────────────────────────────────────────────────────────┘

The centre switches between the 3D and the 2D world view (buttons, keys 3 / 2); both
draw the same data from the ONE controller (one UDP receiver, one track store, one
cached Scenario Export). [⤢] on a world view maximises it inside this window (Esc
returns, F11 fullscreen); [⤢] on the PPI opens it in its own window (the same widget
is moved - widgets.popout). The aircraft ambience plays while the 3D view is on
screen (widgets.aircraft_audio).
"""

from __future__ import annotations

from pathlib import Path

from PySide6 import QtCore, QtGui, QtWidgets

from visualization.aircraft_ppi import AircraftPPI
from visualization.view_2d import View2D
from visualization.view_3d import View3D
from widgets.aircraft_audio import AircraftAmbience
from widgets.navigation_panel import NavigationPanel
from widgets.ownship_panel import OwnshipPanel
from widgets.popout import PopOutManager
from widgets.rdp_status import RdpStatusBar
from widgets.track_table import TrackTable

LIVE_PILL = {"LIVE": "online", "WAITING FOR DATA": "paused", "NO DATA / STALE": "nodata",
             "UDP UNAVAILABLE": "nodata", "RDP OFF": "paused"}

RATES = [0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0]


class PlaybackBar(QtWidgets.QFrame):
    def __init__(self, controller, parent=None):
        super().__init__(parent, objectName="PlaybackBar")
        self.ctrl = controller
        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(8, 4, 8, 4)
        h.setSpacing(4)

        def tb(text, tip, fn):
            b = QtWidgets.QToolButton(text=text, toolTip=tip)
            b.clicked.connect(fn)
            h.addWidget(b)
            return b

        tb("⏮", "Go to start", lambda: controller.seek(controller.t0))
        tb("−1 s", "Step back 1 s", lambda: controller.step(-1.0))
        self.play = tb("▶  Play", "Play / pause (Space)", controller.toggle)
        self.play.setObjectName("PlayButton")
        tb("+1 s", "Step forward 1 s", lambda: controller.step(1.0))
        tb("⏭", "Go to end", lambda: controller.seek(controller.t1))
        self.rate = QtWidgets.QComboBox()
        for r in RATES:
            self.rate.addItem(f"{r:g}×", r)
        self.rate.setCurrentIndex(RATES.index(1.0))
        self.rate.currentIndexChanged.connect(lambda: controller.set_rate(self.rate.currentData()))
        self.rate.setToolTip("Playback rate (slow rates show the elevation sweep of the beam)")
        h.addWidget(self.rate)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, 10000)
        self.slider.sliderMoved.connect(self._slid)
        h.addWidget(self.slider, 1)
        self.time = QtWidgets.QLabel("T —", objectName="TimeLabel")
        h.addWidget(self.time)
        controller.frame_ready.connect(self.on_frame)
        controller.playing_changed.connect(
            lambda on: self.play.setText("⏸  Pause" if on else "▶  Play"))

    def _slid(self, v):
        c = self.ctrl
        c.seek(c.t0 + (c.t1 - c.t0) * v / 10000.0)

    def on_frame(self, frame):
        c = self.ctrl
        span = max(c.t1 - c.t0, 1e-9)
        if not self.slider.isSliderDown():
            self.slider.blockSignals(True)
            self.slider.setValue(int(10000 * (frame.t - c.t0) / span))
            self.slider.blockSignals(False)
        self.time.setText(f"T {frame.t:8.2f} s  /  {c.t1:.2f} s")


class ValidationDialog(QtWidgets.QDialog):
    """Scenario Export checks and RDP packet rejections (text, can be saved)."""

    def __init__(self, sections: list[tuple[str, str]], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Data Validation Report")
        self.resize(980, 640)
        lay = QtWidgets.QVBoxLayout(self)
        tabs = QtWidgets.QTabWidget()
        self.texts = sections
        for title, text in sections:
            txt = QtWidgets.QPlainTextEdit(readOnly=True)
            txt.setFont(QtGui.QFont("Consolas", 9))
            txt.setPlainText(text)
            tabs.addTab(txt, title)
        lay.addWidget(tabs, 1)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Checks never change data. Nothing is substituted for missing data."), 1)
        save = QtWidgets.QPushButton("Save Report…")
        save.clicked.connect(lambda: self._save(tabs.currentIndex()))
        close = QtWidgets.QPushButton("Close")
        close.clicked.connect(self.accept)
        row.addWidget(save)
        row.addWidget(close)
        lay.addLayout(row)

    def _save(self, i):
        if not self.texts:
            return
        title, text = self.texts[i]
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save validation report",
                                                        f"{title.replace(' ', '_')}_report.txt", "Text (*.txt)")
        if path:
            Path(path).write_text(text, encoding="utf-8")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.setWindowTitle("MSDF Display")
        avail = QtGui.QGuiApplication.primaryScreen().availableGeometry()
        self.resize(min(1680, avail.width()), min(1000, avail.height() - 40))

        central = QtWidgets.QWidget(objectName="Root")
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_header())
        root.addWidget(RdpStatusBar(controller))
        self.banner = QtWidgets.QLabel("", objectName="ErrorBanner")
        self.banner.setWordWrap(True)
        self.banner.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.banner.setVisible(False)
        root.addWidget(self.banner)

        # centre: ONE world view (3D or 2D) + playback bar
        self.view3d = View3D(controller)
        self.view2d = View2D(controller)
        self.stack = QtWidgets.QStackedWidget()
        self.stack.addWidget(self.view3d)
        self.stack.addWidget(self.view2d)
        centre = QtWidgets.QWidget()
        cl = QtWidgets.QVBoxLayout(centre)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(0)
        cl.addWidget(self.stack, 1)
        cl.addWidget(PlaybackBar(controller))

        # right: PPI (always visible) + ownship
        self.ppi = AircraftPPI(controller)
        right = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        right.addWidget(self.ppi)
        own_scroll = QtWidgets.QScrollArea()
        own_scroll.setWidgetResizable(True)
        own_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        own_scroll.setWidget(OwnshipPanel(controller))
        right.addWidget(own_scroll)
        right.setStretchFactor(0, 3)
        right.setStretchFactor(1, 1)
        right.setChildrenCollapsible(False)
        self.right_split = right

        self.nav = NavigationPanel(controller)
        self.nav.setMinimumWidth(215)
        self.nav.settings.reload_requested.connect(controller.load_scenario)
        self.nav.settings.report_requested.connect(self.show_report)

        top = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        top.addWidget(self.nav)
        top.addWidget(centre)
        top.addWidget(right)
        top.setStretchFactor(1, 1)
        top.setSizes([270, 1000, 420])
        top.setChildrenCollapsible(False)

        self.table = TrackTable(controller)
        self.table.setMinimumHeight(150)
        main = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        main.addWidget(top)
        main.addWidget(self.table)
        main.setStretchFactor(0, 3)
        main.setStretchFactor(1, 1)
        main.setChildrenCollapsible(False)
        self.main_split, self.top_split = main, top
        self._split_done = False
        root.addWidget(main, 1)
        self.setCentralWidget(central)

        # maximise the world view in place (3D never changes window: its OpenGL context would be
        # recreated, see widgets.popout); the PPI pops out into its own window
        self.popouts = PopOutManager(self)
        self.popouts.register_in_window("world", centre, "World view", self, [self.nav, right, self.table])
        self.popouts.register("ppi", self.ppi, "Aircraft PPI")
        self.popout_btns = {}
        for key, view, text, tip, name in (
                ("3d", self.view3d, "⤢ Maximise", "Maximise the world view in this window (Esc returns, F11 fullscreen)",
                 "world"),
                ("2d", self.view2d, "⤢ Maximise", "Maximise the world view in this window (Esc returns, F11 fullscreen)",
                 "world"),
                ("ppi", self.ppi, "⤢ Pop out", "Open the Aircraft PPI in its own maximised window (Esc returns)",
                 "ppi")):
            b = QtWidgets.QToolButton(text=text if key == "ppi" else "⤢", checkable=True, toolTip=tip)
            b.clicked.connect(lambda _=False, n=name: self.popouts.toggle(n))
            view.findChild(QtWidgets.QFrame, "ViewToolbar").layout().addWidget(b)
            self.popout_btns[key] = b
        self.popouts.changed.connect(self._popout_changed)
        self.nav.view_selected.connect(self._nav_view)

        # aircraft ambience: 3D view only, one instance, independent of the data
        self.ambience = AircraftAmbience(self.view3d, controller.config.get("audio", {}), controller.app_dir, self)
        self.sound_btn = QtWidgets.QToolButton(text="🔊", checkable=True, checked=self.ambience.enabled)
        self.sound_btn.setToolTip("Aircraft background sound while the 3D view is on screen (Settings: volume)")
        self.sound_btn.toggled.connect(self._set_sound)
        self.view3d.findChild(QtWidgets.QFrame, "ViewToolbar").layout().addWidget(self.sound_btn)
        self.nav.settings.sound_toggled.connect(self._set_sound)
        self.nav.settings.volume_changed.connect(self.ambience.set_volume)
        if self.ambience.error:
            self.sound_btn.setEnabled(False)
            self.sound_btn.setToolTip(f"Aircraft background sound unavailable: {self.ambience.error}")
            QtCore.QTimer.singleShot(0, lambda: self.status.showMessage(
                "Aircraft background sound unavailable. 3D visualization will continue without audio."))

        self._build_menu()
        self.status = self.statusBar()
        controller.status_message.connect(lambda m: self.status.showMessage(m))
        controller.view_mode_changed.connect(self._apply_view_mode)
        controller.playing_changed.connect(lambda *_: self._update_state())
        controller.data_loaded.connect(lambda *_: self._update_state())
        controller.data_error.connect(self._on_data_error)
        controller.loading.connect(lambda *_: self._update_state())
        controller.live_changed.connect(lambda *_: self._update_state())
        self._apply_view_mode(controller.view_mode)
        self._update_state()

        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Space), self, controller.toggle)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Right), self, lambda: controller.step(1.0))
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Left), self, lambda: controller.step(-1.0))
        QtGui.QShortcut(QtGui.QKeySequence("3"), self, lambda: controller.set_view_mode("3d"))
        QtGui.QShortcut(QtGui.QKeySequence("2"), self, lambda: controller.set_view_mode("2d"))
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_F11), self, self._toggle_fullscreen)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Escape), self, lambda: self.popouts.restore("world"))

    def _nav_view(self, mode):
        if mode == "ppi":
            self.popouts.toggle("ppi")
        else:
            self.ctrl.set_view_mode(mode)

    def _set_sound(self, on: bool):
        """One switch, two controls (3D toolbar button and Settings checkbox) kept in step."""
        for w in (self.sound_btn, self.nav.settings.sound):
            if w.isChecked() != on:
                w.blockSignals(True)
                w.setChecked(on)
                w.blockSignals(False)
        self.ambience.set_enabled(on)

    def _popout_changed(self, name, out):
        widgets = [self.popout_btns["3d"], self.popout_btns["2d"]] if name == "world" else \
            [self.popout_btns["ppi"], self.nav.view_btns["ppi"]]
        for w in widgets:
            w.blockSignals(True)
            w.setChecked(out)
            w.blockSignals(False)
        self.ambience.update()

    def _toggle_fullscreen(self):
        if self.isFullScreen():
            self.showMaximized()
        else:
            self.showFullScreen()

    def showEvent(self, e):
        super().showEvent(e)
        if not self._split_done:
            QtCore.QTimer.singleShot(300, self._initial_split)

    def _initial_split(self):
        """Proportions of the final (maximized) window: table ~28 %, PPI ~62 % of the right column."""
        self._split_done = True
        h = self.main_split.height()
        self.main_split.setSizes([int(h * 0.72), int(h * 0.28)])
        w = self.top_split.width()
        self.top_split.setSizes([int(w * 0.15), int(w * 0.60), int(w * 0.25)])
        rh = self.right_split.height()
        self.right_split.setSizes([int(rh * 0.64), int(rh * 0.36)])

    # ------------------------------------------------------------------ header
    def _build_header(self):
        bar = QtWidgets.QFrame(objectName="Header")
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(14, 6, 14, 6)
        title = QtWidgets.QLabel("MSDF DISPLAY", objectName="AppTitle")
        h.addWidget(title)
        self.file_label = QtWidgets.QLabel("Moving Radar · Multi-Sensor Data Fusion", objectName="ViewInfo")
        h.addWidget(self.file_label)
        h.addStretch(1)
        self.mode_group = QtWidgets.QButtonGroup(self)
        self.mode_btns = {}
        for mode, text in (("3d", "3D View"), ("2d", "2D View")):
            b = QtWidgets.QPushButton(text, checkable=True, objectName="ModeButton")
            self.mode_group.addButton(b)
            b.clicked.connect(lambda _=False, m=mode: self.ctrl.set_view_mode(m))
            h.addWidget(b)
            self.mode_btns[mode] = b
        h.addSpacing(16)
        self.live_pill = QtWidgets.QLabel("", objectName="StatePill")
        self.live_pill.setToolTip("Live state from the actual packet timing on the UDP receiver")
        h.addWidget(self.live_pill)
        rdp = self.ctrl.rdp
        self.udp_label = QtWidgets.QLabel(f"UDP {rdp.host}:{rdp.port}", objectName="ViewInfo")
        h.addWidget(self.udp_label)
        self.live_btn = QtWidgets.QPushButton("Go Live", objectName="ModeButton")
        self.live_btn.setToolTip("Follow the packet time again (after a replay / seek)")
        self.live_btn.clicked.connect(self.ctrl.go_live)
        h.addWidget(self.live_btn)
        h.addSpacing(8)
        self.state_pill = QtWidgets.QLabel("", objectName="StatePill")
        self.state_pill.setToolTip("World views: following the packets (LIVE), replaying the export, or loading")
        h.addWidget(self.state_pill)
        return bar

    def _build_menu(self):
        m = self.menuBar().addMenu("&File")
        m.addAction("&Reload Scenario Export", self.ctrl.load_scenario, QtGui.QKeySequence.Refresh)
        m.addAction("&Validation Report…", self.show_report)
        m.addSeparator()
        m.addAction("E&xit", self.close)
        v = self.menuBar().addMenu("&View")
        v.addAction("&3D View", lambda: self.ctrl.set_view_mode("3d"))
        v.addAction("&2D View", lambda: self.ctrl.set_view_mode("2d"))
        v.addAction("&Maximise World View", lambda: self.popouts.toggle("world"))
        v.addAction("Pop out Aircraft &PPI", lambda: self.popouts.toggle("ppi"))
        v.addSeparator()
        v.addAction("&Go Live", self.ctrl.go_live)
        v.addAction("&Fullscreen", self._toggle_fullscreen, QtGui.QKeySequence(QtCore.Qt.Key_F11))
        h = self.menuBar().addMenu("&Help")
        h.addAction("&Conventions", self._conventions)

    # ------------------------------------------------------------------ behaviour
    def _apply_view_mode(self, mode):
        self.stack.setCurrentWidget(self.view3d if mode == "3d" else self.view2d)
        self.mode_btns[mode].setChecked(True)
        self.nav.view_btns[mode].setChecked(True)
        self.ambience.update()                      # 3D off screen -> ambience stops now

    @staticmethod
    def _pill(label, text, state):
        label.setText(f"●  {text}")
        label.setProperty("state", state)
        label.style().unpolish(label)
        label.style().polish(label)

    def _update_state(self):
        c = self.ctrl
        self._pill(self.live_pill, c.live_state + (f"  ·  {c.live_detail}" if c.live_detail else ""),
                   LIVE_PILL.get(c.live_state, "paused"))
        if c.load_error:
            text, state = "EXPORT ERROR", "nodata"
        elif not c.is_ready:
            text, state = "LOADING EXPORT", "paused"
        elif c.live_following:
            text, state = "FOLLOWING PACKETS", "online"
        elif c.is_playing:
            text, state = "REPLAY", "paused"
        else:
            text, state = "PAUSED" if c.replay_mode else "SCENARIO LOADED", "paused"
        self._pill(self.state_pill, text, state)
        self.live_btn.setVisible(c.replay_mode and c.live_state == "LIVE")
        if c.is_ready:
            self.banner.setVisible(False)

    def _on_data_error(self, message):
        self.banner.setText("SCENARIO EXPORT UNAVAILABLE - the world views show nothing until it loads "
                            "(no substitute data is used). Use File > Reload when the export is complete.\n"
                            + message)
        self.banner.setVisible(True)
        self._update_state()

    def show_report(self):
        c = self.ctrl
        lines = [f"Scenario Export root: {c.export.root}", ""]
        issues = list(c.export_issues) + (list(c.scenario.issues) if c.scenario is not None else [])
        if c.load_error:
            lines += ["LOAD FAILED:", c.load_error, ""]
        lines += [str(i) for i in issues] or ["No issues."]
        st = c.rdp.stats
        rdp = [f"RDP receiver: UDP {c.rdp.host}:{c.rdp.port}  ({c.rdp.connection} {c.rdp.connection_detail})",
               f"packets {st.get('packets', 0):,}   invalid {st.get('invalid', 0):,}   "
               f"stale packets {st.get('stale_packets', 0):,}   restarts {st.get('restarts', 0)}",
               f"invalid by reason: {st.get('invalid_by_reason', {})}",
               f"unknown sensor ids: {st.get('unknown_sensor_ids', {})}", "",
               "Most recent rejected packets:"]
        rdp += [f"  {r.reason:14s} {r.sender:22s} {r.detail}" for r in list(c.rdp.store.rejected)[-200:]] \
            or ["  none"]
        ValidationDialog([("Scenario Export", "\n".join(lines)), ("RDP packets", "\n".join(rdp))], self).exec()

    def closeEvent(self, e):
        self.ambience.shutdown()
        self.popouts.restore_all()
        self.ctrl.shutdown()
        super().closeEvent(e)

    def _conventions(self):
        QtWidgets.QMessageBox.information(self, "Conventions", (
            "SCENARIO EXPORT (3D / 2D world views, ownship panel)\n"
            "  Time: export 'Time' in ms (canonical timestamp_ms), shown in s\n"
            "  World frame: ENU (X East, Y North, Z Up), metres\n"
            "  Ownship Yaw / Pitch / Roll and Radar_Yaw / Pitch / Roll as exported (deg)\n\n"
            "  Truth targets: 3D Trajectory/target_<TgtId>.csv (X/Y/Z, Vx/Vy/Vz, Auth, IFF)\n\n"
            "RDP SYSTEMTRACK PACKETS (UDP, all views and Track Details)\n"
            "  436 bytes, big-endian: id, time, latestSensor, Xfused[10], numSensors,\n"
            "  sensorIds[4], sensorStates[4][10]\n"
            "  State order: x, vx, ax, y, vy, ay, w, z, vz, az\n"
            "  Frame: sensor-relative, x Right, y Forward, z Up = radar FRD (y, x, -z)\n"
            "  Placement: aircraft P_B = mount + R_BR L; world P_W = P_WR(t) + R_WR(t) L\n"
            "  Time: seconds; sensorId 1 = Primary, 2 = Secondary\n\n"
            "LIVE: the world clock follows the newest packet time; play / seek = replay."))
