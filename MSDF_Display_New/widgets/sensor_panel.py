"""Sensor panel: configuration and live scan state of each physical sensor."""

from __future__ import annotations

from PySide6 import QtGui, QtWidgets

from visualization import style


class SensorCard(QtWidgets.QFrame):
    def __init__(self, controller, cfg, palette, parent=None):
        super().__init__(parent, objectName="Card")
        self.ctrl = controller
        self.cfg = cfg
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)
        top = QtWidgets.QHBoxLayout()
        swatch = QtWidgets.QLabel()
        pm = QtGui.QPixmap(12, 12)
        pm.fill(QtGui.QColor(palette.sensor_color(cfg.sensor_id)))
        swatch.setPixmap(pm)
        top.addWidget(swatch)
        self.enable = QtWidgets.QCheckBox(cfg.name)
        self.enable.setChecked(True)
        self.enable.setToolTip("Show this sensor's coverage, beam and PPI geometry")
        self.enable.toggled.connect(lambda on: controller.set_sensor_visible(cfg.sensor_id, on))
        top.addWidget(self.enable, 1)
        bpm = QtGui.QPixmap(12, 12)
        bpm.fill(QtGui.QColor(palette.beam_color(cfg.sensor_id)))
        beam_sw = QtWidgets.QLabel()
        beam_sw.setPixmap(bpm)
        beam_sw.setToolTip("scan-beam colour (the sensor colour, drawn stronger)")
        top.addWidget(beam_sw)
        lay.addLayout(top)
        number = next((d.get("sensor_number") for d in controller.config.get("sensors", [])
                       if d["id"] == cfg.sensor_id), "?")
        static = (f"source id  {cfg.sensor_id} ({cfg.sensor_type}), RDP sensorId {number}\n"
                  f"range      {cfg.r_max / 1000:g} km\n"
                  f"coverage   {cfg.az_coverage_deg:g}° az × {cfg.el_coverage_deg:g}° el\n"
                  f"beam       {cfg.az_beamwidth_deg:g}° × {cfg.el_beamwidth_deg:g}°\n"
                  f"dwells     {cfg.n_az} az × {cfg.n_el} el = {cfg.n_dwells}\n"
                  f"dwell      {cfg.dwell_time_s * 1e3:.3f} ms · scan {cfg.scan_time_s:g} s\n"
                  f"mount FRD  {tuple(cfg.mount_xyz_frd)} m, ypr {tuple(cfg.mount_ypr_deg)}°")
        s = QtWidgets.QLabel(static, objectName="Mono")
        lay.addWidget(s)
        self.live = QtWidgets.QLabel("—", objectName="MonoLive")
        lay.addWidget(self.live)

    def update_frame(self, frame, rdp_tracks: int):
        """Beam and radar pose from the Scenario Export; track count from the RDP stream."""
        st = next((s for s in frame.sensors if s.config.sensor_id == self.cfg.sensor_id), None) \
            if frame is not None else None
        if st is None:
            self.live.setText(f"no exported radar pose at this time\nRDP sensor tracks {rdp_tracks}")
            return
        b = st.beam
        beam = (f"scan {b.scan_id} · dwell {b.dwell_id}\n"
                f"beam az {b.az_deg:+6.1f}° el {b.el_deg:+6.1f}°") if b else "beam: outside the exported schedule"
        self.live.setText(f"{beam}  (export)\n"
                          f"radar yaw {st.yaw_deg:6.1f}° pitch {st.pitch_deg:+5.1f}° roll {st.roll_deg:+5.1f}°\n"
                          f"RDP sensor tracks (visible) {rdp_tracks}")


class SensorPanel(QtWidgets.QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        palette = style.Palette(controller.config)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(QtWidgets.QLabel("SENSORS", objectName="PanelTitle"))
        note = QtWidgets.QLabel("From Scenario_Export/Sensor_Properties/sensor_properties.json. One "
                                "coverage volume per sensor; the scan beam is the exported dwell.",
                                objectName="Hint")
        note.setWordWrap(True)
        lay.addWidget(note)
        self.ctrl = controller
        self.cards = [SensorCard(controller, c, palette) for c in controller.sensors]
        for c in self.cards:
            lay.addWidget(c)
        if not self.cards:
            lay.addWidget(QtWidgets.QLabel("No sensors: the exported sensor properties could not be read.",
                                           objectName="Hint"))
        lay.addStretch(1)
        self._frame = None
        self._counts: dict = {}
        controller.frame_ready.connect(self.on_frame)
        controller.rdp_updated.connect(self.on_rdp)

    def on_frame(self, frame):
        self._frame = frame
        self._refresh()

    def on_rdp(self, views, stats):
        counts: dict = {}
        for tv in views:
            if tv.kind == "sensor":
                counts[tv.state.source] = counts.get(tv.state.source, 0) + 1
        self._counts = counts
        self._refresh()

    def _refresh(self):
        if not self.isVisible():
            return
        for c in self.cards:
            c.update_frame(self._frame, self._counts.get(c.cfg.sensor_id, 0))
