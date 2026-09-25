"""Ownship readout from the Scenario Export: pose [X Y Z Yaw Pitch Roll] and motion.

Values are the exported ones. The export has no acceleration or turn rate, so
those are shown as "not in export" rather than estimated.
"""

from __future__ import annotations

import math

from PySide6 import QtCore, QtWidgets


class OwnshipPanel(QtWidgets.QFrame):
    FIELDS = [
        ("pose", "X (East)", "m"), ("pose", "Y (North)", "m"), ("pose", "Z (Up)", "m"),
        ("pose", "Yaw", "°"), ("pose", "Pitch", "°"), ("pose", "Roll", "°"),
        ("motion", "Vx", "m/s"), ("motion", "Vy", "m/s"), ("motion", "Vz", "m/s"),
        ("motion", "Ax", "m/s²"), ("motion", "Ay", "m/s²"), ("motion", "Az", "m/s²"),
        ("motion", "Omega", "rad/s"),
        ("derived", "Speed", "m/s"), ("derived", "Course (velocity)", "°"),
        ("derived", "Flight path", "°"),
        # recorded vs velocity-derived: validation only, the export is never corrected
        ("derived", "Yaw - course", "°"), ("derived", "Pitch - flight path", "° = AoA"),
        ("derived", "Estimated bank", "° not measured"),
        # what the recorded POSITIONS do - this is what the views draw (msdf_math.motion)
        ("path", "Course over ground", "° drawn"), ("path", "Climb angle", "°"),
        ("path", "Speed along path", "m/s"), ("path", "Yaw - course over ground", "°"),
    ]

    def __init__(self, controller, parent=None):
        super().__init__(parent, objectName="Panel")
        self.ctrl = controller
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)
        lay.addWidget(QtWidgets.QLabel("OWNSHIP 6-DoF  ·  Scenario Export", objectName="PanelTitle"))
        self.header = QtWidgets.QLabel("—", objectName="ViewInfo")
        self.header.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        lay.addWidget(self.header)
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(0)
        self.values = {}
        names = {"pose": "POSE (exported)", "motion": "MOTION (exported)", "derived": "DERIVED",
                 "path": "FROM THE FLOWN PATH (drawn)"}
        block_col = {"pose": 0, "derived": 0, "motion": 4, "path": 4}
        rows = {0: 0, 4: 0}
        section = None
        for sec, name, unit in self.FIELDS:
            c = block_col[sec]
            if sec != section:
                grid.addWidget(QtWidgets.QLabel(names[sec], objectName="SectionLabel"), rows[c], c, 1, 3)
                rows[c] += 1
                section = sec
            n = QtWidgets.QLabel(name, objectName="FieldName")
            v = QtWidgets.QLabel("—", objectName="FieldValue")
            u = QtWidgets.QLabel(unit, objectName="FieldUnit")
            grid.addWidget(n, rows[c], c)
            grid.addWidget(v, rows[c], c + 1)
            grid.addWidget(u, rows[c], c + 2)
            self.values[name] = v
            rows[c] += 1
        grid.setColumnMinimumWidth(3, 10)
        lay.addLayout(grid)
        lay.addStretch(1)
        self._clock = QtCore.QElapsedTimer()
        self._clock.start()
        controller.frame_ready.connect(self.on_frame)
        controller.data_error.connect(lambda msg: self._blank("Scenario Export unavailable"))
        controller.loading.connect(lambda msg: self._blank("loading ..."))

    def _blank(self, text):
        self.header.setText(text)
        for v in self.values.values():
            v.setText("—")

    def on_frame(self, frame):
        if self.ctrl.is_playing and self._clock.elapsed() < 100:     # 10 Hz text refresh
            return
        self._clock.restart()
        own = frame.ownship
        if own is None:
            self._blank("no ownship row at this time")
            return
        self.header.setText(f"export row t = {own.timestamp_ms:.1f} ms  ({own.timestamp:.3f} s)")
        f = lambda x, p=1: "not in export" if not math.isfinite(x) else f"{x:,.{p}f}"   # noqa: E731
        vh = math.hypot(own.vx, own.vy)
        vals = {
            "X (East)": f(own.x), "Y (North)": f(own.y), "Z (Up)": f(own.z),
            "Yaw": f(own.yaw_deg, 2), "Pitch": f(own.pitch_deg, 2), "Roll": f(own.roll_deg, 2),
            "Vx": f(own.vx, 2), "Vy": f(own.vy, 2), "Vz": f(own.vz, 2),
            "Ax": f(own.ax, 3), "Ay": f(own.ay, 3), "Az": f(own.az, 3), "Omega": f(own.omega, 5),
            "Speed": f(own.speed, 1),
            "Course (velocity)": f(math.degrees(math.atan2(own.vx, own.vy)) % 360.0, 2),
            "Flight path": f(math.degrees(math.atan2(own.vz, vh)), 2),
            "Course over ground": f(own.course_deg, 2), "Climb angle": f(own.climb_deg, 2),
            "Speed along path": f(own.path_speed_mps, 1),
            "Yaw - course over ground": f(own.attitude_delta_deg, 2),
        }
        chk = frame.attitude or {}
        vals.update({"Yaw - course": f(chk.get("d_heading_deg", math.nan), 2),
                     "Pitch - flight path": f(chk.get("d_pitch_deg", math.nan), 2),
                     "Estimated bank": f(chk.get("estimated_bank_deg", math.nan), 2)})
        for k, v in vals.items():
            self.values[k].setText(v)
        # the exported attitude and the flown path disagree in this scenario: say so, don't hide it
        if own.attitude_disagrees:
            self.header.setText(
                f"export row t = {own.timestamp_ms:.1f} ms  ·  exported Yaw {own.yaw_deg:.2f}° vs "
                f"course over ground {own.course_deg:.2f}°  (Δ {own.attitude_delta_deg:+.2f}°) — "
                "drawn along the path")
