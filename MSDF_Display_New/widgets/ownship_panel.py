"""Ownship state: position, velocity and attitude, exactly as the Scenario Export records them.

Nine numbers from the frame's own ownship row. Nothing here is integrated, smoothed, derived
or estimated - if a value is not in the export it reads "not in export" rather than being
filled in from somewhere else.

Conventions are the project's, unchanged (docs/COORDINATE_AND_ATTITUDE.md):

    world   ENU   X = East, Y = North, Z = Up
    yaw     compass, 0 = North, 90 = East
    pitch   + = nose up
    roll    + = right wing down
"""

from __future__ import annotations

import math

from PySide6 import QtCore, QtWidgets

#: (section, label, attribute on the ownship row, unit, decimals)
FIELDS = [
    ("POSITION", "X (East)", "x", "m", 1),
    ("POSITION", "Y (North)", "y", "m", 1),
    ("POSITION", "Z (Up)", "z", "m", 1),
    ("VELOCITY", "Vx", "vx", "m/s", 2),
    ("VELOCITY", "Vy", "vy", "m/s", 2),
    ("VELOCITY", "Vz", "vz", "m/s", 2),
    ("ATTITUDE", "Yaw", "yaw_deg", "°", 2),
    ("ATTITUDE", "Pitch", "pitch_deg", "°", 2),
    ("ATTITUDE", "Roll", "roll_deg", "°", 2),
]

#: The export's Vx / Vy columns are transposed with respect to its X / Y position columns
#: (msdf_math.motion). The recorded values are shown as recorded - correcting them here would
#: hide an upstream inconsistency the rest of the application deliberately surfaces - so the
#: note lives in the tooltip instead of in the numbers.
VELOCITY_NOTE = ("Exported velocity columns, as recorded.\n"
                 "Note: upstream, Vx carries the NORTH component and Vy the EAST one "
                 "(they are transposed with respect to X / Y).\n"
                 "The views draw the aircraft along the course its recorded POSITIONS imply.")


class OwnshipPanel(QtWidgets.QFrame):
    """Compact read-out of the current ownship row."""

    def __init__(self, controller, parent=None):
        super().__init__(parent, objectName="Panel")
        self.ctrl = controller
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)
        lay.addWidget(QtWidgets.QLabel("OWNSHIP STATE", objectName="PanelTitle"))
        self.header = QtWidgets.QLabel("—", objectName="ViewInfo")
        self.header.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        lay.addWidget(self.header)

        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(0)
        grid.setColumnStretch(1, 1)
        self.values: dict[str, QtWidgets.QLabel] = {}
        row = 0
        section = None
        for sec, label, _attr, unit, _dp in FIELDS:
            if sec != section:
                if section is not None:
                    grid.setRowMinimumHeight(row, 8)
                    row += 1
                head = QtWidgets.QLabel(sec, objectName="SectionLabel")
                if sec == "VELOCITY":
                    head.setToolTip(VELOCITY_NOTE)
                grid.addWidget(head, row, 0, 1, 3)
                row += 1
                section = sec
            name = QtWidgets.QLabel(label, objectName="FieldName")
            value = QtWidgets.QLabel("—", objectName="FieldValue")
            value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            if sec == "VELOCITY":
                name.setToolTip(VELOCITY_NOTE)
                value.setToolTip(VELOCITY_NOTE)
            grid.addWidget(name, row, 0)
            grid.addWidget(value, row, 1)
            grid.addWidget(QtWidgets.QLabel(unit, objectName="FieldUnit"), row, 2)
            self.values[label] = value
            row += 1
        lay.addLayout(grid)
        lay.addStretch(1)

        self._clock = QtCore.QElapsedTimer()
        self._clock.start()
        controller.frame_ready.connect(self.on_frame)
        controller.data_error.connect(lambda *_: self._blank("Scenario Export unavailable"))
        controller.loading.connect(lambda *_: self._blank("loading ..."))

    def _blank(self, text: str):
        self.header.setText(text)
        for value in self.values.values():
            value.setText("—")

    def on_frame(self, frame):
        if self.ctrl.is_playing and self._clock.elapsed() < 100:       # 10 Hz text refresh
            return
        self._clock.restart()
        own = frame.ownship
        if own is None:
            self._blank("no ownship row at this time")
            return
        self.header.setText(f"export row t = {own.timestamp:.3f} s")
        for _sec, label, attr, _unit, dp in FIELDS:
            raw = getattr(own, attr, float("nan"))
            self.values[label].setText("not in export" if not math.isfinite(raw)
                                       else f"{raw:,.{dp}f}")
