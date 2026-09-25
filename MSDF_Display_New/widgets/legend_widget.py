"""Legend built from the central style: classification colours and track types, kept apart."""

from __future__ import annotations

from PySide6 import QtCore, QtWidgets

from visualization import style, symbols


class LegendWidget(QtWidgets.QFrame):
    """Three labelled groups, so sensor and system tracks are never one ambiguous category:

        CLASSIFICATION (colour)
        SENSOR TRACKS  square - what one sensor sees (primary, secondary / IFF)
        SYSTEM TRACKS  triangle - the fusion output (and the truth target where shown)

    Shape is the track type and colour is the classification, in every view.
    """

    def __init__(self, columns: int = 2, include_target: bool = True, parent=None):
        super().__init__(parent, objectName="Legend")
        grid = QtWidgets.QGridLayout(self)
        grid.setContentsMargins(8, 4, 8, 4)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(1)

        def cell(pixmap, text, tip):
            w = QtWidgets.QWidget()
            h = QtWidgets.QHBoxLayout(w)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(5)
            ic = QtWidgets.QLabel()
            ic.setPixmap(pixmap)
            ic.setFixedSize(pixmap.size())
            h.addWidget(ic)
            lab = QtWidgets.QLabel(text, objectName="LegendText")
            h.addWidget(lab, 1)
            w.setToolTip(tip)
            return w

        grid.addWidget(QtWidgets.QLabel("CLASSIFICATION  (colour)", objectName="LegendHead"), 0, 0, 1, columns)
        r = 1
        for i, cs in enumerate(style.CLASSIFICATIONS.values()):
            grid.addWidget(cell(symbols.swatch_pixmap(cs.color), cs.label,
                                f"{cs.label}: {cs.color}"), r + i // columns, i % columns)
        r += (len(style.CLASSIFICATIONS) + columns - 1) // columns
        grid.addWidget(QtWidgets.QLabel("SENSOR TRACKS  □ square", objectName="LegendHead"),
                       r, 0, 1, columns)
        r += 1
        sensor_srcs = [style.SOURCES[k] for k in (style.PRIMARY, style.SECONDARY)]
        for i, src in enumerate(sensor_srcs):
            grid.addWidget(cell(symbols.icon_pixmap(src, style.TYPE_ICON_COLOR, 18), src.label,
                                f"{src.label}: a track from this one sensor. Square = sensor track; "
                                "the outline style and size say which sensor. Colour is the "
                                "classification, never the sensor."), r + i // columns, i % columns)
        r += (len(sensor_srcs) + columns - 1) // columns
        grid.addWidget(QtWidgets.QLabel("SYSTEM TRACKS  △ triangle", objectName="LegendHead"),
                       r, 0, 1, columns)
        r += 1
        system_srcs = [style.SOURCES[style.FUSED]] + ([style.SOURCES[style.TARGET]] if include_target else [])
        for i, src in enumerate(system_srcs):
            tip = ("Fused system track: the fusion engine's output. Triangle = fused track, drawn "
                    "larger and brighter than the sensor tracks it comes from"
                    if src.key == style.FUSED else
                    "Scenario Export truth target: the recorded aircraft, not a track")
            grid.addWidget(cell(symbols.icon_pixmap(src, style.TYPE_ICON_COLOR, 18), src.label, tip),
                           r + i // columns, i % columns)
