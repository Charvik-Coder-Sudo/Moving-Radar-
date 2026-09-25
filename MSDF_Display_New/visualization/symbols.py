"""Qt symbols for the 2D view, the Aircraft PPI, the legends and the track table.

Shapes come from style.SOURCES (what the record is); colours come from
style.classify (classification). A symbol is built here once and reused, so
the legend, the table icons and the plots always show the same thing.

    sensor track (primary / secondary)   square, hollow (secondary dashed)
    fused system track                   filled triangle
    truth target                         aircraft silhouette, rotated to its heading
    ownship (PPI)                        larger outlined triangle with a notched tail

Paths use pyqtgraph's symbol convention: unit box +-0.5, y pointing DOWN on
screen, nose / forward = -y.
"""

from __future__ import annotations

import pyqtgraph as pg
from PySide6 import QtCore, QtGui

from visualization import style


def _poly(points) -> QtGui.QPainterPath:
    p = QtGui.QPainterPath()
    p.moveTo(*points[0])
    for q in points[1:]:
        p.lineTo(*q)
    p.closeSubpath()
    return p


def aircraft_path() -> QtGui.QPainterPath:
    """Top-down silhouette of a generic military jet (swept wings, tailplanes), nose up."""
    right = [(0.0, -0.50), (0.045, -0.40), (0.07, -0.22), (0.075, -0.08), (0.46, 0.13), (0.46, 0.21),
             (0.085, 0.17), (0.085, 0.27), (0.25, 0.41), (0.25, 0.47), (0.07, 0.43), (0.05, 0.50)]
    left = [(-x, y) for x, y in reversed(right[1:])]
    return _poly(right + left + [(0.0, 0.50)])


def triangle_path(half=0.5) -> QtGui.QPainterPath:
    """Fused system track: a filled triangle, apex up."""
    return _poly([(0.0, -half), (half * 0.92, half * 0.72), (-half * 0.92, half * 0.72)])


def ownship_path() -> QtGui.QPainterPath:
    """Own aircraft in the PPI: a larger outlined triangle with a notched tail, so it cannot be
    read as a fused track (a smaller, plain, filled triangle)."""
    return _poly([(0.0, -0.5), (0.42, 0.42), (0.0, 0.18), (-0.42, 0.42)])


AIRCRAFT = aircraft_path()
TRIANGLE = triangle_path()
OWNSHIP = ownship_path()
_PG_SYMBOL = {"square": "s", "diamond": "d"}
_ROTATED: dict[tuple[int, int], QtGui.QPainterPath] = {}


def rotated(path: QtGui.QPainterPath, angle_deg: float) -> QtGui.QPainterPath:
    """Rotate clockwise on screen by angle_deg (cached per whole degree)."""
    a = int(round(angle_deg)) % 360
    key = (id(path), a)
    if key not in _ROTATED:
        _ROTATED[key] = QtGui.QTransform().rotate(a).map(path)
    return _ROTATED[key]


def symbol(src: style.SourceStyle, heading_deg: float = 0.0):
    """pyqtgraph symbol for a track type: square = sensor track, triangle = fused system track,
    aircraft silhouette = truth target (rotated to its heading)."""
    if src.symbol == "aircraft":
        return rotated(AIRCRAFT, heading_deg)
    if src.symbol == "triangle":
        return TRIANGLE
    return _PG_SYMBOL[src.symbol]


class PenCache:
    """Pens and brushes are expensive to create per spot per frame; build each once."""

    def __init__(self):
        self._pens: dict = {}
        self._brushes: dict = {}
        self.no_brush = pg.mkBrush(None)
        self.no_pen = pg.mkPen(None)

    def pen(self, colour: str, alpha: int = 255, width: float = 1.0, dashed: bool = False):
        k = (colour, alpha, width, dashed)
        if k not in self._pens:
            c = QtGui.QColor(colour)
            c.setAlpha(alpha)
            self._pens[k] = pg.mkPen(c, width=width, style=QtCore.Qt.DashLine if dashed else QtCore.Qt.SolidLine)
        return self._pens[k]

    def brush(self, colour: str, alpha: int = 255):
        k = (colour, alpha)
        if k not in self._brushes:
            c = QtGui.QColor(colour)
            c.setAlpha(alpha)
            self._brushes[k] = pg.mkBrush(c)
        return self._brushes[k]

    def marker(self, src: style.SourceStyle, class_colour: str, alpha: int = 255):
        """(pen, brush) for a marker: body and classification kept separate.

        target     muted military fill, classification-coloured outline
        sensor     hollow square; primary solid outline, secondary dashed outline
        fused      filled classification-coloured triangle with a dark edge for contrast
        """
        if src.symbol == "aircraft":
            return (self.pen(class_colour, alpha, src.pen_width),
                    self.brush(style.AIRCRAFT_ICON_FILL, min(alpha, 235)))
        if src.symbol == "triangle":
            return self.pen("#020617", alpha, 1.0), self.brush(class_colour, alpha)
        return self.pen(class_colour, alpha, src.pen_width, src.dashed), self.no_brush


_ICON_PENS = PenCache()


def icon_pixmap(src: style.SourceStyle, class_colour: str, size: int = 18) -> QtGui.QPixmap:
    """Small pixmap of a marker, for legends and the track table."""
    pm = QtGui.QPixmap(size, size)
    pm.fill(QtCore.Qt.transparent)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.Antialiasing)
    p.translate(size / 2, size / 2)
    scale = size * 0.82
    p.scale(scale, scale)
    pen, brush = _ICON_PENS.marker(src, class_colour)
    qpen = QtGui.QPen(pen)
    qpen.setCosmetic(True)                  # width in pixels, independent of the icon scale
    qpen.setWidthF(max(pen.widthF(), 1.0) * 1.25)
    p.setPen(qpen)
    p.setBrush(brush)
    if src.symbol == "square":
        p.drawRect(QtCore.QRectF(-0.38, -0.38, 0.76, 0.76))
    elif src.symbol == "triangle":
        p.drawPath(TRIANGLE)
    else:
        p.drawPath(symbol(src))
    p.end()
    return pm


def swatch_pixmap(colour: str, size: int = 14) -> QtGui.QPixmap:
    """Round classification swatch."""
    pm = QtGui.QPixmap(size, size)
    pm.fill(QtCore.Qt.transparent)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.Antialiasing)
    p.setPen(QtGui.QPen(QtGui.QColor("#0f172a"), 1))
    p.setBrush(QtGui.QColor(colour))
    p.drawEllipse(QtCore.QRectF(1, 1, size - 2, size - 2))
    p.end()
    return pm
