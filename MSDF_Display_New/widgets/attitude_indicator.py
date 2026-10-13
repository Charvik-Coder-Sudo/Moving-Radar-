"""An attitude indicator for the ownship: the exported Yaw / Pitch / Roll, drawn.

The numbers are already in the Ownship panel beside this. They are the record, and they stay;
this reads them as an attitude instead, because a column of degrees does not tell you at a
glance that the aircraft is 10 deg nose-up and wings level, and a horizon does.

It shows what the Scenario Export recorded and nothing else. Nothing here is integrated,
smoothed or invented - every frame is painted from that frame's own row.

Two things it is careful to keep apart
--------------------------------------
RECORDED roll is what the export states (0.00 deg throughout this scenario: the trajectory
carries no bank). ESTIMATED bank is what ``msdf_math.motion`` derives from the turn the
recorded path actually flies, and is NOT a measurement. They are drawn differently and
labelled differently, so the instrument can never imply the aircraft was recorded banking
when it was not.

Conventions, the same ones the rest of the display uses (docs/COORDINATE_AND_ATTITUDE.md):

    yaw     compass, 0 = North, 90 = East
    pitch   + = nose up
    roll    + = right wing down
"""

from __future__ import annotations

import math

from PySide6 import QtCore, QtGui, QtWidgets

# Instrument palette: sky and ground, deliberately desaturated so the instrument never
# competes with the tracks, and never a classification colour (green / red / yellow / white
# mean Friend / Hostile / Neutral / Unknown everywhere else in this application).
SKY = QtGui.QColor("#2f4a69")
GROUND = QtGui.QColor("#4a3c2c")
HORIZON = QtGui.QColor("#dbe4ee")
LADDER = QtGui.QColor("#aebccd")
BEZEL = QtGui.QColor("#121a24")
FRAME = QtGui.QColor("#2b3b4d")
SYMBOL = QtGui.QColor("#e9f0f8")          # the fixed aircraft: neutral, not a classification
RECORDED = QtGui.QColor("#cfdbe9")
ESTIMATED = QtGui.QColor("#6f9bc4")       # the derived bank, visibly a different thing
TEXT = QtGui.QColor("#9fb0c4")
WARN = QtGui.QColor("#c08a5a")

#: Pitch ladder scale at the reference ball size; scaled with the dial so the instrument
#: stays readable in a narrow sidebar as well as popped out.
PX_PER_DEG = 2.6
REFERENCE_BALL_PX = 136.0
LADDER_STEPS = (-30, -20, -10, 10, 20, 30)
ROLL_TICKS = (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60)


def _finite(value, fallback=0.0) -> float:
    """A float that can be drawn. Missing data must not move the instrument to zero silently,
    so callers check ``valid`` separately - this only keeps the painter safe."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return fallback
    return v if math.isfinite(v) else fallback


class AttitudeIndicator(QtWidgets.QWidget):
    """Artificial horizon, roll scale and heading tape, painted from one ownship row."""

    def __init__(self, parent=None):
        super().__init__(parent)
        # Fixed, not Expanding: this sits in a scroll area whose number grid is wider than
        # the visible panel, and an expanding instrument would stretch to that width and hang
        # off the side. Fixed means the layout uses sizeHint and leaves it alone - it can
        # still be resized explicitly, which is how it is rendered at other sizes.
        self.setMinimumSize(168, 140)
        self.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.setToolTip(
            "Ownship attitude, exactly as recorded in the Scenario Export.\n"
            "Pitch and roll are the exported values; the hollow marker is the ESTIMATED\n"
            "bank derived from the flown path, which is not a measurement.")
        self._own = None
        self._course = float("nan")
        self._bank = float("nan")

    def sizeHint(self):
        return QtCore.QSize(208, 150)

    # ------------------------------------------------------------------ data
    def set_ownship(self, own, estimated_bank_deg=float("nan")):
        """Take one frame's ownship row. ``None`` blanks the instrument."""
        self._own = own
        self._bank = _finite(estimated_bank_deg, float("nan"))
        self._course = _finite(getattr(own, "course_deg", float("nan")), float("nan")) \
            if own is not None else float("nan")
        self.update()

    # ------------------------------------------------------------------ painting
    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.setRenderHint(QtGui.QPainter.TextAntialiasing, True)
        tape = 16
        box = QtCore.QRectF(self.rect()).adjusted(1, 1, -1, -1)
        ball = QtCore.QRectF(box.left(), box.top() + tape, box.width(), box.height() - tape - 16)
        side = min(ball.width(), ball.height())
        ball = QtCore.QRectF(ball.center().x() - side / 2, ball.top(), side, side)

        p.fillRect(box, BEZEL)
        if self._own is None:
            p.setPen(TEXT)
            p.drawText(box, QtCore.Qt.AlignCenter, "no ownship row")
            return

        pitch = _finite(self._own.pitch_deg)
        roll = _finite(self._own.roll_deg)
        yaw = _finite(self._own.yaw_deg)

        scale = max(ball.width() / REFERENCE_BALL_PX, 0.55)
        self._draw_ball(p, ball, pitch, roll, scale)
        self._draw_roll_scale(p, ball, roll)
        self._draw_symbol(p, ball, scale)
        self._draw_heading_tape(p, QtCore.QRectF(box.left(), box.top(), box.width(), tape), yaw)
        self._draw_readout(p, QtCore.QRectF(box.left(), box.bottom() - 15, box.width(), 15),
                           yaw, pitch, roll)

    def _draw_ball(self, p, ball, pitch, roll, scale=1.0):
        """Sky, ground and the pitch ladder, rolled and pitched about the centre."""
        p.save()
        path = QtGui.QPainterPath()
        path.addEllipse(ball)
        p.setClipPath(path)
        p.translate(ball.center())
        # + roll is right wing down, so the horizon's RIGHT side rises; screen y is down, so
        # that is a negative rotation in Qt's sense
        p.rotate(-roll)
        # + pitch is nose up, which carries the horizon DOWN the instrument
        p.translate(0.0, pitch * PX_PER_DEG * scale)

        reach = ball.width() * 2.0                      # overdraw: the ball is rotated inside
        p.fillRect(QtCore.QRectF(-reach, -reach, 2 * reach, reach), SKY)
        p.fillRect(QtCore.QRectF(-reach, 0.0, 2 * reach, reach), GROUND)
        p.setPen(QtGui.QPen(HORIZON, 1.4))
        p.drawLine(QtCore.QPointF(-reach, 0.0), QtCore.QPointF(reach, 0.0))

        f = QtGui.QFont(self.font())
        f.setPointSizeF(6.5)
        p.setFont(f)
        p.setPen(QtGui.QPen(LADDER, 1.0))
        for deg in LADDER_STEPS:
            y = -deg * PX_PER_DEG * scale
            half = (20.0 if deg % 20 == 0 else 12.0) * scale
            p.drawLine(QtCore.QPointF(-half, y), QtCore.QPointF(half, y))
            if deg % 20 == 0:
                for side in (-1, 1):
                    r = QtCore.QRectF(side * (half + 2) - (0 if side > 0 else 18), y - 6, 18, 12)
                    p.drawText(r, QtCore.Qt.AlignVCenter
                               | (QtCore.Qt.AlignLeft if side > 0 else QtCore.Qt.AlignRight),
                               f"{abs(deg)}")
        p.restore()

    def _draw_roll_scale(self, p, ball, roll):
        """Fixed tick scale round the top of the ball, with the recorded roll pointer and,
        separately, the estimated bank."""
        c = ball.center()
        r = ball.width() / 2.0
        p.save()
        p.setPen(QtGui.QPen(FRAME, 1.4))
        p.drawEllipse(ball)
        p.setPen(QtGui.QPen(LADDER, 1.0))
        for deg in ROLL_TICKS:
            a = math.radians(90.0 + deg)                # measured up from the top of the dial
            inner = r - (6.0 if deg % 30 == 0 else 3.5)
            p.drawLine(QtCore.QPointF(c.x() - r * math.cos(a), c.y() - r * math.sin(a)),
                       QtCore.QPointF(c.x() - inner * math.cos(a), c.y() - inner * math.sin(a)))

        def pointer(angle_deg, colour, filled):
            a = math.radians(90.0 + angle_deg)
            tip = QtCore.QPointF(c.x() - (r - 7) * math.cos(a), c.y() - (r - 7) * math.sin(a))
            base = QtCore.QPointF(c.x() - (r - 15) * math.cos(a), c.y() - (r - 15) * math.sin(a))
            n = QtCore.QPointF(-(tip.y() - base.y()), tip.x() - base.x())
            length = math.hypot(n.x(), n.y()) or 1.0
            n = QtCore.QPointF(n.x() / length * 4.0, n.y() / length * 4.0)
            tri = QtGui.QPolygonF([tip, base + n, base - n])
            p.setPen(QtGui.QPen(colour, 1.2))
            p.setBrush(QtGui.QBrush(colour) if filled else QtCore.Qt.NoBrush)
            p.drawPolygon(tri)

        pointer(roll, RECORDED, True)                   # what the export records
        if math.isfinite(self._bank) and abs(self._bank - roll) > 0.5:
            # hollow, and a different colour: a derived value must never look like a reading
            pointer(self._bank, ESTIMATED, False)
        p.setBrush(QtCore.Qt.NoBrush)
        p.restore()

    def _draw_symbol(self, p, ball, scale=1.0):
        """The fixed aircraft: the thing the horizon moves behind."""
        c = ball.center()
        p.save()
        p.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 170), 3.0))
        for _ in range(2):                              # dark outline, then the symbol over it
            wing, inner, drop = 26.0 * scale, 9.0 * scale, 6.0 * scale
            p.drawLine(QtCore.QPointF(c.x() - wing, c.y()), QtCore.QPointF(c.x() - inner, c.y()))
            p.drawLine(QtCore.QPointF(c.x() + inner, c.y()), QtCore.QPointF(c.x() + wing, c.y()))
            p.drawLine(QtCore.QPointF(c.x() - inner, c.y()), QtCore.QPointF(c.x(), c.y() + drop))
            p.drawLine(QtCore.QPointF(c.x(), c.y() + drop), QtCore.QPointF(c.x() + inner, c.y()))
            p.setPen(QtGui.QPen(SYMBOL, 1.6))
        p.restore()

    def _draw_heading_tape(self, p, strip, yaw):
        """A moving compass tape: the recorded Yaw under a fixed index."""
        p.save()
        p.setClipRect(strip)
        p.fillRect(strip, QtGui.QColor("#18222e"))
        px_per_deg = 1.5
        centre = strip.center().x()
        f = QtGui.QFont(self.font())
        f.setPointSizeF(6.5)
        p.setFont(f)
        span = int(strip.width() / (2 * px_per_deg)) + 12
        start = int(round(yaw)) - span
        for deg in range(start, start + 2 * span + 1):
            if deg % 5:
                continue
            x = centre + ((deg - yaw + 540.0) % 360.0 - 180.0) * px_per_deg
            if not (strip.left() - 12 <= x <= strip.right() + 12):
                continue
            if deg % 30 == 0:
                p.setPen(QtGui.QPen(LADDER, 1.0))
                p.drawLine(QtCore.QPointF(x, strip.bottom() - 4), QtCore.QPointF(x, strip.bottom()))
                p.setPen(TEXT)
                label = {0: "N", 90: "E", 180: "S", 270: "W"}.get(deg % 360, f"{deg % 360:03d}")
                p.drawText(QtCore.QRectF(x - 14, strip.top() - 1, 28, strip.height() - 3),
                           QtCore.Qt.AlignCenter, label)
            else:
                p.setPen(QtGui.QPen(FRAME, 1.0))
                p.drawLine(QtCore.QPointF(x, strip.bottom() - 2), QtCore.QPointF(x, strip.bottom()))
        p.setPen(QtGui.QPen(HORIZON, 1.2))
        p.drawLine(QtCore.QPointF(centre, strip.top()), QtCore.QPointF(centre, strip.bottom()))
        p.restore()

    def _draw_readout(self, p, strip, yaw, pitch, roll):
        """The three numbers, so the instrument is never the only way to read them."""
        f = QtGui.QFont(self.font())
        f.setPointSizeF(7.0)
        p.setFont(f)
        p.setPen(TEXT)
        text = f"YAW {yaw:06.2f}°   PITCH {pitch:+06.2f}°   ROLL {roll:+06.2f}°"
        p.drawText(strip, QtCore.Qt.AlignCenter, text)
        if math.isfinite(self._bank) and abs(self._bank - roll) > 0.5:
            p.setPen(ESTIMATED)
            p.drawText(strip, QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
                       f"est bank {self._bank:+.1f}°  ")
