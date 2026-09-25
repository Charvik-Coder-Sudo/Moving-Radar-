"""Attitude derived from the path an aircraft actually flies.

Why this module exists
----------------------
In the recorded scenario the exported attitude does not agree with the exported path.
Measured over the whole flight (and in the source trajectory files, ``*_ms.tdf``):

    ownship path velocity      (136.006,  63.266, +10) m/s    ->  course  65.05 deg
    ownship velocity columns   ( 63.266, 136.005, +10) m/s    ->  yaw     24.95 deg

Same speed (150.000 m/s), the East and North components exchanged: the ``Vx`` / ``Vy``
columns are swapped with respect to ``X`` / ``Y``. The position columns are the consistent
ones - the export's own ``Azimuth`` column equals ``atan2(X, Y)`` (compass) to 0.0001 deg,
and every radar measurement is built from the positions. The exported ``Yaw`` is computed
upstream as ``atan2(Vx, Vy)`` (``Coordinate_Geometry.Velocity_Altitudes``), so it inherits
the swap and is 40.1 deg away from the direction the ownship travels.

This module derives the attitude the aircraft must have to fly the recorded path:

    course over ground  atan2(dEast, dNorth)        compass, 0 = North
    climb angle         atan2(dUp, horizontal)      + = climbing
    bank                atan(V * course rate / g)   coordinated turn, ESTIMATE

Nothing here changes recorded data. The recorded values stay visible next to the derived
ones (Ownship panel, hover, DEBUG overlay) together with their difference, so the upstream
inconsistency stays reported rather than hidden. See docs/COORDINATE_AND_ATTITUDE.md.

Positions are recorded to 0.1 m, so a course taken from two neighbouring rows (3 m apart at
150 m/s) would carry about 2 deg of quantisation noise. The baseline is therefore widened to
``window_s`` of recorded rows, which also makes a turning aircraft rotate smoothly instead of
stepping between samples.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

G = 9.80665
MIN_BASELINE_M = 5.0          # shorter than this, 0.1 m position rounding dominates the course
RATE_DEADBAND_DEG_S = 0.05    # below this the aircraft is flying straight: no estimated bank


@dataclass(frozen=True, slots=True)
class PathAttitude:
    """Attitude implied by the recorded path. ``valid`` is False when it cannot be derived."""

    course_deg: float = float("nan")      # compass, 0 = North, 90 = East
    climb_deg: float = float("nan")       # flight path angle, + = climbing
    bank_deg: float = float("nan")        # coordinated-turn estimate, never a measurement
    speed_mps: float = float("nan")       # speed along the path
    course_rate_deg_s: float = float("nan")
    baseline_s: float = float("nan")
    baseline_m: float = float("nan")
    valid: bool = False


def course_climb(delta_enu, dt_s: float) -> tuple[float, float, float]:
    """(course deg, climb deg, speed m/s) of one ENU displacement over ``dt_s`` seconds."""
    d = np.asarray(delta_enu, dtype=float)
    horizontal = float(np.hypot(d[0], d[1]))
    course = float(np.degrees(np.arctan2(d[0], d[1])) % 360.0)
    climb = float(np.degrees(np.arctan2(d[2], horizontal))) if horizontal > 0 or d[2] else 0.0
    speed = float(np.linalg.norm(d) / dt_s) if dt_s > 0 else float("nan")
    return course, climb, speed


def angle_difference(a_deg, b_deg):
    """a - b wrapped to [-180, +180)."""
    return (np.asarray(a_deg, dtype=float) - np.asarray(b_deg, dtype=float) + 180.0) % 360.0 - 180.0


def blend_heading(previous_deg: float, new_deg: float, alpha: float) -> float:
    """Exponential smoothing of a compass heading, the short way round.

    ``alpha`` is the weight of the new value (1.0 = no smoothing). Used for headings taken
    from sparse live updates, where two consecutive positions can be only a few metres apart.
    """
    if not np.isfinite(previous_deg):
        return float(new_deg)
    return float((previous_deg + alpha * angle_difference(new_deg, previous_deg)) % 360.0)


def bank_estimate_deg(speed_mps: float, course_rate_deg_s: float, limit_deg: float = 60.0) -> float:
    """Coordinated-turn bank angle: tan(phi) = V * psi_dot / g. ESTIMATE, not a measurement."""
    if not (np.isfinite(speed_mps) and np.isfinite(course_rate_deg_s)):
        return float("nan")
    bank = np.degrees(np.arctan(speed_mps * np.radians(course_rate_deg_s) / G))
    return float(np.clip(bank, -limit_deg, limit_deg))


def path_attitude(t_ms: np.ndarray, pos: np.ndarray, row: int, window_s: float = 0.5,
                  rate_window_s: float = 2.0) -> PathAttitude:
    """Attitude implied by the recorded rows around ``row`` (centred difference).

    t_ms           (N,) recorded timestamps, ms, increasing
    pos            (N,3) recorded ENU positions, m
    window_s       half-window for the course and climb angle
    rate_window_s  half-window for the course rate, from which the bank is estimated. It is
                   wider on purpose: a rate is a second difference, so on positions rounded
                   to 0.1 m a short baseline would make a straight-flying aircraft rock.
    """
    n = len(t_ms)
    if n < 2 or not 0 <= row < n:
        return PathAttitude()
    i0, i1 = _window(t_ms, row, window_s)
    if i0 is None:
        return PathAttitude()
    d = np.asarray(pos[i1], float) - np.asarray(pos[i0], float)
    dt = float(t_ms[i1] - t_ms[i0]) / 1000.0
    baseline = float(np.linalg.norm(d))
    if dt <= 0 or baseline < MIN_BASELINE_M:
        return PathAttitude(baseline_s=dt, baseline_m=baseline)
    course, climb, speed = course_climb(d, dt)
    rate = _course_rate(t_ms, pos, row, max(rate_window_s, window_s))
    return PathAttitude(course_deg=course, climb_deg=climb,
                        bank_deg=bank_estimate_deg(speed, rate), speed_mps=speed,
                        course_rate_deg_s=rate, baseline_s=dt, baseline_m=baseline, valid=True)


def _window(t_ms, row, window_s):
    """Row indices ``window_s`` either side of ``row`` (clamped to the file), or (None, None)."""
    n = len(t_ms)
    half = window_s * 1000.0
    i0 = int(np.searchsorted(t_ms, t_ms[row] - half, side="left"))
    i1 = int(np.searchsorted(t_ms, t_ms[row] + half, side="right")) - 1
    i0, i1 = max(0, min(i0, row)), min(n - 1, max(i1, row))
    if i1 > i0:
        return i0, i1
    i0, i1 = (row - 1, row) if row > 0 else (row, row + 1)       # at an end of the file
    return (i0, i1) if 0 <= i0 < i1 < n else (None, None)


def _course_rate(t_ms, pos, row, window_s) -> float:
    """Course rate (deg/s) from the two half-windows either side of ``row``."""
    i0, i1 = _window(t_ms, row, window_s)
    if i0 is None or i1 - i0 < 2:
        return float("nan")
    mid = row if i0 < row < i1 else (i0 + i1) // 2
    first = np.asarray(pos[mid], float) - np.asarray(pos[i0], float)
    second = np.asarray(pos[i1], float) - np.asarray(pos[mid], float)
    if np.hypot(first[0], first[1]) < MIN_BASELINE_M or np.hypot(second[0], second[1]) < MIN_BASELINE_M:
        return float("nan")
    c0 = np.degrees(np.arctan2(first[0], first[1]))
    c1 = np.degrees(np.arctan2(second[0], second[1]))
    dt = float(t_ms[i1] - t_ms[i0]) / 2000.0        # centre-to-centre of the two half windows
    if dt <= 0:
        return float("nan")
    rate = float(angle_difference(c1, c0) / dt)
    return 0.0 if abs(rate) < RATE_DEADBAND_DEG_S else rate


def trail_attitude(trail: np.ndarray, times_s=None, window_s: float = 2.0) -> PathAttitude:
    """Same, for a live track's own position history (RDP world trail).

    The newest points are used; the baseline grows until it is long enough to be meaningful,
    so a track that has only just appeared keeps ``valid = False`` rather than jittering.
    """
    if trail is None:
        return PathAttitude()
    pts = np.asarray(trail, dtype=float).reshape(-1, 3)
    if len(pts) < 2:
        return PathAttitude()
    if times_s is not None and len(times_s) == len(pts):
        t_ms = np.asarray(times_s, float) * 1000.0
        return path_attitude(t_ms, pts, len(pts) - 1, window_s=window_s)
    j = len(pts) - 1
    i = j - 1
    while i > 0 and np.linalg.norm(pts[j] - pts[i]) < MIN_BASELINE_M:
        i -= 1
    d = pts[j] - pts[i]
    baseline = float(np.linalg.norm(d))
    if baseline < MIN_BASELINE_M:
        return PathAttitude(baseline_m=baseline)
    course, climb, _speed = course_climb(d, 1.0)
    return PathAttitude(course_deg=course, climb_deg=climb, baseline_m=baseline, valid=True)
