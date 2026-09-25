"""Coordinate conversions between world ENU, radar FRD and spherical measurements.

Radar-frame measurement convention (existing project, ``Measurements_Geometry``
and the analysis notebook):

    Range     = |L|
    Azimuth   = atan2(L_R, L_F)                   0 = boresight, +90 = right
    Elevation = atan2(-L_D, sqrt(L_F^2 + L_R^2))  positive above the boresight plane

    direction(az, el) in FRD:  F = cos(el) cos(az),  R = cos(el) sin(az),  D = -sin(el)

World-frame relative geometry (used by the Track Details table) is expressed as
a compass bearing: 0 = North, 90 = East, measured from the ownship.
"""

from __future__ import annotations

import numpy as np


def wrap360(angle_deg):
    """Absolute angle into [0, 360)."""
    return np.mod(angle_deg, 360.0)


def wrap180(angle_deg):
    """Angular difference into [-180, +180)."""
    return (np.asarray(angle_deg, dtype=float) + 180.0) % 360.0 - 180.0


def frd_direction(az_deg, el_deg) -> np.ndarray:
    """Unit vector(s) in radar FRD for azimuth / elevation in degrees."""
    a = np.radians(np.asarray(az_deg, dtype=float))
    e = np.radians(np.asarray(el_deg, dtype=float))
    ce = np.cos(e)
    return np.stack([ce * np.cos(a), ce * np.sin(a), -np.sin(e)], axis=-1)


def world_to_frame(P_origin, R_W_frame, points_world) -> np.ndarray:
    """World ENU points -> local frame:  L = R^T (P - P_origin). (N,3) in, (N,3) out."""
    pts = np.atleast_2d(np.asarray(points_world, dtype=float))
    return (pts - np.asarray(P_origin, dtype=float)) @ np.asarray(R_W_frame, dtype=float)


def frame_to_world(P_origin, R_W_frame, points_local) -> np.ndarray:
    """Local frame points -> world ENU:  P = P_origin + R L. (N,3) in, (N,3) out."""
    pts = np.atleast_2d(np.asarray(points_local, dtype=float))
    return np.asarray(P_origin, dtype=float) + pts @ np.asarray(R_W_frame, dtype=float).T


def radar_measurements(L_frd) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Range (m), azimuth (deg, [0,360)), elevation (deg) from FRD line-of-sight vectors."""
    L = np.atleast_2d(np.asarray(L_frd, dtype=float))
    f, r, d = L[:, 0], L[:, 1], L[:, 2]
    rng = np.sqrt(f * f + r * r + d * d)
    az = np.degrees(np.arctan2(r, f)) % 360.0
    el = np.degrees(np.arctan2(-d, np.hypot(f, r)))
    return rng, az, el


def relative_rae_enu(origin, points) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Range, compass bearing and elevation of world points seen from ``origin``.

    Bearing is 0 = North, 90 = East (compass), elevation positive up.
    """
    d = np.atleast_2d(np.asarray(points, dtype=float)) - np.asarray(origin, dtype=float)
    de, dn, du = d[:, 0], d[:, 1], d[:, 2]
    horiz = np.hypot(de, dn)
    rng = np.sqrt(horiz * horiz + du * du)
    brg = np.degrees(np.arctan2(de, dn)) % 360.0
    el = np.degrees(np.arctan2(du, horiz))
    return rng, brg, el


def convex_hull_2d(points: np.ndarray) -> np.ndarray:
    """Monotone-chain convex hull of (N,2) points; returns the hull ring (M,2), CCW."""
    pts = np.unique(np.asarray(points, dtype=float), axis=0)
    if len(pts) < 3:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list = []
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])
