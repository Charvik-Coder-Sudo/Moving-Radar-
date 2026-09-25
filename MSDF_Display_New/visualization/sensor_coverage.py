"""Sensor coverage geometry.

One coverage volume per physical sensor: the spherical sector

    |az| <= az_coverage / 2,   |el| <= el_coverage / 2,   range <= r_max

about the radar boresight, built in the radar's own FRD frame and carried into
the world by the sensor pose (P_WR, R_WR). Coverage is the complete field of
regard; the instantaneous beam lives in scan_beam.py and is never merged into it.
Tracks are never used to shape coverage.
"""

from __future__ import annotations

import numpy as np

from models.sensor_state import SensorConfig, SensorState
from msdf_math import coordinates


def coverage_surface_frd(cfg: SensorConfig, rng: float, n_az: int = 36, n_el: int = 18):
    """Closed triangle mesh of the coverage sector in radar FRD.

    Returns (points (N,3), faces (M,3) int). Far surface + four side faces meet at
    the radar origin (index 0).
    """
    az0, az1 = cfg.az_limits
    el0, el1 = cfg.el_limits
    azs = np.linspace(az0, az1, n_az + 1)
    els = np.linspace(el0, el1, n_el + 1)
    A, E = np.meshgrid(azs, els)                       # (n_el+1, n_az+1)
    far = coordinates.frd_direction(A, E) * rng        # (n_el+1, n_az+1, 3)
    pts = np.vstack([np.zeros((1, 3)), far.reshape(-1, 3)])

    def vid(i, j):
        return 1 + i * (n_az + 1) + j

    faces = []
    for i in range(n_el):
        for j in range(n_az):
            a, b, c, d = vid(i, j), vid(i, j + 1), vid(i + 1, j + 1), vid(i + 1, j)
            faces += [(a, b, c), (a, c, d)]
    # side faces: fan from the origin along each boundary of the far surface
    for j in range(n_az):
        faces.append((0, vid(0, j + 1), vid(0, j)))            # bottom (el0)
        faces.append((0, vid(n_el, j), vid(n_el, j + 1)))      # top (el1)
    for i in range(n_el):
        faces.append((0, vid(i, 0), vid(i + 1, 0)))            # left (az0)
        faces.append((0, vid(i + 1, n_az), vid(i, n_az)))      # right (az1)
    return pts, np.array(faces, dtype=np.int64)


def coverage_edges_frd(cfg: SensorConfig, rng: float, n: int = 48) -> list[np.ndarray]:
    """Outline polylines in radar FRD: 4 radial edges, 4 far-boundary arcs, boresight."""
    az0, az1 = cfg.az_limits
    el0, el1 = cfg.el_limits
    out = []
    for a, e in ((az0, el0), (az1, el0), (az1, el1), (az0, el1)):
        out.append(np.vstack([np.zeros(3), coordinates.frd_direction(a, e) * rng]))
    azs = np.linspace(az0, az1, n)
    els = np.linspace(el0, el1, n)
    for e in (el0, el1, 0.0):
        out.append(coordinates.frd_direction(azs, np.full(n, e)) * rng)
    for a in (az0, az1):
        out.append(coordinates.frd_direction(np.full(n, a), els) * rng)
    return out


def footprint_xy(state: SensorState, rng: float, n_az: int = 13, n_el: int = 7) -> np.ndarray:
    """Top-down (X East, Y North) outline of the coverage volume projected on the ground plane.

    Convex hull of the projected boundary; the sector's azimuth span is below 180
    degrees so the projection of the volume is convex.
    """
    cfg = state.config
    az0, az1 = cfg.az_limits
    el0, el1 = cfg.el_limits
    A, E = np.meshgrid(np.linspace(az0, az1, n_az), np.linspace(el0, el1, n_el))
    far = coordinates.frd_direction(A, E).reshape(-1, 3) * rng
    pts = state.to_world(np.vstack([np.zeros((1, 3)), far]))
    return coordinates.convex_hull_2d(pts[:, :2])


def ground_sector_xy(state: SensorState, rng: float, n: int = 49):
    """Plan view of the field of regard: (outline, arcs) in world X/Y for the 2D map.

    A plan view answers one question - **where on the ground can this sensor look** - so the
    field of regard is drawn as the azimuth sector it really has:

        apex      the sensor's own X/Y position
        edges     the radar's own azimuth limits, taken through the sensor pose, so the sector
                  turns with the aircraft
        arc       the sensor's own maximum range

    The azimuth limits are evaluated in the radar's horizontal plane (elevation 0) and then
    projected, which is what a plan view of a field of regard means. Projecting the whole 3D
    volume and taking its convex hull instead - the previous drawing - produced a blob whose
    outline was the elevation extent, not the azimuth coverage, and hid the range limit.

    ``arcs`` are range rings at fractions of r_max: scale without another filled shape.
    """
    az0, az1 = state.config.az_limits
    azs = np.linspace(az0, az1, n)
    dirs = coordinates.frd_direction(azs, np.zeros(n))          # radar FRD, horizontal plane
    ground = state.to_world(dirs * rng)[:, :2] - np.asarray(state.P_WR, float)[:2]
    # the plan view shows ground distance, so each azimuth reaches the sensor's own maximum
    # range on the map rather than the foreshortened projection of a pitched radar plane
    length = np.linalg.norm(ground, axis=1, keepdims=True)
    far = np.asarray(state.P_WR, float)[:2] + ground / np.where(length > 1e-9, length, 1.0) * rng
    apex = np.asarray(state.P_WR, float)[:2]
    outline = np.vstack([apex, far, apex])
    arcs = [np.vstack([apex + (p - apex) * f for p in far]) for f in (0.33, 0.66)]
    return outline, arcs


def ppi_sector(cfg: SensorConfig, rng: float, n: int = 64) -> np.ndarray:
    """PPI (radar-frame) coverage sector outline as (x right, y forward) points."""
    az = np.radians(np.linspace(*cfg.az_limits, n))
    arc = np.column_stack([rng * np.sin(az), rng * np.cos(az)])
    return np.vstack([[0.0, 0.0], arc, [0.0, 0.0]])
