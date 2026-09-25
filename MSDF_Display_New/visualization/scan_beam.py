"""Instantaneous scan-beam geometry.

The beam is the single dwell the scheduler is on at time t (models.sensor_state
BeamState). Two shapes are provided, both in radar FRD:

    beam pyramid     the 2 x 2 deg dwell itself (apex at the radar)
    azimuth column   the elevation column currently being swept at the beam's
                     azimuth. The pattern is azimuth-major: a whole column of
                     elevation bars is visited before the azimuth steps, so the
                     column is the part of the scan that moves smoothly on screen.

Neither is ever part of the coverage volume.
"""

from __future__ import annotations

import numpy as np

from models.sensor_state import BeamState, SensorConfig, SensorState
from msdf_math import coordinates


def beam_pyramid_frd(beam: BeamState, rng: float):
    """(points (5,3), faces (6,3)): apex + 4 far corners."""
    corners = coordinates.frd_direction(
        np.array([beam.left, beam.right, beam.right, beam.left]),
        np.array([beam.bottom, beam.bottom, beam.top, beam.top])) * rng
    pts = np.vstack([np.zeros((1, 3)), corners])
    faces = np.array([[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1], [1, 2, 3], [1, 3, 4]])
    return pts, faces


def beam_axis_frd(beam: BeamState, rng: float) -> np.ndarray:
    return np.vstack([np.zeros(3), coordinates.frd_direction(beam.az_deg, beam.el_deg) * rng])


def azimuth_column_frd(cfg: SensorConfig, beam: BeamState, rng: float, n: int = 16):
    """(points, faces) of the current elevation column at the beam azimuth."""
    el0, el1 = cfg.el_limits
    els = np.linspace(el0, el1, n)
    left = coordinates.frd_direction(np.full(n, beam.left), els) * rng
    right = coordinates.frd_direction(np.full(n, beam.right), els) * rng
    pts = np.vstack([np.zeros((1, 3)), left, right])
    faces = []
    for i in range(n - 1):
        a, b = 1 + i, 1 + i + 1
        c, d = 1 + n + i + 1, 1 + n + i
        faces += [(a, d, c), (a, c, b)]
    faces += [(0, 1, 1 + n), (0, n, 2 * n)]
    return pts, np.array(faces, dtype=np.int64)


def beam_footprint_xy(state: SensorState, rng: float) -> np.ndarray:
    """Top-down outline of the instantaneous beam (world X/Y)."""
    pts, _ = beam_pyramid_frd(state.beam, rng)
    world = state.to_world(pts)
    return coordinates.convex_hull_2d(world[:, :2])


def ppi_wedge(beam: BeamState, rng: float, n: int = 6) -> np.ndarray:
    """Beam wedge in PPI coordinates (x right, y forward)."""
    az = np.radians(np.linspace(beam.left, beam.right, n))
    arc = np.column_stack([rng * np.sin(az), rng * np.cos(az)])
    return np.vstack([[0.0, 0.0], arc, [0.0, 0.0]])
