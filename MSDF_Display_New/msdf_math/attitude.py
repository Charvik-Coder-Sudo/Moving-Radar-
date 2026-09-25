"""Attitude and rotation matrices.

The two rotation builders are copies of the existing project's
``Radar_Mathematics/Moving_Airborne_Geometry.py`` (``Coordinate_Geometry.
Aircraft_Body_to_World`` and ``Coordinate_Geometry.Radar_to_Aircraft_Body``),
reproduced here so that the display uses exactly the same equations without
importing (and therefore without touching) the existing code base.

    R_WB = T_ENU<-NED @ Rz(yaw) @ Ry(pitch) @ Rx(roll)     aircraft FRD -> world ENU
    R_BR = Rz(yaw) @ Ry(pitch) @ Rx(roll)                  radar FRD    -> aircraft FRD
    R_WR = R_WB @ R_BR                                     radar FRD    -> world ENU

The columns of R_WB are the Forward, Right and Down body axes expressed in
East / North / Up.

Angles are degrees. Yaw is a compass heading (0 = North, 90 = East), pitch is
positive nose-up and roll is positive right-wing-down.
"""

from __future__ import annotations

import numpy as np


def body_to_world(yaw_deg, pitch_deg, roll_deg=0.0) -> np.ndarray:
    """R_WB: aircraft FRD -> world ENU. Vectorised; returns (..., 3, 3)."""
    psi = np.radians(np.asarray(yaw_deg, dtype=float))
    theta = np.radians(np.asarray(pitch_deg, dtype=float))
    phi = np.radians(np.asarray(roll_deg, dtype=float))

    cos_psi, sin_psi = np.cos(psi), np.sin(psi)
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    cos_phi, sin_phi = np.cos(phi), np.sin(phi)

    R = np.empty(np.broadcast(psi, theta, phi).shape + (3, 3), dtype=float)

    R[..., 0, 0] = sin_psi * cos_theta
    R[..., 0, 1] = sin_psi * sin_theta * sin_phi + cos_psi * cos_phi
    R[..., 0, 2] = sin_psi * sin_theta * cos_phi - cos_psi * sin_phi

    R[..., 1, 0] = cos_psi * cos_theta
    R[..., 1, 1] = cos_psi * sin_theta * sin_phi - sin_psi * cos_phi
    R[..., 1, 2] = cos_psi * sin_theta * cos_phi + sin_psi * sin_phi

    R[..., 2, 0] = sin_theta
    R[..., 2, 1] = -cos_theta * sin_phi
    R[..., 2, 2] = -cos_theta * cos_phi
    return R


def radar_to_body(yaw_deg, pitch_deg, roll_deg=0.0) -> np.ndarray:
    """R_BR: radar FRD -> aircraft FRD (radar mounting). Returns (..., 3, 3)."""
    psi = np.radians(np.asarray(yaw_deg, dtype=float))
    theta = np.radians(np.asarray(pitch_deg, dtype=float))
    phi = np.radians(np.asarray(roll_deg, dtype=float))

    cy, sy = np.cos(psi), np.sin(psi)
    cp, sp = np.cos(theta), np.sin(theta)
    cr, sr = np.cos(phi), np.sin(phi)

    R = np.empty(np.broadcast(psi, theta, phi).shape + (3, 3), dtype=float)

    R[..., 0, 0] = cy * cp
    R[..., 0, 1] = cy * sp * sr - sy * cr
    R[..., 0, 2] = cy * sp * cr + sy * sr

    R[..., 1, 0] = sy * cp
    R[..., 1, 1] = sy * sp * sr + cy * cr
    R[..., 1, 2] = sy * sp * cr - cy * sr

    R[..., 2, 0] = -sp
    R[..., 2, 1] = cp * sr
    R[..., 2, 2] = cp * cr
    return R


def frd_to_ypr(R_W_frd: np.ndarray) -> tuple[float, float, float]:
    """Yaw / pitch / roll (deg) of an FRD frame expressed in world ENU.

    Same extraction as the existing ``Ownship.Radar_Angles``.
    """
    fwd_e, fwd_n, fwd_u = R_W_frd[0, 0], R_W_frd[1, 0], R_W_frd[2, 0]
    yaw = np.degrees(np.arctan2(fwd_e, fwd_n)) % 360.0
    pitch = np.degrees(np.arctan2(fwd_u, np.hypot(fwd_e, fwd_n)))
    roll = np.degrees(np.arctan2(-R_W_frd[2, 1], -R_W_frd[2, 2]))
    return float(yaw), float(pitch), float(roll)


def homogeneous(R: np.ndarray, p, scale: float = 1.0) -> np.ndarray:
    """4x4 transform  x_world = p + scale * R @ x_local."""
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float) * scale
    T[:3, 3] = np.asarray(p, dtype=float)
    return T
