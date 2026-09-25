"""Kinematic relations used by the display.

Everything here derives quantities *from* recorded values. Nothing replaces a
recorded position with an integrated one: the only propagation in this module
(``predict_coordinated_turn``) produces a clearly separate predicted path.

Omega sign convention
---------------------
The track file carries a single scalar turn rate ``Omega`` (rad/s). It is a
rate about the local vertical, not a body-rate vector. Its sign is set in the
configuration (``kinematics.omega_positive``):

    "clockwise"         Omega = d(Yaw)/dt of the compass heading.
                        Positive = right turn (seen from above).
    "counterclockwise"  Omega = rate about ENU +Z (Up).
                        Positive = left turn (seen from above).

``heading_rate`` converts either to the compass heading rate.
"""

from __future__ import annotations

import numpy as np

G = 9.81  # m/s^2


def heading_rate(omega, omega_positive: str = "clockwise"):
    """Compass heading rate (rad/s, positive = right turn) from the file Omega."""
    if omega_positive == "clockwise":
        return omega
    if omega_positive == "counterclockwise":
        return -np.asarray(omega)
    raise ValueError(f"unknown omega_positive convention {omega_positive!r}")


def speed(vx, vy, vz):
    return np.sqrt(np.asarray(vx) ** 2 + np.asarray(vy) ** 2 + np.asarray(vz) ** 2)


def horizontal_speed(vx, vy):
    return np.hypot(vx, vy)


def yaw_from_velocity_deg(vx, vy):
    """Compass heading from ENU velocity: Yaw = atan2(Vx, Vy), in [0, 360)."""
    return np.degrees(np.arctan2(vx, vy)) % 360.0


def pitch_from_velocity_deg(vx, vy, vz):
    """Flight-path pitch: atan2(Vz, sqrt(Vx^2 + Vy^2)). Positive = climbing / nose-up."""
    return np.degrees(np.arctan2(vz, np.hypot(vx, vy)))


def estimated_roll_deg(v_horizontal, heading_rate_rad_s, g: float = G, limit_deg: float = 80.0):
    """Coordinated-turn bank angle: tan(Roll) = V_h * Omega / g.

    This is an ESTIMATE derived from the turn rate; the track file carries no
    measured roll. Positive = right wing down (right turn).
    """
    roll = np.degrees(np.arctan2(np.asarray(v_horizontal) * np.asarray(heading_rate_rad_s), g))
    return np.clip(roll, -limit_deg, limit_deg)


def acceleration_components(v, a):
    """Split acceleration into along-track and cross-track magnitudes.

    Returns (a_along, a_cross) for vectors v, a of shape (3,).
    """
    v = np.asarray(v, dtype=float)
    a = np.asarray(a, dtype=float)
    s = np.linalg.norm(v)
    if s < 1e-9:
        return 0.0, float(np.linalg.norm(a))
    along = float(np.dot(a, v) / s)
    cross = float(np.linalg.norm(a - along * v / s))
    return along, cross


def predict_coordinated_turn(p, v, omega_heading, horizon_s: float, n: int = 40) -> np.ndarray:
    """Predicted path over ``horizon_s`` assuming a constant-rate coordinated turn.

    ``omega_heading`` is the compass heading rate (rad/s, positive = right turn).
    Horizontal speed and vertical rate are held constant. Returns (n, 3) points,
    the first being ``p``. This is a PREDICTION for display only.
    """
    p = np.asarray(p, dtype=float)
    vx, vy, vz = (float(c) for c in v)
    t = np.linspace(0.0, horizon_s, n)
    vh = np.hypot(vx, vy)
    psi0 = np.arctan2(vx, vy)
    w = float(omega_heading) if np.isfinite(omega_heading) else 0.0

    if abs(w) < 1e-6 or vh < 1e-6:
        dx, dy = vx * t, vy * t
    else:
        # x' = Vh sin(psi0 + w t),  y' = Vh cos(psi0 + w t)
        dx = vh / w * (np.cos(psi0) - np.cos(psi0 + w * t))
        dy = vh / w * (np.sin(psi0 + w * t) - np.sin(psi0))
    return np.column_stack([p[0] + dx, p[1] + dy, p[2] + vz * t])


def observed_heading_rate(t, vx, vy):
    """Heading rate (rad/s, clockwise positive) observed from successive velocities."""
    psi = np.unwrap(np.arctan2(np.asarray(vx, float), np.asarray(vy, float)))
    dt = np.diff(np.asarray(t, float))
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.diff(psi) / dt


G = 9.80665


def attitude_check(yaw_deg, pitch_deg, roll_deg, vx, vy, vz, yaw_rate_deg_s=None,
                   roll_source="exported") -> dict:
    """Compare a recorded attitude with what its own velocity implies. Validation only.

    The recorded values are never replaced: this returns the recorded attitude, the values
    derived from velocity and their differences, so the display can show the disagreement.

        heading = atan2(Vx, Vy)                 compass, 0 = North (the project's convention)
        flight path = atan2(Vz, hypot(Vx, Vy))  climb angle
        d_pitch = recorded pitch - flight path  (a steady offset is the angle of attack)
        estimated bank = atan(V * yaw_rate / g) coordinated-turn estimate, NEVER measured roll
    """
    speed_h = float(np.hypot(vx, vy))
    speed = float(np.sqrt(vx * vx + vy * vy + vz * vz))
    heading = float(yaw_from_velocity_deg(vx, vy)) if speed_h > 0.1 else float("nan")
    fpa = float(pitch_from_velocity_deg(vx, vy, vz)) if speed > 0.1 else float("nan")
    bank = float(np.degrees(np.arctan(speed * np.radians(yaw_rate_deg_s) / G))) \
        if yaw_rate_deg_s is not None and np.isfinite(yaw_rate_deg_s) else float("nan")
    return {"yaw_deg": float(yaw_deg), "pitch_deg": float(pitch_deg), "roll_deg": float(roll_deg),
            "heading_deg": heading, "fpa_deg": fpa,
            "d_heading_deg": float((yaw_deg - heading + 180.0) % 360.0 - 180.0) if np.isfinite(heading) else float("nan"),
            "d_pitch_deg": float(pitch_deg - fpa) if np.isfinite(fpa) else float("nan"),
            "yaw_rate_deg_s": float(yaw_rate_deg_s) if yaw_rate_deg_s is not None else float("nan"),
            "estimated_bank_deg": bank, "roll_source": roll_source, "speed_mps": speed}
