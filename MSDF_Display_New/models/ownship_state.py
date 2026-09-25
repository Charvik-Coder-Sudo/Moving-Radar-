"""Ownship 6-DoF state.

Pose   = [X, Y, Z, Yaw, Pitch, Roll]
Motion = [Vx, Vy, Vz, Ax, Ay, Az, Omega]

Filled from the Scenario Export ownship file: position, velocity and the
exported Yaw / Pitch / Roll (``attitude_source``). The export carries no
acceleration or turn rate, so those are NaN and shown as unavailable.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True)
class OwnshipState:
    timestamp: float         # seconds (display)
    track_id: str

    # pose
    x: float
    y: float
    z: float
    yaw_deg: float           # compass heading, 0 = North, 90 = East
    pitch_deg: float         # + = nose up
    roll_deg: float          # + = right wing down

    # motion
    vx: float
    vy: float
    vz: float
    ax: float
    ay: float
    az: float
    omega: float             # rad/s

    heading_valid: bool
    roll_valid: bool
    R_WB: np.ndarray         # aircraft FRD -> world ENU, from the EXPORTED Yaw/Pitch/Roll
    row: int = -1
    timestamp_ms: float = float("nan")      # canonical Scenario Export timestamp
    attitude_source: str = ""

    # Attitude the recorded path implies (msdf_math.motion). The exported Yaw disagrees with
    # the flown path in this scenario (swapped Vx/Vy upstream), so the views draw the aircraft
    # with R_WB_display while the exported values above stay visible and reported.
    course_deg: float = float("nan")        # compass course over ground, from the positions
    climb_deg: float = float("nan")         # flight path angle from the positions
    bank_deg: float = float("nan")          # coordinated-turn estimate, never measured
    path_speed_mps: float = float("nan")
    R_WB_display: np.ndarray | None = None  # what the 3D view poses the model with
    attitude_display_source: str = ""
    attitude_delta_deg: float = float("nan")  # exported Yaw - course over ground

    @property
    def R_WB_drawn(self) -> np.ndarray:
        """Rotation the views draw with: path-derived when available, exported otherwise."""
        return self.R_WB if self.R_WB_display is None else self.R_WB_display

    @property
    def heading_drawn(self) -> float:
        """Compass heading the views draw the aircraft with (course over ground when derivable)."""
        return self.course_deg if np.isfinite(self.course_deg) else self.yaw_deg

    @property
    def attitude_disagrees(self) -> bool:
        """True when the exported Yaw and the flown course differ enough to report."""
        return bool(np.isfinite(self.attitude_delta_deg) and abs(self.attitude_delta_deg) > 2.0)

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @property
    def velocity(self) -> np.ndarray:
        return np.array([self.vx, self.vy, self.vz])

    @property
    def acceleration(self) -> np.ndarray:
        return np.array([self.ax, self.ay, self.az])

    @property
    def pose(self) -> list[float]:
        return [self.x, self.y, self.z, self.yaw_deg, self.pitch_deg, self.roll_deg]

    @property
    def motion(self) -> list[float]:
        return [self.vx, self.vy, self.vz, self.ax, self.ay, self.az, self.omega]

    @property
    def speed(self) -> float:
        return float(np.sqrt(self.vx ** 2 + self.vy ** 2 + self.vz ** 2))
