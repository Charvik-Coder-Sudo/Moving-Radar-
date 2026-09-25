"""Track record used throughout the display.

``TrackState`` holds one track as last reported by the RDP (a fused/system
track or one sensor track carried in a SystemTrack packet), in SI units:

    position      m        sensor-relative frame of the RDP: x Right, y Forward, z Up
    velocity      m/s
    acceleration  m/s^2
    omega         rad/s    the state's ``w`` element
    timestamp     s        packet time, as sent by the RDP

``associated_tracks`` lists, for a fused track, the sensor tracks contained in
the same packet as ((source, track_id), ...). It is never inferred.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

KIND_OWNSHIP = "ownship"
KIND_TARGET = "target"      # truth target (not currently displayed)
KIND_SENSOR = "sensor"      # individual sensor track (PRIMARY, SECONDARY, ...)
KIND_FUSED = "fused"        # fused / system track


@dataclass(frozen=True, slots=True)
class TrackState:
    timestamp: float
    track_id: str
    target_id: str

    x: float
    y: float
    z: float

    vx: float
    vy: float
    vz: float

    omega: float

    ax: float
    ay: float
    az: float

    classification: str
    source: str
    status: str

    row: int = -1
    associated_tracks: tuple = ()

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
    def speed(self) -> float:
        return float(np.sqrt(self.vx * self.vx + self.vy * self.vy + self.vz * self.vz))

    @property
    def horizontal_speed(self) -> float:
        return float(np.hypot(self.vx, self.vy))

    @property
    def has_acceleration(self) -> bool:
        return bool(np.all(np.isfinite([self.ax, self.ay, self.az])))

    @property
    def has_omega(self) -> bool:
        return bool(np.isfinite(self.omega))

    @property
    def contributing_sensors(self) -> tuple[str, ...]:
        """Distinct sensor sources in ``associated_tracks``, in packet order."""
        return tuple(dict.fromkeys(src for src, _ in self.associated_tracks))
