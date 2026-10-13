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

    # Identity, as decided upstream and carried on the packet. The display never works any of
    # this out for itself: see docs/IDENTITY.md.
    #   auth  - the IFF interrogation result in MultiTrack's own words: "friend" or
    #           "unknown". "unknown" is the ABSENCE of a reply, never evidence against.
    #   icao  - the 24-bit address that authenticated it, spelled as the source spells it
    #           ("0x000FA0"), or None when the track has none.
    # Truth targets read from the Scenario Export carry auth "" and icao None: the export is
    # ground truth, not an interrogation, and must never look like one.
    auth: str = ""
    icao: str | None = None

    row: int = -1
    associated_tracks: tuple = ()

    @property
    def authenticated(self) -> bool:
        """True only when an IFF interrogation actually authenticated this track."""
        return self.auth == "friend"

    @property
    def auth_label(self) -> str:
        """How the authentication reads in the UI, kept in one place.

        Deliberately not the same words as the classification: MultiTrack decides identity
        with a single field, and showing 'Friendly / Authenticated' twice would imply two
        independent confirmations where there is only one.
        """
        if self.auth == "friend":
            return "Authenticated (IFF)"
        if self.auth == "unknown":
            return "No IFF reply"
        return "—"

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
