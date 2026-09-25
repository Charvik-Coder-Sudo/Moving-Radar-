"""Sensor configuration, scan schedule and per-frame sensor state.

The scan pattern reproduces the existing ``Sensor/Moving_Sensor_Airborne.py``
(``Moving_Radar.generate_scan_schedule``) so that the display's beam sits where
the simulation's beam sits:

    step      = beamwidth - overlap
    n_dwells  = ceil(1 + (coverage - beamwidth) / step)         per axis
    dwell     = scan_time / (n_az * n_el)
    g         = floor((t - start) / dwell)                      global dwell index
    scan_id   = g // (n_az * n_el)
    dwell_id  = g %  (n_az * n_el)
    az_idx    = dwell_id %  n_az        (azimuth steps fastest: a full azimuth bar
    el_idx    = dwell_id // n_az         is swept at each elevation, as the scheduler's
                                         Az_Dwell_ID / El_Dwell_ID)
    beam_az   = -cov_az/2 + bw_az/2 + az_offset + az_idx * step_az
    beam_el   = -cov_el/2 + bw_el/2 + el_offset + el_idx * step_el

Coverage (the whole field of regard) and the beam (the instantaneous dwell) are
kept strictly separate: ``SensorConfig`` describes the one coverage volume of a
sensor; ``BeamState`` describes where the beam is at one instant.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from msdf_math import attitude, coordinates


@dataclass(frozen=True)
class SensorConfig:
    sensor_id: str               # also the ``source`` value of this sensor's tracks
    name: str
    sensor_type: str             # "primary" or "secondary"
    color: str
    mount_xyz_frd: tuple         # m, in aircraft FRD
    mount_ypr_deg: tuple         # radar mounting yaw, pitch, roll relative to the body
    r_max: float                 # m
    az_coverage_deg: float
    el_coverage_deg: float
    az_beamwidth_deg: float
    el_beamwidth_deg: float
    az_overlap_deg: float
    el_overlap_deg: float
    az_offset_deg: float
    el_offset_deg: float
    scan_time_s: float
    start_time_s: float
    stop_time_s: float | None
    enabled: bool = True

    @classmethod
    def from_dict(cls, d: dict) -> "SensorConfig":
        return cls(
            sensor_id=str(d["id"]),
            name=str(d.get("name", d["id"])),
            sensor_type=str(d.get("type", "primary")),
            color=str(d.get("color", "#3b82f6")),
            mount_xyz_frd=tuple(float(v) for v in d.get("mount_xyz_frd", (0, 0, 0))),
            mount_ypr_deg=tuple(float(v) for v in d.get("mount_ypr_deg", (0, 0, 0))),
            r_max=float(d["r_max_m"]),
            az_coverage_deg=float(d["az_coverage_deg"]),
            el_coverage_deg=float(d["el_coverage_deg"]),
            az_beamwidth_deg=float(d["az_beamwidth_deg"]),
            el_beamwidth_deg=float(d["el_beamwidth_deg"]),
            az_overlap_deg=float(d.get("az_overlap_deg", 0.0)),
            el_overlap_deg=float(d.get("el_overlap_deg", 0.0)),
            az_offset_deg=float(d.get("az_offset_deg", 0.0)),
            el_offset_deg=float(d.get("el_offset_deg", 0.0)),
            scan_time_s=float(d["scan_time_s"]),
            start_time_s=float(d.get("start_time_s", 0.0)),
            stop_time_s=None if d.get("stop_time_s") is None else float(d["stop_time_s"]),
            enabled=bool(d.get("enabled", True)),
        )

    # ---------------- scan geometry ----------------

    @property
    def az_step(self) -> float:
        return self.az_beamwidth_deg - self.az_overlap_deg

    @property
    def el_step(self) -> float:
        return self.el_beamwidth_deg - self.el_overlap_deg

    @property
    def n_az(self) -> int:
        return math.ceil(1 + (self.az_coverage_deg - self.az_beamwidth_deg) / self.az_step)

    @property
    def n_el(self) -> int:
        return math.ceil(1 + (self.el_coverage_deg - self.el_beamwidth_deg) / self.el_step)

    @property
    def n_dwells(self) -> int:
        return self.n_az * self.n_el

    @property
    def dwell_time_s(self) -> float:
        return self.scan_time_s / self.n_dwells

    @property
    def az_limits(self) -> tuple[float, float]:
        return (-self.az_coverage_deg / 2.0, self.az_coverage_deg / 2.0)

    @property
    def el_limits(self) -> tuple[float, float]:
        return (-self.el_coverage_deg / 2.0, self.el_coverage_deg / 2.0)

    @property
    def R_BR(self) -> np.ndarray:
        return attitude.radar_to_body(*self.mount_ypr_deg)

    def active_at(self, t: float) -> bool:
        if not self.enabled or t < self.start_time_s:
            return False
        return self.stop_time_s is None or t <= self.stop_time_s

    def beam_at(self, t: float) -> "BeamState | None":
        if not self.active_at(t):
            return None
        g = int(np.floor((t - self.start_time_s) / self.dwell_time_s + 1e-9))
        scan_id, dwell_id = divmod(g, self.n_dwells)
        el_idx, az_idx = divmod(dwell_id, self.n_az)
        az = -self.az_coverage_deg / 2 + self.az_beamwidth_deg / 2 + self.az_offset_deg + az_idx * self.az_step
        el = -self.el_coverage_deg / 2 + self.el_beamwidth_deg / 2 + self.el_offset_deg + el_idx * self.el_step
        return BeamState(scan_id, dwell_id, az_idx, el_idx, az, el,
                         self.az_beamwidth_deg, self.el_beamwidth_deg)

    def in_coverage(self, rng, az_deg, el_deg):
        """Coverage gates of the existing pipeline: range, |az| <= cov/2, |el| <= cov/2."""
        return ((np.asarray(rng) <= self.r_max)
                & (np.abs(coordinates.wrap180(az_deg)) <= self.az_coverage_deg / 2.0)
                & (np.abs(np.asarray(el_deg)) <= self.el_coverage_deg / 2.0))


@dataclass(frozen=True, slots=True)
class BeamState:
    scan_id: int
    dwell_id: int
    az_idx: int
    el_idx: int
    az_deg: float            # beam centre, radar frame
    el_deg: float
    az_width_deg: float
    el_width_deg: float

    @property
    def left(self) -> float:
        return self.az_deg - self.az_width_deg / 2

    @property
    def right(self) -> float:
        return self.az_deg + self.az_width_deg / 2

    @property
    def bottom(self) -> float:
        return self.el_deg - self.el_width_deg / 2

    @property
    def top(self) -> float:
        return self.el_deg + self.el_width_deg / 2


@dataclass(frozen=True, slots=True)
class SensorState:
    """One sensor at one instant: where it is, where it points, where its beam is."""

    config: SensorConfig
    P_WR: np.ndarray         # radar position, world ENU
    R_WR: np.ndarray         # radar FRD -> world ENU
    beam: BeamState | None
    yaw_deg: float
    pitch_deg: float
    roll_deg: float

    def to_radar(self, points_world) -> np.ndarray:
        return coordinates.world_to_frame(self.P_WR, self.R_WR, points_world)

    def to_world(self, points_radar) -> np.ndarray:
        return coordinates.frame_to_world(self.P_WR, self.R_WR, points_radar)

    @property
    def transform(self) -> np.ndarray:
        return attitude.homogeneous(self.R_WR, self.P_WR)


def sensor_state(config: SensorConfig, P_WA, R_WB, t: float) -> SensorState:
    """Radar pose from the aircraft pose: P_WR = P_WA + R_WB r_BR, R_WR = R_WB R_BR."""
    P_WR = np.asarray(P_WA, float) + R_WB @ np.asarray(config.mount_xyz_frd, float)
    R_WR = R_WB @ config.R_BR
    yaw, pitch, roll = attitude.frd_to_ypr(R_WR)
    return SensorState(config, P_WR, R_WR, config.beam_at(t), yaw, pitch, roll)
