"""Display frame types shared by the views.

FrameState   world picture at one Scenario Export timestamp: ownship, sensors and the
             scenario's truth targets
TrackView    one displayable track: an RDP fused / sensor track, or a scenario truth
             target (kind "target"), with the geometry the views / table / hover need

RDP tracks keep their state exactly as sent (sensor-relative). ``world`` / ``body``
placements are derived by processing.track_projection with the project's own
R_WR / R_BR; they are None when they cannot be derived (never guessed).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from models.ownship_state import OwnshipState
from models.sensor_state import SensorState
from models.track_state import TrackState


@dataclass
class RadarView:
    range_m: float
    az_deg: float
    el_deg: float
    in_coverage: bool


@dataclass
class TrackView:
    key: tuple[str, str]
    kind: str
    state: TrackState
    age_s: float                           # wall-clock seconds since the last update
    heading_deg: float
    trail: np.ndarray                      # (M,3) previous positions, same frame as state
    prediction: np.ndarray | None = None
    range_m: float = np.nan
    bearing_deg: float = np.nan            # azimuth in the track's frame (see ``frame``)
    elevation_deg: float = np.nan
    radar: dict[str, RadarView] = field(default_factory=dict)
    body: RadarView | None = None          # geometry used by the PPI
    fused_into: tuple = ()                 # system track ids that carry this sensor track
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    origin: str = "rdp"                    # where the record came from
    frame: str = "sensor-relative (x Right, y Forward, z Up)"
    system_track_id: int | None = None
    stale: bool = False
    latest_sensor: int | None = None       # RDP latestSensor of the packet
    world: np.ndarray | None = None        # (3,) ENU position (derived, see module note)
    world_trail: np.ndarray | None = None  # (M,3) ENU positions of earlier updates
    body_xyz: np.ndarray | None = None     # (3,) aircraft FRD position (PPI)
    body_trail: np.ndarray | None = None   # (M,3) aircraft FRD positions of earlier updates
    placement: str = ""                    # how world / body were derived (shown in hover)
    meta: dict = field(default_factory=dict)   # extra exported fields (e.g. IFF) for hover

    @property
    def in_any_coverage(self) -> bool:
        return any(rv.in_coverage for rv in self.radar.values())


@dataclass
class FrameState:
    t: float                               # seconds (display)
    t_ms: float                            # canonical Scenario Export timestamp, ms
    ownship: OwnshipState | None
    ownship_trail: np.ndarray
    ownship_prediction: np.ndarray | None
    sensors: list[SensorState]
    tracks: list[TrackView] = field(default_factory=list)
    targets: list[TrackView] = field(default_factory=list)   # scenario truth targets at t
    attitude: dict = field(default_factory=dict)             # recorded vs velocity-derived (validation)
