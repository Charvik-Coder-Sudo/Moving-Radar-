"""Place RDP tracks in world ENU (3D / 2D views) and in the aircraft frame (PPI).

THE PACKET STATE IS A WORLD ENU POSITION, not a radar-relative one. That is a property
of the tracker upstream, verified in its source and against the data:

* `MultiTrack/imm3dc.py::Plot.calculateCartesian` builds the plot's Cartesian position as
  ``x = r cos(phi) sin(theta)``, ``y = r cos(phi) cos(theta)``, ``z = r sin(phi)``, with
  ``theta`` measured from North. That is x = East, y = North, z = Up - ENU.
* `Sensor/Moving_Sensor_Airborne.py::Moving_Radar.export_sensor_csv` writes ``r/theta/phi``
  as the target's spherical coordinates **about the world origin**, so the Cartesian
  position above is the target's world position. Measured on the committed scenario:
  it equals the exported truth trajectory to **0.00 m over all 65 rows**.
* `imm3dc.py` feeds that straight to the filters - ``z = plot.pos[:2]`` for the horizontal
  IMM and ``plot.pos[2]`` for the vertical one - so the track state, and the fused
  ``Xfused`` built from it, stay in the same global frame.

So the two placements are:

    world ENU      P_W = (x, y, z)                             taken as sent
    aircraft FRD   P_B = R_WB(t)^T (P_W - P_WA(t))             ownship pose at the packet
                                                               time t; this is what the
                                                               aircraft-relative PPI needs

An earlier version read the state as a radar-frame line of sight and placed it with
``P_WR + R_WR @ (y, x, -z)``. The algebra was right but the premise was not, and it put
tracks 17.7-20.5 km from the truth they were built from, with PPI bearings ~110 deg out
and the elevation sign inverted. Nothing upstream changed; only this interpretation did.

Velocities are never transformed - they stay as sent.

Placements are computed once per packet state (the store publishes at 10 Hz) and
cached per track with a bounded history - nothing is recomputed per rendered frame.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from models.track_state import KIND_FUSED
from msdf_math import coordinates


def rdp_to_world(x, y, z) -> np.ndarray:
    """RDP track state -> world ENU metres.

    Identity: the tracker already works in world ENU (see the module docstring). It is a
    named function rather than an inline tuple so the frame it asserts is searchable, and
    so there is one place to change if the upstream contract ever does.
    """
    return np.array([x, y, z], dtype=float)


class TrackProjector:
    def __init__(self, sensor_names: dict[int, str], sensors, trail_points: int = 200):
        """sensor_names  RDP sensorId -> display source id (e.g. 1 -> "PRIMARY")
        sensors       SensorConfig list (mount position / R_BR from the Scenario Export)"""
        self.number_of = {name: num for num, name in sensor_names.items()}
        self.config_of = {num: next((s for s in sensors if s.sensor_id == name), None)
                          for num, name in sensor_names.items()}
        self.trail_points = int(trail_points)
        self._hist: dict = {}          # key -> (last packet time, deque world, deque body)

    def sensor_number(self, tv) -> int | None:
        if tv.kind == KIND_FUSED:
            return tv.latest_sensor
        return self.number_of.get(tv.state.source)

    def project(self, views, processor) -> None:
        """Fill tv.world / world_trail / body_xyz / body_trail / placement in place."""
        live = set()
        for tv in views:
            live.add(tv.key)
            num = self.sensor_number(tv)
            cfg = self.config_of.get(num)
            st = tv.state
            world = rdp_to_world(st.x, st.y, st.z)      # the state is already world ENU
            body = None
            # The PPI is aircraft-relative, so it needs the ownship subtracted and the
            # remainder rotated into the body frame. R_WB_drawn is the attitude the
            # recorded path implies, which is what every other view poses the aircraft
            # with - using it here keeps the PPI and the 3D view on one attitude.
            own, _row = processor.ownship_at(st.timestamp * 1000.0) if processor is not None \
                else (None, -1)
            if own is not None:
                body = coordinates.world_to_frame(own.position, own.R_WB_drawn, world)[0]
            last_t, hw, hb = self._hist.get(tv.key, (None, None, None))
            if hw is None or (last_t is not None and st.timestamp < last_t):      # new / restarted track
                hw, hb = deque(maxlen=self.trail_points), deque(maxlen=self.trail_points)
            if last_t != st.timestamp:
                if world is not None:
                    hw.append(world)
                if body is not None:
                    hb.append(body)
            self._hist[tv.key] = (st.timestamp, hw, hb)
            tv.world, tv.body_xyz = world, body
            tv.world_trail = np.array(hw) if len(hw) else None
            tv.body_trail = np.array(hb) if len(hb) else None
            sensor = cfg.sensor_id if cfg is not None else f"sensorId {num}"
            tv.placement = ("world ENU as sent by the tracker (MultiTrack filters in global "
                            "coordinates)")
            if tv.kind != KIND_FUSED:
                tv.placement += f"; reported by {sensor}"
            if body is None:
                tv.placement += ("; no aircraft-frame position - the packet time is outside "
                                 "the Scenario Export, so the ownship pose is unknown")
        for key in [k for k in self._hist if k not in live]:       # hidden tracks: drop their history
            del self._hist[key]
