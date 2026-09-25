"""Place RDP tracks in world ENU (3D / 2D views) and in the aircraft frame (PPI).

What the packet carries (see rdp/packet.py): per sensor, the RDP's state built from
that radar's range / azimuth / elevation (MultiTrack imm3dc.Plot.calculateCartesian):

    x = r cos(el) sin(az)      y = r cos(el) cos(az)      z = r sin(el)

With the project's radar measurement definitions (Measurements_Geometry:
az = atan2(L_R, L_F), el = atan2(-L_D, hypot(L_F, L_R))) this is exactly

    x = L_R,   y = L_F,   z = -L_D          i.e.  L_FRD = (y, x, -z)

so the radar-frame line of sight is recovered without any new convention, and the
existing rotations place it:

    aircraft FRD   P_B = mount_xyz_frd + R_BR @ L_FRD          (no pose needed)
    world ENU      P_W = P_WR(t) + R_WR(t) @ L_FRD            (exported radar pose at the
                                                               packet time t)

Sensor tracks use their own sensor. The fused state (Xfused) is formed by the RDP from
states of different radars, so it has no single radar frame; it is placed with the
frame of the packet's latestSensor and labelled as such. Velocities are never
transformed (they stay as sent, relative to the moving radar).

Placements are computed once per packet state (the store publishes at 10 Hz) and
cached per track with a bounded history - nothing is recomputed per rendered frame.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from models.track_state import KIND_FUSED


def rdp_to_frd(x, y, z) -> np.ndarray:
    """RDP sensor-relative (x Right, y Forward, z Up) -> radar FRD line of sight."""
    return np.array([y, x, -z], dtype=float)


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
            L = rdp_to_frd(st.x, st.y, st.z)
            body = world = None
            if cfg is not None:
                body = np.asarray(cfg.mount_xyz_frd, float) + cfg.R_BR @ L
            pose = processor.radar_pose(num, st.timestamp * 1000.0) if processor is not None and num else None
            if pose is not None:
                world = pose[0] + pose[1] @ L
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
            if world is None and body is None:
                tv.placement = f"not placed: no sensor configuration / pose for {sensor}"
            elif tv.kind == KIND_FUSED:
                tv.placement = (f"fused state placed in the frame of latestSensor {sensor} "
                                "(Xfused combines radar frames)")
            else:
                tv.placement = f"{sensor} radar frame -> exported radar pose at packet time"
                if world is None:
                    tv.placement += " (packet time outside the Scenario Export: world position unknown)"
        for key in [k for k in self._hist if k not in live]:       # hidden tracks: drop their history
            del self._hist[key]
