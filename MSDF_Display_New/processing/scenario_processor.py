"""World frame from the Scenario Export at a given export timestamp (ms).

Everything is taken from recorded rows (sample-and-hold of the latest row at or
before t); nothing is integrated, interpolated or generated:

    ownship   X/Y/Z, Vx/Vy/Vz, Yaw/Pitch/Roll        3D Trajectory/Ownship_Primary_Radar.csv
    radar     Radar_X/Y/Z, Radar_Yaw/Pitch/Roll       3D Trajectory/Ownship_<sensor>.csv
    beam      Scan_ID, Dwell_ID, BeamAngle/Elevation  Scan_Scheduler/Scan_Scheduler_<sensor>.csv
    targets   X/Y/Z, Vx/Vy/Vz, TgtId, Auth, IFF          3D Trajectory/target_<TgtId>.csv

Radar orientation R_WR is built from the exported Radar_Yaw/Pitch/Roll with the
project's own body-to-world convention (msdf_math.attitude, copied from
Coordinate_Geometry.Aircraft_Body_to_World).
"""

from __future__ import annotations

import numpy as np

from data_loader.scenario_export import ScenarioData, SensorTimeline, TargetTimeline
from models.ownship_state import OwnshipState
from models.sensor_state import BeamState, SensorState
from models.track_state import KIND_TARGET, TrackState
from msdf_math import attitude, coordinates, kinematics, motion
from processing.frame import FrameState, RadarView, TrackView

TRAIL_SAMPLE_MS = 1000.0     # target history is drawn from recorded rows ~1 s apart (+ the current row)
COURSE_WINDOW_S = 0.5        # half-window of recorded rows used for a course over ground (+-0.5 s)


class ScenarioProcessor:
    def __init__(self, data: ScenarioData):
        self.data = data
        self.sensors = [s.config for s in data.sensors]
        self.by_number = {s.sensor_number: s for s in data.sensors}
        # recorded rows ~1 s apart, computed once: the target history polyline is taken from these
        self._trail_idx = {tg.target_id: self._sample_rows(tg.t_ms) for tg in data.targets}

    @staticmethod
    def _sample_rows(t_ms: np.ndarray) -> np.ndarray:
        if not len(t_ms):
            return np.empty(0, int)
        grid = np.arange(t_ms[0], t_ms[-1], TRAIL_SAMPLE_MS)
        return np.unique(np.searchsorted(t_ms, grid, side="left").clip(0, len(t_ms) - 1))

    def radar_pose(self, sensor_number: int, t_ms: float):
        """(P_WR, R_WR) of one radar from its exported pose row at or before t, or None."""
        tl = self.by_number.get(sensor_number)
        if tl is None or not len(tl.pose_t_ms) or t_ms < tl.pose_t_ms[0] or t_ms > tl.pose_t_ms[-1] + 1000.0:
            return None
        i = int(np.searchsorted(tl.pose_t_ms, t_ms, side="right")) - 1
        x, y, z, yaw, pitch, roll = (float(v) for v in tl.pose[i])
        return np.array([x, y, z]), attitude.body_to_world(yaw, pitch, roll)

    def ownship_at(self, t_ms: float):
        own = self.data.ownship
        i = own.index_at(t_ms)
        if i < 0:
            return None, -1
        yaw, pitch, roll = (float(v) for v in own.ypr[i])
        # attitude the recorded path implies; the exported Yaw is kept as recorded beside it
        pa = motion.path_attitude(own.t_ms, own.pos, i, window_s=COURSE_WINDOW_S)
        R_disp, source, delta = None, "Scenario Export (recorded)", float("nan")
        if pa.valid:
            # pitch keeps the exported nose-up offset (angle of attack); only the heading and
            # the bank come from the path, because the exported Yaw contradicts the positions
            aoa = pitch - kinematics.pitch_from_velocity_deg(*own.vel[i])
            R_disp = attitude.body_to_world(pa.course_deg, pa.climb_deg + aoa,
                                            pa.bank_deg if np.isfinite(pa.bank_deg) else roll)
            source = f"course over ground ({COURSE_WINDOW_S * 2:.0f} s of recorded positions)"
            delta = float(motion.angle_difference(yaw, pa.course_deg))
        return OwnshipState(
            timestamp=float(own.t_ms[i]) / 1000.0, track_id="OWNSHIP",
            x=float(own.pos[i, 0]), y=float(own.pos[i, 1]), z=float(own.pos[i, 2]),
            yaw_deg=yaw, pitch_deg=pitch, roll_deg=roll,
            vx=float(own.vel[i, 0]), vy=float(own.vel[i, 1]), vz=float(own.vel[i, 2]),
            ax=np.nan, ay=np.nan, az=np.nan, omega=np.nan,
            heading_valid=True, roll_valid=True,
            R_WB=attitude.body_to_world(yaw, pitch, roll), row=i,
            timestamp_ms=float(own.t_ms[i]), attitude_source="Scenario Export",
            course_deg=pa.course_deg, climb_deg=pa.climb_deg, bank_deg=pa.bank_deg,
            path_speed_mps=pa.speed_mps, R_WB_display=R_disp,
            attitude_display_source=source, attitude_delta_deg=delta,
        ), i

    @staticmethod
    def sensor_at(tl: SensorTimeline, t_ms: float) -> SensorState | None:
        i = int(np.searchsorted(tl.pose_t_ms, t_ms, side="right")) - 1
        if i < 0:
            return None
        x, y, z, yaw, pitch, roll = (float(v) for v in tl.pose[i])
        beam = None
        k = int(np.searchsorted(tl.sched_t_ms, t_ms, side="right")) - 1
        cfg = tl.config
        # a dwell lasts one dwell time; outside the schedule there is no beam
        if k >= 0 and t_ms - float(tl.sched_t_ms[k]) <= cfg.dwell_time_s * 1000.0 + 1e-6:
            dwell = int(tl.dwell_id[k])
            el_idx, az_idx = divmod(dwell, cfg.n_az)        # scheduler: azimuth steps fastest
            beam = BeamState(int(tl.scan_id[k]), dwell, az_idx, el_idx, float(tl.beam_az[k]),
                             float(tl.beam_el[k]), cfg.az_beamwidth_deg, cfg.el_beamwidth_deg)
        return SensorState(cfg, np.array([x, y, z]), attitude.body_to_world(yaw, pitch, roll), beam,
                           yaw % 360.0, pitch, roll)

    def frame(self, t_ms: float, trail_s: float) -> FrameState:
        own, oi = self.ownship_at(t_ms)
        trail = np.empty((0, 3))
        if own is not None:
            o = self.data.ownship
            i0 = int(np.searchsorted(o.t_ms, t_ms - trail_s * 1000.0, side="left"))
            trail = o.pos[i0:oi + 1]
        sensors = [s for s in (self.sensor_at(tl, t_ms) for tl in self.data.sensors) if s is not None]
        targets = [tv for tv in (self.target_at(tg, t_ms, own) for tg in self.data.targets) if tv is not None]
        return FrameState(t=t_ms / 1000.0, t_ms=t_ms, ownship=own, ownship_trail=trail,
                          ownship_prediction=None, sensors=sensors, tracks=[], targets=targets,
                          attitude=self.attitude_check(own, oi))

    def attitude_check(self, own, row: int) -> dict:
        """Recorded attitude vs the attitude its own velocity implies (validation, never a fix).

        The yaw rate comes from the exported yaw of the neighbouring rows, so the estimated
        (coordinated-turn) bank angle is derived from the export alone."""
        if own is None:
            return {}
        o = self.data.ownship
        rate = None
        if 0 < row < len(o.t_ms) - 1:
            dt = (o.t_ms[row + 1] - o.t_ms[row - 1]) / 1000.0
            if dt > 0:
                d_yaw = (o.ypr[row + 1, 0] - o.ypr[row - 1, 0] + 180.0) % 360.0 - 180.0
                rate = float(d_yaw / dt)
        chk = kinematics.attitude_check(own.yaw_deg, own.pitch_deg, own.roll_deg,
                                        own.vx, own.vy, own.vz, rate,
                                        roll_source=f"{own.attitude_source} (recorded)")
        # the third opinion: where the recorded positions actually go
        chk.update(course_deg=own.course_deg, climb_deg=own.climb_deg,
                   path_speed_mps=own.path_speed_mps, bank_deg=own.bank_deg,
                   d_course_deg=own.attitude_delta_deg,
                   attitude_display_source=own.attitude_display_source)
        return chk

    def target_at(self, tg: TargetTimeline, t_ms: float, own: OwnshipState | None) -> TrackView | None:
        """Truth target at t (recorded row at or before t), None outside its trajectory."""
        i = tg.index_at(t_ms)
        if i < 0:
            return None
        pos, vel = tg.pos[i], tg.vel[i]
        st = TrackState(timestamp=float(tg.t_ms[i]) / 1000.0, track_id=f"T{tg.target_id}",
                        target_id=str(tg.target_id), x=float(pos[0]), y=float(pos[1]), z=float(pos[2]),
                        vx=float(vel[0]), vy=float(vel[1]), vz=float(vel[2]), omega=np.nan,
                        ax=np.nan, ay=np.nan, az=np.nan, classification=tg.auth, source="TRUTH",
                        status="scenario", row=i)
        idx = self._trail_idx[tg.target_id]
        rows = idx[:np.searchsorted(idx, i, side="right")]
        trail = np.vstack([tg.pos[rows], pos[None]])
        trail_t = np.append(tg.t_ms[rows], tg.t_ms[i]) / 1000.0
        # heading from the trajectory the target actually flies, not from the velocity columns
        # (they are swapped upstream - see msdf_math.motion); climb and speed are unaffected
        pa = motion.path_attitude(tg.t_ms, tg.pos, i, window_s=COURSE_WINDOW_S)
        vel_heading = float(np.degrees(np.arctan2(vel[0], vel[1])) % 360.0)
        heading = pa.course_deg if pa.valid else vel_heading
        pitch = pa.climb_deg if pa.valid else float(np.degrees(np.arctan2(vel[2], np.hypot(vel[0], vel[1]))))
        tv = TrackView(key=("TARGET", st.track_id), kind=KIND_TARGET, state=st, age_s=0.0,
                       heading_deg=heading, trail=trail, pitch_deg=pitch,
                       roll_deg=float(pa.bank_deg) if pa.valid and np.isfinite(pa.bank_deg) else 0.0,
                       origin="scenario", frame="world ENU (Scenario Export)", world=pos, world_trail=trail,
                       placement="exported truth position",
                       meta={"IFF enabled": tg.iff_enabled, "IFF key": tg.iff_key, "file": tg.source,
                             "trail_t": trail_t, "course_source": (
                                 "course over ground (recorded positions)" if pa.valid
                                 else "velocity columns (path too short for a course)"),
                             "heading_from_velocity_deg": vel_heading,
                             "course_delta_deg": float(motion.angle_difference(vel_heading, heading))
                             if pa.valid else float("nan")})
        if own is not None:
            rng, brg, el = coordinates.relative_rae_enu(own.position, pos)
            tv.range_m, tv.bearing_deg, tv.elevation_deg = float(rng[0]), float(brg[0]), float(el[0])
            body = coordinates.world_to_frame(own.position, own.R_WB, pos[None])[0]      # aircraft FRD
            tv.body_xyz = body
            b_rng, b_az, b_el = coordinates.radar_measurements(body)
            tv.body = RadarView(float(b_rng[0]), float(b_az[0]), float(b_el[0]), True)
        return tv
