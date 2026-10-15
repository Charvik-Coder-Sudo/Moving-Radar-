"""Display-side store of RDP system tracks, with stale-track handling.

Holds the latest SystemTrackState per systemTrackId plus short position
histories for trails. The RDP's own track lifecycle is not modified or
second-guessed; the display only decides what is still worth drawing:

    ACTIVE   updated within ``stale_timeout_s`` (wall clock) and not older than
             ``stream_stale_timeout_s`` in packet time than the newest packet
    STALE    otherwise, until ``hide_timeout_s`` without an update - drawn faded
    HIDDEN   no update for longer than ``hide_timeout_s`` - not drawn

A packet older (in packet time) than the ACTIVE track it updates is counted as
a stale packet and ignored. If the track is already stale / hidden, a packet with
an earlier time is taken as a restart of the RDP and replaces the track.

Associations come only from the packet: a fused (system) track and the sensor
states it carries in sensorIds / sensorStates. No other association is made.
"""

from __future__ import annotations

import logging
import math
from collections import Counter, deque
from dataclasses import dataclass, field

import numpy as np

from models.track_state import KIND_FUSED, KIND_SENSOR, TrackState
from processing.frame import RadarView, TrackView
from rdp.packet import SystemTrackState, components
from visualization import style

log = logging.getLogger(__name__)

ACTIVE, STALE, HIDDEN = "ACTIVE", "STALE", "HIDDEN"


@dataclass
class TrackRecord:
    state: SystemTrackState
    first_wall: float
    last_update_wall: float
    updates: int = 1
    status: str = ACTIVE
    fused_history: deque = field(default_factory=deque)
    sensor_history: dict = field(default_factory=dict)       # sensorId -> deque of xyz


@dataclass
class RejectedPacket:
    wall: float
    reason: str
    detail: str
    sender: str = ""


class TrackStore:
    def __init__(self, cfg: dict, sensor_names: dict[int, str]):
        self.stale_timeout_s = float(cfg.get("stale_timeout_s", 3.0))
        self.hide_timeout_s = float(cfg.get("hide_timeout_s", 10.0))
        self.stream_stale_timeout_s = float(cfg.get("stream_stale_timeout_s", 5.0))
        self.trail_points = int(cfg.get("trail_points", 200))
        # How long a track that has stopped updating is REMEMBERED after it stops being drawn.
        # Without this the record dictionary only ever grows: a hidden track keeps its trail
        # buffers for ever, and MultiTrack mints a fresh system track id whenever a report
        # fails to correlate, so over a long run the store accumulates thousands of dead
        # tracks that nothing will ever show again.
        self.forget_timeout_s = float(cfg.get("forget_timeout_s", 120.0))
        self.max_records = int(cfg.get("max_records", 4096))
        self.forgotten = 0
        self.sensor_names = dict(sensor_names)          # RDP sensorId -> display source id
        self.records: dict[int, TrackRecord] = {}
        self.packets_total = 0
        self.packets_invalid = 0
        self.invalid_by_reason: Counter = Counter()
        # How many times a system track that HAD been authenticated came back unauthenticated.
        # Not a display fault and not corrected here - the packet is shown as it arrives - but
        # it is invisible otherwise, and it is the signature of an upstream identity reset
        # (docs/IDENTITY.md, upstream note 4). Counted so the status bar can say so.
        self.identity_downgrades = 0
        self._downgraded: set[int] = set()
        self.stale_packets = 0
        self.restarts = 0
        self.unknown_sensor_ids: Counter = Counter()
        self.last_packet_wall: float | None = None
        self.last_packet_time: float | None = None
        self.newest_time: float | None = None
        self.rejected: deque = deque(maxlen=500)
        self.rate_window_s = float(cfg.get("packet_rate_window_s", 5.0))
        self._arrivals: deque = deque()                 # wall times of recent valid packets

    # ------------------------------------------------------------------ input
    def reject(self, reason: str, detail: str, wall: float, sender: str = ""):
        self.packets_total += 1
        self.packets_invalid += 1
        self.invalid_by_reason[reason] += 1
        self.last_packet_wall = wall
        self.rejected.append(RejectedPacket(wall, reason, detail, sender))

    def ingest(self, st: SystemTrackState) -> bool:
        """Apply one decoded packet. Returns False when it was ignored as stale."""
        self.packets_total += 1
        self.last_packet_wall = st.received_wall
        self._arrivals.append(st.received_wall)
        for sid in st.sensor_ids:
            if sid not in self.sensor_names:
                self.unknown_sensor_ids[sid] += 1
        rec = self.records.get(st.system_track_id)
        if rec is not None and rec.state.auth == "friend" and st.auth != "friend":
            self.identity_downgrades += 1
            if st.system_track_id not in self._downgraded:
                self._downgraded.add(st.system_track_id)
                log.warning("system track %s lost its IFF identity: auth %r -> %r, icao %r -> %r. "
                            "The packet is shown as received; see docs/IDENTITY.md.",
                            st.system_track_id, rec.state.auth, st.auth, rec.state.icao, st.icao)
        if rec is not None and st.time < rec.state.time:
            if rec.status == ACTIVE:
                self.stale_packets += 1
                self.rejected.append(RejectedPacket(st.received_wall, "stale_packet",
                                                    f"systemTrackId {st.system_track_id}: time {st.time} < "
                                                    f"last {rec.state.time}", st.sender))
                return False
            self.restarts += 1
            rec = None                                       # RDP restarted: start this track afresh
        if rec is None:
            rec = TrackRecord(st, st.received_wall, st.received_wall)
            self.records[st.system_track_id] = rec
        else:
            rec.state = st
            rec.last_update_wall = st.received_wall
            rec.updates += 1
        rec.status = ACTIVE
        self._push(rec.fused_history, _xyz(st.fused_state))
        for sid, s in zip(st.sensor_ids, st.sensor_states):
            self._push(rec.sensor_history.setdefault(sid, deque()), _xyz(s))
        self.last_packet_time = st.time
        # The HIGHEST packet time seen, not the most recent one received. MultiTrack sends one
        # packet per incoming sensor report and interleaves system tracks, so arrivals are not
        # ordered in time even when no packet is stale for its own track. Taking the last
        # arrival would make this value walk backwards, which the world clock reads as the RDP
        # having restarted - and it would then drag the displayed time back with it.
        self.newest_time = st.time if self.newest_time is None else max(self.newest_time, st.time)
        return True

    def _push(self, dq: deque, xyz):
        dq.append(xyz)
        while len(dq) > self.trail_points:
            dq.popleft()

    # ------------------------------------------------------------------ lifetime
    def tick(self, now_wall: float) -> bool:
        """Re-evaluate ACTIVE / STALE / HIDDEN. Returns True if any status changed."""
        changed = False
        for rec in self.records.values():
            age = now_wall - rec.last_update_wall
            behind = (self.newest_time - rec.state.time) if self.newest_time is not None else 0.0
            if age <= self.stale_timeout_s and behind <= self.stream_stale_timeout_s:
                status = ACTIVE
            elif age <= self.hide_timeout_s:
                status = STALE
            else:
                status = HIDDEN
            if status != rec.status:
                rec.status = status
                changed = True
        changed |= self._forget(now_wall)
        return changed

    def _forget(self, now_wall: float) -> bool:
        """Drop tracks that stopped updating long ago, so the store stays bounded.

        Only HIDDEN tracks are considered, and only once they have been silent for
        ``forget_timeout_s`` - well past the point where they are still drawn. If the same
        system track id ever reappears it is simply treated as new, which is the right reading
        after that long a silence.
        """
        dead = [tid for tid, rec in self.records.items()
                if rec.status == HIDDEN and now_wall - rec.last_update_wall > self.forget_timeout_s]
        if len(self.records) - len(dead) > self.max_records:
            # a hard ceiling as well, in case tracks arrive faster than they age out
            extra = sorted(((r.last_update_wall, t) for t, r in self.records.items()
                            if t not in set(dead)))[:len(self.records) - len(dead) - self.max_records]
            dead += [t for _wall, t in extra]
        for tid in dead:
            del self.records[tid]
        self.forgotten += len(dead)
        if dead:
            log.debug("forgot %d track(s) silent for more than %.0fs; %d remain",
                      len(dead), self.forget_timeout_s, len(self.records))
        return bool(dead)

    # ------------------------------------------------------------------ output
    def sensor_source(self, sid: int) -> str:
        return self.sensor_names.get(sid, f"SENSOR-{sid}")

    def views(self, now_wall: float) -> list[TrackView]:
        out = []
        for tid, rec in sorted(self.records.items()):
            if rec.status == HIDDEN:
                continue
            st = rec.state
            age = now_wall - rec.last_update_wall
            fused_id = f"ST{tid}"
            assoc = tuple((self.sensor_source(sid), f"{fused_id}:S{sid}") for sid in st.sensor_ids)
            out.append(_view(KIND_FUSED, "FUSED", fused_id, st, st.fused_state, rec.status, age,
                             rec.fused_history, assoc=assoc, system_track_id=tid))
            out[-1].latest_sensor = st.latest_sensor
            for sid, s in zip(st.sensor_ids, st.sensor_states):
                out.append(_view(KIND_SENSOR, self.sensor_source(sid), f"{fused_id}:S{sid}", st, s,
                                 rec.status, age, rec.sensor_history.get(sid, ()),
                                 fused_into=(fused_id,), system_track_id=tid))
        return out

    def packet_rate(self, now_wall: float) -> float | None:
        """Valid packets per second over the last ``packet_rate_window_s`` (None before any)."""
        while self._arrivals and now_wall - self._arrivals[0] > self.rate_window_s:
            self._arrivals.popleft()
        if self.last_packet_wall is None:
            return None
        return len(self._arrivals) / self.rate_window_s

    def stats(self, now_wall: float) -> dict:
        visible = [r for r in self.records.values() if r.status != HIDDEN]
        active = [r for r in visible if r.status == ACTIVE]
        return dict(
            packets=self.packets_total, invalid=self.packets_invalid,
            identity_downgrades=self.identity_downgrades,
            tracks_downgraded=len(self._downgraded),
            invalid_by_reason=dict(self.invalid_by_reason), stale_packets=self.stale_packets,
            restarts=self.restarts, unknown_sensor_ids=dict(self.unknown_sensor_ids),
            last_packet_age_s=None if self.last_packet_wall is None else now_wall - self.last_packet_wall,
            last_packet_time=self.last_packet_time,
            active_system_tracks=len(active), stale_system_tracks=len(visible) - len(active),
            active_sensor_tracks=sum(r.state.num_sensors for r in active),
            known_system_tracks=len(self.records), forgotten_tracks=self.forgotten,
            packet_rate_hz=self.packet_rate(now_wall),
            newest_time=self.newest_time,
        )


def _xyz(state) -> tuple:
    c = components(state)
    return (c["x"], c["y"], c["z"])


def _view(kind, source, track_id, st: SystemTrackState, state, status, age, history,
          assoc=(), fused_into=(), system_track_id=None) -> TrackView:
    c = components(state)
    # Identity comes off the packet and is applied to every view of this system track - the
    # fused track and each contributing sensor track alike. The IFF belongs to the TRACK, not
    # to one sensor's report of it: MultiTrack carries it through fusion onto the SystemTrack
    # (ci_ici_mulitgt.SystemTrack), so a sensor track shown under a friendly system track is
    # a view of that same friendly aircraft.
    ts = TrackState(timestamp=st.time, track_id=track_id, target_id="",
                    x=c["x"], y=c["y"], z=c["z"], vx=c["vx"], vy=c["vy"], vz=c["vz"], omega=c["w"],
                    ax=c["ax"], ay=c["ay"], az=c["az"],
                    classification=style.classification_from_auth(st.auth),
                    source=source, status=status, auth=st.auth, icao=st.icao,
                    associated_tracks=assoc)
    rng = math.sqrt(c["x"] ** 2 + c["y"] ** 2 + c["z"] ** 2)
    az = math.degrees(math.atan2(c["x"], c["y"])) % 360.0          # 0 = forward, + right
    el = math.degrees(math.atan2(c["z"], math.hypot(c["x"], c["y"])))
    rv = RadarView(rng, az, el, True)
    trail = np.array(list(history), dtype=float).reshape(-1, 3)
    return TrackView(key=(source, track_id), kind=kind, state=ts, age_s=age,
                     heading_deg=math.degrees(math.atan2(c["vx"], c["vy"])) % 360.0, trail=trail,
                     range_m=rng, bearing_deg=az, elevation_deg=el, body=rv, fused_into=fused_into,
                     origin="rdp", system_track_id=system_track_id, stale=status == STALE)
