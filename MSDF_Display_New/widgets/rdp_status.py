"""Live status line: is simulation data arriving, and what does it contain.

    Status        WAITING FOR DATA / LIVE / NO DATA / STALE / UDP UNAVAILABLE (packet timing)
    UDP           bind address of the one receiver
    Packets       valid packets (and rejected ones, with reasons)
    Rate          valid packets per second, measured over the last few seconds
    Tracks        Sensor N (per sensor) · Fused N · Total N  (RDP). A sensor track belongs to
                  one sensor (Primary radar, Secondary / IFF); a fused track is the fusion
                  engine's system-level track. They are counted separately and never merged.
    Targets       truth targets at the displayed time (Scenario Export)
    Sim time      displayed time and newest packet time
    Last packet   age of the newest packet

Every value is a count or a timing of what actually arrived; nothing is estimated.
"""

from __future__ import annotations

from PySide6 import QtCore, QtWidgets

from models.track_state import KIND_FUSED

STATE_STYLE = {"LIVE": "ok", "WAITING FOR DATA": "idle", "NO DATA / STALE": "warn",
               "UDP UNAVAILABLE": "error", "RDP OFF": "idle"}


class RdpStatusBar(QtWidgets.QFrame):
    def __init__(self, controller, parent=None):
        super().__init__(parent, objectName="RdpBar")
        self.ctrl = controller
        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(14, 3, 14, 3)
        h.setSpacing(18)
        self.fields = {}
        for key, label in (("status", "Status"), ("udp", "UDP"), ("count", "Packets"), ("rate", "Rate"),
                           ("tracks", "Tracks"), ("targets", "Targets"), ("time", "Sim time"),
                           ("last", "Last packet")):
            lab = QtWidgets.QLabel("—", objectName="RdpValue")
            cap = QtWidgets.QLabel(label, objectName="RdpCaption")
            if key == "tracks":
                tip = ("Sensor tracks: tracks reported by one sensor (Primary radar, "
                       "Secondary / IFF), from the sensorIds / sensorStates of each "
                       "SystemTrack packet.\n"
                       "Fused tracks: the fusion engine's system-level tracks, one per "
                       "systemTrackId.\n"
                       "Total: sensor + fused. Stale: not updated recently, drawn faded.\n"
                       "These are counts of tracks, never sensor ids.")
                lab.setToolTip(tip)
                cap.setToolTip(tip)
            box = QtWidgets.QHBoxLayout()
            box.setSpacing(5)
            box.addWidget(cap)
            box.addWidget(lab)
            h.addLayout(box)
            self.fields[key] = lab
        h.addStretch(1)
        rdp = controller.rdp
        self.fields["udp"].setText(f"{rdp.host}:{rdp.port}" + ("" if rdp.enabled else " (disabled)"))
        self._targets = 0
        self._frame_t = None
        self._clock = QtCore.QElapsedTimer()
        self._clock.start()
        controller.live_changed.connect(self.set_state)
        controller.rdp_updated.connect(self.on_rdp)
        controller.frame_ready.connect(self.on_frame)
        self.set_state(controller.live_state, controller.live_detail)

    def set_state(self, state, detail):
        lab = self.fields["status"]
        lab.setText(f"● {state}" + (f"  ({detail})" if detail else ""))
        lab.setProperty("state", STATE_STYLE.get(state, "idle"))
        lab.style().unpolish(lab)
        lab.style().polish(lab)

    def on_frame(self, frame):
        self._targets = len(frame.targets)
        self._frame_t = frame.t
        if self._clock.elapsed() >= 250:          # a status line: 4 Hz is plenty
            self._clock.restart()
            self._refresh_time()
            self.fields["targets"].setText(f"{self._targets}")

    def _set_tracks(self, views, stats):
        """Say which is which: sensor tracks and fused system tracks are not the same thing.

        A SystemTrack packet carries one fused state (``Xfused``, the fusion engine's own track)
        and the per-sensor states that fed it (``sensorIds`` / ``sensorStates``). The per-sensor
        states are counted here by the sensor that reported them - Primary radar, Secondary /
        IFF - which is a count of tracks, never a sensor id. Stale = not updated recently
        (drawn faded); hidden tracks are not counted at all.
        """
        by_source: dict[str, int] = {}
        fused = 0
        for tv in views or []:
            if tv.stale:
                continue
            if tv.kind == KIND_FUSED:
                fused += 1
            else:
                by_source[tv.state.source] = by_source.get(tv.state.source, 0) + 1
        sensor = sum(by_source.values())
        names = {s["id"]: s.get("name", s["id"])
                 for s in self.ctrl.config.get("scenario_export", {}).get("sensors", {}).values()}
        detail = " · ".join(f"{names.get(src, src)} {n}" for src, n in sorted(by_source.items()))
        stale = stats.get("stale_system_tracks", 0)
        self.fields["tracks"].setText(
            f"Sensor {sensor}" + (f" ({detail})" if detail else "")
            + f"  ·  Fused {fused}  ·  Total {sensor + fused}"
            + (f"  (+{stale} stale)" if stale else ""))

    def _refresh_time(self):
        st = self.ctrl.rdp.stats or {}
        pkt = st.get("newest_time")
        disp = "—" if self._frame_t is None else f"{self._frame_t:.2f} s"
        self.fields["time"].setText(disp + ("" if pkt is None else f"  (packet {pkt:.2f} s)"))

    def on_rdp(self, views, stats):
        if not stats:
            return
        age = stats.get("last_packet_age_s")
        self.fields["last"].setText("none yet" if age is None else f"{age:.1f} s ago")
        inv = stats.get("invalid", 0)
        reasons = ", ".join(f"{k} {v}" for k, v in stats.get("invalid_by_reason", {}).items())
        valid = stats.get("packets", 0) - inv
        self.fields["count"].setText(f"{valid:,}" + (f"  (+{inv:,} rejected: {reasons})" if inv else ""))
        rate = stats.get("packet_rate_hz")
        self.fields["rate"].setText("—" if rate is None else f"{rate:.1f} Hz")
        self._set_tracks(views, stats)
        self._refresh_time()
