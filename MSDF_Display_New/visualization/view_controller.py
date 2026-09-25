"""Visualization controller.

Two real data sources, no generated data:

    Scenario_Export/ (files)  ->  ScenarioExport loader (background thread)
                                  -> ScenarioProcessor -> FrameState -> 3D view, 2D view,
                                                                         ownship panel
    RDP UDP stream            ->  RdpClient (receiver thread + TrackStore)
                                  -> TrackViews + stats -> Aircraft PPI, Track Details,
                                                           RDP status bar

LIVE: while SystemTrack packets arrive, the world clock follows the packet time
(seconds, on the same timeline as the export's ``Time`` in ms): it is anchored to
the newest packet time and advances with the wall clock in between packets, never
more than ``live.max_lead_s`` ahead of the newest packet. The ownship, radar poses,
beam and truth targets are looked up at that time from the cached export, and the
RDP tracks are placed in the world / aircraft frame once per packet update
(processing.track_projection). Pressing play / seeking switches to REPLAY of the
export timeline; "Go Live" returns to following the packets.

If the export is missing or incomplete the world views show that error and nothing else.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PySide6 import QtCore

from data_loader.scenario_export import ScenarioExport, ScenarioExportError
from models.sensor_state import SensorConfig
from processing.scenario_processor import ScenarioProcessor
from processing.track_projection import TrackProjector
from rdp.receiver import RdpClient

# live-state values (header pill / status bar)
WAITING, LIVE, STALE, UNAVAILABLE, RDP_OFF = ("WAITING FOR DATA", "LIVE", "NO DATA / STALE",
                                              "UDP UNAVAILABLE", "RDP OFF")

SETTING_KEYS = ("trail_seconds", "velocity_vector_seconds", "acceleration_vector_scale_s2",
                "aircraft_scale", "target_marker_scale", "coverage_draw_range_m")


class _LoadThread(QtCore.QThread):
    progress = QtCore.Signal(str)
    done = QtCore.Signal(object)
    failed = QtCore.Signal(str)

    def __init__(self, export: ScenarioExport):
        super().__init__()
        self.export = export

    def run(self):
        try:
            self.done.emit(self.export.load(progress=self.progress.emit))
        except ScenarioExportError as exc:
            self.failed.emit(str(exc))
        except Exception as exc:                                  # noqa: BLE001
            self.failed.emit(f"ERROR   unexpected failure reading the Scenario Export: {exc!r}")


class ViewController(QtCore.QObject):
    frame_ready = QtCore.Signal(object)          # FrameState (Scenario Export)
    data_loaded = QtCore.Signal(object)          # ScenarioData
    data_error = QtCore.Signal(str)
    loading = QtCore.Signal(str)
    rdp_updated = QtCore.Signal(object, object)  # list[TrackView], stats
    playing_changed = QtCore.Signal(bool)
    layers_changed = QtCore.Signal()
    settings_changed = QtCore.Signal(str)
    selection_changed = QtCore.Signal(object)
    view_mode_changed = QtCore.Signal(str)
    status_message = QtCore.Signal(str)
    live_changed = QtCore.Signal(str, str)      # live state, detail

    def __init__(self, config: dict, app_dir: Path, rdp_enabled: bool = True):
        super().__init__()
        self.config = config
        self.app_dir = Path(app_dir)
        disp = config.get("display", {})
        self.layers: dict[str, bool] = dict(config.get("layers", {}))
        self.settings: dict = {k: disp.get(k) for k in SETTING_KEYS}
        for k, default in (("trail_seconds", 60.0), ("velocity_vector_seconds", 20.0),
                           ("acceleration_vector_scale_s2", 150.0), ("aircraft_scale", 40.0),
                           ("target_marker_scale", 1.0)):
            self.settings[k] = float(self.settings[k] or default)
        self.selected_key = None
        self.view_mode = "3d"

        # --- Scenario Export: sensor configuration now, the big files in the background
        self.export = ScenarioExport(config, self.app_dir)
        sensor_dicts, self.export_issues = self.export.sensor_dicts()
        config["sensors"] = sensor_dicts                     # runtime only; read from the export
        self.sensors = [SensorConfig.from_dict(d) for d in sensor_dicts]
        self.sensor_visible = {s.sensor_id: True for s in self.sensors}
        self.scenario = None
        self.processor: ScenarioProcessor | None = None
        self.load_error: str | None = None
        self.loading_message = ""
        self._loader: _LoadThread | None = None
        self.current_frame = None
        self.t_ms = 0.0
        self.t0_ms, self.t1_ms = 0.0, 0.0
        self.rate = float(disp.get("playback_rate", 1.0))
        self.loop = True

        self._timer = QtCore.QTimer(self)
        self._timer.setTimerType(QtCore.Qt.PreciseTimer)
        self._timer.setInterval(int(1000 / max(1, int(disp.get("frame_rate_hz", 30)))))
        self._timer.timeout.connect(self._tick)
        self._clock = QtCore.QElapsedTimer()

        # --- RDP stream ------------------------------------------------------------
        rdp_cfg = dict(config.get("rdp", {}))
        if not rdp_enabled:
            rdp_cfg["enabled"] = False
        names = {int(k): v["id"] for k, v in config.get("scenario_export", {}).get("sensors", {}).items()}
        self.rdp = RdpClient(rdp_cfg, names, parent=self)
        self.rdp.updated.connect(self._on_rdp)
        self.projector = TrackProjector(names, self.sensors, int(rdp_cfg.get("trail_points", 200)))
        self.track_views: list = []                 # RDP views with world / body placement

        # --- live clock (follows packet time) ------------------------------------
        live = config.get("live", {})
        self.follow_packets = bool(live.get("follow_packets", True))
        self.max_lead_ms = 1000.0 * float(live.get("max_lead_s", 2.0))
        self.restart_jump_ms = 1000.0 * float(live.get("restart_jump_s", 5.0))
        self.world_track_window_ms = 1000.0 * float(live.get("world_track_window_s", 10.0))
        self.replay_mode = False                    # True after the user plays / seeks the export
        self._anchor = None                         # (newest packet time ms, wall s at arrival)
        self._anchor_wall_seen = None
        self._live_wall = QtCore.QElapsedTimer()
        self._live_wall.start()
        self.live_state = RDP_OFF if not self.rdp.enabled else WAITING
        self.live_detail = ""

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        self.rdp.start()
        self.load_scenario()

    def shutdown(self):
        self._stop_timer()
        self.rdp.stop()
        if self._loader is not None:
            self._loader.wait(5000)

    def load_scenario(self):
        if self._loader is not None and self._loader.isRunning():
            return
        self._stop_timer()
        self.load_error = None
        self._loader = _LoadThread(self.export)
        self._loader.progress.connect(self._on_progress)
        self._loader.done.connect(self._on_loaded)
        self._loader.failed.connect(self._on_failed)
        self._on_progress(f"Loading Scenario Export from {self.export.root} ...")
        self._loader.start()

    def _on_progress(self, msg):
        self.loading_message = msg
        self.loading.emit(msg)
        self.status_message.emit(msg)

    def _on_loaded(self, data):
        self.scenario = data
        self.processor = ScenarioProcessor(data)
        self.t0_ms, self.t1_ms = data.time_span_ms
        self.t_ms = self.t0_ms
        if self.live_following:                     # packets were already arriving: join them
            self.t_ms = self._live_time()
            self._clock.restart()
            self._timer.start()
            self.playing_changed.emit(True)
        self.loading_message = ""
        self._emit_frame()
        self.data_loaded.emit(data)
        n_warn = sum(1 for i in data.issues if i.severity == "warning")
        self.status_message.emit(
            f"Scenario Export loaded: {self.t0_ms / 1000:.2f} .. {self.t1_ms / 1000:.2f} s "
            f"(export ms {self.t0_ms:.1f} .. {self.t1_ms:.1f}) · {len(data.sensors)} sensors · "
            f"{n_warn} warning(s)")

    def _on_failed(self, message):
        self.scenario = None
        self.processor = None
        self.current_frame = None
        self.load_error = message
        self.loading_message = ""
        self.data_error.emit(message)
        self.status_message.emit("Scenario Export unavailable - see the error banner")

    @property
    def is_ready(self) -> bool:
        return self.processor is not None

    # ------------------------------------------------------------------ clock (seconds API)
    @property
    def t(self) -> float:
        return self.t_ms / 1000.0

    @property
    def t0(self) -> float:
        return self.t0_ms / 1000.0

    @property
    def t1(self) -> float:
        return self.t1_ms / 1000.0

    def scene_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        if self.scenario is None:
            return np.array([-50000.0, -50000.0, 0.0]), np.array([50000.0, 50000.0, 10000.0])
        pos = self.scenario.ownship.pos
        return pos.min(axis=0), pos.max(axis=0)

    @property
    def is_playing(self) -> bool:
        return self._timer.isActive()

    def play(self):
        if not self.is_ready:
            return
        self.replay_mode = True
        if self.t_ms >= self.t1_ms:
            self.t_ms = self.t0_ms
        self._clock.restart()
        self._timer.start()
        self.playing_changed.emit(True)

    def pause(self):
        if self.live_following:
            self.replay_mode = True                 # pausing while live freezes the view (replay)
        if self._timer.isActive():
            self._timer.stop()
            self.playing_changed.emit(False)

    def _stop_timer(self):
        """Internal stop (reload / shutdown): not a user pause, so live following is kept."""
        if self._timer.isActive():
            self._timer.stop()
            self.playing_changed.emit(False)

    @property
    def live_following(self) -> bool:
        """World clock currently follows the packet time."""
        return self.follow_packets and not self.replay_mode and self._anchor is not None

    def go_live(self):
        """Leave replay and follow the packet time again."""
        self.replay_mode = False
        if self.is_ready and self._anchor is not None:
            self.t_ms = float(np.clip(self._anchor[0], self.t0_ms, self.t1_ms))
            self._clock.restart()
            self._timer.start()
            self.playing_changed.emit(True)
        self._update_live_state()

    def toggle(self):
        if self.is_playing:
            self.pause()
        else:
            self.play()

    def set_rate(self, rate: float):
        self.rate = float(rate)

    def seek(self, t_s: float):
        """Seek, in seconds (the export timeline is ms internally). Switches to replay."""
        if not self.is_ready:
            return
        self.replay_mode = True
        self.t_ms = float(np.clip(t_s * 1000.0, self.t0_ms, self.t1_ms))
        self._emit_frame()

    def step(self, dt_s: float):
        self.seek(self.t + dt_s)

    def _tick(self):
        dt_wall = self._clock.restart() / 1000.0
        if self.live_following:
            self.t_ms = self._live_time()
            self._emit_frame()
            return
        self.t_ms += dt_wall * self.rate * 1000.0
        if self.t_ms > self.t1_ms:
            if self.loop:
                self.t_ms = self.t0_ms
            else:
                self.t_ms = self.t1_ms
                self.pause()
        self._emit_frame()

    def _live_time(self) -> float:
        """Newest packet time + wall time since it arrived, capped at max_lead ahead; never runs
        backwards (an RDP restart is handled when the anchor is set)."""
        t_pkt, wall = self._anchor
        lead = min(max(self._live_wall.elapsed() / 1000.0 - wall, 0.0) * 1000.0, self.max_lead_ms)
        t = max(t_pkt + lead, self.t_ms)
        return float(np.clip(t, self.t0_ms, self.t1_ms))

    # ------------------------------------------------------------------ RDP
    def _on_rdp(self, views, stats):
        """One authoritative RDP update: place the tracks once, then publish to every view."""
        newest = stats.get("newest_time") if stats else None
        wall = self.rdp.store.last_packet_wall
        if newest is not None and wall is not None and wall != self._anchor_wall_seen:
            self._anchor_wall_seen = wall
            t_ms = float(newest) * 1000.0
            now_s = self._live_wall.elapsed() / 1000.0
            restarted = self._anchor is not None and self._anchor[0] - t_ms > self.restart_jump_ms
            if self._anchor is None or t_ms > self._anchor[0] or restarted:
                if restarted:
                    self.t_ms = t_ms                   # RDP restarted: the clock may jump back
                self._anchor = (t_ms, now_s)
            if self.live_following and self.is_ready and not self._timer.isActive():
                self._clock.restart()
                self._timer.start()
                self.playing_changed.emit(True)
        self.projector.project(views, self.processor)
        self.track_views = views
        self._update_live_state(stats)
        self.rdp_updated.emit(views, stats)

    def _update_live_state(self, stats=None):
        stats = stats if stats is not None else self.rdp.stats
        if not self.rdp.enabled:
            state, detail = RDP_OFF, ""
        elif self.rdp.connection == "unavailable":
            state, detail = UNAVAILABLE, self.rdp.connection_detail
        elif not stats or not stats.get("packets"):
            state, detail = WAITING, f"UDP {self.rdp.host}:{self.rdp.port}"
        elif stats.get("receiving"):
            state, detail = LIVE, ("replay view - press Go Live" if self.replay_mode else "")
        else:
            state, detail = STALE, f"last packet {stats.get('last_packet_age_s') or 0:.0f} s ago"
        if self.is_ready and self._anchor is not None and not (self.t0_ms <= self._anchor[0] <= self.t1_ms):
            detail = (detail + " · " if detail else "") + (
                f"packet time {self._anchor[0] / 1000:.1f} s is outside the Scenario Export "
                f"({self.t0_ms / 1000:.1f}-{self.t1_ms / 1000:.1f} s): export and simulation differ")
        if (state, detail) != (self.live_state, self.live_detail):
            self.live_state, self.live_detail = state, detail
            self.live_changed.emit(state, detail)

    def world_tracks(self, t_ms: float | None = None) -> list:
        """RDP tracks placed in the world whose packet time is near the displayed time."""
        t_ms = self.t_ms if t_ms is None else t_ms
        return [tv for tv in self.track_views
                if tv.world is not None and abs(tv.state.timestamp * 1000.0 - t_ms) <= self.world_track_window_ms]

    def _emit_frame(self):
        if not self.is_ready:
            return
        self.current_frame = self.processor.frame(self.t_ms, self.settings["trail_seconds"])
        self.frame_ready.emit(self.current_frame)

    def refresh(self):
        self._emit_frame()

    # ------------------------------------------------------------------ state
    def set_layer(self, name: str, on: bool):
        self.layers[name] = bool(on)
        self.layers_changed.emit()
        self.refresh()
        self.rdp_updated.emit(self.rdp.views, self.rdp.stats)

    def set_setting(self, name: str, value):
        self.settings[name] = value
        if name == "coverage_draw_range_m":
            self.config.setdefault("display", {})[name] = value     # in memory only
        self.settings_changed.emit(name)
        self.refresh()

    def set_sensor_visible(self, sensor_id: str, on: bool):
        self.sensor_visible[sensor_id] = bool(on)
        self.refresh()

    def select(self, key):
        self.selected_key = key
        self.selection_changed.emit(key)
        self.rdp_updated.emit(self.rdp.views, self.rdp.stats)

    def set_view_mode(self, mode: str):
        if mode != self.view_mode:
            self.view_mode = mode
            self.view_mode_changed.emit(mode)
