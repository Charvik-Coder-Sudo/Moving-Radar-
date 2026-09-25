"""Aircraft background ambience for the 3D view (visual immersion only).

The sound represents nothing: not the radar, telemetry, detections, IFF, packet
activity, engine state or any distance / velocity. It is a fixed looping ambience.

    plays    while the 3D view is on screen - in the main window or popped out -
             and the sound is switched on (Settings, or the speaker button in the
             3D view's toolbar)
    stops    when the 3D view is not on screen (its window minimised or closed, the
             3D pane collapsed) and when the application closes

One QSoundEffect instance for the whole application (so there is never more than one
ambience playing), looping without gaps (QSoundEffect loops the PCM buffer; the asset is
built to be periodic). Loading is asynchronous in Qt, play / stop return immediately:
nothing blocks the GUI thread. It never reads the UDP stream, the track store or the
simulation clock. A missing / unreadable file only logs a warning - the 3D view works
without audio.
"""

from __future__ import annotations

import logging
from pathlib import Path

import paths
from PySide6 import QtCore

log = logging.getLogger("msdf.audio")
UNAVAILABLE = "Aircraft background sound unavailable. 3D visualization will continue without audio."


class AircraftAmbience(QtCore.QObject):
    playing_changed = QtCore.Signal(bool)
    message = QtCore.Signal(str)

    def __init__(self, view3d, cfg: dict, app_dir: Path, parent=None):
        super().__init__(parent)
        self.view = view3d
        self.enabled = bool(cfg.get("enabled", True))
        self.volume = min(max(float(cfg.get("volume", 0.25)), 0.0), 1.0)
        # relative to the project folder, or ASSETS_DIR when that has been moved (MSDF_ASSETS_DIR)
        configured = cfg.get("file", "assets/aircraft_background.wav")
        self.path = paths.resolve(configured, Path(app_dir))
        if not self.path.is_file():
            self.path = paths.ASSETS_DIR / Path(configured).name
        self.available = False
        self.playing = False
        self.effect = None
        self.error: str | None = None                        # why the sound is unavailable
        self._want_play = False
        if not self.path.is_file():
            self._fail(f"file not found: {self.path}")
        else:
            try:
                from PySide6.QtMultimedia import QSoundEffect
            except ImportError as exc:                       # Qt built without QtMultimedia
                self._fail(f"QtMultimedia not available ({exc})")
            else:
                self.effect = QSoundEffect(self)
                self.effect.statusChanged.connect(self._on_status)
                self.effect.setLoopCount(QSoundEffect.Loop.Infinite.value)
                self.effect.setVolume(self.volume)
                self.effect.setSource(QtCore.QUrl.fromLocalFile(str(self.path)))    # loads asynchronously
        self._poll = QtCore.QTimer(self)
        self._poll.setInterval(400)                          # visibility check: cheap, 2.5 Hz
        self._poll.timeout.connect(self.update)
        self._poll.start()

    # ------------------------------------------------------------------ state
    def _fail(self, why: str):
        self.available = False
        self.effect = None
        self.error = why
        log.warning("%s (%s)", UNAVAILABLE, why)
        self.message.emit(UNAVAILABLE)

    def _on_status(self):
        from PySide6.QtMultimedia import QSoundEffect
        status = self.effect.status() if self.effect is not None else None
        if status == QSoundEffect.Status.Ready:
            self.available = True
            self.update()
        elif status == QSoundEffect.Status.Error:
            self._fail(f"cannot decode {self.path.name}")

    def view_on_screen(self) -> bool:
        v = self.view
        win = v.window()
        handle = win.windowHandle()
        return (v.isVisible() and win.isVisible() and not win.isMinimized()
                and v.width() > 40 and v.height() > 40
                and (handle is None or handle.isExposed()))

    def update(self):
        """Start / stop the single ambience instance to match the 3D view's visibility."""
        want = self.enabled and self.view_on_screen()
        self._want_play = want
        if self.effect is None or not self.available:
            return
        if want and not self.playing:
            self.effect.play()
            self.playing = True
            self.playing_changed.emit(True)
        elif not want and self.playing:
            self.effect.stop()
            self.playing = False
            self.playing_changed.emit(False)

    # ------------------------------------------------------------------ controls
    def set_enabled(self, on: bool):
        self.enabled = bool(on)
        self.update()

    def set_volume(self, fraction: float):
        self.volume = min(max(float(fraction), 0.0), 1.0)
        if self.effect is not None:
            self.effect.setVolume(self.volume)

    def shutdown(self):
        self._poll.stop()
        if self.effect is not None and self.playing:
            self.effect.stop()
        self.playing = False
