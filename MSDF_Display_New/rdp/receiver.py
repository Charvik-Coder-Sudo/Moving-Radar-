"""UDP receiver for the RDP SystemTrack stream.

Threading:
    UdpReceiverWorker   runs in its own QThread: blocking socket receive with a short
                        timeout, decoding, batching. It never touches a widget.
    RdpClient           lives in the GUI thread: receives batches through queued
                        signals, updates the TrackStore, and publishes
                        ``updated(views, stats)`` on a 10 Hz timer.

The socket only receives; nothing is ever sent back to the RDP.
"""

from __future__ import annotations

import socket
import time

from PySide6 import QtCore

from rdp.packet import PacketError, decode
from rdp.track_store import TrackStore

RECV_TIMEOUT_S = 0.05
MAX_BATCH = 500
RETRY_BIND_S = 2.0


class UdpReceiverWorker(QtCore.QObject):
    batch_ready = QtCore.Signal(object)          # list of ("ok", SystemTrackState) / ("error", RejectInfo)
    state_changed = QtCore.Signal(str, str)      # "listening" | "unavailable" | "stopped", detail

    def __init__(self, host: str, port: int):
        super().__init__()
        self.host, self.port = host, int(port)
        self.bound_port: int | None = None
        self._stop = False

    @QtCore.Slot()
    def run(self):
        sock = None
        while not self._stop:
            if sock is None:
                sock = self._open()
                if sock is None:
                    self._sleep(RETRY_BIND_S)
                    continue
            batch = []
            try:
                data, addr = sock.recvfrom(65535)
                batch.append(self._decode(data, addr))
                sock.settimeout(0.0)                              # drain what is queued, no waiting
                while len(batch) < MAX_BATCH:
                    try:
                        data, addr = sock.recvfrom(65535)
                    except (BlockingIOError, socket.timeout):
                        break
                    batch.append(self._decode(data, addr))
            except socket.timeout:
                pass
            except ConnectionResetError:
                pass                                              # Windows ICMP port-unreachable noise
            except OSError as exc:
                self.state_changed.emit("unavailable", f"socket error: {exc}")
                try:
                    sock.close()
                finally:
                    sock = None
                continue
            finally:
                if sock is not None:
                    sock.settimeout(RECV_TIMEOUT_S)
            if batch:
                self.batch_ready.emit(batch)
        if sock is not None:
            sock.close()
        self.state_changed.emit("stopped", "")

    def stop(self):
        self._stop = True

    def _open(self):
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if hasattr(socket, "SIO_UDP_CONNRESET"):
                s.ioctl(socket.SIO_UDP_CONNRESET, False)
            s.bind((self.host, self.port))
            s.settimeout(RECV_TIMEOUT_S)
        except OSError as exc:
            if s is not None:
                s.close()
            self.state_changed.emit("unavailable", f"cannot bind UDP {self.host}:{self.port} - {exc}")
            return None
        self.bound_port = s.getsockname()[1]
        self.state_changed.emit("listening", f"{self.host}:{self.bound_port}")
        return s

    def _sleep(self, seconds):
        end = time.monotonic() + seconds
        while not self._stop and time.monotonic() < end:
            time.sleep(0.05)

    @staticmethod
    def _decode(data: bytes, addr):
        wall = time.monotonic()
        sender = f"{addr[0]}:{addr[1]}"
        try:
            return "ok", decode(data, wall, sender)
        except PacketError as exc:
            return "error", (exc.reason, exc.detail, wall, sender)
        except Exception as exc:                                  # noqa: BLE001 - never kill the thread
            return "error", ("decode_exception", repr(exc), wall, sender)


class RdpClient(QtCore.QObject):
    """GUI-thread side: owns the receiver thread and the TrackStore."""

    updated = QtCore.Signal(object, object)      # list[TrackView], stats dict
    connection_changed = QtCore.Signal(str, str)

    def __init__(self, cfg: dict, sensor_names: dict[int, str], parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self.enabled = bool(cfg.get("enabled", True))
        self.host = cfg.get("bind_host", "127.0.0.1")
        self.port = int(cfg.get("port", 9000))
        self.silent_timeout_s = float(cfg.get("sender_silent_timeout_s", 5.0))
        self.store = TrackStore(cfg, sensor_names)
        self.connection = "disabled" if not self.enabled else "starting"
        self.connection_detail = ""
        self.views: list = []
        self.stats: dict = self.store.stats(time.monotonic())
        self._dirty = True
        self._thread = None
        self._worker = None
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._tick)

    # lifecycle ----------------------------------------------------------------
    def start(self):
        if not self.enabled or self._thread is not None:
            self._timer.start()
            return
        self._thread = QtCore.QThread()
        self._worker = UdpReceiverWorker(self.host, self.port)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.batch_ready.connect(self._on_batch, QtCore.Qt.QueuedConnection)
        self._worker.state_changed.connect(self._on_state, QtCore.Qt.QueuedConnection)
        self._thread.start()
        self._timer.start()

    def stop(self):
        self._timer.stop()
        if self._worker is not None:
            self._worker.stop()
        if self._thread is not None:
            self._thread.quit()
            self._thread.wait(2000)
        self._thread = self._worker = None

    @property
    def bound_port(self):
        return self._worker.bound_port if self._worker is not None else None

    # slots --------------------------------------------------------------------------
    @QtCore.Slot(str, str)
    def _on_state(self, state, detail):
        self.connection, self.connection_detail = state, detail
        self.connection_changed.emit(state, detail)
        self._dirty = True

    @QtCore.Slot(object)
    def _on_batch(self, batch):
        for kind, payload in batch:
            if kind == "ok":
                self.store.ingest(payload)
            else:
                self.store.reject(*payload)
        self._dirty = True

    def _tick(self):
        now = time.monotonic()
        changed = self.store.tick(now)
        if not (self._dirty or changed):
            # ages keep growing; refresh the status line about once a second
            if int(now * 10) % 10:
                return
        self._dirty = False
        self.views = self.store.views(now)
        self.stats = self.store.stats(now)
        self.stats["connection"] = self.connection
        self.stats["connection_detail"] = self.connection_detail
        age = self.stats["last_packet_age_s"]
        self.stats["receiving"] = age is not None and age <= self.silent_timeout_s
        self.updated.emit(self.views, self.stats)
