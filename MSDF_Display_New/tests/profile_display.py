"""Where the frame time actually goes. Evidence before optimisation.

    "D:\\Moving Radar\\.venv\\Scripts\\python.exe" tests/profile_display.py [real.pkts]

Opens a window, loads the real Scenario Export, feeds real MultiTrack packets, then runs a
fixed number of frames and reports:

  * wall time per frame, as a distribution - the tail is what the operator feels as a freeze
  * a cProfile of the render path, cumulative, so the actual hot spot is named
  * how often the terrain is rebuilt, and what each rebuild costs
  * whether any file is opened while frames are running (a per-frame read is a bug)

It changes nothing. It only measures.
"""

from __future__ import annotations

import builtins
import cProfile
import io
import json
import os
import pstats
import sys
import time
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

import paths  # noqa: E402
from PySide6 import QtCore, QtGui, QtWidgets  # noqa: E402

import verify_multitrack_interface as vmi  # noqa: E402

FRAMES = 240


class FileWatch:
    """Counts files opened between enter and exit, by path.

    Anything opened while frames are running is doing I/O in the render path, which is the
    difference between a display that streams and one that stutters.
    """

    def __init__(self):
        self.opened: dict[str, int] = {}
        self._real = builtins.open

    def __enter__(self):
        def watched(file, *a, **kw):
            name = str(file)
            if "__pycache__" not in name:
                self.opened[name] = self.opened.get(name, 0) + 1
            return self._real(file, *a, **kw)

        builtins.open = watched
        return self

    def __exit__(self, *_exc):
        builtins.open = self._real
        return False


def main() -> int:
    pkts_path = Path(sys.argv[1]) if len(sys.argv) > 1 else (APP / "real.pkts")
    packets = vmi.load_packets(pkts_path) if pkts_path.is_file() else []
    print(f"packets available: {len(packets)}")

    from visualization.view_3d_quick import surface_format

    QtGui.QSurfaceFormat.setDefaultFormat(surface_format())
    app = QtWidgets.QApplication([])
    from visualization.view_controller import ViewController
    from widgets.main_window import MainWindow
    from widgets.theme import QSS

    app.setStyleSheet(QSS)
    cfg = json.loads((APP / "config" / "display_config.json").read_text(encoding="utf-8"))
    export, _tried = paths.find_export(cfg.get("scenario_export", {}).get("root"))
    if export is not None:
        cfg["scenario_export"]["root"] = str(export)
    import socket

    free = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    free.bind(("127.0.0.1", 0))
    cfg["rdp"]["port"] = free.getsockname()[1]
    free.close()

    t0 = time.perf_counter()
    ctrl = ViewController(cfg, APP)
    win = MainWindow(ctrl)
    win.show()
    ctrl.start()
    print(f"window up in {time.perf_counter() - t0:.1f}s; loading export ...", flush=True)

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.processEvents(QtCore.QEventLoop.AllEvents, 20)
            time.sleep(0.002)

    t0 = time.perf_counter()
    while not ctrl.is_ready and time.perf_counter() - t0 < 300:
        pump(0.05)
    print(f"export ready in {time.perf_counter() - t0:.1f}s  (ready={ctrl.is_ready})", flush=True)

    if packets:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for raw in packets[-80:]:
            tx.sendto(raw, ("127.0.0.1", cfg["rdp"]["port"]))
        pump(1.5)
    print(f"tracks live: {len(ctrl.track_views)}", flush=True)

    # ---- frame timing, with file access watched --------------------------------------
    ctrl.play()
    times: list[float] = []
    with FileWatch() as watch:
        for _ in range(FRAMES):
            t = time.perf_counter()
            ctrl._tick()                      # noqa: SLF001 - one frame, exactly as the timer does
            app.processEvents(QtCore.QEventLoop.AllEvents, 5)
            times.append((time.perf_counter() - t) * 1000.0)

    times.sort()
    n = len(times)
    print()
    print(f"FRAME TIME over {n} frames (ms)")
    print(f"  median {times[n // 2]:7.1f}   mean {sum(times) / n:7.1f}")
    print(f"  p90    {times[int(n * 0.90)]:7.1f}   p99  {times[int(n * 0.99)]:7.1f}")
    print(f"  worst  {times[-1]:7.1f}   <- what the operator feels")
    budget = 1000.0 / 30.0
    print(f"  frames over the 30 Hz budget ({budget:.0f} ms): "
          f"{sum(1 for t in times if t > budget)} of {n}")

    if watch.opened:
        print("\nFILES OPENED DURING FRAMES  (any entry here is I/O in the render path)")
        for name, count in sorted(watch.opened.items(), key=lambda kv: -kv[1])[:10]:
            print(f"  {count:5} x  {name}")
    else:
        print("\nFILES OPENED DURING FRAMES: none")

    # ---- where the time goes ---------------------------------------------------------
    prof = cProfile.Profile()
    prof.enable()
    for _ in range(60):
        ctrl._tick()                          # noqa: SLF001
        app.processEvents(QtCore.QEventLoop.AllEvents, 5)
    prof.disable()
    buf = io.StringIO()
    pstats.Stats(prof, stream=buf).sort_stats("cumulative").print_stats(22)
    print("\nPROFILE (cumulative, 60 frames)")
    for line in buf.getvalue().splitlines():
        if "MSDF_Display_New" in line or "cumtime" in line:
            print("  " + line.strip())

    win.close()
    ctrl.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
