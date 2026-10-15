"""Long-run stability: does the display stay healthy, or degrade?

    "D:\\Moving Radar\\.venv\\Scripts\\python.exe" tests/soak_display.py SECONDS [real.pkts]

Opens a window, loads the real Scenario Export, streams real MultiTrack packets continuously,
and watches for the things that only show up over time:

  * simulation time running BACKWARDS during normal playback (never allowed)
  * frame time degrading as the run goes on
  * memory growing without bound
  * track count growing without bound (actors or records leaking)
  * timers or threads multiplying
  * the clock stalling

Reports PASS/FAIL per property. Changes nothing; it only measures.
"""

from __future__ import annotations

import json
import os
import socket
import sys
import time
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

import paths  # noqa: E402
from PySide6 import QtCore, QtGui, QtWidgets  # noqa: E402

import verify_multitrack_interface as vmi  # noqa: E402

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
PKTS = Path(sys.argv[2]) if len(sys.argv) > 2 else (APP / "real.pkts")
failures: list[str] = []


def check(name, ok, detail=""):
    print(("PASS  " if ok else "FAIL  ") + name + (f"   {detail}" if detail else ""), flush=True)
    if not ok:
        failures.append(name)


def rss_mb() -> float:
    """Resident set size in MB, or 0.0 when it cannot be read (the soak still runs)."""
    try:
        import psutil

        return psutil.Process(os.getpid()).memory_info().rss / (1024 * 1024)
    except Exception:
        return 0.0


def main() -> int:
    packets = vmi.load_packets(PKTS) if PKTS.is_file() else []
    print(f"soak {SECONDS:.0f}s, {len(packets)} real packet(s) available\n", flush=True)

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
    free = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    free.bind(("127.0.0.1", 0))
    port = free.getsockname()[1]
    free.close()
    cfg["rdp"]["port"] = port

    ctrl = ViewController(cfg, APP)
    win = MainWindow(ctrl)
    win.show()
    ctrl.start()

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.processEvents(QtCore.QEventLoop.AllEvents, 20)
            time.sleep(0.002)

    t0 = time.perf_counter()
    while not ctrl.is_ready and time.perf_counter() - t0 < 300:
        pump(0.05)
    check("export loads once and becomes ready", ctrl.is_ready,
          f"{time.perf_counter() - t0:.1f}s")

    timers_before = len(win.findChildren(QtCore.QTimer)) + len(ctrl.findChildren(QtCore.QTimer))
    threads_before = len(ctrl.findChildren(QtCore.QThread))
    base_rss = rss_mb()
    ctrl.play()

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frame_ms: list[float] = []
    rss: list[float] = []
    tracks: list[int] = []
    backwards: list[tuple[float, float]] = []
    loops: list[float] = []
    last_t = ctrl.t
    sent = 0
    start = time.monotonic()
    next_sample = start

    while time.monotonic() - start < SECONDS:
        if packets:                               # keep the stream flowing, as live would
            for _ in range(4):
                tx.sendto(packets[sent % len(packets)], ("127.0.0.1", port))
                sent += 1
        # the cost of ONE frame, not of the pump window: pump() sleeps by construction, so
        # timing it would measure the sleep and hide any real growth underneath it
        t = time.perf_counter()
        ctrl._tick()                              # noqa: SLF001
        frame_ms.append((time.perf_counter() - t) * 1000.0)
        pump(0.04)
        now_t = ctrl.t
        # A backwards step during NORMAL playback is the fault being hunted. Reaching the end
        # of the export and wrapping to the start is NOT that - it is the configured loop, and
        # it shows up as a single step back of almost the whole timeline.
        if now_t < last_t - 1e-6:
            wrapped = (last_t - now_t) > 0.9 * (ctrl.t1_ms - ctrl.t0_ms) / 1000.0
            if wrapped:
                loops.append(last_t)
            else:
                backwards.append((last_t, now_t))
        last_t = now_t
        if time.monotonic() >= next_sample:
            next_sample += 5.0
            rss.append(rss_mb())
            tracks.append(len(ctrl.track_views))
            print(f"  t={time.monotonic() - start:5.0f}s  sim={ctrl.t:8.2f}s  "
                  f"tracks={tracks[-1]:3}  rss={rss[-1]:7.1f} MB", flush=True)

    # ---- verdicts ---------------------------------------------------------------------
    print()
    check("simulation time never ran backwards during playback", not backwards,
          f"{len(backwards)} unexplained reversal(s)"
          + (f", worst {max(a - b for a, b in backwards):.2f}s" if backwards else "")
          + (f"; {len(loops)} intentional loop wrap(s)" if loops else ""))
    check("simulation time advanced", ctrl.t > 0.0, f"reached {ctrl.t:.1f}s")

    half = len(frame_ms) // 2
    first, second = frame_ms[:half], frame_ms[half:]
    mean_first = sum(first) / max(1, len(first))
    mean_second = sum(second) / max(1, len(second))
    check("frame time did not degrade over the run", mean_second < mean_first * 1.5 + 2.0,
          f"first half {mean_first:.1f} ms, second half {mean_second:.1f} ms")

    if rss and base_rss:
        growth = rss[-1] - rss[0]
        check("memory stayed bounded", growth < 150.0,
              f"{rss[0]:.0f} -> {rss[-1]:.0f} MB ({growth:+.0f} MB)")
    if tracks:
        # the capture holds 31 system tracks; each shows as a fused view plus one per
        # contributing sensor, so ~96 views is the data's own ceiling. What would signal a
        # leak is growth past that, or growth that never settles.
        settled = max(tracks[len(tracks) // 2:]) if len(tracks) > 3 else max(tracks)
        check("track count stayed bounded", max(tracks) <= 120 and settled <= 120,
              f"peak {max(tracks)}, settled {settled} views")

    timers_after = len(win.findChildren(QtCore.QTimer)) + len(ctrl.findChildren(QtCore.QTimer))
    threads_after = len(ctrl.findChildren(QtCore.QThread))
    check("no timers accumulated", timers_after <= timers_before,
          f"{timers_before} -> {timers_after}")
    check("no threads accumulated", threads_after <= threads_before,
          f"{threads_before} -> {threads_after}")

    stats = ctrl.rdp.stats or {}
    check("packets were accepted", stats.get("packets", 0) > 0,
          f"{stats.get('packets')} received, {stats.get('invalid', 0)} invalid")
    check("no packets were rejected as malformed", not stats.get("invalid"),
          str(stats.get("invalid_by_reason") or {}))
    if stats.get("identity_downgrades"):
        print(f"      note: {stats['identity_downgrades']} IFF downgrade(s) across "
              f"{stats.get('tracks_downgraded')} track(s) - upstream, see docs/IDENTITY.md")

    # ---- shutdown ---------------------------------------------------------------------
    try:
        win.close()
        ctrl.shutdown()
        pump(0.5)
        check("shutdown completed without error", True)
    except Exception as exc:
        check("shutdown completed without error", False, f"{type(exc).__name__}: {exc}")

    print("\nOK" if not failures else f"\n{len(failures)} FAILURE(S): {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
