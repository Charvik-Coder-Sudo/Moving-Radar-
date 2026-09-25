"""GUI smoke test: drives the real window (opens a window; not part of the unittest run).

    "D:\\Moving Radar\\.venv\\Scripts\\python.exe" tests/gui_smoke.py OUTPUT_DIR [--no-sound]

Needs the Scenario Export. Binds the RDP receiver to a free UDP port and sends a few
SystemTrack packets built with rdp.packet.encode_for_test - TEST VECTORS only, used to
exercise the display path (they are not scenario data). Checks: WAITING -> LIVE, tracks
placed in the world and aircraft frames, 3D / 2D switching, world-view maximise and
restore, PPI pop-out and restore, a single aircraft-ambience instance that follows the
3D view. Saves a screenshot per stage and exits non-zero on any failure.
"""

import json
import os
import socket
import sys
import time
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")   # the app resolves its own paths: no chdir needed

import paths  # noqa: E402
from PySide6 import QtCore, QtWidgets  # noqa: E402

from rdp.packet import encode_for_test  # noqa: E402
from visualization.view_controller import ViewController  # noqa: E402
from widgets.main_window import MainWindow  # noqa: E402
from widgets.theme import QSS  # noqa: E402

OUT = Path(sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else ".")
OUT.mkdir(parents=True, exist_ok=True)
free = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
free.bind(("127.0.0.1", 0))
PORT = free.getsockname()[1]
free.close()

app = QtWidgets.QApplication([])
app.setStyleSheet(QSS)
cfg = json.loads((APP / "config" / "display_config.json").read_text(encoding="utf-8"))
export, tried = paths.find_export(cfg.get("scenario_export", {}).get("root"))
if export is None:
    print("no Scenario Export found; looked in:")
    for candidate in tried:
        print(f"  {candidate}")
    print("Set MSDF_SCENARIO_EXPORT to run this smoke test.")
    sys.exit(2)
cfg["scenario_export"]["root"] = str(export)
cfg["rdp"]["port"] = PORT
if "--no-sound" in sys.argv:
    cfg.setdefault("audio", {})["enabled"] = False
ctrl = ViewController(cfg, APP)
win = MainWindow(ctrl)
win.show()
ctrl.start()
failures = []


def check(name, ok):
    print(("PASS  " if ok else "FAIL  ") + name, flush=True)
    if not ok:
        failures.append(name)


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents(QtCore.QEventLoop.AllEvents, 20)
        time.sleep(0.003)


def wait_for(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        pump(0.05)
        if cond():
            return True
    return False


def shot(tag):
    pump(0.4)
    win.grab().save(str(OUT / f"{tag}.png"))


check("export loaded", wait_for(lambda: ctrl.is_ready, 300))
check("waiting for data", wait_for(lambda: ctrl.live_state == "WAITING FOR DATA", 5))
shot("1_waiting")

tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t_s = (ctrl.t0_ms + 60_000.0) / 1000.0          # inside the export, so world placement is possible
state = (1000.0, 5.0, 0.0, 30000.0, -80.0, 0.0, 0.0, 2000.0, 0.0, 0.0)   # test vector: x R, y F, z U
for k in range(20):
    tx.sendto(encode_for_test(1, t_s + 0.5 * k, 1, state, [1, 2], [state, state]), ("127.0.0.1", PORT))
    pump(0.1)
check("LIVE after packets", wait_for(lambda: ctrl.live_state == "LIVE", 5))
check("world clock follows packets", wait_for(lambda: ctrl.live_following, 5))
check("tracks placed in world and aircraft frames",
      wait_for(lambda: ctrl.track_views and all(tv.world is not None and tv.body_xyz is not None
                                                 for tv in ctrl.track_views), 5))
shot("2_live_3d")
ctrl.set_view_mode("2d")
pump(1.0)
check("2D view shown", win.stack.currentWidget() is win.view2d)
shot("3_live_2d")
sound_on = win.ambience.available and win.ambience.enabled
if sound_on:
    check("ambience off in 2D", not win.ambience.playing)
ctrl.set_view_mode("3d")
pump(1.0)
if sound_on:
    check("ambience on in 3D", win.ambience.playing)
win.popouts.pop_out("world")
pump(1.0)
check("world view maximised", not win.table.isVisible() and win.view3d.isVisible())
shot("4_maximised")
win.popouts.restore("world")
pump(1.0)
check("world view restored", win.table.isVisible())
win.popouts.pop_out("ppi")
pump(1.0)
check("PPI popped out", win.popouts.is_out("ppi") and win.ppi.window() is not win)
win.popouts.out["ppi"][0].close()
pump(1.0)
check("PPI restored", win.ppi.window() is win)
from PySide6.QtMultimedia import QSoundEffect  # noqa: E402
check("single ambience instance", len(win.ambience.findChildren(QSoundEffect)) <= 1)
shot("5_final")
win.close()
print("OK" if not failures else f"{len(failures)} FAILURE(S): {failures}")
sys.exit(1 if failures else 0)
