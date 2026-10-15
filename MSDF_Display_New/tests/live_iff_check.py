"""End-to-end IFF check: real MultiTrack packets into the real window.

    "D:\\Moving Radar\\.venv\\Scripts\\python.exe" tests/live_iff_check.py OUTPUT_DIR

Opens a window. Not part of the unittest run.

What makes this different from ``tests/gui_smoke.py``: that one sends packets built by
``rdp.packet.encode_for_test``, which proves the decoder round-trips but proves NOTHING about
what MultiTrack actually puts on the wire. This replays the recorded sensor track logs through
MultiTrack's own fusion code, takes the bytes the producer's own encoder produced, sends those
over a real UDP socket to a real MainWindow, and then reads the identity back out of the live
track table.

Nothing under MultiTrack is written. The log files are opened read-only and the fusion objects
are driven in-process.
"""

import faulthandler
import os
import socket
import sys
import time
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

import json  # noqa: E402

import paths  # noqa: E402
from PySide6 import QtCore, QtGui, QtWidgets  # noqa: E402

import verify_multitrack_interface as vmi  # noqa: E402

OUT = Path(sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else ".")
OUT.mkdir(parents=True, exist_ok=True)
failures = []


def check(name, ok, detail=""):
    print(("PASS  " if ok else "FAIL  ") + name + (f"   {detail}" if detail else ""), flush=True)
    if not ok:
        failures.append(name)


# ---- 1. real packets, captured from the real producer ----------------------------------
# Produced by:  python verify_multitrack_interface.py --save real.pkts
# which drives MultiTrack's OWN fusion over the recorded sensor logs and keeps the bytes the
# producer's OWN encoder emitted. Loading them here keeps this test to seconds instead of
# re-running fusion, and means it sends exactly what the live engine would send.
PKTS = Path(sys.argv[2]) if len(sys.argv) > 2 else (APP / "real.pkts")
if not PKTS.is_file():
    print(f"no captured packets at {PKTS}")
    print(f"run:  python verify_multitrack_interface.py --save {PKTS}")
    sys.exit(2)

packets = vmi.load_packets(PKTS)
from rdp.packet import decode  # noqa: E402

friends = [b for b in packets if decode(b).auth == "friend" and decode(b).icao]
check("real producer emitted IFF packets", bool(friends),
      f"{len(friends)} of {len(packets)} carry auth=friend with an ICAO")
if not friends:
    sys.exit(1)
want = decode(friends[-1])
print(f"       using ST{want.system_track_id}  auth={want.auth}  icao={want.icao}", flush=True)

# ---- 2. the real window ------------------------------------------------------------------
free = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
free.bind(("127.0.0.1", 0))
PORT = free.getsockname()[1]
free.close()

from visualization.view_3d_quick import surface_format  # noqa: E402

QtGui.QSurfaceFormat.setDefaultFormat(surface_format())
app = QtWidgets.QApplication([])
from visualization.view_controller import ViewController  # noqa: E402
from widgets.main_window import MainWindow  # noqa: E402
from widgets.theme import QSS  # noqa: E402

app.setStyleSheet(QSS)
cfg = json.loads((APP / "config" / "display_config.json").read_text(encoding="utf-8"))
export, _tried = paths.find_export(cfg.get("scenario_export", {}).get("root"))
if export is not None:
    cfg["scenario_export"]["root"] = str(export)
cfg["rdp"]["port"] = PORT
print("       building window ...", flush=True)
ctrl = ViewController(cfg, APP)
win = MainWindow(ctrl)
win.show()
ctrl.start()
print("       window up, waiting for the Scenario Export ...", flush=True)


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


# the export is only needed so the world views have an ownship to place tracks against; the
# identity checks below do not depend on it, so a slow load is waited for but not required
ready = wait_for(lambda: ctrl.is_ready, 300)
print(f"       export loaded: {ready}", flush=True)
pump(1.0)

# ---- 3. send the real bytes --------------------------------------------------------------
# if the display wedges on real traffic, that is a finding: print where, do not hang
faulthandler.dump_traceback_later(90, exit=True)
print("       sending real packets ...", flush=True)
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send = [b for b in packets if decode(b).auth == "friend"][-20:] or packets[-20:]
for n, raw in enumerate(send, 1):
    tx.sendto(raw, ("127.0.0.1", PORT))
    pump(0.05)
    if n % 5 == 0:
        print(f"       sent {n}/{len(send)}", flush=True)
pump(2.0)
print("       sent, checking ...", flush=True)

check("display accepted the real packets",
      (ctrl.rdp.stats or {}).get("packets", 0) > 0 and not (ctrl.rdp.stats or {}).get("invalid"),
      f"packets={(ctrl.rdp.stats or {}).get('packets')} invalid={(ctrl.rdp.stats or {}).get('invalid')}")

views = {tv.state.track_id: tv for tv in ctrl.track_views}
authed = [tv for tv in ctrl.track_views if tv.state.authenticated]
with_icao = [tv for tv in ctrl.track_views if tv.state.icao]
check("IFF reached the canonical track model", bool(authed) and bool(with_icao),
      f"{len(authed)} authenticated, {len(with_icao)} with an ICAO")

if authed:
    tv = authed[0]
    st = tv.state
    print(f"       {st.track_id}: class={st.classification} auth={st.auth} icao={st.icao}", flush=True)
    check("classification follows the packet's IFF result", st.classification == "FRIENDLY")
    check("ICAO is the source's own spelling",
          isinstance(st.icao, str) and st.icao.startswith("0x") and len(st.icao) == 8, st.icao)
    check("authentication reads as authenticated", "Authenticated" in st.auth_label, st.auth_label)

    # the same TrackState must be what every view consumes
    from widgets.track_table import COLUMNS

    headers = [c[0] for c in COLUMNS]
    row = {h: COLUMNS[i][1](tv) for i, h in enumerate(headers)}
    check("Track Details shows the packet ICAO", row["ICAO"] == st.icao, str(row["ICAO"]))
    check("Track Details shows the packet classification", "Friendly" in str(row["Classification"]),
          str(row["Classification"]))
    check("Track Details shows authentication", "Authenticated" in str(row["Authentication"]),
          str(row["Authentication"]))

    # truth must NOT have inherited any of it
    truth = [t for t in ctrl.track_views if t.state.source == "TRUTH"]
    check("truth targets carry no IFF", all(not t.state.icao and not t.state.auth for t in truth),
          f"{len(truth)} truth target(s)")
    from visualization import style

    check("truth is drawn neutral, not classified",
          all(style.track_color(t.kind, t.state.classification) == style.TRUTH_COLOR
              for t in truth))

faulthandler.cancel_dump_traceback_later()
pump(0.5)
win.grab().save(str(OUT / "iff_live.png"))
ctrl.set_view_mode("2d")
pump(1.0)
win.grab().save(str(OUT / "iff_live_2d.png"))
win.close()
print("OK" if not failures else f"{len(failures)} FAILURE(S): {failures}")
sys.exit(1 if failures else 0)
