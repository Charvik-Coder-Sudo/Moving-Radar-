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
from PySide6 import QtCore, QtGui, QtWidgets  # noqa: E402

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

# the 3D surface format must be chosen before the application exists
from visualization.view_3d_quick import surface_format  # noqa: E402
QtGui.QSurfaceFormat.setDefaultFormat(surface_format())
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

# ---- camera: every preset reachable, and the operator able to move within it -------------
import numpy as np  # noqa: E402

from visualization import camera as cam  # noqa: E402

v3 = win.view3d


def framed_gap():
    """How far the ownship is from the middle of the picture, in half-screens.

    The measure that matters for "is the aircraft still held": 0 is dead centre, 1 is the
    edge of the view. Taken from the pose actually rendered.
    """
    own = ctrl.current_frame.ownship
    if own is None:
        return float("nan")
    view = np.asarray(v3.camera_focal, float) - np.asarray(v3.camera_eye, float)
    to_own = np.asarray(own.position, float) - np.asarray(v3.camera_eye, float)
    cos = float(np.dot(view, to_own) / (np.linalg.norm(view) * np.linalg.norm(to_own) + 1e-9))
    off = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))
    return off / (0.5 * cam.VIEW_ANGLE_DEG)


poses = {}
for _label, key, _sc in cam.CAMERA_PRESETS:
    v3.apply_camera(key)
    pump(0.25)
    poses[key] = np.asarray(v3.camera_eye, float).copy()
check("every camera preset gives a usable pose",
      all(np.all(np.isfinite(p)) for p in poses.values()))
check("the presets are actually different viewpoints", len({tuple(np.round(p, 1))
                                                            for p in poses.values()}) >= 8)

v3.apply_camera("chase")
pump(0.4)
behind = np.asarray(v3.camera_eye, float).copy()
focal_at_chase = np.asarray(v3.camera_focal, float).copy()
held_before = framed_gap()
# 180 deg of orbit at ORBIT_DEG_PER_PX per pixel: drag from behind the aircraft to in front
v3._on_orbit(-180.0 / cam.ORBIT_DEG_PER_PX, 0.0)
pump(0.4)
infront = np.asarray(v3.camera_eye, float).copy()
own_xy = np.asarray(ctrl.current_frame.ownship.position, float)
check("orbit moves the camera to the far side of the aircraft",
      float(np.linalg.norm(infront - behind)) > 0.8 * float(np.linalg.norm(behind - own_xy)))
check("orbit keeps the aircraft framed (follow is not dropped)",
      v3.follow_btn.isChecked() and framed_gap() < 1.0, )
# Chase looks at a point AHEAD of the aircraft, so orbiting pivots there rather than on the
# jet; the distance that must hold is the one to what the camera is actually looking at.
orbit_radius = np.linalg.norm(infront - np.asarray(v3.camera_focal, float))
check("orbit holds the viewing distance",
      abs(orbit_radius - np.linalg.norm(behind - focal_at_chase)) < 0.02 * orbit_radius)

# elevation: look at it from below, then from above, without the picture rolling over
v3._on_orbit(0.0, -90.0 / cam.ORBIT_DEG_PER_PX)
pump(0.3)
check("orbit can get under the aircraft", v3.camera_eye[2] < own_xy[2])
v3._on_orbit(0.0, 200.0 / cam.ORBIT_DEG_PER_PX)
pump(0.3)
_az, el, _d = cam.spherical(np.asarray(v3.camera_eye, float) - np.asarray(v3.camera_focal, float))
check("elevation stops short of the pole", el <= cam.ORBIT_ELEVATION_LIMITS[1] + 1e-6)

far = np.linalg.norm(np.asarray(v3.camera_eye, float) - np.asarray(v3.camera_focal, float))
v3._on_zoom(3)
pump(0.3)
near = np.linalg.norm(np.asarray(v3.camera_eye, float) - np.asarray(v3.camera_focal, float))
check("wheel zooms in", near < far * 0.95)

focal_before = np.asarray(v3.camera_focal, float).copy()
v3._on_pan(60.0, 40.0)
pump(0.3)
check("middle drag pans what the camera looks at",
      float(np.linalg.norm(np.asarray(v3.camera_focal, float) - focal_before)) > 1.0)

clock_before, selected_before = ctrl.current_frame.t_ms, ctrl.selected_key
v3.reset_view()
pump(0.5)
check("reset view returns to the preset framing",
      not v3.director.adjusted and abs(framed_gap() - held_before) < 0.25)
check("reset view leaves the simulation alone",
      ctrl.is_ready and ctrl.selected_key == selected_before
      and ctrl.current_frame.t_ms == clock_before)

# Follow ownship holds the aircraft itself, so orbiting there inspects the jet from any angle
v3.apply_camera("follow_ownship")
pump(0.4)
own_now = np.asarray(ctrl.current_frame.ownship.position, float)
check("follow ownship pivots on the aircraft",
      float(np.linalg.norm(np.asarray(v3.camera_focal, float) - own_now)) < 1.0)
ring = []
for _ in range(4):
    v3._on_orbit(-90.0 / cam.ORBIT_DEG_PER_PX, 0.0)
    pump(0.25)
    ring.append(framed_gap())
check("the aircraft stays framed from every bearing", max(ring) < 0.5)

v3.apply_camera("orbit")
pump(0.4)
check("free orbit releases the aircraft but still answers the mouse",
      not v3.follow_btn.isChecked())
free_before = np.asarray(v3.camera_eye, float).copy()
v3._on_orbit(120.0, 25.0)
pump(0.3)
check("free orbit moves the camera",
      float(np.linalg.norm(np.asarray(v3.camera_eye, float) - free_before)) > 1.0)
shot("2b_camera")

v3.apply_camera("chase")
pump(0.3)
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
