"""Check that this machine can actually run the display, and say what is wrong if it cannot.

    python verify_installation.py            full check (opens no window)
    python verify_installation.py --quick    skip the rendering checks

Checked: Python version, required packages, the project layout, the configuration, the assets,
the Scenario Export, Qt, pyqtgraph, PyVista/VTK rendering (off screen) and the UDP port. Each
line is PASS, WARN or FAIL; the exit code is non-zero only when something would stop the
application from starting.
"""

from __future__ import annotations

import os
import socket
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
os.environ.setdefault("QT_API", "pyside6")
for stream in (sys.stdout, sys.stderr):                 # paths may hold non-ASCII characters
    try:
        stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

PASS, WARN, FAIL = "PASS", "WARN", "FAIL"
results: list[tuple[str, str, str]] = []


def record(state: str, what: str, detail: str = "") -> None:
    results.append((state, what, detail))
    print(f"{state}  {what}" + (f"  -  {detail}" if detail else ""), flush=True)


def check_python() -> None:
    import environment
    problem = environment.python_problem()
    if problem:
        record(FAIL, "Python version", problem.why)
    else:
        record(PASS, "Python version", f"{sys.version.split()[0]} "
                                       f"(needs {environment.MIN_PYTHON[0]}.{environment.MIN_PYTHON[1]}+)")


def check_packages() -> bool:
    import environment
    missing = environment.missing_packages()
    for name, (dist, why) in environment.REQUIRED.items():
        if name in missing:
            record(FAIL, f"package {dist}", f"not installed - needed for {why}")
        else:
            import importlib.metadata as md
            try:
                version = md.version(dist)
            except Exception:                               # noqa: BLE001
                version = "installed"
            record(PASS, f"package {dist}", version)
    return not missing


def check_layout() -> None:
    import paths
    for name, path in paths.describe().items():
        if name in ("log_dir", "cache_dir", "data_dir"):
            continue
        record(PASS if path.is_dir() else FAIL, f"folder {name}", str(path))
    made = paths.ensure_writable_dirs()
    record(PASS, "writable folders", f"logs and cache ready"
           + (f" (created {', '.join(p.name for p in made)})" if made else ""))
    model = paths.ASSETS_DIR / "aircraft_background.wav"
    record(PASS if model.is_file() else WARN, "asset aircraft_background.wav",
           str(model) if model.is_file() else "missing - the 3D view runs without ambience")


def check_config() -> dict:
    import json
    import paths
    path = paths.config_file()
    if not path.is_file():
        record(FAIL, "configuration", f"{path} not found")
        return {}
    try:
        cfg = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:                                # noqa: BLE001
        record(FAIL, "configuration", f"{path}: {exc}")
        return {}
    record(PASS, "configuration", str(path))
    root = str(cfg.get("scenario_export", {}).get("root", ""))
    absolute = Path(root).is_absolute()
    record(WARN if absolute else PASS, "configured export path",
           f"{root!r}" + (" - absolute, so this config is machine-specific" if absolute
                          else " (relative to the project: portable)"))
    return cfg


def check_export(cfg: dict) -> None:
    import paths
    found, tried = paths.find_export(cfg.get("scenario_export", {}).get("root"))
    if found is None:
        record(WARN, "Scenario Export", "not found; looked in: "
               + ", ".join(str(t) for t in tried)
               + "  (pass --export PATH or set MSDF_SCENARIO_EXPORT)")
        return
    record(PASS, "Scenario Export", str(found))
    names = cfg.get("scenario_export", {})
    for key in ("ownship_file", "sensor_properties_file"):
        rel = names.get(key)
        if rel and not (found / rel).is_file():
            record(WARN, f"export file {key}", f"missing: {found / rel}")


def check_qt() -> bool:
    import environment
    problem = environment.qt_problem()
    if problem:
        record(FAIL, "Qt display", f"{problem.why} - {problem.fix}")
        return False
    try:
        from PySide6 import QtWidgets
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        w = QtWidgets.QWidget()
        w.resize(80, 40)
        record(PASS, "Qt (PySide6)", f"{QtWidgets.QApplication.applicationName() or 'ready'}")
        w.deleteLater()
        return True
    except Exception as exc:                                # noqa: BLE001
        record(FAIL, "Qt (PySide6)", f"{type(exc).__name__}: {exc}")
        return False


def check_pyqtgraph() -> None:
    try:
        import numpy as np
        import pyqtgraph as pg
        plot = pg.PlotWidget()
        plot.plot(np.arange(5), np.arange(5))
        record(PASS, "pyqtgraph (2D view, PPI)", pg.__version__)
    except Exception as exc:                                # noqa: BLE001
        record(FAIL, "pyqtgraph (2D view, PPI)", f"{type(exc).__name__}: {exc}")


def check_vtk() -> None:
    """Render one frame off screen: this is what fails on machines without usable OpenGL."""
    try:
        import pyvista as pv
        plotter = pv.Plotter(off_screen=True, window_size=(160, 120))
        plotter.add_mesh(pv.Sphere())
        image = plotter.screenshot(return_img=True)
        plotter.close()
        ok = image is not None and getattr(image, "size", 0) > 0
        record(PASS if ok else FAIL, "PyVista / VTK rendering",
               f"pyvista {pv.__version__}, rendered {image.shape[1]}x{image.shape[0]}" if ok
               else "the renderer produced no image")
    except Exception as exc:                                # noqa: BLE001
        record(FAIL, "PyVista / VTK rendering",
               f"{type(exc).__name__}: {exc} - the 3D view needs working OpenGL. On a server "
               "install mesa (libgl1, libglx-mesa0) or use the 2D view and the PPI.")


def check_rdp_port(cfg: dict) -> None:
    rdp = cfg.get("rdp", {})
    host, port = rdp.get("bind_host", "127.0.0.1"), int(rdp.get("port", 9000))
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind((host, port))
        record(PASS, "RDP UDP port", f"{host}:{port} is free")
    except OSError as exc:
        record(WARN, "RDP UDP port", f"{host}:{port} is in use ({exc.strerror or exc}); another "
                                     "display or the Global Display may be listening - use "
                                     "--rdp-port N")
    finally:
        s.close()


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    quick = "--quick" in argv
    print("MSDF Display - installation check\n" + "-" * 60)
    check_python()
    have_packages = check_packages()
    check_layout()
    cfg = check_config()
    check_export(cfg)
    if have_packages and not quick:
        if check_qt():
            check_pyqtgraph()
            check_vtk()
    elif not have_packages:
        record(WARN, "graphics checks", "skipped: packages are missing")
    if cfg:
        check_rdp_port(cfg)

    fails = [r for r in results if r[0] == FAIL]
    warns = [r for r in results if r[0] == WARN]
    print("-" * 60)
    print(f"{len(results) - len(fails) - len(warns)} passed, {len(warns)} warning(s), "
          f"{len(fails)} failure(s)")
    if fails:
        print("\nThe application will not start until these are fixed:")
        for _state, what, detail in fails:
            print(f"  - {what}: {detail}")
    elif warns:
        print("\nReady to start. Warnings are things to be aware of, not blockers.")
    else:
        print("\nEverything checks out.")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
