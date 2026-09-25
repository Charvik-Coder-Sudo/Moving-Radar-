"""MSDF Radar Display - isolated prototype.

Data sources (and nothing else):
    Scenario_Export/   ownship, radar poses, scan beams, sensor properties,
                       truth target trajectories                            -> 3D / 2D / PPI
    RDP UDP stream     SystemTrack packets (sensor + fused tracks)           -> 3D / 2D / PPI /
                                                                               Track Details
Start it, then start the simulation: it waits for packets on 127.0.0.1:9000 and follows
them automatically (LIVE). Play / seek replays the export; "Go Live" returns.

    python main.py                          export from the config, RDP on the configured port
    python main.py --export PATH            use this Scenario Export folder
    python main.py --validate-only          check / load the Scenario Export, print the report
    python main.py --rdp-port 9001          override the UDP port
    python main.py --no-rdp                 world views only
    python main.py --no-sound               no aircraft ambience in the 3D view
    python main.py --time 60 --view 2d --screenshot out.png

Nothing depends on the working directory: every path is resolved from this file through
``paths.py``, so the project runs wherever it is copied. Existing files are only read; the
RDP is only listened to. See docs/INSTALLATION.md, or run the setup script for your system.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent
if str(APP_DIR) not in sys.path:
    sys.path.insert(0, str(APP_DIR))
os.environ.setdefault("QT_API", "pyside6")
for _stream in (sys.stdout, sys.stderr):        # never fail on a non-UTF-8 console
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

import environment  # noqa: E402
import logging_setup  # noqa: E402
import paths  # noqa: E402


def load_config(path: Path) -> dict:
    """The configuration file, with a clear message when it is missing or malformed."""
    path = Path(path)
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise SystemExit(
            f"Configuration file not found:\n  {path}\n"
            f"  why:  the application needs config/display_config.json (or --config PATH).\n"
            f"  fix:  copy the one from the project, or run the setup script for your system.")
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Configuration file is not valid JSON:\n  {path}\n  {exc}")


def apply_export(config: dict, explicit=None) -> tuple[dict, object]:
    """Point the configuration at a Scenario Export that actually exists on this machine.

    Order: --export, MSDF_SCENARIO_EXPORT, the configured path, then the usual places beside
    the project. The configured value stays untouched when it already resolves.
    """
    cfg_value = config.get("scenario_export", {}).get("root")
    found, tried = paths.find_export(cfg_value, explicit)
    if found is not None:
        config.setdefault("scenario_export", {})["root"] = str(found)
    return config, (found, tried)


def validate_only(config) -> int:
    from data_loader.scenario_export import ScenarioExport, ScenarioExportError, describe_files
    exp = ScenarioExport(config, paths.PROJECT_ROOT)
    print(f"Scenario Export root: {exp.root}")
    if exp.root.is_dir():
        for rel, size, _mtime in describe_files(exp.root):
            print(f"  {rel:70s} {size:>14,}")
    for what, p in exp.required_files().items():
        print(f"{'OK     ' if p.is_file() else 'MISSING'} {what:34s} {p}")
    try:
        data = exp.load(progress=print)
    except ScenarioExportError as exc:
        print("\nSCENARIO EXPORT UNAVAILABLE:\n" + str(exc))
        return 1
    print()
    for issue in data.issues:
        print(issue)
    print(f"\nownship rows kept: {len(data.ownship.t_ms):,}   time {data.ownship.t_ms[0]:.1f} .. "
          f"{data.ownship.t_ms[-1]:.1f} ms")
    for s in data.sensors:
        print(f"sensor {s.sensor_number} {s.config.sensor_id:10s} pose rows {len(s.pose_t_ms):,}   "
              f"dwells {len(s.sched_t_ms):,}")
    return 1 if any(i.severity == "error" for i in data.issues) else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="MSDF Radar Display (isolated prototype)")
    ap.add_argument("--config", default=None,
                    help="configuration file (default: config/display_config.json, "
                         "or the MSDF_CONFIG environment variable)")
    ap.add_argument("--export", default=None,
                    help="Scenario Export folder (default: the configured one, or the "
                         "MSDF_SCENARIO_EXPORT environment variable)")
    ap.add_argument("--log-level", default="INFO",
                    choices=["DEBUG", "INFO", "WARNING", "ERROR"], help="file log level")
    ap.add_argument("--validate-only", action="store_true", help="check the Scenario Export and exit")
    ap.add_argument("--no-rdp", action="store_true", help="do not open the RDP UDP receiver")
    ap.add_argument("--rdp-port", type=int, default=None, help="UDP port of the RDP SystemTrack stream")
    ap.add_argument("--time", type=float, default=None, help="start at this export time (s)")
    ap.add_argument("--view", choices=["3d", "2d"], default="3d", help="world view shown at start")
    ap.add_argument("--no-sound", action="store_true", help="no aircraft ambience in the 3D view")
    ap.add_argument("--rate", type=float, default=None, help="playback rate")
    ap.add_argument("--play", action="store_true", help="start playing as soon as the export is loaded")
    ap.add_argument("--screenshot", default=None, help="save a PNG of the window once loaded and exit")
    ap.add_argument("--screenshot-delay", type=float, default=2.5, help="seconds to wait after loading")
    ap.add_argument("--camera", choices=["chase", "top", "side", "rear"], default="chase")
    args = ap.parse_args(argv)

    problems = environment.check()
    if problems:
        print(environment.report(problems), file=sys.stderr)
        return 2

    paths.ensure_writable_dirs()
    log = logging_setup.setup(level=args.log_level)
    config_path = paths.config_file(args.config)
    config = load_config(config_path)
    config, (export_dir, tried) = apply_export(config, args.export)
    if args.rdp_port is not None:
        config.setdefault("rdp", {})["port"] = args.rdp_port
    if args.no_sound:
        config.setdefault("audio", {})["enabled"] = False
    logging_setup.log_startup(log, config_path, export_dir,
                              {"rdp": f"{config.get('rdp', {}).get('bind_host')}:"
                                      f"{config.get('rdp', {}).get('port')}"
                                      f"{' (disabled)' if args.no_rdp else ''}"})
    if export_dir is None:
        log.error("no Scenario Export found; looked in: %s", ", ".join(str(t) for t in tried))
        print("Scenario Export not found. The display needs the simulation's exported files.\n"
              "  looked in:\n" + "".join(f"    {t}\n" for t in tried)
              + "  fix:  pass --export PATH, set MSDF_SCENARIO_EXPORT, or edit\n"
                f"        {config_path} -> scenario_export.root\n"
                "        (a folder holding 'Sensor_Properties' and '3D Trajectory')",
              file=sys.stderr)
        if args.validate_only:
            return 1

    if args.validate_only:
        return validate_only(config)

    from PySide6 import QtCore, QtWidgets

    from visualization.view_controller import ViewController
    from widgets.main_window import MainWindow
    from widgets.theme import QSS

    QtWidgets.QApplication.setHighDpiScaleFactorRoundingPolicy(
        QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName("MSDF Radar Display")
    app.setStyleSheet(QSS)

    ctrl = ViewController(config, paths.PROJECT_ROOT, rdp_enabled=not args.no_rdp)
    win = MainWindow(ctrl)
    ctrl.set_view_mode(args.view)
    win.showMaximized()
    if args.rate:
        ctrl.set_rate(args.rate)
    win.view3d.apply_camera(args.camera)

    def on_loaded(*_):
        if args.time is not None:
            ctrl.seek(args.time)
        win.view3d.apply_camera(args.camera)
        if args.play:
            ctrl.play()
        if args.screenshot:
            QtCore.QTimer.singleShot(int(args.screenshot_delay * 1000), shoot)

    def shoot():
        ctrl.refresh()
        app.processEvents()
        out = Path(args.screenshot)
        win.screen().grabWindow(win.winId()).save(str(out))
        if args.view == "3d" and ctrl.is_ready:
            win.view3d.plotter.screenshot(str(out.with_name(out.stem + "_3d.png")))
        print(f"saved {out}")
        app.quit()

    ctrl.data_loaded.connect(on_loaded)
    if args.screenshot:
        ctrl.data_error.connect(lambda *_: QtCore.QTimer.singleShot(1500, shoot))
    ctrl.data_error.connect(lambda msg: log.error("scenario export: %s", msg))
    ctrl.data_loaded.connect(lambda data: log.info(
        "scenario loaded: %s .. %s ms, %d sensors, %d target(s), %d issue(s)",
        f"{data.time_span_ms[0]:.0f}", f"{data.time_span_ms[1]:.0f}", len(data.sensors),
        len(data.targets), len(data.issues)))
    ctrl.live_changed.connect(lambda state, detail: log.info("rdp %s %s", state, detail))
    ctrl.rdp.connection_changed.connect(
        lambda state, detail: (log.error if state == "unavailable" else log.info)(
            "rdp receiver %s %s", state, detail))
    ctrl.start()
    rc = app.exec()
    ctrl.shutdown()
    log.info("---- MSDF Display closed (exit %s) ----", rc)
    return rc


if __name__ == "__main__":
    sys.exit(main())
